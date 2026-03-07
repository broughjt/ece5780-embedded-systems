#include "main.h"
#include "stm32f0xx_hal.h"

void SystemClock_Config(void);
void GPIO_Init(void);
void I2C2_Init(void);
uint8_t I2C2_ReadRegister(uint8_t slave_addr, uint8_t reg_addr);

int main(void)
{
  HAL_Init();
  SystemClock_Config();

  GPIO_Init();
  I2C2_Init();

  // Give the gyro time to switch into I2C mode after PC0 goes high
  // HAL_Delay(10);

  // Read WHO_AM_I register (0x0F) from L3GD20 (slave address 0x6B).
  // Expected value is 0xD4.
  uint8_t who_am_i = I2C2_ReadRegister(0x6B, 0x0F);

  if (who_am_i == 0xD4)
  {
    // Success: light green LED (PC9)
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_SET);
  }
  else
  {
    // Failure: light red LED (PC6)
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_SET);
  }

  while (1)
  {
  }
  return -1;
}

/**
 * @brief Configure GPIO pins for I2C2 and support signals.
 *
 * PB11 — I2C2_SDA: alternate function (AF1), open-drain
 * PB13 — I2C2_SCL: alternate function (AF5), open-drain
 * PB14 — Gyro SA0 (slave-address select): output, push-pull, HIGH
 * PC0  — Gyro CS/SDO (SPI/I2C mode select): output, push-pull, HIGH
 * PC6/7/8/9 — LEDs: output, push-pull
 */
void GPIO_Init(void)
{
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  GPIO_InitTypeDef gpio = {0};

  // PB11: I2C2_SDA — alternate function, open-drain
  gpio.Pin       = GPIO_PIN_11;
  gpio.Mode      = GPIO_MODE_AF_OD;
  gpio.Pull      = GPIO_NOPULL;
  gpio.Speed     = GPIO_SPEED_FREQ_HIGH;
  gpio.Alternate = GPIO_AF1_I2C2;
  HAL_GPIO_Init(GPIOB, &gpio);

  // PB13: I2C2_SCL — alternate function, open-drain
  gpio.Pin       = GPIO_PIN_13;
  gpio.Alternate = GPIO_AF5_I2C2;
  HAL_GPIO_Init(GPIOB, &gpio);

  // PB14: slave address select pin — output, push-pull, HIGH
  gpio.Pin   = GPIO_PIN_14;
  gpio.Mode  = GPIO_MODE_OUTPUT_PP;
  gpio.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &gpio);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);

  // PC0: SPI/I2C mode select pin — output, push-pull, HIGH (selects I2C)
  gpio.Pin = GPIO_PIN_0;
  HAL_GPIO_Init(GPIOC, &gpio);
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_SET);

  // PB15: leave in input mode
  gpio.Pin  = GPIO_PIN_15;
  gpio.Mode = GPIO_MODE_INPUT;
  HAL_GPIO_Init(GPIOB, &gpio);

  // PC6 (red), PC7 (blue), PC8 (orange), PC9 (green) — LED outputs
  gpio.Pin = GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9;
  HAL_GPIO_Init(GPIOC, &gpio);
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9, GPIO_PIN_RESET);
}

/**
 * @brief Initialize I2C2 in master mode at 100 kHz (standard mode).
 *
 * TIMINGR values from Figure 5.4 for 8 MHz system clock, 100 kHz SM:
 *   PRESC=1, SCLDEL=4, SDADEL=2, SCLH=0xF, SCLL=0x13 → 0x10420F13
 */
void I2C2_Init(void)
{
  __HAL_RCC_I2C2_CLK_ENABLE();

  // Configure timing for 100 kHz standard-mode at 8 MHz
  I2C2->TIMINGR = (1U << 28) | (4U << 20) | (2U << 16) | (0xFU << 8) | 0x13U;

  // Enable the I2C peripheral (locks system-wide config registers)
  I2C2->CR1 |= I2C_CR1_PE;
}

/**
 * @brief Perform a register read from an I2C slave device.
 *
 * Executes a write (to select the register address) followed by a restart
 * read (to retrieve the value). No AUTOEND — manual START/STOP control.
 *
 * @param slave_addr  7-bit I2C slave address
 * @param reg_addr    Device register address to read
 * @return            Byte read from the device, or 0xFF on NACK error
 */
uint8_t I2C2_ReadRegister(uint8_t slave_addr, uint8_t reg_addr)
{
  // Step 1: Set up write transaction (1 byte, no AUTOEND)
  //  Clear NBYTES[23:16] and SADD[9:0], then set them.
  // 7-bit addresses occupy SADD[7:1], so shift left by 1.
  I2C2->CR2 &= ~((0xFFU << 16) | (0x3FFU << 0));
  I2C2->CR2 |= (1U << 16) | ((uint32_t)slave_addr << 1);
  I2C2->CR2 &= ~I2C_CR2_RD_WRN; // write direction
  I2C2->CR2 |= I2C_CR2_START; // generate START + address frame

  // Step 2: Wait for TXIS (address acknowledged) or NACKF (no response)
  while (!(I2C2->ISR & (I2C_ISR_TXIS | I2C_ISR_NACKF)));
  if (I2C2->ISR & I2C_ISR_NACKF)
  {
    I2C2->ICR |= I2C_ICR_NACKCF;
    I2C2->CR2 |= I2C_CR2_STOP;
    return 0xFF;
  }

  // Step 3: Transmit the register address
  I2C2->TXDR = reg_addr;

  // Step 4: Wait for transfer complete (all NBYTES sent, waiting for restart/stop)
  while (!(I2C2->ISR & I2C_ISR_TC));

  // Step 5: Restart as a read transaction
  I2C2->CR2 &= ~((0xFFU << 16) | (0x3FFU << 0));
  I2C2->CR2 |= (1U << 16) | ((uint32_t)slave_addr << 1) | I2C_CR2_RD_WRN;
  I2C2->CR2 |= I2C_CR2_START;      // restart condition

  // Step 6: Wait for RXNE (data received) or NACKF
  while (!(I2C2->ISR & (I2C_ISR_RXNE | I2C_ISR_NACKF)));
  if (I2C2->ISR & I2C_ISR_NACKF)
  {
    I2C2->ICR |= I2C_ICR_NACKCF;
    I2C2->CR2 |= I2C_CR2_STOP;
    return 0xFF;
  }

  // Step 7: Wait for transfer complete
  while (!(I2C2->ISR & I2C_ISR_TC));

  // Step 8: Read received byte from RXDR
  uint8_t data = (uint8_t)I2C2->RXDR;

  // Step 9: Release the bus
  I2C2->CR2 |= I2C_CR2_STOP;

  return data;
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void)
{
  __disable_irq();
  while (1)
  {
  }
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
}
#endif /* USE_FULL_ASSERT */
