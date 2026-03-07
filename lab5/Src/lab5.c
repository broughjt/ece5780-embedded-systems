#include "main.h"
#include "stm32f0xx_hal.h"

void SystemClock_Config(void);
void GPIO_Init(void);
void I2C2_Init(void);
uint8_t I2C2_ReadRegister(uint8_t slave_addr, uint8_t reg_addr);
void I2C2_WriteRegister(uint8_t slave_addr, uint8_t reg_addr, uint8_t data);
void I2C2_ReadRegisters(uint8_t slave_addr, uint8_t reg_addr, uint8_t *buf, uint8_t len);

#define GYRO_ADDR  0x69
#define THRESHOLD  500

int main(void)
{
  HAL_Init();
  SystemClock_Config();

  GPIO_Init();
  I2C2_Init();

  I2C2_WriteRegister(GYRO_ADDR, 0x20, 0x0B);

  int x = 0;
  int y = 0;

  while (1)
  {
    uint8_t buf[4];

    I2C2_ReadRegisters(GYRO_ADDR, 0xA8, buf, 4);

    int16_t dx = (int16_t)((buf[1] << 8) | buf[0]);
    int16_t dy = (int16_t)((buf[3] << 8) | buf[2]);

    x += dx;
    y += dy;

    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9, GPIO_PIN_RESET);

    int16_t abs_x = x < 0 ? -x : x;
    int16_t abs_y = y < 0 ? -y : y;

    if (abs_x > THRESHOLD || abs_y > THRESHOLD)
    {
      if (abs_x >= abs_y)
      {
        if (x > 0)
          HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_SET); // green: +X
        else
          HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_SET); // orange: -X
      }
      else
      {
        if (y > 0)
          HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_SET); // red: +Y
        else
          HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_SET); // blue: -Y
      }
    }

    HAL_Delay(100);
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
  HAL_GPIO_Init(GPIOB, &gpio);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);

  // PC0: SPI/I2C mode select pin — output, push-pull, HIGH (selects I2C)
  gpio.Pin = GPIO_PIN_0;
  HAL_GPIO_Init(GPIOC, &gpio);
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_SET);

  // PB15: leave in AF open-drain (connected to PB11 via jumper)
  gpio.Pin  = GPIO_PIN_15;
  gpio.Mode = GPIO_MODE_AF_OD;
  HAL_GPIO_Init(GPIOB, &gpio);

  // PC6 (red), PC7 (blue), PC8 (orange), PC9 (green) — LED outputs
  gpio.Pin  = GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9;
  gpio.Mode = GPIO_MODE_OUTPUT_PP;
  HAL_GPIO_Init(GPIOC, &gpio);
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9, GPIO_PIN_RESET);
}

/**
 * @brief Initialize I2C2 in master mode at 100 kHz.
 *
 * TIMINGR values from Figure 5.4
 * PRESC=1, SCLDEL=4, SDADEL=2, SCLH=0xF, SCLL=0x13
 */
void I2C2_Init(void)
{
  __HAL_RCC_I2C2_CLK_ENABLE();

  I2C2->TIMINGR = (1U << 28) | (4U << 20) | (2U << 16) | (0xFU << 8) | 0x13U;

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
 * @return            Byte read from the device, or 0xFF/0xFE on NACK error
 */
uint8_t I2C2_ReadRegister(uint8_t slave_addr, uint8_t reg_addr)
{
  // Step 1: Set up write transaction (1 byte, no AUTOEND)
  I2C2->CR2 &= ~((0xFFU << 16) | (0x3FFU << 0));
  I2C2->CR2 |= (1U << 16) | ((uint32_t)slave_addr << 1);
  I2C2->CR2 &= ~(I2C_CR2_RD_WRN | I2C_CR2_AUTOEND);
  I2C2->CR2 |= I2C_CR2_START;

  // Step 2: Wait for TXIS (address acknowledged) or NACKF (no response)
  while (!(I2C2->ISR & (I2C_ISR_TXIS | I2C_ISR_NACKF)));
  if (I2C2->ISR & I2C_ISR_NACKF)
  {
    I2C2->ICR |= I2C_ICR_NACKCF;
    I2C2->CR2 |= I2C_CR2_STOP;
    while (I2C2->ISR & I2C_ISR_BUSY);
    return 0xFF;
  }

  // Step 3: Transmit the register address
  I2C2->TXDR = reg_addr;

  // Step 4: Wait for transfer complete (all NBYTES sent, waiting for restart/stop)
  while (!(I2C2->ISR & I2C_ISR_TC));

  // Step 5: Restart as a read transaction
  I2C2->CR2 &= ~((0xFFU << 16) | (0x3FFU << 0));
  I2C2->CR2 |= (1U << 16) | ((uint32_t)slave_addr << 1) | I2C_CR2_RD_WRN;
  I2C2->CR2 &= ~I2C_CR2_AUTOEND;
  I2C2->CR2 |= I2C_CR2_START;

  // Step 6: Wait for RXNE (data received) or NACKF
  while (!(I2C2->ISR & (I2C_ISR_RXNE | I2C_ISR_NACKF)));
  if (I2C2->ISR & I2C_ISR_NACKF)
  {
    I2C2->ICR |= I2C_ICR_NACKCF;
    I2C2->CR2 |= I2C_CR2_STOP;
    while (I2C2->ISR & I2C_ISR_BUSY);
    return 0xFE;
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
 * @brief Write a single byte to a slave device register.
 *
 * Sends START, slave address, register address, data, STOP.
 *
 * @param slave_addr  7-bit I2C slave address
 * @param reg_addr    Device register address to write
 * @param data        Byte to write
 */
void I2C2_WriteRegister(uint8_t slave_addr, uint8_t reg_addr, uint8_t data)
{
  // 2-byte write: reg_addr then data, no AUTOEND
  I2C2->CR2 &= ~((0xFFU << 16) | (0x3FFU << 0));
  I2C2->CR2 |= (2U << 16) | ((uint32_t)slave_addr << 1);
  I2C2->CR2 &= ~(I2C_CR2_RD_WRN | I2C_CR2_AUTOEND);
  I2C2->CR2 |= I2C_CR2_START;

  // Send register address
  while (!(I2C2->ISR & (I2C_ISR_TXIS | I2C_ISR_NACKF)));
  if (I2C2->ISR & I2C_ISR_NACKF)
  {
    I2C2->ICR |= I2C_ICR_NACKCF;
    I2C2->CR2 |= I2C_CR2_STOP;
    while (I2C2->ISR & I2C_ISR_BUSY);
    return;
  }
  I2C2->TXDR = reg_addr;

  // Send data byte
  while (!(I2C2->ISR & (I2C_ISR_TXIS | I2C_ISR_NACKF)));
  if (I2C2->ISR & I2C_ISR_NACKF)
  {
    I2C2->ICR |= I2C_ICR_NACKCF;
    I2C2->CR2 |= I2C_CR2_STOP;
    while (I2C2->ISR & I2C_ISR_BUSY);
    return;
  }
  I2C2->TXDR = data;

  // Wait for all bytes transmitted
  while (!(I2C2->ISR & I2C_ISR_TC));
  I2C2->CR2 |= I2C_CR2_STOP;
  while (I2C2->ISR & I2C_ISR_BUSY);
}

/**
 * @brief Read multiple consecutive registers from an I2C slave device.
 *
 * Executes a write to select the starting register address followed by a
 * restart read of `len` bytes. The register address must have its MSB set to
 * enable auto-increment on the L3GD20
 *
 * @param slave_addr  7-bit I2C slave address
 * @param reg_addr    Starting register address (MSB set for auto-increment)
 * @param buf         Buffer to store received bytes
 * @param len         Number of bytes to read
 */
void I2C2_ReadRegisters(uint8_t slave_addr, uint8_t reg_addr, uint8_t *buf, uint8_t len)
{
  // Write phase: send starting register address (no AUTOEND)
  I2C2->CR2 &= ~((0xFFU << 16) | (0x3FFU << 0));
  I2C2->CR2 |= (1U << 16) | ((uint32_t)slave_addr << 1);
  I2C2->CR2 &= ~(I2C_CR2_RD_WRN | I2C_CR2_AUTOEND);
  I2C2->CR2 |= I2C_CR2_START;

  while (!(I2C2->ISR & (I2C_ISR_TXIS | I2C_ISR_NACKF)));
  if (I2C2->ISR & I2C_ISR_NACKF)
  {
    I2C2->ICR |= I2C_ICR_NACKCF;
    I2C2->CR2 |= I2C_CR2_STOP;
    while (I2C2->ISR & I2C_ISR_BUSY);
    return;
  }
  I2C2->TXDR = reg_addr;

  while (!(I2C2->ISR & I2C_ISR_TC));

  // Read phase: restart and read len bytes (no AUTOEND)
  I2C2->CR2 &= ~((0xFFU << 16) | (0x3FFU << 0));
  I2C2->CR2 |= ((uint32_t)len << 16) | ((uint32_t)slave_addr << 1) | I2C_CR2_RD_WRN;
  I2C2->CR2 &= ~I2C_CR2_AUTOEND;
  I2C2->CR2 |= I2C_CR2_START;

  for (uint8_t i = 0; i < len; i++)
  {
    while (!(I2C2->ISR & (I2C_ISR_RXNE | I2C_ISR_NACKF)));
    if (I2C2->ISR & I2C_ISR_NACKF)
    {
      I2C2->ICR |= I2C_ICR_NACKCF;
      I2C2->CR2 |= I2C_CR2_STOP;
      while (I2C2->ISR & I2C_ISR_BUSY);
      return;
    }
    buf[i] = (uint8_t)I2C2->RXDR;
  }

  // Wait for TC (set after last byte read from RXDR), then stop
  while (!(I2C2->ISR & I2C_ISR_TC));
  I2C2->CR2 |= I2C_CR2_STOP;
  while (I2C2->ISR & I2C_ISR_BUSY);
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
