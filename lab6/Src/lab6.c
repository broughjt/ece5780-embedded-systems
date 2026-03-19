#include "main.h"
#include "stm32f0xx_hal.h"

void SystemClock_Config(void);
void GPIO_Init(void);
void ADC_Init(void);
void DAC_Init(void);

// Sine wave: 8-bit, 32 samples/cycle
static const uint8_t sine_table[32] = {
  127,151,175,197,216,232,244,251,254,251,244,
  232,216,197,175,151,127,102,78,56,37,21,9,2,0,2,9,21,37,56,78,102
};

int main(void)
{
  HAL_Init();
  SystemClock_Config();

  GPIO_Init();
  ADC_Init();
  DAC_Init();

  uint32_t dac_idx = 0;

  while (1)
  {
    // Checkoff 6.1: read potentiometer, drive LEDs
    /* uint8_t adc_val = (uint8_t)ADC1->DR; */

    /* HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6,  adc_val >  64 ? GPIO_PIN_SET : GPIO_PIN_RESET); */
    /* HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7,  adc_val > 128 ? GPIO_PIN_SET : GPIO_PIN_RESET); */
    /* HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8,  adc_val > 191 ? GPIO_PIN_SET : GPIO_PIN_RESET); */
    /* HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9,  adc_val > 220 ? GPIO_PIN_SET : GPIO_PIN_RESET); */

    // Checkoff 6.2: output sine wave using DAC on PA4
    DAC->DHR8R1 = sine_table[dac_idx];
    dac_idx = (dac_idx + 1) % 32;

    HAL_Delay(1);
  }
  return -1;
}

/**
 * @brief Configure GPIO pins for ADC input and LEDs.
 *
 * PA0: analog mode, no pull-up/down
 * PC6/7/8/9 LEDs: output, push-pull
 */
void GPIO_Init(void)
{
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  GPIO_InitTypeDef gpio = {0};

  // PA0: ADC_IN0 analog input, no pull-up/down
  gpio.Pin  = GPIO_PIN_0;
  gpio.Mode = GPIO_MODE_ANALOG;
  gpio.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &gpio);

  // PA4: DAC_OUT1 analog output, no pull-up/down
  gpio.Pin  = GPIO_PIN_4;
  gpio.Mode = GPIO_MODE_ANALOG;
  gpio.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &gpio);

  // PC6 (red), PC7 (blue), PC8 (orange), PC9 (green) — LED outputs
  gpio.Pin  = GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9;
  gpio.Mode = GPIO_MODE_OUTPUT_PP;
  HAL_GPIO_Init(GPIOC, &gpio);
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9, GPIO_PIN_RESET);
}

void DAC_Init(void)
{
  RCC->APB1ENR |= RCC_APB1ENR_DACEN;

  // Enable channel 1, no hardware trigger (TEN1=0)
  DAC->CR |= DAC_CR_EN1;
}

void ADC_Init(void)
{
  RCC->APB2ENR |= RCC_APB2ENR_ADCEN;
  ADC1->CFGR1 = (2U << ADC_CFGR1_RES_Pos) | ADC_CFGR1_CONT;
  ADC1->CHSELR = ADC_CHSELR_CHSEL0;

  ADC1->CR |= ADC_CR_ADCAL;
  while (ADC1->CR & ADC_CR_ADCAL);

  ADC1->CR |= ADC_CR_ADEN;
  while (!(ADC1->ISR & ADC_ISR_ADRDY));

  ADC1->CR |= ADC_CR_ADSTART;
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

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                              | RCC_CLOCKTYPE_PCLK1;
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
  /* User can add their own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
}

#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* User can add their own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
}
#endif /* USE_FULL_ASSERT */
