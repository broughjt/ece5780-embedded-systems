#include "main.h"
#include "stm32f0xx_hal.h"
#include <assert.h>

void SystemClock_Config(void);

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();
  /* Configure the system clock */
  SystemClock_Config();

  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  // 2.2 button
  GPIO_InitTypeDef button_config = {
    GPIO_PIN_0,
    GPIO_MODE_INPUT,
    GPIO_SPEED_FREQ_LOW,
    GPIO_PULLDOWN
  };
  HAL_GPIO_Init(GPIOA, &button_config);

  GPIO_InitTypeDef leds_config = {
    GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9,
    GPIO_MODE_OUTPUT_PP,
    GPIO_SPEED_FREQ_LOW,
    GPIO_NOPULL
  };
  HAL_GPIO_Init(GPIOC, &leds_config);
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6 | GPIO_PIN_9, GPIO_PIN_SET);

  // 2.2 EXTI configuration
  EXTI->RTSR |= EXTI_RTSR_TR0;
  EXTI->IMR |= EXTI_IMR_MR0;

  // TODO: Check if we need `RCC->APB2ENR |= RCC_APB2ENR_SYSCFGCOMPEN;` instead
  __HAL_RCC_SYSCFG_CLK_ENABLE();

  assert((SYSCFG->EXTICR[0] & SYSCFG_EXTICR1_EXTI0) == 0);
  SYSCFG->EXTICR[0] &= ~SYSCFG_EXTICR1_EXTI0;
  assert((SYSCFG->EXTICR[0] & SYSCFG_EXTICR1_EXTI0) == 0);

  // 2.4
  NVIC_EnableIRQ(EXTI0_1_IRQn);
  NVIC_SetPriority(EXTI0_1_IRQn, 3);
  // TODO: Part of part 2.7
  NVIC_SetPriority(SysTick_IRQn, 2);

  while (1) {
    HAL_Delay(500);

    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_6);
  }
  return -1;
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
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

void EXTI0_1_IRQHandler(void)
{
  HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_8 | GPIO_PIN_9);

  for (volatile int i = 0; i < 1500000; i++) {
    // Empty loop -- volatile prevents optimization
  }

  HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_8 | GPIO_PIN_9);

  EXTI->PR |= EXTI_PR_PR0;
}
