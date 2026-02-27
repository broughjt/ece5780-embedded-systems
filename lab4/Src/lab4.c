#include "main.h"
#include "stm32f0xx_hal.h"

void SystemClock_Config(void);
void USART3_Init(void);
void LED_Init(void);
void transmit_char(char c);
void transmit_string(const char *str);

UART_HandleTypeDef huart3;

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

  USART3_Init();
  LED_Init();

  while (1)
  {
    /* Wait until a character is received */
    while (!(USART3->ISR & USART_ISR_RXNE));
    char c = USART3->RDR;

    switch (c)
    {
      case 'r': HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_6); break; /* red    */
      case 'b': HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_7); break; /* blue   */
      case 'o': HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_8); break; /* orange */
      case 'g': HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_9); break; /* green  */
      default:  transmit_string("Error: unknown command\r\n"); break;
    }
  }
  return -1;
}

/**
  * @brief  Initialize USART3 on PC10 (TX) and PC11 (RX) at 115200 baud.
  * @retval None
  */
void USART3_Init(void)
{
  // Step 2: Configure PC10 (USART3_TX) and PC11 (USART3_RX) pins
  __HAL_RCC_GPIOC_CLK_ENABLE();

  GPIO_InitTypeDef gpio_config = {0};
  gpio_config.Pin       = GPIO_PIN_10 | GPIO_PIN_11;
  gpio_config.Mode      = GPIO_MODE_AF_PP;
  gpio_config.Pull      = GPIO_NOPULL;
  gpio_config.Speed     = GPIO_SPEED_FREQ_LOW;
  gpio_config.Alternate = GPIO_AF1_USART3;
  HAL_GPIO_Init(GPIOC, &gpio_config);

  // Step 3: Initialize USART3 at 115200 baud, 8N1
  __HAL_RCC_USART3_CLK_ENABLE();

  huart3.Instance            = USART3;
  huart3.Init.BaudRate       = 115200;
  huart3.Init.WordLength     = UART_WORDLENGTH_8B;
  huart3.Init.StopBits       = UART_STOPBITS_1;
  huart3.Init.Parity         = UART_PARITY_NONE;
  huart3.Init.Mode           = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl      = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling   = UART_OVERSAMPLING_16;
  HAL_UART_Init(&huart3);
}

/**
  * @brief  Transmit a single character over USART3 (blocking).
  * @param  c: character to send
  * @retval None
  */
void transmit_char(char c)
{
  /* Wait until the transmit data register is empty */
  while (!(USART3->ISR & USART_ISR_TXE));
  /* Writing to TDR clears TXE automatically */
  USART3->TDR = c;
}

/**
  * @brief  Initialize PC6 (red), PC7 (blue), PC8 (orange), PC9 (green) as outputs.
  * @retval None
  */
void LED_Init(void)
{
  /* GPIOC clock already enabled in USART3_Init, but safe to call again */
  __HAL_RCC_GPIOC_CLK_ENABLE();

  GPIO_InitTypeDef gpio_config = {0};
  gpio_config.Pin   = GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9;
  gpio_config.Mode  = GPIO_MODE_OUTPUT_PP;
  gpio_config.Pull  = GPIO_NOPULL;
  gpio_config.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &gpio_config);

  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6 | GPIO_PIN_7, GPIO_PIN_SET);
}

/**
  * @brief  Transmit a null-terminated string over USART3 (blocking).
  * @param  str: pointer to the string to send
  * @retval None
  */
void transmit_string(const char *str)
{
  while (*str != '\0')
  {
    transmit_char(*str);
    str++;
  }
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
