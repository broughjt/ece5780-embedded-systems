#include "main.h"
#include "stm32f0xx_hal.h"

void SystemClock_Config(void);
void USART3_Init(void);
void LED_Init(void);
void transmit_char(char c);
void transmit_string(const char *str);

UART_HandleTypeDef huart3;

volatile uint8_t rx_data = 0;
volatile uint8_t rx_flag = 0;

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
    /* Receive first character: LED color */
    transmit_string("CMD?\r\n");

    while (!rx_flag);
    rx_flag = 0;
    char color = rx_data;

    uint16_t pin;
    const char *color_name;
    switch (color)
    {
      case 'r': pin = GPIO_PIN_6; color_name = "red";    break;
      case 'b': pin = GPIO_PIN_7; color_name = "blue";   break;
      case 'o': pin = GPIO_PIN_8; color_name = "orange"; break;
      case 'g': pin = GPIO_PIN_9; color_name = "green";  break;
      default:
        transmit_string("Error: unknown color\r\n");
        continue;
    }

    /* Receive second character: action (0=off, 1=on, 2=toggle) */
    while (!rx_flag);
    rx_flag = 0;
    char action = rx_data;

    switch (action)
    {
      case '0':
        HAL_GPIO_WritePin(GPIOC, pin, GPIO_PIN_RESET);
        transmit_string("Turned off ");
        transmit_string(color_name);
        transmit_string("\r\n");
        break;
      case '1':
        HAL_GPIO_WritePin(GPIOC, pin, GPIO_PIN_SET);
        transmit_string("Turned on ");
        transmit_string(color_name);
        transmit_string("\r\n");
        break;
      case '2':
        HAL_GPIO_TogglePin(GPIOC, pin);
        transmit_string("Toggled ");
        transmit_string(color_name);
        transmit_string("\r\n");
        break;
      default:
        transmit_string("Error: unknown action\r\n");
        break;
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

  /* Enable receive register not empty interrupt */
  USART3->CR1 |= USART_CR1_RXNEIE;

  /* Enable USART3_4 interrupt in the NVIC */
  HAL_NVIC_SetPriority(USART3_4_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(USART3_4_IRQn);
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
