#include "stm32f0xx_hal.h"

void init_timer2(void) {
  // Enable TIM2 in the RCC
  // TODO: Check that this is right
  RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;

  // Configure the Timer for 4 Hz
  // f = clock / ((prescaler + 1) * (arr + 1))
  // clock = 8 MHz
  // f = 4 Hz
  // So denominator needs to be 2 MHz
  // (prescaler + 1) = (arr + 1) = (7999 + 1) * (249 + 1) = 2_000_000
  TIM2->PSC = 7999;
  TIM2->ARR = 249;

  // Enable the Update Interrupt
  TIM2->DIER |= TIM_DIER_UIE;
  // Enable/Start the Timer
  TIM2->CR1 |= TIM_CR1_CEN;

  // Enable the Interrupt in the NVIC
  NVIC_EnableIRQ(TIM2_IRQn);
  // TODO: Check if needed
  // NVIC_SetPriority(TIM2_IRQn, 1);  // Optional: set priority

}

void TIM2_IRQHandler(void)
{
  // Toggle both LEDs
  HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_8 | GPIO_PIN_9);

  // Clear the update interrupt flag
  TIM2->SR &= ~TIM_SR_UIF;
}
