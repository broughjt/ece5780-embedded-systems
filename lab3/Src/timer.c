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

void init_timer3(void) {
  // Enable timer 3 in the RCC
  RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;

  // Configure the timer for 800Hz
  TIM3->PSC = 0;
  TIM3->ARR = 9999;

  // Configure CCMR1 for PWM Output
  // Set channels 1 & 2 to output mode (CC1S = 00, CC2S = 00)
  TIM3->CCMR1 &= ~(TIM_CCMR1_CC1S | TIM_CCMR1_CC2S);
  // Set channel 1 to PWM Mode 2 (OC1M = 111)
  TIM3->CCMR1 |= (TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_0);
  // Set channel 2 to PWM Mode 1 (OC2M = 110)
  TIM3->CCMR1 |= (TIM_CCMR1_OC2M_2 | TIM_CCMR1_OC2M_1);
  // Enable output compare preload for both channels
  TIM3->CCMR1 |= (TIM_CCMR1_OC1PE | TIM_CCMR1_OC2PE);

  // Enable Output in CCER
  TIM3->CCER |= (TIM_CCER_CC1E | TIM_CCER_CC2E);

  // Set CCR Values to 20% Duty Cycle
  // Note these are inverted, so CCR1 needs to be close to ARR to be dim, CCR2
  // should be close to zero to be dim
  TIM3->CCR1 = 8500;
  TIM3->CCR2 = 500;

  // Start the timer
  TIM3->CR1 |= TIM_CR1_CEN;
}

void TIM2_IRQHandler(void)
{
  // Toggle both LEDs
  HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_8 | GPIO_PIN_9);

  // Clear the update interrupt flag
  TIM2->SR &= ~TIM_SR_UIF;
}
