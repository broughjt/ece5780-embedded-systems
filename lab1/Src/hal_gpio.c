#include <stdint.h>
#include <stm32f0xx_hal.h>
#include <stm32f0xx_hal_gpio.h>

#define GPIO_COUNT 16U

uint32_t expand_bits(uint32_t x) {
    uint32_t y = x & 0xFFFF;

    y = (y | (y << 8)) & 0x00FF00FF;
    y = (y | (y << 4)) & 0x0F0F0F0F;
    y = (y | (y << 2)) & 0x33333333;
    y = (y | (y << 1)) & 0x55555555;

    return y | (y << 1);
}

uint32_t repeat2(uint32_t p) {
  p &= 0x3; // Keep only the low 2 bits
  return p * 0x55555555; // Repeats these two low bits across the whole string
}

void My_HAL_GPIO_Init(GPIO_TypeDef *port, GPIO_InitTypeDef *config)
{
  const uint32_t pin_expanded = expand_bits(config->Pin);
  const uint32_t mode_bitmask = repeat2(config->Mode) & pin_expanded;
  const uint32_t speed_bitmask = repeat2(config->Speed) & pin_expanded;
  const uint32_t pull_bitmask = repeat2(config->Pull) & pin_expanded;

  port->MODER &= ~pin_expanded;
  port->MODER |= mode_bitmask;

  // Push pull 0, open drain 1
  // Hardcode push-pull for now, GPIO_Init struct doesn't let you configure this
  port->OTYPER &= ~config->Mode;

  port->OSPEEDR &= ~pin_expanded;
  port->OSPEEDR |= speed_bitmask;

  port->PUPDR &= ~pin_expanded;
  port->PUPDR |= pull_bitmask;
}

/*
void My_HAL_GPIO_DeInit(GPIO_TypeDef  *GPIOx, uint32_t GPIO_Pin)
{
}
*/


GPIO_PinState My_HAL_GPIO_ReadPin(GPIO_TypeDef* port, uint16_t pin)
{
  assert_param(IS_GPIO_PIN(pin));

  GPIO_PinState status = ((port->IDR & pin) != (uint32_t)GPIO_PIN_RESET) ?
    GPIO_PIN_SET :
    GPIO_PIN_RESET;

  return status;
}

void My_HAL_GPIO_WritePin(GPIO_TypeDef* port, uint16_t pin, GPIO_PinState state)
{
  assert_param(IS_GPIO_PIN(pin));
  assert_param(IS_GPIO_PIN_ACTION(state));

  if (state != GPIO_PIN_RESET)
  {
    port->BSRR = (uint32_t)pin;
  }
  else
  {
    port->BRR = (uint32_t)pin;
  }
}


void My_HAL_GPIO_TogglePin(GPIO_TypeDef* port, uint16_t pin)
{
  uint32_t odr;

  assert_param(IS_GPIO_PIN(pin));
  odr = port->ODR;

  /* Set selected pins that were at low level, and reset ones that were high */
  port->BSRR = ((odr & pin) << GPIO_COUNT) | (~odr & pin);
}

void My_HAL_RCC_GPIOC_CLK_ENABLE() {
  /* SET_BIT(RCC->AHBENR, RCC_AHBENR_GPIOCEN); */
  /* __IO uint32_t t = READ_BIT(RCC->AHBENR, RCC_AHBENR_GPIOCEN); */
  /* UNUSED(t); */

  RCC->AHBENR |= RCC_AHBENR_GPIOCEN | RCC_AHBENR_GPIOAEN;
}
