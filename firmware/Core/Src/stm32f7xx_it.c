#include "main.h"
#include "stm32f7xx_it.h"

void SysTick_Handler(void) {
  HAL_IncTick(); 
}

void EXTI9_5_IRQHandler(void) {
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_5);
}