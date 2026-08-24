/**
 * TorquEFI / STRIX — interrupt handlers
 * TIM5 crank, TIM2 cam1, TIM3 cam2
 */
#include "main.h"
#include "stm32f4xx_it.h"

extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim5;
extern ADC_HandleTypeDef hadc1;

/* Optional UART if CubeMX enables USART6 */
#ifdef HAL_UART_MODULE_ENABLED
extern UART_HandleTypeDef huart6;
#endif

void NMI_Handler(void)
{
  while (1) {
  }
}

void HardFault_Handler(void)
{
  while (1) {
  }
}

void MemManage_Handler(void)
{
  while (1) {
  }
}

void BusFault_Handler(void)
{
  while (1) {
  }
}

void UsageFault_Handler(void)
{
  while (1) {
  }
}

void SVC_Handler(void)
{
}

void DebugMon_Handler(void)
{
}

void PendSV_Handler(void)
{
}

void SysTick_Handler(void)
{
  HAL_IncTick();
}

/* Crank — PA0 TIM5_CH1 */
void TIM5_IRQHandler(void)
{
  HAL_TIM_IRQHandler(&htim5);
}

/* Cam 1 — PA15 TIM2_CH1 */
void TIM2_IRQHandler(void)
{
  HAL_TIM_IRQHandler(&htim2);
}

/* Cam 2 — PB4 TIM3_CH1 */
void TIM3_IRQHandler(void)
{
  HAL_TIM_IRQHandler(&htim3);
}

void ADC_IRQHandler(void)
{
  HAL_ADC_IRQHandler(&hadc1);
}

#ifdef HAL_UART_MODULE_ENABLED
void USART6_IRQHandler(void)
{
  HAL_UART_IRQHandler(&huart6);
}
#endif
