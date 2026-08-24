/**
 * TIM5 crank PA0, TIM2 cam1 PA15, TIM3 cam2 PB4
 */
#include "main.h"
#include "stm32f4xx_it.h"

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim5;
extern PCD_HandleTypeDef hpcd_USB_OTG_FS;

void NMI_Handler(void) { while (1) {} }
void HardFault_Handler(void) { while (1) {} }
void MemManage_Handler(void) { while (1) {} }
void BusFault_Handler(void) { while (1) {} }
void UsageFault_Handler(void) { while (1) {} }
void SVC_Handler(void) {}
void DebugMon_Handler(void) {}
void PendSV_Handler(void) {}
void SysTick_Handler(void) { HAL_IncTick(); }

void TIM5_IRQHandler(void) { HAL_TIM_IRQHandler(&htim5); }
void TIM2_IRQHandler(void) { HAL_TIM_IRQHandler(&htim2); }
void TIM3_IRQHandler(void) { HAL_TIM_IRQHandler(&htim3); }

void OTG_FS_IRQHandler(void) { HAL_PCD_IRQHandler(&hpcd_USB_OTG_FS); }
