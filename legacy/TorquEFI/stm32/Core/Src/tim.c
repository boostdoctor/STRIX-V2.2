/**
 * TIM5 = crank PA0 (CH1 IC)
 * TIM2 = cam1  PA15 (CH1 IC)  — PA15 must leave JTAG JTDI
 * TIM3 = cam2  PB4  (CH1 IC)  — PB4 must leave JTAG NJTRST
 *
 * Timer clock: APB1 timers = 96 MHz (APB1=48, x2 when presc!=1)
 * Prescaler 95 → 1 MHz tick (1 µs), Period 0xFFFFFFFF free-run.
 */
#include "main.h"

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;

static void tim_ic_1mhz(TIM_HandleTypeDef *htim, TIM_TypeDef *inst)
{
  htim->Instance = inst;
  htim->Init.Prescaler = 95; /* 96 MHz / 96 = 1 MHz */
  htim->Init.CounterMode = TIM_COUNTERMODE_UP;
  htim->Init.Period = 0xFFFFFFFFu;
  htim->Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim->Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_IC_Init(htim) != HAL_OK)
    Error_Handler();

  TIM_MasterConfigTypeDef sMaster = {0};
  sMaster.MasterOutputTrigger = TIM_TRGO_RESET;
  sMaster.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(htim, &sMaster) != HAL_OK)
    Error_Handler();

  TIM_IC_InitTypeDef sIC = {0};
  sIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sIC.ICPrescaler = TIM_ICPSC_DIV1;
  sIC.ICFilter = 8; /* stronger HW debounce */ /* light digital filter */
  if (HAL_TIM_IC_ConfigChannel(htim, &sIC, TIM_CHANNEL_1) != HAL_OK)
    Error_Handler();
}

void MX_TIM5_Init(void)
{
  /* Crank PA0 */
  tim_ic_1mhz(&htim5, TIM5);
}

void MX_TIM2_Init(void)
{
  /* Cam1 PA15 */
  tim_ic_1mhz(&htim2, TIM2);
}

void MX_TIM3_Init(void)
{
  /* Cam2 PB4 */
  tim_ic_1mhz(&htim3, TIM3);
}

void MX_TIM1_Init(void)
{
  /* PWM outputs — minimal safe init if CubeMX missing */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 95;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 999;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
    Error_Handler();
}

void MX_TIM4_Init(void)
{
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 95;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 999;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
    Error_Handler();
}
