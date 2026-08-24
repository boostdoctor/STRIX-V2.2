/**
 * MSP: TIM IC only.
 * ADC MSP lives in adc.c (CubeMX) — do not redefine here.
 */
#include "main.h"

void HAL_MspInit(void)
{
  __HAL_RCC_SYSCFG_CLK_ENABLE();
  __HAL_RCC_PWR_CLK_ENABLE();
}

/** Shared IC GPIO + IRQ setup for crank/cam timers */
static void msp_tim_ic_gpio(TIM_HandleTypeDef* htim)
{
  GPIO_InitTypeDef g = {0};
  if (htim->Instance == TIM5) {
    __HAL_RCC_TIM5_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    /* PA0 TIM5_CH1 AF2 */
    g.Pin = GPIO_PIN_0;
    g.Mode = GPIO_MODE_AF_PP;
    g.Pull = GPIO_PULLUP;
    g.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    g.Alternate = GPIO_AF2_TIM5;
    HAL_GPIO_Init(GPIOA, &g);
    HAL_NVIC_SetPriority(TIM5_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(TIM5_IRQn);
  } else if (htim->Instance == TIM2) {
    __HAL_RCC_TIM2_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    /* PA15 TIM2_CH1 AF1 (was JTDI) */
    g.Pin = GPIO_PIN_15;
    g.Mode = GPIO_MODE_AF_PP;
    g.Pull = GPIO_PULLUP;
    g.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    g.Alternate = GPIO_AF1_TIM2;
    HAL_GPIO_Init(GPIOA, &g);
    HAL_NVIC_SetPriority(TIM2_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(TIM2_IRQn);
  } else if (htim->Instance == TIM3) {
    __HAL_RCC_TIM3_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    /* PB4 TIM3_CH1 AF2 (was NJTRST) */
    g.Pin = GPIO_PIN_4;
    g.Mode = GPIO_MODE_AF_PP;
    g.Pull = GPIO_PULLUP;
    g.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    g.Alternate = GPIO_AF2_TIM3;
    HAL_GPIO_Init(GPIOB, &g);
    HAL_NVIC_SetPriority(TIM3_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(TIM3_IRQn);
  }
}

void HAL_TIM_IC_MspInit(TIM_HandleTypeDef* htim)
{
  msp_tim_ic_gpio(htim);
}

/* Cube may call Base MSP when starting the counter */
void HAL_TIM_Base_MspInit(TIM_HandleTypeDef* htim)
{
  if (htim->Instance == TIM5 || htim->Instance == TIM2 || htim->Instance == TIM3)
    msp_tim_ic_gpio(htim);
}

void HAL_TIM_IC_MspDeInit(TIM_HandleTypeDef* htim)
{
  if (htim->Instance == TIM5) {
    __HAL_RCC_TIM5_CLK_DISABLE();
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_0);
    HAL_NVIC_DisableIRQ(TIM5_IRQn);
  } else if (htim->Instance == TIM2) {
    __HAL_RCC_TIM2_CLK_DISABLE();
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_15);
    HAL_NVIC_DisableIRQ(TIM2_IRQn);
  } else if (htim->Instance == TIM3) {
    __HAL_RCC_TIM3_CLK_DISABLE();
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_4);
    HAL_NVIC_DisableIRQ(TIM3_IRQn);
  }
}

void HAL_TIM_Base_MspDeInit(TIM_HandleTypeDef* htim)
{
  HAL_TIM_IC_MspDeInit(htim);
}

void HAL_TIM_PWM_MspInit(TIM_HandleTypeDef* htim)
{
  if (htim->Instance == TIM1)
    __HAL_RCC_TIM1_CLK_ENABLE();
  else if (htim->Instance == TIM4)
    __HAL_RCC_TIM4_CLK_ENABLE();
}
