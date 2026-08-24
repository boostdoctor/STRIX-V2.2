#include "main.h"

void HAL_MspInit(void)
{
  __HAL_RCC_SYSCFG_CLK_ENABLE();
  __HAL_RCC_PWR_CLK_ENABLE();
}

void HAL_ADC_MspInit(ADC_HandleTypeDef* hadc)
{
  GPIO_InitTypeDef g = {0};
  if (hadc->Instance == ADC1) {
    __HAL_RCC_ADC1_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    /* PA1-PA7 analog */
    g.Pin = GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|
            GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7;
    g.Mode = GPIO_MODE_ANALOG;
    g.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &g);
    /* PB12 pedal */
    g.Pin = GPIO_PIN_12;
    HAL_GPIO_Init(GPIOB, &g);
  }
}

void HAL_ADC_MspDeInit(ADC_HandleTypeDef* hadc)
{
  if (hadc->Instance == ADC1) {
    __HAL_RCC_ADC1_CLK_DISABLE();
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|
                            GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7);
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_12);
  }
}

void HAL_TIM_IC_MspInit(TIM_HandleTypeDef* htim)
{
  GPIO_InitTypeDef g = {0};
  if (htim->Instance == TIM5) {
    __HAL_RCC_TIM5_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    g.Pin = GPIO_PIN_0;
    g.Mode = GPIO_MODE_AF_PP;
    g.Pull = GPIO_PULLUP;
    g.Speed = GPIO_SPEED_FREQ_HIGH;
    g.Alternate = GPIO_AF2_TIM5;
    HAL_GPIO_Init(GPIOA, &g);
    HAL_NVIC_SetPriority(TIM5_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(TIM5_IRQn);
  } else if (htim->Instance == TIM2) {
    __HAL_RCC_TIM2_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    g.Pin = GPIO_PIN_15;
    g.Mode = GPIO_MODE_AF_PP;
    g.Pull = GPIO_PULLUP;
    g.Speed = GPIO_SPEED_FREQ_HIGH;
    g.Alternate = GPIO_AF1_TIM2;
    HAL_GPIO_Init(GPIOA, &g);
    HAL_NVIC_SetPriority(TIM2_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(TIM2_IRQn);
  }
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
  }
}

void HAL_UART_MspInit(UART_HandleTypeDef* huart)
{
  GPIO_InitTypeDef g = {0};
  if (huart->Instance == USART6) {
    __HAL_RCC_USART6_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    /* PC6 TX, PC7 RX AF8 */
    g.Pin = GPIO_PIN_6|GPIO_PIN_7;
    g.Mode = GPIO_MODE_AF_PP;
    g.Pull = GPIO_PULLUP;
    g.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    g.Alternate = GPIO_AF8_USART6;
    HAL_GPIO_Init(GPIOC, &g);
    HAL_NVIC_SetPriority(USART6_IRQn, 3, 0);
    HAL_NVIC_EnableIRQ(USART6_IRQn);
  }
}

void HAL_UART_MspDeInit(UART_HandleTypeDef* huart)
{
  if (huart->Instance == USART6) {
    __HAL_RCC_USART6_CLK_DISABLE();
    HAL_GPIO_DeInit(GPIOC, GPIO_PIN_6|GPIO_PIN_7);
    HAL_NVIC_DisableIRQ(USART6_IRQn);
  }
}

void HAL_TIM_PWM_MspInit(TIM_HandleTypeDef* htim)
{
  GPIO_InitTypeDef g = {0};
  if (htim->Instance == TIM4) {
    __HAL_RCC_TIM4_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    g.Pin = GPIO_PIN_8;
    g.Mode = GPIO_MODE_AF_PP;
    g.Pull = GPIO_NOPULL;
    g.Speed = GPIO_SPEED_FREQ_LOW;
    g.Alternate = GPIO_AF2_TIM4;
    HAL_GPIO_Init(GPIOB, &g);
  } else if (htim->Instance == TIM1) {
    __HAL_RCC_TIM1_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    /* PA8 TIM1_CH1 ETB, PA10 TIM1_CH3 VVT1 */
    g.Pin = GPIO_PIN_8|GPIO_PIN_10;
    g.Mode = GPIO_MODE_AF_PP;
    g.Pull = GPIO_NOPULL;
    g.Speed = GPIO_SPEED_FREQ_HIGH;
    g.Alternate = GPIO_AF1_TIM1;
    HAL_GPIO_Init(GPIOA, &g);
    /* PB14 TIM1_CH2N */
    g.Pin = GPIO_PIN_14;
    g.Alternate = GPIO_AF1_TIM1;
    HAL_GPIO_Init(GPIOB, &g);
  }
}

void HAL_TIM_PWM_MspDeInit(TIM_HandleTypeDef* htim)
{
  if (htim->Instance == TIM1) {
    __HAL_RCC_TIM1_CLK_DISABLE();
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_10);
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_14);
  }
}

