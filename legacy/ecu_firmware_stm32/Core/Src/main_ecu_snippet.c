/**
 * Snippet — merge into CubeIDE-generated main.c / stm32f4xx_it.c
 * Do not compile this file as-is; copy the fragments.
 */

#if 0
/* ---- main.c ---- */
#include "ecu_app.h"

/* put near top */
static uint8_t uartRxByte;

int main(void)
{
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_USART2_UART_Init();
  MX_TIM2_Init();

  ECU_Init();

  HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1);
  HAL_UART_Receive_IT(&huart2, &uartRxByte, 1);

  while (1) {
    ECU_Loop();
  }
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM2) {
    uint32_t c = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
    ECU_CrankCapture(c);
  }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART2) {
    ECU_UART_RxByte(uartRxByte);
    HAL_UART_Receive_IT(&huart2, &uartRxByte, 1);
  }
}
#endif
