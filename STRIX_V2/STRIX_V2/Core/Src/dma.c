/**
 * ADC1 → DMA2 Stream0 Channel 0 (STM32F411)
 * Circular half-word, matches continuous multi-rank scan.
 */
#include "dma.h"

DMA_HandleTypeDef hdma_adc1;

void MX_DMA_Init(void)
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init — priority below crank timers is fine */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
}

/**
 * Note: HAL_ADC_MspInit in stm32f4xx_hal_msp.c must still link
 * __HAL_LINKDMA(hadc, DMA_Handle, hdma_adc1) and configure the stream.
 * If MSP only references hdma_adc1 without Init, add stream setup there
 * or call the block below once after MX_DMA_Init (before HAL_ADC_Start_DMA).
 */
void ECU_DMA_ADC1_Config(ADC_HandleTypeDef *hadc)
{
  if (hadc == NULL) return;
  __HAL_RCC_DMA2_CLK_ENABLE();

  hdma_adc1.Instance = DMA2_Stream0;
  hdma_adc1.Init.Channel = DMA_CHANNEL_0;
  hdma_adc1.Init.Direction = DMA_PERIPH_TO_MEMORY;
  hdma_adc1.Init.PeriphInc = DMA_PINC_DISABLE;
  hdma_adc1.Init.MemInc = DMA_MINC_ENABLE;
  hdma_adc1.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
  hdma_adc1.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
  hdma_adc1.Init.Mode = DMA_CIRCULAR;
  hdma_adc1.Init.Priority = DMA_PRIORITY_HIGH;
  hdma_adc1.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
  if (HAL_DMA_Init(&hdma_adc1) != HAL_OK) {
    /* Do not Error_Handler — fall back to ADC polling so USB/telemetry stay alive */
    hadc->DMA_Handle = NULL;
    return;
  }
  __HAL_LINKDMA(hadc, DMA_Handle, hdma_adc1);
}
