/**
 * DMA for ADC1 (DMA2 Stream0)
 * Required so HAL_ADC_MspInit / DMA2_Stream0_IRQHandler link.
 */
#ifndef __DMA_H
#define __DMA_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

extern DMA_HandleTypeDef hdma_adc1;

void MX_DMA_Init(void);

#ifdef __cplusplus
}
#endif

#endif /* __DMA_H */
