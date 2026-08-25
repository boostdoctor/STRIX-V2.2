/**
 * STRIX V2 — Continuous multi-rank ADC + DMA buffer
 *
 * ADC1 continuous scan → DMA2 Stream0 circular.
 * Ranks (must match MX_ADC1_Init):
 *   0 MAP PA1, 1 TPS PA2, 2 CLT PA3, 3 IAT PA4, 4 O2 PA5, 5 VBATT PA7
 * PA6 is FLEX frequency input (not in this sequence).
 */
#ifndef ECU_ADC_H
#define ECU_ADC_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** Number of ranks in the regular sequence (must match CubeMX / MX_ADC1_Init) */
#define ECU_ADC_RANK_COUNT  6

/** Buffer indices — order must match ADC regular ranks 1..6 */
enum {
  ECU_ADC_IX_MAP = 0,
  ECU_ADC_IX_TPS,
  ECU_ADC_IX_CLT,
  ECU_ADC_IX_IAT,
  ECU_ADC_IX_O2,
  ECU_ADC_IX_VBATT
};

/** Latest DMA frame (volatile — written by DMA) */
extern volatile uint16_t adcDmaBuf[ECU_ADC_RANK_COUNT];

/** 1 = DMA running; 0 = polling fallback */
extern volatile uint8_t  ecuAdcDmaRunning;

void ECU_Adc_Init(void);
void ECU_Adc_Stop(void);

static inline uint16_t ECU_Adc_Raw(unsigned ix)
{
  if (ix >= ECU_ADC_RANK_COUNT) return 0;
  return adcDmaBuf[ix];
}

uint16_t readAdc(uint32_t ch);
void ECU_Adc_Snapshot(uint16_t out[ECU_ADC_RANK_COUNT]);

#ifdef __cplusplus
}
#endif

#endif /* ECU_ADC_H */
