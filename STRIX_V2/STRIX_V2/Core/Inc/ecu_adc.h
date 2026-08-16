/**
 * STRIX V2 — Timer-triggered ADC scan + DMA buffer
 *
 * TIM9 Update TRGO starts a full regular sequence (8 ranks).
 * DMA2 stores results into adcDmaBuf[] (circular).
 * readAdc() / ECU_Adc_Raw() read the latest frame with no polling wait.
 *
 * CubeMX: see CUBEMX_ADC_DMA.md
 */
#ifndef ECU_ADC_H
#define ECU_ADC_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** Number of ranks in the regular sequence (must match CubeMX) */
#define ECU_ADC_RANK_COUNT  8

/** Default scan rate when TIM9 is used as trigger (Hz) */
#ifndef ECU_ADC_SCAN_HZ
#define ECU_ADC_SCAN_HZ     1000u
#endif

/** Buffer indices — order must match ADC regular ranks 1..8 */
enum {
  ECU_ADC_IX_MAP = 0,
  ECU_ADC_IX_TPS,
  ECU_ADC_IX_CLT,
  ECU_ADC_IX_IAT,
  ECU_ADC_IX_O2,
  ECU_ADC_IX_KNOCK,
  ECU_ADC_IX_VBATT,
  ECU_ADC_IX_PEDAL
};

/** Latest DMA frame (volatile — written by DMA) */
extern volatile uint16_t adcDmaBuf[ECU_ADC_RANK_COUNT];

/** 1 = DMA running; 0 = polling fallback */
extern volatile uint8_t  ecuAdcDmaRunning;

/**
 * Start timer-triggered DMA scan (or continuous DMA / poll fallback).
 * Call once from ECU_Init after MX_ADC1_Init / MX_TIM9_Init / MX_DMA_Init.
 */
void ECU_Adc_Init(void);

/** Stop DMA (e.g. before flash heavy work — optional) */
void ECU_Adc_Stop(void);

/** Raw sample by buffer index 0..7 */
static inline uint16_t ECU_Adc_Raw(unsigned ix)
{
  if (ix >= ECU_ADC_RANK_COUNT) return 0;
  return adcDmaBuf[ix];
}

/**
 * Compatible with existing call sites: map ADC_CHANNEL_x → latest sample.
 * Non-blocking when DMA is running.
 */
uint16_t readAdc(uint32_t ch);

/** Snapshot all ranks into a local array (optional) */
void ECU_Adc_Snapshot(uint16_t out[ECU_ADC_RANK_COUNT]);

#ifdef __cplusplus
}
#endif

#endif /* ECU_ADC_H */
