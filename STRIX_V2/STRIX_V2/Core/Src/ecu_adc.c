/**
 * Timer-triggered multi-channel ADC via DMA (STM32F411)
 *
 * Preferred path: ADC1 continuous scan → DMA2 circular (F411: no TIM9_TRGO).
 * Fallback A: continuous circular DMA (no timer).
 * Fallback B: blocking HAL poll (legacy).
 */
#include "ecu_adc.h"
#include "ecu_pins.h"
#include "main.h"
#include <string.h>

volatile uint16_t adcDmaBuf[ECU_ADC_RANK_COUNT];
volatile uint8_t  ecuAdcDmaRunning = 0;

extern ADC_HandleTypeDef hadc1;

/*
 * TIM9 is optional until CubeMX enables it. Provide a weak stub so projects
 * without TIM9 still link; strong htim9 from tim.c overrides this.
 */
#if defined(HAL_TIM_MODULE_ENABLED)
TIM_HandleTypeDef htim9 __attribute__((weak));
static uint8_t tim9_present(void)
{
  /* Cube-generated init sets Instance = TIM9 */
  return (htim9.Instance == TIM9) ? 1u : 0u;
}
#endif

static uint16_t poll_one(uint32_t ch)
{
  ADC_ChannelConfTypeDef s = {0};
  s.Channel = ch;
  s.Rank = 1;
  s.SamplingTime = ADC_SAMPLETIME_84CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &s) != HAL_OK)
    return 0;
  if (HAL_ADC_Start(&hadc1) != HAL_OK)
    return 0;
  if (HAL_ADC_PollForConversion(&hadc1, 5) != HAL_OK) {
    HAL_ADC_Stop(&hadc1);
    return 0;
  }
  uint16_t v = (uint16_t)HAL_ADC_GetValue(&hadc1);
  HAL_ADC_Stop(&hadc1);
  return v;
}

/* Optional: complete DMA stream setup if MSP left handle uninitialised */
void ECU_DMA_ADC1_Config(ADC_HandleTypeDef *hadc);

void ECU_Adc_Init(void)
{
  memset((void *)adcDmaBuf, 0, sizeof(adcDmaBuf));
  ecuAdcDmaRunning = 0;

#if defined(HAL_ADC_MODULE_ENABLED)
  /* Ensure hdma_adc1 is initialised and linked (idempotent if MSP already did it) */
  if (hadc1.DMA_Handle == NULL)
    ECU_DMA_ADC1_Config(&hadc1);
  /*
   * Try DMA circular into adcDmaBuf.
   * CubeMX: Scan ON, 8 ranks, DMA Continuous Requests ON,
   * ContinuousConvMode ON, ExternalTrig = Software start
   * (F411 cannot select TIM9_TRGO for ADC1).
   *
   * Length = ECU_ADC_RANK_COUNT so each sequence fills one frame.
   */
  if (HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adcDmaBuf, ECU_ADC_RANK_COUNT) == HAL_OK) {
    ecuAdcDmaRunning = 1;
  }
#endif

#if defined(HAL_TIM_MODULE_ENABLED)
  /*
   * Continuous+DMA needs no timer trigger.
   * Optional: if you use TIM4_TRGO instead, start TIM4 base here.
   */
  if (tim9_present()) {
    /* unused on F411 continuous path; left for boards that still enable TIM9 */
    (void)0;
  }
#endif
}

void ECU_Adc_Stop(void)
{
#if defined(HAL_ADC_MODULE_ENABLED)
  if (ecuAdcDmaRunning) {
    HAL_ADC_Stop_DMA(&hadc1);
    ecuAdcDmaRunning = 0;
  }
#endif
#if defined(HAL_TIM_MODULE_ENABLED)
  if (tim9_present())
    (void)HAL_TIM_Base_Stop(&htim9);
#endif
}

uint16_t readAdc(uint32_t ch)
{
  if (ecuAdcDmaRunning) {
    switch (ch) {
      case ECU_ADC_CH_MAP:   return adcDmaBuf[ECU_ADC_IX_MAP];
      case ECU_ADC_CH_TPS:   return adcDmaBuf[ECU_ADC_IX_TPS];
      case ECU_ADC_CH_CLT:   return adcDmaBuf[ECU_ADC_IX_CLT];
      case ECU_ADC_CH_IAT:   return adcDmaBuf[ECU_ADC_IX_IAT];
      case ECU_ADC_CH_O2:    return adcDmaBuf[ECU_ADC_IX_O2];
      case ECU_ADC_CH_VBATT: return adcDmaBuf[ECU_ADC_IX_VBATT];
      /* FLEX is frequency on PA6 — not ADC */
      case ECU_ADC_CH_FLEX:  return 0;
      case ECU_ADC_CH_PEDAL: return 0; /* not in current rank list */
      default: return 0;
    }
  }
  /* Legacy blocking path if DMA not configured yet */
  return poll_one(ch);
}

void ECU_Adc_Snapshot(uint16_t out[ECU_ADC_RANK_COUNT])
{
  if (!out) return;
  for (unsigned i = 0; i < ECU_ADC_RANK_COUNT; i++)
    out[i] = adcDmaBuf[i];
}
