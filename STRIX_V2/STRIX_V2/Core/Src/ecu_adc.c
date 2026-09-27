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
#include "dma.h"
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
  if ((ADC1->CR2 & ADC_CR2_ADON) == 0)
    ADC1->CR2 = ADC_CR2_ADON;
  ADC1->SQR1 = 0; /* 1 conversion */
  ADC1->SQR3 = (ch & 0x1Fu);
  ADC1->SR = 0;
  ADC1->CR2 |= ADC_CR2_SWSTART;
  uint32_t t0 = HAL_GetTick();
  while ((ADC1->SR & ADC_SR_EOC) == 0) {
    if ((HAL_GetTick() - t0) > 2u)
      return 0;
  }
  return (uint16_t)ADC1->DR;
}

/* Optional: complete DMA stream setup if MSP left handle uninitialised */
void ECU_DMA_ADC1_Config(ADC_HandleTypeDef *hadc);

static void adc_cfg_rank(uint32_t ch, uint32_t rank)
{
  ADC_ChannelConfTypeDef s = {0};
  s.Channel = ch;
  s.Rank = rank;
  s.SamplingTime = ADC_SAMPLETIME_84CYCLES;
  (void)HAL_ADC_ConfigChannel(&hadc1, &s);
}

void ECU_Adc_Init(void)
{
  memset((void *)adcDmaBuf, 0, sizeof(adcDmaBuf));
  ecuAdcDmaRunning = 0; /* poll only — DMA/HAL_ADC_Init locked the core */

  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_ADC1_CLK_ENABLE();
  GPIO_InitTypeDef g = {0};
  g.Pin = GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_7;
  g.Mode = GPIO_MODE_ANALOG;
  g.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &g);

  /* ADC clock PCLK2/4, ADON, 84-cycle sample. No DMA, no scan. */
  ADC->CCR = (ADC->CCR & ~ADC_CCR_ADCPRE) | ADC_CCR_ADCPRE_0;
  ADC1->CR1 = 0;
  ADC1->CR2 = ADC_CR2_ADON;
  ADC1->SMPR2 = (5u << 3) | (5u << 6) | (7u << 9) | (7u << 12) | (5u << 15) | (5u << 21);
  hadc1.Instance = ADC1;
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
