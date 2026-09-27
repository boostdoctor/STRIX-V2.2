/* TIM5 CH2 compare = one-shot scheduler. IRQ only while a pulse is queued.
 * Overdue edges are applied in thread context so CC2 cannot spin at 2 µs. */
#include "main.h"
#include "ecu_oc.h"
#include "ecu_pins.h"
#include "ecu_runtime.h"
#include "ecu_config.h"

extern TIM_HandleTypeDef htim5;

typedef struct {
  uint32_t onAt;
  uint32_t offAt;
  uint8_t  armed;
  uint8_t  isOn;
} OcCh;

static OcCh ignCh[MAX_CYL + 1];
static OcCh injCh[MAX_CYL + 1];

static uint32_t t5now(void)
{
  return (htim5.Instance != NULL) ? TIM5->CNT : 0u;
}

static void apply_due(uint32_t now)
{
  for (uint8_t i = 1; i <= MAX_CYL; i++) {
    OcCh *c = &ignCh[i];
    if (c->armed && !c->isOn && (int32_t)(now - c->onAt) >= 0) {
      ECU_IGN_HI(i);
      coilState[i] = 1;
      coilStartUs[i] = micros();
      c->isOn = 1;
    }
    if (c->armed && c->isOn && (int32_t)(now - c->offAt) >= 0) {
      ECU_IGN_LO(i);
      coilState[i] = 0;
      coilFired[i] = 1;
      c->armed = 0;
      c->isOn = 0;
    }
    c = &injCh[i];
    if (c->armed && !c->isOn && (int32_t)(now - c->onAt) >= 0) {
      ECU_INJ_HI(i);
      injOn[i] = 1;
      c->isOn = 1;
    }
    if (c->armed && c->isOn && (int32_t)(now - c->offAt) >= 0) {
      ECU_INJ_LO(i);
      injOn[i] = 0;
      c->armed = 0;
      c->isOn = 0;
    }
  }
}

static void arm_next(void)
{
  if (htim5.Instance == NULL)
    return;
  uint32_t now = t5now();
  apply_due(now);
  uint32_t soon = 0;
  uint8_t any = 0;
  for (uint8_t i = 1; i <= MAX_CYL; i++) {
    OcCh *list[2] = { &ignCh[i], &injCh[i] };
    for (uint8_t k = 0; k < 2; k++) {
      OcCh *c = list[k];
      if (!c->armed) continue;
      uint32_t t = c->isOn ? c->offAt : c->onAt;
      if (!any || (int32_t)(t - soon) < 0) {
        soon = t;
        any = 1;
      }
    }
  }
  if (!any) {
    TIM5->DIER &= ~TIM_DIER_CC2IE;
    return;
  }
  /* At least 20 µs in the future — never CCR == CNT (IRQ storm). */
  if ((int32_t)(soon - now) < 20)
    soon = now + 20u;
  TIM5->CCR2 = soon;
  TIM5->SR = ~TIM_SR_CC2IF;
  TIM5->DIER |= TIM_DIER_CC2IE;
}

void ECU_Oc_Init(void)
{
  if (htim5.Instance == NULL)
    return;
  TIM5->CCMR1 &= ~(TIM_CCMR1_OC2M | TIM_CCMR1_CC2S);
  TIM5->CCER &= ~TIM_CCER_CC2E; /* do not take PA1 (MAP) */
  TIM5->DIER &= ~TIM_DIER_CC2IE;
  TIM5->SR = ~TIM_SR_CC2IF;
}

void ECU_Oc_CancelAll(void)
{
  for (uint8_t i = 1; i <= MAX_CYL; i++) {
    ignCh[i].armed = ignCh[i].isOn = 0;
    injCh[i].armed = injCh[i].isOn = 0;
    ECU_IGN_LO(i);
    ECU_INJ_LO(i);
    coilState[i] = 0;
    injOn[i] = 0;
  }
  if (htim5.Instance)
    TIM5->DIER &= ~TIM_DIER_CC2IE;
}

uint8_t ECU_Oc_IgnBusy(uint8_t ch)
{
  return (ch >= 1 && ch <= MAX_CYL && ignCh[ch].armed) ? 1u : 0u;
}
uint8_t ECU_Oc_InjBusy(uint8_t ch)
{
  return (ch >= 1 && ch <= MAX_CYL && injCh[ch].armed) ? 1u : 0u;
}

static void pulse(OcCh *c, uint32_t delayUs, uint32_t widthUs)
{
  if (widthUs < 80u) widthUs = 80u;
  if (widthUs > 25000u) widthUs = 25000u;
  uint32_t now = t5now();
  c->onAt = now + delayUs;
  c->offAt = c->onAt + widthUs;
  c->armed = 1;
  c->isOn = 0;
  arm_next();
}

void ECU_Oc_PulseIgn(uint8_t ch, uint32_t delayUs, uint32_t widthUs)
{
  if (ch >= 1 && ch <= MAX_CYL)
    pulse(&ignCh[ch], delayUs, widthUs);
}
void ECU_Oc_PulseInj(uint8_t ch, uint32_t delayUs, uint32_t widthUs)
{
  if (ch >= 1 && ch <= MAX_CYL)
    pulse(&injCh[ch], delayUs, widthUs);
}

void ECU_Oc_IRQ(void)
{
  if (htim5.Instance == NULL)
    return;
  if ((TIM5->SR & TIM_SR_CC2IF) == 0)
    return;
  TIM5->SR = ~TIM_SR_CC2IF;
  arm_next();
}

/* Planner safety net — apply anything due even if CC2 was masked. */
void ECU_Oc_Poll(void)
{
  apply_due(t5now());
}
