/* TIM5 CH2 output-compare scheduler — GPIO edges at 1 µs resolution.
 * Pins stay GPIO (PB2 has no timer AF). TIM5 CNT is the 1 MHz timebase. */
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
  return TIM5->CNT;
}

static void arm_next(void)
{
  uint32_t now = t5now();
  uint32_t soon = now + 50000u; /* 50 ms cap */
  uint8_t any = 0;
  for (uint8_t i = 1; i <= MAX_CYL; i++) {
    if (ignCh[i].armed) {
      uint32_t t = ignCh[i].isOn ? ignCh[i].offAt : ignCh[i].onAt;
      if (!any || (int32_t)(t - soon) < 0) { soon = t; any = 1; }
    }
    if (injCh[i].armed) {
      uint32_t t = injCh[i].isOn ? injCh[i].offAt : injCh[i].onAt;
      if (!any || (int32_t)(t - soon) < 0) { soon = t; any = 1; }
    }
  }
  if (!any)
    soon = now + 20000u;
  if ((int32_t)(soon - now) < 2)
    soon = now + 2u;
  TIM5->CCR2 = soon;
  TIM5->DIER |= TIM_DIER_CC2IE;
}

void ECU_Oc_Init(void)
{
  if (htim5.Instance == NULL)
    return;
  TIM5->CCMR1 &= ~(TIM_CCMR1_OC2M | TIM_CCMR1_CC2S);
  TIM5->CCER &= ~TIM_CCER_CC2E;
  TIM5->SR = ~TIM_SR_CC2IF;
  TIM5->CCR2 = TIM5->CNT + 1000u;
  TIM5->DIER |= TIM_DIER_CC2IE;
}

void ECU_Oc_CancelAll(void)
{
  for (uint8_t i = 1; i <= MAX_CYL; i++) {
    ignCh[i].armed = 0;
    injCh[i].armed = 0;
    ECU_IGN_LO(i);
    ECU_INJ_LO(i);
    coilState[i] = 0;
    injOn[i] = 0;
  }
}

uint8_t ECU_Oc_IgnBusy(uint8_t ch)
{
  return (ch <= MAX_CYL && ignCh[ch].armed) ? 1u : 0u;
}
uint8_t ECU_Oc_InjBusy(uint8_t ch)
{
  return (ch <= MAX_CYL && injCh[ch].armed) ? 1u : 0u;
}

static void pulse(OcCh *c, uint32_t delayUs, uint32_t widthUs)
{
  if (widthUs < 50u) widthUs = 50u;
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
  if (ch < 1 || ch > MAX_CYL) return;
  pulse(&ignCh[ch], delayUs, widthUs);
}
void ECU_Oc_PulseInj(uint8_t ch, uint32_t delayUs, uint32_t widthUs)
{
  if (ch < 1 || ch > MAX_CYL) return;
  pulse(&injCh[ch], delayUs, widthUs);
}

static void step(OcCh *c, uint32_t now, uint8_t ch, uint8_t isIgn)
{
  if (!c->armed) return;
  if (!c->isOn && (int32_t)(now - c->onAt) >= 0) {
    if (isIgn) { ECU_IGN_HI(ch); coilState[ch] = 1; coilStartUs[ch] = micros(); }
    else       { ECU_INJ_HI(ch); injOn[ch] = 1; }
    c->isOn = 1;
  }
  if (c->isOn && (int32_t)(now - c->offAt) >= 0) {
    if (isIgn) { ECU_IGN_LO(ch); coilState[ch] = 0; coilFired[ch] = 1; }
    else       { ECU_INJ_LO(ch); injOn[ch] = 0; }
    c->armed = 0;
    c->isOn = 0;
  }
}

void ECU_Oc_IRQ(void)
{
  if ((TIM5->SR & TIM_SR_CC2IF) == 0)
    return;
  TIM5->SR = ~TIM_SR_CC2IF;
  uint32_t now = t5now();
  for (uint8_t i = 1; i <= MAX_CYL; i++) {
    step(&ignCh[i], now, i, 1);
    step(&injCh[i], now, i, 0);
  }
  arm_next();
}
