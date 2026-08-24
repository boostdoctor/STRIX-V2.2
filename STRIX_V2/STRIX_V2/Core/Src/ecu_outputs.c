/* ecu_outputs.c — auto-split from ecu_app.c */
#include "main.h"
#include "ecu_config.h"
#include "ecu_pins.h"
#include "ecu_serial.h"
#include "ecu_flash.h"
#include "ecu_settings.h"
#include "ecu_idle.h"
#include "ecu_maps.h"
#include "ecu_runtime.h"
#include "ecu_internal.h"
#include <string.h>
#include <stdio.h>
#include <math.h>
#include <stdint.h>

/* ---- lines 2040-2066 ---- */
float etbLookup(float pedalPct, float rpm)
{
  if (pedalPct < 0) pedalPct = 0;
  if (pedalPct > 100) pedalPct = 100;
  float pc = pedalPct * (ETB_COLS - 1) / 100.0f;
  uint8_t c0 = (uint8_t)pc;
  if (c0 >= ETB_COLS - 1) c0 = ETB_COLS - 2;
  uint8_t c1 = c0 + 1;
  float cf = pc - (float)c0;
  float rr = rpm / 8000.0f * (ETB_ROWS - 1);
  if (rr < 0) rr = 0;
  if (rr > ETB_ROWS - 1) rr = ETB_ROWS - 1;
  uint8_t r0 = (uint8_t)rr;
  if (r0 >= ETB_ROWS - 1) r0 = ETB_ROWS - 2;
  uint8_t r1 = r0 + 1;
  float rf = rr - (float)r0;
  float v = (1-cf)*(1-rf)*etbMap[r0][c0] + cf*(1-rf)*etbMap[r0][c1]
          + (1-cf)*rf*etbMap[r1][c0] + cf*rf*etbMap[r1][c1];
  return v;
}


/* Drive-by-wire idle control lives in ecu_idle.c */

void vvtMapsDefault(void)
{
  for (int r = 0; r < VVT_MAP_N; r++)
    for (int c = 0; c < VVT_MAP_N; c++) {
      /* mild mid-rpm advance, less at idle/high */
      int8_t v = 0;
      if (c >= 2 && c <= 5) v = (int8_t)(10 + (c - 2) * 3);
      if (r >= 5) v = (int8_t)(v / 2);
      vvtInMap[r][c] = v;
      vvtExMap[r][c] = (int8_t)(v / 2);
    }
}

float vvtLookup(const int8_t m[VVT_MAP_N][VVT_MAP_N], float rpm, float load)
{
  if (load < 0) load = 0;
  if (load > 1) load = 1;
  int ci = 0;
  while (ci < VVT_MAP_N - 2 && rpm > vvtRpmBins[ci + 1]) ci++;
  int ri = 0;
  while (ri < VVT_MAP_N - 2 && load > vvtLoadBins[ri + 1]) ri++;
  float rpm0 = vvtRpmBins[ci], rpm1 = vvtRpmBins[ci + 1];
  float ld0 = vvtLoadBins[ri], ld1 = vvtLoadBins[ri + 1];
  float cf = (rpm1 > rpm0) ? (rpm - rpm0) / (rpm1 - rpm0) : 0;
  float rf = (ld1 > ld0) ? (load - ld0) / (ld1 - ld0) : 0;
  if (cf < 0) cf = 0;
  if (cf > 1) cf = 1;
  if (rf < 0) rf = 0;
  if (rf > 1) rf = 1;
  float v = (1-cf)*(1-rf)*m[ri][ci] + cf*(1-rf)*m[ri][ci+1]
          + (1-cf)*rf*m[ri+1][ci] + cf*rf*m[ri+1][ci+1];
  return v;
}

void serviceVvtClosedLoop(void)
{
  if (!vvtClEnable || !syncLocked || rpmLive < 500) {
    ECU_SetVVT(0, 0);
    vvtInIntegral = vvtExIntegral = 0;
    return;
  }
  float load = engLoad;
  if (load < 0) load = engTps / 100.0f;
  float tgtIn = vvtLookup(vvtInMap, (float)rpmLive, load);
  float tgtEx = vvtLookup(vvtExMap, (float)rpmLive, load);

  /* Feedback: measured phase at last cam edges (0-720 scaled to ~0-50 useful) */
  float measIn = cam1PhaseDeg;
  if (measIn > 180.0f) measIn = 360.0f - measIn; /* fold */
  if (measIn < 0) measIn = 0;
  if (measIn > 60.0f) measIn = 60.0f;

  float measEx = cam2PhaseDeg;
  if (measEx > 180.0f) measEx = 360.0f - measEx;
  if (measEx < 0) measEx = 0;
  if (measEx > 60.0f) measEx = 60.0f;

  /* If cam not synced, open-loop duty from target (scaled) */
  float dutyIn, dutyEx;
  if (camSynced) {
    float err = tgtIn - measIn;
    vvtInIntegral += err * 0.01f;
    if (vvtInIntegral > 30) vvtInIntegral = 30;
    if (vvtInIntegral < -30) vvtInIntegral = -30;
    float d = err - vvtInPrevErr;
    vvtInPrevErr = err;
    dutyIn = VVT_KP * err + VVT_KI * vvtInIntegral + VVT_KD * d;
    /* bias: map target also opens solenoid proportionally */
    dutyIn += tgtIn * 1.2f;
  } else {
    dutyIn = tgtIn * 1.5f;
    vvtInIntegral = 0;
  }
  if (cam2Synced) {
    float err = tgtEx - measEx;
    vvtExIntegral += err * 0.01f;
    if (vvtExIntegral > 30) vvtExIntegral = 30;
    if (vvtExIntegral < -30) vvtExIntegral = -30;
    float d = err - vvtExPrevErr;
    vvtExPrevErr = err;
    dutyEx = VVT_KP * err + VVT_KI * vvtExIntegral + VVT_KD * d;
    dutyEx += tgtEx * 1.2f;
  } else {
    dutyEx = tgtEx * 1.5f;
    vvtExIntegral = 0;
  }
  if (dutyIn < 0) dutyIn = 0;
  if (dutyIn > 100) dutyIn = 100;
  if (dutyEx < 0) dutyEx = 0;
  if (dutyEx > 100) dutyEx = 100;
  ECU_SetVVT((uint8_t)dutyIn, (uint8_t)dutyEx);
}


/* ---- lines 2342-2545 ---- */
void serviceETB(void)
{
  if (!etbEnable || !gDbwEnable || htim1.Instance == NULL) {
    /* DBW off: still allow idle actuator via serviceIdleControl floor */
    if (!gDbwEnable && idleActive) {
      /* idle throttle uses same ETB pin as 2-wire / PWM idle */
    } else {
      __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
      return;
    }
  }

  /* Target = pedal map; idle control raises floor when foot off */
  float target = etbLookup(engPedal, (float)rpmLive);
  if (target < 0.0f) target = 0.0f;
  if (target > 100.0f) target = 100.0f;

  /* Idle / dashpot floor when pedal closed */
  if (idleActive || dashpotPct > 0.5f) {
    float floor = idleActive ? idleThrottle : ETB_IDLE_PCT;
    if (dashpotPct > floor) floor = dashpotPct;
    if (target < floor)
      target = floor;
  } else if (rpmLive > 200 && target < ETB_IDLE_PCT) {
    target = ETB_IDLE_PCT;
  }

  /* Explicit software override still wins */
  if (etbTargetPct >= 0.0f)
    target = etbTargetPct;

  float actual = engTps;
  if (actual < 0.0f) actual = 0.0f;
  if (actual > 100.0f) actual = 100.0f;

  float err = target - actual;

  /* Stop integral wind-up near rails */
  etbIntegral += err * 0.01f;  /* ~100 Hz loop assumed */
  if (etbIntegral > 40.0f) etbIntegral = 40.0f;
  if (etbIntegral < -40.0f) etbIntegral = -40.0f;
  if ((err > 0 && actual > 95.0f) || (err < 0 && actual < 2.0f))
    etbIntegral *= 0.9f;

  float deriv = err - etbPrevErr;
  etbPrevErr = err;

  float u = ETB_KP * err + ETB_KI * etbIntegral + ETB_KD * deriv;

  /* Deadband: avoid chatter at setpoint */
  if (err < 0.4f && err > -0.4f && actual > 1.0f) {
    u *= 0.3f;
  }

  /* H-bridge: DIR + PWM magnitude */
  float mag = u;
  if (mag < 0.0f) mag = -mag;
  if (mag > 100.0f) mag = 100.0f;
  /* Minimum drive to overcome stiction when error is significant */
  if (mag < 8.0f && (err > 1.0f || err < -1.0f))
    mag = 8.0f;
  if (err < 0.5f && err > -0.5f)
    mag = 0.0f;

  if (u >= 0.0f)
    HAL_GPIO_WritePin(ETB_DIR_GPIO_Port, ETB_DIR_Pin, GPIO_PIN_SET);   /* open */
  else
    HAL_GPIO_WritePin(ETB_DIR_GPIO_Port, ETB_DIR_Pin, GPIO_PIN_RESET); /* close */

  uint32_t ccr = (uint32_t)((mag * 1000.0f) / 100.0f);
  if (ccr > 1000U) ccr = 1000U;
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, ccr);
}

void ECU_SetThrottleTarget(float pct)
{
  /* pct < 0 → cancel override and follow pedal */
  if (pct < 0.0f) { etbTargetPct = -1.0f; return; }
  if (pct > 100.0f) pct = 100.0f;
  etbTargetPct = pct;
}

void ECU_EnableETB(uint8_t en)
{
  etbEnable = en ? 1 : 0;
  if (!etbEnable) {
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
    etbIntegral = 0.0f;
  }
}

void serviceOutputs(void) {
  static uint32_t lastRun = 0;
  uint32_t now = millis();

  /* Fuel pump: prime window, or running with RPM/sync, or run-on */
  if (fpPrimeUntilMs && now < fpPrimeUntilMs) {
    fpOn = 1;
  } else if (syncLocked || rpmLive > 50) {
    lastRun = now;
    fpOn = 1;
  } else if (now - lastRun > 3000) {
    fpOn = 0;
  }

  /* Fan with hysteresis: on at gFanOnC, off at gFanOffC */
  if (!gFanEnable) {
    fanOn = 0;
  } else if (fanOn) {
    fanOn = (engEct > gFanOffC) ? 1 : 0;
  } else {
    fanOn = (engEct >= gFanOnC) ? 1 : 0;
  }
  if (fpOn) ECU_FP_HI(); else ECU_FP_LO();
  if (fanOn) ECU_FAN_HI(); else ECU_FAN_LO();
}

/** One-shot injector prime at start of cranking */
void serviceStartPrime(void)
{
  uint32_t now = millis();

  if (rpmLive < 30) {
    if (lastZeroRpmMs == 0)
      lastZeroRpmMs = now;
    /* Re-arm prime after 2 s at zero RPM */
    if (injPrimeDone && (now - lastZeroRpmMs) > 2000u) {
      injPrimeDone = 0;
      injPrimeActive = 0;
    }
    if (injPrimeActive) {
      injPrimeActive = 0;
      for (uint8_t i = 1; i <= gCyl && i <= MAX_CYL; i++)
        ECU_INJ_LO(i);
    }
    return;
  }
  lastZeroRpmMs = 0;

  if (gInjPrimeEn && !injPrimeDone && rpmLive >= 80 && gInjPrimeMs > 0) {
    injPrimeDone = 1;
    injPrimeActive = 1;
    /* Less prime when warm (ECT > 60 °C → 30% of configured pulse) */
    uint16_t primeMs = gInjPrimeMs;
    if (sensEctEn && engEct > 60.0f)
      primeMs = (uint16_t)(gInjPrimeMs * 30 / 100);
    else if (sensEctEn && engEct > 40.0f)
      primeMs = (uint16_t)(gInjPrimeMs * 60 / 100);
    if (primeMs < 1 && gInjPrimeMs > 0) primeMs = 1;
    injPrimeEndMs = now + (uint32_t)primeMs;
    for (uint8_t i = 1; i <= gCyl && i <= MAX_CYL; i++)
      ECU_INJ_HI(i);
  }

  if (injPrimeActive) {
    if (now >= injPrimeEndMs) {
      injPrimeActive = 0;
      for (uint8_t i = 1; i <= gCyl && i <= MAX_CYL; i++)
        ECU_INJ_LO(i);
    } else {
      for (uint8_t i = 1; i <= gCyl && i <= MAX_CYL; i++)
        ECU_INJ_HI(i);
    }
  }
}

/**
 * Set dual-VVT solenoid PWM duty (0-100%).
 * VVT1 intake  → TIM1_CH3  PA10
 * VVT2 exhaust → TIM1_CH2N PB14  (CCR2 drives complementary)
 * Requires MX_TIM1_Init + HAL_TIM_PWM_Start / HAL_TIMEx_PWMN_Start.
 */
void ECU_SetVVT(uint8_t intake_pct, uint8_t exhaust_pct)
{
  if (intake_pct > 100U)  intake_pct = 100U;
  if (exhaust_pct > 100U) exhaust_pct = 100U;

  vvt1Duty = intake_pct;
  vvt2Duty = exhaust_pct;

  /* ARR = 999 → 1000 counts full scale */
  uint32_t c1 = ((uint32_t)intake_pct  * 1000U) / 100U;
  uint32_t c2 = ((uint32_t)exhaust_pct * 1000U) / 100U;
  if (c1 > 1000U) c1 = 1000U;
  if (c2 > 1000U) c2 = 1000U;

  if (htim1.Instance != NULL) {
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, c1);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, c2);
  }
}

/* ── ADC ────────────────────────────────────────────────────── */
/* readAdc() is only defined in ecu_adc.c */

