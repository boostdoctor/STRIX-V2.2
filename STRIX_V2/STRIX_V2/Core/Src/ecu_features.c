/* ecu_features.c — auto-split from ecu_app.c */
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

/* ---- lines 1517-1669 ---- */
void dtcSet(uint16_t code)
{
  for (uint8_t i = 0; i < dtcCount; i++) {
    if (dtcList[i].code == code) {
      dtcList[i].active = 1;
      return;
    }
  }
  if (dtcCount >= DTC_MAX_ACTIVE) return;
  dtcList[dtcCount].code = code;
  dtcList[dtcCount].active = 1;
  dtcList[dtcCount].setMs = millis();
  dtcCount++;
}

void dtcClearCode(uint16_t code)
{
  for (uint8_t i = 0; i < dtcCount; i++) {
    if (dtcList[i].code == code)
      dtcList[i].active = 0;
  }
}

void serviceDtcSanity(void)
{
  uint32_t now = millis();
  if (now - lastDtcEvalMs < 100) return;
  lastDtcEvalMs = now;

  /* MAP */
  if (engMap < 5.0f && syncLocked && rpmLive > 600)
    dtcSet(DTC_MAP_LOW);
  else
    dtcClearCode(DTC_MAP_LOW);
  if (engMap > 320.0f)
    dtcSet(DTC_MAP_HIGH);
  else
    dtcClearCode(DTC_MAP_HIGH);

  /* TPS cal span */
  if (tpsCalValid && (tpsOpenAdc <= tpsClosedAdc + 50))
    dtcSet(DTC_TPS_RANGE);
  else
    dtcClearCode(DTC_TPS_RANGE);

  /* ECT / IAT ADC rails (open sensor ~0 or ~4095) */
  if (adcEct < 20 || adcEct > 4070)
    dtcSet(DTC_ECT_OPEN);
  else
    dtcClearCode(DTC_ECT_OPEN);
  if (engEct > 130.0f)
    dtcSet(DTC_ECT_HIGH);
  else
    dtcClearCode(DTC_ECT_HIGH);
  if (adcIat < 20 || adcIat > 4070)
    dtcSet(DTC_IAT_OPEN);
  else
    dtcClearCode(DTC_IAT_OPEN);

  /* Battery */
  if (engBat > 0.5f && engBat < 10.5f && rpmLive > 500)
    dtcSet(DTC_BAT_LOW);
  else
    dtcClearCode(DTC_BAT_LOW);
  if (engBat > 16.5f)
    dtcSet(DTC_BAT_HIGH);
  else
    dtcClearCode(DTC_BAT_HIGH);

  /* Sync */
  if (syncLosses > 20)
    dtcSet(DTC_SYNC_LOSS);
  if (syncLocked && gCamMode != 0 && !camSynced && rpmLive > 800)
    dtcSet(DTC_CAM_LOSS);
  else
    dtcClearCode(DTC_CAM_LOSS);

  /* O2 stuck (NB) or AFR range (WB) */
  if (o2SensorMode == O2_MODE_NB && o2ClEnable && rpmLive > 1000) {
    if (o2StuckLast < 0.0f) o2StuckLast = o2Filt;
    if (fabsf(o2Filt - o2StuckLast) < 0.02f)
      o2StuckSameMs += 100;
    else {
      o2StuckSameMs = 0;
      o2StuckLast = o2Filt;
    }
    if (o2StuckSameMs > 15000)
      dtcSet(DTC_O2_STUCK);
    else
      dtcClearCode(DTC_O2_STUCK);
  }
  if (o2SensorMode == O2_MODE_WB) {
    if (engAfr < 8.0f || engAfr > 22.0f)
      dtcSet(DTC_AFR_RANGE);
    else
      dtcClearCode(DTC_AFR_RANGE);
  }
}

void ECU_Features_Init(void)
{
  for (uint8_t i = 0; i <= MAX_CYL; i++)
    cylTrimPct[i] = 0.0f;
  dtcCount = 0;
  engAfr = 14.7f;
  o2SensorMode = O2_MODE_NB;
}

void ECU_Features_Service(void)
{
  serviceDtcSanity();
}


void ECU_Dtc_Clear(void)
{
  dtcCount = 0;
  for (uint8_t i = 0; i < DTC_MAX_ACTIVE; i++) {
    dtcList[i].code = 0;
    dtcList[i].active = 0;
  }
  o2StuckSameMs = 0;
}

uint8_t ECU_Dtc_Count(void)
{
  uint8_t n = 0;
  for (uint8_t i = 0; i < dtcCount; i++)
    if (dtcList[i].active) n++;
  return n;
}

uint16_t ECU_Dtc_Get(uint8_t index)
{
  uint8_t n = 0;
  for (uint8_t i = 0; i < dtcCount; i++) {
    if (!dtcList[i].active) continue;
    if (n == index) return dtcList[i].code;
    n++;
  }
  return DTC_NONE;
}

float ECU_GetAfr(void) { return engAfr; }
uint8_t ECU_GetO2Mode(void) { return o2SensorMode; }

float ECU_GetCylTrim(uint8_t cyl)
{
  if (cyl < 1 || cyl > MAX_CYL) return 0.0f;
  return cylTrimPct[cyl];
}

void ECU_SetCylTrim(uint8_t cyl, float pct)
{
  if (cyl < 1 || cyl > MAX_CYL) return;
  if (pct > 25.0f) pct = 25.0f;
  if (pct < -25.0f) pct = -25.0f;
  cylTrimPct[cyl] = pct;
}


/* ---- lines 1814-2040 ---- */
void serviceKnockGoertzel(void)
{
  if (!knkEnable || !syncLocked || rpmLive < 1200) {
    /* restore any residual retard slowly when inactive */
    if (knockRetardDeg > 0.0f) {
      knockRetardDeg -= knkRestoreDps * 0.01f;
      if (knockRetardDeg < 0.0f) knockRetardDeg = 0.0f;
    }
    knkCollecting = 0;
    knkIdx = 0;
    return;
  }

  /* Approximate ATDC window using crankDeg (0-360 or 0-720) */
  float deg = crankDeg;
  while (deg >= 360.0f) deg -= 360.0f;
  /* Window ~15-50° ATDC */
  uint8_t inWin = (deg >= 15.0f && deg <= 50.0f) ? 1 : 0;

  if (inWin) {
    if (!knkCollecting) {
      knkCollecting = 1;
      knkIdx = 0;
    }
    if (knkIdx < KNK_WIN_N) {
      /* Burst-sample knock ADC to fill window (busy sample) */
      while (knkIdx < KNK_WIN_N) {
        uint16_t raw = readAdc(ECU_ADC_CH_KNOCK);
        knkBuf[knkIdx++] = (float)raw;
      }
    }
    if (knkIdx >= KNK_WIN_N && knkCollecting) {
      knkIntensity = Goertzel_KnockIntensity(
          knkBuf, KNK_WIN_N, KNK_FS_HZ, KNK_F1_HZ, KNK_F2_HZ);
      engKnock = knkIntensity;

      /* Threshold + max retard from RPM logic tables (or single values) */
      float thr;
      float maxR;
      if (knkUseTable) {
        thr = msRetardLookup(knkThrTbl, (float)rpmLive);
        maxR = msRetardLookup(knkMaxTbl, (float)rpmLive);
      } else {
        thr = knkThreshold * (1.0f + (float)rpmLive / 10000.0f);
        maxR = knkMaxRetard;
      }

      if (knkIntensity > thr) {
        knockRetardDeg += knkStepDeg;
        if (knockRetardDeg > maxR) knockRetardDeg = maxR;
      } else {
        knockRetardDeg -= knkRestoreDps * 0.02f;
        if (knockRetardDeg < 0.0f) knockRetardDeg = 0.0f;
      }
      knkCollecting = 0;
      knkIdx = 0;
    }
  } else {
    knkCollecting = 0;
    knkIdx = 0;
  }
}

void serviceMotorsport(void)
{
  clutchPressed = readClutch();

  /* Launch control: clutch in + high TPS → hold RPM + optional boost target */
  launchActive = 0;
  static uint8_t lcWasCutting = 0;
  if (launchEnable && clutchPressed && engTps >= launchTpsMin && syncLocked) {
    launchActive = 1;
    if ((float)rpmLive > launchRpm) {
      rpmCutActive = 1;
      lcWasCutting = 1;
    } else if (lcWasCutting && (float)rpmLive < launchRpm - 150.0f) {
      rpmCutActive = 0;
      lcWasCutting = 0;
    }
    if (launchBoostKpa > 5.0f && boostTargetKpa < launchBoostKpa)
      boostTargetKpa = launchBoostKpa;
  } else {
    if (lcWasCutting) {
      rpmCutActive = 0;
      lcWasCutting = 0;
    }
  }

  /* Flat-foot shift: clutch + high TPS while moving */
  ffsActive = 0;
  if (ffsEnable && clutchPressed && engTps >= ffsTpsMin && rpmLive > 2000) {
    ffsActive = 1;
  }

  /* Anti-lag: during LC or FFS, limited by max duration + cooldown */
  {
    uint32_t now = HAL_GetTick();
    uint8_t want = (alsEnable && (launchActive || ffsActive) && engTps >= 40.0f) ? 1 : 0;

    if (now < alsBlockUntilMs) {
      /* still in cooldown after max-duration timeout */
      alsActive = 0;
      alsTimedOut = 1;
    } else if (!want) {
      alsActive = 0;
      alsStartMs = 0;
      alsTimedOut = 0;
    } else {
      if (alsStartMs == 0)
        alsStartMs = now;
      float elapsed = (float)(now - alsStartMs) * 0.001f;
      if (alsMaxSec > 0.1f && elapsed >= alsMaxSec) {
        /* hit max duration - drop ALS and start cooldown */
        alsActive = 0;
        alsTimedOut = 1;
        alsStartMs = 0;
        alsBlockUntilMs = now + (uint32_t)(alsCooldownSec * 1000.0f);
      } else {
        alsActive = 1;
        alsTimedOut = 0;
        if (alsExVvt)
          vvt2Duty = 80;
      }
    }
  }
}

void serviceBoost(void)
{
  if (!boostEnable || htim4.Instance == NULL) {
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 0);
    return;
  }

  float map = engMap;
  if (map > BOOST_MAX_KPA) {
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 0);
    boostIntegral = 0.0f;
    return;
  }

  /* Bilinear sample of 8×8 map (rows=TPS, cols=RPM) */
  float mapVal = 0.0f;
  if (bstMapEnable) {
    float rpm = (float)rpmLive, tps = engTps;
    int ci = 0, ri = 0;
    while (ci < BST_N - 2 && rpm > bstRpm[ci + 1]) ci++;
    while (ri < BST_N - 2 && tps > bstTps[ri + 1]) ri++;
    float r0 = bstRpm[ci], r1 = bstRpm[ci + 1];
    float t0 = bstTps[ri], t1 = bstTps[ri + 1];
    float cf = (r1 > r0) ? (rpm - r0) / (r1 - r0) : 0;
    float rf = (t1 > t0) ? (tps - t0) / (t1 - t0) : 0;
    if (cf < 0) cf = 0;
  if (cf > 1) cf = 1;
    if (rf < 0) rf = 0;
  if (rf > 1) rf = 1;
    mapVal = (1-cf)*(1-rf)*bstMap[ri][ci] + cf*(1-rf)*bstMap[ri][ci+1]
           + (1-cf)*rf*bstMap[ri+1][ci] + cf*rf*bstMap[ri+1][ci+1];
  }

  if (engTps < 15.0f || rpmLive < 1500) {
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 0);
    boostIntegral *= 0.95f;
    return;
  }

  float duty = 0.0f;

  if (bstOpenLoop) {
    /* Open-loop: map cells are solenoid duty % (0-100) */
    duty = mapVal;
    if (boostTargetKpa > 0.5f && boostTargetKpa <= 100.0f)
      duty = boostTargetKpa; /* optional single % override via SET:BOOST */
  } else {
    /* Closed-loop: map cells are gauge kPa targets */
    float tgtGauge = bstMapEnable ? mapVal : boostTargetKpa;
    if (boostTargetKpa > tgtGauge)
      tgtGauge = boostTargetKpa;
    if (tgtGauge < 5.0f) {
      __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 0);
      boostIntegral *= 0.95f;
      return;
    }
    float atm = 100.0f;
    float targetAbs = atm + tgtGauge;
    float err = targetAbs - map;
    boostIntegral += err * 0.01f;
    if (boostIntegral > 50.0f) boostIntegral = 50.0f;
    if (boostIntegral < -50.0f) boostIntegral = -50.0f;
    float deriv = err - boostPrevErr;
    boostPrevErr = err;
    float u = BOOST_KP * err + BOOST_KI * boostIntegral + BOOST_KD * deriv;
    duty = u;
    if (!boostDutyRaisesBoost)
      duty = -duty;
  }

  if (duty < BOOST_MIN_DUTY) duty = BOOST_MIN_DUTY;
  if (duty > BOOST_MAX_DUTY) duty = BOOST_MAX_DUTY;
  if (duty < 0) duty = 0;
  if (duty > 100) duty = 100;

  uint32_t ccr = (uint32_t)((duty * 1000.0f) / 100.0f);
  if (ccr > 1000U) ccr = 1000U;
  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, ccr);
}

void ECU_SetBoostTarget(float gauge_kpa)
{
  if (gauge_kpa < 0.0f) gauge_kpa = 0.0f;
  if (gauge_kpa > 300.0f) gauge_kpa = 300.0f;
  boostTargetKpa = gauge_kpa;
  if (gauge_kpa < 1.0f)
    boostIntegral = 0.0f;
}

void ECU_EnableBoost(uint8_t en)
{
  boostEnable = en ? 1 : 0;
  if (!boostEnable) {
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 0);
    boostIntegral = 0.0f;
  }
}


