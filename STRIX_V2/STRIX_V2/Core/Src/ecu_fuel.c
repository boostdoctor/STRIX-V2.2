/* ecu_fuel.c — auto-split from ecu_app.c */
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

/* ---- lines 1302-1517 ---- */
void serviceInjection(void) {
  uint32_t now = micros();

  /* Flood clear: TPS open while cranking → cut all injectors */
  floodClearActive = 0;
  if (floodClearEnable && rpmLive > 0 && rpmLive < (int)crankAdvRpm
      && engTps >= floodClearTps) {
    floodClearActive = 1;
  }

  if ((rpmCutActive && gRpmCutMode == 0) || dfcoActive || floodClearActive) {
    for (uint8_t i = 1; i <= MAX_CYL; i++) {
      ECU_INJ_LO(i); injOn[i] = 0; injReq[i] = 0;
    }
    return;
  }
  uint16_t pw = injPwUs;
  if (rpmLive > 0 && rpmLive < 200)
    pw = 3000; /* cranking standard 3 ms */
  else if (pw < 1000)
    pw = 1000;
  if (pw > 20000) pw = 20000;

  for (uint8_t i = 1; i <= MAX_CYL; i++) {
    if (injOn[i] && (int32_t)(now - injEndUs[i]) >= 0) {
      ECU_INJ_LO(i);
      injOn[i] = 0;
      injFiredCyc[i] = 1;
      injReq[i] = 0;
    }
  }

  if (!syncLocked || toothPeriodUs < 40) {
    for (uint8_t i = 1; i <= MAX_CYL; i++) {
      if (!injOn[i]) {
        ECU_INJ_LO(i);
        injReq[i] = 0;
      }
    }
    return;
  }

  /* Hysteresis so we don't batch-fire and angle-fire on the same rev. */
  static uint8_t crankingInj;
  if (rpmLive < 180)
    crankingInj = 1;
  else if (rpmLive > 240)
    crankingInj = 0;

  static uint16_t injStamp[MAX_CYL + 1];

  if (crankingInj) {
    uint8_t n = gCyl;
    if (n > MAX_CYL) n = MAX_CYL;
    if (n < 1) n = 1;
    if (toothIndex <= 1) {
      for (uint8_t i = 1; i <= n; i++) {
        if (injOn[i])
          continue;
        if (injStamp[i] == crankRevId)
          continue; /* already fired this gap */
        if (injDisableMask & (1u << (i - 1)))
          continue;
        ECU_INJ_HI(i);
        injOn[i] = 1;
        injFiredCyc[i] = 1;
        injStamp[i] = crankRevId;
        injEndUs[i] = now + pw;
      }
    }
    return;
  }

  float usPerRev = (float)toothPeriodUs * (float)gTeeth;
  if (usPerRev < 400.0f) return;

  uint8_t seq = injSequentialActive();
  float cycle = seq ? 720.0f : 360.0f;
  float deg = crankDeg;
  float band = 360.0f / (float)((gTeeth > 0) ? gTeeth : 36);
  if (band < 4.0f) band = 4.0f;
  if (band > 20.0f) band = 20.0f;
  if (rpmLive < 1500 && band < 25.0f) band = 25.0f;
  {
    uint32_t T = toothPeriodFilt ? toothPeriodFilt : toothPeriodUs;
    uint32_t age = (lastToothUs && now > lastToothUs) ? (now - lastToothUs) : 0;
    if (T >= 80u && age < T * 3u) {
      float extra = 360.0f * ((float)age / ((float)T * (float)((gTeeth > 1) ? gTeeth : 36)));
      if (extra > 0.0f && extra < 30.0f)
        deg = wrapAngle(deg + extra, cycle);
    }
  }
  float eoiOfs = gEoiBtdc;
  if (eoiOfs < 10.0f) eoiOfs = 10.0f;
  if (eoiOfs > 540.0f) eoiOfs = 540.0f;
  float trig = (float)gTrigAngle;
  float degPerUs = 360.0f / usPerRev;

  uint8_t n = gCyl;
  if (n > MAX_CYL) n = MAX_CYL;
  if (!seq && n > 4) n = 4;

  for (uint8_t i = 1; i <= n; i++) {
    float tdc;
    if (seq)
      tdc = tdcDeg(i);
    else if (i == 1 || i == 4)
      tdc = 0.0f;
    else if (i == 2 || i == 3)
      tdc = 180.0f;
    else
      continue;

    /* Same frame as spark: gap 0° + trigger = compression TDC. */
    float trueTdc = wrapAngle(tdc + trig, cycle);
    float eoi = wrapAngle(trueTdc - eoiOfs, cycle);
    float pwDeg = (float)pw * degPerUs;
    if (pwDeg < 1.0f) pwDeg = 1.0f;
    float soi = wrapAngle(eoi - pwDeg, cycle);

    /* One pulse per missing-tooth gap. Do not re-arm mid-window. */
    (void)injReq[i];
    injReq[i] = 0;
    if (injStamp[i] == crankRevId)
      continue;

    if (!injOn[i] && !injFiredCyc[i]) {
      float cap = band * 1.5f;
      if (cap < 8.0f) cap = 8.0f;
      if (cap > 18.0f) cap = 18.0f;
      if (angleActive(deg, soi, wrapAngle(soi + cap, cycle), cycle)) {
        /* Diagnostic injector kill: bit0 = cyl1 */
        if (injDisableMask & (1u << (i - 1))) {
          ECU_INJ_LO(i);
          injOn[i] = 0;
          injFiredCyc[i] = 1;
          continue;
        }
        uint16_t pwc = pw;
        float tr = 1.0f + cylTrimPct[i] * 0.01f;
        if (tr < 0.75f) tr = 0.75f;
        if (tr > 1.25f) tr = 1.25f;
        /* Idle 5×5 fuel correction near idle */
        if (idleActive || (rpmLive > 0 && rpmLive < 1400 && engTps < 5.0f)) {
          tr *= 1.0f + idleFuelLookup(engEct, (float)rpmLive) * 0.01f;
        }
        pwc = (uint16_t)((float)pw * tr);
        if (rpmLive < 200) { if (pwc < 3000) pwc = 3000; }
        else if (pwc < 1000) pwc = 1000;
        if (pwc > 20000) pwc = 20000;
        ECU_INJ_HI(i);
        injOn[i] = 1;
        injFiredCyc[i] = 1;
        injStamp[i] = crankRevId;
        injEndUs[i] = now + pwc;
      }
    }
  }
}


/* Globals for boost/O2/ALS: ecu_runtime.c — ECU_SetCylTrim: ecu_features.c */

void serviceO2ClosedLoop(void)
{
  uint32_t now = millis();
  if (o2LastMs == 0) o2LastMs = now;
  float dt = (float)(now - o2LastMs) * 0.001f;
  if (dt < 0.001f) dt = 0.001f;
  if (dt > 0.05f) dt = 0.05f;
  o2LastMs = now;

  /* Filter O2 voltage (controller 0–3.3 V scaled) */
  o2Filt = o2Filt * 0.82f + engO2 * 0.18f;

  o2ClActive = 0;
  if (!o2ClEnable || o2SensorMode == O2_MODE_OFF || !sensO2En) {
    stftPct *= (1.0f - 0.4f * dt); /* bleed toward 0 when disabled */
    return;
  }

  /* Freeze learning during transient events */
  if (aseActive || dfcoActive || alsActive || rpmCutActive || floodClearActive
      || launchActive || launchDecayActive) {
    /* keep last STFT but do not learn */
    return;
  }
  if (!syncLocked || engBat < 11.0f) {
    stftPct *= (1.0f - 0.3f * dt);
    return;
  }

  if (o2SensorMode == O2_MODE_WB) {
    /* ── Wideband closed loop ──────────────────────────────────
     * Error in AFR: lean (engAfr > tgt) → add fuel (STFT +)
     *               rich (engAfr < tgt) → cut fuel (STFT −)
     * Proportional rate scales with |err|; deadband ±0.05 AFR.
     * Authority: STFT_MAX. LTFT learns only in steady cruise.
     */
    if (rpmLive < 500 || rpmLive > 7500) {
      stftPct *= (1.0f - 0.25f * dt);
      return;
    }
    if (engEct < 50.0f) return;          /* cold: ASE/WUE own fuel */
    if (engTps > 92.0f && rpmLive > 4500) {
      /* WOT high-RPM: hold STFT, no further learn (safety) */
      o2ClActive = 1;
      return;
    }

    float loadAxis = (engMap > 5.0f) ? engMap : engLoad;
    float tgt = afrTargetLookup(loadAxis, (float)rpmLive);
    if (tgt < 10.0f) tgt = 10.0f;
    if (tgt > 16.5f) tgt = 16.5f;

    float err = engAfr - tgt;            /* + lean, − rich */
    const float dead = 0.05f;
    o2ClActive = 1;

    if (err > dead) {
      /* lean → add fuel; gain rises with error up to ~2 AFR */
      float mag = err - dead;
      if (mag > 2.0f) mag = 2.0f;
      float rate = STFT_STEP * (0.6f + mag * 1.4f) * (dt * 100.0f);
      stftPct += rate;
    } else if (err < -dead) {
      float mag = -err - dead;
      if (mag > 2.0f) mag = 2.0f;
      float rate = STFT_STEP * (0.6f + mag * 1.4f) * (dt * 100.0f);
      stftPct -= rate;
    } else {
      /* in band — mild decay toward 0 so LTFT can absorb */
      stftPct *= (1.0f - 0.15f * dt);
    }

    if (stftPct >  STFT_MAX) stftPct =  STFT_MAX;
    if (stftPct < -STFT_MAX) stftPct = -STFT_MAX;

    /* LTFT: only when nearly on-target, mid load, warm, no boost */
    uint8_t steady = (fabsf(err) < 0.15f)
                  && (engTps > 8.0f && engTps < 55.0f)
                  && (rpmLive > 1200 && rpmLive < 4000)
                  && (engEct >= 70.0f)
                  && (boostTargetKpa < 5.0f);
    if (steady) {
      ltftPct += (stftPct - ltftPct) * LTFT_RATE * (dt * 40.0f);
      if (ltftPct >  LTFT_MAX) ltftPct =  LTFT_MAX;
      if (ltftPct < -LTFT_MAX) ltftPct = -LTFT_MAX;
    }
    return;
  }

  /* ── Narrowband closed loop ──────────────────────────────── */
  if (rpmLive < 800 || rpmLive > 4000) return;
  if (engEct < 60.0f) return;
  if (engTps > 60.0f) return;
  if (boostTargetKpa > 10.0f) return;

  o2ClActive = 1;
  if (o2Filt > O2_RICH_V) {
    o2RichMs += (uint32_t)(dt * 1000.0f);
    o2LeanMs = 0;
  } else if (o2Filt < O2_LEAN_V) {
    o2LeanMs += (uint32_t)(dt * 1000.0f);
    o2RichMs = 0;
  } else {
    o2RichMs = 0;
    o2LeanMs = 0;
  }
  if (o2RichMs > 20)
    stftPct -= STFT_STEP * (dt * 100.0f);
  else if (o2LeanMs > 20)
    stftPct += STFT_STEP * (dt * 100.0f);
  else
    stftPct *= (1.0f - 0.5f * dt);

  if (stftPct >  STFT_MAX) stftPct =  STFT_MAX;
  if (stftPct < -STFT_MAX) stftPct = -STFT_MAX;
  ltftPct += (stftPct - ltftPct) * LTFT_RATE * (dt * 50.0f);
  if (ltftPct >  LTFT_MAX) ltftPct =  LTFT_MAX;
  if (ltftPct < -LTFT_MAX) ltftPct = -LTFT_MAX;
}

/**
 * Fuel trim integration:
 *   base × LTFT (always) × STFT (only when o2ClActive)
 * STFT learn/apply frozen during ASE, DFCO, ALS so event fuel does not corrupt trims.
 */
float fuelTrimMul(void)
{
  if (!sensO2En) return 1.0f;
  float pct = ltftPct;           /* always applied */
  if (o2ClActive)
    pct += stftPct;              /* STFT while CL active (NB or WB) */
  if (pct >  STFT_MAX + LTFT_MAX) pct =  STFT_MAX + LTFT_MAX;
  if (pct < -(STFT_MAX + LTFT_MAX)) pct = -(STFT_MAX + LTFT_MAX);
  return 1.0f + pct * 0.01f;
}

/* legacy name */
float o2FuelMul(void) { return fuelTrimMul(); }

float totalTrimPct(void)
{
  float pct = ltftPct;
  if (o2ClActive) pct += stftPct;
  return pct;
}

void ECU_EnableO2CL(uint8_t en)
{
  o2ClEnable = en ? 1 : 0;
  if (!o2ClEnable) {
    o2ClActive = 0;
    stftPct = 0.0f;
  }
}

void ECU_SetLTFT(float pct)
{
  if (pct >  LTFT_MAX) pct =  LTFT_MAX;
  if (pct < -LTFT_MAX) pct = -LTFT_MAX;
  ltftPct = pct;
}

void ECU_ResetFuelTrim(void)
{
  stftPct = 0.0f;
  ltftPct = 0.0f;
}

uint8_t readClutch(void)
{
  /* PB13 clutch switch - active low with pull-up (pressed = 0) */
  return (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_13) == GPIO_PIN_RESET) ? 1 : 0;
}


uint16_t readAdc(uint32_t channel); /* fwd */


float msRetardLookup(const float *tbl, float rpm)
{
  if (rpm <= msRpmBins[0]) return tbl[0];
  if (rpm >= msRpmBins[MS_RPM_N - 1]) return tbl[MS_RPM_N - 1];
  int i = 0;
  while (i < MS_RPM_N - 2 && rpm > msRpmBins[i + 1]) i++;
  float r0 = msRpmBins[i], r1 = msRpmBins[i + 1];
  float f = (r1 > r0) ? (rpm - r0) / (r1 - r0) : 0.0f;
  if (f < 0) f = 0;
  if (f > 1) f = 1;
  return tbl[i] * (1.0f - f) + tbl[i + 1] * f;
}


/* ---- lines 2218-2249 ---- */
void serviceDfco(void)
{
  uint8_t want = 0;
  /* Closed-throttle coast: high RPM, warm, foot off */
  if (dfcoEnable && syncLocked && rpmLive >= dfcoEnterRpm
      && engEct >= dfcoMinEct
      && engTps <= dfcoMaxTps && engPedal <= dfcoMaxTps
      && !rpmCutActive) {
    want = 1;
  }

  uint32_t now = millis();
  if (want) {
    if (dfcoEnterMs == 0)
      dfcoEnterMs = now;
    if ((now - dfcoEnterMs) >= dfcoDelayMs)
      dfcoActive = 1;
  } else {
    dfcoEnterMs = 0;
    /* Exit hysteresis: stay cut until RPM falls or throttle opens */
    if (dfcoActive) {
      if (rpmLive <= dfcoExitRpm || engTps > dfcoMaxTps + 2.0f
          || engPedal > dfcoMaxTps + 2.0f || !syncLocked)
        dfcoActive = 0;
    }
  }
  /* Always exit if throttle opens */
  if (engTps > dfcoMaxTps + 2.0f || engPedal > dfcoMaxTps + 2.0f)
    dfcoActive = 0;
}


/* ---- lines 3778-3925 ---- */
float coldStartEnrichMul(void)
{
  if (!sensEctEn) return 1.0f;
  float ect = engEct;
  if (ect <= cseTemp[0]) return 1.0f + csePct[0] * 0.01f;
  if (ect >= cseTemp[CSE_N - 1]) return 1.0f + csePct[CSE_N - 1] * 0.01f;
  int i = 0;
  while (i < CSE_N - 2 && ect > cseTemp[i + 1]) i++;
  float t0 = cseTemp[i], t1 = cseTemp[i + 1];
  float f = (t1 > t0) ? (ect - t0) / (t1 - t0) : 0;
  if (f < 0) f = 0;
  if (f > 1) f = 1;
  float pct = csePct[i] * (1 - f) + csePct[i + 1] * f;
  return 1.0f + pct * 0.01f;
}

/* After-start: full aseInitialPct at t=0, linear decay to 0 over aseDecaySec */
/** ALS enrichment: 1 + fuel% when anti-lag active */

/** Acceleration enrichment: 1 + aePctLive% based on TPS rate tip-in */
float accelEnrichMul(void)
{
  if (!aeEnable || aePctLive < 0.05f)
    return 1.0f;
  float pct = aePctLive;
  if (pct > aeMaxPct) pct = aeMaxPct;
  return 1.0f + pct * 0.01f;
}

void serviceAccelEnrich(void)
{
  uint32_t now = HAL_GetTick();
  if (aeLastMs == 0) {
    aeLastMs = now;
    aePrevTps = engTps;
    return;
  }
  float dt = (float)(now - aeLastMs) * 0.001f;
  aeLastMs = now;
  if (dt < 0.001f) dt = 0.001f;
  if (dt > 0.05f) dt = 0.05f;

  float tps = engTps;
  if (tps < 0.0f) tps = 0.0f;
  if (tps > 100.0f) tps = 100.0f;
  float tpsDot = (tps - aePrevTps) / dt; /* %/s */
  aePrevTps = tps;

  if (!aeEnable) {
    aePctLive = 0.0f;
    return;
  }
  static float aePeakPct = 0.0f;
  /* Only enrich on positive tip-in above threshold */
  if (tpsDot > aeTpsDotThresh) {
    float excess = tpsDot - aeTpsDotThresh;
    float pct = excess * aeGain;
    if (pct > aeMaxPct) pct = aeMaxPct;
    if (pct > aePeakPct) {
      aePeakPct = pct;
      aePctLive = pct;
      aeDecayUntilMs = now + (uint32_t)aeDecayMs;
    }
  }
  /* Linear decay from peak to 0 over aeDecayMs */
  if (aePeakPct > 0.0f) {
    if (now >= aeDecayUntilMs) {
      aePctLive = 0.0f;
      aePeakPct = 0.0f;
    } else {
      float remain = (float)(aeDecayUntilMs - now) / (float)(aeDecayMs > 0 ? aeDecayMs : 1u);
      if (remain < 0.0f) remain = 0.0f;
      if (remain > 1.0f) remain = 1.0f;
      aePctLive = aePeakPct * remain;
      if (aePctLive < 0.05f) {
        aePctLive = 0.0f;
        aePeakPct = 0.0f;
      }
    }
  }
}

float flexFuelMul(void)
{
  if (!gFlexEnable)
    return 1.0f;
  float pct = (engEthanol / 10.0f) * gFlexFuelPctPer10;
  if (pct < 0.0f) pct = 0.0f;
  if (pct > 80.0f) pct = 80.0f;
  return 1.0f + pct * 0.01f;
}

/*
 * PA6 flex fuel sensor: frequency input 40 Hz (E0) … 160 Hz (E100).
 * Software edge detection (rising) — no extra timer; accurate at 40–160 Hz.
 */
#ifndef FLEX_HZ_E0
#define FLEX_HZ_E0    40.0f
#endif
#ifndef FLEX_HZ_E100
#define FLEX_HZ_E100  160.0f
#endif

float engFlexHz = 0.0f; /* latest measured flex frequency */

void ECU_Flex_Init(void)
{
  __HAL_RCC_GPIOA_CLK_ENABLE();
  GPIO_InitTypeDef g = {0};
  g.Pin = FLEX_Pin;
  g.Mode = GPIO_MODE_INPUT;
  g.Pull = GPIO_PULLUP; /* open-collector friendly; drive low from sensor */
  HAL_GPIO_Init(FLEX_GPIO_Port, &g);
}

void serviceFlexFuel(void)
{
  static uint8_t lastLvl = 1;
  static uint32_t lastEdgeUs = 0;
  static float hzFilt = 0.0f;

  /* Always sample edges so Hz is available; map to ethanol when enabled */
  uint8_t lvl = (HAL_GPIO_ReadPin(FLEX_GPIO_Port, FLEX_Pin) == GPIO_PIN_SET) ? 1u : 0u;
  if (lvl && !lastLvl) {
    uint32_t now = micros();
    if (lastEdgeUs != 0u) {
      uint32_t dt = now - lastEdgeUs;
      /* 40–160 Hz → period 6250–25000 µs; allow 20–500 Hz window */
      if (dt >= 2000u && dt <= 50000u) {
        float hz = 1000000.0f / (float)dt;
        if (hzFilt < 1.0f)
          hzFilt = hz;
        else
          hzFilt = hzFilt * 0.75f + hz * 0.25f;
        engFlexHz = hzFilt;
      }
    }
    lastEdgeUs = now;
  }
  lastLvl = lvl;

  /* Timeout: no edge for 100 ms → treat as 0 Hz */
  if (lastEdgeUs != 0u && (micros() - lastEdgeUs) > 100000u) {
    hzFilt = 0.0f;
    engFlexHz = 0.0f;
    lastEdgeUs = 0u;
  }

  if (!gFlexEnable)
    return;

  float span = FLEX_HZ_E100 - FLEX_HZ_E0;
  if (span < 1.0f)
    span = 1.0f;
  float e = (engFlexHz - FLEX_HZ_E0) * (100.0f / span);
  if (e < 0.0f) e = 0.0f;
  if (e > 100.0f) e = 100.0f;
  engEthanol = e;
}

float alsFuelMul(void)
{
  if (!alsActive)
    return 1.0f;
  float pct = alsFuelUseTable
    ? msRetardLookup(alsFuelTbl, (float)rpmLive)
    : alsFuelPct;
  if (pct < 0.0f) pct = 0.0f;
  if (pct > 100.0f) pct = 100.0f;
  return 1.0f + pct * 0.01f;
}

float afterStartMul(void)
{
  if (!sensEctEn) return 1.0f;
  if (!aseActive)
    return 1.0f;
  uint32_t now = HAL_GetTick();
  float elapsed = (float)(now - aseStartMs) * 0.001f;
  if (elapsed >= aseDecaySec || aseDecaySec <= 0.1f) {
    aseActive = 0;
    return 1.0f;
  }
  float remain = 1.0f - (elapsed / aseDecaySec);
  if (remain < 0) remain = 0;
  if (remain > 1) remain = 1;
  return 1.0f + (aseInitialPct * 0.01f) * remain;
}

void serviceAfterStart(void)
{
  /* Rising edge: engine begins running while cold */
  uint8_t running = (syncLocked && rpmLive >= 400) ? 1 : 0;
  if (running && !wasRunning) {
    if (engEct < aseMinEct) {
      aseActive = 1;
      aseStartMs = HAL_GetTick();
    } else {
      aseActive = 0;
    }
  }
  if (!running && wasRunning) {
    /* stalled - allow ASE again on next start if still cold */
    if (rpmLive < 200)
      aseActive = 0;
  }
  wasRunning = running;
  if (aseActive && engEct >= (aseMinEct + 10.0f)) {
    /* fully warm - cancel remaining ASE */
    aseActive = 0;
  }
}


/**
 * Unified ignition timing
 *   final = base_map - soft_limit - ALS - FFS  (+ cranking override)
 *   slew-rate limited, clamped to [gIgnMinAdv, gIgnMaxAdv]
 */
float computeIgnitionAdvance(int8_t base_adv)
{
  float a = (float)base_adv;
  float retard = 0.0f;

  /* Fixed advance while cranking */
  if (crankAdvEnable && rpmLive > 0 && rpmLive < (int)crankAdvRpm) {
    a = crankAdvDeg;
    totalRetardDeg = 0.0f;
    softLimitRetardDeg = 0.0f;
    advTargetDeg = (int16_t)(a < 0 ? (a - 0.5f) : (a + 0.5f));
    return a;
  }

  softLimitRetardDeg = 0.0f;
  if (gRpmCutMode == 1 && gRpmLimit > 500) {
    float over = (float)rpmLive - ((float)gRpmLimit - 300.0f);
    if (over > 0.0f) {
      softLimitRetardDeg = over * (20.0f / 500.0f);
      if (softLimitRetardDeg > 25.0f) softLimitRetardDeg = 25.0f;
      retard += softLimitRetardDeg;
    }
  }

  if (alsActive) {
    float r = alsUseTable ? msRetardLookup(alsRetardTbl, (float)rpmLive)
                          : alsRetardDeg;
    retard += r;
  }
  if (ffsActive) {
    float r = ffsUseTable ? msRetardLookup(ffsRetardTbl, (float)rpmLive)
                          : ffsRetardDeg;
    retard += r;
  }
  if (launchDecayActive && launchDecayRetardDeg > 0.1f) {
    retard += launchDecayRetardDeg;
  }

  /* Idle 5×5 ign correction (ECT × RPM), only near idle */
  if (idleActive || (rpmLive > 0 && rpmLive < 1400 && engTps < 5.0f)) {
    a += idleIgnLookup(engEct, (float)rpmLive);
  }

  totalRetardDeg = retard;
  a -= retard;

  if (a < gIgnMinAdv) a = gIgnMinAdv;
  if (a > gIgnMaxAdv) a = gIgnMaxAdv;

  advTargetDeg = (int16_t)(a < 0 ? (a - 0.5f) : (a + 0.5f));

  float cur = (float)ignAdvanceDeg;
  float dt = 0.01f;
  float maxStep = advSlewDps * dt;
  float diff = (float)advTargetDeg - cur;
  if (diff > maxStep) diff = maxStep;
  if (diff < -maxStep) diff = -maxStep;
  cur += diff;
  return cur;
}


/**
 * Engine load for map axis + LOAD: telemetry.
 *
 * Mode 0 — Speed-density (MAP):
 *   load = MAP_kPa / gMapLoadRefKpa     (default ref 100 kPa → load 1.0 at 1 bar)
 *
 * Mode 1 — Alpha-N (TPS):
 *   load = TPS% / 100
 *   mild RPM fill factor below 1500 rpm (lower VE proxy)
 *
 * Mode 2 — Hybrid:
 *   load_map = MAP_kPa / ref
 *   load_tps = TPS% / 100
 *   w_rpm = clamp((rpm - 1200) / 2800, 0..1)   // more MAP weight as RPM rises
 *   w_thr = clamp(TPS%/100, 0..1)               // more MAP weight as throttle opens
 *   w = 0.5*w_rpm + 0.5*w_thr
 *   load = (1-w)*load_tps + w*load_map
 *
 * Sensor enables: disabled MAP/TPS fall back to the other, then 0.5.
 * Result is clamped to 0..5 (supports ~500 kPa on 100 kPa ref).
 */
