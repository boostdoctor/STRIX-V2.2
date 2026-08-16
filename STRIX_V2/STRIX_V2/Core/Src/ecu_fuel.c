/* ecu_fuel.c — injection scheduling */
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

/*
 * BATCH / wasted path: angle-based SOI only (NO injReq).
 * SEQUENTIAL: injReq from gap/cam only (NO angle SOI).
 * This prevents the historic double-fire when both paths armed the same cyl.
 */
void serviceInjection(void)
{
  uint32_t now = micros();
  if ((rpmCutActive && gRpmCutMode == 0) || dfcoActive) {
    for (uint8_t i = 1; i <= MAX_CYL; i++) {
      ECU_INJ_LO(i); injOn[i] = 0; injReq[i] = 0;
    }
    return;
  }
  uint16_t pw = injPwUs;
  if (pw < 400) pw = 400;   /* min 400 us */
  if (pw > 20000) pw = 20000;

  for (uint8_t i = 1; i <= MAX_CYL; i++) {
    if (injOn[i] && (int32_t)(now - injEndUs[i]) >= 0) {
      ECU_INJ_LO(i);
      injOn[i] = 0;
      injFiredCyc[i] = 1;
    }
  }

  if (!syncLocked || toothPeriodUs < 40 || toothPeriodUs > 800000UL) {
    for (uint8_t i = 1; i <= MAX_CYL; i++) {
      if (!injOn[i]) {
        ECU_INJ_LO(i);
        injReq[i] = 0;
      }
    }
    return;
  }

  float usPerRev = (float)toothPeriodUs * (float)((gTeeth > 0) ? gTeeth : 36);
  if (usPerRev < 400.0f) return;

  uint8_t seq = injSequentialActive();
  float cycle = seq ? 720.0f : 360.0f;
  float deg = crankDeg;
  float band = 360.0f / (float)((gTeeth > 0) ? gTeeth : 36);
  float eoiOfs = gEoiBtdc;
  if (eoiOfs < 10.0f) eoiOfs = 10.0f;
  if (eoiOfs > 400.0f) eoiOfs = 400.0f;

  uint8_t n = gCyl;
  if (n > MAX_CYL) n = MAX_CYL;
  if (!seq) n = (gCyl >= 4) ? 4 : gCyl;

  for (uint8_t i = 1; i <= n; i++) {
    float tdc;
    if (seq) {
      tdc = tdcDeg(i);
    } else {
      if (i == 1 || i == 4) tdc = 0.0f;
      else if (i == 2 || i == 3) tdc = 180.0f;
      else continue;
    }

    float eoi = wrapAngle(tdc - eoiOfs, cycle);

    /* Re-arm once well past EOI */
    if (injFiredCyc[i] && !injOn[i]) {
      float past = wrapAngle(deg - eoi, cycle);
      if (past > cycle * 0.40f && past < cycle * 0.95f)
        injFiredCyc[i] = 0;
    }

    uint8_t start = 0;

    if (seq) {
      /* Sequential: ONLY injReq from decoder (once per 720°) */
      if (injReq[i] && !injOn[i] && !injFiredCyc[i]) {
        start = 1;
        injReq[i] = 0;
      }
    } else {
      /* Batch: ONLY angle SOI — ignore injReq to stop double-fire */
      injReq[i] = 0;
      if (!injOn[i] && !injFiredCyc[i]) {
        float degPerUs = 360.0f / usPerRev;
        float pwDeg = (float)pw * degPerUs;
        if (pwDeg < 1.0f) pwDeg = 1.0f;
        float soi = wrapAngle(eoi - pwDeg, cycle);
        float cap = band * 1.5f;
        if (cap < 8.0f) cap = 8.0f;
        if (cap > 18.0f) cap = 18.0f;
        if (angleActive(deg, soi, wrapAngle(soi + cap, cycle), cycle))
          start = 1;
      }
    }

    if (start) {
      uint16_t pwc = pw;
      if (i >= 1 && i <= MAX_CYL) {
        float tr = 1.0f + cylTrimPct[i] * 0.01f;
        if (tr < 0.75f) tr = 0.75f;
        if (tr > 1.25f) tr = 1.25f;
        pwc = (uint16_t)((float)pw * tr);
        if (pwc < 400) pwc = 400;
        if (pwc > 20000) pwc = 20000;
      }
      ECU_INJ_HI(i);
      injOn[i] = 1;
      injEndUs[i] = now + pwc;
      injReq[i] = 0;
      injFiredCyc[i] = 1;
    }
  }
}

void serviceO2ClosedLoop(void)
{
  uint32_t now = millis();
  if (o2LastMs == 0) o2LastMs = now;
  float dt = (float)(now - o2LastMs) * 0.001f;
  if (dt < 0.001f) dt = 0.001f;
  if (dt > 0.05f) dt = 0.05f;
  o2LastMs = now;

  /* Filter O2 voltage */
  o2Filt = o2Filt * 0.85f + engO2 * 0.15f;

  /* Enable conditions */
  o2ClActive = 0;
  if (!o2ClEnable || o2SensorMode == O2_MODE_OFF) {
    stftPct *= 0.99f;
    return;
  }
  if (aseActive || dfcoActive || alsActive)
    return;
  if (!syncLocked || rpmLive < 800 || rpmLive > 4000) return;
  if (engEct < 60.0f) return;
  if (engTps > 60.0f) return;
  if (boostTargetKpa > 10.0f) return;
  if (engBat < 11.0f) return;

  o2ClActive = 1;

  if (o2SensorMode == O2_MODE_WB) {
    /* Wideband: drive engAfr → targetAfr (rich = low AFR → cut fuel) */
    float err = engAfr - targetAfr; /* >0 lean, <0 rich */
    if (err < -0.05f)
      stftPct -= STFT_STEP * (dt * 100.0f);
    else if (err > 0.05f)
      stftPct += STFT_STEP * (dt * 100.0f);
    else
      stftPct *= (1.0f - 0.5f * dt);
  } else {
    /* Narrowband voltage windows */
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
  }
  if (stftPct >  STFT_MAX) stftPct =  STFT_MAX;
  if (stftPct < -STFT_MAX) stftPct = -STFT_MAX;

  /* LTFT learns slow average of STFT */
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
    pct += stftPct;              /* STFT only when NB CL active */
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


/** Windowed dual-Goertzel knock → intensity → progressive retard */
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
 * Unified ignition timing (strong definition — replaces weak stub in runtime)
 *   final = base_map - soft_limit - ALS - FFS - knock
 *   slew-rate limited, clamped to [gIgnMinAdv, gIgnMaxAdv]
 */
float computeIgnitionAdvance(int8_t base_adv)
{
  float a = (float)base_adv;
  float retard = 0.0f;

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
  retard += knockRetardDeg;

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
