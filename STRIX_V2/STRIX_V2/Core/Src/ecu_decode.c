/* ecu_decode.c — auto-split from ecu_app.c */
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
#include "ecu_wheels.h"

/* Provided here as weak so an older ecu_runtime.h still links.
 * Strong definitions in ecu_runtime.c override these when present. */
__attribute__((weak)) volatile uint8_t missedGapStreak = 0;
__attribute__((weak)) volatile uint8_t missedGapArmed  = 0;

/* Cam pulse seen during the current crank revolution (gap→gap). */
static volatile uint8_t camSeenThisRev = 0;
static volatile uint8_t goodGapStreak  = 0;

#include <string.h>
#include <stdio.h>
#include <math.h>
#include <stdint.h>

/* ---- lines 397-1091 ---- */

/* ── Edge debounce (crank / cam) ───────────────────────────────
 * Time-based: reject edges closer than min interval.
 * Crank min scales with filtered tooth period (never above ~35% of T).
 * Cam min scales with expected cam period when locked.
 */
enum {
  CRANK_DEB_ABS_US   = 50UL,   /* absolute floor (~20 kHz) */
  CRANK_DEB_MAX_US   = 2500UL, /* never wait longer than this while running */
  CAM_DEB_ABS_US     = 2000UL, /* 2 ms absolute floor */
  CAM_DEB_IDLE_US    = 8000UL  /* default when period unknown */
};

/** Return 1 if edge should be accepted, 0 if bounce/noise. */
static uint8_t __attribute__((unused)) crankDebounceOk(uint32_t dt_us, uint32_t Tfilt)
{
  uint32_t minUs = CRANK_DEB_ABS_US;
  if (Tfilt >= 100UL) {
    /* ~15% of tooth period, clamped */
    uint32_t frac = Tfilt / 7UL; /* ≈14% */
    if (frac > minUs) minUs = frac;
    if (minUs > CRANK_DEB_MAX_US) minUs = CRANK_DEB_MAX_US;
    /* At high RPM Tfilt is small — keep absolute floor */
    if (Tfilt < 200UL) minUs = CRANK_DEB_ABS_US;
  }
  return (dt_us >= minUs) ? 1u : 0u;
}

static uint8_t camDebounceOk(uint32_t dt_us, uint8_t locked)
{
  uint32_t minUs = CAM_DEB_IDLE_US;
  if (locked && toothPeriodUs > 0 && gTeeth >= 2) {
    uint32_t exp = toothPeriodUs * (uint32_t)gTeeth; /* ~1 crank rev */
    if (exp < 20000UL) exp = 20000UL;
    minUs = exp / 8UL; /* reject chatter < 1/8 rev */
    if (minUs < CAM_DEB_ABS_US) minUs = CAM_DEB_ABS_US;
    if (minUs > 50000UL) minUs = 50000UL;
  } else if (!locked) {
    minUs = CAM_DEB_ABS_US; /* allow faster edges while seeking */
  }
  return (dt_us >= minUs) ? 1u : 0u;
}

void ECU_CamCapture(uint32_t capt) {
  (void)capt;
  static uint32_t lastCamUs = 0;
  uint32_t now = micros();

  if (lastCamUs) {
    uint32_t dtCam = now - lastCamUs;
    if (!camDebounceOk(dtCam, camSynced))
      return;
  }

  lastCamUs = now;
  lastCamEdgeUs = now;
  camPulseSeen = 1;
  camSeenThisRev = 1;
  cam1PhaseDeg = crankDeg;
  /* Lock is decided at the crank gap (one expected pulse per rev).
   * Do not lock here — noise bursts would set camSynced in <2 ms. */
}


/* ── Cam 2 (PB4 / TIM3_CH1) - second phase sensor ───────────── */
void ECU_Cam2Capture(uint32_t capt)
{
  (void)capt;
  static uint32_t lastCam2Us = 0;
  uint32_t now = micros();
  if (lastCam2Us) {
    uint32_t dtCam = now - lastCam2Us;
    if (!camDebounceOk(dtCam, cam2Synced))
      return;
  }

  lastCam2Us = now;
  lastCam2EdgeUs = now;
  cam2UnlockMiss = 0;
  cam2PhaseDeg = crankDeg;

  if (!cam2Synced) {
    if (cam2LockHits < 255)
      cam2LockHits++;
    if (cam2LockHits >= 3)
      cam2Synced = 1;
  }
}

/* htim3 defined in tim.c / main.c */
extern TIM_HandleTypeDef htim3;

/**
 * HAL input-capture IRQ → decoder.
 * CubeMX must configure:
 *   TIM5_CH1 = PA0  crank  (1 MHz tick recommended)
 *   TIM2_CH1 = PA15 cam1
 *   TIM3_CH1 = PB4  cam2 (optional)
 * and enable NVIC for TIM5 / TIM2 / TIM3.
 */
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
  if (htim == NULL)
    return;

  if (htim->Instance == TIM5 && htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1) {
    uint32_t capt = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
    ECU_CrankCapture(capt);
    return;
  }
  if (htim->Instance == TIM2 && htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1) {
    uint32_t capt = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
    ECU_CamCapture(capt);
    return;
  }
  if (htim->Instance == TIM3 && htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1) {
    uint32_t capt = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
    ECU_Cam2Capture(capt);
    return;
  }
}

/* Catch a TIM5 CC1 edge if the IRQ was delayed or dropped */
void ECU_CrankPoll(void)
{
  if (htim5.Instance == NULL)
    return;
  if (__HAL_TIM_GET_FLAG(&htim5, TIM_FLAG_CC1) == RESET)
    return;
  uint32_t capt = HAL_TIM_ReadCapturedValue(&htim5, TIM_CHANNEL_1);
  __HAL_TIM_CLEAR_FLAG(&htim5, TIM_FLAG_CC1);
  ECU_CrankCapture(capt);
}

void ECU_CrankCam_Start(void)
{
  /* TIM5 free-run + IC on CH1 (PA0 crank) */
  if (htim5.Instance != NULL) {
    __HAL_TIM_SET_COUNTER(&htim5, 0);
    HAL_TIM_IC_Start_IT(&htim5, TIM_CHANNEL_1);
    /* Keep counter running even if IC alone is enough on F4 */
    HAL_TIM_Base_Start(&htim5);
  }
  /* TIM2 CH1 (PA15 cam1) */
  if (htim2.Instance != NULL) {
    __HAL_TIM_SET_COUNTER(&htim2, 0);
    HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1);
    HAL_TIM_Base_Start(&htim2);
  }
  /* TIM3 CH1 (PB4 cam2) — optional */
  if (htim3.Instance != NULL) {
    __HAL_TIM_SET_COUNTER(&htim3, 0);
    HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_1);
    HAL_TIM_Base_Start(&htim3);
  }
}

/* ── RPM / period filter (no Kalman) ──────────────────────────
 * Missing-tooth decoders work in *period*, not RPM.
 *   1. Keep last 3 accepted tooth periods
 *   2. Median of 3 rejects a single spike
 *   3. IIR on the median: T = (3T + Tmed)/4
 * RPM = 60e6 / (T * teeth)
 * Kalman / 2-state CV filter removed — it fought gap lock and
 * produced a second RPM that the ISR did not actually use.
 */
static uint32_t perHist[3];
static uint8_t  perHistN = 0;

static uint32_t periodMedian3(uint32_t a, uint32_t b, uint32_t c)
{
  if (a > b) { uint32_t t = a; a = b; b = t; }
  if (b > c) { uint32_t t = b; b = c; c = t; }
  if (a > b) { uint32_t t = a; a = b; b = t; }
  return b;
}

static void rpmPeriodReset(void)
{
  perHist[0] = perHist[1] = perHist[2] = 0;
  perHistN = 0;
  toothPeriodUs = 0;
  toothPeriodFilt = 0;
  kf_rpm = 0.0f;
  kf_acc = 0.0f;
  kf_ready = 0;
  kf_nis_ema = 1.0f;
}

static uint16_t __attribute__((unused)) rpmFromPeriod(uint32_t T, uint8_t teeth)
{
  if (T < 40UL || teeth < 2)
    return 0;
  float z = 60000000.0f / ((float)T * (float)teeth);
  if (z < 30.0f) return 0;
  if (z > 15000.0f) z = 15000.0f;
  return (uint16_t)(z + 0.5f);
}

/** Accept one tooth (or gap-equivalent) period into the filter. */
static uint32_t __attribute__((unused)) rpmAcceptPeriod(uint32_t dt)
{
  if (dt < 40UL)
    return toothPeriodFilt ? toothPeriodFilt : dt;
  if (dt > 400000UL)
    dt = 400000UL;

  if (perHistN < 3) {
    perHist[perHistN++] = dt;
    toothPeriodFilt = dt;
    toothPeriodUs = dt;
    return dt;
  }
  perHist[0] = perHist[1];
  perHist[1] = perHist[2];
  perHist[2] = dt;
  uint32_t med = periodMedian3(perHist[0], perHist[1], perHist[2]);
  if (toothPeriodFilt == 0)
    toothPeriodFilt = med;
  else
    toothPeriodFilt = (toothPeriodFilt * 3UL + med) / 4UL;
  toothPeriodUs = toothPeriodFilt;
  return toothPeriodFilt;
}

void rpmKalmanReset(void)
{
  rpmPeriodReset();
}

uint16_t rpmKalmanUpdate(float z_rpm, float dt_s)
{
  /* Kept so older call sites link. RPM is owned by rpmAcceptPeriod(). */
  (void)dt_s;
  if (z_rpm < 0.0f) z_rpm = 0.0f;
  if (z_rpm > 15000.0f) z_rpm = 15000.0f;
  kf_rpm = z_rpm;
  return (uint16_t)(z_rpm + 0.5f);
}

uint16_t rpmComplementaryBlend(float rpm_fast, float rpm_slow, float alpha)
{
  if (alpha < 0.0f) alpha = 0.0f;
  if (alpha > 1.0f) alpha = 1.0f;
  float y = alpha * rpm_fast + (1.0f - alpha) * rpm_slow;
  kf_rpm = y;
  return (uint16_t)(y + 0.5f);
}


/* ── Adaptive outlier thresholds for tooth period ───────────────
 * Returns lo/hi as fraction of filtered period (percent, e.g. 55 = 55%).
 * Tightens when locked + steady; widens when noisy, unlocking, or low RPM.
 */

/**
 * Predict next edge interval from filtered tooth period + short-term accel.
 * Tpred = Tfilt + alpha*(Tfilt - Tprev), clamped.
 * Classifies measurement dt against tooth window and (optional) gap window.
 * Returns: 0 = noise (reject), 1 = normal tooth, 2 = likely gap.
 */


/* ── Digital period PLL (tooth clock) ──────────────────────────
 * Type-2 software PLL on inter-edge time.
 *
 *  e[k]  = dt_meas - T_hat * N     (N=1 tooth, N=missing+1 gap)
 *  T_hat += Kp*e + Ki*∫e           (period tracks frequency)
 *  phase accumulator optional for scheduling
 *
 * Gains scheduled by lock state and |e|/T (wide when seeking).
 */
static uint32_t pll_T = 0;          /* estimated tooth period [us] */
static int32_t  pll_integ = 0;      /* PI integrator (scaled) */
static uint8_t  pll_freqLock = 0;   /* 1 when |e| stayed small */

/** Reset period PLL (stall / enter SEEK). */
static void crankPeriodPllReset(void)
{
  pll_T = 0;
  pll_integ = 0;
  pll_freqLock = 0;
}

/**
 * Update period estimate from a measured interval spanning nTeeth periods.
 * nTeeth = 1 for normal tooth, (missing+1) for gap.
 * Returns updated tooth-period estimate [us].
 */
static uint32_t __attribute__((unused)) crankPeriodPllUpdate(uint32_t dt_us, uint8_t nTeeth, uint8_t is_gap)
{
  if (nTeeth < 1) nTeeth = 1;
  if (dt_us < 40) return pll_T ? pll_T : 40;

  /* Seed on first good sample */
  if (pll_T < 40) {
    pll_T = dt_us / (uint32_t)nTeeth;
    if (pll_T < 40) pll_T = 40;
    pll_integ = 0;
    pll_freqLock = 0;
    return pll_T;
  }

  /* Expected interval and error */
  int32_t expect = (int32_t)(pll_T * (uint32_t)nTeeth);
  int32_t e = (int32_t)dt_us - expect;

  /* Normalize error to one tooth for gain scheduling */
  int32_t e1 = e / (int32_t)nTeeth;

  /* Relative error |e|/T */
  int32_t T = (int32_t)pll_T;
  int32_t abs_e = e1 < 0 ? -e1 : e1;
  int32_t rel_pct = (T > 0) ? (abs_e * 100) / T : 100;

  /* Reject absurd errors from PLL update (noise) — keep last T */
  if (rel_pct > 80 && syncLocked && !is_gap) {
    /* don't slew period on obvious outliers while locked */
    return pll_T;
  }

  /*
   * PI gains (fixed-point):
   *   dT = (Kp_num * e1) / Kp_den + integ
   * Seeking: higher Kp, modest Ki — lock fast
   * Locked:  lower Kp, small Ki — smooth
   */
  int32_t Kp_num, Kp_den, Ki_num, Ki_den;
  if (!syncLocked || crankPllState == CRANK_PLL_SEEK || crankPllState == CRANK_PLL_CONFIRM) {
    Kp_num = 1; Kp_den = 2;   /* 0.50 */
    Ki_num = 1; Ki_den = 32;  /* 0.031 */
  } else if (crankPllState == CRANK_PLL_SOFTERR) {
    Kp_num = 1; Kp_den = 3;   /* 0.33 */
    Ki_num = 1; Ki_den = 48;
  } else {
    Kp_num = 1; Kp_den = 5;   /* 0.20 */
    Ki_num = 1; Ki_den = 64;  /* 0.016 */
  }

  /* Integrator with anti-windup (±25% of T) */
  pll_integ += (e1 * Ki_num) / Ki_den;
  {
    int32_t lim = T / 4;
    if (lim < 20) lim = 20;
    if (pll_integ > lim) pll_integ = lim;
    if (pll_integ < -lim) pll_integ = -lim;
  }

  int32_t dT = (e1 * Kp_num) / Kp_den + pll_integ;
  /* Slew limit per update (±15% T seeking, ±8% locked) */
  {
    int32_t slew = (!syncLocked) ? (T * 15) / 100 : (T * 8) / 100;
    if (slew < 10) slew = 10;
    if (dT > slew) dT = slew;
    if (dT < -slew) dT = -slew;
  }

  int32_t Tnew = T + dT;
  if (Tnew < 40) Tnew = 40;
  if (Tnew > 800000) Tnew = 800000;
  pll_T = (uint32_t)Tnew;

  /* Frequency lock flag: |e1| < 12% for several teeth tracked via soft flag */
  if (rel_pct < 12)
    pll_freqLock = 1;
  else if (rel_pct > 35)
    pll_freqLock = 0;

  return pll_T;
}

/** Predicted next tooth edge interval from PLL */
static uint32_t __attribute__((unused)) crankPeriodPllPredict(uint8_t nTeeth)
{
  if (nTeeth < 1) nTeeth = 1;
  if (pll_T < 40) return 0;
  return pll_T * (uint32_t)nTeeth;
}


static uint8_t toothPredictClass(uint32_t dt, uint32_t Tfilt, uint32_t Tprev,
                                 uint8_t miss, uint8_t locked);

/**
 * Missing-tooth gap detector (36-1, 60-2, 24-1, …).
 *
 * Combines:
 *  1) Period ratio vs Tfilt / Tpred  — gap ≈ (missing+1)×T
 *  2) Predicted edge class (toothPredictClass)
 *  3) Tooth-count between gaps ≈ (teeth − missing)
 *  4) Minimum teeth since last gap (anti double-trigger)
 *
 * Returns 1 if this edge is the missing-tooth gap, else 0.
 */

/**
 * Missing-tooth gap detection — multi-cue + position hysteresis
 *
 * Methods combined into a score (need threshold to accept):
 *  A) Period ratio: dt / T ≈ (missing+1)     classic 36-1 / 60-2
 *  B) Predictor class (accel-compensated windows)
 *  C) Position prior: teethSinceGap near "phys" (teeth − missing)
 *
 * Hysteresis:
 *  - Near expected gap index → wide timing window (easy accept)
 *  - Far from expected index → tight window + high score (hard false gap)
 *  - Locked: never unlock from a single missed classification; PLL streak does that
 *
 * Returns 1 if this edge is the gap.
 */
static uint8_t __attribute__((unused)) detectMissingToothGap(
    uint32_t dt,
    uint32_t Tfilt,
    uint32_t Tprev,
    uint8_t miss,
    uint8_t phys,
    uint8_t locked)
{
  if (miss < 1 || Tfilt < 40)
    return 0;

  /* --- Tpred with limited accel --- */
  int32_t dT = (int32_t)Tfilt - (int32_t)(Tprev ? Tprev : Tfilt);
  if (dT > (int32_t)(Tfilt / 5)) dT = (int32_t)(Tfilt / 5);
  if (dT < -(int32_t)(Tfilt / 5)) dT = -(int32_t)(Tfilt / 5);
  int32_t Tp = (int32_t)Tfilt + dT / 4;
  if (Tp < 40) Tp = 40;
  uint32_t Tpred = (uint32_t)Tp;

  uint32_t gapMul = (uint32_t)miss + 1UL; /* 36-1 → 2, 60-2 → 3 */
  uint32_t gapNom = Tpred * gapMul;

  /* Position relative to expected gap (teeth between gaps ≈ phys) */
  int16_t pos = (int16_t)teethSinceGap;
  int16_t expect = (int16_t)phys;
  int16_t posErr = (int16_t)(pos - expect);
  if (posErr < 0) posErr = (int16_t)(-posErr);

  /* In the "gap expected" band: phys−3 … phys+5 (wider when seeking) */
  uint8_t inGapZone;
  if (!locked) {
    inGapZone = (pos >= (int16_t)(phys / 2)) ? 1u : 0u; /* seeking: any mid-rev long edge */
  } else {
    int16_t lo = (int16_t)phys - 4;
    int16_t hi = (int16_t)phys + 6;
    if (lo < 4) lo = 4;
    inGapZone = (pos >= lo && pos <= hi) ? 1u : 0u;
  }

  /* Dual timing windows (Schmitt-style) */
  uint32_t tolLo, tolHi;
  if (!locked || crankPllState == CRANK_PLL_SOFTERR || crankPllState == CRANK_PLL_CONFIRM) {
    tolLo = 55; tolHi = 110; /* wide while finding sync */
  } else if (inGapZone) {
    /* Locked + right place in wheel: generous accept */
    if (rpmLive < 1000) { tolLo = 45; tolHi = 70; }
    else if (rpmLive < 3000) { tolLo = 35; tolHi = 50; }
    else { tolLo = 28; tolHi = 40; }
  } else {
    /* Locked but wrong place: only accept almost perfect gap timing */
    tolLo = 15; tolHi = 20;
  }

  uint32_t gapMin = (gapNom * (100UL - tolLo)) / 100UL;
  uint32_t gapMax = (gapNom * (100UL + tolHi)) / 100UL;
  uint32_t minAbove = Tpred + (Tpred / 3UL); /* must beat a fat tooth */
  if (gapMin < minAbove)
    gapMin = minAbove;
  if (gapMax < gapMin + Tpred)
    gapMax = gapMin + Tpred;

  uint8_t timingOk = (dt >= gapMin && dt <= gapMax) ? 1u : 0u;

  /* Score cues */
  int score = 0;
  if (timingOk)
    score += inGapZone ? 3 : 5; /* out-of-zone needs strong timing */

  if (toothPredictClass(dt, Tfilt, Tprev, miss, locked) == 2)
    score += 2;

  /* Ratio vs Tfilt alone (robust if Tpred skewed) */
  {
    uint32_t rNom = Tfilt * gapMul;
    if (dt > (rNom * 70UL) / 100UL && dt < (rNom * 140UL) / 100UL)
      score += 1;
  }

  /* Position cue */
  if (locked && inGapZone)
    score += 2;
  else if (locked && posErr <= 2)
    score += 1;
  else if (locked && !inGapZone)
    score -= 2; /* penalize false gap mid-tooth-train */

  /* Anti double-gap: too few teeth since last */
  {
    uint16_t minTeeth = (uint16_t)(phys / 3);
    if (minTeeth < 6) minTeeth = 6;
    if (lastGapUs != 0 && teethSinceGap < minTeeth && locked) {
      gapRejectStreak = (gapRejectStreak < 255) ? (uint8_t)(gapRejectStreak + 1) : 255;
      return 0;
    }
  }

  /* Accept thresholds (hysteresis via state) */
  int need = 4;
  if (!locked)
    need = 3;
  else if (inGapZone)
    need = 3; /* easier in zone */
  else
    need = 6; /* hard out of zone */

  if (score >= need) {
    gapRejectStreak = 0;
    return 1;
  }

  if (gapRejectStreak < 255)
    gapRejectStreak++;
  return 0;
}

static uint8_t toothPredictClass(uint32_t dt, uint32_t Tfilt, uint32_t Tprev,
                                 uint8_t miss, uint8_t locked)
{
  if (Tfilt < 40) Tfilt = 40;

  /* Acceleration term: how period is changing tooth-to-tooth */
  int32_t dT = (int32_t)Tfilt - (int32_t)(Tprev ? Tprev : Tfilt);
  /* Limit accel influence (noise / single spike) */
  if (dT > (int32_t)(Tfilt / 5)) dT = (int32_t)(Tfilt / 5);
  if (dT < -(int32_t)(Tfilt / 5)) dT = -(int32_t)(Tfilt / 5);

  /* alpha on accel: stronger when unlocked (track starter), milder when locked */
  int32_t a_num = locked ? 1 : 2; /* /4 */
  int32_t Tpred_i = (int32_t)Tfilt + (dT * a_num) / 4;
  if (Tpred_i < 40) Tpred_i = 40;
  if (Tpred_i > 800000) Tpred_i = 800000;
  uint32_t Tpred = (uint32_t)Tpred_i;

  /* Window scale % — wider when seeking / low RPM */
  uint32_t loPct, hiPct;
  if (!locked || rpmLive < 600) {
    loPct = 55; hiPct = 150;
  } else if (rpmLive < 1500) {
    loPct = 65; hiPct = 140;
  } else if (rpmLive < 3500) {
    loPct = 70; hiPct = 130;
  } else {
    loPct = 75; hiPct = 125;
  }

  uint32_t toothLo = (Tpred * loPct) / 100UL;
  uint32_t toothHi = (Tpred * hiPct) / 100UL;
  if (toothLo < 40) toothLo = 40;

  if (dt >= toothLo && dt <= toothHi)
    return 1; /* normal tooth */

  if (miss >= 1) {
    uint32_t gapMul = (uint32_t)(miss + 1);
    uint32_t gapNom = Tpred * gapMul;
    /* Gap windows slightly wider than tooth windows */
    uint32_t glo = (gapNom * (loPct > 10 ? loPct - 10 : loPct)) / 100UL;
    uint32_t ghi = (gapNom * (hiPct + 15)) / 100UL;
    if (glo < toothHi + (Tpred / 5UL))
      glo = toothHi + (Tpred / 5UL); /* must sit above tooth band */
    if (dt >= glo && dt <= ghi)
      return 2; /* predicted gap */
  }

  /* Outside both windows */
  if (!locked) {
    /* Unlocking: still allow very long intervals as possible first gap */
    if (miss >= 1 && dt > toothHi && dt < (Tpred * (miss + 1) * 2UL))
      return 2;
    if (dt >= toothLo && dt < (toothHi * 2UL))
      return 1; /* soft accept while seeking */
  }
  return 0; /* noise */
}

static void __attribute__((unused)) toothOutlierPct(uint32_t *loPct, uint32_t *hiPct)
{
  /* Base by RPM band */
  uint16_t rpm = rpmLive;
  uint32_t lo, hi;
  if (rpm < 400) {
    lo = 35; hi = 200;           /* cranking: very open */
  } else if (rpm < 900) {
    lo = 42; hi = 180;
  } else if (rpm < 2000) {
    lo = 50; hi = 165;
  } else if (rpm < 4000) {
    lo = 55; hi = 150;
  } else {
    lo = 60; hi = 140;           /* high RPM: noise is short */
  }

  /* Unlock / soft lock → widen */
  if (!syncLocked) {
    lo = (lo * 80UL) / 100UL;
    if (lo < 30) lo = 30;
    hi = (hi * 120UL) / 100UL;
    if (hi > 220) hi = 220;
  } else if (gapConfirm < 2) {
    lo = (lo * 90UL) / 100UL;
    hi = (hi * 110UL) / 100UL;
  }

  /* Recent tooth error pressure: widen if noisy so we don't reject everything */
  static uint16_t errEma = 0;    /* 0..1000 scaled */
  /* caller bumps toothErrors; approximate rate via low-pass of errors flag is hard —
   * use toothErrors absolute steps vs previous. */
  static uint32_t lastErrCnt = 0;
  uint32_t e = toothErrors;
  uint32_t de = (e >= lastErrCnt) ? (e - lastErrCnt) : 0;
  lastErrCnt = e;
  if (de > 0)
    errEma = (uint16_t)((errEma * 3u + 200u) / 4u);
  else
    errEma = (uint16_t)((errEma * 7u) / 8u);

  if (errEma > 120) {
    /* noisy environment — open gates */
    lo = (lo * 85UL) / 100UL;
    hi = (hi * 115UL) / 100UL;
  } else if (errEma < 20 && syncLocked && rpm > 1500) {
    /* clean + locked — tighten */
    lo = (lo * 105UL) / 100UL;
    if (lo > 70) lo = 70;
    hi = (hi * 95UL) / 100UL;
    if (hi < 130) hi = 130;
  }

  /* Kalman innovation: high NIS → model/measurement disagree → widen */
  if (kf_nis_ema > 3.0f) {
    lo = (lo * 88UL) / 100UL;
    hi = (hi * 112UL) / 100UL;
  } else if (kf_nis_ema < 0.4f && syncLocked && rpm > 1500) {
    lo = (lo * 103UL) / 100UL;
    hi = (hi * 97UL) / 100UL;
  }

  if (lo < 28) lo = 28;
  if (lo > 70) lo = 70;
  if (hi < 125) hi = 125;
  if (hi > 230) hi = 230;

  *loPct = lo;
  *hiPct = hi;
}


/* ── Crank PLL + hysteresis ────────────────────────────────────
 * SEEK → CONFIRM → LOCKED ⇄ SOFTERR → SEEK
 *
 * Hysteresis rules (enter hard / leave soft):
 *  - LOCK:   need PLL_GAPS_LOCK consecutive good gaps (2 while cranking)
 *  - SOFTERR: 1 missed gap while LOCKED (sync stays true)
 *  - SEEK:   PLL_MISS_UNLOCK consecutive missed gaps while SOFTERR
 *  - Recover LOCKED from SOFTERR: PLL_GAPS_RECOVER good gaps in a row
 *  - Edge noise never forces SEEK by itself
 */
enum {
  PLL_GAPS_LOCK      = 3,  /* good gaps to enter LOCKED */
  PLL_GAPS_LOCK_CRANK= 2,  /* at starter RPM */
  PLL_GAPS_RECOVER   = 2,  /* keep 2 good gaps to leave SOFTERR */  /* good gaps to leave SOFTERR */
  PLL_MISS_TO_SOFT   = 1,  /* missed gaps LOCKED → SOFTERR */
  PLL_MISS_UNLOCK    = 5,  /* was 3 — more hysteresis */  /* missed gaps SOFTERR → SEEK */
  PLL_MISS_UNLOCK_LO = 6,  /* was 4 */  /* same at low RPM */
  PLL_MISS_ABORT_CFM = 3,  /* missed gaps abort CONFIRM */
  PLL_SOFTERR_NOISE  = 80, /* edge-noise score → SOFTERR only */
  PLL_CAM_LOCK_HITS  = 3,  /* cam edges to lock */
  PLL_CAM_UNLOCK_MISS= 5   /* missed cam windows to unlock */
};

static void crankPllEnterSeek(void)
{
  crankPllState = CRANK_PLL_SEEK;
  syncLocked = 0;
  gapConfirm = 0;
  pllSoftErr = 0;
  pllGoodStreak = 0;
  missedGapStreak = 0;
  missedGapArmed = 0;
  toothIndex = 0;
  teethSinceGap = 0;
  crankPeriodPllReset();
}

static void __attribute__((unused)) crankPllOnGoodGap(void)
{
  pllSoftErr = 0;
  missedGapStreak = 0;
  missedGapArmed = 0;
  if (pllGoodStreak < 255) pllGoodStreak++;
  if (gapConfirm < 255) gapConfirm++;

  switch (crankPllState) {
  case CRANK_PLL_SEEK:
    crankPllState = CRANK_PLL_CONFIRM;
    gapConfirm = 1;
    pllGoodStreak = 1;
    /* Soft assist only while cranking — still CONFIRM */
    if (rpmLive > 0 && rpmLive < 400)
      syncLocked = 1;
    break;

  case CRANK_PLL_CONFIRM:
    if (gapConfirm >= PLL_GAPS_LOCK ||
        (gapConfirm >= PLL_GAPS_LOCK_CRANK && rpmLive > 0 && rpmLive < 900)) {
      crankPllState = CRANK_PLL_LOCKED;
      syncLocked = 1;
      pllGoodStreak = 0;
    } else if (gapConfirm >= 1 && rpmLive > 0 && rpmLive < 450) {
      syncLocked = 1; /* fuel/spark assist; state remains CONFIRM */
    }
    break;

  case CRANK_PLL_SOFTERR:
    /* Leave soft-error only after consecutive good gaps */
    if (pllGoodStreak >= PLL_GAPS_RECOVER) {
      crankPllState = CRANK_PLL_LOCKED;
      syncLocked = 1;
      pllGoodStreak = 0;
    } else {
      syncLocked = 1;
    }
    break;

  case CRANK_PLL_LOCKED:
  default:
    syncLocked = 1;
    break;
  }
}

static void __attribute__((unused)) crankPllOnMissedGap(void)
{
  if (missedGapArmed)
    return;
  missedGapArmed = 1;

  if (missedGapStreak < 255)
    missedGapStreak++;

  if (pllSoftErr < 250)
    pllSoftErr = (uint8_t)(pllSoftErr + 8);

  pllGoodStreak = 0; /* break recover chain */

  if (crankPllState == CRANK_PLL_LOCKED) {
    if (missedGapStreak >= PLL_MISS_TO_SOFT)
      crankPllState = CRANK_PLL_SOFTERR;
    /* syncLocked stays 1 */
  } else if (crankPllState == CRANK_PLL_SOFTERR) {
    uint8_t need = (rpmLive < 800) ? PLL_MISS_UNLOCK_LO : PLL_MISS_UNLOCK;
    if (missedGapStreak >= need) {
      syncLosses++;
      rpmKalmanReset();
      crankPllEnterSeek();
    }
  } else if (crankPllState == CRANK_PLL_CONFIRM) {
    if (missedGapStreak >= PLL_MISS_ABORT_CFM)
      crankPllEnterSeek();
    else if (gapConfirm > 0)
      gapConfirm--;
  }
}

static void __attribute__((unused)) crankPllOnError(uint8_t weight)
{
  if (pllSoftErr < 250)
    pllSoftErr = (uint8_t)(pllSoftErr + weight);
  if (crankPllState == CRANK_PLL_LOCKED && pllSoftErr >= PLL_SOFTERR_NOISE)
    crankPllState = CRANK_PLL_SOFTERR;
}

static void __attribute__((unused)) crankPllOnGoodTooth(void)
{
  if (pllSoftErr)
    pllSoftErr--;
  /* SOFTERR → LOCKED only via good gaps (PLL_GAPS_RECOVER) */
}


/* Reject bounce and 15k-cap noise. Max 12000 RPM. */
static void rpmAccept(uint32_t periodUs, uint8_t teeth)
{
  if (periodUs < 80UL || teeth < 2)
    return;
  uint32_t z = 60000000UL / (periodUs * (uint32_t)teeth);
  if (z < 20UL || z > 12000UL)
    return;
  if (rpmLive >= 40u) {
    uint32_t prev = rpmLive;
    if (z > prev * 3UL + 200UL)
      return; /* single spike */
    rpmLive = (uint16_t)((prev * 3u + (uint16_t)z) / 4u);
  } else {
    rpmLive = (uint16_t)z;
  }
}

static void decoderPublishAngle(uint8_t use720)
{
  /* Wheel degrees from the gap. Trigger offset is applied at the coil,
   * not here — adding it in both places slid sequential TDC by 2×trig. */
  uint16_t ax = ECU_Trigger_AngleX10((uint8_t)(toothIndex > 255 ? 255 : toothIndex));
  float a = (float)ax * 0.1f;
  if (use720 && cycleHalf)
    a += 360.0f;
  while (a >= 720.0f) a -= 720.0f;
  while (a < 0.0f) a += (use720 ? 720.0f : 360.0f);
  crankDeg = a;
}

/* ── Crank (missing-tooth + cam-home) ─────────────────────────
 * Gap = first edge after the missing teeth. That edge is toothIndex=0, 0°.
 * Cam pulse during a crank rev marks that rev as 720° half 0.
 * Wasted/batch stay on 360°. Sequential uses 720° only when camSynced.
 */
void ECU_CrankCapture(uint32_t capt)
{
  /*
   * rusEFI-style gap-RATIO decoder (original STRIX code, not rusEFI source).
   * ISR stays integer-only: compare dt / prevDt to (missing+1).
   * No Kalman, no PLL, no float in the hot path — avoids lockups.
   */
  static uint32_t lastCapt = 0;
  static uint32_t prevToothDt = 0;
  static uint8_t  gapHits = 0;
  uint32_t now = micros();

  crankEdgeCount++;

  if (lastCapt == 0) {
    lastCapt = capt;
    lastToothUs = now;
    return;
  }

  uint32_t dt = capt - lastCapt;
  lastCapt = capt;

  /* Hard reject bounce / stalled overflow — no rollback games */
  if (dt < 80UL || dt > 800000UL)
    return;

  lastToothUs = now;

  /* Wall-clock tooth period — independent of TIM tick unit. */
  {
    static uint32_t prevEdgeUs;
    if (prevEdgeUs != 0) {
      uint32_t du = now - prevEdgeUs;
      if (du >= 80UL && du <= 500000UL) {
        toothPeriodUs = du;
        if (toothPeriodFilt)
          toothPeriodFilt = (toothPeriodFilt * 3UL + du) / 4UL;
        else
          toothPeriodFilt = du;
      }
    }
    prevEdgeUs = now;
  }

  const EcuTriggerShape *sh = ECU_Trigger_Shape();
  uint8_t miss = sh->missing;
  uint8_t nom  = sh->count;
  uint8_t phys = sh->phys;
  if (nom < 4) nom = (gTeeth > 1) ? gTeeth : 36;
  if (phys < 2) phys = 2;

  uint8_t isGap = 0;
  if (miss >= 1 && prevToothDt >= 150UL && prevToothDt <= 80000UL) {
    uint32_t rq = (dt << 8) / prevToothDt;
    if (rq >= sh->gap_lo_q8 && rq <= sh->gap_hi_q8)
      isGap = 1;
  }

  if (isGap) {
    missedGapStreak = 0;
    if (gapHits < 255)
      gapHits++;
    if (gapHits >= 2) {
      syncLocked = 1;
      crankPllState = CRANK_PLL_LOCKED;
    }

    uint32_t T = dt / ((uint32_t)miss + 1UL);
    if (T < 150UL) T = 150UL;
    toothPeriodUs = T;
    toothPeriodFilt = T;
    prevToothDt = T;

    if (lastGapUs) {
      uint32_t rev = now - lastGapUs;
      if (rev >= 5000UL && rev <= 2000000UL)
        rpmAccept(rev / (uint32_t)nom, nom);
    }
    lastGapUs = now;
    toothIndex = 0;
    teethSinceGap = 0;
    crankRevId++;

    if (camSeenThisRev) {
      camUnlockMiss = 0;
      if (camLockHits < 255)
        camLockHits++;
      if (camLockHits >= 2)
        camSynced = 1;
    } else if (syncLocked) {
      if (camUnlockMiss < 255)
        camUnlockMiss++;
      if (camUnlockMiss >= 5) {
        camSynced = 0;
        camLockHits = 0;
      }
    }

    /* 720° half only from a real cam pulse. XOR free-run is 360° out
     * half the time and is what made sequential look "out of sync". */
    if ((gIgnMode == 1 || gInjMode >= 2) && camSynced)
      cycleHalf = camSeenThisRev ? 0u : 1u;
    else if (!camSynced)
      cycleHalf = 0;
    camSeenThisRev = 0;
    decoderPublishAngle(camSynced && (gIgnMode == 1 || gInjMode >= 2));
    return;
  }

  /* Normal tooth — skip period update on long outliers (false gaps) */
  {
    uint32_t rq = (prevToothDt >= 120UL) ? ((dt << 8) / prevToothDt) : 256UL;
    if (rq >= 160UL && rq <= 400UL) { /* 0.63x .. 1.56x */
      prevToothDt = dt;
      toothPeriodUs = dt;
      if (toothPeriodFilt)
        toothPeriodFilt = (toothPeriodFilt * 3UL + dt) / 4UL;
      else
        toothPeriodFilt = dt;
    }
  }

  rpmAccept(toothPeriodFilt ? toothPeriodFilt : toothPeriodUs, nom);

  if (toothIndex < 65000)
    toothIndex++;
  if (teethSinceGap < 60000)
    teethSinceGap++;

  if (miss == 0 && toothIndex >= nom) {
    toothIndex = 0;
    if (ignSequentialActive() || injSequentialActive()) {
      if (camSynced)
        cycleHalf = camSeenThisRev ? 0u : 1u;
    } else {
      cycleHalf = 0;
    }
    camSeenThisRev = 0;
    syncLocked = 1;
  }

  if (miss >= 1 && syncLocked && teethSinceGap > (uint16_t)(phys * 6u + 4u)) {
    missedGapStreak++;
    teethSinceGap = 0;
    if (missedGapStreak >= 8) {
      syncLocked = 0;
      camSynced = 0;
      gapHits = 0;
      crankPllState = CRANK_PLL_SEEK;
      syncLosses++;
    }
  }

  decoderPublishAngle(camSynced && (gIgnMode == 1 || gInjMode >= 2));
}

/* ── Maps ───────────────────────────────────────────────────── */
/**
 * Bilinear interpolation on 12×22 maps (same math used for VE-style fuel tables).
 *
 * Axis: rpmBinsLive[c] (RPM), mapBinsLive[r] (normalised load).
 * Cell values: advMap[r][c] degrees; injMap[r][c] in 0.1 ms units.
 *
 *   V = (1-cf)(1-rf) V00 + cf(1-rf) V01 + (1-cf) rf V10 + cf rf V11
 *
 * where cf, rf ∈ [0,1] are fractional positions between the bracketing bins.
 * Below first bin / above last bin → clamp to edge cell (no extrapolation).
 */
