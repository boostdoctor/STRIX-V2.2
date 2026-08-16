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
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim5;

#include <string.h>
#include <stdio.h>
#include <math.h>
#include <stdint.h>

/* ---- lines 397-1091 ---- */
void ECU_CamCapture(uint32_t capt) {
  (void)capt;
  static uint32_t lastCamUs = 0;
  uint32_t now = micros();

  /* Debounce: ignore edges closer than ~8 ms (noise / bounce) */
  if (lastCamUs && (now - lastCamUs) < 8000UL)
    return;

  /* Always stamp edge for live CAM activity (strip shows pulse even in Batch). */
  lastCamEdgeUs = now;
  camPulseSeen = 1;
  {
    uint8_t wantSeq = (gIgnMode == 1) || (gInjMode == 2) || (gInjMode == 3);
    if (!wantSeq) {
      /* Batch / wasted: report edge only — no 720° phase lock */
      if (camSynced) {
        camSynced = 0;
        camLockHits = 0;
      }
      lastCamUs = now;
      return;
    }
  }

  /* Expected cam period ~ 1 crank rev (2-stroke phase) or 2 revs.
   * Reject absurdly fast repeats already handled; reject if still spinning
   * and edge is far outside 0.3×-3× expected window after lock. */
  if (camSynced && toothPeriodUs > 0 && gTeeth >= 2) {
    uint32_t expUs = toothPeriodUs * (uint32_t)gTeeth; /* ~1 crank rev */
    if (expUs < 20000UL) expUs = 20000UL;
    uint32_t dtCam = now - lastCamUs;
    if (lastCamUs && dtCam < (expUs / 4UL)) {
      /* chatter relative to engine speed */
      return;
    }
  }

  lastCamUs = now;
  lastCamEdgeUs = now;
  camUnlockMiss = 0; /* good edge resets unlock counter */

  cam1PhaseDeg = crankDeg;

  /* Hysteresis to LOCK: need 2 valid edges (or 1 if already had recent activity) */
  if (!camSynced) {
    if (camLockHits < 255)
      camLockHits++;
    if (camLockHits >= 2) {
      camSynced = 1;
      cycleHalf = 0; /* CAM home = 720° phase absolute zero */
      camLockHits = 0;
    }
  } else {
    /* Each cam home edge re-asserts 720° phase (once per two crank revs) */
    cycleHalf = 0;
  }
  /* Never reset toothIndex/crankDeg here - crank decoder owns angle */
}


/* ── Cam 2 (PB4 / TIM3_CH1) - second phase sensor ───────────── */
void ECU_Cam2Capture(uint32_t capt)
{
  (void)capt;
  static uint32_t lastCam2Us = 0;
  uint32_t now = micros();
  if (lastCam2Us && (now - lastCam2Us) < 8000UL)
    return;

  if (cam2Synced && toothPeriodUs > 0 && gTeeth >= 2) {
    uint32_t expUs = toothPeriodUs * (uint32_t)gTeeth;
    if (expUs < 20000UL) expUs = 20000UL;
    if (lastCam2Us && (now - lastCam2Us) < (expUs / 4UL))
      return;
  }

  lastCam2Us = now;
  lastCam2EdgeUs = now;
  cam2UnlockMiss = 0;
  cam2PhaseDeg = crankDeg;

  if (!cam2Synced) {
    if (cam2LockHits < 255)
      cam2LockHits++;
    if (cam2LockHits >= 2)
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
  /* Instance-only match — Channel field is unreliable across HAL versions */
  if (htim->Instance == TIM5) {
    uint32_t capt = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
    ECU_CrankCapture(capt);
    return;
  }
  if (htim->Instance == TIM2) {
    uint32_t capt = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
    ECU_CamCapture(capt);
    return;
  }
  if (htim->Instance == TIM3) {
    uint32_t capt = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
    ECU_Cam2Capture(capt);
    return;
  }
}


/** Start IC IRQs after MX_TIMx_Init() — call once from ECU_Init(). */
void ECU_CrankCam_Start(void)
{
  if (htim5.Instance != NULL) {
    __HAL_TIM_SET_COUNTER(&htim5, 0);
    __HAL_TIM_CLEAR_FLAG(&htim5, TIM_FLAG_CC1 | TIM_FLAG_CC1OF);
    (void)HAL_TIM_Base_Start(&htim5);
    /* Start capture with IRQ */
    if (HAL_TIM_IC_Start_IT(&htim5, TIM_CHANNEL_1) != HAL_OK)
      (void)HAL_TIM_IC_Start_IT(&htim5, TIM_CHANNEL_1);
    __HAL_TIM_ENABLE_IT(&htim5, TIM_IT_CC1);
  }
  if (htim2.Instance != NULL) {
    __HAL_TIM_SET_COUNTER(&htim2, 0);
    __HAL_TIM_CLEAR_FLAG(&htim2, TIM_FLAG_CC1 | TIM_FLAG_CC1OF);
    (void)HAL_TIM_Base_Start(&htim2);
    (void)HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1);
    __HAL_TIM_ENABLE_IT(&htim2, TIM_IT_CC1);
  }
  if (htim3.Instance != NULL) {
    __HAL_TIM_SET_COUNTER(&htim3, 0);
    __HAL_TIM_CLEAR_FLAG(&htim3, TIM_FLAG_CC1 | TIM_FLAG_CC1OF);
    (void)HAL_TIM_Base_Start(&htim3);
    (void)HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_1);
    __HAL_TIM_ENABLE_IT(&htim3, TIM_IT_CC1);
  }
}

/** Poll TIM5 CC1 if IRQ was missed — call from ECU_Loop. */
void ECU_CrankPoll(void)
{
  if (htim5.Instance == NULL)
    return;
  if (__HAL_TIM_GET_FLAG(&htim5, TIM_FLAG_CC1) != RESET) {
    __HAL_TIM_CLEAR_FLAG(&htim5, TIM_FLAG_CC1);
    uint32_t capt = HAL_TIM_ReadCapturedValue(&htim5, TIM_CHANNEL_1);
    ECU_CrankCapture(capt);
  }
}


/** Reset RPM Kalman (stall / first edge). */
void rpmKalmanReset(void)
{
  kf_rpm = 0.0f;
  kf_acc = 0.0f;
  kf_p00 = 1.0e4f;
  kf_p01 = 0.0f;
  kf_p10 = 0.0f;
  kf_p11 = 1.0e4f;
  kf_ready = 0;
  kf_nis_ema = 1.0f;
  kf_R_adapt = 150.0f;
  kf_q_adapt = 12000.0f;
}


/**
 * Adaptive process noise spectral density q_a [(rpm/s²)²·s].
 * Raise Q during tip-in (high |accel| or high NIS) so filter tracks;
 * lower Q when steady so RPM is smooth.
 */
float rpmKalmanAdaptQ(float dt_s)
{
  float q = 8000.0f; /* baseline */

  if (!syncLocked)
    q *= 2.5f;
  else if (gapConfirm < 2)
    q *= 1.5f;

  /* |accel| schedule */
  float a = kf_acc;
  if (a < 0.0f) a = -a;
  if (a > 5000.0f)
    q *= 3.0f;
  else if (a > 2500.0f)
    q *= 2.0f;
  else if (a > 1000.0f)
    q *= 1.4f;
  else if (a < 200.0f && syncLocked)
    q *= 0.6f; /* very steady */

  /* Innovation history: NIS>>1 means model is too stiff → raise Q */
  if (kf_nis_ema > 4.0f)
    q *= 2.0f;
  else if (kf_nis_ema > 2.0f)
    q *= 1.4f;
  else if (kf_nis_ema < 0.3f && syncLocked)
    q *= 0.7f; /* overconfident residuals → can lower Q */

  /* Low RPM: more process uncertainty */
  if (kf_rpm < 800.0f)
    q *= 1.5f;

  /* Clamp */
  if (q < 1500.0f) q = 1500.0f;
  if (q > 80000.0f) q = 80000.0f;

  /* Smooth q_a itself so Q does not jump every tooth */
  kf_q_adapt = 0.90f * kf_q_adapt + 0.10f * q;
  (void)dt_s;
  return kf_q_adapt;
}

/**
 * Adaptive measurement noise R [RPM²].
 * Base from operating point; then innovation-based inflate/deflate.
 * y = innovation, S0 = p00 + R_base (before adapt).
 */
float rpmKalmanAdaptR(float z_rpm, float y, float p00)
{
  /* Base R from sync + RPM band */
  float R = syncLocked ? 90.0f : 320.0f;
  if (z_rpm < 500.0f)
    R *= 2.0f;
  else if (z_rpm < 1000.0f)
    R *= 1.5f;
  else if (z_rpm < 2000.0f)
    R *= 1.15f;
  else if (z_rpm > 5000.0f)
    R *= 0.85f; /* cleaner tooth periods at high RPM */

  if (toothErrors > 15)
    R *= 1.3f;

  float S0 = p00 + R;
  if (S0 < 1.0f) S0 = 1.0f;

  /* Normalized innovation squared χ² ≈ y²/S  (1-DOF) */
  float nis = (y * y) / S0;
  /* EMA of NIS for long-term adapt */
  kf_nis_ema = 0.92f * kf_nis_ema + 0.08f * nis;

  /* Soft inflation curve: R *= max(1, c * nis) with limit */
  if (nis > 9.0f)       /* ~3σ */
    R *= 5.0f;
  else if (nis > 4.0f)  /* ~2σ */
    R *= 2.5f;
  else if (nis > 2.0f)
    R *= 1.5f;
  else if (nis < 0.25f && syncLocked)
    R *= 0.85f; /* consistently small residuals */

  /* Blend with previous R to avoid gain chatter */
  kf_R_adapt = 0.75f * kf_R_adapt + 0.25f * R;
  if (kf_R_adapt < 40.0f) kf_R_adapt = 40.0f;
  if (kf_R_adapt > 5000.0f) kf_R_adapt = 5000.0f;
  return kf_R_adapt;
}

/**
 * 2-state constant-velocity Kalman filter for engine RPM.
 *
 * State:    x = [ rpm ,  accel_rpm_per_s ]^T
 * Model:    x_k = F x_{k-1} + w ,   F = [ 1  dt ]
 *                                   [ 0   1 ]
 * Measure:  z = H x + v ,           H = [ 1  0 ]
 *
 * Predict:  x̂⁻ = F x̂
 *           P⁻  = F P Fᵀ + Q
 * Update:   y  = z - H x̂⁻
 *           S  = H P⁻ Hᵀ + R
 *           K  = P⁻ Hᵀ S⁻¹
 *           x̂  = x̂⁻ + K y
 *           P  = (I-KH) P⁻ (I-KH)ᵀ + K R Kᵀ   (Joseph form)
 *
 * Q scales with dt; R is higher when unlocked / low RPM / large innovation.
 * Returns filtered RPM in [0, 15000].
 */
uint16_t rpmKalmanUpdate(float z_rpm, float dt_s)
{
  /* ---- sanitize inputs ---- */
  if (z_rpm < 0.0f)     z_rpm = 0.0f;
  if (z_rpm > 15000.0f) z_rpm = 15000.0f;
  if (dt_s < 1.0e-5f)   dt_s = 1.0e-5f;
  if (dt_s > 0.5f)      dt_s = 0.5f;

  /* ---- cold start: seed state from first measurement ---- */
  if (!kf_ready) {
    kf_rpm = z_rpm;
    kf_acc = 0.0f;
    kf_p00 = 400.0f;   /* RPM variance */
    kf_p01 = 0.0f;
    kf_p10 = 0.0f;
    kf_p11 = 2500.0f;  /* accel variance */
    kf_ready = 1;
    return (uint16_t)(z_rpm + 0.5f);
  }

  const float dt = dt_s;
  const float dt2 = dt * dt;

  /* ========== PREDICT ========== */
  /* x̂⁻ = F x̂ */
  float x0 = kf_rpm + kf_acc * dt;
  float x1 = kf_acc;

  /* Adaptive process noise Q (discrete white-noise accel) */
  float q_a = rpmKalmanAdaptQ(dt);
  float Q00 = q_a * dt2 * dt2 / 4.0f;
  float Q01 = q_a * dt2 * dt / 2.0f;
  float Q11 = q_a * dt2;
  Q00 += 20.0f * dt; /* small RPM random walk */

  /* P⁻ = F P Fᵀ + Q
   * F P Fᵀ:
   *  p00' = p00 + dt*(p10+p01) + dt²*p11
   *  p01' = p01 + dt*p11
   *  p10' = p10 + dt*p11
   *  p11' = p11
   */
  float p00 = kf_p00 + dt * (kf_p10 + kf_p01) + dt2 * kf_p11 + Q00;
  float p01 = kf_p01 + dt * kf_p11 + Q01;
  float p10 = kf_p10 + dt * kf_p11 + Q01;
  float p11 = kf_p11 + Q11;

  /* ========== UPDATE ========== */
  float y = z_rpm - x0;                 /* innovation (before R adapt) */
  float R = rpmKalmanAdaptR(z_rpm, y, p00);
  float S = p00 + R;
  if (S < 1.0f)
    S = 1.0f;

  /* Kalman gain K = P⁻ Hᵀ / S ,  Hᵀ = [1; 0] → K = [p00; p10] / S */
  float k0 = p00 / S;
  float k1 = p10 / S;
  /* Clamp gains for numerical safety */
  if (k0 < 0.0f) k0 = 0.0f;
  if (k0 > 1.0f) k0 = 1.0f;
  if (k1 >  50.0f) k1 =  50.0f;
  if (k1 < -50.0f) k1 = -50.0f;

  /* x̂ = x̂⁻ + K y */
  x0 += k0 * y;
  x1 += k1 * y;

  /* Joseph form: P = (I-KH) P⁻ (I-KH)ᵀ + K R Kᵀ
   * I-KH = [ 1-k0   0 ]
   *        [  -k1   1 ]
   */
  float a00 = 1.0f - k0;
  float a10 = -k1;
  /* A = I-KH;  AP = A * P⁻ */
  float ap00 = a00 * p00;
  float ap01 = a00 * p01;
  float ap10 = a10 * p00 + p10;
  float ap11 = a10 * p01 + p11;
  /* (AP) Aᵀ  +  K R Kᵀ */
  float p00n = ap00 * a00 + k0 * k0 * R;
  float p01n = ap01 + k0 * k1 * R;          /* ap01 * 1 + … */
  float p10n = ap10 * a00 + k1 * k0 * R;
  float p11n = ap11 + k1 * k1 * R;

  /* Symmetry + positive-definite floors */
  p01n = 0.5f * (p01n + p10n);
  p10n = p01n;
  if (p00n < 1.0f)   p00n = 1.0f;
  if (p11n < 1.0f)   p11n = 1.0f;
  if (p00n > 1.0e6f) p00n = 1.0e6f;
  if (p11n > 1.0e7f) p11n = 1.0e7f;

  /* Commit state */
  if (x0 < 0.0f)      x0 = 0.0f;
  if (x0 > 15000.0f)  x0 = 15000.0f;
  if (x1 >  25000.0f) x1 =  25000.0f;
  if (x1 < -25000.0f) x1 = -25000.0f;

  kf_rpm = x0;
  kf_acc = x1;
  kf_p00 = p00n;
  kf_p01 = p01n;
  kf_p10 = p10n;
  kf_p11 = p11n;

  return (uint16_t)(kf_rpm + 0.5f);
}



/**
 * Schedule complementary α (weight on fast/Kalman path).
 *
 * Raises α when:
 *  - filters agree (low |fast - slow|)
 *  - locked and mid/high RPM
 *  - high |accel| (need response - still keep some slow bias)
 * Lowers α when:
 *  - cranking / unlocked
 *  - large disagreement (trust rev more to pull bias out)
 *  - high tooth error rate
 */
float rpmAlphaSchedule(float rpm_fast, float rpm_slow)
{
  float alpha = 0.80f; /* baseline when locked & calm */

  /* Sync state */
  if (!syncLocked)
    alpha = 0.65f;
  else if (gapConfirm < 2)
    alpha = 0.72f;

  /* RPM band */
  float rpm_ref = rpm_fast;
  if (rpm_slow > 1.0f)
    rpm_ref = 0.5f * (rpm_fast + rpm_slow);
  if (rpm_ref < 400.0f)
    alpha -= 0.20f;       /* heavy rev weight while cranking */
  else if (rpm_ref < 900.0f)
    alpha -= 0.10f;
  else if (rpm_ref > 4500.0f)
    alpha += 0.05f;       /* tooth path cleaner at high RPM */

  /* Agreement: |fast - slow| / mean */
  float mean = 0.5f * (rpm_fast + rpm_slow);
  if (mean < 100.0f)
    mean = 100.0f;
  float rel = (rpm_fast - rpm_slow);
  if (rel < 0.0f) rel = -rel;
  rel /= mean;
  if (rel < 0.02f)
    alpha += 0.08f;       /* excellent agreement */
  else if (rel < 0.05f)
    alpha += 0.03f;
  else if (rel > 0.15f)
    alpha -= 0.12f;       /* disagree - trust rev */
  else if (rel > 0.08f)
    alpha -= 0.06f;

  /* Transient: |accel| high → prefer fast path */
  float a = kf_acc;
  if (a < 0.0f) a = -a;
  if (a > 3000.0f)
    alpha += 0.06f;
  else if (a > 1500.0f)
    alpha += 0.03f;

  /* Tooth errors → more conservative (rev) */
  if (toothErrors > 20 && toothErrors > (syncLosses + 5))
    alpha -= 0.08f;

  /* Clamp usable range */
  if (alpha < 0.40f) alpha = 0.40f;
  if (alpha > 0.95f) alpha = 0.95f;
  return alpha;
}

/**
 * Complementary blend of fast (tooth/Kalman) and slow (rev) RPM.
 * alpha = weight on fast path (0..1). Typical 0.75-0.90 when locked.
 */
uint16_t rpmComplementaryBlend(float rpm_fast, float rpm_slow, float alpha)
{
  if (alpha < 0.0f) alpha = 0.0f;
  if (alpha > 1.0f) alpha = 1.0f;
  if (rpm_fast < 0.0f) rpm_fast = 0.0f;
  if (rpm_slow < 0.0f) rpm_slow = 0.0f;
  if (rpm_fast > 15000.0f) rpm_fast = 15000.0f;
  if (rpm_slow > 15000.0f) rpm_slow = 15000.0f;
  float y = alpha * rpm_fast + (1.0f - alpha) * rpm_slow;
  if (y < 0.0f) y = 0.0f;
  if (y > 15000.0f) y = 15000.0f;
  /* Keep Kalman state consistent with blended output */
  kf_rpm = y;
  return (uint16_t)(y + 0.5f);
}

/* ── Crank ──────────────────────────────────────────────────── */
void ECU_CrankCapture(uint32_t capt) {
  /* capt = TIM5 CNT at edge (timer @ 1 MHz → us). Prefer hardware stamp. */
  static uint32_t lastCapt = 0;
    uint32_t now = micros();
  uint32_t dt;

  crankEdgeCount++; /* count every IRQ for diagnostics */

  if (lastCapt == 0) {
    lastCapt = capt;
    lastToothUs = now;
    return;
  }

  uint32_t prevCapt = lastCapt;
  dt = capt - lastCapt; /* 32-bit free-running TIM5 */
  lastCapt = capt;

  /* --- SEEKING: always publish RPM from any plausible interval --- */
  if (!syncLocked) {
    if (dt < 40UL) {
      toothErrors++;
      lastCapt = prevCapt;
      return;
    }
    if (dt > 2000000UL) {
      lastCapt = capt;
      lastToothUs = now;
      toothPeriodUs = 0;
      toothPeriodFilt = 0;
      return;
    }
    lastToothUs = now;
    if (toothPeriodUs == 0 || toothPeriodFilt == 0) {
      toothPeriodUs = dt;
      toothPeriodFilt = dt;
    } else {
      toothPeriodFilt = (toothPeriodFilt + dt) / 2UL;
      toothPeriodUs = toothPeriodFilt;
    }
    if (gTeeth < 2 || gTeeth > 60) gTeeth = 60;
    {
      uint32_t den = toothPeriodFilt * (uint32_t)gTeeth;
      if (den == 0) den = 1;
      uint32_t z = 60000000UL / den;
      if (z > 15000UL) z = 15000UL;
      if (z >= 30UL) {
        if (rpmLive < 30)
          rpmLive = (uint16_t)z;
        else
          rpmLive = (uint16_t)((rpmLive * 3u + (uint16_t)z) / 4u);
        kf_rpm = (float)rpmLive;
      }
    }
    /* fall through for gap/lock acquisition */
  }

  /* locked: soft debounce */
  if (syncLocked) {
    uint32_t minD = toothPeriodFilt ? (toothPeriodFilt / 10UL) : 50UL;
    if (minD < 40) minD = 40;
    if (dt < minD) {
      toothErrors++;
      lastCapt = prevCapt;
      return;
    }
  }

  if (toothPeriodUs == 0) {
    if (dt < 40 || dt > 800000UL) return;
    lastToothUs = now;
    toothPeriodUs = dt;
    toothPeriodFilt = dt;
    if (gTeeth < 2 || gTeeth > 60) gTeeth = 60;
    if (gTeeth >= 2) {
      float z = 60000000.0f / ((float)dt * (float)gTeeth);
      if (z > 15000.0f) z = 15000.0f;
      if (z >= 30.0f) {
        rpmLive = (uint16_t)(z + 0.5f);
        kf_rpm = z;
      }
    }
    return;
  }

uint32_t T = toothPeriodUs;
  uint8_t miss = gMissing; /* 0 = no missing tooth (even wheel) */
  uint8_t phys = (gTeeth > miss) ? (uint8_t)(gTeeth - miss) : gTeeth;
  if (phys < 2) phys = 2;

  /* Noise reject: only drop obvious double-hits; keep low-RPM edges */
  uint32_t minTooth = (T * 30UL) / 100UL;
  if (minTooth < 40) minTooth = 40;
  if (dt < minTooth) { toothErrors++; return; }

  lastToothUs = now;

  /* Pre-classify likely gap so Kalman is not fed a half-speed sample */
  uint8_t likelyGap = 0;
  if (miss >= 1 && T > 0) {
    uint32_t gapNom = T * (uint32_t)(miss + 1);
    if (dt > (T + T / 2UL) && dt > (gapNom / 3UL))
      likelyGap = 1;
  }

  /* Kalman RPM + light period assist (skip gap intervals) */
  if (dt > 40 && gTeeth >= 2) {
    uint32_t Tf = toothPeriodFilt ? toothPeriodFilt : T;

    /* Outlier reject when locked: drop edges <55% of filtered period (noise). */
    if (syncLocked && miss >= 1 && Tf > 0) {
      uint32_t lo = (Tf * 55UL) / 100UL;
      if (dt < lo) {
        toothErrors++;
        return;
      }
    }

    if (!likelyGap) {
      float z_rpm = 60000000.0f / ((float)dt * (float)gTeeth);
      if (z_rpm > 15000.0f) z_rpm = 15000.0f;
      float dt_s = (float)dt * 1.0e-6f;
      rpmLive = rpmKalmanUpdate(z_rpm, dt_s);

      if (rpmLive >= 30 && gTeeth >= 2) {
        uint32_t per = 60000000UL / ((uint32_t)rpmLive * (uint32_t)gTeeth);
        if (per < 40UL) per = 40UL;
        if (per > 800000UL) per = 800000UL;
        toothPeriodFilt = (per * 3UL + dt) / 4UL;
        toothPeriodUs = toothPeriodFilt;
      } else {
        toothPeriodFilt = (Tf * 3UL + dt) / 4UL;
        toothPeriodUs = toothPeriodFilt;
        if (toothPeriodUs < 40) toothPeriodUs = 40;
      }
    }
    /* On gap: leave Kalman state; gap handler updates period from gap/n */
  }

  /* ── Missing-tooth gap detection ─────────────────────────────
   * Nominal gap time = (missing+1) * toothPeriod
   * e.g. 36-1 → 2×T, 60-2 → 3×T
   * Use ratio dt/T with adaptive windows; validate tooth count between gaps.
   */
  uint8_t isGap = 0;
  if (miss >= 1 && T >= 40) {
    uint32_t gapMul = (uint32_t)(miss + 1); /* tooth periods spanned by gap */
    uint32_t gapNom = T * gapMul;
    /* ±% of nominal gap: wider when unlocking / low RPM */
    /* Wider windows at high RPM — period jitter grows with noise/ISR load */
    uint32_t tolLo, tolHi;
    if (!syncLocked || rpmLive < 800) { tolLo = 50; tolHi = 80; }
    else if (rpmLive < 2000) { tolLo = 40; tolHi = 55; }
    else if (rpmLive < 4500) { tolLo = 45; tolHi = 70; } /* harden >2k */
    else { tolLo = 50; tolHi = 85; }
    uint32_t gapMin = (gapNom * (100UL - tolLo)) / 100UL;
    uint32_t gapMax = (gapNom * (100UL + tolHi)) / 100UL;
    /* Must be longer than a normal tooth */
    if (gapMin < T + (T / 5UL))
      gapMin = T + (T / 5UL);
    if (gapMax < gapMin + T)
      gapMax = gapMin + T * 2UL;

    if (dt >= gapMin && dt <= gapMax) {
      /* Tooth-count check: between gaps expect ~phys teeth (gTeeth - missing) */
      uint8_t countOk = 1;
      if (syncLocked && lastGapUs != 0 && teethSinceGap > 0) {
        /* Allow ±3 teeth (jitter / partial miss) once locked */
        int16_t expect = (int16_t)phys;
        int16_t got = (int16_t)teethSinceGap;
        int16_t err = got - expect;
        if (err < 0) err = (int16_t)(-err);
        if (err > 3 && gapConfirm >= 2) {
          /* Suspicious - may still accept first few gaps */
          countOk = 0;
          toothErrors++;
        }
      }
      /* Refuse a second "gap" too soon (< half a rev of teeth) */
      if (lastGapUs != 0 && teethSinceGap < (phys / 3) && syncLocked)
        countOk = 0;

      if (countOk) {
        isGap = 1;
        gapRejectStreak = 0;
      } else {
        if (gapRejectStreak < 255) gapRejectStreak++;
        /* If many rejects, loosen and accept pure timing gap */
        if (gapRejectStreak >= 4)
          isGap = 1;
      }
    }
  }

  if (isGap) {
    if (lastGapUs != 0) {
      uint32_t revUs = now - lastGapUs;
      if (revUs >= 3000UL && revUs <= 2000000UL) {
        float z_rev = 60000000.0f / (float)revUs;
        if (z_rev > 15000.0f) z_rev = 15000.0f;
        /* 1) Kalman update with full-rev measurement */
        rpmLive = rpmKalmanUpdate(z_rev, (float)revUs * 1.0e-6f);
        /* 2) Complementary blend with scheduled α */
        {
          float alpha = rpmAlphaSchedule(kf_rpm, z_rev);
          rpmLive = rpmComplementaryBlend(kf_rpm, z_rev, alpha);
        }
        uint32_t tRev = revUs / (uint32_t)gTeeth;
        if (tRev >= 40 && tRev <= 80000UL) {
          toothPeriodUs = (toothPeriodUs * 3UL + tRev) / 4UL;
          toothPeriodFilt = toothPeriodUs;
        }
      }
    }
    lastGapUs = now;
    toothIndex = 0;
    teethSinceGap = 0;
    /* Angle at gap = TDC reference (0° or 360° half) */
    crankDeg = (injSequentialActive() || ignSequentialActive()) ? (cycleHalf ? 360.0f : 0.0f) : 0.0f;
    for (uint8_t i = 1; i <= gCyl && i <= MAX_CYL; i++) {
      coilFired[i] = 0;
    }
    pllSoftErr = 0;
    pllGoodStreak++;
    if (gapConfirm < 255)
      gapConfirm++;
    /* Require 2 confirmed gaps before hard lock (except pure cranking assist) */
    if (gapConfirm >= 2 || (rpmLive > 0 && rpmLive < 400 && gapConfirm >= 1))
      syncLocked = 1;
    else if (!syncLocked && gapConfirm >= 1 && rpmLive < 600)
      syncLocked = 1; /* soft lock for cranking */

    /* Infer tooth period from gap span (miss+1 tooth times) */
    uint32_t tGap = dt / (uint32_t)(miss + 1);
    if (tGap >= 40 && tGap <= 200000UL) {
      toothPeriodUs = (toothPeriodUs * 2UL + tGap) / 3UL;
      toothPeriodFilt = toothPeriodUs;
    }

    /* Sequential+cam: request by true 720° TDC half
     * cyl1 TDC@0 → inj window in 360-720 half (prior rev / intake)
     * Use EOI-based half, not injAt==360 boundary bug */
    if (injSequentialActive()) {
      /* Full sequential - one injector per EOI half of 720° */
      for (uint8_t s = 0; s < gCyl; s++) {
        uint8_t cyl = cylAtSlot(s);
        float tdc = tdcDeg(cyl);
        float eoi = tdc - gEoiBtdc;
        while (eoi < 0.0f) eoi += 720.0f;
        while (eoi >= 720.0f) eoi -= 720.0f;
        uint8_t eoiHalf = (eoi >= 360.0f) ? 1u : 0u;
        if (eoiHalf == cycleHalf)
          injReq[cyl] = 1;
      }
    } else {
      /* Batch: do NOT set injReq — angle SOI in serviceInjection only.
       * Setting injReq here caused double-fire with the angle path. */
    }
    /* Advance 720° half every crank gap (2 gaps per cam cycle) */
    cycleHalf ^= 1u;
  } else if (miss >= 1 && dt > (T * (uint32_t)(miss + 1) + T)) {
    /* Oversize interval but outside gap window - noise or partial sync */
    toothErrors++;
    toothIndex++;
    if (teethSinceGap < 60000U)
      teethSinceGap++;
    /* If we expected a gap soon and got a huge interval, force resync hint */
    if (syncLocked && teethSinceGap >= (uint16_t)phys) {
      if (++pllSoftErr >= 40) {
        syncLocked = 0;
        gapConfirm = 0;
        syncLosses++;
        toothIndex = 0;
        teethSinceGap = 0;
        pllSoftErr = 0;
      }
    }
  } else {
    /* Normal tooth */
    toothIndex++;
    if (teethSinceGap < 60000U)
      teethSinceGap++;
    float degPer = 360.0f / (float)((gTeeth > 0) ? gTeeth : 36);
    float base = (camSynced && CFG_SEQUENTIAL && cycleHalf) ? 360.0f : 0.0f;
    crankDeg = base + (float)toothIndex * degPer;
    /* Must wrap fully — single subtract fails when toothIndex >> phys */
    if (!(injSequentialActive() || ignSequentialActive())) {
      while (crankDeg >= 360.0f) crankDeg -= 360.0f;
      while (crankDeg < 0.0f) crankDeg += 360.0f;
    } else {
      while (crankDeg >= 720.0f) crankDeg -= 720.0f;
      while (crankDeg < 0.0f) crankDeg += 720.0f;
    }

    /* Soft-lock: enough clean teeth → allow spark/fuel even before first gap */
    {
      /* Cranking: soft-lock after ~6 clean teeth so spark works below 300 RPM */
      uint16_t needSoft = 6;
      if (phys > 12) needSoft = 8;
      if (rpmLive >= 400) needSoft = (phys > 6) ? (uint16_t)(phys / 2) : (uint16_t)phys;
      if (!syncLocked && toothIndex >= needSoft) {
        syncLocked = 1;
        pllGoodStreak++;
        for (uint8_t i = 1; i <= gCyl && i <= MAX_CYL; i++) {
          coilFired[i] = 0;
          injFiredCyc[i] = 0;
        }
      }
    }

    if (syncLocked) {
      /* Synthetic gap: keep angle domain valid when real gap not yet seen.
       * Without this, soft-lock below ~300 RPM never resets toothIndex →
       * crankDeg drifts and coils never see a fire window. */
      if (miss >= 1 && teethSinceGap >= (uint16_t)phys) {
        toothIndex = 0;
        teethSinceGap = 0;
        cycleHalf ^= 1u;
        for (uint8_t i = 1; i <= gCyl && i <= MAX_CYL; i++)
          coilFired[i] = 0;
        crankDeg = (injSequentialActive() || ignSequentialActive())
                     ? (cycleHalf ? 360.0f : 0.0f) : 0.0f;
        /* Do not bump gapConfirm — this is not a verified missing-tooth */
      }
      if (miss >= 1 && toothIndex > (uint16_t)phys + ((rpmLive > 2000) ? 96U : 48U)) {
        if (++pllSoftErr >= ((rpmLive > 2000) ? 160 : 80)) {
          /* Only unlock after sustained bad tooth count (higher bar above 2k) */
          syncLocked = 0;
          syncLosses++;
          toothIndex = 0;
          pllSoftErr = 0;
        }
      } else {
        if (pllSoftErr) pllSoftErr--;
      }
      /* Even wheel: roll index every full set of teeth */
      if (miss == 0 && toothIndex >= gTeeth) {
        toothIndex = 0;
        cycleHalf ^= 1;
        for (uint8_t i = 1; i <= gCyl && i <= MAX_CYL; i++) {
          coilFired[i] = 0;
          /* no injReq prime for batch */
        }
      }
    }
  }
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
