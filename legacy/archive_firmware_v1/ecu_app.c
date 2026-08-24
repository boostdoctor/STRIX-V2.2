/**
 * TorquEFI Basic - STM32F411 full sequential ECU core
 * Cam (PA15) + crank (PA0) → 720° phase; per-cylinder spark & inject.
 */
#include "ecu_app.h"
#include "ecu_goertzel.h"
#include "ecu_wheels.h"
#include "ecu_config.h"
#include "ecu_pins.h"
#include "ecu_serial.h"
#include "ecu_flash.h"
#include "ecu_features.h"
#include "stm32f4xx_hal.h"
#include <string.h>
#include <stdio.h>
#include <math.h>

#define ROWS 15
#define COLS 22
#define MAX_CYL CFG_MAX_COILS

/* 22-column RPM axis (matches tuner: 250 + c*375, 250-8125 RPM) */
static const float rpmBins[COLS] = {
  250,  625, 1000, 1375, 1750, 2125, 2500, 2875,
 3250, 3625, 4000, 4375, 4750, 5125, 5500, 5875,
 6250, 6625, 7000, 7375, 7750, 8125
};
/* 15-row load axis (0.10-1.08 normalised) */
static const float mapBins[ROWS] = {
  0.100f, 0.170f, 0.240f, 0.310f, 0.380f, 0.450f, 0.520f, 0.590f,
  0.660f, 0.730f, 0.800f, 0.870f, 0.940f, 1.010f, 1.080f
};

static int8_t  advMap[ROWS][COLS];
static uint8_t injMap[ROWS][COLS];
/* Per-cylinder fuel trim % (-25..+25), index 1..MAX_CYL */
static float cylTrimPct[MAX_CYL + 1];
/* Live breakpoints (overwritten by tuner SET:RPMB / SET:MAPB) */
static float rpmBinsLive[COLS];
static float mapBinsLive[ROWS];
/* Pedal→throttle target map: 16 RPM bands × 17 pedal points */
#define ETB_ROWS 16
#define ETB_COLS 17
static uint8_t etbMap[ETB_ROWS][ETB_COLS];
static float engLoad = 0.0f;
static uint8_t mapCellR = 0, mapCellC = 0; /* last lookup cell */
static int8_t  baseAdvDeg = 0;   /* map only, pre-retard */
static float   baseInjMs  = 0;   /* map only, pre-trim */
static uint8_t sensorPhase = 0;


static volatile uint8_t  gTeeth = CFG_TEETH;
static volatile uint8_t  gMissing = CFG_MISSING;
static volatile uint16_t gTrigAngle = CFG_TRIG_ANGLE;
static volatile uint16_t gRpmLimit = CFG_RPM_LIMIT;
static volatile uint8_t  gRpmCutMode = 0; /* 0=hard cut, 1=soft cut */
static volatile uint8_t  rpmCutActive = 0;

/* DFCO - declared early for serviceInjection */
static uint8_t  dfcoEnable   = 1;
static uint8_t  dfcoActive   = 0;
static uint16_t dfcoEnterRpm = 1600;
static uint16_t dfcoExitRpm  = 1200;
static float    dfcoMaxTps   = 3.0f;
static float    dfcoMinEct   = 50.0f;
static uint32_t dfcoEnterMs  = 0;
static uint16_t dfcoDelayMs  = 200;
static volatile uint8_t  gUseTps = CFG_LOAD_ALPHA_N;
static volatile uint8_t  gCyl = CFG_CYLINDERS;
static volatile uint8_t  gCoilSmart = 1;  /* 1=smart 0=dumb */
static volatile uint8_t  gDbwEnable = 1;  /* 0=idle actuator only */
static volatile uint8_t  gIdleOutMode = 0; /* 0=2wire 1=1wire 2=stepper */
static volatile uint8_t  gFireOrder = 0;  /* 0=1-3-4-2 1=1-2-4-3 2=1-3-2-4 */
#define BAT_CAL_N 15
static float batVoltTbl[BAT_CAL_N];
static float batAdcTbl[BAT_CAL_N];
static float batCompTbl[BAT_CAL_N];
static uint8_t batCalReady = 0;
#define MAP_CAL_N 15
static float mapCalKpa[MAP_CAL_N];
static float mapCalAdc[MAP_CAL_N];
static uint8_t mapCalReady = 0;
/* Cold-start fuel enrichment vs ECT (°C → extra % fuel) */
#define CSE_N 10
static float cseTemp[CSE_N] = {-20,0,10,20,30,40,50,60,70,80};
static float csePct[CSE_N]  = {80,55,40,28,18,12,7,3,0,0}; /* % add */
/* After-start enrichment: extra % that decays to 0 over time */
static float aseInitialPct = 35.0f;   /* % extra at start */
static float aseDecaySec   = 25.0f;   /* seconds to decay to 0 */
static float aseMinEct     = 60.0f;   /* skip ASE if already warm */
static uint32_t aseStartMs = 0;
static uint8_t  aseActive  = 0;
static uint8_t  wasRunning = 0;
/* Injection mode: 0=AUTO 1=BATCH 2=SEQUENTIAL 3=HYBRID (seq below RPM, batch above) */
static volatile uint8_t  gInjMode = 0;
static volatile uint16_t gBatchAboveRpm = 3000; /* hybrid switch point */

/* Crank / cam */
static volatile uint32_t lastToothUs = 0, lastGapUs = 0;
static volatile uint16_t toothIndex = 0;
static volatile uint8_t  syncLocked = 0, camSynced = 0, cam2Synced = 0;
static float  cam1PhaseDeg = 0.0f;
static float  cam2PhaseDeg = 0.0f;
static volatile uint16_t syncLosses = 0, toothErrors = 0;
static volatile uint32_t toothPeriodUs = 0;
static volatile uint32_t toothPeriodFilt = 0; /* EMA-smoothed tooth us */
/* 2-state Kalman filter for RPM: x=[rpm, accel(rpm/s)] */
static float kf_rpm = 0.0f;
static float kf_acc = 0.0f;
static float kf_p00 = 1.0e4f, kf_p01 = 0.0f, kf_p10 = 0.0f, kf_p11 = 1.0e4f;
static uint8_t kf_ready = 0;
static float kf_nis_ema = 1.0f;   /* EMA of normalized innovation² */
static float kf_R_adapt = 150.0f; /* adapted measurement noise */
static float kf_q_adapt = 12000.0f; /* adapted accel spectral density */
static volatile uint32_t lastCamEdgeUs = 0;   /* for cam timeout */
static volatile uint32_t lastCam2EdgeUs = 0;
static volatile uint8_t  camLockHits = 0;     /* hysteresis to lock */
static volatile uint8_t  camUnlockMiss = 0;   /* hysteresis to unlock */
static volatile uint8_t  cam2LockHits = 0;
static volatile uint8_t  cam2UnlockMiss = 0;
static volatile uint16_t rpmLive = 0;
static uint8_t gWheelId = 6; /* default 36-1 */
static volatile uint8_t mapDumpBusy = 0;
static uint8_t gCamMode = 0;

/* Bulk-upload state (UPLOAD:ADV / UPLOAD:INJ from tuner) */
static uint8_t uploadMode = 0;   /* 0=idle  1=ADV  2=INJ */
static volatile uint8_t savePending = 0; /* 1 = do flash in ECU_Loop */
static volatile uint8_t mapsDirty  = 0; /* RAM maps changed since load/save */
static volatile int8_t  saveLastErr = 0;

static uint8_t uploadRow  = 0;
static volatile uint32_t crankEdgeCount = 0;
static volatile float    crankDeg = 0.0f;   /* 0..720 when sequential */
static volatile uint8_t  cycleHalf = 0;     /* 0 or 1 from cam */
static volatile uint8_t  pllSoftErr = 0, pllGoodStreak = 0;
static volatile uint8_t  gapConfirm = 0;
static volatile uint16_t teethSinceGap = 0; /* physical teeth since last gap */
static volatile uint8_t  gapRejectStreak = 0;

static volatile int16_t  ignAdvanceDeg = 10; /* signed: negative = ATDC */
static float softLimitRetardDeg = 0.0f;
static float totalRetardDeg = 0.0f;
static int16_t advTargetDeg = 10;
static float advSlewDps = 200.0f; /* max °/s change of final advance */
static float gIgnMinAdv = -15.0f;
static float gIgnMaxAdv = 45.0f;
static volatile uint16_t injPwUs = 2000;
static volatile uint16_t dwellTargetUs = CFG_DWELL_NOM_US;
static volatile uint16_t dwellActualUs = 0;

/* Per-cylinder coil / injector */
static volatile uint8_t  coilState[MAX_CYL+1];
static volatile uint32_t coilStartUs[MAX_CYL+1];
static volatile uint8_t  coilFired[MAX_CYL+1];
static volatile uint8_t  injReq[MAX_CYL+1];
static uint8_t injFiredCyc[MAX_CYL+1];
#ifndef CFG_EOI_BTDC_DEG
#define CFG_EOI_BTDC_DEG 60.0f
#endif
static float gEoiBtdc = CFG_EOI_BTDC_DEG;
static uint8_t  injOn[MAX_CYL+1];
static uint32_t injEndUs[MAX_CYL+1];

static uint8_t fanOn = 0, fpOn = 0;
static float gFanOnC  = 95.0f;
static float gFanOffC = 90.0f; /* hysteresis: off below on-hyst */
/* VVT duty 0-100% via TIM1 PWM (period 1000 counts) */
static uint8_t vvt1Duty = 0, vvt2Duty = 0;
extern TIM_HandleTypeDef htim1;
/* Closed-loop ETB */
static float etbTargetPct = -1.0f;  /* <0 = follow pedal; 0-100 = override */
static float etbIntegral  = 0.0f;
static float etbPrevErr   = 0.0f;
static uint8_t etbEnable = 1;
/* PID gains - tune on vehicle */
static float ETB_KP = 4.0f;
static float ETB_KI = 8.0f;
static float ETB_KD = 0.05f;
static float ETB_IDLE_PCT = 3.0f;  /* min open when running */

static float engMap, engTps, engEct, engIat, engBat, engO2, engKnock, engPedal;
/* Goertzel knock */
#define KNK_WIN_N     64
#define KNK_FS_HZ     50000.0f
#define KNK_F1_HZ     7000.0f
#define KNK_F2_HZ     10000.0f
static float    knkBuf[KNK_WIN_N];
static uint16_t knkIdx = 0;
static uint8_t  knkCollecting = 0;
static float    knkIntensity = 0.0f;
static float    knkThreshold = 50.0f;  /* scale depends on sensor gain */
static float    knockRetardDeg = 0.0f;
static float    knkStepDeg = 2.0f;
static float    knkRestoreDps = 5.0f;  /* degrees per second restore */
static float    knkMaxRetard = 12.0f;
static uint8_t  knkEnable = 1;
static uint16_t adcMap, adcTps, adcEct, adcIat, adcBat, adcO2, adcKnock, adcPedal;
/* TPS / pedal endpoint calibration (12-bit ADC) */
static uint16_t tpsClosedAdc  = 400;
static uint16_t tpsOpenAdc    = 3600;
static uint16_t pedClosedAdc  = 400;
static uint16_t pedOpenAdc    = 3600;
static uint8_t  tpsCalValid   = 0;


static char rxBuf[192];
static uint8_t rxLen = 0;

static int clampi(int a, int lo, int hi) {
  if (a < lo) return lo;
  if (a > hi) return hi;
  return a;
}

static inline uint32_t micros(void) {
  return DWT->CYCCNT / (SystemCoreClock / 1000000U);
}
static inline uint32_t millis(void) { return HAL_GetTick(); }

static int8_t clampAdv(int v) { return (int8_t)clampi(v, -10, 45); }
static uint8_t clampInj(float ms) {
  if (ms < 0) ms = 0;
  if (ms > 20) ms = 20;
  return (uint8_t)(ms * 10.0f + 0.5f);
}

/** Parse int from string; returns chars consumed or 0 on failure */
static int parse_int(const char *s, int *out)
{
  const char *p = s;
  int sign = 1, v = 0, digits = 0;
  while (*p == ' ') p++;
  if (*p == '-') { sign = -1; p++; }
  else if (*p == '+') p++;
  while (*p >= '0' && *p <= '9') {
    v = v * 10 + (*p - '0');
    p++;
    digits++;
  }
  if (!digits) return 0;
  *out = sign * v;
  return (int)(p - s);
}

/** Parse float without scanf %f (newlib-nano often lacks it). */
static int parse_float(const char *s, float *out)
{
  const char *p = s;
  int sign = 1, digits = 0;
  float v = 0.0f, frac = 0.1f;
  while (*p == ' ') p++;
  if (*p == '-') { sign = -1; p++; }
  else if (*p == '+') p++;
  while (*p >= '0' && *p <= '9') {
    v = v * 10.0f + (float)(*p - '0');
    p++;
    digits++;
  }
  if (*p == '.') {
    p++;
    while (*p >= '0' && *p <= '9') {
      v += frac * (float)(*p - '0');
      frac *= 0.1f;
      p++;
      digits++;
    }
  }
  if (!digits) return 0;
  *out = (float)sign * v;
  return (int)(p - s);
}


static void defaultMaps(void) {
  for (uint8_t r = 0; r < ROWS; r++)
    for (uint8_t c = 0; c < COLS; c++) {
      advMap[r][c] = clampAdv(10 + r + c);
      injMap[r][c] = clampInj(2.0f + r * 0.4f + c * 0.5f);
    }
}


/* INJ1 on PB15 (free pin; avoids PB4 NJTRST). Init all inj pins as GPIO. */
static void ecuInjGpioInit(void)
{
  GPIO_InitTypeDef g = {0};
  __HAL_RCC_GPIOB_CLK_ENABLE();
  g.Mode  = GPIO_MODE_OUTPUT_PP;
  g.Pull  = GPIO_NOPULL;
  g.Speed = GPIO_SPEED_FREQ_HIGH;
  g.Pin   = INJ1_Pin | INJ2_Pin | INJ3_Pin | INJ4_Pin;
#if defined(INJ5_Pin)
  g.Pin  |= INJ5_Pin | INJ6_Pin;
#endif
  HAL_GPIO_Init(INJ1_GPIO_Port, &g);
  HAL_GPIO_WritePin(INJ1_GPIO_Port, g.Pin, GPIO_PIN_RESET);
}

static void allOutputsOff(void) {
  for (uint8_t i = 1; i <= MAX_CYL; i++) {
    ECU_IGN_LO(i);
    ECU_INJ_LO(i);
    coilState[i] = 0;
    injOn[i] = 0;
    injReq[i] = 0;
  }
  ECU_FAN_LO();
  ECU_FP_LO();
  vvt1Duty = vvt2Duty = 0;
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
}

/* Firing order 1-3-4-2 for 4-cyl; 1-5-3-6-2-4 for 6-cyl */
static const uint8_t order4[4] = {1, 3, 4, 2};
static const uint8_t order6[6] = {1, 5, 3, 6, 2, 4};

static uint8_t cylAtSlot(uint8_t slot) {
  if (gCyl <= 4) return order4[slot % 4];
  return order6[slot % 6];
}

/* TDC angles ° on 720° cycle for each cylinder (cyl 1 at 0) */
static float tdcDeg(uint8_t cyl) {
  if (gCyl <= 4) {
    switch (cyl) {
      case 1: return 0.0f;
      case 3: return 180.0f;
      case 4: return 360.0f;
      case 2: return 540.0f;
      default: return 0.0f;
    }
  }
  /* 6-cyl even fire every 120° */
  static const float t6[7] = {0, 0, 480, 240, 600, 120, 360};
  return (cyl >= 1 && cyl <= 6) ? t6[cyl] : 0.0f;
}


/* Effective sequential? AUTO uses cam; mode 2 forces seq when cam locked */
static uint8_t injSequentialActive(void)
{
  if (gInjMode == 1) return 0; /* force batch */
  if (gInjMode == 2) return camSynced ? 1u : 0u;
  if (gInjMode == 3) {
    /* HYBRID: sequential below threshold (needs CAM), batch at/above */
    if (!camSynced) return 0;
    if (rpmLive >= gBatchAboveRpm) return 0;
    return 1;
  }
  /* AUTO: sequential whenever CAM locked */
  return (CFG_SEQUENTIAL && camSynced) ? 1u : 0u;
}

/* ── Cam (720° phase) ───────────────────────────────────────── */
void ECU_CamCapture(uint32_t capt) {
  (void)capt;
  static uint32_t lastCamUs = 0;
  uint32_t now = micros();

  /* Debounce: ignore edges closer than ~8 ms (noise / bounce) */
  if (lastCamUs && (now - lastCamUs) < 8000UL)
    return;

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
      cycleHalf = 0; /* phase reference */
      camLockHits = 0;
    }
  } else {
    /* Stay locked; phase nudge only */
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

/* htim3 defined in tim.c */
extern TIM_HandleTypeDef htim3;


/** Reset RPM Kalman (stall / first edge). */
static void rpmKalmanReset(void)
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
static float rpmKalmanAdaptQ(float dt_s)
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
static float rpmKalmanAdaptR(float z_rpm, float y, float p00)
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
static uint16_t rpmKalmanUpdate(float z_rpm, float dt_s)
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
static float rpmAlphaSchedule(float rpm_fast, float rpm_slow)
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
static uint16_t rpmComplementaryBlend(float rpm_fast, float rpm_slow, float alpha)
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
  dt = capt - lastCapt; /* 32-bit free-running TIM5 */
  lastCapt = capt;

  /* Reject only extreme double-edges (<40 us ≈ 25 kHz) */
  if (dt < 40) {
    toothErrors++;
    return;
  }

  if (toothPeriodUs == 0) {
    /* First valid period: accept very wide range (slow crank → high RPM) */
    if (dt < 40 || dt > 800000UL) return;
    lastToothUs = now;
    toothPeriodUs = dt;
    toothPeriodFilt = dt;
    if (gTeeth >= 2) {
      float z = 60000000.0f / ((float)dt * (float)gTeeth);
      if (z > 15000.0f) z = 15000.0f;
      rpmKalmanReset();
      rpmLive = rpmKalmanUpdate(z, (float)dt * 1.0e-6f);
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
    uint32_t tolLo = (!syncLocked || rpmLive < 800) ? 50UL : 35UL;
    uint32_t tolHi = (!syncLocked || rpmLive < 800) ? 80UL : 45UL;
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
    crankDeg = injSequentialActive() ? (cycleHalf ? 360.0f : 0.0f) : 0.0f;
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
      /* Batch / semi-seq on 360°: banks 1+4 and 2+3 */
      injReq[1] = 1;
      injReq[4] = 1;
      if (cycleHalf) {
        injReq[2] = 1;
        injReq[3] = 1;
      }
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
    if (crankDeg >= 360.0f && !injSequentialActive())
      crankDeg -= 360.0f;

    /* Soft-lock: enough clean teeth → allow spark/fuel even before first gap */
    if (!syncLocked && toothIndex >= (phys > 6 ? phys / 2 : phys)) {
      syncLocked = 1;
      pllGoodStreak++;
      for (uint8_t i = 1; i <= gCyl && i <= MAX_CYL; i++)
        coilFired[i] = 0;
      /* Prime injectors once soft-locked */
      injReq[1] = injReq[4] = 1;
      injReq[2] = injReq[3] = 1;
    }

    if (syncLocked) {
      if (miss >= 1 && toothIndex > (uint16_t)phys + 48U) {
        if (++pllSoftErr >= 80) {
          /* Only unlock after sustained bad tooth count */
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
        for (uint8_t i = 1; i <= gCyl && i <= MAX_CYL; i++)
          coilFired[i] = 0;
        injReq[1] = injReq[4] = 1;
        if (cycleHalf) { injReq[2] = injReq[3] = 1; }
      }
    }
  }
}

/* ── Maps ───────────────────────────────────────────────────── */
static void lookupMaps(float load, float rpm, int8_t *advOut, float *injOut) {
  uint8_t c0 = 0, c1 = COLS - 1;
  for (uint8_t i = 0; i < COLS - 1; i++) {
    if (rpm >= rpmBinsLive[i] && rpm <= rpmBinsLive[i + 1]) { c0 = i; c1 = i + 1; break; }
    if (rpm > rpmBinsLive[COLS - 1]) { c0 = c1 = COLS - 1; }
  }
  uint8_t r0 = 0, r1 = ROWS - 1;
  for (uint8_t i = 0; i < ROWS - 1; i++) {
    if (load >= mapBinsLive[i] && load <= mapBinsLive[i + 1]) { r0 = i; r1 = i + 1; break; }
    if (load > mapBinsLive[ROWS - 1]) { r0 = r1 = ROWS - 1; }
  }
  float cf = (rpmBinsLive[c1] != rpmBinsLive[c0]) ?
    (rpm - rpmBinsLive[c0]) / (rpmBinsLive[c1] - rpmBinsLive[c0]) : 0;
  float rf = (mapBinsLive[r1] != mapBinsLive[r0]) ?
    (load - mapBinsLive[r0]) / (mapBinsLive[r1] - mapBinsLive[r0]) : 0;
  if (cf < 0) cf = 0;
  if (cf > 1) cf = 1;
  if (rf < 0) rf = 0;
  if (rf > 1) rf = 1;

  float adv = (1-cf)*(1-rf)*advMap[r0][c0] + cf*(1-rf)*advMap[r0][c1]
            + (1-cf)*rf*advMap[r1][c0] + cf*rf*advMap[r1][c1];
  float inj = ((1-cf)*(1-rf)*injMap[r0][c0] + cf*(1-rf)*injMap[r0][c1]
            + (1-cf)*rf*injMap[r1][c0] + cf*rf*injMap[r1][c1]) * 0.1f;
  *advOut = clampAdv((int)(adv + 0.5f));
  *injOut = inj;
  mapCellR = r0;
  mapCellC = c0;
  baseAdvDeg = *advOut;
  baseInjMs  = inj;
}

/* ── Sequential coils ───────────────────────────────────────── */
static void scheduleCoils(uint32_t now) {
  enum { COIL_HANG_US = 8000u }; /* hard max dwell - never leave coil charged */
  for (uint8_t i = 1; i <= MAX_CYL; i++) {
    if (coilState[i] && (now - coilStartUs[i]) > COIL_HANG_US) {
      ECU_IGN_LO(i);
      coilState[i] = 0;
      coilFired[i] = 1;
    }
  }
  /* RPM limiter: hard = full spark cut; soft = retard + skip every other */
  if (rpmLive >= gRpmLimit)
    rpmCutActive = 1;
  else if (rpmLive + (gRpmCutMode ? 150 : 200) < gRpmLimit)
    rpmCutActive = 0;

  if (!syncLocked || rpmLive < 30 || toothPeriodUs < 40) {
    for (uint8_t i = 1; i <= gCyl && i <= MAX_CYL; i++) {
      ECU_IGN_LO(i);
      coilState[i] = 0;
    }
    return;
  }
  if (rpmCutActive && gRpmCutMode == 0) {
    /* Hard cut - all coils off */
    for (uint8_t i = 1; i <= gCyl && i <= MAX_CYL; i++) {
      ECU_IGN_LO(i);
      coilState[i] = 0;
    }
    return;
  }

  float usPerRev = (float)toothPeriodUs * (float)gTeeth;
  if (usPerRev < 400.0f) return;
  float degPerUs = 360.0f / usPerRev;
  float dwellDeg = (float)dwellTargetUs * degPerUs;
  if (dwellDeg > 90.0f) dwellDeg = 90.0f;
  if (dwellDeg < 2.0f) dwellDeg = 2.0f;
  float band = 360.0f / (float)gTeeth;
  float adv = (float)ignAdvanceDeg;
  /* soft limiter retard already folded into ignAdvanceDeg via computeIgnitionAdvance */
  float trig = (float)gTrigAngle;
  float deg = crankDeg;
  float cycle = injSequentialActive() ? 720.0f : 360.0f;

  uint8_t n = gCyl;
  if (n > MAX_CYL) n = MAX_CYL;
  /* Without cam sequential, fire as wasted-spark pairs on 360° */
  if (!injSequentialActive()) {
    n = (gCyl >= 4) ? 4 : gCyl;
  }

  for (uint8_t i = 1; i <= n; i++) {
    if (rpmCutActive && gRpmCutMode == 1 && (i & 1u)) {
      ECU_IGN_LO(i); coilState[i] = 0; continue; /* soft: drop odd cyl */
    }
    float tdc = tdcDeg(i);
    if (!injSequentialActive()) {
      /* Map to 360° wasted: cyl1/4 @ 0, cyl2/3 @ 180 */
      if (i == 1 || i == 4) tdc = 0.0f;
      else if (i == 2 || i == 3) tdc = 180.0f;
      else continue;
    }
    /* Angle domain: gap → crankDeg=0. True TDC is +trig later.
     * Spark at (TDC − advance) = tdc + trig − adv.
     * Previous "tdc - adv - trig" advanced spark by ~2×trig vs map. */
    float fire = tdc + trig - adv;
    while (fire < 0) fire += cycle;
    while (fire >= cycle) fire -= cycle;
    float dwellStart = fire - dwellDeg;
    while (dwellStart < 0) dwellStart += cycle;

    uint8_t inDwell; (void)inDwell;
    if (dwellStart <= fire)
      inDwell = (deg >= dwellStart && deg < fire);
    else
      inDwell = (deg >= dwellStart || deg < fire);

#if CFG_COIL_SMART
    if (!coilFired[i] && deg >= fire && deg < fire + band * 2.0f) {
      if (!coilState[i]) {
        ECU_IGN_HI(i);
        coilState[i] = 1;
        coilStartUs[i] = now;
      }
    }
    if (coilState[i] && (now - coilStartUs[i]) >= (uint32_t)CFG_DWELL_NOM_US) {
      ECU_IGN_LO(i);
      dwellActualUs = (uint16_t)(now - coilStartUs[i]);
      coilState[i] = 0;
      coilFired[i] = 1;
    }
#else
    if (!coilFired[i] && !coilState[i] && inDwell) {
      ECU_IGN_HI(i);
      coilState[i] = 1;
      coilStartUs[i] = now;
    }
    if (coilState[i] && !coilFired[i]) {
      uint8_t atFire = (deg >= fire && deg < fire + band * 2.0f);
      uint8_t timeUp = (now - coilStartUs[i]) >= dwellTargetUs;
      if (atFire || (timeUp && deg >= fire - band)) {
        ECU_IGN_LO(i);
        dwellActualUs = (uint16_t)(now - coilStartUs[i]);
        coilState[i] = 0;
        coilFired[i] = 1;
      }
    }
#endif
    if (coilState[i] && (now - coilStartUs[i]) > (uint32_t)CFG_DWELL_MAX_US + 500U) {
      ECU_IGN_LO(i);
      coilState[i] = 0;
      coilFired[i] = 1;
    }
  }
}


/* ── EOI-based sequential injection ───────────────────────────
 * Injection ends at (compression TDC - gEoiBtdc) on the engine cycle.
 * SOI = EOI - pulse_width_in_degrees (from current PW and RPM).
 * 720° when sequential + cam; 360° wasted/semi-seq otherwise.
 * Time-based close is a safety backup if angle is missed.
 */
#ifndef CFG_EOI_BTDC_DEG
#define CFG_EOI_BTDC_DEG  60.0f
#endif

static float wrapAngle(float a, float cycle)
{
  while (a < 0.0f) a += cycle;
  while (a >= cycle) a -= cycle;
  return a;
}

static uint8_t angleActive(float deg, float start, float end, float cycle)
{
  start = wrapAngle(start, cycle);
  end   = wrapAngle(end, cycle);
  deg   = wrapAngle(deg, cycle);
  if (start <= end)
    return (deg >= start && deg < end) ? 1u : 0u;
  /* window crosses 0 */
  return (deg >= start || deg < end) ? 1u : 0u;
}

static void serviceInjection(void) {
  uint32_t now = micros();
  if ((rpmCutActive && gRpmCutMode == 0) || dfcoActive) {
    for (uint8_t i = 1; i <= MAX_CYL; i++) {
      ECU_INJ_LO(i); injOn[i] = 0; injReq[i] = 0;
    }
    return;
  }
  uint16_t pw = injPwUs;
  if (pw < 800) pw = 800;
  if (pw > 20000) pw = 20000;

  /* Timed close - exact pulse width */
  for (uint8_t i = 1; i <= MAX_CYL; i++) {
    if (injOn[i] && (int32_t)(now - injEndUs[i]) >= 0) {
      ECU_INJ_LO(i);
      injOn[i] = 0;
      injFiredCyc[i] = 1;
    }
  }

  if (!syncLocked || rpmLive < 30 || toothPeriodUs < 40) {
    for (uint8_t i = 1; i <= MAX_CYL; i++) {
      if (!injOn[i]) {
        ECU_INJ_LO(i);
        injReq[i] = 0;
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
  float eoiOfs = gEoiBtdc;
  if (eoiOfs < 10.0f) eoiOfs = 10.0f;
  if (eoiOfs > 400.0f) eoiOfs = 400.0f;

  /* One pulse per cycle: clear "fired" only well after EOI (past mid-cycle) */
  uint8_t n = gCyl;
  if (n > MAX_CYL) n = MAX_CYL;
  if (!seq) n = (gCyl >= 4) ? 4 : gCyl;

  for (uint8_t i = 1; i <= n; i++) {
    float tdc;
    if (seq) {
      tdc = tdcDeg(i);
    } else {
      /* batch: 1+4 @ 0°, 2+3 @ 180° */
      if (i == 1 || i == 4) tdc = 0.0f;
      else if (i == 2 || i == 3) tdc = 180.0f;
      else continue;
    }

    float eoi = wrapAngle(tdc - eoiOfs, cycle);
    /* Arm again only when well past EOI (half cycle later) */
    if (injFiredCyc[i] && !injOn[i]) {
      float past = wrapAngle(deg - eoi, cycle);
      if (past > cycle * 0.35f && past < cycle * 0.95f)
        injFiredCyc[i] = 0;
    }

    /* Primary trigger: injReq from gap ISR (once per event) */
    uint8_t start = 0;
    if (injReq[i] && !injOn[i] && !injFiredCyc[i]) {
      start = 1;
      injReq[i] = 0;
    }

    /* Secondary: angle near SOI only if no req path (narrow, edge-like) */
    if (!start && !injOn[i] && !injFiredCyc[i]) {
      float degPerUs = 360.0f / usPerRev;
      float pwDeg = (float)pw * degPerUs;
      if (pwDeg < 1.0f) pwDeg = 1.0f;
      float soi = wrapAngle(eoi - pwDeg, cycle);
      /* narrow: one tooth only - avoids multi-fire while lingering in window */
      float cap = band * 1.2f;
      if (cap < 6.0f) cap = 6.0f;
      if (cap > 15.0f) cap = 15.0f;
      if (angleActive(deg, soi, wrapAngle(soi + cap, cycle), cycle))
        start = 1;
    }

    if (start) {
      uint16_t pwc = pw;
      if (i >= 1 && i <= MAX_CYL) {
        float tr = 1.0f + cylTrimPct[i] * 0.01f;
        if (tr < 0.75f) tr = 0.75f;
        if (tr > 1.25f) tr = 1.25f;
        pwc = (uint16_t)((float)pw * tr);
        if (pwc < 800) pwc = 800;
        if (pwc > 20000) pwc = 20000;
      }
      ECU_INJ_HI(i);
      injOn[i] = 1;
      injEndUs[i] = now + pwc;
      injFiredCyc[i] = 0; /* will set on close */
    }
  }
}


/* Closed-loop electronic throttle: pedal (or etbTarget) → TPS feedback → H-bridge */

/* ── Closed-loop boost (MAP feedback → wastegate solenoid PWM) ─ */
static float boostTargetKpa = 0.0f;   /* gauge target; 0 = open-loop off */
static float boostIntegral  = 0.0f;
static float boostPrevErr   = 0.0f;
static uint8_t boostEnable  = 1;
static float BOOST_KP = 1.2f;
static float BOOST_KI = 0.4f;
static float BOOST_KD = 0.02f;
static float BOOST_MAX_KPA = 250.0f;  /* absolute MAP safety (includes atm) */
static float BOOST_MIN_DUTY = 0.0f;
static float BOOST_MAX_DUTY = 85.0f;  /* leave headroom */
/* 1 = more duty raises boost (vent WG top); 0 = inverted */
static uint8_t boostDutyRaisesBoost = 1;
#define BST_N 8
static float bstMap[BST_N][BST_N]; /* gauge kPa target, RPM×TPS */
static const float bstRpm[BST_N] = {1500,2000,2500,3000,3500,4000,5000,6000};
static const float bstTps[BST_N] = {20,30,40,50,60,70,80,100};
static uint8_t bstMapEnable = 1;
static uint8_t bstOpenLoop = 0; /* 0=CL target kPa  1=OL duty % */
/* Launch / ALS / Flat-foot */
static uint8_t  launchEnable = 0;
static float    launchRpm = 4000.0f;
static float    launchTpsMin = 80.0f;
static float    launchBoostKpa = 50.0f;
static uint8_t  launchActive = 0;
static uint8_t  alsEnable = 0;
static float    alsRetardDeg = 15.0f; /* fallback if table unused */
static uint8_t  alsExVvt = 1;
#define MS_RPM_N 8
static const float msRpmBins[MS_RPM_N] = {1500,2000,2500,3000,4000,5000,6000,7000};
static float alsRetardTbl[MS_RPM_N] = {10,12,15,18,20,22,25,25};
static float ffsRetardTbl[MS_RPM_N] = {12,14,16,18,20,22,22,20};
/* ALS extra fuel % vs RPM (on top of base PW when ALS active) */
static float alsFuelTbl[MS_RPM_N] = {25,30,35,40,45,50,50,45};
static float alsFuelPct = 40.0f; /* fallback single value */
static uint8_t alsFuelUseTable = 1;
static float knkThrTbl[MS_RPM_N] = {40,45,50,55,60,70,80,90};
static float knkMaxTbl[MS_RPM_N] = {8,10,12,12,14,14,12,10};
static uint8_t knkUseTable = 1;
static uint8_t alsUseTable = 1;
static uint8_t ffsUseTable = 1;
static uint8_t  alsActive = 0;
static float    alsMaxSec = 3.0f;      /* max continuous ALS time */
static float    alsCooldownSec = 5.0f;    /* block re-entry after timeout */
static uint32_t alsStartMs = 0;
static uint32_t alsBlockUntilMs = 0;
static uint8_t  alsTimedOut = 0;
static uint8_t  ffsEnable = 0;
static float    ffsTpsMin = 70.0f;
static float    ffsRetardDeg = 20.0f;
static uint8_t  ffsActive = 0;
static uint8_t  clutchPressed = 0;
extern TIM_HandleTypeDef htim4;


/* ── Narrowband O2 closed-loop fuel trim ─────────────────────── */
static uint8_t  o2ClEnable   = 1;
static uint8_t  o2ClActive   = 0;
static float    stftPct      = 0.0f;   /* short-term trim -25..+25 % */
static float    ltftPct      = 0.0f;   /* long-term trim  -25..+25 % */
static float    o2Filt       = 0.45f;
static uint32_t o2RichMs     = 0;
static uint32_t o2LeanMs     = 0;
static uint32_t o2LastMs     = 0;
/* Voltage thresholds (NB zirconia ~0.1-0.9 V) */
static float O2_RICH_V = 0.55f;
static float O2_LEAN_V = 0.35f;
static float STFT_STEP = 0.15f;   /* % per 10 ms tick when held rich/lean */
static float STFT_MAX  = 25.0f;
static float LTFT_RATE = 0.002f;  /* LTFT slowly follows STFT when active */
static float LTFT_MAX  = 25.0f;

/* ── Wideband / AFR ─────────────────────────────────────────── */
static uint8_t o2SensorMode = O2_MODE_NB; /* OFF / NB / WB */
static float   engAfr       = 14.7f;
static float   wbAfrMin     = 10.0f;  /* AFR at 0 V */
static float   wbAfrMax     = 20.0f;  /* AFR at wbVMax */
static float   wbVMax       = 3.3f;   /* full-scale after divider */
static float   targetAfr    = 14.7f;  /* WB closed-loop target (AFR) */
static float   stoichAfr    = 14.7f;  /* fuel stoich AFR: petrol 14.7, E85 ~9.8 */

static inline float afrToLambda(float afr)
{
  return (stoichAfr > 0.1f) ? (afr / stoichAfr) : 1.0f;
}
static inline float lambdaToAfr(float lam)
{
  return lam * stoichAfr;
}



/* ── DTC ────────────────────────────────────────────────────── */
typedef struct {
  uint16_t code;
  uint8_t  active;
  uint32_t setMs;
} DtcSlot;
static DtcSlot dtcList[DTC_MAX_ACTIVE];
static uint8_t dtcCount = 0;
static uint32_t lastDtcEvalMs = 0;
static uint32_t o2StuckSameMs = 0;
static float    o2StuckLast   = -1.0f;



/* ── Feature helpers: DTC / WB / cyl trim ────────────────────── */
static void dtcSet(uint16_t code)
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

static void dtcClearCode(uint16_t code)
{
  for (uint8_t i = 0; i < dtcCount; i++) {
    if (dtcList[i].code == code)
      dtcList[i].active = 0;
  }
}

static void serviceDtcSanity(void)
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

static void serviceO2ClosedLoop(void)
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
static float fuelTrimMul(void)
{
  float pct = ltftPct;           /* always applied */
  if (o2ClActive)
    pct += stftPct;              /* STFT only when NB CL active */
  if (pct >  STFT_MAX + LTFT_MAX) pct =  STFT_MAX + LTFT_MAX;
  if (pct < -(STFT_MAX + LTFT_MAX)) pct = -(STFT_MAX + LTFT_MAX);
  return 1.0f + pct * 0.01f;
}

/* legacy name */
static float o2FuelMul(void) { return fuelTrimMul(); }

static float totalTrimPct(void)
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

static uint8_t readClutch(void)
{
  /* PB13 clutch switch - active low with pull-up (pressed = 0) */
  return (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_13) == GPIO_PIN_RESET) ? 1 : 0;
}


/** Windowed dual-Goertzel knock → intensity → progressive retard */
static uint16_t readAdc(uint32_t channel); /* fwd */


static float msRetardLookup(const float *tbl, float rpm)
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

static void serviceKnockGoertzel(void)
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

static void serviceMotorsport(void)
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

static void serviceBoost(void)
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


static float etbLookup(float pedalPct, float rpm)
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


/* ── Drive-by-wire idle control ────────────────────────────────
 * Outer loop: RPM error → idle throttle adder (0-IDLE_MAX_PCT)
 * Active when pedal closed, not in cut, engine running.
 * Target idle RPM from ECT (cold raised). Dashpot on tip-out.
 */
static uint8_t  idleEnable     = 1;
static uint8_t  idleActive     = 0;
static float    idleTargetRpm  = 850.0f;
static float    idleThrottle   = 0.0f;   /* learned/commanded idle open % */
static float    idleIntegral   = 0.0f;
static float    idlePrevRpmErr = 0.0f;
static float    dashpotPct     = 0.0f;
static float    prevTpsIdle    = 0.0f;
static uint32_t idleLastMs     = 0;

static float IDLE_KP = 0.012f;   /* % throttle per RPM error */
static float IDLE_KI = 0.008f;
static float IDLE_KD = 0.002f;
static float IDLE_MAX_PCT = 18.0f;
static float IDLE_MIN_PCT = 1.5f;
static float IDLE_ENTRY_PEDAL = 5.0f;  /* TPS/pedal below this = idle region */
static float IDLE_EXIT_PEDAL  = 8.0f;
static float IDLE_ENTRY_TPS   = 5.0f;  /* hard gate: only idle if TPS < 5% */
static float DASHPOT_GAIN     = 0.35f;  /* TPS drop → hold % */
static float DASHPOT_DECAY    = 0.92f;  /* per 10ms-ish tick (~0.92^100 ≈ slow) */
static float DASHPOT_MAX     = 25.0f;
static float DASHPOT_MIN_DTPS = 6.0f;   /* min TPS drop % to trigger */
static float DASHPOT_MIN_TPS  = 12.0f;  /* only if TPS was above this */

/* ECT → target idle RPM (simple piecewise) */
static float idleTargetFromEct(float ectC)
{
  if (ectC < -10.0f) return 1400.0f;
  if (ectC < 20.0f)  return 1200.0f - (ectC + 10.0f) * (200.0f / 30.0f); /* 1400→1200 */
  if (ectC < 60.0f)  return 1000.0f - (ectC - 20.0f) * (150.0f / 40.0f); /* ~1000→850 */
  if (ectC < 90.0f)  return idleTargetRpm > 700.0f ? idleTargetRpm : 850.0f;
  return idleTargetRpm; /* hot: use setpoint (default 850) */
}


/* ── Deceleration fuel cut (DFCO) ───────────────────────────────
 * Cut fuel when coasting: high RPM, closed throttle/pedal, warm.
 * Restore fuel before idle (hysteresis) to avoid stall.
 */
/* Overrun / deceleration fuel cut - same state as DFCO */

/* ── Closed-loop dual VVT (intake + exhaust), 8×8 target maps ─ */
#define VVT_MAP_N 8
static int8_t vvtInMap[VVT_MAP_N][VVT_MAP_N];  /* target cam ° advance */
static int8_t vvtExMap[VVT_MAP_N][VVT_MAP_N];
/* cam1PhaseDeg / cam2PhaseDeg declared with sync flags above */
static float  vvtInIntegral = 0.0f, vvtExIntegral = 0.0f;
static float  vvtInPrevErr = 0.0f, vvtExPrevErr = 0.0f;
static uint8_t vvtClEnable = 1;
static float VVT_KP = 2.0f, VVT_KI = 0.4f, VVT_KD = 0.05f;

static const float vvtRpmBins[VVT_MAP_N] = {
  800, 1200, 1800, 2500, 3500, 4500, 5500, 6500
};
static const float vvtLoadBins[VVT_MAP_N] = {
  0.10f, 0.20f, 0.30f, 0.45f, 0.55f, 0.70f, 0.85f, 1.00f
};

static void vvtMapsDefault(void)
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

static float vvtLookup(const int8_t m[VVT_MAP_N][VVT_MAP_N], float rpm, float load)
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

static void serviceVvtClosedLoop(void)
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

static void serviceDfco(void)
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

static void serviceIdleControl(void)
{
  uint32_t now = millis();
  if (idleLastMs == 0) idleLastMs = now;
  float dt = (float)(now - idleLastMs) * 0.001f;
  if (dt < 0.001f) dt = 0.001f;
  if (dt > 0.05f) dt = 0.05f;
  idleLastMs = now;

  float pedal = engPedal;
  if (pedal < 0.0f) pedal = 0.0f;

  /* Dashpot: capture tip-out (cal via SET:DASHPOT,gain,decay,max) */
  float dTps = prevTpsIdle - engTps;
  if (dTps >= DASHPOT_MIN_DTPS && prevTpsIdle >= DASHPOT_MIN_TPS) {
    float add = dTps * DASHPOT_GAIN;
    if (add > DASHPOT_MAX) add = DASHPOT_MAX;
    if (add > dashpotPct) dashpotPct = add;
  }
  /* Decay toward 0 - DASHPOT_DECAY is per-loop factor at ~100 Hz */
  {
    float dec = DASHPOT_DECAY;
    if (dec < 0.50f) dec = 0.50f;
    if (dec > 0.995f) dec = 0.995f;
    /* Scale decay by dt so calibration is roughly independent of loop rate */
    float steps = dt * 100.0f;
    if (steps < 0.5f) steps = 0.5f;
    if (steps > 5.0f) steps = 5.0f;
    for (int i = 0; i < (int)steps; i++)
      dashpotPct *= dec;
  }
  if (dashpotPct < 0.15f) dashpotPct = 0.0f;
  prevTpsIdle = engTps;

  if (!idleEnable || !etbEnable) {
    idleActive = 0;
    idleIntegral *= 0.95f;
    return;
  }

  /* Entry / exit hysteresis on pedal */
  /* Idle only when TPS below 5% (and pedal closed) */
  if (engTps > IDLE_ENTRY_TPS) {
    idleActive = 0;
  } else if (!idleActive) {
    if (pedal <= IDLE_ENTRY_PEDAL && syncLocked && rpmLive > 400)
      idleActive = 1;
  } else {
    if (pedal >= IDLE_EXIT_PEDAL || !syncLocked || rpmLive < 200)
      idleActive = 0;
  }

  if (!idleActive) {
    /* Decay integrator slowly when leaving idle */
    idleIntegral *= (1.0f - 2.0f * dt);
    if (idleThrottle > ETB_IDLE_PCT)
      idleThrottle += (ETB_IDLE_PCT - idleThrottle) * (2.0f * dt);
    return;
  }

  float tgtRpm = idleTargetFromEct(engEct);
  /* Blend toward configured hot idle */
  if (engEct >= 70.0f)
    tgtRpm = idleTargetRpm;

  float rpmErr = tgtRpm - (float)rpmLive;

  /* Anti-stall: large positive open if RPM collapsing */
  float antiStall = 0.0f;
  if (rpmLive < tgtRpm - 200.0f && rpmLive > 400) {
    antiStall = (tgtRpm - (float)rpmLive) * 0.02f;
    if (antiStall > 12.0f) antiStall = 12.0f;
  }

  idleIntegral += rpmErr * dt;
  if (idleIntegral > 800.0f) idleIntegral = 800.0f;
  if (idleIntegral < -200.0f) idleIntegral = -200.0f;
  /* Freeze integral if throttle at rail */
  if (idleThrottle >= IDLE_MAX_PCT - 0.5f && rpmErr > 0.0f)
    idleIntegral *= 0.98f;

  float deriv = (rpmErr - idlePrevRpmErr) / dt;
  idlePrevRpmErr = rpmErr;

  float u = IDLE_KP * rpmErr + IDLE_KI * idleIntegral + IDLE_KD * deriv;
  u += antiStall;
  u += dashpotPct * 0.5f;

  idleThrottle = u;
  if (idleThrottle < IDLE_MIN_PCT) idleThrottle = IDLE_MIN_PCT;
  if (idleThrottle > IDLE_MAX_PCT) idleThrottle = IDLE_MAX_PCT;
}

static void serviceETB(void)
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

static void serviceOutputs(void) {
  static uint32_t lastRun = 0;
  uint32_t now = millis();
  if (syncLocked || rpmLive > 50) { lastRun = now; fpOn = 1; }
  else if (now - lastRun > 3000) fpOn = 0;
  /* Fan with hysteresis: on at gFanOnC, off at gFanOffC */
  if (fanOn)
    fanOn = (engEct > gFanOffC) ? 1 : 0;
  else
    fanOn = (engEct >= gFanOnC) ? 1 : 0;
  if (fpOn) ECU_FP_HI(); else ECU_FP_LO();
  if (fanOn) ECU_FAN_HI(); else ECU_FAN_LO();
  /* VVT duty applied in ECU_SetVVT via TIM1 PWM */
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
static uint16_t readAdc(uint32_t channel) {
  ADC_ChannelConfTypeDef s = {0};
  s.Channel = channel;
  s.Rank = 1;
  s.SamplingTime = ADC_SAMPLETIME_84CYCLES;
  HAL_ADC_ConfigChannel(&hadc1, &s);
  HAL_ADC_Start(&hadc1);
  HAL_ADC_PollForConversion(&hadc1, 5);
  uint16_t v = (uint16_t)HAL_ADC_GetValue(&hadc1);
  HAL_ADC_Stop(&hadc1);
  return v;
}

static float ntcBetaC(uint16_t adc) {
  if (adc < 1) adc = 1;
  if (adc > 4094) adc = 4094;
  float v = (float)adc / CFG_ADC_MAX;
  float r = CFG_NTC_PULLUP_OHM * v / (1.0f - v);
  float st = logf(r / CFG_NTC_R0_OHM) / CFG_NTC_BETA + 1.0f / 298.15f;
  return (1.0f / st) - 273.15f;
}


/** Map raw ADC to 0-100% using closed/open endpoints */
static float adcToPctCal(uint16_t adc, uint16_t closed, uint16_t open)
{
  int32_t span = (int32_t)open - (int32_t)closed;
  if (span > -50 && span < 50) {
    /* Uncalibrated / invalid span - fall back to full-scale */
    return (float)adc * (100.0f / CFG_ADC_MAX);
  }
  float pct = 100.0f * ((float)((int32_t)adc - (int32_t)closed) / (float)span);
  if (pct < 0.0f) pct = 0.0f;
  if (pct > 100.0f) pct = 100.0f;
  return pct;
}

static void readSensors(void) {
  /* Round-robin: 2 channels per loop so ignition path is not blocked */
  float scale = CFG_BAT_ADC_REF_V / CFG_ADC_MAX;
  switch (sensorPhase & 3u) {
    case 0:
      adcMap = readAdc(ECU_ADC_CH_MAP);
      adcTps = readAdc(ECU_ADC_CH_TPS);
      if (mapCalReady) {
        float a = (float)adcMap; int i = 0;
        while (i < MAP_CAL_N - 2 && a > mapCalAdc[i + 1]) i++;
        float a0 = mapCalAdc[i], a1 = mapCalAdc[i + 1];
        float f = (a1 > a0) ? (a - a0) / (a1 - a0) : 0;
        if (f < 0) f = 0;
  if (f > 1) f = 1;
        engMap = mapCalKpa[i] * (1 - f) + mapCalKpa[i + 1] * f;
      } else engMap = CFG_MAP_OFFSET_KPA + adcMap * scale * CFG_MAP_GAIN_KPA_V;
      engTps = adcToPctCal(adcTps, tpsClosedAdc, tpsOpenAdc);
      break;
    case 1:
      adcPedal = readAdc(ECU_ADC_CH_PEDAL);
      adcBat   = readAdc(ECU_ADC_CH_VBATT);
      engPedal = adcToPctCal(adcPedal, pedClosedAdc, pedOpenAdc);
      if (batCalReady) {
        /* interpolate voltage from ADC cal table */
        float a = (float)adcBat;
        int i = 0;
        while (i < BAT_CAL_N - 2 && a > batAdcTbl[i + 1]) i++;
        float a0 = batAdcTbl[i], a1 = batAdcTbl[i + 1];
        float f = (a1 > a0) ? (a - a0) / (a1 - a0) : 0;
        if (f < 0) f = 0;
  if (f > 1) f = 1;
        engBat = (batVoltTbl[i] * (1 - f) + batVoltTbl[i + 1] * f) * batCompTbl[i];
      } else {
        engBat = adcBat * scale * CFG_BAT_DIVIDER;
      }
      break;
    case 2:
      adcEct = readAdc(ECU_ADC_CH_CLT);
      adcIat = readAdc(ECU_ADC_CH_IAT);
      engEct = ntcBetaC(adcEct);
      engIat = ntcBetaC(adcIat);
      break;
    default:
      adcO2    = readAdc(ECU_ADC_CH_O2);
      adcKnock = readAdc(ECU_ADC_CH_KNOCK);
      engO2    = adcO2 * scale;
      engKnock = (float)adcKnock;
      if (o2SensorMode == O2_MODE_WB) {
        float vv = engO2;
        if (vv < 0.0f) vv = 0.0f;
        if (vv > wbVMax) vv = wbVMax;
        float nrm = (wbVMax > 0.01f) ? (vv / wbVMax) : 0.0f;
        engAfr = wbAfrMin + nrm * (wbAfrMax - wbAfrMin);
      } else if (o2SensorMode == O2_MODE_NB) {
        engAfr = (engO2 > 0.45f) ? 12.5f : 16.5f;
      } else {
        engAfr = 14.7f;
      }
      break;
  }
  sensorPhase++;
}

/* ── Serial ─────────────────────────────────────────────────── */
static void uartWrite(const char *s) {
  ECU_Serial_Write(s);  /* USB CDC (Black Pill) */
}
static void uartErr(const char *cmd, const char *why) {
  char b[72];
  snprintf(b, sizeof b, "ERR:%s,%s\r\n", cmd, why);
  uartWrite(b);
}


static void sendTelemetry(void) {
  /* Split into two frames - keeps each snprintf well under buffer limit
   * and avoids -Wformat-truncation (single line was 400B dest, ~400-700B need). */
  char b[320];
  unsigned pw_us = (unsigned)injPwUs;
  int ign_d = (int)ignAdvanceDeg;

  int n = snprintf(b, sizeof b,
    "RPM:%u,PW:%.2f,INJ:%.2f,IGN:%d,TRET:%.1f,MAP:%.1f,TPS:%.0f,TMP:%.0f,IAT:%.0f,BAT:%.1f,"
    "EADC:%u,TADC:%u,BADC:%u,IADC:%u,MADC:%u,"
    "SYNC:%u,CAM:%u,CAM2:%u,FAN:%u,FP:%u,LOST:%u,"
    "TOOTH:%u,DEG:%.0f,TERR:%u,DWELL:%u,CYL:%u,",
    (unsigned)rpmLive,
    (double)(pw_us * 0.001f), (double)(pw_us * 0.001f), ign_d,
    (double)totalRetardDeg,
    (double)engMap, (double)engTps, (double)engEct, (double)engIat, (double)engBat,
    (unsigned)adcEct, (unsigned)adcTps, (unsigned)adcBat,
    (unsigned)adcIat, (unsigned)adcMap,
    (unsigned)syncLocked, (unsigned)camSynced, (unsigned)cam2Synced,
    (unsigned)fanOn, (unsigned)fpOn, (unsigned)syncLosses,
    (unsigned)(syncLocked ? toothIndex : (crankEdgeCount & 0xFFFFu)),
    (double)crankDeg, (unsigned)toothErrors,
    (unsigned)dwellActualUs, (unsigned)gCyl);
  if (n > 0)
    uartWrite(b);

  float als_f = 0.0f;
  if (alsActive)
    als_f = alsFuelUseTable
              ? msRetardLookup(alsFuelTbl, (float)rpmLive)
              : alsFuelPct;

  n = snprintf(b, sizeof b,
    "AFR:%.2f,LAM:%.3f,MCELL:%u:%u,BASEIGN:%d,BASEINJ:%u,O2:%.2f,KNK:%.1f,KRET:%.1f,STFT:%.1f,LTFT:%.1f,TTRIM:%.1f,CL:%u,LOAD:%.2f,"
    "PWUS:%u,INJMODE:%u,SEQ:%u,BATCHRPM:%u,IDLE:%u,IRPM:%.0f,ITHR:%.1f,DASH:%.1f,"
    "DFCO:%u,OFC:%u,VVT1:%u,VVT2:%u,C1PH:%.0f,C2PH:%.0f,ASE:%u,CLTCH:%u,"
    "LC:%u,ALS:%u,ALSTO:%u,ALSF:%.0f,FFS:%u\r\n",
    (double)engAfr, (double)afrToLambda(engAfr),
    (unsigned)mapCellR, (unsigned)mapCellC,
    (int)baseAdvDeg, (unsigned)(baseInjMs * 10.0f + 0.5f),
    (double)engO2, (double)engKnock, (double)knockRetardDeg,
    (double)stftPct, (double)ltftPct, (double)totalTrimPct(),
    (unsigned)o2ClActive, (double)engLoad,
    pw_us, (unsigned)gInjMode, (unsigned)injSequentialActive(),
    (unsigned)gBatchAboveRpm,
    (unsigned)idleActive, (double)idleTargetFromEct(engEct),
    (double)idleThrottle, (double)dashpotPct,
    (unsigned)dfcoActive, (unsigned)dfcoActive,
    (unsigned)vvt1Duty, (unsigned)vvt2Duty,
    (double)cam1PhaseDeg, (double)cam2PhaseDeg, (unsigned)aseActive,
    (unsigned)clutchPressed, (unsigned)launchActive,
    (unsigned)alsActive, (unsigned)alsTimedOut,
    (double)als_f, (unsigned)ffsActive);
  if (n > 0)
    uartWrite(b);
}


static void ECU_ApplyWheelId(uint8_t id)
{
  const EcuWheelProfile *w = ECU_WheelById(id);
  if (!w) return;
  gWheelId = w->id;
  if (w->teeth >= 2 && w->teeth <= 60) gTeeth = w->teeth;
  gMissing = w->missing;
  gCamMode = (uint8_t)w->cam;
  syncLocked = 0;
  camSynced = (w->cam == ECU_CAM_NONE) ? 0 : camSynced;
}

/* Parse one CSV row into advMap or injMap during UPLOAD: sequence */
static void handleUploadRow(char *line)
{
  char *p = line;
  for (uint8_t c = 0; c < COLS; c++) {
    while (*p == ' ' || *p == '\t') p++;
    float val = 0.0f;
    int n = parse_float(p, &val);
    if (n <= 0) return; /* malformed → abort row */
    if (uploadMode == 1) advMap[uploadRow][c] = clampAdv((int)(val + (val >= 0 ? 0.5f : -0.5f)));
    else                 injMap[uploadRow][c] = clampInj(val);
    mapsDirty = 1;
    p += n;
    if (*p == ',') p++;
  }
  uploadRow++;
  if (uploadRow >= ROWS) {
    uploadMode = 0;
    uploadRow  = 0;
    {
      char db[48];
      snprintf(db, sizeof db, "OK:UPLOAD:DONE,A0:%d\r\n", (int)advMap[0][0]);
      uartWrite(db);
    }
  }
}


/** Pack current RAM tune into flash blob (maps, cal, wheel, LTFT). */
static void fillFlashBlob(EcuFlashBlob *blob)
{
  memset(blob, 0, sizeof(*blob));
  blob->tpsClosed = tpsClosedAdc;
  blob->tpsOpen   = tpsOpenAdc;
  blob->pedClosed = pedClosedAdc;
  blob->pedOpen   = pedOpenAdc;
  blob->trigAngle = gTrigAngle;
  blob->teeth     = gTeeth;
  blob->missing   = gMissing;
  {
    float x = ltftPct * 100.0f;
    if (x >  2500.0f) x =  2500.0f;
    if (x < -2500.0f) x = -2500.0f;
    blob->ltftCenti = (int16_t)x;
  }
  /* Always copy full map grids — these are what must survive power-cycle */
  for (uint8_t r = 0; r < ECU_FLASH_MAP_ROWS; r++) {
    for (uint8_t c = 0; c < ECU_FLASH_MAP_COLS; c++) {
      if (r < ROWS && c < COLS) {
        blob->advMap[r][c] = advMap[r][c];
        blob->injMap[r][c] = injMap[r][c];
      } else {
        blob->advMap[r][c] = 0;
        blob->injMap[r][c] = 0;
      }
    }
  }
}


/** Run from ECU_Loop — pack RAM maps, program NVM, verify read-back */
static void servicePendingSave(void)
{
  if (!savePending)
    return;
  savePending = 0;

  EcuFlashBlob blob;
  fillFlashBlob(&blob);

  int8_t ramA0 = advMap[0][0];
  uint8_t ramI0 = injMap[0][0];

  /* Let BUSY:SAVE leave the TX ring before IRQs go offline */
  for (int i = 0; i < 40; i++)
    ECU_Serial_Service();

  int err = ECU_Flash_Save(&blob);
  saveLastErr = (int8_t)err;

  /* USB stack needs time after long IRQ-off window */
  for (int i = 0; i < 120; i++)
    ECU_Serial_Service();

  /* Read-back first adv cell from programmed flash (not RAM) */
  int8_t flashA0 = -128;
  uint8_t flashI0 = 0;
  uint32_t crc = 0, addr = ECU_Flash_SectorAddr();
  if (err == 0 && ECU_Flash_Present()) {
    EcuFlashBlob rb;
    if (ECU_Flash_Load(&rb)) {
      flashA0 = rb.advMap[0][0];
      flashI0 = rb.injMap[0][0];
      crc = rb.crc32;
      mapsDirty = 0;
    } else {
      err = -6; /* CRC/load failed after program */
      saveLastErr = (int8_t)err;
    }
  }

  char b[160];
  if (err == 0) {
    snprintf(b, sizeof b,
             "OK:SAVE,RAMA0:%d,FLA0:%d,RAMI0:%.1f,FLI0:%.1f,CRC:0x%08lX,ADDR:0x%08lX\r\n",
             (int)ramA0, (int)flashA0,
             (double)(ramI0 / 10.0f), (double)(flashI0 / 10.0f),
             (unsigned long)crc, (unsigned long)addr);
    uartWrite(b);
    for (int i = 0; i < 40; i++)
      ECU_Serial_Service();
    uartWrite(b); /* duplicate — first packet often lost after flash */
  } else {
    const char *why = "FAIL";
    if (err == -2) why = "ERASE";
    else if (err == -3) why = "PROGRAM";
    else if (err == -4) why = "VERIFY";
    else if (err == -5) why = "SIZE";
    else if (err == -6) why = "CRC";
    else if (err == -1) why = "NULL";
    snprintf(b, sizeof b, "ERR:SAVE,%d,%s,RAMA0:%d,ADDR:0x%08lX\r\n",
             err, why, (int)ramA0, (unsigned long)addr);
    uartWrite(b);
  }
  for (int i = 0; i < 60; i++)
    ECU_Serial_Service();
}


static void handleLine(char *line) {
  /* Trim leading spaces / CR */
  while (line && (*line == ' ' || *line == '\t' || *line == '\r'))
    line++;
  if (!line || !*line)
    return;

  /* ── Bulk upload protocol: UPLOAD:ADV / UPLOAD:INJ + CSV rows ── */
  /* Abort bulk upload on SAVE/GET/CFG so flash save is never eaten as CSV */
  if (uploadMode != 0) {
    if (!strncmp(line, "SAVE", 4) || !strncmp(line, "GET", 3) ||
        !strncmp(line, "CFG", 3) || !strncmp(line, "ABORT", 5)) {
      uploadMode = 0;
      uploadRow  = 0;
      uartWrite("OK:UPLOAD:ABORT\r\n");
      if (!strncmp(line, "ABORT", 5))
        return;
      /* else fall through to SAVE / GET / CFG */
    } else if (!strncmp(line, "UPLOAD:", 7)) {
      uploadMode = 0;
      uploadRow  = 0;
      /* fall through to start new upload */
    } else {
      handleUploadRow(line);
      return;
    }
  }

  /* SAVE — queue NVM write (executed in ECU_Loop) */
  /* SAVE — write NVM immediately (maps must already be in RAM) */
  if (!strncmp(line, "SAVE", 4)) {
    uploadMode = 0;
    uploadRow  = 0;
    savePending = 0;

    uartWrite("BUSY:SAVE\r\n");
    for (int i = 0; i < 30; i++)
      ECU_Serial_Service();

    EcuFlashBlob blob;
    fillFlashBlob(&blob);

    int8_t ramA0 = advMap[0][0];
    uint8_t ramI0 = injMap[0][0];
    uint32_t injSum = 0;
    for (uint8_t r = 0; r < ROWS; r++)
      for (uint8_t c = 0; c < COLS; c++)
        injSum += injMap[r][c];

    int err = ECU_Flash_Save(&blob);
    saveLastErr = (int8_t)err;

    for (int i = 0; i < 100; i++)
      ECU_Serial_Service();

    int8_t flashA0 = -128;
    uint8_t flashI0 = 0;
    uint32_t crc = 0, addr = ECU_Flash_SectorAddr();
    if (err == 0 && ECU_Flash_Present()) {
      EcuFlashBlob rb;
      if (ECU_Flash_Load(&rb)) {
        flashA0 = rb.advMap[0][0];
        flashI0 = rb.injMap[0][0];
        crc = rb.crc32;
        mapsDirty = 0;
      } else {
        err = -6;
      }
    }

    char b[180];
    if (err == 0) {
      snprintf(b, sizeof b,
               "OK:SAVE,RAMA0:%d,FLA0:%d,RAMI0:%.1f,FLI0:%.1f,"
               "INJSUM:%lu,CRC:0x%08lX,ADDR:0x%08lX,SEC:%lu\r\n",
               (int)ramA0, (int)flashA0,
               (double)(ramI0 / 10.0f), (double)(flashI0 / 10.0f),
               (unsigned long)injSum, (unsigned long)crc,
               (unsigned long)addr, (unsigned long)ECU_Flash_SectorIndex());
      uartWrite(b);
      for (int i = 0; i < 40; i++)
        ECU_Serial_Service();
      uartWrite(b);
    } else {
      const char *why = "FAIL";
      if (err == -2) why = "ERASE";
      else if (err == -3) why = "PROGRAM";
      else if (err == -4) why = "VERIFY";
      else if (err == -5) why = "SIZE";
      else if (err == -6) why = "CRC";
      snprintf(b, sizeof b,
               "ERR:SAVE,%d,%s,RAMA0:%d,INJSUM:%lu,ADDR:0x%08lX,SEC:%lu\r\n",
               err, why, (int)ramA0, (unsigned long)injSum,
               (unsigned long)addr, (unsigned long)ECU_Flash_SectorIndex());
      uartWrite(b);
    }
    for (int i = 0; i < 40; i++)
      ECU_Serial_Service();
    return;
  }


  if (!strncmp(line, "UPLOAD:ADV", 10)) {
    uploadMode = 1; uploadRow = 0;
    uartWrite("OK:UPLOAD:ADV\r\n");
    return;
  }
  if (!strncmp(line, "UPLOAD:INJ", 10)) {
    uploadMode = 2; uploadRow = 0;
    uartWrite("OK:UPLOAD:INJ\r\n");
    return;
  }

  if (!strncmp(line, "SET:A,", 6)) {
    const char *p = line + 6;
    int r = 0, c = 0, n;
    float v = 0.0f;
    n = parse_int(p, &r); if (n <= 0) { uartErr("SET:A", "PARSE"); return; }
    p += n; if (*p == ',') p++;
    n = parse_int(p, &c); if (n <= 0) { uartErr("SET:A", "PARSE"); return; }
    p += n; if (*p == ',') p++;
    n = parse_float(p, &v); if (n <= 0) { uartErr("SET:A", "PARSE"); return; }
    if (r < 0 || r >= ROWS || c < 0 || c >= COLS) {
      uartErr("SET:A", "RANGE");
      return;
    }
    advMap[r][c] = clampAdv((int)(v + (v >= 0 ? 0.5f : -0.5f)));
    mapsDirty = 1;
    return;
  }
  if (!strncmp(line, "SET:I,", 6)) {
    const char *p = line + 6;
    int r = 0, c = 0, n;
    float v = 0.0f;
    n = parse_int(p, &r); if (n <= 0) { uartErr("SET:I", "PARSE"); return; }
    p += n; if (*p == ',') p++;
    n = parse_int(p, &c); if (n <= 0) { uartErr("SET:I", "PARSE"); return; }
    p += n; if (*p == ',') p++;
    n = parse_float(p, &v); if (n <= 0) { uartErr("SET:I", "PARSE"); return; }
    if (r < 0 || r >= ROWS || c < 0 || c >= COLS) {
      uartErr("SET:I", "RANGE");
      return;
    }
    if (v < 0.0f || v > 25.0f) {
      uartErr("SET:I", "VALUE");
      return;
    }
    injMap[r][c] = clampInj(v);
    mapsDirty = 1;
    return;
  }
  if (!strncmp(line, "SET:WHEEL,", 10) || !strncmp(line, "CFG:WHEEL,", 10)) {
    int id = 0;
    if (strchr(line, ',') && sscanf(strchr(line, ',') + 1, "%d", &id) == 1) {
      ECU_ApplyWheelId((uint8_t)id);
      char b[40];
      snprintf(b, sizeof b, "OK:WHEEL,%u\r\n", (unsigned)gWheelId);
      uartWrite(b);
    }
    return;
  }

  if (!strncmp(line, "CFG:", 4)) {
    int te, mi, an;
    if (sscanf(line + 4, "%d,%d,%d", &te, &mi, &an) >= 3) {
      if (te >= 12 && te <= 60) gTeeth = (uint8_t)te;
      if (mi >= 1 && mi < gTeeth) gMissing = (uint8_t)mi;
      gTrigAngle = (uint16_t)an;
      syncLocked = 0;
      camSynced = 0;
    }
    return;
  }

  if (!strncmp(line, "GETCFG", 6)) {
    char b[80];
    snprintf(b, sizeof b, "CFG:%u,%u,%u,CYL:%u,SEQ:%u\r\n",
             (unsigned)gTeeth, (unsigned)gMissing, (unsigned)gTrigAngle,
             (unsigned)gCyl, (unsigned)gInjMode);
    uartWrite(b);
    return;
  }

  if (!strncmp(line, "GETWHEEL", 8)) {
    char wb[64];
    const EcuWheelProfile *w = ECU_WheelById(gWheelId);
    snprintf(wb, sizeof wb, "WHEEL:%u,%u,%u,%u,%s\r\n",
      (unsigned)gWheelId, (unsigned)gTeeth, (unsigned)gMissing,
      (unsigned)gCamMode, w ? w->name : "?");
    uartWrite(wb);
    return;
  }

  if (!strncmp(line, "GETCRC", 6) || !strncmp(line, "GET:CRC", 7)) {
    char b[48];
    if (ECU_Flash_Present()) {
      snprintf(b, sizeof b, "OK:CRC,0x%08lX,VER:%u\r\n",
               (unsigned long)ECU_Flash_StoredCrc(), (unsigned)ECU_FLASH_VERSION);
    } else {
      snprintf(b, sizeof b, "ERR:CRC,NONE\r\n");
    }
    uartWrite(b);
    return;
  }

  if (!strncmp(line, "GETMAP", 6)) {
    mapDumpBusy = 1;
    uploadMode = 0;
    uartWrite("MAP:ADV\r\n");
    ECU_Serial_Service();
    for (uint8_t r = 0; r < ROWS; r++) {
      char row[160]; int n = 0;
      for (uint8_t c = 0; c < COLS; c++)
        n += snprintf(row + n, (int)sizeof row - n, c ? ",%d" : "%d", (int)advMap[r][c]);
      uartWrite(row);
      uartWrite("\r\n");
      for (int k = 0; k < 8; k++) ECU_Serial_Service();
    }
    uartWrite("MAP:INJ\r\n");
    ECU_Serial_Service();
    /* Integer tenths-of-ms — nano.specs has no printf %f */
    for (uint8_t r = 0; r < ROWS; r++) {
      char row[160]; int n = 0;
      for (uint8_t c = 0; c < COLS; c++)
        n += snprintf(row + n, (int)sizeof row - n, c ? ",%u" : "%u",
                      (unsigned)injMap[r][c]);
      uartWrite(row);
      uartWrite("\r\n");
      for (int k = 0; k < 16; k++) ECU_Serial_Service();
    }
    uartWrite("MAP:END\r\n");
    for (int k = 0; k < 20; k++) ECU_Serial_Service();
    mapDumpBusy = 0;
    return;
  }

  if (!strncmp(line, "GET:TPSCAL", 10) || !strncmp(line, "GETTPSCAL", 9)) {
    char b[80];
    snprintf(b, sizeof b, "TPSCAL:%u,%u,PED:%u,%u,VALID:%u\r\n",
             (unsigned)tpsClosedAdc, (unsigned)tpsOpenAdc,
             (unsigned)pedClosedAdc, (unsigned)pedOpenAdc,
             (unsigned)tpsCalValid);
    uartWrite(b);
    return;
  }

  if (!strncmp(line, "SET:TPS,", 8)) {
    int a, b;
    if (sscanf(line + 8, "%d,%d", &a, &b) != 2) {
      uartErr("TPS", "PARSE");
      return;
    }
    if (a < 0 || a > 4095 || b < 0 || b > 4095 || b <= a + 50) {
      uartErr("TPS", "RANGE");
      return;
    }
    tpsClosedAdc = (uint16_t)a;
    tpsOpenAdc = (uint16_t)b;
    tpsCalValid = 1;
    uartWrite("OK:TPS\r\n");
    return;
  }

  if (!strncmp(line, "SET:TRIG,", 9) || !strncmp(line, "SET:B,", 6)) {
    int a = 0;
    const char *p = strchr(line, ',');
    if (!p || sscanf(p + 1, "%d", &a) != 1) {
      uartErr("TRIG", "PARSE");
      return;
    }
    if (a < 0 || a > 360) {
      uartErr("TRIG", "RANGE");
      return;
    }
    gTrigAngle = (uint16_t)a;
    char b[32];
    snprintf(b, sizeof b, "OK:TRIG,%u\r\n", (unsigned)gTrigAngle);
    uartWrite(b);
    return;
  }


  if (!strncmp(line, "GETDTC", 6)) {
    char b[160];
    int n = snprintf(b, sizeof b, "DTC:%u", (unsigned)ECU_Dtc_Count());
    for (uint8_t i = 0; i < ECU_Dtc_Count() && n < (int)sizeof b - 12; i++) {
      n += snprintf(b + n, sizeof b - (size_t)n, ",0x%04X",
                    (unsigned)ECU_Dtc_Get(i));
    }
    if (n < (int)sizeof b - 3) {
      b[n++] = '\r'; b[n++] = '\n'; b[n] = 0;
    }
    uartWrite(b);
    return;
  }
  if (!strncmp(line, "CLRDTC", 6) || !strncmp(line, "CLEARDTC", 8)) {
    ECU_Dtc_Clear();
    uartWrite("OK:CLRDTC\r\n");
    return;
  }
  if (!strncmp(line, "SET:O2MODE,", 11)) {
    int m = 0;
    if (sscanf(line + 11, "%d", &m) != 1) {
      uartErr("O2MODE", "PARSE");
      return;
    }
    if (m < 0 || m > 2) {
      uartErr("O2MODE", "RANGE");
      return;
    }
    o2SensorMode = (uint8_t)m;
    if (o2SensorMode == O2_MODE_OFF) o2ClEnable = 0;
    char b[32];
    snprintf(b, sizeof b, "OK:O2MODE,%u\r\n", (unsigned)o2SensorMode);
    uartWrite(b);
    return;
  }
  if (!strncmp(line, "SET:WB,", 7)) {
    float a0, a1, vm = 3.3f;
    int n = sscanf(line + 7, "%f,%f,%f", &a0, &a1, &vm);
    if (n < 2) {
      uartErr("WB", "PARSE");
      return;
    }
    if (!(a0 >= 5.0f && a0 < a1 && a1 <= 30.0f)) {
      uartErr("WB", "RANGE");
      return;
    }
    if (n >= 3 && !(vm >= 1.0f && vm <= 5.5f)) {
      uartErr("WB", "VMAX");
      return;
    }
    wbAfrMin = a0;
    wbAfrMax = a1;
    if (n >= 3) wbVMax = vm;
    char wbok[72];
    snprintf(wbok, sizeof wbok, "OK:WB,AFR:%.1f-%.1f,LAM:%.3f-%.3f\r\n",
             (double)wbAfrMin, (double)wbAfrMax,
             (double)afrToLambda(wbAfrMin), (double)afrToLambda(wbAfrMax));
    uartWrite(wbok);
    return;
  }
  if (!strncmp(line, "SET:STOICH,", 11)) {
    float s;
    if (sscanf(line + 11, "%f", &s) != 1) {
      uartErr("STOICH", "PARSE");
      return;
    }
    if (!(s > 5.0f && s < 20.0f)) {
      uartErr("STOICH", "RANGE");
      return;
    }
    float lam = afrToLambda(targetAfr);
    stoichAfr = s;
    targetAfr = lambdaToAfr(lam);
    char b[48];
    snprintf(b, sizeof b, "OK:STOICH,%.2f,TGT_AFR:%.2f,TGT_LAM:%.3f\r\n",
             (double)stoichAfr, (double)targetAfr, (double)afrToLambda(targetAfr));
    uartWrite(b);
    return;
  }
  if (!strncmp(line, "SET:TARGETAFR,", 14)) {
    float a;
    if (sscanf(line + 14, "%f", &a) != 1) {
      uartErr("TARGETAFR", "PARSE");
      return;
    }
    if (!(a >= 8.0f && a <= 22.0f)) {
      uartErr("TARGETAFR", "RANGE");
      return;
    }
    targetAfr = a;
    char b[64];
    snprintf(b, sizeof b, "OK:TARGETAFR,%.2f,LAM:%.3f\r\n",
             (double)targetAfr, (double)afrToLambda(targetAfr));
    uartWrite(b);
    return;
  }
  if (!strncmp(line, "SET:TARGETLAMBDA,", 17) || !strncmp(line, "SET:LAMBDA,", 11)) {
    const char *p = strchr(line, ',');
    float lam;
    if (!p || sscanf(p + 1, "%f", &lam) != 1) {
      uartErr("TARGETLAMBDA", "PARSE");
      return;
    }
    if (!(lam >= 0.60f && lam <= 1.40f)) {
      uartErr("TARGETLAMBDA", "RANGE");
      return;
    }
    targetAfr = lambdaToAfr(lam);
    char b[64];
    snprintf(b, sizeof b, "OK:TARGETLAMBDA,%.3f,AFR:%.2f\r\n",
             (double)lam, (double)targetAfr);
    uartWrite(b);
    return;
  }
  if (!strncmp(line, "SET:WBL,", 8)) {
    float l0, l1, vm = 3.3f;
    int n = sscanf(line + 8, "%f,%f,%f", &l0, &l1, &vm);
    if (n < 2) {
      uartErr("WBL", "PARSE");
      return;
    }
    if (!(l0 >= 0.50f && l0 < l1 && l1 <= 1.50f)) {
      uartErr("WBL", "RANGE");
      return;
    }
    if (n >= 3 && !(vm >= 1.0f && vm <= 5.5f)) {
      uartErr("WBL", "VMAX");
      return;
    }
    wbAfrMin = lambdaToAfr(l0);
    wbAfrMax = lambdaToAfr(l1);
    if (n >= 3) wbVMax = vm;
    char b[72];
    snprintf(b, sizeof b, "OK:WBL,%.3f,%.3f,AFR:%.1f-%.1f\r\n",
             (double)l0, (double)l1, (double)wbAfrMin, (double)wbAfrMax);
    uartWrite(b);
    return;
  }
  if (!strncmp(line, "SET:CYLTRIM,", 12)) {
    int cyl; float pct;
    if (sscanf(line + 12, "%d,%f", &cyl, &pct) != 2) {
      uartErr("CYLTRIM", "PARSE");
      return;
    }
    if (cyl < 1 || cyl > (int)MAX_CYL || cyl > (int)gCyl) {
      uartErr("CYLTRIM", "CYL");
      return;
    }
    if (!(pct >= -25.0f && pct <= 25.0f)) {
      uartErr("CYLTRIM", "RANGE");
      return;
    }
    ECU_SetCylTrim((uint8_t)cyl, pct);
    char b[40];
    snprintf(b, sizeof b, "OK:CYLTRIM,%d,%.1f\r\n", cyl, (double)ECU_GetCylTrim((uint8_t)cyl));
    uartWrite(b);
    return;
  }
  if (!strncmp(line, "GETCYLTRIM", 10)) {
    char b[96];
    int n = snprintf(b, sizeof b, "CYLTRIM");
    for (uint8_t i = 1; i <= gCyl && i <= MAX_CYL; i++)
      n += snprintf(b + n, sizeof b - (size_t)n, ",%.1f", (double)cylTrimPct[i]);
    if (n < (int)sizeof b - 3) { b[n++] = '\r'; b[n++] = '\n'; b[n] = 0; }
    uartWrite(b);
    return;
  }
  if (!strncmp(line, "SET:IDLEPID,", 12)) {
    float kp, ki, kd;
    if (sscanf(line + 12, "%f,%f,%f", &kp, &ki, &kd) != 3) {
      uartErr("IDLEPID", "PARSE");
      return;
    }
    /* Reject NaN / absurd gains that would wind up ETB */
    if (!(kp >= 0.0f && kp <= 1.0f) ||
        !(ki >= 0.0f && ki <= 1.0f) ||
        !(kd >= 0.0f && kd <= 1.0f)) {
      uartErr("IDLEPID", "RANGE");
      return;
    }
    IDLE_KP = kp; IDLE_KI = ki; IDLE_KD = kd;
    uartWrite("OK:IDLEPID\r\n");
    return;
  }
  if (!strncmp(line, "GETIDLEPID", 10)) {
    char b[64];
    snprintf(b, sizeof b, "IDLEPID:%.4f,%.4f,%.4f,THR:%.1f,ACT:%u\r\n",
             (double)IDLE_KP, (double)IDLE_KI, (double)IDLE_KD,
             (double)idleThrottle, (unsigned)idleActive);
    uartWrite(b);
    return;
  }
  if (!strncmp(line, "GETAFR", 6) || !strncmp(line, "GETLAMBDA", 9) ||
      !strncmp(line, "GET:AFR", 7) || !strncmp(line, "GET:LAMBDA", 10)) {
    char b[96];
    float lam = afrToLambda(engAfr);
    float tlam = afrToLambda(targetAfr);
    snprintf(b, sizeof b,
             "AFR:%.2f,LAM:%.3f,MODE:%u,O2V:%.3f,TGT_AFR:%.2f,TGT_LAM:%.3f,STOICH:%.2f\r\n",
             (double)engAfr, (double)lam, (unsigned)o2SensorMode, (double)engO2,
             (double)targetAfr, (double)tlam, (double)stoichAfr);
    uartWrite(b);
    return;
  }

  if (!strncmp(line, "GETFLASH", 8)) {
    char b[192];
    uint32_t addr = ECU_Flash_SectorAddr();
    uint32_t sec  = ECU_Flash_SectorIndex();
    uint32_t sz   = ECU_Flash_SectorSize();
    if (!ECU_Flash_Present()) {
      snprintf(b, sizeof b,
               "FLASH:NONE,ADDR:0x%08lX,SEC:%lu,SZ:%lu\r\n",
               (unsigned long)addr, (unsigned long)sec, (unsigned long)sz);
      uartWrite(b);
      return;
    }
    EcuFlashBlob rb;
    if (!ECU_Flash_Load(&rb)) {
      snprintf(b, sizeof b,
               "FLASH:BADCRC,ADDR:0x%08lX,SEC:%lu\r\n",
               (unsigned long)addr, (unsigned long)sec);
      uartWrite(b);
      return;
    }
    uint32_t sumA = 0, sumI = 0;
    for (uint8_t r = 0; r < ECU_FLASH_MAP_ROWS; r++)
      for (uint8_t c = 0; c < ECU_FLASH_MAP_COLS; c++) {
        sumA += (uint8_t)rb.advMap[r][c]; /* abs-ish */
        sumI += rb.injMap[r][c];
      }
    snprintf(b, sizeof b,
             "FLASH:OK,A0:%d,I0:%.1f,SUMA:%lu,SUMI:%lu,CRC:0x%08lX,"
             "ADDR:0x%08lX,SEC:%lu,SZ:%lu,VER:%u\r\n",
             (int)rb.advMap[0][0], (double)(rb.injMap[0][0] / 10.0f),
             (unsigned long)sumA, (unsigned long)sumI,
             (unsigned long)rb.crc32, (unsigned long)addr,
             (unsigned long)sec, (unsigned long)sz,
             (unsigned)rb.version);
    uartWrite(b);
    return;
  }
  
  /* Live ignition vs map — diagnoses +5° style mismatches */
  if (!strncmp(line, "GETIGNDBG", 9)) {
    char b[220];
    uint8_t r0 = mapCellR, c0 = mapCellC;
    uint8_t r1 = (r0 + 1 < ROWS) ? (uint8_t)(r0 + 1) : r0;
    uint8_t c1 = (c0 + 1 < COLS) ? (uint8_t)(c0 + 1) : c0;
    snprintf(b, sizeof b,
             "IGNDBG:RPM:%u,LOAD:%.3f,UTPS:%u,MC:%u:%u,"
             "A00:%d,A01:%d,A10:%d,A11:%d,"
             "BASE:%d,IGN:%d,TRET:%.1f,TRIG:%u\r\n",
             (unsigned)rpmLive, (double)engLoad, (unsigned)gUseTps,
             (unsigned)r0, (unsigned)c0,
             (int)advMap[r0][c0], (int)advMap[r0][c1],
             (int)advMap[r1][c0], (int)advMap[r1][c1],
             (int)baseAdvDeg, (int)ignAdvanceDeg,
             (double)totalRetardDeg, (unsigned)gTrigAngle);
    uartWrite(b);
    return;
  }
if (!strncmp(line, "GETMAPSUM", 9)) {
    int32_t sumA = 0, sumI = 0;
    for (uint8_t r = 0; r < ROWS; r++)
      for (uint8_t c = 0; c < COLS; c++) {
        sumA += advMap[r][c];
        sumI += injMap[r][c];
      }
    char b[96];
    snprintf(b, sizeof b,
             "MAPSUM:A:%ld,I:%ld,A00:%d,I00:%u,DIRTY:%u\r\n",
             (long)sumA, (long)sumI,
             (int)advMap[0][0], (unsigned)injMap[0][0],
             (unsigned)mapsDirty);
    uartWrite(b);
    return;
  }
  if (!strncmp(line, "GETUART", 7) || !strncmp(line, "GETCDC", 6)) {
    char b[160];
    snprintf(b, sizeof b,
             "UART:ERR:0x%08lX,TXDROP:%lu,RXDROP:%lu,TXFAIL:%lu,TXBUSY:%lu,"
             "LINEOVF:%lu,TXPEND:%u,RXPEND:%u,DTR:%u,CFG:%u\r\n",
             (unsigned long)ECU_Serial_GetErrors(),
             (unsigned long)g_cdc_tx_drop,
             (unsigned long)g_cdc_rx_drop,
             (unsigned long)g_cdc_tx_fail,
             (unsigned long)g_cdc_tx_busy,
             (unsigned long)g_cdc_line_ovf,
             (unsigned)ECU_Serial_TxPending(),
             (unsigned)ECU_Serial_RxPending(),
             (unsigned)ECU_CDC_GetDTR(),
             (unsigned)ECU_Serial_HostReady());
    uartWrite(b);
    return;
  }
  if (!strncmp(line, "CLRUART", 7) || !strncmp(line, "CLRUARTERR", 10)) {
    ECU_Serial_ClearErrors();
    g_cdc_tx_drop = 0;
    g_cdc_rx_drop = 0;
    g_cdc_tx_fail = 0;
    g_cdc_tx_busy = 0;
    g_cdc_line_ovf = 0;
    uartWrite("OK:CLRUART\r\n");
    return;
  }
  if (!strncmp(line, "RESET", 5) || !strncmp(line, "REBOOT", 6)
      || !strncmp(line, "SET:RESET", 9)) {
    uartWrite("OK:RESET\r\n");
    for (int i = 0; i < 20; i++) ECU_Serial_Service();
    HAL_Delay(80);
    NVIC_SystemReset();
    return;
  }
}

void ECU_UART_RxByte(uint8_t b) {
  if (b == '\n' || b == '\r') {
    if (rxLen) {
      rxBuf[rxLen] = 0;
      handleLine(rxBuf);
      rxLen = 0;
    }
  } else if (rxLen < sizeof(rxBuf) - 1) {
    rxBuf[rxLen++] = (char)b;
  } else {
    /* Line too long - discard and flag */
    rxLen = 0;
    ECU_Serial_NoteLineOverflow();
  }
}


void ECU_Init(void) {
  vvtMapsDefault();
  {
    __HAL_RCC_GPIOB_CLK_ENABLE();
    GPIO_InitTypeDef g = {0};
    g.Pin = GPIO_PIN_13;
    g.Mode = GPIO_MODE_INPUT;
    g.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOB, &g); /* clutch switch */
  }
  for (int r = 0; r < BST_N; r++)
    for (int c = 0; c < BST_N; c++) {
      float base = 20.0f + c * 8.0f; /* more boost at high TPS col */
      if (r < 2) base *= 0.5f;
      if (r > 5) base *= 0.85f;
      bstMap[r][c] = base;
    }
  for (int i = 0; i < BAT_CAL_N; i++) {
    batVoltTbl[i] = 9.0f + i * (7.0f / (BAT_CAL_N - 1));
    batCompTbl[i] = 1.0f;
    batAdcTbl[i] = batVoltTbl[i] / 11.0f / 3.3f * 4096.0f;
  }
  batCalReady = 1;
  ECU_ApplyWheelId(6); /* default 36-1 */
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

  defaultMaps();
  ECU_Flash_CrcInit();
  ECU_Features_Init();
  for (uint8_t i = 0; i < COLS; i++) rpmBinsLive[i] = rpmBins[i];
  for (uint8_t i = 0; i < ROWS; i++) mapBinsLive[i] = mapBins[i];
  for (uint8_t r = 0; r < ETB_ROWS; r++)
    for (uint8_t c = 0; c < ETB_COLS; c++) {
      /* default linear pedal response */
      etbMap[r][c] = (uint8_t)((c * 100) / (ETB_COLS - 1));
    }
  gCyl = CFG_CYLINDERS;
  if (gCyl > MAX_CYL) gCyl = MAX_CYL;
  ecuInjGpioInit(); /* reclaim PB4 from JTAG NJTRST */
  allOutputsOff();

  /* Restore maps + TPS cal from flash if present */
  {
    EcuFlashBlob blob;
    if (ECU_Flash_Load(&blob)) {
      tpsClosedAdc = blob.tpsClosed;
      tpsOpenAdc   = blob.tpsOpen;
      pedClosedAdc = blob.pedClosed;
      pedOpenAdc   = blob.pedOpen;
      tpsCalValid  = (tpsOpenAdc > tpsClosedAdc + 50) ? 1 : 0;
      if (blob.teeth >= 12 && blob.teeth <= 60) gTeeth = blob.teeth;
      if (blob.missing >= 1 && blob.missing < gTeeth) gMissing = blob.missing;
      gTrigAngle = blob.trigAngle;
      ltftPct = (float)blob.ltftCenti / 100.0f;
      if (ltftPct >  25.0f) ltftPct =  25.0f;
      if (ltftPct < -25.0f) ltftPct = -25.0f;
      stftPct = 0.0f; /* always start STFT fresh */
      /* Load advance always */
      for (uint8_t r = 0; r < ROWS; r++)
        for (uint8_t c = 0; c < COLS; c++)
          advMap[r][c] = blob.advMap[r][c];

      /* Reject all-zero inj blob (corrupt / never-written) — keep defaults */
      {
        uint32_t injSum = 0;
        for (uint8_t r = 0; r < ROWS; r++)
          for (uint8_t c = 0; c < COLS; c++)
            injSum += blob.injMap[r][c];
        if (injSum > 0) {
          for (uint8_t r = 0; r < ROWS; r++)
            for (uint8_t c = 0; c < COLS; c++)
              injMap[r][c] = blob.injMap[r][c];
        }
        /* else leave defaultMaps() values */
      }
    }
  }
}


static float coldStartEnrichMul(void)
{
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
static float alsFuelMul(void)
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

static float afterStartMul(void)
{
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

static void serviceAfterStart(void)
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
 * Unified ignition timing:
 *   final = base_map - soft_limit - ALS - FFS - knock
 *   slew-rate limited, clamped to [-15, +45]
 */
static int16_t computeIgnitionAdvance(int8_t base_adv)
{
  float a = (float)base_adv;
  float retard = 0.0f;

  /* Soft RPM limiter: progressive retard as RPM approaches / exceeds limit */
  softLimitRetardDeg = 0.0f;
  if (gRpmCutMode == 1 && gRpmLimit > 500) {
    float over = (float)rpmLive - ((float)gRpmLimit - 300.0f);
    if (over > 0.0f) {
      /* 0° at limit-300, up to 20° at limit+200 */
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

  /* Rate-limit final advance toward target (durability / smoothness) */
  float cur = (float)ignAdvanceDeg;
  float dt = 0.01f; /* ~10 ms loop assumption */
  float maxStep = advSlewDps * dt;
  float diff = (float)advTargetDeg - cur;
  if (diff > maxStep) diff = maxStep;
  if (diff < -maxStep) diff = -maxStep;
  cur += diff;
  return (int16_t)(cur < 0 ? (cur - 0.5f) : (cur + 0.5f));
}

void ECU_Loop(void) {
  ECU_Serial_Service();
  servicePendingSave(); /* flash SAVE queued by serial */
  /* Time-critical first: coils / injectors before slow ADC & closed-loop */
  scheduleCoils(micros());
  serviceInjection();

  readSensors(); /* only 2 ADC channels per pass */

  /* Load axis is NORMALISED 0.10..1.08 (same as mapBins / tuner map_bins).
   * Alpha-N: TPS% / 100
   * Speed-density: MAP_kPa / 100  (100 kPa ≈ load 1.0)
   * Do NOT pass raw kPa into lookupMaps. */
  float load = gUseTps ? (engTps * 0.01f) : (engMap * 0.01f);
  if (load < 0.0f) load = 0.0f;
  if (load > 1.2f) load = 1.2f;
  engLoad = load;
  int8_t adv;
  float injMs;
  lookupMaps(load, (float)rpmLive, &adv, &injMs);
  ignAdvanceDeg = computeIgnitionAdvance(adv);
  serviceAfterStart();
  float pw = injMs * 1000.0f * o2FuelMul() * coldStartEnrichMul()
           * afterStartMul() * alsFuelMul();
  if (pw < 800) pw = 800;
  if (pw > 20000) pw = 20000;
  if (dfcoActive)
    pw = 0; /* deceleration fuel cut */
  injPwUs = (uint16_t)pw;

  if (lastToothUs != 0 && (micros() - lastToothUs) > 2000000UL) {
    /* 2 s without any accepted tooth = stalled */
    if (syncLocked) syncLosses++;
    syncLocked = 0;
    rpmLive = 0;
    toothPeriodUs = 0;
    toothPeriodFilt = 0;
    rpmKalmanReset();
    gapConfirm = 0;
    teethSinceGap = 0;
    gapRejectStreak = 0;
    camLockHits = 0;
    cam2LockHits = 0;
    allOutputsOff();
  }
  /* Cam hysteresis unlock:
   * - Engine stopped (no crank > 2 s): clear immediately with crank stall path
   * - Running: require ~3 consecutive missed cam windows before unlock
   *   Expected cam period ≈ 1-2 crank revolutions
   */
  {
    uint32_t nowu = micros();
    uint32_t expCam = 200000UL;
    if (toothPeriodUs > 0 && gTeeth >= 2) {
      expCam = toothPeriodUs * (uint32_t)gTeeth; /* ~1 rev */
      if (expCam < 30000UL) expCam = 30000UL;
      if (expCam > 500000UL) expCam = 500000UL;
    }
    /* unlock threshold = 2.5 × expected (missed edges) */
    uint32_t camTimeout = (expCam * 5UL) / 2UL;

    if (camSynced) {
      if (lastCamEdgeUs == 0)
        lastCamEdgeUs = nowu;
      if ((nowu - lastCamEdgeUs) > camTimeout) {
        if (camUnlockMiss < 255)
          camUnlockMiss++;
        /* Need 3 consecutive timeout checks (~ loop rate) before drop */
        if (camUnlockMiss >= 3 || (nowu - lastCamEdgeUs) > (camTimeout * 3UL)) {
          camSynced = 0;
          camLockHits = 0;
          camUnlockMiss = 0;
        }
      } else {
        camUnlockMiss = 0;
      }
    } else {
      /* decay lock hits if no recent cam while unlocked */
      if (lastCamEdgeUs && (nowu - lastCamEdgeUs) > (camTimeout * 2UL))
        camLockHits = 0;
    }

    if (cam2Synced) {
      if (lastCam2EdgeUs == 0)
        lastCam2EdgeUs = nowu;
      if ((nowu - lastCam2EdgeUs) > camTimeout) {
        if (cam2UnlockMiss < 255)
          cam2UnlockMiss++;
        if (cam2UnlockMiss >= 3 || (nowu - lastCam2EdgeUs) > (camTimeout * 3UL)) {
          cam2Synced = 0;
          cam2LockHits = 0;
          cam2UnlockMiss = 0;
        }
      } else {
        cam2UnlockMiss = 0;
      }
    }
  }

  float v = engBat;
  if (v < 8) v = 8;
  if (v > 16) v = 16;
  float dus = (float)CFG_DWELL_NOM_US * (14.0f / v);
  if (dus < CFG_DWELL_MIN_US) dus = CFG_DWELL_MIN_US;
  if (dus > CFG_DWELL_MAX_US) dus = CFG_DWELL_MAX_US;
  dwellTargetUs = (uint16_t)dus;

  serviceO2ClosedLoop();
  ECU_Features_Service();
  serviceVvtClosedLoop();
  serviceDfco();
  serviceIdleControl();
  serviceVvtClosedLoop();
  serviceDfco();
  serviceETB();
  serviceKnockGoertzel();
  serviceMotorsport();
  serviceBoost();
  serviceOutputs();

  static uint32_t lastTel = 0;
  if (!mapDumpBusy && millis() - lastTel >= 100) {
    lastTel = millis();
    sendTelemetry();
  }
}

void ECU_1kHzTick(void) {}
