/**
 * TorquEFI Basic — STM32F411 full sequential ECU core
 * Cam (PA15) + crank (PA0) → 720° phase; per-cylinder spark & inject.
 */
#include "ecu_app.h"
#include "ecu_goertzel.h"
#include "ecu_wheels.h"
#include "ecu_config.h"
#include "ecu_pins.h"
#include "ecu_serial.h"
#include "ecu_flash.h"
#include "stm32f4xx_hal.h"
#include <string.h>
#include <stdio.h>
#include <math.h>

#define ROWS 15
#define COLS 22
#define MAX_CYL CFG_MAX_COILS

/* 22-column RPM axis (matches tuner: 250 + c*375, 250–8125 RPM) */
static const float rpmBins[COLS] = {
  250,  625, 1000, 1375, 1750, 2125, 2500, 2875,
 3250, 3625, 4000, 4375, 4750, 5125, 5500, 5875,
 6250, 6625, 7000, 7375, 7750, 8125
};
/* 15-row load axis (0.10–1.08 normalised) */
static const float mapBins[ROWS] = {
  0.100f, 0.170f, 0.240f, 0.310f, 0.380f, 0.450f, 0.520f, 0.590f,
  0.660f, 0.730f, 0.800f, 0.870f, 0.940f, 1.010f, 1.080f
};

static int8_t  advMap[ROWS][COLS];
static uint8_t injMap[ROWS][COLS];
/* Live breakpoints (overwritten by tuner SET:RPMB / SET:MAPB) */
static float rpmBinsLive[COLS];
static float mapBinsLive[ROWS];
/* Pedal→throttle target map: 16 RPM bands × 17 pedal points */
#define ETB_ROWS 16
#define ETB_COLS 17
static uint8_t etbMap[ETB_ROWS][ETB_COLS];
static float engLoad = 0.0f;
static uint8_t sensorPhase = 0;


static volatile uint8_t  gTeeth = CFG_TEETH;
static volatile uint8_t  gMissing = CFG_MISSING;
static volatile uint16_t gTrigAngle = CFG_TRIG_ANGLE;
static volatile uint16_t gRpmLimit = CFG_RPM_LIMIT;
static volatile uint8_t  gRpmCutMode = 0; /* 0=hard cut, 1=soft cut */
static volatile uint8_t  rpmCutActive = 0;

/* DFCO — declared early for serviceInjection */
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
static volatile uint16_t rpmLive = 0;
static uint8_t gWheelId = 6; /* default 36-1 */
static volatile uint8_t mapDumpBusy = 0;
static uint8_t gCamMode = 0;

/* Bulk-upload state (UPLOAD:ADV / UPLOAD:INJ from tuner) */
static uint8_t uploadMode = 0;   /* 0=idle  1=ADV  2=INJ */
static uint8_t uploadRow  = 0;
static volatile uint32_t crankEdgeCount = 0;
static volatile float    crankDeg = 0.0f;   /* 0..720 when sequential */
static volatile uint8_t  cycleHalf = 0;     /* 0 or 1 from cam */
static volatile uint8_t  pllSoftErr = 0, pllGoodStreak = 0;
static volatile uint8_t  gapConfirm = 0;

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
/* VVT duty 0–100% via TIM1 PWM (period 1000 counts) */
static uint8_t vvt1Duty = 0, vvt2Duty = 0;
extern TIM_HandleTypeDef htim1;
/* Closed-loop ETB */
static float etbTargetPct = -1.0f;  /* <0 = follow pedal; 0–100 = override */
static float etbIntegral  = 0.0f;
static float etbPrevErr   = 0.0f;
static uint8_t etbEnable = 1;
/* PID gains — tune on vehicle */
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
  static uint8_t  camHits = 0;
  uint32_t now = micros();
  /* Reject chatter */
  if (lastCamUs && (now - lastCamUs) < 5000UL)
    return;
  /* Accept edges up to ~250 ms apart (idle cam) */
  if (lastCamUs && (now - lastCamUs) > 250000UL) {
    /* long gap — do not force unlock; just count this as a new edge */
  }
  lastCamUs = now;
  if (camHits < 255)
    camHits++;
  /* Lock after first valid edge; stay locked (timeout in ECU_Loop) */
  camSynced = 1;
  cam1PhaseDeg = crankDeg;
  /* Cam = phase reference: next crank gap is the start of 0–360° half */
  cycleHalf = 0;
  /* Never reset toothIndex/crankDeg here — crank decoder owns angle */
}


/* ── Cam 2 (PB4 / TIM3_CH1) — second phase sensor ───────────── */
void ECU_Cam2Capture(uint32_t capt)
{
  (void)capt;
  static uint32_t lastCam2Us = 0;
  uint32_t now = micros();
  if (lastCam2Us && (now - lastCam2Us) < 5000UL)
    return;
  lastCam2Us = now;
  cam2Synced = 1;
  cam2PhaseDeg = crankDeg;
}

/* htim3 lives in CubeMX tim.c — do not define here */
extern TIM_HandleTypeDef htim3;

/* Optional: only if CubeMX TIM3 not generated. Prefer MX_TIM3_Init(). */
void ECU_TIM3_Cam2_Init(void)
{
  /* CubeMX already configured TIM3 on PB4 — nothing to do.
   * If TIM3 is missing from the .ioc, enable TIM3_CH1 IC there instead
   * of defining htim3 in this file (avoids link multiple definition). */
  (void)htim3;
}

/* ── Crank ──────────────────────────────────────────────────── */
void ECU_CrankCapture(uint32_t capt) {
  /* capt = TIM5 CNT at edge (timer @ 1 MHz → µs). Prefer hardware stamp. */
  static uint32_t lastCapt = 0;
  uint32_t now = micros();
  uint32_t dt;

  if (lastCapt == 0) {
    lastCapt = capt;
    lastToothUs = now;
    return;
  }
  dt = capt - lastCapt; /* 32-bit free-running TIM5 */
  lastCapt = capt;

  /* Reject noise / double-edges */
  if (dt < 80) return;

  if (toothPeriodUs == 0) {
    /* Allow down to ~20 RPM on 60-tooth (tooth ~50 ms) */
    if (dt < 50 || dt > 500000UL) return;
    lastToothUs = now;
    toothPeriodUs = dt;
    /* Provisional RPM before first gap lock */
    if (gTeeth >= 12) {
      uint32_t rpm = 60000000UL / (dt * (uint32_t)gTeeth);
      if (rpm > 15000UL) rpm = 15000UL;
      rpmLive = (uint16_t)rpm;
    }
    return;
  }

  uint32_t T = toothPeriodUs;
  uint8_t miss = gMissing; /* 0 = no missing tooth (even wheel) */
  uint8_t phys = (gTeeth > miss) ? (uint8_t)(gTeeth - miss) : gTeeth;
  if (phys < 2) phys = 2;

  /* Noise reject: <45% of running period (was 70% — too tight at cranking) */
  uint32_t minTooth = (T * 45UL) / 100UL;
  if (minTooth < 80) minTooth = 80;
  if (dt < minTooth) { toothErrors++; return; }

  lastToothUs = now;
  crankEdgeCount++;

  /* Provisional RPM every accepted tooth */
  if (dt > 40 && gTeeth >= 2) {
    if (!syncLocked || rpmLive < 500)
      toothPeriodUs = (T * 3UL + dt) / 4UL; /* fast adapt at cranking */
    else
      toothPeriodUs = (T * 15UL + dt) / 16UL;
    if (toothPeriodUs < 40) toothPeriodUs = 40;
    uint32_t prpm = 60000000UL / (toothPeriodUs * (uint32_t)gTeeth);
    if (prpm > 15000UL) prpm = 15000UL;
    rpmLive = (uint16_t)prpm;
  }

  uint8_t isGap = 0;
  if (miss >= 1) {
    /* Gap ≈ (missing+1) * T  — generous window for VR/Hall jitter */
    uint32_t gapNom = T * (uint32_t)(miss + 1);
    /* Very wide at low RPM — VR/Hall jitter and speed change */
    uint32_t gapMin = gapNom / 3UL;
    uint32_t gapMax = gapNom * 3UL;
    if (gapMin < T + (T / 8UL)) gapMin = T + (T / 8UL);
    if (gapMax < gapMin + T) gapMax = gapMin + T * 2UL;
    isGap = (dt >= gapMin && dt <= gapMax) ? 1 : 0;
  }

  if (isGap) {
    if (lastGapUs != 0) {
      uint32_t revUs = now - lastGapUs;
      if (revUs >= 3000UL && revUs <= 2000000UL) {
        uint32_t rpm = 60000000UL / revUs;
        if (rpm > 15000UL) rpm = 15000UL;
        rpmLive = (uint16_t)rpm;
        uint32_t tRev = revUs / (uint32_t)gTeeth;
        if (tRev >= 40 && tRev <= 80000UL)
          toothPeriodUs = (toothPeriodUs * 3UL + tRev) / 4UL;
      }
    }
    lastGapUs = now;
    toothIndex = 0;
    /* 720° phase: use current half for scheduling, then advance */
    crankDeg = injSequentialActive() ? (cycleHalf ? 360.0f : 0.0f) : 0.0f;
    for (uint8_t i = 1; i <= gCyl && i <= MAX_CYL; i++) {
      coilFired[i] = 0;
      /* do not clear injFiredCyc every gap — once-per-cycle in serviceInjection */
    }
    pllSoftErr = 0;
    pllGoodStreak++;
    if (gapConfirm < 255)
      gapConfirm++;
    if (syncLocked || gapConfirm >= 2 || rpmLive < 800 || gapConfirm >= 1)
      syncLocked = 1;

    uint32_t tGap = dt / (uint32_t)(miss + 1);
    if (tGap >= 40 && tGap <= 200000UL)
      toothPeriodUs = (toothPeriodUs * 3UL + tGap) / 4UL;

    /* Sequential+cam: request by true 720° TDC half
     * cyl1 TDC@0 → inj window in 360–720 half (prior rev / intake)
     * Use EOI-based half, not injAt==360 boundary bug */
    if (injSequentialActive()) {
      /* Full sequential — one injector per EOI half of 720° */
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
    /* Oversize interval but not a clean gap — count error, keep going */
    toothErrors++;
    toothIndex++;
  } else {
    /* Normal tooth */
    toothIndex++;
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
}

/* ── Sequential coils ───────────────────────────────────────── */
static void scheduleCoils(uint32_t now) {
  enum { COIL_HANG_US = 8000u }; /* hard max dwell — never leave coil charged */
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
    /* Hard cut — all coils off */
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
    float fire = tdc - adv - trig;
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
 * Injection ends at (compression TDC − gEoiBtdc) on the engine cycle.
 * SOI = EOI − pulse_width_in_degrees (from current PW and RPM).
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

  /* Timed close — exact pulse width */
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
      /* narrow: one tooth only — avoids multi-fire while lingering in window */
      float cap = band * 1.2f;
      if (cap < 6.0f) cap = 6.0f;
      if (cap > 15.0f) cap = 15.0f;
      if (angleActive(deg, soi, wrapAngle(soi + cap, cycle), cycle))
        start = 1;
    }

    if (start) {
      ECU_INJ_HI(i);
      injOn[i] = 1;
      injEndUs[i] = now + pw;
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
static float    stftPct      = 0.0f;   /* short-term trim −25..+25 % */
static float    ltftPct      = 0.0f;   /* long-term trim  −25..+25 % */
static float    o2Filt       = 0.45f;
static uint32_t o2RichMs     = 0;
static uint32_t o2LeanMs     = 0;
static uint32_t o2LastMs     = 0;
/* Voltage thresholds (NB zirconia ~0.1–0.9 V) */
static float O2_RICH_V = 0.55f;
static float O2_LEAN_V = 0.35f;
static float STFT_STEP = 0.15f;   /* % per 10 ms tick when held rich/lean */
static float STFT_MAX  = 25.0f;
static float LTFT_RATE = 0.002f;  /* LTFT slowly follows STFT when active */
static float LTFT_MAX  = 25.0f;

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

  /* Enable conditions for NB stoich control */
  o2ClActive = 0;
  if (!o2ClEnable) {
    stftPct *= 0.99f;
    return;
  }
  /* Freeze STFT learn + CL while ASE / DFCO / ALS (ALS fuel would skew O2) */
  if (aseActive || dfcoActive || alsActive) {
    return;  /* o2ClActive stays 0 → STFT not applied; LTFT held */
  }
  if (!syncLocked || rpmLive < 800 || rpmLive > 4000) return;
  if (engEct < 60.0f) return;              /* cold — LTFT only, no STFT learn */
  if (engTps > 60.0f) return;              /* high load — open loop */
  if (boostTargetKpa > 10.0f) return;
  if (engBat < 11.0f) return;

  o2ClActive = 1;

  /* Time spent rich / lean */
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

  /* STFT: rich → reduce fuel; lean → add fuel */
  if (o2RichMs > 20) {
    stftPct -= STFT_STEP * (dt * 100.0f);
  } else if (o2LeanMs > 20) {
    stftPct += STFT_STEP * (dt * 100.0f);
  } else {
    /* slight decay toward 0 in transition band */
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
  /* PB13 clutch switch — active low with pull-up (pressed = 0) */
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
  if (f < 0) f = 0; if (f > 1) f = 1;
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

  /* Approximate ATDC window using crankDeg (0–360 or 0–720) */
  float deg = crankDeg;
  while (deg >= 360.0f) deg -= 360.0f;
  /* Window ~15–50° ATDC */
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
        /* hit max duration — drop ALS and start cooldown */
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
    if (cf < 0) cf = 0; if (cf > 1) cf = 1;
    if (rf < 0) rf = 0; if (rf > 1) rf = 1;
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
    /* Open-loop: map cells are solenoid duty % (0–100) */
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
 * Outer loop: RPM error → idle throttle adder (0–IDLE_MAX_PCT)
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
/* Overrun / deceleration fuel cut — same state as DFCO */

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

  /* Feedback: measured phase at last cam edges (0–720 scaled to ~0–50 useful) */
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
  /* Decay toward 0 — DASHPOT_DECAY is per-loop factor at ~100 Hz */
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
 * Set dual-VVT solenoid PWM duty (0–100%).
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


/** Map raw ADC to 0–100% using closed/open endpoints */
static float adcToPctCal(uint16_t adc, uint16_t closed, uint16_t open)
{
  int32_t span = (int32_t)open - (int32_t)closed;
  if (span > -50 && span < 50) {
    /* Uncalibrated / invalid span — fall back to full-scale */
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
        if (f < 0) f = 0; if (f > 1) f = 1;
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
        if (f < 0) f = 0; if (f > 1) f = 1;
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
      break;
  }
  sensorPhase++;
}

/* ── Serial ─────────────────────────────────────────────────── */
static void uartWrite(const char *s) {
  ECU_Serial_Write(s);  /* USB CDC (Black Pill) */
}

static void sendTelemetry(void) {
  char b[400];
  /* PW/IGN early in line so they survive any truncation; integer µs avoids nano-printf float issues */
  unsigned pw_us = (unsigned)injPwUs;
  int ign_d = (int)ignAdvanceDeg;
  snprintf(b, sizeof b,
    "RPM:%u,PW:%.2f,INJ:%.2f,IGN:%d,TRET:%.1f,MAP:%.1f,TPS:%.0f,TMP:%.0f,IAT:%.0f,BAT:%.1f,"
    "EADC:%u,TADC:%u,BADC:%u,IADC:%u,MADC:%u,"
    "SYNC:%u,CAM:%u,CAM2:%u,FAN:%u,FP:%u,LOST:%u,"
    "TOOTH:%u,DEG:%.0f,TERR:%u,DWELL:%u,CYL:%u,"
    "O2:%.2f,KNK:%.1f,KRET:%.1f,STFT:%.1f,LTFT:%.1f,TTRIM:%.1f,CL:%u,LOAD:%.2f,PWUS:%u,INJMODE:%u,SEQ:%u,BATCHRPM:%u,IDLE:%u,IRPM:%.0f,ITHR:%.1f,DASH:%.1f,DFCO:%u,OFC:%u,VVT1:%u,VVT2:%u,C1PH:%.0f,C2PH:%.0f,ASE:%u,CLTCH:%u,LC:%u,ALS:%u,ALSTO:%u,ALSF:%.0f,FFS:%u\r\n",
    (unsigned)rpmLive,
    (double)(pw_us * 0.001f), (double)(pw_us * 0.001f), ign_d,
    (double)totalRetardDeg,
    engMap, engTps, engEct, engIat, engBat,
    (unsigned)adcEct, (unsigned)adcTps, (unsigned)adcBat,
    (unsigned)adcIat, (unsigned)adcMap,
    (unsigned)syncLocked, (unsigned)camSynced, (unsigned)cam2Synced,
    (unsigned)fanOn, (unsigned)fpOn, (unsigned)syncLosses,
    (unsigned)(syncLocked ? toothIndex : (crankEdgeCount & 0xFFFF)),
    (double)crankDeg, (unsigned)toothErrors,
    (unsigned)dwellActualUs, (unsigned)gCyl,
    engO2, engKnock, knockRetardDeg, stftPct, ltftPct, totalTrimPct(), (unsigned)o2ClActive, engLoad,
    pw_us, (unsigned)gInjMode, (unsigned)injSequentialActive(),
    (unsigned)gBatchAboveRpm,
    (unsigned)idleActive, idleTargetFromEct(engEct), idleThrottle, dashpotPct,
    (unsigned)dfcoActive, (unsigned)dfcoActive,
    (unsigned)vvt1Duty, (unsigned)vvt2Duty,
    cam1PhaseDeg, cam2PhaseDeg, (unsigned)aseActive,
    (unsigned)clutchPressed, (unsigned)launchActive,
    (unsigned)alsActive, (unsigned)alsTimedOut,
    alsActive ? (alsFuelUseTable ? msRetardLookup(alsFuelTbl,(float)rpmLive) : alsFuelPct) : 0.0f,
    (unsigned)ffsActive);
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
    while (*p == ' ') p++;          /* skip leading spaces */
    float val = 0.0f;
    int  n    = 0;
    if (sscanf(p, "%f%n", &val, &n) != 1) return; /* malformed → abort row */
    if (uploadMode == 1) advMap[uploadRow][c] = clampAdv((int)val);
    else                 injMap[uploadRow][c] = clampInj(val);
    p += n;
    if (*p == ',') p++;             /* advance past comma */
  }
  uploadRow++;
  if (uploadRow >= ROWS) {
    uploadMode = 0;
    uploadRow  = 0;
    uartWrite("OK:UPLOAD:DONE\r\n");
  }
}

static void handleLine(char *line) {
  /* ── Bulk upload protocol: UPLOAD:ADV / UPLOAD:INJ + CSV rows ── */
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
  /* While in upload mode, treat incoming lines as data rows */
  if (uploadMode != 0) {
    handleUploadRow(line);
    return;
  }

  if (!strncmp(line, "SET:A,", 6)) {
    int r, c; float v;
    if (sscanf(line + 6, "%d,%d,%f", &r, &c, &v) == 3 &&
        r >= 0 && r < ROWS && c >= 0 && c < COLS)
      advMap[r][c] = clampAdv((int)v);
  } else if (!strncmp(line, "SET:I,", 6)) {
    int r, c; float v;
    if (sscanf(line + 6, "%d,%d,%f", &r, &c, &v) == 3 &&
        r >= 0 && r < ROWS && c >= 0 && c < COLS)
      injMap[r][c] = clampInj(v);
  } else if (!strncmp(line, "SET:WHEEL,", 10) || !strncmp(line, "CFG:WHEEL,", 10)) {
    int id = 0;
    if (sscanf(strchr(line, ',') + 1, "%d", &id) == 1) {
      ECU_ApplyWheelId((uint8_t)id);
      /* Persist teeth/missing/trig with current maps (SAVE path) */
      {
        EcuFlashBlob blob;
        memset(&blob, 0, sizeof blob);
        blob.tpsClosed = tpsClosedAdc;
        blob.tpsOpen   = tpsOpenAdc;
        blob.pedClosed = pedClosedAdc;
        blob.pedOpen   = pedOpenAdc;
        blob.trigAngle = gTrigAngle;
        blob.teeth     = gTeeth;
        blob.missing   = gMissing;
        {
          float x = ltftPct * 100.0f;
          if (x >  2500.0f) x =  2500.0f;
          if (x < -2500.0f) x = -2500.0f;
          blob.ltftCenti = (int16_t)x;
        }
        for (uint8_t r = 0; r < ROWS; r++)
          for (uint8_t c = 0; c < COLS; c++) {
            blob.advMap[r][c] = advMap[r][c];
            blob.injMap[r][c] = injMap[r][c];
          }
        int err = ECU_Flash_Save(&blob);
        if (err == 0) {
          char b[48];
          snprintf(b, sizeof b, "OK:WHEEL,%u,SAVED\r\n", (unsigned)gWheelId);
          uartWrite(b);
        } else {
          char b[40];
          snprintf(b, sizeof b, "OK:WHEEL,%u,ERR:%d\r\n", (unsigned)gWheelId, err);
          uartWrite(b);
        }
      }
    }
  } else if (!strncmp(line, "CFG:", 4)) {
    int t, m, a;
    if (sscanf(line + 4, "%d,%d,%d", &t, &m, &a) >= 3) {
      if (t >= 12 && t <= 60) gTeeth = (uint8_t)t;
      if (m >= 1 && m < gTeeth) gMissing = (uint8_t)m;
      gTrigAngle = (uint16_t)a;
      syncLocked = 0;
      camSynced = 0;
    }
  } else if (!strncmp(line, "GETWHEEL", 8)) {
    char wb[64];
    const EcuWheelProfile *w = ECU_WheelById(gWheelId);
    snprintf(wb, sizeof wb, "WHEEL:%u,%u,%u,%u,%s\r\n",
      (unsigned)gWheelId, (unsigned)gTeeth, (unsigned)gMissing,
      (unsigned)gCamMode, w ? w->name : "?");
    uartWrite(wb);
  } else if (!strncmp(line, "GETCFG", 6)) {
    /* falls through to response below */
    char b[64];
    snprintf(b, sizeof b, "CFG:%u,%u,%u,CYL:%u,SEQ:%u\r\n",
             (unsigned)gTeeth, (unsigned)gMissing, (unsigned)gTrigAngle,
             (unsigned)gCyl, (unsigned)gInjMode);
    uartWrite(b);
  } else if (!strncmp(line, "GETMAP", 6)) {
    mapDumpBusy = 1;
    uartWrite("MAP:ADV\r\n");
    for (uint8_t r = 0; r < ROWS; r++) {
      /* 160 bytes: worst case "-10," * 22 = 88 + null — plenty of margin */
      char row[160]; int n = 0;
      for (uint8_t c = 0; c < COLS; c++)
        n += snprintf(row + n, (int)sizeof row - n, c ? ",%d" : "%d", (int)advMap[r][c]);
      uartWrite(row); uartWrite("\r\n");
    }
    uartWrite("MAP:INJ\r\n");
    for (uint8_t r = 0; r < ROWS; r++) {
      /* 160 bytes: worst case "20.0," * 22 = 110 + null — was 96 (overflow!) */
      char row[160]; int n = 0;
      for (uint8_t c = 0; c < COLS; c++)
        n += snprintf(row + n, (int)sizeof row - n, c ? ",%.1f" : "%.1f",
                      (double)(injMap[r][c] / 10.0f));
      uartWrite(row); uartWrite("\r\n");
    }
    uartWrite("MAP:END\r\n");
    mapDumpBusy = 0;

  } else if (!strncmp(line, "SET:OVERRUN,", 12) || !strncmp(line, "SET:DFCO,", 9)) {
    /* Overrun fuel cut = DFCO: SET:OVERRUN,en,enterRpm,exitRpm */
    int en = dfcoEnable, ent = dfcoEnterRpm, ex = dfcoExitRpm;
    const char *p = strstr(line, ",");
    int n = p ? sscanf(p + 1, "%d,%d,%d", &en, &ent, &ex) : 0;
    if (n >= 1) dfcoEnable = en ? 1 : 0;
    if (n >= 2) {
      if (ent < 800) ent = 800;
      if (ent > 6000) ent = 6000;
      dfcoEnterRpm = (uint16_t)ent;
    }
    if (n >= 3) {
      if (ex < 600) ex = 600;
      if (ex >= (int)dfcoEnterRpm) ex = (int)dfcoEnterRpm - 100;
      dfcoExitRpm = (uint16_t)ex;
    }
    if (!dfcoEnable) dfcoActive = 0;
    {
      char b[64];
      snprintf(b, sizeof b, "OK:OVERRUN,%u,%u,%u\r\n",
               (unsigned)dfcoEnable, (unsigned)dfcoEnterRpm, (unsigned)dfcoExitRpm);
      uartWrite(b);
    }
  } else if (!strncmp(line, "SET:IDLE,", 9)) {
    float r = 850.0f;
    sscanf(line + 9, "%f", &r);
    if (r < 500.0f) r = 500.0f;
    if (r > 2000.0f) r = 2000.0f;
    idleTargetRpm = r;
    {
      char b[40];
      snprintf(b, sizeof b, "OK:IDLE,%.0f\r\n", idleTargetRpm);
      uartWrite(b);
    }
  } else if (!strncmp(line, "SET:IDLEEN,", 11)) {
    int en = 1;
    sscanf(line + 11, "%d", &en);
    idleEnable = en ? 1 : 0;
    if (!idleEnable) { idleActive = 0; idleIntegral = 0.0f; }
    uartWrite(idleEnable ? "OK:IDLEEN,1\r\n" : "OK:IDLEEN,0\r\n");
  } else if (!strncmp(line, "SET:INJBATCHRPM,", 16) || !strncmp(line, "CFG:INJBATCHRPM,", 16)) {
    int r = 3000;
    {
      const char *p = strstr(line, "INJBATCHRPM,");
      if (p) sscanf(p + 12, "%d", &r);
    }
    if (r < 500) r = 500;
    if (r > 12000) r = 12000;
    gBatchAboveRpm = (uint16_t)r;
    {
      char b[48];
      snprintf(b, sizeof b, "OK:INJBATCHRPM,%u\r\n", (unsigned)gBatchAboveRpm);
      uartWrite(b);
    }
  } else if (!strncmp(line, "SET:INJMODE,", 12) || !strncmp(line, "CFG:INJMODE,", 12)) {
    int m = 0;
    {
      const char *p = strstr(line, "INJMODE,");
      if (p) sscanf(p + 8, "%d", &m);
    }
    if (m < 0) m = 0;
    if (m > 3) m = 3;
    gInjMode = (uint8_t)m;
    {
      char b[48];
      snprintf(b, sizeof b, "OK:INJMODE,%u\r\n", (unsigned)gInjMode);
      uartWrite(b);
    }
  } else if (!strncmp(line, "SET:INJTEST,", 12)) {
    int ch = 1, us = 2000;
    sscanf(line + 12, "%d,%d", &ch, &us);
    if (ch < 1) ch = 1;
    if (ch > (int)MAX_CYL) ch = MAX_CYL;
    if (us < 500) us = 500;
    if (us > 10000) us = 10000;
    ECU_INJ_HI((uint8_t)ch);
    /* busy wait µs via DWT */
    {
      uint32_t t0 = micros();
      while ((micros() - t0) < (uint32_t)us) { }
    }
    ECU_INJ_LO((uint8_t)ch);
    {
      char b[48];
      snprintf(b, sizeof b, "OK:INJTEST,%d,%d\r\n", ch, us);
      uartWrite(b);
    }
  } else if (!strncmp(line, "SET:RLIM,", 9)) {
    int v = 7000;
    if (sscanf(line + 9, "%d", &v) == 1) {
      if (v < 1000) v = 1000;
      if (v > 15000) v = 15000;
      gRpmLimit = (uint16_t)v;
    }
    { char b[40]; snprintf(b, sizeof b, "OK:RLIM,%u,%u\r\n",
        (unsigned)gRpmLimit, (unsigned)gRpmCutMode); uartWrite(b); }
  } else if (!strncmp(line, "SET:RCUT,", 9)) {
    int m = 0;
    if (sscanf(line + 9, "%d", &m) == 1)
      gRpmCutMode = (m != 0) ? 1u : 0u;
    { char b[40]; snprintf(b, sizeof b, "OK:RCUT,%u\r\n", (unsigned)gRpmCutMode); uartWrite(b); }
  } else if (!strncmp(line, "SET:EOI,", 8)) {
    float e = 60.0f;
    if (sscanf(line + 8, "%f", &e) == 1) {
      if (e < 10.0f) e = 10.0f;
      if (e > 400.0f) e = 400.0f;
      gEoiBtdc = e;
    }
    { char b[32]; snprintf(b, sizeof b, "OK:EOI,%.1f\r\n", (double)gEoiBtdc); uartWrite(b); }
  } else if (!strncmp(line, "SET:ASE,", 8)) {
    /* SET:ASE,initialPct,decaySec,minEctC */
    float ip = aseInitialPct, ds = aseDecaySec, me = aseMinEct;
    int n = sscanf(line + 8, "%f,%f,%f", &ip, &ds, &me);
    if (n >= 1) { if (ip < 0) ip = 0; if (ip > 150) ip = 150; aseInitialPct = ip; }
    if (n >= 2) { if (ds < 1) ds = 1; if (ds > 120) ds = 120; aseDecaySec = ds; }
    if (n >= 3) { aseMinEct = me; }
    { char b[64]; snprintf(b,sizeof b,"OK:ASE,%.0f,%.0f,%.0f\r\n",
      (double)aseInitialPct,(double)aseDecaySec,(double)aseMinEct); uartWrite(b); }
  } else if (!strncmp(line, "SET:CSE,", 8)) {
    /* SET:CSE,i,tempC,pctAdd */
    int i = 0; float te = 0, pct = 0;
    if (sscanf(line + 8, "%d,%f,%f", &i, &te, &pct) == 3 && i >= 0 && i < CSE_N) {
      cseTemp[i] = te;
      if (pct < 0) pct = 0; if (pct > 150) pct = 150;
      csePct[i] = pct;
      { char b[48]; snprintf(b,sizeof b,"OK:CSE,%d,%.0f,%.0f\r\n",i,(double)te,(double)pct); uartWrite(b); }
    }
  } else if (!strncmp(line, "SET:MAPCAL,", 11)) {
    int i = 0; float kpa = 0, adc = 0;
    if (sscanf(line + 11, "%d,%f,%f", &i, &kpa, &adc) == 3 && i >= 0 && i < MAP_CAL_N) {
      mapCalKpa[i] = kpa; mapCalAdc[i] = adc; mapCalReady = 1;
      { char b[48]; snprintf(b,sizeof b,"OK:MAPCAL,%d,%.1f,%.0f\r\n",i,(double)kpa,(double)adc); uartWrite(b); }
    }
  } else if (!strncmp(line, "SET:BATV,", 9)) {
    int i = 0; float v = 0;
    if (sscanf(line + 9, "%d,%f", &i, &v) == 2 && i >= 0 && i < BAT_CAL_N) {
      batVoltTbl[i] = v; batCalReady = 1;
      { char b[40]; snprintf(b,sizeof b,"OK:BATV,%d,%.2f\r\n",i,(double)v); uartWrite(b); }
    }
  } else if (!strncmp(line, "SET:BATA,", 9)) {
    int i = 0; float a = 0;
    if (sscanf(line + 9, "%d,%f", &i, &a) == 2 && i >= 0 && i < BAT_CAL_N) {
      batAdcTbl[i] = a; batCalReady = 1;
      { char b[40]; snprintf(b,sizeof b,"OK:BATA,%d,%.0f\r\n",i,(double)a); uartWrite(b); }
    }
  } else if (!strncmp(line, "SET:BATC,", 9)) {
    int i = 0; float c = 1;
    if (sscanf(line + 9, "%d,%f", &i, &c) == 2 && i >= 0 && i < BAT_CAL_N) {
      batCompTbl[i] = c;
      { char b[40]; snprintf(b,sizeof b,"OK:BATC,%d,%.3f\r\n",i,(double)c); uartWrite(b); }
    }
  } else if (!strncmp(line, "SET:COIL,", 9)) {
    int s = 1; sscanf(line + 9, "%d", &s);
    gCoilSmart = s ? 1 : 0;
    uartWrite(gCoilSmart ? "OK:COIL,SMART\r\n" : "OK:COIL,DUMB\r\n");
  } else if (!strncmp(line, "SET:DBW,", 8)) {
    int e = 1; sscanf(line + 8, "%d", &e);
    gDbwEnable = e ? 1 : 0;
    etbEnable = gDbwEnable;
    uartWrite(gDbwEnable ? "OK:DBW,1\r\n" : "OK:DBW,0\r\n");
  } else if (!strncmp(line, "SET:IDLEOUT,", 12)) {
    int m = 0; sscanf(line + 12, "%d", &m);
    if (m < 0) m = 0; if (m > 2) m = 2;
    gIdleOutMode = (uint8_t)m;
    { char b[32]; snprintf(b,sizeof b,"OK:IDLEOUT,%u\r\n",(unsigned)gIdleOutMode); uartWrite(b); }
  } else if (!strncmp(line, "SET:CYL,", 8)) {
    int c = 4; sscanf(line + 8, "%d", &c);
    if (c < 1) c = 1; if (c > MAX_CYL) c = MAX_CYL;
    gCyl = (uint8_t)c;
    { char b[24]; snprintf(b,sizeof b,"OK:CYL,%u\r\n",(unsigned)gCyl); uartWrite(b); }
  } else if (!strncmp(line, "SET:FIRE,", 9)) {
    int f = 0; sscanf(line + 9, "%d", &f);
    if (f < 0) f = 0; if (f > 2) f = 2;
    gFireOrder = (uint8_t)f;
    { char b[24]; snprintf(b,sizeof b,"OK:FIRE,%u\r\n",(unsigned)gFireOrder); uartWrite(b); }
  } else if (!strncmp(line, "SET:VVTCL,", 9)) {
    int en = 1; sscanf(line + 9, "%d", &en);
    vvtClEnable = en ? 1 : 0;
    uartWrite(vvtClEnable ? "OK:VVTCL,1\r\n" : "OK:VVTCL,0\r\n");
  } else if (!strncmp(line, "SET:VVTIN,", 9)) {
    /* SET:VVTIN,r,c,val  8x8 cell */
    int r=0,c=0,v=0;
    if (sscanf(line+9, "%d,%d,%d", &r, &c, &v) == 3
        && r >= 0 && r < VVT_MAP_N && c >= 0 && c < VVT_MAP_N) {
      if (v < 0) v = 0;
      if (v > 50) v = 50;
      vvtInMap[r][c] = (int8_t)v;
      { char b[40]; snprintf(b,sizeof b,"OK:VVTIN,%d,%d,%d\r\n",r,c,v); uartWrite(b); }
    }
  } else if (!strncmp(line, "SET:VVTEX,", 9)) {
    int r=0,c=0,v=0;
    if (sscanf(line+9, "%d,%d,%d", &r, &c, &v) == 3
        && r >= 0 && r < VVT_MAP_N && c >= 0 && c < VVT_MAP_N) {
      if (v < 0) v = 0;
      if (v > 50) v = 50;
      vvtExMap[r][c] = (int8_t)v;
      { char b[40]; snprintf(b,sizeof b,"OK:VVTEX,%d,%d,%d\r\n",r,c,v); uartWrite(b); }
    }
  } else if (!strncmp(line, "SET:VVT,", 8)) {
    int a = 0, b = 0;
    if (sscanf(line + 8, "%d,%d", &a, &b) >= 1)
      ECU_SetVVT((uint8_t)a, (uint8_t)b);
    { char buf[40]; snprintf(buf, sizeof buf, "OK:VVT,%u,%u\r\n",
             (unsigned)vvt1Duty, (unsigned)vvt2Duty); uartWrite(buf); }
  } else if (!strncmp(line, "SET:O2CL,", 9)) {
    int en = 1;
    if (sscanf(line + 9, "%d", &en) == 1) ECU_EnableO2CL(en ? 1 : 0);
    uartWrite(o2ClEnable ? "OK:O2CL,1\r\n" : "OK:O2CL,0\r\n");
  } else if (!strncmp(line, "SET:STFT,", 9)) {
    float v = 0;
    if (sscanf(line + 9, "%f", &v) == 1) {
      if (v > STFT_MAX) v = STFT_MAX;
      if (v < -STFT_MAX) v = -STFT_MAX;
      stftPct = v;
    }
    { char b[32]; snprintf(b, sizeof b, "OK:STFT,%.1f\r\n", stftPct); uartWrite(b); }
  } else if (!strncmp(line, "GET:TRIM", 8)) {
    { char b[80]; snprintf(b, sizeof b,
      "OK:TRIM,STFT:%.1f,LTFT:%.1f,TOTAL:%.1f,CL:%u\r\n",
      stftPct, ltftPct, totalTrimPct(), (unsigned)o2ClActive); uartWrite(b); }
  } else if (!strncmp(line, "SET:LTFT,", 9)) {
    float v = 0;
    if (sscanf(line + 9, "%f", &v) == 1) ECU_SetLTFT(v);
    { char b[32]; snprintf(b, sizeof b, "OK:LTFT,%.1f\r\n", ltftPct); uartWrite(b); }
  } else if (!strncmp(line, "TRIMRESET", 9)) {
    ECU_ResetFuelTrim();
    uartWrite("OK:TRIMRESET\r\n");
  } else if (!strncmp(line, "SET:BSTMAP,", 11)) {
    int r=0,c=0; float v=0;
    if (sscanf(line+11, "%d,%d,%f", &r, &c, &v) == 3
        && r >= 0 && r < BST_N && c >= 0 && c < BST_N) {
      if (v < 0) v = 0;
      if (bstOpenLoop) { if (v > 100) v = 100; }
      else { if (v > 300) v = 300; }
      bstMap[r][c] = v;
      { char b[40]; snprintf(b,sizeof b,"OK:BSTMAP,%d,%d,%.0f\r\n",r,c,(double)v); uartWrite(b); }
    }
  } else if (!strncmp(line, "SET:ADVSLEW,", 12)) {
    float s = 200;
    if (sscanf(line + 12, "%f", &s) == 1) {
      if (s < 20) s = 20; if (s > 1000) s = 1000;
      advSlewDps = s;
    }
    { char b[32]; snprintf(b,sizeof b,"OK:ADVSLEW,%.0f\r\n",(double)advSlewDps); uartWrite(b); }
  } else if (!strncmp(line, "SET:KNKTHR,", 11)) {
    int i = 0; float v = 0;
    if (sscanf(line + 11, "%d,%f", &i, &v) == 2 && i >= 0 && i < MS_RPM_N) {
      if (v < 1) v = 1; if (v > 500) v = 500;
      knkThrTbl[i] = v; knkUseTable = 1;
      { char b[40]; snprintf(b,sizeof b,"OK:KNKTHR,%d,%.0f\r\n",i,(double)v); uartWrite(b); }
    }
  } else if (!strncmp(line, "SET:KNKMAX,", 11)) {
    int i = 0; float v = 0;
    if (sscanf(line + 11, "%d,%f", &i, &v) == 2 && i >= 0 && i < MS_RPM_N) {
      if (v < 0) v = 0; if (v > 20) v = 20;
      knkMaxTbl[i] = v; knkUseTable = 1;
      { char b[40]; snprintf(b,sizeof b,"OK:KNKMAX,%d,%.0f\r\n",i,(double)v); uartWrite(b); }
    }
  } else if (!strncmp(line, "SET:IGNLIM,", 11)) {
    float mn = -15, mx = 45;
    if (sscanf(line + 11, "%f,%f", &mn, &mx) >= 1) {
      if (mn < -30) mn = -30; if (mn > 10) mn = 10;
      if (mx < 10) mx = 10; if (mx > 60) mx = 60;
      if (mx <= mn) mx = mn + 5;
      gIgnMinAdv = mn; gIgnMaxAdv = mx;
    }
    { char b[40]; snprintf(b,sizeof b,"OK:IGNLIM,%.0f,%.0f\r\n",(double)gIgnMinAdv,(double)gIgnMaxAdv); uartWrite(b); }
  } else if (!strncmp(line, "GET:IGNLIM", 10)) {
    { char b[40]; snprintf(b,sizeof b,"OK:IGNLIM,%.0f,%.0f\r\n",(double)gIgnMinAdv,(double)gIgnMaxAdv); uartWrite(b); }
  } else if (!strncmp(line, "SET:FAN,", 8)) {
    float on = 95, hyst = 5;
    if (sscanf(line + 8, "%f,%f", &on, &hyst) >= 1) {
      if (on < 60) on = 60; if (on > 130) on = 130;
      if (hyst < 1) hyst = 1; if (hyst > 20) hyst = 20;
      gFanOnC = on; gFanOffC = on - hyst;
    }
    { char b[40]; snprintf(b,sizeof b,"OK:FAN,%.0f,%.0f\r\n",(double)gFanOnC,(double)gFanOffC); uartWrite(b); }
  } else if (!strncmp(line, "GET:FAN", 7)) {
    { char b[40]; snprintf(b,sizeof b,"OK:FAN,%.0f,%.0f\r\n",(double)gFanOnC,(double)gFanOffC); uartWrite(b); }
  } else if (!strncmp(line, "SET:KNK,", 8)) {
    /* SET:KNK,en,threshold,stepDeg,maxRetard */
    int en = 1; float thr = knkThreshold, step = knkStepDeg, mx = knkMaxRetard;
    int n = sscanf(line + 8, "%d,%f,%f,%f", &en, &thr, &step, &mx);
    if (n >= 1) knkEnable = en ? 1 : 0;
    if (n >= 2) { knkThreshold = thr; knkUseTable = 0; }
    if (n >= 3) { if (step < 0.5f) step = 0.5f; if (step > 5) step = 5; knkStepDeg = step; }
    if (n >= 4) { if (mx < 0) mx = 0; if (mx > 20) mx = 20; knkMaxRetard = mx; }
    { char b[64]; snprintf(b,sizeof b,"OK:KNK,%u,%.1f,%.1f,%.1f\r\n",
      (unsigned)knkEnable,(double)knkThreshold,(double)knkStepDeg,(double)knkMaxRetard);
      uartWrite(b); }
  } else if (!strncmp(line, "SET:LAUNCH,", 11)) {
    /* SET:LAUNCH,en,rpm,tpsMin,boostKpa */
    int en = 0; float rpm = 4000, tps = 80, bst = 50;
    int n = sscanf(line + 11, "%d,%f,%f,%f", &en, &rpm, &tps, &bst);
    if (n >= 1) launchEnable = en ? 1 : 0;
    if (n >= 2) launchRpm = rpm;
    if (n >= 3) launchTpsMin = tps;
    if (n >= 4) launchBoostKpa = bst;
    { char b[64]; snprintf(b,sizeof b,"OK:LAUNCH,%u,%.0f,%.0f,%.0f\r\n",
      (unsigned)launchEnable, (double)launchRpm, (double)launchTpsMin, (double)launchBoostKpa);
      uartWrite(b); }
  } else if (!strncmp(line, "SET:ALSFUELTBL,", 14)) {
    int i = 0; float p = 0;
    if (sscanf(line + 14, "%d,%f", &i, &p) == 2 && i >= 0 && i < MS_RPM_N) {
      if (p < 0) p = 0; if (p > 100) p = 100;
      alsFuelTbl[i] = p; alsFuelUseTable = 1;
      { char b[40]; snprintf(b,sizeof b,"OK:ALSFUELTBL,%d,%.0f\r\n",i,(double)p); uartWrite(b); }
    }
  } else if (!strncmp(line, "SET:ALSFUEL,", 12)) {
    float p = 40;
    if (sscanf(line + 12, "%f", &p) == 1) {
      if (p < 0) p = 0; if (p > 100) p = 100;
      alsFuelPct = p; alsFuelUseTable = 0;
    }
    { char b[32]; snprintf(b,sizeof b,"OK:ALSFUEL,%.0f\r\n",(double)alsFuelPct); uartWrite(b); }
  } else if (!strncmp(line, "SET:ALSTBL,", 11)) {
    int i = 0; float d = 0;
    if (sscanf(line + 11, "%d,%f", &i, &d) == 2 && i >= 0 && i < MS_RPM_N) {
      if (d < 0) d = 0; if (d > 40) d = 40;
      alsRetardTbl[i] = d; alsUseTable = 1;
      { char b[40]; snprintf(b,sizeof b,"OK:ALSTBL,%d,%.0f\r\n",i,(double)d); uartWrite(b); }
    }
  } else if (!strncmp(line, "SET:FFSTBL,", 11)) {
    int i = 0; float d = 0;
    if (sscanf(line + 11, "%d,%f", &i, &d) == 2 && i >= 0 && i < MS_RPM_N) {
      if (d < 0) d = 0; if (d > 40) d = 40;
      ffsRetardTbl[i] = d; ffsUseTable = 1;
      { char b[40]; snprintf(b,sizeof b,"OK:FFSTBL,%d,%.0f\r\n",i,(double)d); uartWrite(b); }
    }
  } else if (!strncmp(line, "SET:ALS,", 8)) {
    /* SET:ALS,en,retardDeg,exVvt[,maxSec[,coolSec]] */
    int en = 0, ex = 1; float ret = 15, mx = alsMaxSec, cool = alsCooldownSec;
    int n = sscanf(line + 8, "%d,%f,%d,%f,%f", &en, &ret, &ex, &mx, &cool);
    if (n >= 1) alsEnable = en ? 1 : 0;
    if (n >= 2) { if (ret < 0) ret = 0; if (ret > 40) ret = 40; alsRetardDeg = ret; }
    if (n >= 3) alsExVvt = ex ? 1 : 0;
    if (n >= 4) { if (mx < 0.5f) mx = 0.5f; if (mx > 30.0f) mx = 30.0f; alsMaxSec = mx; }
    if (n >= 5) { if (cool < 0.0f) cool = 0.0f; if (cool > 60.0f) cool = 60.0f; alsCooldownSec = cool; }
    alsStartMs = 0; alsBlockUntilMs = 0; alsTimedOut = 0;
    { char b[72]; snprintf(b,sizeof b,"OK:ALS,%u,%.0f,%u,%.1f,%.1f\r\n",
      (unsigned)alsEnable, (double)alsRetardDeg, (unsigned)alsExVvt,
      (double)alsMaxSec, (double)alsCooldownSec); uartWrite(b); }
  } else if (!strncmp(line, "SET:FFS,", 8)) {
    /* SET:FFS,en,tpsMin,retardDeg */
    int en = 0; float tps = 70, ret = 20;
    int n = sscanf(line + 8, "%d,%f,%f", &en, &tps, &ret);
    if (n >= 1) ffsEnable = en ? 1 : 0;
    if (n >= 2) ffsTpsMin = tps;
    if (n >= 3) { if (ret < 0) ret = 0; if (ret > 40) ret = 40; ffsRetardDeg = ret; }
    { char b[48]; snprintf(b,sizeof b,"OK:FFS,%u,%.0f,%.0f\r\n",
      (unsigned)ffsEnable, (double)ffsTpsMin, (double)ffsRetardDeg); uartWrite(b); }
  } else if (!strncmp(line, "SET:BSTMODE,", 12)) {
    int m = 0; sscanf(line + 12, "%d", &m);
    bstOpenLoop = m ? 1 : 0;
    boostIntegral = 0.0f;
    uartWrite(bstOpenLoop ? "OK:BSTMODE,OPEN\r\n" : "OK:BSTMODE,CLOSED\r\n");
  } else if (!strncmp(line, "SET:BSTEN,", 10)) {
    int e = 1; sscanf(line+10, "%d", &e);
    bstMapEnable = e ? 1 : 0;
    uartWrite(bstMapEnable ? "OK:BSTEN,1\r\n" : "OK:BSTEN,0\r\n");
  } else if (!strncmp(line, "SET:BOOST,", 10)) {
    float bk = 0;
    if (sscanf(line + 10, "%f", &bk) == 1)
      ECU_SetBoostTarget(bk);
    {
      char b[32];
      snprintf(b, sizeof b, "OK:BOOST,%.1f\r\n", boostTargetKpa);
      uartWrite(b);
    }
  } else if (!strncmp(line, "SET:TPS,CLOSED", 14)) {
    /* SET:TPS,CLOSED          → use live ADC
       SET:TPS,CLOSED,<adc>    → explicit value */
    int v = -1;
    if (line[14] == ',' && sscanf(line + 15, "%d", &v) == 1 && v >= 0 && v <= 4095)
      tpsClosedAdc = (uint16_t)v;
    else
      tpsClosedAdc = adcTps;
    tpsCalValid = (tpsOpenAdc > tpsClosedAdc + 50) ? 1 : 0;
    {
      char b[48];
      snprintf(b, sizeof b, "OK:TPS,CLOSED,%u\r\n", (unsigned)tpsClosedAdc);
      uartWrite(b);
    }
  } else if (!strncmp(line, "SET:TPS,OPEN", 12)) {
    int v = -1;
    if (line[12] == ',' && sscanf(line + 13, "%d", &v) == 1 && v >= 0 && v <= 4095)
      tpsOpenAdc = (uint16_t)v;
    else
      tpsOpenAdc = adcTps;
    tpsCalValid = (tpsOpenAdc > tpsClosedAdc + 50) ? 1 : 0;
    {
      char b[48];
      snprintf(b, sizeof b, "OK:TPS,OPEN,%u\r\n", (unsigned)tpsOpenAdc);
      uartWrite(b);
    }
  } else if (!strncmp(line, "SET:PED,CLOSED", 14)) {
    int v = -1;
    if (line[14] == ',' && sscanf(line + 15, "%d", &v) == 1 && v >= 0 && v <= 4095)
      pedClosedAdc = (uint16_t)v;
    else
      pedClosedAdc = adcPedal;
    {
      char b[48];
      snprintf(b, sizeof b, "OK:PED,CLOSED,%u\r\n", (unsigned)pedClosedAdc);
      uartWrite(b);
    }
  } else if (!strncmp(line, "SET:PED,OPEN", 12)) {
    int v = -1;
    if (line[12] == ',' && sscanf(line + 13, "%d", &v) == 1 && v >= 0 && v <= 4095)
      pedOpenAdc = (uint16_t)v;
    else
      pedOpenAdc = adcPedal;
    {
      char b[48];
      snprintf(b, sizeof b, "OK:PED,OPEN,%u\r\n", (unsigned)pedOpenAdc);
      uartWrite(b);
    }
  } else if (!strncmp(line, "GET:TPSCAL", 10) || !strncmp(line, "GETTPSCAL", 9)) {
    char b[80];
    snprintf(b, sizeof b, "TPSCAL:%u,%u,PED:%u,%u,VALID:%u\r\n",
             (unsigned)tpsClosedAdc, (unsigned)tpsOpenAdc,
             (unsigned)pedClosedAdc, (unsigned)pedOpenAdc,
             (unsigned)tpsCalValid);
    uartWrite(b);
  } else if (!strncmp(line, "RESET", 5) || !strncmp(line, "REBOOT", 6)
           || !strncmp(line, "SET:RESET", 9)) {
    uartWrite("OK:RESET\r\n");
    /* Allow USB TX to flush before hard reset */
    HAL_Delay(80);
    NVIC_SystemReset();
  } else if (!strncmp(line, "SAVE", 4)) {
    EcuFlashBlob blob;
    memset(&blob, 0, sizeof blob);
    blob.tpsClosed = tpsClosedAdc;
    blob.tpsOpen   = tpsOpenAdc;
    blob.pedClosed = pedClosedAdc;
    blob.pedOpen   = pedOpenAdc;
    blob.trigAngle = gTrigAngle;
    blob.teeth     = gTeeth;
    blob.missing   = gMissing;
    /* Persist LTFT only (STFT is transient) */
    {
      float x = ltftPct * 100.0f;
      if (x >  2500.0f) x =  2500.0f;
      if (x < -2500.0f) x = -2500.0f;
      blob.ltftCenti = (int16_t)x;
    }
    for (uint8_t r = 0; r < ROWS; r++)
      for (uint8_t c = 0; c < COLS; c++) {
        blob.advMap[r][c] = advMap[r][c];
        blob.injMap[r][c] = injMap[r][c];
      }
    int err = ECU_Flash_Save(&blob);
    if (err == 0) {
      char b[40];
      snprintf(b, sizeof b, "OK:SAVE,MAPS,LTFT:%.1f\r\n", ltftPct);
      uartWrite(b);
    } else {
      char b[24];
      {
        const char *why = "FAIL";
        if (err == -2) why = "ERASE";
        else if (err == -3) why = "PROGRAM";
        else if (err == -4) why = "VERIFY";
        else if (err == -5) why = "SIZE";
        else if (err == -1) why = "NULL";
        snprintf(b, sizeof b, "ERR:SAVE,%d,%s\r\n", err, why);
      }
      uartWrite(b);
    }
  }
}

void ECU_UART_RxByte(uint8_t b) {
  if (b == '\n' || b == '\r') {
    if (rxLen) { rxBuf[rxLen] = 0; handleLine(rxBuf); rxLen = 0; }
  } else if (rxLen < sizeof(rxBuf) - 1) {
    rxBuf[rxLen++] = (char)b;
  } else rxLen = 0;
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
      for (uint8_t r = 0; r < ROWS; r++)
        for (uint8_t c = 0; c < COLS; c++) {
          advMap[r][c] = blob.advMap[r][c];
          injMap[r][c] = blob.injMap[r][c];
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
    /* stalled — allow ASE again on next start if still cold */
    if (rpmLive < 200)
      aseActive = 0;
  }
  wasRunning = running;
  if (aseActive && engEct >= (aseMinEct + 10.0f)) {
    /* fully warm — cancel remaining ASE */
    aseActive = 0;
  }
}


/**
 * Unified ignition timing:
 *   final = base_map − soft_limit − ALS − FFS − knock
 *   slew-rate limited, clamped to [−15, +45]
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
  /* Time-critical first: coils / injectors before slow ADC & closed-loop */
  scheduleCoils(micros());
  serviceInjection();

  readSensors(); /* only 2 ADC channels per pass */

  float load = gUseTps ? (engTps / 100.0f) : (engMap / 100.0f);
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

  if (syncLocked && (micros() - lastToothUs) > 2000000UL) {
    /* 2 s without tooth = stalled (allows slow crank) */
    syncLocked = 0;
    syncLosses++;
    rpmLive = 0;
    gapConfirm = 0;
    allOutputsOff();
  }
  /* Cam stays locked unless engine truly stopped > 3 s */
  if (camSynced && (micros() - lastToothUs) > 5000000UL) {
    camSynced = 0;
  }
  {
    static uint32_t lastCam2Check;
    /* cam2 sticky — clear if no edges for 5 s when engine running */
    if (cam2Synced && rpmLive > 200 && (micros() - lastToothUs) > 5000000UL)
      cam2Synced = 0;
    (void)lastCam2Check;
  }

  float v = engBat;
  if (v < 8) v = 8;
  if (v > 16) v = 16;
  float dus = (float)CFG_DWELL_NOM_US * (14.0f / v);
  if (dus < CFG_DWELL_MIN_US) dus = CFG_DWELL_MIN_US;
  if (dus > CFG_DWELL_MAX_US) dus = CFG_DWELL_MAX_US;
  dwellTargetUs = (uint16_t)dus;

  serviceO2ClosedLoop();
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
