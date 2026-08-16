/* STRIX V2 firmware — 12x22 maps, sensor enables, GETUID */
/**
 * TorquEFI Basic - STM32F411 full sequential ECU core
 * Cam (PA15) + crank (PA0) → 720° phase; per-cylinder spark & inject.
 */
#include "main.h"
#include "ecu_config.h"
#include "ecu_pins.h"
#include "ecu_runtime.h"
#include "ecu_wheels.h"
#include "stm32f4xx_hal.h"
#include <string.h>
#include <stdio.h>
#include <math.h>

/* 22-column RPM axis (matches tuner: 250 + c*375, 250-8125 RPM) */
const float rpmBins[COLS] = {
  250,  625, 1000, 1375, 1750, 2125, 2500, 2875,
 3250, 3625, 4000, 4375, 4750, 5125, 5500, 5875,
 6250, 6625, 7000, 7375, 7750, 8125
};
/* 15-row load axis (0.10-1.08 normalised) */
/* 12-row load axis: 20..240 kPa default (engMap kPa/100 → 0.20..2.40) */
const float mapBins[ROWS] = {
  0.20f, 0.40f, 0.60f, 0.80f, 1.00f, 1.20f,
  1.40f, 1.60f, 1.80f, 2.00f, 2.20f, 2.40f
};

int8_t  advMap[ROWS][COLS];
uint8_t injMap[ROWS][COLS];
/* Per-cylinder fuel trim % (-25..+25), index 1..MAX_CYL */
float cylTrimPct[MAX_CYL + 1];
/* Live breakpoints (overwritten by tuner SET:RPMB / SET:MAPB) */
float rpmBinsLive[COLS];
float mapBinsLive[ROWS];
/* Pedal→throttle target map: 16 RPM bands × 17 pedal points */
#define ETB_ROWS 16
#define ETB_COLS 17
uint8_t etbMap[ETB_ROWS][ETB_COLS];
float engLoad = 0.0f;
/* Sensor enables — 0 = out of fuel/timing calc (still telemetered) */
uint8_t sensEctEn = 1;
uint8_t sensIatEn = 1;
uint8_t sensO2En  = 0;
uint8_t sensMapEn = 1;
uint8_t sensTpsEn = 1;
char gDeviceUid[16] = "STRIXV2";
uint8_t mapCellR = 0, mapCellC = 0; /* last lookup cell */
int8_t  baseAdvDeg = 0;   /* map only, pre-retard */
float   baseInjMs  = 0;   /* map only, pre-trim */
uint8_t sensorPhase = 0;


volatile uint8_t  gTeeth = 60;
volatile uint8_t  gMissing = 2;
volatile uint16_t gTrigAngle = CFG_TRIG_ANGLE;
volatile uint16_t gRpmLimit = CFG_RPM_LIMIT;
volatile uint8_t  gRpmCutMode = 0; /* 0=hard cut, 1=soft cut */
volatile uint8_t  rpmCutActive = 0;

/* DFCO - declared early for serviceInjection */
uint8_t  dfcoEnable   = 1;
uint8_t  dfcoActive   = 0;
uint16_t dfcoEnterRpm = 1600;
uint16_t dfcoExitRpm  = 1200;
float    dfcoMaxTps   = 3.0f;
float    dfcoMinEct   = 50.0f;
uint32_t dfcoEnterMs  = 0;
uint16_t dfcoDelayMs  = 200;
volatile uint8_t  gUseTps = 0; /* 1=Alpha-N (TPS), 0=Speed-density (MAP) */
/* 0=MAP, 1=TPS, 2=Hybrid MAP+TPS+RPM — SET:L sets this */
volatile uint8_t  gLoadMode = 0;
/* MAP reference for load=1.0 (kPa). 100 = atmospheric baseline */
float gMapLoadRefKpa = 100.0f;
volatile uint8_t  gCyl = CFG_CYLINDERS;
volatile uint8_t  gCoilSmart = 1;  /* 1=smart 0=dumb */
volatile uint8_t  gDbwEnable = 1;  /* 0=idle actuator only */
volatile uint8_t  gIdleOutMode = 0; /* 0=2wire 1=1wire 2=stepper */
volatile uint8_t  gFireOrder = 0;  /* 0=1-3-4-2 1=1-2-4-3 2=1-3-2-4 */
/* BAT_CAL_N / MAP_CAL_N / CSE_N from ecu_runtime.h */
float batVoltTbl[BAT_CAL_N];
float batAdcTbl[BAT_CAL_N];
float batCompTbl[BAT_CAL_N];
uint8_t batCalReady = 0;
float mapCalKpa[MAP_CAL_N];
float mapCalAdc[MAP_CAL_N];
uint8_t mapCalReady = 0;
/* Cold-start fuel enrichment vs ECT (°C → extra % fuel) */
float cseTemp[CSE_N] = {-20,0,10,20,30,40,50,60,70,80};
float csePct[CSE_N]  = {80,55,40,28,18,12,7,3,0,0}; /* % add */
/* After-start enrichment: extra % that decays to 0 over time */
float aseInitialPct = 35.0f;   /* % extra at start */
float aseDecaySec   = 25.0f;   /* seconds to decay to 0 */
float aseMinEct     = 60.0f;   /* skip ASE if already warm */
uint32_t aseStartMs = 0;
uint8_t  aseActive  = 0;
uint8_t  wasRunning = 0;
/* Injection mode: 0=AUTO 1=BATCH 2=SEQUENTIAL 3=HYBRID (seq below RPM, batch above) */
volatile uint8_t  gInjMode = 1; /* 1=batch 2=seq 3=hybrid */
volatile uint8_t  gIgnMode = 0; /* 0=wasted spark 1=sequential */
volatile uint16_t gBatchAboveRpm = 3000; /* hybrid switch point */

/* Crank / cam */
volatile uint32_t lastToothUs = 0, lastGapUs = 0;
volatile uint16_t toothIndex = 0;
volatile uint8_t  syncLocked = 0, camSynced = 0, cam2Synced = 0;
float  cam1PhaseDeg = 0.0f;
float  cam2PhaseDeg = 0.0f;
volatile uint16_t syncLosses = 0, toothErrors = 0;
uint8_t syncQualityPct(void)
{
  if (!syncLocked) return 0;
  uint16_t te = toothErrors;
  uint16_t sl = syncLosses;
  int q = 100;
  if (te > 50) q -= 40;
  else if (te > 20) q -= 25;
  else if (te > 5) q -= 10;
  if (sl > 10) q -= 30;
  else if (sl > 3) q -= 15;
  if (!camSynced) q -= 10;
  if (q < 0) q = 0;
  if (q > 100) q = 100;
  return (uint8_t)q;
}
volatile uint32_t toothPeriodUs = 0;
volatile uint32_t toothPeriodFilt = 0; /* EMA-smoothed tooth us */
/* 2-state Kalman filter for RPM: x=[rpm, accel(rpm/s)] */
float kf_rpm = 0.0f;
float kf_acc = 0.0f;
float kf_p00 = 1.0e4f, kf_p01 = 0.0f, kf_p10 = 0.0f, kf_p11 = 1.0e4f;
uint8_t kf_ready = 0;
float kf_nis_ema = 1.0f;   /* EMA of normalized innovation² */
float kf_R_adapt = 150.0f; /* adapted measurement noise */
float kf_q_adapt = 12000.0f; /* adapted accel spectral density */
volatile uint32_t lastCamEdgeUs = 0;   /* for cam timeout */
volatile uint32_t lastCam2EdgeUs = 0;
volatile uint8_t  camLockHits = 0;     /* hysteresis to lock */
volatile uint8_t  camUnlockMiss = 0;   /* hysteresis to unlock */
volatile uint8_t  cam2LockHits = 0;
volatile uint8_t  cam2UnlockMiss = 0;
volatile uint16_t rpmLive = 0;
uint8_t gWheelId = 9; /* default 60-2 + 1-tooth cam */ /* default 60-2 + 1-tooth cam */
volatile uint8_t mapDumpBusy = 0;
uint8_t gCamMode = 1; /* cam sensor present */ /* cam sensor present (60-2+cam); sequential still needs gIgnMode */

/* Bulk-upload state (UPLOAD:ADV / UPLOAD:INJ from tuner) */
uint8_t uploadMode = 0;   /* 0=idle  1=ADV  2=INJ */
volatile uint8_t savePending = 0; /* 1 = do flash in ECU_Loop */
volatile uint8_t mapsDirty  = 0; /* RAM maps changed since load/save */
volatile int8_t  saveLastErr = 0;

uint8_t uploadRow  = 0;
volatile uint32_t crankEdgeCount = 0;
volatile float    crankDeg = 0.0f;   /* 0..720 when sequential */
volatile uint8_t  cycleHalf = 0;     /* 0 or 1 from cam */
volatile uint8_t  pllSoftErr = 0, pllGoodStreak = 0;
volatile uint8_t  gapConfirm = 0;
volatile uint16_t teethSinceGap = 0; /* physical teeth since last gap */
volatile uint8_t  gapRejectStreak = 0;

volatile int16_t  ignAdvanceDeg = 10; /* signed: negative = ATDC */
float softLimitRetardDeg = 0.0f;
float totalRetardDeg = 0.0f;
int16_t advTargetDeg = 10;
float advSlewDps = 200.0f; /* max °/s change of final advance */
float gIgnMinAdv = -15.0f;
float gIgnMaxAdv = 45.0f;
volatile uint16_t injPwUs = 2000;
volatile uint16_t dwellTargetUs = CFG_DWELL_NOM_US;
volatile uint16_t dwellActualUs = 0;

/* Per-cylinder coil / injector */
volatile uint8_t  coilState[MAX_CYL+1];
volatile uint32_t coilStartUs[MAX_CYL+1];
volatile uint8_t  coilFired[MAX_CYL+1];
volatile uint8_t  injReq[MAX_CYL+1];
uint8_t injFiredCyc[MAX_CYL+1];
#ifndef CFG_EOI_BTDC_DEG
#define CFG_EOI_BTDC_DEG 60.0f
#endif
float gEoiBtdc = CFG_EOI_BTDC_DEG;
uint8_t  injOn[MAX_CYL+1];
uint32_t injEndUs[MAX_CYL+1];

uint8_t fanOn = 0, fpOn = 0;
volatile uint8_t gFanEnable = 0; /* IO must enable fan */
volatile uint8_t gTachoEnable = 0;
volatile uint8_t gTachoPpr = 2; /* pulses per crank rev */
/* tacho edge state lives in service path */
/* Fuel-pump prime (ms after power-up or SET:FPPRIME) */
uint16_t gFpPrimeMs = 2000;
uint32_t fpPrimeUntilMs = 0;
/* Start injector prime pulse (once per cranking session) */
uint16_t gInjPrimeMs = 50;   /* pulse width ms */
uint8_t  gInjPrimeEn = 1;
uint8_t  injPrimeDone = 0;
uint32_t injPrimeEndMs = 0;
uint8_t  injPrimeActive = 0;
uint32_t lastZeroRpmMs = 0;
float gFanOnC  = 95.0f;
float gFanOffC = 90.0f; /* hysteresis: off below on-hyst */
/* VVT duty 0-100% via TIM1 PWM (period 1000 counts) */
uint8_t vvt1Duty = 0, vvt2Duty = 0;
extern TIM_HandleTypeDef htim1;
/* Closed-loop ETB */
float etbTargetPct = -1.0f;  /* <0 = follow pedal; 0-100 = override */
float etbIntegral  = 0.0f;
float etbPrevErr   = 0.0f;
uint8_t etbEnable = 1;
/* PID gains - tune on vehicle */
float ETB_KP = 4.0f;
float ETB_KI = 8.0f;
float ETB_KD = 0.05f;
float ETB_IDLE_PCT = 3.0f;  /* min open when running */

float engMap, engTps, engEct, engIat, engBat, engO2, engKnock, engPedal;
/* Goertzel knock */
#define KNK_WIN_N     64
#define KNK_FS_HZ     50000.0f
#define KNK_F1_HZ     7000.0f
#define KNK_F2_HZ     10000.0f
float    knkBuf[KNK_WIN_N];
uint16_t knkIdx = 0;
uint8_t  knkCollecting = 0;
float    knkIntensity = 0.0f;
float    knkThreshold = 50.0f;  /* scale depends on sensor gain */
float    knockRetardDeg = 0.0f;
float    knkStepDeg = 2.0f;
float    knkRestoreDps = 5.0f;  /* degrees per second restore */
float    knkMaxRetard = 12.0f;
uint8_t  knkEnable = 1;
uint16_t adcMap, adcTps, adcEct, adcIat, adcBat, adcO2, adcKnock, adcPedal;
/* TPS / pedal endpoint calibration (12-bit ADC) */
uint16_t tpsClosedAdc  = 400;
uint16_t tpsOpenAdc    = 3600;
uint16_t pedClosedAdc  = 400;
uint16_t pedOpenAdc    = 3600;
uint8_t  tpsCalValid   = 0;


char rxBuf[192];
uint8_t rxLen = 0;



/* Idle PID gains (shared) */
float IDLE_KP = 0.012f;
float IDLE_KI = 0.008f;
float IDLE_KD = 0.002f;
uint8_t idleEnable = 1;
uint8_t idleActive = 0;
float idleTargetRpm = 850.0f;
float idleThrottle = 0.0f;
float prevTpsIdle = 0.0f;
uint32_t idleLastMs = 0;
float idleIntegral = 0.0f;
float idlePrevRpmErr = 0.0f;
float dashpotPct = 0.0f;

/* from monolith lines 1411-1513 */
float boostTargetKpa = 0.0f;   /* gauge target; 0 = open-loop off */
float boostIntegral  = 0.0f;
float boostPrevErr   = 0.0f;
uint8_t boostEnable  = 1;
float BOOST_KP = 1.2f;
float BOOST_KI = 0.4f;
float BOOST_KD = 0.02f;
float BOOST_MAX_KPA = 250.0f;  /* absolute MAP safety (includes atm) */
float BOOST_MIN_DUTY = 0.0f;
float BOOST_MAX_DUTY = 85.0f;  /* leave headroom */
/* 1 = more duty raises boost (vent WG top); 0 = inverted */
uint8_t boostDutyRaisesBoost = 1;
float bstMap[BST_N][BST_N]; /* gauge kPa target, RPM×TPS */
const float bstRpm[BST_N] = {1500,2000,2500,3000,3500,4000,5000,6000};
const float bstTps[BST_N] = {20,30,40,50,60,70,80,100};
uint8_t bstMapEnable = 1;
uint8_t bstOpenLoop = 0; /* 0=CL target kPa  1=OL duty % */
/* Launch / ALS / Flat-foot */
uint8_t  launchEnable = 0;
float    launchRpm = 4000.0f;
float    launchTpsMin = 80.0f;
float    launchBoostKpa = 50.0f;
uint8_t  launchActive = 0;
uint8_t  alsEnable = 0;
float    alsRetardDeg = 15.0f; /* fallback if table unused */
uint8_t  alsExVvt = 1;
const float msRpmBins[MS_RPM_N] = {1500,2000,2500,3000,4000,5000,6000,7000};
float alsRetardTbl[MS_RPM_N] = {10,12,15,18,20,22,25,25};
float ffsRetardTbl[MS_RPM_N] = {12,14,16,18,20,22,22,20};
/* ALS extra fuel % vs RPM (on top of base PW when ALS active) */
float alsFuelTbl[MS_RPM_N] = {25,30,35,40,45,50,50,45};
float alsFuelPct = 40.0f; /* fallback single value */
uint8_t alsFuelUseTable = 1;
float knkThrTbl[MS_RPM_N] = {40,45,50,55,60,70,80,90};
float knkMaxTbl[MS_RPM_N] = {8,10,12,12,14,14,12,10};
uint8_t knkUseTable = 1;
uint8_t alsUseTable = 1;
uint8_t ffsUseTable = 1;
uint8_t  alsActive = 0;
float    alsMaxSec = 3.0f;      /* max continuous ALS time */
float    alsCooldownSec = 5.0f;    /* block re-entry after timeout */
uint32_t alsStartMs = 0;
uint32_t alsBlockUntilMs = 0;
uint8_t  alsTimedOut = 0;
uint8_t  ffsEnable = 0;
float    ffsTpsMin = 70.0f;
float    ffsRetardDeg = 20.0f;
uint8_t  ffsActive = 0;
uint8_t  clutchPressed = 0;
extern TIM_HandleTypeDef htim4;


/* ── Narrowband O2 closed-loop fuel trim ─────────────────────── */
uint8_t  o2ClEnable   = 1;
uint8_t  o2ClActive   = 0;
float    stftPct      = 0.0f;   /* short-term trim -25..+25 % */
float    ltftPct      = 0.0f;   /* long-term trim  -25..+25 % */
float    o2Filt       = 0.45f;
uint32_t o2RichMs     = 0;
uint32_t o2LeanMs     = 0;
uint32_t o2LastMs     = 0;
/* Voltage thresholds (NB zirconia ~0.1-0.9 V) */
float O2_RICH_V = 0.55f;
float O2_LEAN_V = 0.35f;
float STFT_STEP = 0.15f;   /* % per 10 ms tick when held rich/lean */
float STFT_MAX  = 25.0f;
float LTFT_RATE = 0.002f;  /* LTFT slowly follows STFT when active */
float LTFT_MAX  = 25.0f;

/* ── Wideband / AFR ─────────────────────────────────────────── */
uint8_t o2SensorMode = O2_MODE_NB; /* OFF / NB / WB */
float   engAfr       = 14.7f;
float   wbAfrMin     = 10.0f;  /* AFR at 0 V */
float   wbAfrMax     = 20.0f;  /* AFR at wbVMax */
float   wbVMax       = 3.3f;   /* full-scale after divider */
float   targetAfr    = 14.7f;  /* WB closed-loop target (AFR) */
float   stoichAfr    = 14.7f;  /* fuel stoich AFR: petrol 14.7, E85 ~9.8 */

float afrToLambda(float afr)
{
  return (stoichAfr > 0.1f) ? (afr / stoichAfr) : 1.0f;
}
float lambdaToAfr(float lam)
{
  return lam * stoichAfr;
}



/* ── DTC ────────────────────────────────────────────────────── */
DtcSlot dtcList[DTC_MAX_ACTIVE];
uint8_t dtcCount = 0;
uint32_t lastDtcEvalMs = 0;
uint32_t o2StuckSameMs = 0;
float    o2StuckLast   = -1.0f;

/* cylTrimPct defined near maps at top of file */

float IDLE_MAX_PCT = 18.0f;
float IDLE_MIN_PCT = 1.5f;
float IDLE_ENTRY_PEDAL = 5.0f;  /* TPS/pedal below this = idle region */
float IDLE_EXIT_PEDAL  = 8.0f;
float IDLE_ENTRY_TPS   = 5.0f;  /* hard gate: only idle if TPS < 5% */
float DASHPOT_GAIN     = 0.35f;  /* TPS drop → hold % */
float DASHPOT_DECAY    = 0.92f;  /* per 10ms-ish tick (~0.92^100 ≈ slow) */
float DASHPOT_MAX     = 25.0f;
float DASHPOT_MIN_DTPS = 6.0f;   /* min TPS drop % to trigger */
float DASHPOT_MIN_TPS  = 12.0f;  /* only if TPS was above this */

/* idleTargetFromEct() lives in ecu_idle.c */

/* ── Deceleration fuel cut (DFCO) ───────────────────────────────
 * Cut fuel when coasting: high RPM, closed throttle/pedal, warm.
 * Restore fuel before idle (hysteresis) to avoid stall.
 */
/* Overrun / deceleration fuel cut - same state as DFCO */

/* Closed-loop dual VVT (intake + exhaust), 8×8 target maps */
int8_t vvtInMap[VVT_MAP_N][VVT_MAP_N];  /* target cam ° advance */
int8_t vvtExMap[VVT_MAP_N][VVT_MAP_N];
/* cam1PhaseDeg / cam2PhaseDeg declared with sync flags above */
float  vvtInIntegral = 0.0f, vvtExIntegral = 0.0f;
float  vvtInPrevErr = 0.0f, vvtExPrevErr = 0.0f;
uint8_t vvtClEnable = 1;
float VVT_KP = 2.0f, VVT_KI = 0.4f, VVT_KD = 0.05f;

const float vvtRpmBins[VVT_MAP_N] = {
  800, 1200, 1800, 2500, 3500, 4500, 5500, 6500
};
const float vvtLoadBins[VVT_MAP_N] = {
  0.10f, 0.20f, 0.30f, 0.45f, 0.55f, 0.70f, 0.85f, 1.00f
};

/* Helpers (msRetardLookup, readClutch, o2FuelMul, computeIgnitionAdvance)
 * live in ecu_fuel.c — do not redefine here. */

/** Lightweight dual-tone energy estimate (full Goertzel optional later) */
float Goertzel_KnockIntensity(const float *x, int n, float fs, float f1, float f2)
{
  (void)fs;
  (void)f1;
  (void)f2;
  if (!x || n < 4) return 0.0f;
  float sum = 0.0f, mean = 0.0f;
  for (int i = 0; i < n; i++) mean += x[i];
  mean /= (float)n;
  for (int i = 0; i < n; i++) {
    float d = x[i] - mean;
    sum += d * d;
  }
  return sum / (float)n;
}

/* Output test sequencer (RPM must be 0) */
volatile uint8_t  outTestActive = 0;
volatile uint8_t  outTestStep = 0;
volatile uint32_t outTestNextMs = 0;

volatile uint8_t camPulseSeen = 0;

volatile uint8_t gCoilChargeMode = 0; /* 0=duty (smart default) 1=time */
volatile uint8_t gCoilType = 0; /* 0=smart */
