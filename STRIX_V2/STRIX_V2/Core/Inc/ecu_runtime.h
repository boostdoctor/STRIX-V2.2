/**
 * Shared runtime state for split STRIX V2 modules.
 * Definitions live in ecu_runtime.c
 */
#ifndef ECU_RUNTIME_H
#define ECU_RUNTIME_H

#include <stdint.h>
#include "ecu_config.h"
#include "ecu_flash.h"
#include "stm32f4xx_hal.h"

#define ROWS 12
#define COLS 22
#define STRIX_PROTO_VER 2
#define MAX_CYL CFG_MAX_COILS
#define ETB_ROWS 16
#define ETB_COLS 17

#ifndef BST_N
#define BST_N 8
extern volatile uint8_t  outTestActive;
extern volatile uint8_t  outTestStep;
extern volatile uint32_t outTestNextMs;
#endif
#ifndef MS_RPM_N
#define MS_RPM_N 8
extern volatile uint8_t  outTestActive;
extern volatile uint8_t  outTestStep;
extern volatile uint32_t outTestNextMs;
#endif
#ifndef VVT_MAP_N
#define VVT_MAP_N 8
extern volatile uint8_t  outTestActive;
extern volatile uint8_t  outTestStep;
extern volatile uint32_t outTestNextMs;
#endif

#ifndef O2_MODE_OFF
#define O2_MODE_OFF 0
#define O2_MODE_NB  1
#define O2_MODE_WB  2
extern volatile uint8_t  outTestActive;
extern volatile uint8_t  outTestStep;
extern volatile uint32_t outTestNextMs;
#endif

/* Knock Goertzel */
#ifndef KNK_WIN_N
#define KNK_WIN_N     64
#define KNK_FS_HZ     50000.0f
#define KNK_F1_HZ     7000.0f
#define KNK_F2_HZ     10000.0f
extern volatile uint8_t  outTestActive;
extern volatile uint8_t  outTestStep;
extern volatile uint32_t outTestNextMs;
#endif

/* DTC codes */
#ifndef DTC_NONE
#define DTC_NONE        0u
#define DTC_MAP_LOW     1u
#define DTC_MAP_HIGH    2u
#define DTC_TPS_RANGE   3u
#define DTC_ECT_RANGE   4u
#define DTC_ECT_OPEN    5u
#define DTC_ECT_HIGH    6u
#define DTC_IAT_RANGE   7u
#define DTC_IAT_OPEN    8u
#define DTC_BAT_LOW     9u
#define DTC_BAT_HIGH    10u
#define DTC_O2_STUCK    11u
#define DTC_AFR_RANGE   12u
#define DTC_SYNC_LOSS   13u
#define DTC_CAM_LOSS    14u
extern volatile uint8_t  outTestActive;
extern volatile uint8_t  outTestStep;
extern volatile uint32_t outTestNextMs;
#endif

#ifndef BAT_CAL_N
#define BAT_CAL_N  15
extern volatile uint8_t  outTestActive;
extern volatile uint8_t  outTestStep;
extern volatile uint32_t outTestNextMs;
#endif
#ifndef MAP_CAL_N
#define MAP_CAL_N  15
extern volatile uint8_t  outTestActive;
extern volatile uint8_t  outTestStep;
extern volatile uint32_t outTestNextMs;
#endif
#ifndef CSE_N
#define CSE_N  10
extern volatile uint8_t  outTestActive;
extern volatile uint8_t  outTestStep;
extern volatile uint32_t outTestNextMs;
#endif

#ifndef DTC_MAX_ACTIVE
#define DTC_MAX_ACTIVE 16
extern volatile uint8_t  outTestActive;
extern volatile uint8_t  outTestStep;
extern volatile uint32_t outTestNextMs;
#endif

typedef struct {
  uint16_t code;
  uint8_t  active;
  uint32_t setMs;
} DtcSlot;

#ifdef __cplusplus
extern "C" {
extern volatile uint8_t  outTestActive;
extern volatile uint8_t  outTestStep;
extern volatile uint32_t outTestNextMs;
#endif

/* Prefer DWT micros when CYCCNT enabled in ECU_Init */
static inline uint32_t millis(void) { return HAL_GetTick(); }
static inline uint32_t micros(void)
{
  if (DWT->CTRL & DWT_CTRL_CYCCNTENA_Msk)
    return DWT->CYCCNT / (SystemCoreClock / 1000000U);
  return HAL_GetTick() * 1000u;
}

/* Maps */
extern const float rpmBins[COLS];
extern const float mapBins[ROWS];
extern int8_t  advMap[ROWS][COLS];
extern uint8_t injMap[ROWS][COLS];
extern float   rpmBinsLive[COLS];
extern float   mapBinsLive[ROWS];
extern uint8_t etbMap[ETB_ROWS][ETB_COLS];
extern float   engLoad;
extern uint8_t sensEctEn, sensIatEn, sensO2En, sensMapEn, sensTpsEn;
extern uint8_t mapCellR, mapCellC;
extern int8_t  baseAdvDeg;
extern float   baseInjMs;
extern uint8_t sensorPhase;

/* Config live */
extern volatile uint8_t  gTeeth, gMissing;
extern volatile uint16_t gTrigAngle, gRpmLimit;
extern volatile uint8_t  gRpmCutMode, rpmCutActive;
extern volatile uint8_t  gUseTps, gLoadMode, gCyl, gCoilSmart, gDbwEnable, gIdleOutMode;
extern volatile uint8_t  gInjMode;
extern volatile uint8_t  gIgnMode;
extern volatile uint8_t  gCoilChargeMode; /* 0=const duty 1=const charge(time) */
extern volatile uint8_t  gCoilType; /* 0=smart 1=dumb 2=dist */ /* 0 wasted, 1 sequential */
extern volatile uint16_t gBatchAboveRpm;
extern float gMapLoadRefKpa;

/* Sync / RPM */
extern volatile uint32_t lastToothUs, lastGapUs, toothPeriodUs, toothPeriodFilt;
extern volatile uint16_t toothIndex, syncLosses, toothErrors, rpmLive;
extern volatile uint8_t  syncLocked, camSynced, cam2Synced;
extern volatile uint8_t  camPulseSeen; /* live-strip edge activity */
extern volatile float    crankDeg;
extern volatile uint8_t  cycleHalf;
extern volatile uint32_t crankEdgeCount;
extern uint8_t gWheelId, gCamMode;
extern float cam1PhaseDeg, cam2PhaseDeg;
extern volatile uint32_t lastCamEdgeUs, lastCam2EdgeUs;
extern volatile uint8_t  camLockHits, camUnlockMiss, cam2LockHits, cam2UnlockMiss;
extern volatile uint8_t  pllSoftErr, pllGoodStreak, gapConfirm;
extern volatile uint16_t teethSinceGap;
extern volatile uint8_t  gapRejectStreak;
extern volatile uint8_t  coilFired[MAX_CYL + 1];

/* Kalman RPM filter state */
extern float kf_rpm, kf_acc, kf_p00, kf_p01, kf_p10, kf_p11;
extern float kf_nis_ema, kf_R_adapt, kf_q_adapt;
extern uint8_t kf_ready;

/* Sensors eng units + raw ADC */
extern float engMap, engTps, engEct, engIat, engBat, engO2, engKnock, engPedal;
extern float engAfr;
extern uint16_t adcEct, adcTps, adcBat, adcIat, adcMap, adcO2, adcKnock, adcPedal;
extern volatile int16_t ignAdvanceDeg;
extern float totalRetardDeg, softLimitRetardDeg;
extern float advSlewDps, gIgnMinAdv, gIgnMaxAdv;
extern volatile uint16_t injPwUs, dwellActualUs, dwellTargetUs;

/* Outputs / primes */
extern uint8_t fanOn, fpOn;
extern volatile uint8_t gFanEnable;
extern volatile uint8_t gTachoEnable;
extern volatile uint8_t gTachoPpr;
extern uint16_t gFpPrimeMs, gInjPrimeMs;
extern uint8_t  gInjPrimeEn, injPrimeDone, injPrimeActive;
extern uint32_t fpPrimeUntilMs, injPrimeEndMs, lastZeroRpmMs;
extern float gFanOnC, gFanOffC;
extern uint8_t vvt1Duty, vvt2Duty;

/* Idle */
extern uint8_t idleEnable, idleActive;
extern float idleTargetRpm, idleThrottle, dashpotPct;
extern float idleIntegral, idlePrevRpmErr, prevTpsIdle;
extern uint32_t idleLastMs;
extern float IDLE_KP, IDLE_KI, IDLE_KD;
extern float IDLE_MAX_PCT, IDLE_MIN_PCT, IDLE_ENTRY_PEDAL, IDLE_EXIT_PEDAL, IDLE_ENTRY_TPS;
extern float DASHPOT_GAIN, DASHPOT_DECAY, DASHPOT_MAX, DASHPOT_MIN_DTPS, DASHPOT_MIN_TPS;

/* ETB */
extern float etbTargetPct, etbIntegral, etbPrevErr;
extern uint8_t etbEnable;
extern float ETB_KP, ETB_KI, ETB_KD, ETB_IDLE_PCT;

/* DFCO / ASE */
extern uint8_t dfcoEnable, dfcoActive;
extern uint16_t dfcoEnterRpm, dfcoExitRpm, dfcoDelayMs;
extern float dfcoMaxTps, dfcoMinEct;
extern uint32_t dfcoEnterMs;
extern float aseInitialPct, aseDecaySec, aseMinEct;
extern uint32_t aseStartMs;
extern uint8_t aseActive, wasRunning;

/* O2 / fuel trim */
extern uint8_t o2SensorMode, o2ClActive, o2ClEnable;
extern float stftPct, ltftPct, STFT_MAX, LTFT_RATE, LTFT_MAX;
extern float wbAfrMin, wbAfrMax, wbVMax, stoichAfr, targetAfr;
extern float o2Filt, o2StuckLast;
extern float cylTrimPct[MAX_CYL + 1];

/* DTC */
extern DtcSlot dtcList[DTC_MAX_ACTIVE];
extern uint8_t dtcCount;
extern uint32_t lastDtcEvalMs, o2StuckSameMs;

/* Knock */
extern float knkBuf[KNK_WIN_N];
extern uint16_t knkIdx;
extern uint8_t knkCollecting, knkEnable, knkUseTable;
extern float knkIntensity, knkThreshold, knockRetardDeg;
extern float knkStepDeg, knkRestoreDps, knkMaxRetard;
extern float knkThrTbl[MS_RPM_N], knkMaxTbl[MS_RPM_N];

/* Boost */
extern float boostTargetKpa, boostIntegral, boostPrevErr;
extern uint8_t boostEnable, bstMapEnable, bstOpenLoop, boostDutyRaisesBoost;
extern float BOOST_KP, BOOST_KI, BOOST_KD, BOOST_MAX_KPA, BOOST_MIN_DUTY, BOOST_MAX_DUTY;
extern float bstMap[BST_N][BST_N];
extern const float bstRpm[BST_N];
extern const float bstTps[BST_N];

/* Motorsport */
extern uint8_t launchEnable, launchActive, alsEnable, alsActive, alsTimedOut;
extern uint8_t alsExVvt, alsFuelUseTable, alsUseTable, ffsUseTable;
extern uint8_t ffsEnable, ffsActive, clutchPressed;
extern float launchRpm, launchTpsMin, launchBoostKpa;
extern float alsRetardDeg, alsFuelPct, alsMaxSec, alsCooldownSec;
extern float alsRetardTbl[MS_RPM_N], ffsRetardTbl[MS_RPM_N], alsFuelTbl[MS_RPM_N];
extern const float msRpmBins[MS_RPM_N];
extern uint32_t alsStartMs, alsBlockUntilMs;
extern float ffsTpsMin, ffsRetardDeg;

/* VVT maps */
extern int8_t vvtInMap[VVT_MAP_N][VVT_MAP_N];
extern int8_t vvtExMap[VVT_MAP_N][VVT_MAP_N];
extern float vvtInIntegral, vvtExIntegral, vvtInPrevErr, vvtExPrevErr;
extern uint8_t vvtClEnable;
extern float VVT_KP, VVT_KI, VVT_KD;
extern const float vvtRpmBins[VVT_MAP_N];
extern const float vvtLoadBins[VVT_MAP_N];

/* Serial / flash */
extern uint8_t uploadMode, uploadRow;
extern volatile uint8_t mapDumpBusy;
extern volatile uint8_t savePending, mapsDirty;
extern volatile int8_t saveLastErr;
extern char rxBuf[192];
extern uint8_t rxLen;
extern char gDeviceUid[16];

/* Cal tables */
extern float batVoltTbl[BAT_CAL_N], batAdcTbl[BAT_CAL_N], batCompTbl[BAT_CAL_N];
extern uint8_t batCalReady;
extern float mapCalKpa[MAP_CAL_N], mapCalAdc[MAP_CAL_N];
extern uint8_t mapCalReady;
extern uint16_t tpsClosedAdc, tpsOpenAdc, pedClosedAdc, pedOpenAdc;
extern uint8_t tpsCalValid;

/* Coil / inj state */
extern volatile uint8_t coilState[];
extern volatile uint32_t coilStartUs[];
extern uint8_t injOn[];
extern volatile uint8_t injReq[];
extern uint8_t injFiredCyc[];
extern uint32_t injEndUs[];
extern float gEoiBtdc;
extern int16_t advTargetDeg;
extern float cseTemp[CSE_N];
extern float csePct[CSE_N];
extern uint32_t o2RichMs, o2LeanMs, o2LastMs;
extern float O2_RICH_V, O2_LEAN_V, STFT_STEP;

/* Helpers used across modules */
float msRetardLookup(const float *tbl, float rpm);
uint8_t readClutch(void);
float Goertzel_KnockIntensity(const float *x, int n, float fs, float f1, float f2);
float afrToLambda(float afr);
float lambdaToAfr(float lam);
float computeIgnitionAdvance(int8_t mapAdv);
float o2FuelMul(void);

void ECU_Features_Init(void);
void ECU_Features_Service(void);
void ECU_Dtc_Clear(void);
uint8_t ECU_Dtc_Count(void);
uint16_t ECU_Dtc_Get(uint8_t index);
void ECU_SetCylTrim(uint8_t cyl, float pct);
float ECU_GetCylTrim(uint8_t cyl);

#ifdef __cplusplus
}
extern volatile uint8_t  outTestActive;
extern volatile uint8_t  outTestStep;
extern volatile uint32_t outTestNextMs;
#endif

extern volatile uint8_t  outTestActive;
extern volatile uint8_t  outTestStep;
extern volatile uint32_t outTestNextMs;
#endif /* ECU_RUNTIME_H */
