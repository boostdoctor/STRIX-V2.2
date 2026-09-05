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
#endif
#ifndef MS_RPM_N
#define MS_RPM_N 8
#endif
#ifndef VVT_MAP_N
#define VVT_MAP_N 8
#endif

#ifndef O2_MODE_OFF
#define O2_MODE_OFF 0
#define O2_MODE_NB  1
#define O2_MODE_WB  2
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
#endif

#ifndef BAT_CAL_N
#define BAT_CAL_N  15
#endif
#ifndef MAP_CAL_N
#define MAP_CAL_N  15
#endif
#ifndef CSE_N
#define CSE_N  10
#endif

#ifndef DTC_MAX_ACTIVE
#define DTC_MAX_ACTIVE 16
#endif

typedef struct {
  uint16_t code;
  uint8_t  active;
  uint32_t setMs;
} DtcSlot;

#ifdef __cplusplus
extern "C" {
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
extern float   gVePct; /* live VE% from map */
extern uint8_t sensorPhase;

/* Config live */
extern volatile uint8_t  gTeeth, gMissing;
extern volatile uint16_t gTrigAngle, gRpmLimit;
extern volatile uint8_t  gRpmCutMode, rpmCutActive;
extern volatile uint8_t  gUseTps, gLoadMode, gCyl, gCoilSmart, gDbwEnable, gIdleOutMode;
extern volatile uint8_t  gFireOrder;        /* 0=1-3-4-2 1=1-2-4-3 2=1-3-2-4 */
extern volatile uint8_t  gInjMode;
extern volatile uint8_t  gIgnMode;          /* 0=wasted 1=sequential */
extern volatile uint8_t  gCoilType;         /* 0=smart 1=dumb 2=dist */
extern volatile uint8_t  gCoilChargeMode;   /* 0=duty 1=charge */
extern volatile uint16_t gDwellNomUs;
extern volatile uint8_t  gSparkDouble;
extern volatile uint8_t  gSparkDblGapDeg;
extern volatile uint16_t gBatchAboveRpm;
extern float gMapLoadRefKpa;
extern float gMapKpaMin; /* ADC 0 */
extern float gMapKpaMax; /* ADC 4095 = sensor max kPa */

/* Sync / RPM */
extern volatile uint32_t lastToothUs, lastGapUs, toothPeriodUs, toothPeriodFilt;
extern volatile uint16_t toothIndex, syncLosses, toothErrors, rpmLive;
extern volatile uint16_t crankRevId; /* ++ on accepted missing-tooth gap */
extern volatile uint8_t  syncLocked, camSynced, cam2Synced;
extern volatile float    crankDeg;
extern volatile uint8_t  cycleHalf;
extern volatile uint32_t crankEdgeCount;
extern uint8_t gWheelId, gCamMode;
extern float cam1PhaseDeg, cam2PhaseDeg;
extern volatile uint32_t lastCamEdgeUs, lastCam2EdgeUs;
extern volatile uint8_t  camLockHits, camUnlockMiss, cam2LockHits, cam2UnlockMiss;
/* Crank PLL-style lock state machine */
enum {
  CRANK_PLL_SEEK = 0,    /* looking for first valid gap */
  CRANK_PLL_CONFIRM = 1, /* collecting consecutive good gaps */
  CRANK_PLL_LOCKED = 2,  /* hard lock — normal operation */
  CRANK_PLL_SOFTERR = 3  /* locked but accumulating errors */
};
extern volatile uint8_t  crankPllState;
extern volatile uint8_t  pllSoftErr, pllGoodStreak, gapConfirm;
extern volatile uint16_t teethSinceGap;
extern volatile uint8_t  gapRejectStreak;
extern volatile uint8_t  missedGapStreak; /* consecutive expected gaps not seen */
extern volatile uint8_t  missedGapArmed;  /* 1 = already counted this overshoot */
extern volatile uint8_t  coilFired[MAX_CYL + 1];

/* Kalman RPM filter state */
extern float kf_rpm, kf_acc, kf_p00, kf_p01, kf_p10, kf_p11;
extern float kf_nis_ema, kf_R_adapt, kf_q_adapt;
extern uint8_t kf_ready;

/* Sensors eng units + raw ADC */
extern float engMap, engTps, engEct, engIat, engBat, engO2, engPedal;
extern float engAfr, engEthanol, engVssKph;
extern float engFlexHz; /* PA6 flex sensor Hz */
extern uint16_t adcEct, adcTps, adcBat, adcIat, adcMap, adcO2, adcPedal;
extern uint16_t adcFlex;
extern uint8_t  gFlexEnable;
extern uint16_t gFlexAdcE0, gFlexAdcE100;
extern float    gFlexFuelPctPer10, gFlexIgnDegPer10;
extern volatile int16_t ignAdvanceDeg;
extern float totalRetardDeg, softLimitRetardDeg;
extern float advSlewDps, gIgnMinAdv, gIgnMaxAdv;
extern volatile uint16_t injPwUs, dwellActualUs, dwellTargetUs;

/* Outputs / primes */
extern uint8_t fanOn, fpOn;
extern uint8_t gTachoEnable, gTachoPpr;
extern volatile uint8_t camPulseSeen;
extern volatile uint8_t outTestActive, outTestStep;
extern volatile uint32_t outTestNextMs;
extern uint16_t gFpPrimeMs, gInjPrimeMs;
extern uint8_t  gInjPrimeEn, injPrimeDone, injPrimeActive;
extern uint32_t fpPrimeUntilMs, injPrimeEndMs, lastZeroRpmMs;
extern float gFanOnC, gFanOffC;
extern uint8_t gFanEnable;
extern uint8_t vvt1Duty, vvt2Duty;

/* Idle */
extern uint8_t idleEnable, idleActive;
extern float idleTargetRpm, idleThrottle, dashpotPct;
extern float idleTgtEctBins[5];
extern float idleTgtRpmTbl[5];
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
extern uint8_t injDisableMask;          /* bit0=cyl1 … diagnostic kill */
extern uint8_t crankAdvEnable;
extern float   crankAdvDeg;
extern uint16_t crankAdvRpm;
extern uint8_t floodClearEnable, floodClearActive;
extern float   floodClearTps;
#define AFR_MAP_ROWS 12
#define AFR_MAP_COLS 22
extern float afrMap[AFR_MAP_ROWS][AFR_MAP_COLS];
extern uint8_t afrMapEnable;
#define IDLE_MAP_N 5
extern float idleFuelMap[IDLE_MAP_N][IDLE_MAP_N];  /* ECT × RPM % */
extern float idleIgnMap[IDLE_MAP_N][IDLE_MAP_N];   /* ECT × RPM ° */
extern const float idleRpmBins[IDLE_MAP_N];
extern const float idleEctBins[IDLE_MAP_N];
float idleFuelLookup(float ectC, float rpm);
float idleIgnLookup(float ectC, float rpm);
float afrTargetLookup(float load, float rpm);


/* DTC */
extern DtcSlot dtcList[DTC_MAX_ACTIVE];
extern uint8_t dtcCount;
extern uint32_t lastDtcEvalMs, o2StuckSameMs;



/* VSS (PC15) */
extern uint8_t  vssEnable;
extern uint16_t vssPulsesPerKm;          /* pulses per kilometre */
extern volatile uint32_t vssPulseCount;
extern float    engVssKph;

/* Launch VSS decay curves (fuel % add, retard ° vs vehicle speed) */
#ifndef LC_VSS_N
#define LC_VSS_N  8
#endif
extern uint8_t  launchDecayEnable;
extern uint8_t  launchDecayActive;
extern float    launchDecayFuelPct;      /* currently applied */
extern float    launchDecayRetardDeg;
extern float    launchVssBins[LC_VSS_N];
extern float    launchFuelTbl[LC_VSS_N]; /* extra fuel % */
extern float    launchRetardTbl[LC_VSS_N];
float launchFuelFromVss(float kph);
float launchRetardFromVss(float kph);
void  serviceVss(void);
void  ECU_Vss_IrqEdge(void);

/* Boost */
extern float boostTargetKpa, boostIntegral, boostPrevErr;
extern uint8_t boostEnable, bstMapEnable, bstOpenLoop, boostDutyRaisesBoost;
extern uint8_t gVeMode;
extern float gInjFlowCcMin, gReqFuelMs;
extern int8_t gMaxAdvDeg;   /* max advance BTDC */
extern int8_t gMaxRetDeg;   /* max retard magnitude (ATDC) */
extern float gMaxInjMs;     /* max injection pulse ms */
extern float gInjDeadMs;    /* injector deadtime ms @ 13.2V */
extern float gFuelPressureBar;       /* actual rail bar */
extern float gFuelPressureRatedBar;  /* rated flow pressure bar */
extern uint8_t  aeEnable;
extern float    aeTpsDotThresh, aeGain, aeMaxPct, aePctLive, aePrevTps;
extern uint16_t aeDecayMs;
extern uint32_t aeLastMs, aeDecayUntilMs;
float accelEnrichMul(void);
void  serviceAccelEnrich(void);
extern float BOOST_KP, BOOST_KI, BOOST_KD, BOOST_MAX_KPA, BOOST_MIN_DUTY, BOOST_MAX_DUTY;
extern float BOOST_FF_GAIN, BOOST_ARM_KPA, BOOST_I_LIM;
extern float baroKpa, boostDutyOut;
extern uint32_t boostLastMs;
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
extern volatile uint32_t persistDueMs;
extern volatile int8_t saveLastErr;
void ECU_Persist_Touch(void);
void ECU_Persist_Service(void);
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
#endif

#endif /* ECU_RUNTIME_H */
