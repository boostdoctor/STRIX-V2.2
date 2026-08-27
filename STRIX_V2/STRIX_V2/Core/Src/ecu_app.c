/* ecu_app.c — auto-split from ecu_app.c */
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
#include "ecu_watchdog.h"
#include "ecu_adc.h"
#include <string.h>
#include <stdio.h>
#include <math.h>
#include <stdint.h>

/* ---- lines 325-397 ---- */
float gVePct = 0.0f;

void ecuInjGpioInit(void)
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

void allOutputsOff(void) {
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

/* 4-cyl firing orders (gFireOrder 0..2). 6-cyl even-fire 1-5-3-6-2-4. */
static const uint8_t order4[3][4] = {
  {1, 3, 4, 2},
  {1, 2, 4, 3},
  {1, 3, 2, 4}
};
static const uint8_t order6[6] = {1, 5, 3, 6, 2, 4};

uint8_t cylAtSlot(uint8_t slot)
{
  if (gCyl <= 4) {
    uint8_t fo = (gFireOrder <= 2) ? gFireOrder : 0;
    return order4[fo][slot % 4];
  }
  return order6[slot % 6];
}

/* Compression TDC on a 720° cycle. Slot 0 (cyl 1) is 0°. */
float tdcDeg(uint8_t cyl)
{
  uint8_t n = gCyl;
  if (n < 1) n = 1;
  if (n > MAX_CYL) n = MAX_CYL;
  float step = 720.0f / (float)n;
  for (uint8_t s = 0; s < n; s++) {
    if (cylAtSlot(s) == cyl)
      return step * (float)s;
  }
  return 0.0f;
}

/* Sequential ignition only with cam home. Else wasted spark. */
uint8_t ignSequentialActive(void)
{
  return (gIgnMode == 1 && camSynced) ? 1u : 0u;
}

/* Sequential injection only with cam home. Else batch. */
uint8_t injSequentialActive(void)
{
  if (!camSynced)
    return 0;
  if (gInjMode == 2)
    return 1u;
  if (gInjMode == 3)
    return (rpmLive < gBatchAboveRpm) ? 1u : 0u;
  return 0;
}

/* ── Cam (720° phase) ───────────────────────────────────────── */

/* ---- lines 3675-3778 ---- */
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
  /* V2.3 boot stages:
   *  1. trigger  — wheel teeth/missing from flash or default
   *  2. sensors  — ADC DMA + MAP/TPS buffer
   *  3. actuators — GPIO / timers off until sync
   *  4. math     — maps + bins
   *  5. core     — IC IRQs + watchdog
   */
  ECU_Serial_Init();
  fpPrimeUntilMs = millis() + (uint32_t)gFpPrimeMs; /* fuel pump prime on power-up */

  /* Timer-triggered (TIM9) or continuous DMA ADC scan — see CUBEMX_ADC_DMA.md */
  ECU_Adc_Init();
  ECU_Flex_Init(); /* PA6 frequency flex sensor */

  vvtMapsDefault();
  {
    __HAL_RCC_GPIOB_CLK_ENABLE();
    GPIO_InitTypeDef g = {0};
    g.Pin = GPIO_PIN_13;
    g.Mode = GPIO_MODE_INPUT;
    g.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOB, &g); /* clutch switch */
    /* VSS PC15 — input with pull-up; EXTI optional, software edge in serviceVss */
    {
      GPIO_InitTypeDef vg = {0};
      __HAL_RCC_GPIOC_CLK_ENABLE();
      vg.Pin = VSS_Pin;
      vg.Mode = GPIO_MODE_INPUT;
      vg.Pull = GPIO_PULLUP;
      HAL_GPIO_Init(VSS_GPIO_Port, &vg);
    }
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
  ECU_ApplyWheelId(9); /* default 60-2+cam */
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

  defaultMaps();
  ECU_Flash_CrcInit();
  ECU_Features_Init();
  for (uint8_t i = 0; i < COLS; i++) rpmBinsLive[i] = rpmBins[i];
  for (uint8_t i = 0; i < ROWS; i++) mapBinsLive[i] = mapBins[i];
  ECU_SanitizeMapBins();
  for (uint8_t r = 0; r < ETB_ROWS; r++)
    for (uint8_t c = 0; c < ETB_COLS; c++) {
      /* default linear pedal response */
      etbMap[r][c] = (uint8_t)((c * 100) / (ETB_COLS - 1));
    }
  gCyl = CFG_CYLINDERS;
  if (gCyl > MAX_CYL) gCyl = MAX_CYL;
  ecuInjGpioInit(); /* reclaim PB4 from JTAG NJTRST */
  allOutputsOff();

  /* CRITICAL: arm crank/cam input-capture IRQs (PA0 TIM5, PA15 TIM2, PB4 TIM3) */
  ECU_CrankCam_Start();

  /* IWDG ~1 s — must be kicked from ECU_Loop / flash */
  ECU_Watchdog_Init();
  ECU_Watchdog_Kick();

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
      ECU_Trigger_Rebuild(gTeeth, gMissing, gCamMode, gWheelId);
      ltftPct = (float)blob.ltftCenti / 100.0f;
      if (ltftPct >  25.0f) ltftPct =  25.0f;
      if (ltftPct < -25.0f) ltftPct = -25.0f;
      stftPct = 0.0f; /* always start STFT fresh */
      if (blob.version >= 8u)
        ECU_Settings_Apply(&blob.settings);
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
      ECU_Flash_ApplyExtras(&blob);
    }
  }
}



/* ---- lines 3981-4109 ---- */
void ECU_Loop(void) {
  ECU_Watchdog_Kick();
  ECU_Serial_Service();
  ECU_Persist_Service();
  servicePendingSave(); /* flash SAVE queued by serial / auto-persist */
  /* Time-critical first: coils / injectors before slow ADC & closed-loop */
  scheduleCoils(micros());
  serviceInjection();

  readSensors(); /* only 2 ADC channels per pass */

  /* Load for maps + LOAD: telemetry (MAP / TPS / hybrid + RPM) */
  float load = calcEngineLoad();
  engLoad = load;
  int8_t adv;
  float injMs;
  lookupMaps(load, (float)rpmLive, &adv, &injMs);
  ignAdvanceDeg = computeIgnitionAdvance(adv);
  {
    int8_t a = ignAdvanceDeg;
    if (a > gMaxAdvDeg) a = gMaxAdvDeg;
    if (a < -(int8_t)gMaxRetDeg) a = (int8_t)(-gMaxRetDeg);
    ignAdvanceDeg = a;
  }
  serviceAfterStart();
  /* Fuel: direct ms map, or VE% → base PW from MAP/IAT/reqFuel */
  float pw;
  if (gVeMode) {
    float ve = injMs; /* lookup returned VE % when gVeMode */
    if (ve < 0.0f) ve = 0.0f;
    if (ve > 150.0f) ve = 150.0f;
    gVePct = ve;
    float mapAbs = engMap;
    if (mapAbs < 20.0f) mapAbs = 20.0f;
    float iatK = engIat + 273.15f;
    if (iatK < 250.0f) iatK = 250.0f;
    /* Ideal-gas style density vs 100 kPa / 293 K reference */
    float dens = (mapAbs / 100.0f) * (293.15f / iatK);
    float baseMs = gReqFuelMs * (ve / 100.0f) * dens;
    /* Optional scale by injector size vs nominal 220 cc/min */
    {
      float pAct = gFuelPressureBar;
      float pRat = gFuelPressureRatedBar;
      if (pAct < 0.5f) pAct = 0.5f;
      if (pRat < 0.5f) pRat = 0.5f;
      float flowEff = gInjFlowCcMin;
      if (flowEff < 10.0f) flowEff = 10.0f;
      /* flow ∝ √(P_rail / P_rated) */
      flowEff *= sqrtf(pAct / pRat);
      baseMs *= (220.0f / flowEff);
    }
    pw = baseMs * 1000.0f * o2FuelMul() * coldStartEnrichMul()
       * afterStartMul() * alsFuelMul() * accelEnrichMul();
  } else {
    gVePct = injMs; /* duty-mode cell (ms) — tuner labels INJ */
    pw = injMs * 1000.0f * o2FuelMul() * coldStartEnrichMul()
       * afterStartMul() * alsFuelMul() * accelEnrichMul();
  }
  if (launchDecayActive && launchDecayFuelPct > 0.1f) {
    pw *= (1.0f + launchDecayFuelPct * 0.01f);
  }
  if (pw < 1000) pw = 1000; /* min injector duty 1.0 ms */
  {
    float mxUs = gMaxInjMs * 1000.0f;
    if (mxUs < 1000.0f) mxUs = 1000.0f;
    if (mxUs > 30000.0f) mxUs = 30000.0f;
    if (pw > mxUs) pw = mxUs;
  }
  if (dfcoActive)
    pw = 0; /* deceleration fuel cut */
  injPwUs = (uint16_t)pw;

  /* RPM hold: if no tooth for 200 ms, force RPM to 0 (stops slow coast-up) */
  if (lastToothUs != 0 && (micros() - lastToothUs) > 200000UL) {
    if (rpmLive != 0) {
      rpmLive = 0;
      kf_rpm = 0.0f;
      kf_acc = 0.0f;
    }
  }
  if (lastToothUs != 0 && (micros() - lastToothUs) > 800000UL) {
    /* 0.5 s without accepted tooth = stalled / unlock */
    if (syncLocked) syncLosses++;
    syncLocked = 0;
    crankPllState = CRANK_PLL_SEEK;
    gapConfirm = 0;
    pllSoftErr = 0;
    missedGapStreak = 0;
    missedGapArmed = 0;
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
        /* Leave soft: need 5 consecutive timeout ticks OR 4× expected period */
        if (camUnlockMiss >= 5 || (nowu - lastCamEdgeUs) > (camTimeout * 4UL)) {
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
        if (cam2UnlockMiss >= 5 || (nowu - lastCam2EdgeUs) > (camTimeout * 4UL)) {
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
  serviceAccelEnrich();
  serviceETB();
  serviceVss();
  serviceMotorsport();
  serviceBoost();
  serviceOutputs();
  serviceStartPrime();

  static uint32_t lastTel = 0;
  uint32_t now_ms = millis();
  if (mapDumpBusy && (now_ms - lastTel) > 1500U)
    mapDumpBusy = 0;
  if (!mapDumpBusy && (now_ms - lastTel) >= 50U) /* ~20 Hz — less USB TX pressure */ {
    lastTel = now_ms;
    sendTelemetry();
  }
}

void ECU_1kHzTick(void) {}
