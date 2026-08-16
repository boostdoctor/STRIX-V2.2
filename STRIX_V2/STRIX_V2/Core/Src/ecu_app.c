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
#include "ecu_adc.h"
#include <string.h>
#include <stdio.h>
#include <math.h>
#include <stdint.h>

/* ---- lines 325-397 ---- */
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
  ECU_SetVVT(0, 0);
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

uint8_t cylAtSlot(uint8_t slot) {
  if (gCyl <= 4) return order4[slot % 4];
  return order6[slot % 6];
}

/* TDC angles ° on 720° cycle for each cylinder (cyl 1 at 0) */
float tdcDeg(uint8_t cyl) {
  /* Compression TDC on 720° cycle, firing order 1-3-4-2 (4-cyl default).
   * 2-cyl: 1@0 2@360; 3-cyl: 1@0 2@240 3@480; 6-cyl even spacing. */
  static const float t2[3] = {0, 0, 360};
  static const float t3[4] = {0, 0, 240, 480};
  static const float t4[5] = {0, 0, 540, 180, 360}; /* 1-3-4-2 */
  static const float t6[7] = {0, 0, 480, 240, 600, 120, 360};
  if (gCyl <= 2) return (cyl <= 2) ? t2[cyl] : 0.0f;
  if (gCyl == 3) return (cyl <= 3) ? t3[cyl] : 0.0f;
  if (gCyl <= 4) return (cyl <= 4) ? t4[cyl] : 0.0f;
  return (cyl >= 1 && cyl <= 6) ? t6[cyl] : 0.0f;
}



/*
 * Ignition sequential only when IGN mode = sequential AND CAM1 locked.
 * Else wasted spark (360°).
 */
uint8_t ignSequentialActive(void)
{
  if (gIgnMode != 1)
    return 0;
  if (!camSynced)
    return 0;
  return 1;
}

/*
 * Injection sequential only when INJ mode requests it AND CAM1 locked.
 * Else split-batch fuel (360°).
 * Selecting sequential injection forces gCamMode = CAM1 home.
 */
uint8_t injSequentialActive(void)
{
  if (gInjMode == 2 || gInjMode == 3) {
    if (gCamMode == 0)
      gCamMode = 1;
  }
  if (gInjMode == 1)
    return 0; /* batch */
  if (!camSynced)
    return 0;
  if (gInjMode == 2)
    return 1;
  if (gInjMode == 3) {
    if (rpmLive >= gBatchAboveRpm) return 0;
    return 1;
  }
  /* AUTO (0): follow ignition mode if cam locked */
  return (gIgnMode == 1 && camSynced) ? 1u : 0u;
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
  fpPrimeUntilMs = millis() + (uint32_t)gFpPrimeMs; /* fuel pump prime on power-up */

  /* Timer-triggered (TIM9) or continuous DMA ADC scan — see CUBEMX_ADC_DMA.md */
  ECU_Adc_Init();

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
  ECU_ApplyWheelId(9); /* V2.2 default 60-2+cam */
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

  /* CRITICAL: arm crank/cam input-capture IRQs (PA0 TIM5, PA15 TIM2, PB4 TIM3) */
  if (gWheelId == 0) gWheelId = 9;
  ECU_ApplyWheelId(gWheelId);
  ECU_CrankCam_Start();

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
    }
  }
}



/* ---- lines 3981-4109 ---- */

/* Bench output test: 500 ms each, RPM must stay 0 */
static void serviceOutTest(void)
{
  if (!outTestActive) return;
  if (rpmLive > 0 || syncLocked) {
    /* abort if engine spins */
    outTestActive = 0;
    allOutputsOff();
    return;
  }
  uint32_t now = millis();
  if (now < outTestNextMs) return;

  /* step 0.. : turn previous off, next on */
  allOutputsOff();
  /* 1..4 inj, 5..8 ign, 9 VVT1, 10 VVT2, 11 FP, 12 TACHO, 13 FAN, then done */
  switch (outTestStep) {
    case 0: break; /* start delay */
    case 1: ECU_INJ_HI(1); break;
    case 2: ECU_INJ_HI(2); break;
    case 3: ECU_INJ_HI(3); break;
    case 4: ECU_INJ_HI(4); break;
    case 5: ECU_IGN_HI(1); break;
    case 6: ECU_IGN_HI(2); break;
    case 7: ECU_IGN_HI(3); break;
    case 8: ECU_IGN_HI(4); break;
    case 9: ECU_SetVVT(50, 0); break;   /* intake VVT ~50% */
    case 10: ECU_SetVVT(0, 50); break;  /* exhaust VVT ~50% */
    case 11: ECU_FP_HI(); fpOn = 1; break;
    case 12: ECU_TACHO_HI(); break;
    case 13: ECU_FAN_HI(); fanOn = 1; break;
    default:
      outTestActive = 0;
      allOutputsOff();
      uartWrite("OK:OUTTEST:DONE\r\n");
      return;
  }
  outTestStep++;
  outTestNextMs = now + 500U;
}

void ECU_Loop(void) {
  serviceOutTest();
#if defined(HAL_IWDG_MODULE_ENABLED)
  extern IWDG_HandleTypeDef hiwdg;
  HAL_IWDG_Refresh(&hiwdg);
#endif
  ECU_Serial_Service();
  ECU_CrankPoll(); /* TIM5 CC1 if IRQ missed */
  servicePendingSave(); /* flash SAVE queued by serial */
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
    /* High RPM: allow more missed cam windows (ISR load / filter) */
    uint32_t camMul = (rpmLive > 2000) ? 4UL : 3UL;
    uint32_t camTimeout = expCam * camMul;
    if (camTimeout < 40000UL) camTimeout = 40000UL;

    /* Expire edge indicator ~150 ms after last cam edge */
  if (camPulseSeen && lastCamEdgeUs && (micros() - lastCamEdgeUs) > 150000UL)
    camPulseSeen = 0;
  if (camSynced) {
      if (lastCamEdgeUs == 0)
        lastCamEdgeUs = nowu;
      if ((nowu - lastCamEdgeUs) > camTimeout) {
        if (camUnlockMiss < 255)
          camUnlockMiss++;
        uint8_t needMiss = (rpmLive > 2000) ? 6 : 3;
        if (camUnlockMiss >= needMiss || (nowu - lastCamEdgeUs) > (camTimeout * 4UL)) {
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
  /* Coil charge: 0=constant duty (smart default), 1=constant time (dumb) */
  float dus;
  if (gCoilChargeMode == 0) {
    /* Constant duty: <300 RPM → 60%, >500 RPM → max 40%, else ramp */
    float duty;
    if (rpmLive < 300u) duty = 0.60f;
    else if (rpmLive < 500u) {
      float t = (float)(rpmLive - 300u) / 200.0f;
      duty = 0.60f - t * 0.20f; /* 60% → 40% */
    } else duty = 0.40f;
    /* sparks per crank rev: 4-cyl wasted/seq ≈ 2 */
    uint8_t spr = (gCyl >= 2) ? (uint8_t)(gCyl / 2u) : 1u;
    if (spr < 1) spr = 1;
    float periodUs = 0.0f;
    if (rpmLive >= 50u)
      periodUs = 60000000.0f / (float)rpmLive / (float)spr;
    dus = duty * periodUs;
  } else {
    /* Constant charge time, battery-compensated */
    dus = (float)CFG_DWELL_NOM_US * (14.0f / v);
  }
  if (dus < (float)CFG_DWELL_MIN_US) dus = (float)CFG_DWELL_MIN_US;
  if (dus > (float)CFG_DWELL_MAX_US) dus = (float)CFG_DWELL_MAX_US; /* hard 8 ms */
  /* Below 300 RPM still allow up to 8 ms but duty path already set 60% */
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
  serviceStartPrime();

  static uint32_t lastTel = 0;
  uint32_t now_ms = millis();
  if (mapDumpBusy && (now_ms - lastTel) > 3000U)
    mapDumpBusy = 0;
  if (!mapDumpBusy && (now_ms - lastTel) >= 100U) {
    lastTel = now_ms;
    sendTelemetry();
  }
}

void ECU_1kHzTick(void) {}
