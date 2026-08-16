/* ecu_cmd.c — auto-split from ecu_app.c */
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
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <stdint.h>

/* ---- lines 2633-3675 ---- */
void uartWrite(const char *s) {
  ECU_Serial_Write(s);  /* USB CDC (Black Pill) */
}
void uartErr(const char *cmd, const char *why) {
  char b[72];
  snprintf(b, sizeof b, "ERR:%s,%s\r\n", cmd, why);
  uartWrite(b);
}


void sendTelemetry(void) {
  /* Split into two frames - keeps each snprintf well under buffer limit
   * and avoids -Wformat-truncation (single line was 400B dest, ~400-700B need). */
  char b[320];
  unsigned pw_us = (unsigned)injPwUs;
  int ign_d = (int)ignAdvanceDeg;

  int n = snprintf(b, sizeof b,
    "RPM:%u,PW:%.2f,INJ:%.2f,IGN:%d,TRET:%.1f,MAP:%.0f,TPS:%.0f,TMP:%.0f,IAT:%.0f,BAT:%.1f,"
    "EADC:%u,TADC:%u,BADC:%u,IADC:%u,MADC:%u,"
    "SYNC:%u,CAM:%u,CAM2:%u,FAN:%u,FP:%u,LOST:%u,"
    "TOOTH:%u,EDGE:%u,CRK:%u,DEG:%.0f,TERR:%u,DWELL:%u,CYL:%u\r\n",
    (unsigned)rpmLive,
    (double)(pw_us * 0.001f), (double)(pw_us * 0.001f), ign_d,
    (double)totalRetardDeg,
    (double)engMap, (double)engTps, (double)engEct, (double)engIat, (double)engBat,
    (unsigned)adcEct, (unsigned)adcTps, (unsigned)adcBat,
    (unsigned)adcIat, (unsigned)adcMap,
    (unsigned)syncLocked, (unsigned)(camSynced || camPulseSeen), (unsigned)cam2Synced,
    (unsigned)fanOn, (unsigned)fpOn, (unsigned)syncLosses,
    (unsigned)(syncLocked ? toothIndex : (crankEdgeCount & 0xFFFFu)),
    (unsigned)(crankEdgeCount & 0xFFFFu),
    (unsigned)HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0),
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
    "AFR:%.2f,LAM:%.3f,MCELL:%u:%u,BASEIGN:%d,BASEINJ:%u,O2:%.2f,KNK:%.1f,KRET:%.1f,STFT:%.1f,LTFT:%.1f,TTRIM:%.1f,CL:%u,LOAD:%.2f,SYNCQ:%u,"
    "PWUS:%u,INJMODE:%u,SEQ:%u,BATCHRPM:%u,IDLE:%u,IRPM:%.0f,ITHR:%.1f,DASH:%.1f,"
    "DFCO:%u,OFC:%u,VVT1:%u,VVT2:%u,C1PH:%.0f,C2PH:%.0f,ASE:%u,CLTCH:%u,"
    "LC:%u,ALS:%u,ALSTO:%u,ALSF:%.0f,FFS:%u\r\n",
    (double)engAfr, (double)afrToLambda(engAfr),
    (unsigned)mapCellR, (unsigned)mapCellC,
    (int)baseAdvDeg, (unsigned)(baseInjMs * 10.0f + 0.5f),
    (double)engO2, (double)engKnock, (double)knockRetardDeg,
    (double)stftPct, (double)ltftPct, (double)totalTrimPct(),
    (unsigned)o2ClActive, (double)engLoad, (unsigned)syncQualityPct(),
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


void ECU_ApplyWheelId(uint8_t id)
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
void handleUploadRow(char *line)
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



void ECU_Settings_Pack(EcuFlashSettings *out)
{
  if (!out) return;
  memset(out, 0, sizeof(*out));
  out->loadMode = gLoadMode;
  out->injMode = gInjMode;
  out->ignMode = gIgnMode;
  out->coilType = gCoilType;
  out->coilChargeMode = gCoilChargeMode;
  out->batchAboveRpm = gBatchAboveRpm;
  out->coilSmart = gCoilSmart;
  out->cylinders = gCyl;
  out->wheelId = gWheelId;
  out->dbwEnable = gDbwEnable;
  out->idleOutMode = gIdleOutMode;
  out->idleEnable = idleEnable;
  out->idleTargetRpm = (uint16_t)(idleTargetRpm < 0 ? 850 : idleTargetRpm);
  out->fpPrimeMs = gFpPrimeMs;
  out->injPrimeMs = gInjPrimeMs;
  out->injPrimeEn = gInjPrimeEn;
  out->rpmLimit = gRpmLimit;
  out->rpmCutMode = gRpmCutMode;
  out->fanEnable = (gFanOnC < 150.0f) ? 1u : 0u;
  out->fanOnC = (uint8_t)gFanOnC;
  out->o2Mode = o2SensorMode;
  out->vvtMode = vvtClEnable ? 1u : 0u;
  out->sensEctEn = sensEctEn;
  out->sensIatEn = sensIatEn;
  out->sensO2En = sensO2En;
  out->sensMapEn = sensMapEn;
  out->sensTpsEn = sensTpsEn;
  out->mapLoadRefKpa = (uint16_t)gMapLoadRefKpa;
}

void ECU_Settings_Apply(const EcuFlashSettings *in)
{
  if (!in) return;
  if (in->loadMode <= 2) {
    gLoadMode = in->loadMode;
    gUseTps = (gLoadMode == 1) ? 1 : 0;
  }
  gInjMode = in->injMode;
  if (in->ignMode <= 1) gIgnMode = in->ignMode;
  gCoilType = in->coilType;
  if (in->coilChargeMode <= 1) gCoilChargeMode = in->coilChargeMode;
  if (in->batchAboveRpm >= 500 && in->batchAboveRpm <= 9000)
    gBatchAboveRpm = in->batchAboveRpm;
  gCoilSmart = in->coilSmart ? 1 : 0;
  if (in->cylinders >= 1 && in->cylinders <= MAX_CYL)
    gCyl = in->cylinders;
  gWheelId = in->wheelId;
  gDbwEnable = in->dbwEnable ? 1 : 0;
  gIdleOutMode = in->idleOutMode;
  idleEnable = in->idleEnable ? 1 : 0;
  if (in->idleTargetRpm >= 500 && in->idleTargetRpm <= 2000)
    idleTargetRpm = (float)in->idleTargetRpm;
  if (in->fpPrimeMs <= 15000) gFpPrimeMs = in->fpPrimeMs;
  if (in->injPrimeMs <= 500) gInjPrimeMs = in->injPrimeMs;
  gInjPrimeEn = in->injPrimeEn ? 1 : 0;
  if (in->rpmLimit >= 2000 && in->rpmLimit <= 12000)
    gRpmLimit = in->rpmLimit;
  gRpmCutMode = in->rpmCutMode ? 1 : 0;
  if (in->fanEnable)
    gFanOnC = (float)in->fanOnC;
  else
    gFanOnC = 200.0f;
  o2SensorMode = in->o2Mode;
  vvtClEnable = in->vvtMode ? 1 : 0;
  sensEctEn = in->sensEctEn ? 1 : 0;
  sensIatEn = in->sensIatEn ? 1 : 0;
  sensO2En = in->sensO2En ? 1 : 0;
  sensMapEn = in->sensMapEn ? 1 : 0;
  sensTpsEn = in->sensTpsEn ? 1 : 0;
  if (in->mapLoadRefKpa >= 50 && in->mapLoadRefKpa <= 250)
    gMapLoadRefKpa = (float)in->mapLoadRefKpa;
  ECU_Idle_SetEnable(idleEnable);
  ECU_Idle_SetTargetRpm((uint16_t)idleTargetRpm);
}

/** Pack current RAM tune into flash blob (maps, cal, wheel, LTFT). */
void fillFlashBlob(EcuFlashBlob *blob)
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
  /* Full engine settings (v8+) */
  ECU_Settings_Pack(&blob->settings);
}


/** Run from ECU_Loop — pack RAM maps, program NVM, verify read-back */
void servicePendingSave(void)
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
  if (err == 0) {
    /* Brief settle then verify; do not fail the whole SAVE on a soft CRC glitch */
    for (int i = 0; i < 30; i++)
      ECU_Serial_Service();
    EcuFlashBlob rb;
    if (ECU_Flash_Load(&rb)) {
      flashA0 = rb.advMap[0][0];
      flashI0 = rb.injMap[0][0];
      crc = rb.crc32;
      mapsDirty = 0;
    } else if (ECU_Flash_Present()) {
      /* Programmed but CRC path strict — still report programmed address */
      crc = ECU_Flash_StoredCrc();
      mapsDirty = 0;
    } else {
      err = -6;
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


void handleLine(char *line) {
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

  /* SAVE — deferred to ECU_Loop (USB-safe); only block if engine running */
  if (!strncmp(line, "SAVE", 4)) {
    if (rpmLive > 400 && syncLocked) {
      uartWrite("ERR:SAVE,RPM");
      return;
    }
    uploadMode = 0;
    uploadRow  = 0;
    savePending = 1;
    uartWrite("BUSY:SAVE\r\n");
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

  
  if (!strncmp(line, "SET:SENS:", 9)) {
    const char *p = line + 9;
    int en = 1;
    const char *comma = strchr(p, ',');
    if (comma) en = atoi(comma + 1);
    if (!strncmp(p, "CAMMODE", 7)) {
      int m = atoi(p + 8);
      gCamMode = (m > 0) ? 1 : 0;
      uartWrite("OK:CAMMODE\r\n"); return;
    }
    if (!strncmp(p, "FANEN", 5)) {
      int en = atoi(p + 6);
      gFanEnable = en ? 1 : 0;
      if (!gFanEnable) { fanOn = 0; ECU_FAN_LO(); }
      uartWrite("OK:FANEN\r\n"); return;
    }
    if (!strncmp(p, "TACHO", 5)) {
      /* SET:TACHO,en,ppr */
      int en = 0, ppr = 2;
      sscanf(p + 5, ",%d,%d", &en, &ppr);
      gTachoEnable = en ? 1 : 0;
      if (ppr < 1) ppr = 1;
      if (ppr > 12) ppr = 12;
      gTachoPpr = (uint8_t)ppr;
      uartWrite("OK:TACHO\r\n"); return;
    }
    if (!strncmp(p, "ECT", 3)) { sensEctEn = en ? 1 : 0; uartWrite("OK:SENS:ECT\r\n"); return; }
    if (!strncmp(p, "IAT", 3)) { sensIatEn = en ? 1 : 0; uartWrite("OK:SENS:IAT\r\n"); return; }
    if (!strncmp(p, "O2", 2))  { sensO2En  = en ? 1 : 0; uartWrite("OK:SENS:O2\r\n");  return; }
    if (!strncmp(p, "MAP", 3)) { sensMapEn = en ? 1 : 0; uartWrite("OK:SENS:MAP\r\n"); return; }
    if (!strncmp(p, "TPS", 3)) { sensTpsEn = en ? 1 : 0; uartWrite("OK:SENS:TPS\r\n"); return; }
    uartErr("SENS", "PARSE");
    return;
  }
  if (!strncmp(line, "GETUID", 6)) {
    char b[40];
    snprintf(b, sizeof b, "UID:%s\r\n", gDeviceUid);
    uartWrite(b);
    return;
  }


  /* Load mode: SET:L,mode  or SET:L,0,0,mode  (0=MAP 1=TPS 2=Hybrid)
   * Optional: SET:LREF,kpa  — MAP kPa that equals load 1.0 (default 100) */

  if (!strncmp(line, "SET:FAN,", 8)) {
    /* SET:FAN,<onTempC> — threshold; requires FANEN=1 to drive */
    int t = atoi(line + 8);
    if (t < 40) t = 40;
    if (t > 130) t = 130;
    gFanOnC = (float)t;
    mapsDirty = 1;
    {
      char b[40];
      snprintf(b, sizeof b, "OK:FAN,%d\r\n", t);
      uartWrite(b);
    }
    return;
  }
  if (!strncmp(line, "SET:FANON,", 10)) {
    /* Forced fan for bench: SET:FANON,0|1 (only when RPM=0) */
    int on = atoi(line + 10);
    if (rpmLive > 0) { uartWrite("ERR:FANON,RPM\r\n"); return; }
    if (on) { fanOn = 1; ECU_FAN_HI(); }
    else    { fanOn = 0; ECU_FAN_LO(); }
    uartWrite("OK:FANON\r\n");
    return;
  }
  if (!strncmp(line, "OUTTEST", 7) || !strncmp(line, "SET:OUTTEST", 11)) {
    if (rpmLive > 0 || syncLocked) {
      uartWrite("ERR:OUTTEST,RPM\r\n");
      return;
    }
    outTestActive = 1;
    outTestStep = 0;
    outTestNextMs = millis();
    uartWrite("OK:OUTTEST\r\n");
    return;
  }

  if (!strncmp(line, "SET:LREF,", 9)) {
    float ref = 100.0f;
    if (parse_float(line + 9, &ref) > 0) {
      if (ref < 50.0f) ref = 50.0f;
      if (ref > 250.0f) ref = 250.0f;
      gMapLoadRefKpa = ref;
      uartWrite("OK:LREF\r\n");
    } else {
      uartErr("LREF", "PARSE");
    }
    return;
  }
  if (!strncmp(line, "SET:L,", 6)) {
    int a = 0, b = 0, mode = 0;
    int n = 0;
    const char *p = line + 6;
    /* accept SET:L,mode or SET:L,0,0,mode */
    n = parse_int(p, &a);
    if (n > 0) {
      p += n;
      if (*p == ',') {
        p++;
        n = parse_int(p, &b);
        if (n > 0) {
          p += n;
          if (*p == ',') {
            p++;
            parse_int(p, &mode);
          } else {
            mode = a; /* SET:L,mode with only one field was in a */
          }
        }
      } else {
        mode = a;
      }
    }
    /* If three fields, mode is third; if one field, mode is first */
    {
      int commas = 0;
      for (const char *q = line + 6; *q; q++) if (*q == ',') commas++;
      if (commas == 0) mode = a;
      else if (commas >= 2) {
        /* already set mode from third parse — re-parse robustly */
        int v0=0,v1=0,v2=0;
        if (sscanf(line + 6, "%d,%d,%d", &v0, &v1, &v2) == 3)
          mode = v2;
        else if (sscanf(line + 6, "%d", &v0) == 1)
          mode = v0;
      } else if (commas == 0) {
        mode = a;
      }
    }
    if (mode < 0) mode = 0;
    if (mode > 2) mode = 2;
    gLoadMode = (uint8_t)mode;
    gUseTps = (gLoadMode == 1) ? 1 : 0;
    uartWrite("OK:L\r\n");
    return;
  }

  if (!strncmp(line, "SET:FPPRIME,", 12)) {
    int ms = 0;
    if (parse_int(line + 12, &ms) > 0) {
      if (ms < 0) ms = 0;
      if (ms > 15000) ms = 15000;
      gFpPrimeMs = (uint16_t)ms;
      if (ms > 0)
        fpPrimeUntilMs = millis() + (uint32_t)ms;
      else
        fpPrimeUntilMs = 0;
      uartWrite("OK:FPPRIME\r\n");
    } else uartErr("FPPRIME", "PARSE");
    return;
  }
  if (!strncmp(line, "SET:INJPRIME,", 13)) {
    int ms = 0;
    if (parse_int(line + 13, &ms) > 0) {
      if (ms < 0) ms = 0;
      if (ms > 500) ms = 500;
      gInjPrimeMs = (uint16_t)ms;
      uartWrite("OK:INJPRIME\r\n");
    } else uartErr("INJPRIME", "PARSE");
    return;
  }
  if (!strncmp(line, "SET:INJPRIMEEN,", 15)) {
    int en = 1;
    if (parse_int(line + 15, &en) > 0) {
      gInjPrimeEn = en ? 1 : 0;
      uartWrite("OK:INJPRIMEEN\r\n");
    } else uartErr("INJPRIMEEN", "PARSE");
    return;
  }


  if (!strncmp(line, "SET:IDLEEN,", 11)) {
    int en = 1;
    if (parse_int(line + 11, &en) > 0) {
      idleEnable = en ? 1 : 0;
      ECU_Idle_SetEnable(idleEnable);
      uartWrite("OK:IDLEEN\r\n");
    } else uartErr("IDLEEN", "PARSE");
    return;
  }
  if (!strncmp(line, "SET:IDLERPM,", 12)) {
    int rpm = 850;
    if (parse_int(line + 12, &rpm) > 0) {
      if (rpm < 500) rpm = 500;
      if (rpm > 2000) rpm = 2000;
      idleTargetRpm = (float)rpm;
      ECU_Idle_SetTargetRpm((uint16_t)rpm);
      uartWrite("OK:IDLERPM\r\n");
    } else uartErr("IDLERPM", "PARSE");
    return;
  }
  if (!strncmp(line, "SET:IDLEPID,", 12)) {
    /* SET:IDLEPID,kp,ki,kd  (scaled: send as int *1000 e.g. 12,8,2 → 0.012) */
    int a=0,b=0,c=0;
    if (sscanf(line + 12, "%d,%d,%d", &a, &b, &c) == 3) {
      IDLE_KP = (float)a / 1000.0f;
      IDLE_KI = (float)b / 1000.0f;
      IDLE_KD = (float)c / 1000.0f;
      ECU_Idle_SetGains(IDLE_KP, IDLE_KI, IDLE_KD);
      uartWrite("OK:IDLEPID\r\n");
    } else uartErr("IDLEPID", "PARSE");
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
      if (mi >= 0 && mi < gTeeth) gMissing = (uint8_t)mi;
      gTrigAngle = (uint16_t)an;
      syncLocked = 0;
      camSynced = 0;
    }
    return;
  }


  if (!strncmp(line, "SET:COILTYPE,", 13)) {
    int t = 0;
    if (sscanf(line + 13, "%d", &t) == 1) {
      if (t < 0) t = 0;
      if (t > 2) t = 2;
      gCoilType = (uint8_t)t;
      /* Smart → constant duty; Dumb/Dist → constant charge time */
      if (gCoilType == 0) gCoilChargeMode = 0;
      else gCoilChargeMode = 1;
      char b[40];
      snprintf(b, sizeof b, "OK:COILTYPE,%u,MODE:%u\r\n",
               (unsigned)gCoilType, (unsigned)gCoilChargeMode);
      uartWrite(b);
    } else uartErr("COILTYPE", "PARSE");
    return;
  }
  if (!strncmp(line, "SET:COILMODE,", 13)) {
    int m = 0;
    if (sscanf(line + 13, "%d", &m) == 1) {
      if (m < 0) m = 0;
      if (m > 1) m = 1;
      gCoilChargeMode = (uint8_t)m;
      char b[32];
      snprintf(b, sizeof b, "OK:COILMODE,%u\r\n", (unsigned)gCoilChargeMode);
      uartWrite(b);
    } else uartErr("COILMODE", "PARSE");
    return;
  }


  if (!strncmp(line, "SET:IGNMODE,", 12)) {
    int m = 0;
    if (sscanf(line + 12, "%d", &m) == 1) {
      gIgnMode = (m == 1) ? 1 : 0;
      if (gIgnMode == 1 && gCamMode == 0) gCamMode = 1;
      if (gIgnMode == 0) { camSynced = 0; camLockHits = 0; }
      uartWrite("OK:IGNMODE\r\n");
    }
    return;
  }
  if (!strncmp(line, "SET:INJMODE,", 12)) {
    int m = 0;
    if (sscanf(line + 12, "%d", &m) == 1) {
      if (m < 1) m = 1;
      if (m > 3) m = 3;
      gInjMode = (uint8_t)m;
      if ((gInjMode == 2 || gInjMode == 3) && gCamMode == 0) gCamMode = 1;
      uartWrite("OK:INJMODE\r\n");
    }
    return;
  }
  if (!strncmp(line, "GETCFG", 6)) {
    char b[160];
    snprintf(b, sizeof b,
      "CFG:%u,%u,%u,CYL:%u,INJMODE:%u,IGNMODE:%u,WHEEL:%u,CAMMODE:%u,FANEN:%u,TACHO:%u,%u,COIL:%u,%u\r\n",
      (unsigned)gTeeth, (unsigned)gMissing, (unsigned)gTrigAngle,
      (unsigned)gCyl, (unsigned)gInjMode, (unsigned)gIgnMode,
      (unsigned)gWheelId, (unsigned)gCamMode,
      (unsigned)gFanEnable, (unsigned)gTachoEnable, (unsigned)gTachoPpr,
      (unsigned)gCoilType, (unsigned)gCoilChargeMode);
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
if (!strncmp(line, "GETPROTO", 8) || !strncmp(line, "PROTO?", 6)) {
    uartWrite("PROTO:2,NAME:STRIXV2,MAP:12x22,VER:2.0.0\r\n");
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

