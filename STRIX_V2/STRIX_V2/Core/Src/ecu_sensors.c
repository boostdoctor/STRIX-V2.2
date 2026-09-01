/* ecu_sensors.c — auto-split from ecu_app.c */
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

/* Compile even if Cube tree has a stale ecu_runtime.h */
extern float gMapKpaMin;
extern float gMapKpaMax;

/* ---- lines 2545-2633 ---- */
float ntcBetaC(uint16_t adc) {
  if (adc < 1) adc = 1;
  if (adc > 4094) adc = 4094;
  float v = (float)adc / CFG_ADC_MAX;
  float r = CFG_NTC_PULLUP_OHM * v / (1.0f - v);
  float st = logf(r / CFG_NTC_R0_OHM) / CFG_NTC_BETA + 1.0f / 298.15f;
  return (1.0f / st) - 273.15f;
}


/** Map raw ADC to 0-100% using closed/open endpoints */
float adcToPctCal(uint16_t adc, uint16_t closed, uint16_t open)
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

void readSensors(void) {
  /*
   * With timer-triggered DMA (ECU_Adc_Init), all ranks are fresh every scan.
   * When DMA is down, readAdc() polls — keep light RR for CPU only.
   */
  float scale = CFG_BAT_ADC_REF_V / CFG_ADC_MAX;

  if (ecuAdcDmaRunning) {
    /* Full frame each call — no blocking (PA6 FLEX is frequency, not ADC) */
    adcMap   = ECU_Adc_Raw(ECU_ADC_IX_MAP);
    adcTps   = ECU_Adc_Raw(ECU_ADC_IX_TPS);
    adcEct   = ECU_Adc_Raw(ECU_ADC_IX_CLT);
    adcIat   = ECU_Adc_Raw(ECU_ADC_IX_IAT);
    adcO2    = ECU_Adc_Raw(ECU_ADC_IX_O2);
    adcBat   = ECU_Adc_Raw(ECU_ADC_IX_VBATT);
    adcPedal = 0;

    /* Instant raw engineering units
     * Default MAP: linear ADC 0 → gMapKpaMin, ADC 4095 → gMapKpaMax
     * (sensor range from tuner; multi-point mapCal overrides when ready) */
    float map_raw, tps_raw;
    if (mapCalReady) {
      float a = (float)adcMap; int i = 0;
      while (i < MAP_CAL_N - 2 && a > mapCalAdc[i + 1]) i++;
      float a0 = mapCalAdc[i], a1 = mapCalAdc[i + 1];
      float f = (a1 > a0) ? (a - a0) / (a1 - a0) : 0;
      if (f < 0) f = 0;
      if (f > 1) f = 1;
      map_raw = mapCalKpa[i] * (1 - f) + mapCalKpa[i + 1] * f;
    } else {
      float span = gMapKpaMax - gMapKpaMin;
      if (span < 1.0f) span = 1.0f;
      map_raw = gMapKpaMin + ((float)adcMap / CFG_ADC_MAX) * span;
    }
    tps_raw = adcToPctCal(adcTps, tpsClosedAdc, tpsOpenAdc);
    engPedal = adcToPctCal(adcPedal, pedClosedAdc, pedOpenAdc);

    /*
     * MAP / TPS: 8-sample ring + 20 ms publish (rusEFI-style sensor buffering).
     * ADC can run fast; fuel/crosshair see a slowed, averaged value.
     */
    {
      static float mapRing[8], tpsRing[8];
      static uint8_t rix = 0, rfill = 0;
      static uint32_t lastPub = 0;
      mapRing[rix] = map_raw;
      tpsRing[rix] = tps_raw;
      rix = (uint8_t)((rix + 1u) & 7u);
      if (rfill < 8) rfill++;
      uint32_t now = HAL_GetTick();
      if (lastPub == 0 || (now - lastPub) >= 20u) {
        float ms = 0.f, ts = 0.f;
        for (uint8_t i = 0; i < rfill; i++) {
          ms += mapRing[i];
          ts += tpsRing[i];
        }
        float n = (float)rfill;
        float mapAvg = ms / n;
        float tpsAvg = ts / n;
        /* Hysteresis: ignore sub-threshold chatter */
        if (engMap <= 0.0f || mapAvg > engMap + 1.5f || mapAvg + 1.5f < engMap)
          engMap = mapAvg;
        if (engTps <= 0.0f || tpsAvg > engTps + 1.5f || tpsAvg + 1.5f < engTps)
          engTps = tpsAvg;
        lastPub = now;
      }
    }

    /* CLT / IAT: thermal mass is slow — refresh at most every 5 s */
    {
      static uint32_t lastTempMs = 0;
      static uint8_t tempInit = 0;
      uint32_t now = HAL_GetTick();
      if (!tempInit || (now - lastTempMs) >= 5000u) {
        engEct = ntcBetaC(adcEct);
        engIat = ntcBetaC(adcIat);
        lastTempMs = now;
        tempInit = 1;
      }
    }
    if (batCalReady) {
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
    engO2    = adcO2 * scale;
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
    /* Flex fuel: 10 s sample, 0.5–4.5 V → 0–100 % ethanol */
    {
      static uint32_t lastFlexMs;
      uint32_t now = HAL_GetTick();
      if (gFlexEnable && (now - lastFlexMs) >= 10000u) {
        lastFlexMs = now;
        int span = (int)gFlexAdcE100 - (int)gFlexAdcE0;
        if (span > 50 || span < -50) {
          float e = 100.0f * ((float)((int)adcFlex - (int)gFlexAdcE0) / (float)span);
          if (e < 0.0f) e = 0.0f;
          if (e > 100.0f) e = 100.0f;
          engEthanol = e;
        }
      }
    }
    return;
  }

  /* Polling fallback: 2 channels per loop */
  switch (sensorPhase & 3u) {
    case 0:
      adcMap = readAdc(ECU_ADC_CH_MAP);
      adcTps = readAdc(ECU_ADC_CH_TPS);
      {
        float map_raw, tps_raw;
        if (mapCalReady) {
          float a = (float)adcMap; int i = 0;
          while (i < MAP_CAL_N - 2 && a > mapCalAdc[i + 1]) i++;
          float a0 = mapCalAdc[i], a1 = mapCalAdc[i + 1];
          float f = (a1 > a0) ? (a - a0) / (a1 - a0) : 0;
          if (f < 0) f = 0;
          if (f > 1) f = 1;
          map_raw = mapCalKpa[i] * (1 - f) + mapCalKpa[i + 1] * f;
        } else {
          float span = gMapKpaMax - gMapKpaMin;
          if (span < 1.0f) span = 1.0f;
          map_raw = gMapKpaMin + ((float)adcMap / CFG_ADC_MAX) * span;
        }
        tps_raw = adcToPctCal(adcTps, tpsClosedAdc, tpsOpenAdc);
        static float map_f = -1.0f, tps_f = -1.0f;
        if (map_f < 0.0f) { map_f = map_raw; tps_f = tps_raw; }
        map_f += 0.12f * (map_raw - map_f);
        tps_f += 0.15f * (tps_raw - tps_f);
        if (engMap <= 0.0f || map_f > engMap + 1.5f || map_f + 1.5f < engMap)
          engMap = map_f;
        if (engTps <= 0.0f || tps_f > engTps + 1.5f || tps_f + 1.5f < engTps)
          engTps = tps_f;
      }
      break;
    case 1:
      adcPedal = readAdc(ECU_ADC_CH_PEDAL);
      adcBat   = readAdc(ECU_ADC_CH_VBATT);
      engPedal = adcToPctCal(adcPedal, pedClosedAdc, pedOpenAdc);
      if (batCalReady) {
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
      {
        static uint32_t lastTempMsP = 0;
        static uint8_t tempInitP = 0;
        uint32_t now = HAL_GetTick();
        if (!tempInitP || (now - lastTempMsP) >= 5000u) {
          engEct = ntcBetaC(adcEct);
          engIat = ntcBetaC(adcIat);
          lastTempMsP = now;
          tempInitP = 1;
        }
      }
      break;
    default:
      adcO2    = readAdc(ECU_ADC_CH_O2);
      /* FLEX = frequency on PA6 */
      engO2    = adcO2 * scale;
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
