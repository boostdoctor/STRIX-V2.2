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
    /* Full frame each call — no blocking */
    adcMap   = ECU_Adc_Raw(ECU_ADC_IX_MAP);
    adcTps   = ECU_Adc_Raw(ECU_ADC_IX_TPS);
    adcEct   = ECU_Adc_Raw(ECU_ADC_IX_CLT);
    adcIat   = ECU_Adc_Raw(ECU_ADC_IX_IAT);
    adcO2    = ECU_Adc_Raw(ECU_ADC_IX_O2);
    adcFlex  = ECU_Adc_Raw(ECU_ADC_IX_FLEX);
    adcBat   = ECU_Adc_Raw(ECU_ADC_IX_VBATT);
    adcPedal = ECU_Adc_Raw(ECU_ADC_IX_PEDAL);

    if (mapCalReady) {
      float a = (float)adcMap; int i = 0;
      while (i < MAP_CAL_N - 2 && a > mapCalAdc[i + 1]) i++;
      float a0 = mapCalAdc[i], a1 = mapCalAdc[i + 1];
      float f = (a1 > a0) ? (a - a0) / (a1 - a0) : 0;
      if (f < 0) f = 0;
      if (f > 1) f = 1;
      engMap = mapCalKpa[i] * (1 - f) + mapCalKpa[i + 1] * f;
    } else {
      engMap = CFG_MAP_OFFSET_KPA + adcMap * scale * CFG_MAP_GAIN_KPA_V;
    }
    engTps   = adcToPctCal(adcTps, tpsClosedAdc, tpsOpenAdc);
    engPedal = adcToPctCal(adcPedal, pedClosedAdc, pedOpenAdc);
    engEct   = ntcBetaC(adcEct);
    engIat   = ntcBetaC(adcIat);
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
      adcFlex  = readAdc(ECU_ADC_CH_FLEX);
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
