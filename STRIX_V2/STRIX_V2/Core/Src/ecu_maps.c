/* ecu_maps.c — auto-split from ecu_app.c */
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
#include <string.h>
#include <stdio.h>
#include <math.h>
#include <stdint.h>

/* ---- lines 315-325 ---- */
void defaultMaps(void) {
  for (uint8_t r = 0; r < ROWS; r++)
    for (uint8_t c = 0; c < COLS; c++) {
      advMap[r][c] = clampAdv(10 + r + c);
      injMap[r][c] = clampInj(2.0f + r * 0.4f + c * 0.5f);
    }
}


/* INJ1 on PB15 (free pin; avoids PB4 NJTRST). Init all inj pins as GPIO. */

/* ---- lines 1091-1157 ---- */
void lookupMaps(float load, float rpm, int8_t *advOut, float *injOut) {
  uint8_t c0 = 0, c1 = 0;
  if (rpm <= rpmBinsLive[0]) {
    c0 = c1 = 0;
  } else if (rpm >= rpmBinsLive[COLS - 1]) {
    c0 = c1 = COLS - 1;
  } else {
    c0 = 0;
    c1 = 1;
    for (uint8_t i = 0; i < COLS - 1; i++) {
      if (rpm >= rpmBinsLive[i] && rpm <= rpmBinsLive[i + 1]) {
        c0 = i;
        c1 = i + 1;
        break;
      }
    }
  }

  uint8_t r0 = 0, r1 = 0;
  if (load <= mapBinsLive[0]) {
    r0 = r1 = 0;
  } else if (load >= mapBinsLive[ROWS - 1]) {
    r0 = r1 = ROWS - 1;
  } else {
    r0 = 0;
    r1 = 1;
    for (uint8_t i = 0; i < ROWS - 1; i++) {
      if (load >= mapBinsLive[i] && load <= mapBinsLive[i + 1]) {
        r0 = i;
        r1 = i + 1;
        break;
      }
    }
  }

  float cf = 0.0f;
  if (c1 != c0 && rpmBinsLive[c1] != rpmBinsLive[c0])
    cf = (rpm - rpmBinsLive[c0]) / (rpmBinsLive[c1] - rpmBinsLive[c0]);
  float rf = 0.0f;
  if (r1 != r0 && mapBinsLive[r1] != mapBinsLive[r0])
    rf = (load - mapBinsLive[r0]) / (mapBinsLive[r1] - mapBinsLive[r0]);
  if (cf < 0.0f) cf = 0.0f;
  if (cf > 1.0f) cf = 1.0f;
  if (rf < 0.0f) rf = 0.0f;
  if (rf > 1.0f) rf = 1.0f;

  float adv = (1.0f - cf) * (1.0f - rf) * (float)advMap[r0][c0]
            + cf * (1.0f - rf) * (float)advMap[r0][c1]
            + (1.0f - cf) * rf * (float)advMap[r1][c0]
            + cf * rf * (float)advMap[r1][c1];
  /* injMap cells are tenths of ms — scale after blend */
  float inj = (1.0f - cf) * (1.0f - rf) * (float)injMap[r0][c0]
            + cf * (1.0f - rf) * (float)injMap[r0][c1]
            + (1.0f - cf) * rf * (float)injMap[r1][c0]
            + cf * rf * (float)injMap[r1][c1];
  inj *= 0.1f;

  *advOut = clampAdv((int)(adv + (adv >= 0.0f ? 0.5f : -0.5f)));
  *injOut = inj;
  mapCellR = r0; /* low corner of cell (for MCELL crosshair) */
  mapCellC = c0;
  baseAdvDeg = *advOut;
  baseInjMs  = inj;
}

/* ── Sequential coils ───────────────────────────────────────── */

/* ---- lines 3925-3981 ---- */

/** If live MAP bins look like legacy normalised 0.2–2.4, convert to kPa ×100. */
void ECU_SanitizeMapBins(void)
{
  float mx = 0.0f;
  for (uint8_t i = 0; i < ROWS; i++) {
    if (mapBinsLive[i] > mx) mx = mapBinsLive[i];
  }
  if (mx > 0.0f && mx < 5.0f) {
    for (uint8_t i = 0; i < ROWS; i++)
      mapBinsLive[i] *= 100.0f;
  }
}

float calcEngineLoad(void)
{
  /*
   * Return value is in the same units as mapBinsLive:
   *   mode 0 MAP  → absolute kPa (engMap)
   *   mode 1 TPS  → throttle %
   *   mode 2 hybrid → kPa-weighted blend
   * So lookupMaps() and tuner MCELL/crosshair stay aligned.
   */
  float map_kpa = engMap;
  if (map_kpa < 0.0f) map_kpa = 0.0f;
  if (map_kpa > 500.0f) map_kpa = 500.0f;

  float tps_pct = engTps;
  if (tps_pct < 0.0f) tps_pct = 0.0f;
  if (tps_pct > 100.0f) tps_pct = 100.0f;

  uint8_t mode = gLoadMode;
  if (mode == 1) gUseTps = 1;
  else if (mode == 0) gUseTps = 0;

  float load;
  if (mode == 1) {
    /* Alpha-N — axis is TPS % */
    load = sensTpsEn ? tps_pct : map_kpa;
  } else if (mode == 2) {
    /* Hybrid: blend MAP kPa with TPS scaled onto MAP axis via LREF */
    float ref = gMapLoadRefKpa;
    if (ref < 50.0f) ref = 50.0f;
    if (ref > 250.0f) ref = 250.0f;
    float tps_as_kpa = (tps_pct * 0.01f) * ref;
    float rpm_f = (float)rpmLive;
    float w_rpm = (rpm_f - 1200.0f) / 2800.0f;
    if (w_rpm < 0.0f) w_rpm = 0.0f;
    if (w_rpm > 1.0f) w_rpm = 1.0f;
    float w_thr = tps_pct * 0.01f;
    float w = 0.5f * w_rpm + 0.5f * w_thr;
    if (!sensMapEn) w = 0.0f;
    if (!sensTpsEn) w = 1.0f;
    load = (1.0f - w) * tps_as_kpa + w * map_kpa;
  } else {
    /* Speed-density — absolute MAP kPa */
    load = sensMapEn ? map_kpa : tps_pct;
  }

  if (load < 0.0f) load = 0.0f;
  if (load > 500.0f) load = 500.0f;
  return load;
}


