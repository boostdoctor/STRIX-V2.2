/* ecu_util.c — auto-split from ecu_app.c */
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

/* ---- lines 250-397 ---- */
int clampi(int a, int lo, int hi) {
  if (a < lo) return lo;
  if (a > hi) return hi;
  return a;
}

/* millis/micros are static inline in ecu_runtime.h */

int8_t clampAdv(int v) { return (int8_t)clampi(v, -10, 45); }
uint8_t clampInj(float ms) {
  if (ms < 0) ms = 0;
  if (ms > 20) ms = 20;
  return (uint8_t)(ms * 10.0f + 0.5f);
}

/** Parse int from string; returns chars consumed or 0 on failure */
int parse_int(const char *s, int *out)
{
  const char *p = s;
  int sign = 1, v = 0, digits = 0;
  while (*p == ' ') p++;
  if (*p == '-') { sign = -1; p++; }
  else if (*p == '+') p++;
  while (*p >= '0' && *p <= '9') {
    v = v * 10 + (*p - '0');
    p++;
    digits++;
  }
  if (!digits) return 0;
  *out = sign * v;
  return (int)(p - s);
}

/** Parse float without scanf %f (newlib-nano often lacks it). */
int parse_float(const char *s, float *out)
{
  const char *p = s;
  int sign = 1, digits = 0;
  float v = 0.0f, frac = 0.1f;
  while (*p == ' ') p++;
  if (*p == '-') { sign = -1; p++; }
  else if (*p == '+') p++;
  while (*p >= '0' && *p <= '9') {
    v = v * 10.0f + (float)(*p - '0');
    p++;
    digits++;
  }
  if (*p == '.') {
    p++;
    while (*p >= '0' && *p <= '9') {
      v += frac * (float)(*p - '0');
      frac *= 0.1f;
      p++;
      digits++;
    }
  }
  if (!digits) return 0;
  *out = (float)sign * v;
  return (int)(p - s);
}


/* defaultMaps → ecu_maps.c
 * ecuInjGpioInit, allOutputsOff, cylAtSlot, tdcDeg, injSequentialActive → ecu_app.c
 * Do not redefine here.
 */

/* ── Cam (720° phase) ───────────────────────────────────────── */
