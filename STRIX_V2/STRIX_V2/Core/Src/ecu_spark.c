/* ecu_spark.c — wasted-spark / sequential coil scheduling */
#include "main.h"
#include "ecu_config.h"
#include "ecu_pins.h"
#include "ecu_runtime.h"
#include "ecu_internal.h"
#include <stdint.h>

#ifndef CFG_DWELL_MAX_US
#define CFG_DWELL_MAX_US  8000U
#endif

void scheduleCoils(uint32_t now)
{
  /* Safety: force coil off if hung (longer limit while cranking) */
  uint32_t hang = (rpmLive < 400) ? 12000U : ((uint32_t)CFG_DWELL_MAX_US + 500U);
  for (uint8_t i = 1; i <= MAX_CYL; i++) {
    if (coilState[i] && (now - coilStartUs[i]) > hang) {
      ECU_IGN_LO(i);
      coilState[i] = 0;
      coilFired[i] = 1;
    }
  }

  /* Allow spark whenever we have lock and a tooth period.
   * No minimum RPM — required for sub-300 RPM cranking. */
  if (!syncLocked || toothPeriodUs < 40UL || toothPeriodUs > 2000000UL)
    return;

  float usPerRev = (float)toothPeriodUs * (float)((gTeeth > 0) ? gTeeth : 36);
  if (usPerRev < 400.0f) return;
  if (usPerRev > 5000000.0f) return; /* < ~12 RPM: ignore */

  float degPerUs = 360.0f / usPerRev;
  float dwellUs = (float)dwellTargetUs;
  /* At cranking, enforce minimum dwell so coil charges */
  if (rpmLive < 400 && dwellUs < 3000.0f) dwellUs = 3000.0f;
  if (dwellUs > 8000.0f) dwellUs = 8000.0f;

  float dwellDeg = dwellUs * degPerUs;
  if (dwellDeg > 90.0f) dwellDeg = 90.0f;
  if (dwellDeg < 2.0f) dwellDeg = 2.0f;

  float band = 360.0f / (float)((gTeeth > 0) ? gTeeth : 36);
  /* Wide window while cranking — deg steps are large between loop passes */
  float win = (rpmLive < 400) ? (band * 5.0f) : (band * 3.0f);
  if (win < 20.0f) win = 20.0f;
  if (win > 60.0f) win = 60.0f;

  float adv = (float)ignAdvanceDeg;
  /* Cap advance while cranking so spark is not too early */
  if (rpmLive < 400 && adv > 12.0f) adv = 12.0f;
  float trig = (float)gTrigAngle;
  float deg = crankDeg;
  uint8_t seq = ignSequentialActive();
  float cycle = seq ? 720.0f : 360.0f;

  uint8_t n = gCyl;
  if (n > MAX_CYL) n = MAX_CYL;
  if (!seq) {
    n = (gCyl >= 4) ? 4 : gCyl;
  }

  for (uint8_t i = 1; i <= n; i++) {
    if (rpmCutActive && gRpmCutMode == 1 && (i & 1u)) {
      ECU_IGN_LO(i); coilState[i] = 0; continue;
    }

    float tdc;
    if (seq) {
      tdc = tdcDeg(i);
    } else {
      if (i == 1 || i == 4) tdc = 0.0f;
      else if (i == 2 || i == 3) tdc = 180.0f;
      else continue;
    }

    float fire = tdc + trig - adv;
    while (fire < 0.0f) fire += cycle;
    while (fire >= cycle) fire -= cycle;

    float dwellStart = fire - dwellDeg;
    while (dwellStart < 0.0f) dwellStart += cycle;
    while (dwellStart >= cycle) dwellStart -= cycle;

    uint8_t inDwell;
    if (dwellStart <= fire)
      inDwell = (deg >= dwellStart && deg < fire) ? 1u : 0u;
    else
      inDwell = (deg >= dwellStart || deg < fire) ? 1u : 0u;

    uint8_t inFire = 0;
    float fireEnd = fire + win;
    if (fireEnd < cycle) {
      inFire = (deg >= fire && deg < fireEnd) ? 1u : 0u;
    } else {
      inFire = (deg >= fire || deg < (fireEnd - cycle)) ? 1u : 0u;
    }

    if (!coilFired[i] && inFire) {
      ECU_IGN_LO(i);
      coilState[i] = 0;
      coilFired[i] = 1;
      continue;
    }

    if (!coilFired[i] && !coilState[i] && inDwell) {
      ECU_IGN_HI(i);
      coilState[i] = 1;
      coilStartUs[i] = now;
    }

    if (coilState[i] && !coilFired[i] && !inDwell && !inFire) {
      float past = deg - fire;
      if (past < 0.0f) past += cycle;
      if (past < win * 2.0f) {
        ECU_IGN_LO(i);
        coilState[i] = 0;
        coilFired[i] = 1;
      }
    }
  }

  /* Cranking time-assist: if no coil has fired this half-rev and we are
   * past mid-cycle, force a wasted pair so the engine can start even when
   * angle alignment is still coarse (pre-gap). */
  if (rpmLive < 400 && !seq) {
    static uint32_t lastAssistUs = 0;
    uint32_t halfUs = (uint32_t)(usPerRev * 0.5f);
    if (halfUs < 20000UL) halfUs = 20000UL;
    if ((now - lastAssistUs) >= halfUs) {
      uint8_t any = 0;
      for (uint8_t i = 1; i <= n; i++)
        if (coilFired[i]) any = 1;
      if (!any) {
        /* Fire pair based on cycleHalf */
        uint8_t a = cycleHalf ? 2 : 1;
        uint8_t b = cycleHalf ? 3 : 4;
        if (a <= n) {
          if (coilState[a]) { ECU_IGN_LO(a); coilState[a] = 0; }
          else { ECU_IGN_HI(a); coilState[a] = 1; coilStartUs[a] = now; }
          coilFired[a] = 1;
        }
        if (b <= n && b != a) {
          if (coilState[b]) { ECU_IGN_LO(b); coilState[b] = 0; }
          else { ECU_IGN_HI(b); coilState[b] = 1; coilStartUs[b] = now; }
          coilFired[b] = 1;
        }
        lastAssistUs = now;
      } else {
        lastAssistUs = now;
      }
    }
  }
}

float wrapAngle(float a, float cycle)
{
  while (a < 0.0f) a += cycle;
  while (a >= cycle) a -= cycle;
  return a;
}

uint8_t angleActive(float deg, float start, float end, float cycle)
{
  start = wrapAngle(start, cycle);
  end   = wrapAngle(end, cycle);
  deg   = wrapAngle(deg, cycle);
  if (start <= end)
    return (deg >= start && deg < end) ? 1u : 0u;
  return (deg >= start || deg < end) ? 1u : 0u;
}
