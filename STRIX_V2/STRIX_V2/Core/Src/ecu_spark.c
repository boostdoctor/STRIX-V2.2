/* ecu_spark.c — auto-split from ecu_app.c */
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

/* Angle from a to b, 0..cycle */
static float angDelta(float deg, float ref, float cycle)
{
  float d = deg - ref;
  while (d < 0.0f) d += cycle;
  while (d >= cycle) d -= cycle;
  return d;
}

static float wastedTdc(uint8_t cyl)
{
  if (gCyl <= 2)
    return (cyl == 1) ? 0.0f : 180.0f;
  if (gCyl == 3)
    return (float)((cyl - 1u) % 3u) * 120.0f;
  /* 4-cyl wasted: companion pairs 1+4 and 2+3 */
  if (cyl == 1 || cyl == 4) return 0.0f;
  if (cyl == 2 || cyl == 3) return 180.0f;
  return 0.0f;
}

void scheduleCoils(uint32_t now)
{
  static uint32_t lastCoilFireUs[MAX_CYL + 1];
  static uint8_t  coilPulseN[MAX_CYL + 1]; /* 0=idle 1=gap 2=second */
  static float    coilDblDeg[MAX_CYL + 1];

  /* 20 ms hang at cranking — 8 ms was cutting dwell before TDC at low RPM */
  uint32_t hangUs = (rpmLive < 1200) ? 20000u : 8000u;
  for (uint8_t i = 1; i <= MAX_CYL; i++) {
    if (coilState[i] && (now - coilStartUs[i]) > hangUs) {
      ECU_IGN_LO(i);
      coilState[i] = 0;
      coilFired[i] = 1;
      coilPulseN[i] = 0;
    }
    /* Double-spark latch must not outlive the event — that killed all coils. */
    if (!gSparkDouble)
      coilPulseN[i] = 0;
    else if (coilPulseN[i] && lastCoilFireUs[i] &&
             (now - lastCoilFireUs[i]) > 8000u)
      coilPulseN[i] = 0;
  }

  if (rpmLive >= gRpmLimit)
    rpmCutActive = 1;
  else if (rpmLive + (gRpmCutMode ? 150 : 200) < gRpmLimit)
    rpmCutActive = 0;

  {
    uint8_t spinning = (lastToothUs != 0 && (now - lastToothUs) < 250000UL);
    if (!spinning || (toothPeriodUs < 40 && toothPeriodFilt < 40)) {
      for (uint8_t i = 1; i <= gCyl && i <= MAX_CYL; i++) {
        ECU_IGN_LO(i);
        coilState[i] = 0;
        coilPulseN[i] = 0;
      }
      return;
    }
    /* No gap-lock yet: still spark wasted from tooth index so a stim
     * or a weak first-lock produces pulses. */
    if (!syncLocked) {
      uint8_t teeth = (gTeeth > 1) ? gTeeth : 36;
      crankDeg = (float)toothIndex * (360.0f / (float)teeth);
    }
  }
  if (rpmCutActive && gRpmCutMode == 0) {
    for (uint8_t i = 1; i <= gCyl && i <= MAX_CYL; i++) {
      ECU_IGN_LO(i);
      coilState[i] = 0;
      coilPulseN[i] = 0;
    }
    return;
  }

  uint32_t Trev = toothPeriodFilt ? toothPeriodFilt : toothPeriodUs;
  uint8_t teethRev = (gTeeth > 1) ? gTeeth : 36;
  float usPerRev = (float)Trev * (float)teethRev;
  if (usPerRev < 2000.0f)
    return;
  float degPerUs = 360.0f / usPerRev;
  float dwellDeg = (float)dwellTargetUs * degPerUs;
  if (dwellDeg > 80.0f) dwellDeg = 80.0f;
  if (dwellDeg < 2.0f) dwellDeg = 2.0f;

  /* 720° only with cam lock — without it, wasted 360° keeps TDC correct. */
  uint8_t seq = (ignSequentialActive() && camSynced) ? 1u : 0u;
  float cycle = seq ? 720.0f : 360.0f;
  float deg = crankDeg;
  float adv = (float)ignAdvanceDeg;
  float trig = (float)gTrigAngle;
  /* Tight window — 50° made every coil look "at fire" and dumped them together. */
  float band = 8.0f;
  if (rpmLive < 400)
    band = 16.0f;

  {
    uint32_t T = toothPeriodFilt ? toothPeriodFilt : toothPeriodUs;
    uint32_t age = (lastToothUs && now > lastToothUs) ? (now - lastToothUs) : 0;
    if (T >= 80u && age < T * 2u) {
      float extra = 360.0f * ((float)age / ((float)T * (float)((gTeeth > 1) ? gTeeth : 36)));
      if (extra > 0.0f && extra < 20.0f)
        deg = wrapAngle(deg + extra, cycle);
    }
  }

  uint8_t n = gCyl;
  if (n > MAX_CYL) n = MAX_CYL;
  if (!seq && n > 4) n = 4;

  uint32_t revUs = toothPeriodFilt ? toothPeriodFilt : toothPeriodUs;
  revUs *= (uint32_t)((gTeeth > 1) ? gTeeth : 36);
  if (revUs < 4000u) revUs = 4000u;

  for (uint8_t i = 1; i <= n; i++) {
    if (rpmCutActive && gRpmCutMode == 1 && (i & 1u)) {
      ECU_IGN_LO(i);
      coilState[i] = 0;
      continue;
    }

    float tdc = seq ? tdcDeg(i) : wastedTdc(i);
    /* crankDeg is gap-relative. Cyl1 compression TDC = gTrigAngle. */
    float fire = wrapAngle(tdc + trig - adv, cycle);

    /* Sequential: one spark / 720° per coil (>= 1.6 crank revs).
     * Wasted: one spark / 360° (>= 0.45 crank rev). */
    uint32_t minGap = seq ? ((revUs * 8u) / 5u) : (revUs / 2u);
    uint8_t armed = ((now - lastCoilFireUs[i]) >= minGap);
    if (armed && !coilState[i])
      coilFired[i] = 0;

    float pastFire = angDelta(deg, fire, cycle);
    float until = cycle - pastFire; /* degrees to next fire */
    uint8_t atFire = (pastFire < band);
    /* Tooth hit: current slot is the fire tooth (works when loop rate is low) */
    {
      uint8_t teeth = (gTeeth > 1) ? gTeeth : 36;
      uint16_t nidx = seq ? (uint16_t)(teeth * 2u) : teeth;
      uint16_t cur  = seq
        ? (uint16_t)(toothIndex + (uint16_t)cycleHalf * (uint16_t)teeth)
        : (uint16_t)toothIndex;
      uint16_t ft = (uint16_t)(fire * (float)nidx / cycle + 0.5f);
      if (nidx) ft %= nidx;
      uint16_t ft1 = (uint16_t)((ft + 1u) % (nidx ? nidx : 1u));
      if (cur == ft || cur == ft1)
        atFire = 1;
    }

#if !CFG_COIL_SMART
    float dwellStart __attribute__((unused)) = wrapAngle(fire - dwellDeg, cycle);
    uint8_t inDwell = angleActive(deg, dwellStart, fire, cycle);
    if (!inDwell && until <= dwellDeg && until > 0.5f)
      inDwell = 1;
#endif

#if CFG_COIL_SMART
    /* Charge before TDC; spark at fire. Never start after TDC. */
    /* dwellDeg at cranking is often < band, so "until > band" never started
     * the coil; atFire then charged and dumped in the same pass (no pulse). */
    if (!coilFired[i] && armed && !coilState[i] && coilPulseN[i] == 0 &&
        (until <= dwellDeg || atFire || until < 40.0f)) {
      ECU_IGN_HI(i);
      coilState[i] = 1;
      coilStartUs[i] = now;
    }
    {
      uint32_t dneed = dwellTargetUs ? dwellTargetUs : (uint32_t)CFG_DWELL_NOM_US;
      if (coilPulseN[i] == 2 && dneed > 1500u)
        dneed = dneed / 2u;
      if (coilState[i] &&
          ((now - coilStartUs[i]) >= dneed ||
           (atFire && (now - coilStartUs[i]) >= 800u))) {
        ECU_IGN_LO(i);
        dwellActualUs = (uint16_t)(now - coilStartUs[i]);
        coilState[i] = 0;
        lastCoilFireUs[i] = now;
        if (gSparkDouble && coilPulseN[i] == 0) {
          coilPulseN[i] = 1;
          coilDblDeg[i] = deg;
          coilFired[i] = 0;
        } else {
          coilPulseN[i] = 0;
          coilFired[i] = 1;
        }
      }
    }
#else
    if (!coilFired[i] && armed && !coilState[i] && inDwell) {
      ECU_IGN_HI(i);
      coilState[i] = 1;
      coilStartUs[i] = now;
    }
    if (coilState[i] && !coilFired[i] && (atFire || (now - coilStartUs[i]) >= dwellTargetUs)) {
      ECU_IGN_LO(i);
      dwellActualUs = (uint16_t)(now - coilStartUs[i]);
      coilState[i] = 0;
      coilFired[i] = 1;
      lastCoilFireUs[i] = now;
    }
#endif
    if (coilState[i] && (now - coilStartUs[i]) > (uint32_t)(dwellTargetUs ? dwellTargetUs : CFG_DWELL_MAX_US) + 1500U) {
      ECU_IGN_LO(i);
      coilState[i] = 0;
      coilFired[i] = 1;
      coilPulseN[i] = 0;
      lastCoilFireUs[i] = now;
    }
    /* Second pulse after N crank degrees from first spark-off. */
    if (gSparkDouble && coilPulseN[i] == 1 && !coilState[i]) {
      float need = (float)gSparkDblGapDeg;
      if (need < 1.0f) need = 1.0f;
      if (need > 40.0f) need = 40.0f;
      float moved = angDelta(deg, coilDblDeg[i], cycle);
      uint32_t age = now - lastCoilFireUs[i];
      if (rpmLive > 4000 || moved > (need + 25.0f) || age > 12000u) {
        coilPulseN[i] = 0;
        coilFired[i] = 1;
      } else if (moved >= need) {
        ECU_IGN_HI(i);
        coilState[i] = 1;
        coilStartUs[i] = now;
        coilPulseN[i] = 2;
        coilFired[i] = 0;
      }
    }
  }

}


/* ── EOI-based sequential injection ───────────────────────────
 * Injection ends at (compression TDC - gEoiBtdc) on the engine cycle.
 * SOI = EOI - pulse_width_in_degrees (from current PW and RPM).
 * 720° when sequential + cam; 360° wasted/semi-seq otherwise.
 * Time-based close is a safety backup if angle is missed.
 */
#ifndef CFG_EOI_BTDC_DEG
#define CFG_EOI_BTDC_DEG  60.0f
#endif

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
  /* window crosses 0 */
  return (deg >= start || deg < end) ? 1u : 0u;
}

