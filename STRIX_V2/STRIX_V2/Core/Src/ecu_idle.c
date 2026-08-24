/* ecu_idle.c — auto-split from ecu_app.c */
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

/* Idle / VVT state lives in ecu_runtime.c — only functions here */

/* 5-point target idle RPM vs ECT (editable via SET:IDLETGT,i,ect,rpm) */
float idleTgtEctBins[5] = {-10.0f, 20.0f, 40.0f, 60.0f, 90.0f};
float idleTgtRpmTbl[5]  = {1400.0f, 1100.0f, 950.0f, 850.0f, 850.0f};

float idleTargetFromEct(float ectC)
{
  /* Linear interpolate target RPM vs coolant */
  if (ectC <= idleTgtEctBins[0])
    return idleTgtRpmTbl[0];
  if (ectC >= idleTgtEctBins[4])
    return idleTgtRpmTbl[4];
  for (int i = 0; i < 4; i++) {
    float e0 = idleTgtEctBins[i];
    float e1 = idleTgtEctBins[i + 1];
    if (ectC >= e0 && ectC <= e1) {
      float t = (e1 > e0) ? (ectC - e0) / (e1 - e0) : 0.0f;
      return idleTgtRpmTbl[i] + t * (idleTgtRpmTbl[i + 1] - idleTgtRpmTbl[i]);
    }
  }
  return idleTargetRpm > 700.0f ? idleTargetRpm : 850.0f;
}

void serviceIdleControl(void)
{
  uint32_t now = millis();
  if (idleLastMs == 0) idleLastMs = now;
  float dt = (float)(now - idleLastMs) * 0.001f;
  if (dt < 0.001f) dt = 0.001f;
  if (dt > 0.05f) dt = 0.05f;
  idleLastMs = now;

  float pedal = engPedal;
  if (pedal < 0.0f) pedal = 0.0f;

  /* Dashpot: capture tip-out (cal via SET:DASHPOT,gain,decay,max) */
  float dTps = prevTpsIdle - engTps;
  if (dTps >= DASHPOT_MIN_DTPS && prevTpsIdle >= DASHPOT_MIN_TPS) {
    float add = dTps * DASHPOT_GAIN;
    if (add > DASHPOT_MAX) add = DASHPOT_MAX;
    if (add > dashpotPct) dashpotPct = add;
  }
  /* Decay toward 0 - DASHPOT_DECAY is per-loop factor at ~100 Hz */
  {
    float dec = DASHPOT_DECAY;
    if (dec < 0.50f) dec = 0.50f;
    if (dec > 0.995f) dec = 0.995f;
    /* Scale decay by dt so calibration is roughly independent of loop rate */
    float steps = dt * 100.0f;
    if (steps < 0.5f) steps = 0.5f;
    if (steps > 5.0f) steps = 5.0f;
    for (int i = 0; i < (int)steps; i++)
      dashpotPct *= dec;
  }
  if (dashpotPct < 0.15f) dashpotPct = 0.0f;
  prevTpsIdle = engTps;

  if (!idleEnable || !etbEnable) {
    idleActive = 0;
    idleIntegral *= 0.95f;
    return;
  }

  /* Entry / exit hysteresis on pedal */
  /* Idle only when TPS below 5% (and pedal closed) */
  if (engTps > IDLE_ENTRY_TPS) {
    idleActive = 0;
  } else if (!idleActive) {
    if (pedal <= IDLE_ENTRY_PEDAL && syncLocked && rpmLive > 400)
      idleActive = 1;
  } else {
    if (pedal >= IDLE_EXIT_PEDAL || !syncLocked || rpmLive < 200)
      idleActive = 0;
  }

  if (!idleActive) {
    /* Decay integrator slowly when leaving idle */
    idleIntegral *= (1.0f - 2.0f * dt);
    if (idleThrottle > ETB_IDLE_PCT)
      idleThrottle += (ETB_IDLE_PCT - idleThrottle) * (2.0f * dt);
    return;
  }

  float tgtRpm = idleTargetFromEct(engEct);
  /* Blend toward configured hot idle */
  if (engEct >= 70.0f)
    tgtRpm = idleTargetRpm;

  float rpmErr = tgtRpm - (float)rpmLive;

  /* Anti-stall: large positive open if RPM collapsing */
  float antiStall = 0.0f;
  if (rpmLive < tgtRpm - 200.0f && rpmLive > 400) {
    antiStall = (tgtRpm - (float)rpmLive) * 0.02f;
    if (antiStall > 12.0f) antiStall = 12.0f;
  }

  idleIntegral += rpmErr * dt;
  if (idleIntegral > 800.0f) idleIntegral = 800.0f;
  if (idleIntegral < -200.0f) idleIntegral = -200.0f;
  /* Freeze integral if throttle at rail */
  if (idleThrottle >= IDLE_MAX_PCT - 0.5f && rpmErr > 0.0f)
    idleIntegral *= 0.98f;

  float deriv = (rpmErr - idlePrevRpmErr) / dt;
  idlePrevRpmErr = rpmErr;

  float u = IDLE_KP * rpmErr + IDLE_KI * idleIntegral + IDLE_KD * deriv;
  u += antiStall;
  u += dashpotPct * 0.5f;

  idleThrottle = u;
  if (idleThrottle < IDLE_MIN_PCT) idleThrottle = IDLE_MIN_PCT;
  if (idleThrottle > IDLE_MAX_PCT) idleThrottle = IDLE_MAX_PCT;
}



void ECU_Idle_SetEnable(uint8_t en)
{
  idleEnable = en ? 1u : 0u;
  if (!idleEnable) {
    idleActive = 0;
    idleIntegral = 0.0f;
  }
}

void ECU_Idle_SetTargetRpm(uint16_t rpm)
{
  if (rpm < 500) rpm = 500;
  if (rpm > 2000) rpm = 2000;
  idleTargetRpm = (float)rpm;
}

void ECU_Idle_SetGains(float kp, float ki, float kd)
{
  (void)kp; (void)ki; (void)kd;
  /* Gains come from IDLE_KP/KI/KD macros; keep API for serial SET:IDLE */
}

void ECU_Idle_Service(void)
{
  serviceIdleControl();
}

uint8_t ECU_Idle_IsActive(void)
{
  return idleActive ? 1u : 0u;
}

float ECU_Idle_ThrottlePct(void)
{
  return idleThrottle;
}

float ECU_Idle_TargetRpm(void)
{
  return idleTargetRpm;
}
