/**
 * STRIX V2.3 trigger — rusEFI-inspired gap-RATIO decoder
 *
 * Architecture (same idea as rusEFI TriggerDecoder, original code):
 *   SHAFT_PRIMARY  = crank TIM5 CH1 PA0
 *   SHAFT_SECONDARY= cam   TIM2 CH1 PA15
 *   Sync when dt_this / dt_prev ≈ (missing + 1)   e.g. 36-1 → ~2.0
 *   ISR is integer-only. Unlock: 4 wheels × 6 misses.
 *
 * rusEFI sources are GPLv3 — they are NOT vendored here.
 * Ref: github.com/rusefi/rusefi  .../firmware/controllers/trigger
 */
#ifndef ECU_TRIGGER_H
#define ECU_TRIGGER_H
#include <stdint.h>
void ECU_CrankCapture(uint32_t capt);
void ECU_CamCapture(uint32_t capt);
void ECU_Cam2Capture(uint32_t capt);
void ECU_CrankCam_Start(void);
#endif
