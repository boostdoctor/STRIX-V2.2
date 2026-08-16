/**
 * STRIX V2 — Independent Watchdog (IWDG)
 *
 * LSI-based; survives main clock failure.
 * Default timeout ~1.0 s (safe for main loop; flash path must kick).
 *
 * CubeMX optional: if MX_IWDG_Init exists, call ECU_Watchdog_Init() after it
 * or skip Cube and use this module alone.
 */
#ifndef ECU_WATCHDOG_H
#define ECU_WATCHDOG_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** Start IWDG. Safe to call once from ECU_Init. */
void ECU_Watchdog_Init(void);

/** Kick the dog — call from ECU_Loop and long operations (flash). */
void ECU_Watchdog_Kick(void);

/** 1 if IWDG was started */
uint8_t ECU_Watchdog_IsEnabled(void);

#ifdef __cplusplus
}
#endif

#endif /* ECU_WATCHDOG_H */
