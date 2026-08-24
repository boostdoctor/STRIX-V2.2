/**
 * TorquEFI extended features — DTC, idle PID, cyl trim, wideband
 */
#ifndef ECU_FEATURES_H
#define ECU_FEATURES_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ── DTC codes (OBD-style 16-bit) ───────────────────────────── */
#define DTC_NONE        0x0000u
#define DTC_MAP_LOW     0x0100u  /* MAP ADC/kPa too low */
#define DTC_MAP_HIGH    0x0101u
#define DTC_TPS_RANGE   0x0110u  /* TPS inconsistent / out of cal */
#define DTC_ECT_OPEN    0x0120u  /* ECT ADC rail (open/short) */
#define DTC_ECT_HIGH    0x0121u
#define DTC_IAT_OPEN    0x0122u
#define DTC_BAT_LOW     0x0130u
#define DTC_BAT_HIGH    0x0131u
#define DTC_SYNC_LOSS   0x0200u  /* excessive sync losses */
#define DTC_CAM_LOSS    0x0201u
#define DTC_O2_STUCK    0x0300u  /* NB voltage stuck */
#define DTC_AFR_RANGE   0x0301u  /* WB AFR out of range */
#define DTC_MAX_ACTIVE  16

/* O2 sensor mode */
#define O2_MODE_OFF  0
#define O2_MODE_NB   1  /* narrowband voltage */
#define O2_MODE_WB   2  /* wideband linear → AFR */

void ECU_Features_Init(void);
void ECU_Features_Service(void);   /* ~10–100 Hz from ECU_Loop / 1kHz */

/* DTC */
void     ECU_Dtc_Clear(void);
uint8_t  ECU_Dtc_Count(void);
uint16_t ECU_Dtc_Get(uint8_t index);
uint16_t ECU_Dtc_ActiveMask(void); /* bit0.. for first 16 internal slots */

/* Wideband / AFR */
float    ECU_GetAfr(void);
uint8_t  ECU_GetO2Mode(void);

/* Per-cylinder fuel trim % (−25..+25) */
float    ECU_GetCylTrim(uint8_t cyl1based);
void     ECU_SetCylTrim(uint8_t cyl1based, float pct);

#ifdef __cplusplus
}
#endif

#endif
