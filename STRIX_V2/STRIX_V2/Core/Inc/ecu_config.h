/* STRIX V2 firmware — 12x22 maps, sensor enables */
#ifndef ECU_CONFIG_H
#define ECU_CONFIG_H
#include <stdint.h>

#define CFG_CYLINDERS        4
#define CFG_TEETH            36
#define CFG_MISSING          1
#define CFG_TRIG_ANGLE       30
#define CFG_RPM_LIMIT        7000
#define CFG_FAN_C            95
#define CFG_LOAD_ALPHA_N     0
#define CFG_SEQUENTIAL       1
#define CFG_COIL_SMART       1
#define CFG_DWELL_NOM_US     3000
#define CFG_DWELL_MIN_US     1500
#define CFG_DWELL_MAX_US     4500
#define CFG_MAP_OFFSET_KPA   10.0f
#define CFG_MAP_GAIN_KPA_V   50.0f
#define CFG_BAT_DIVIDER      11.00f
#define CFG_BAT_ADC_REF_V    3.30f
#define CFG_USE_NTC_TABLE    0
#define CFG_NTC_R0_OHM       10000.0f
#define CFG_NTC_BETA         3950.0f
#define CFG_NTC_PULLUP_OHM   10000.0f
#define CFG_ADC_BITS         12
#define CFG_ADC_MAX          4095.0f
#define CFG_MAX_COILS        4
#define CFG_MAX_INJECTORS    4
#define CFG_EOI_BTDC_DEG    360.0f  /* EOI ° before compression TDC (360 = intake BDC) */
#endif
