#ifndef STRIX_FW_VERSION
#define STRIX_FW_VERSION  "2.2.0"
#endif
/* STRIX V2.2 firmware — 12x22 maps, sensor enables */
#ifndef ECU_CONFIG_H
#define ECU_CONFIG_H
#include <stdint.h>

#define CFG_CYLINDERS        4
#define CFG_TEETH            60
#define CFG_MISSING          2
#define CFG_TRIG_ANGLE       30
#define CFG_RPM_LIMIT        7000
#define CFG_FAN_C            95
#define CFG_FAN_ENABLE       0  /* off until IO enables */
#define CFG_LOAD_ALPHA_N     0
#define CFG_SEQUENTIAL       0
/* Cam home is expected once per cam rev (2 crank revs); only declare it lost
 * after this many crank revolutions without an edge. */
#define CAM_HOME_CHECK_REVS  4UL
#define CFG_COIL_SMART       1
#define CFG_DWELL_NOM_US     3000
#define CFG_DWELL_MIN_US     1500
#define CFG_DWELL_MAX_US      8000u
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
#define CFG_EOI_BTDC_DEG     60.0f  /* injection ends this many ° before compression TDC */
#endif

#define CFG_WHEEL_ID         7   /* 36-2 + cam home */
#define CFG_TACHO_ENABLE     0
#define CFG_TACHO_PPR        2
