/*
 * ecu_config.h – generated / edited by Engine Setup Tool
 * Do not hand-edit unless you know the profiles; re-run engine_setup_tool.py
 */
#ifndef ECU_CONFIG_H
#define ECU_CONFIG_H

/* ── Engine ─────────────────────────────────────────────────── */
#define CFG_CYLINDERS        4
#define CFG_TEETH            36
#define CFG_MISSING          1
#define CFG_TRIG_ANGLE       30
#define CFG_RPM_LIMIT        7000
#define CFG_FAN_C            95

/* 0 = Speed-Density (MAP), 1 = Alpha-N (TPS) */
#define CFG_LOAD_ALPHA_N     0

/* 0 = dumb coil (ECU controls dwell), 1 = smart coil (logic-level fire only) */
#define CFG_COIL_SMART       0

/* Dwell (dumb coils only) – microseconds @ 14 V nominal */
#define CFG_DWELL_NOM_US     3000
#define CFG_DWELL_MIN_US     1500
#define CFG_DWELL_MAX_US     4500

/* ── Sensor profile IDs (documentation) ─────────────────────── */
/* CLT: 0=Generic 10k NTC  1=GM 3/8 NTC  2=Bosch 024  3=Honda */
/* IAT: 0=Generic 10k NTC  1=GM open-element  2=Bosch air  3=Honda */
/* MAP: 0=MPX4250 10-250kPa  1=MPX5700 15-700kPa  2=GM 1bar  3=GM 2bar */
#define CFG_PROF_CLT         0
#define CFG_PROF_IAT         0
#define CFG_PROF_MAP         0

/* MAP: engMap = MAP_OFFSET_KPA + volts * MAP_GAIN_KPA_PER_V */
#define CFG_MAP_OFFSET_KPA   10.0f
#define CFG_MAP_GAIN_KPA_V   50.0f

/* Battery ADC voltage divider: Vbat = (ADC/1023 * Vref) * ratio
 * Example: 30k top / 10k bottom → 4.0;  20k / 10k → 3.0 */
#define CFG_BAT_DIVIDER      3.00f
#define CFG_BAT_ADC_REF_V    5.00f

/*
 * NTC thermistors (CLT / IAT)
 * Wiring: 5V — Rpull — ADC — NTC — GND
 * 0 = use beta equation (recommended); 1 = use ADC/temp tables below
 */
#define CFG_USE_NTC_TABLE    0
#define CFG_NTC_R0_OHM       10000.0f   /* NTC resistance @ 25 °C */
#define CFG_NTC_BETA         3950.0f    /* typical 10k NTC */
#define CFG_NTC_PULLUP_OHM   10000.0f   /* bias resistor to 5V (use 1000 for many OE sensors) */

/* Optional table curves (only if CFG_USE_NTC_TABLE = 1)
 * Defaults for 10k NTC + 10k pull-up @ 5V */
#define CFG_ECT_N            13
static const float CFG_ECT_TEMP[13] = {
  -10, 0, 10, 20, 25, 30, 40, 50, 60, 70, 80, 90, 100
};
static const uint16_t CFG_ECT_ADC[13] = {
  /* 10k/10k divider approx */
  820, 760, 690, 620, 512, 480, 400, 330, 270, 220, 180, 150, 125
};

#define CFG_IAT_N            11
static const float CFG_IAT_TEMP[11] = {
  -20, -10, 0, 10, 20, 25, 30, 40, 50, 60, 80
};
static const uint16_t CFG_IAT_ADC[11] = {
  880, 820, 760, 690, 620, 512, 480, 400, 330, 270, 180
};

#endif /* ECU_CONFIG_H */
