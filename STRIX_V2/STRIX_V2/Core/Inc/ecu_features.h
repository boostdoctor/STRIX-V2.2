#ifndef ECU_FEATURES_H
#define ECU_FEATURES_H

#include <stdint.h>

void ECU_Features_Init(void);
void ECU_Features_Service(void);
void ECU_Dtc_Clear(void);
uint8_t ECU_Dtc_Count(void);
uint16_t ECU_Dtc_Get(uint8_t index);
void ECU_SetCylTrim(uint8_t cyl, float pct);
float ECU_GetCylTrim(uint8_t cyl);
float ECU_GetAfr(void);
uint8_t ECU_GetO2Mode(void);

void serviceDtcSanity(void);
void serviceKnockGoertzel(void);
void serviceMotorsport(void);
void serviceBoost(void);
void ECU_SetBoostTarget(float gauge_kpa);
void ECU_EnableBoost(uint8_t en);

#endif
