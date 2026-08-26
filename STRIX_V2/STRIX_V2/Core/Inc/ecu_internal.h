#ifndef ECU_INTERNAL_H
#define ECU_INTERNAL_H
#include <stdint.h>
#include "ecu_flash.h"

void uartWrite(const char *s);
void uartErr(const char *cmd, const char *why);
int  parse_int(const char *s, int *out);
int  parse_float(const char *s, float *out);
int  clampi(int a, int lo, int hi);
int8_t clampAdv(int v);
uint8_t clampInj(float ms);

void defaultMaps(void);
void lookupMaps(float load, float rpm, int8_t *advOut, float *injOut);
float calcEngineLoad(void);

void scheduleCoils(uint32_t now);
void serviceInjection(void);
void serviceDfco(void);
void serviceO2ClosedLoop(void);
void serviceAfterStart(void);
float coldStartEnrichMul(void);
float afterStartMul(void);
float alsFuelMul(void);
float accelEnrichMul(void);
void  serviceAccelEnrich(void);
float flexFuelMul(void);
void  serviceFlexFuel(void);
void  ECU_Flex_Init(void);
float fuelTrimMul(void);
float totalTrimPct(void);

void serviceIdleControl(void);
float idleTargetFromEct(float ectC);
void serviceETB(void);
void serviceOutputs(void);
void serviceStartPrime(void);
void serviceVvtClosedLoop(void);
void serviceBoost(void);
void serviceMotorsport(void);
void serviceDtcSanity(void);
void servicePendingSave(void);

void readSensors(void);
void sendTelemetry(void);
void handleLine(char *line);
void handleUploadRow(char *line);
void fillFlashBlob(EcuFlashBlob *blob);
void ECU_Flash_ApplyExtras(const EcuFlashBlob *blob);
void ECU_ApplyWheelId(uint8_t id);

void allOutputsOff(void);
void ecuInjGpioInit(void);
uint8_t cylAtSlot(uint8_t slot);
float tdcDeg(uint8_t cyl);
uint8_t injSequentialActive(void);
uint8_t ignSequentialActive(void);
void ECU_CrankPoll(void);
float ntcBetaC(uint16_t adc);
float adcToPctCal(uint16_t adc, uint16_t closed, uint16_t open);
float etbLookup(float pedalPct, float rpm);
void vvtMapsDefault(void);
float wrapAngle(float a, float cycle);
uint8_t angleActive(float deg, float start, float end, float cycle);
uint32_t millis(void);
uint32_t micros(void);
uint16_t readAdc(uint32_t ch); /* ecu_adc.c — DMA or poll */
void ECU_Adc_Init(void);
void ECU_SanitizeMapBins(void);
void ECU_Adc_Stop(void);

void rpmKalmanReset(void); /* period median+IIR reset (name kept for link compat) */
uint16_t rpmKalmanUpdate(float z, float dt_s);
uint8_t syncQualityPct(void);
void ECU_CrankCapture(uint32_t capt);
void ECU_CamCapture(uint32_t capt);
void ECU_Cam2Capture(uint32_t capt);
void ECU_CrankCam_Start(void);

float afrToLambda(float afr);
float lambdaToAfr(float lam);
float computeIgnitionAdvance(int8_t mapAdv);
float o2FuelMul(void);
void ECU_Features_Init(void);
void ECU_Features_Service(void);
void ECU_Dtc_Clear(void);
uint8_t ECU_Dtc_Count(void);
uint16_t ECU_Dtc_Get(uint8_t index);
void ECU_SetCylTrim(uint8_t cyl, float pct);
float ECU_GetCylTrim(uint8_t cyl);
void ECU_SetVVT(uint8_t intake_pct, uint8_t exhaust_pct);
void ECU_SetThrottleTarget(float pct);
void ECU_EnableETB(uint8_t en);

#endif
