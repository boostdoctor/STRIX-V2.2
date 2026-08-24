#ifndef ECU_APP_H
#define ECU_APP_H
#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif
void ECU_Init(void);
void ECU_Loop(void);
void ECU_CrankCapture(uint32_t capt);
void ECU_CamCapture(uint32_t capt);
void ECU_Cam2Capture(uint32_t capt);
void ECU_TIM3_Cam2_Init(void);
void ECU_UART_RxByte(uint8_t b);
void ECU_1kHzTick(void);
/** VVT PWM duty 0–100%: intake (PA10), exhaust (PB14) */
void ECU_SetVVT(uint8_t intake_pct, uint8_t exhaust_pct);
void ECU_SetThrottleTarget(float pct);
void ECU_EnableETB(uint8_t en);
/** Gauge boost target kPa (0 = off). MAP feedback closed-loop. */
void ECU_SetBoostTarget(float gauge_kpa);
void ECU_EnableBoost(uint8_t en);
void ECU_EnableO2CL(uint8_t en);
void ECU_SetLTFT(float pct);
void ECU_ResetFuelTrim(void);
#ifdef __cplusplus
}
#endif
#endif
