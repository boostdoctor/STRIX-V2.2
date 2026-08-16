/**
 * Closed-loop idle control (DBW throttle adder or idle valve).
 */
#ifndef ECU_IDLE_H
#define ECU_IDLE_H
#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif
void  ECU_Idle_SetEnable(uint8_t en);
void  ECU_Idle_SetTargetRpm(uint16_t rpm);
void  ECU_Idle_SetGains(float kp, float ki, float kd);
void  ECU_Idle_Service(void); /* call from ECU_Loop */
uint8_t ECU_Idle_IsActive(void);
float ECU_Idle_ThrottlePct(void);
float ECU_Idle_TargetRpm(void);
#ifdef __cplusplus
}
#endif
#endif
