#ifndef ECU_OC_H
#define ECU_OC_H
#include <stdint.h>
void ECU_Oc_Init(void);
void ECU_Oc_IRQ(void);
void ECU_Oc_PulseIgn(uint8_t ch, uint32_t delayUs, uint32_t widthUs);
void ECU_Oc_PulseInj(uint8_t ch, uint32_t delayUs, uint32_t widthUs);
uint8_t ECU_Oc_IgnBusy(uint8_t ch);
uint8_t ECU_Oc_InjBusy(uint8_t ch);
void ECU_Oc_CancelAll(void);
#endif
