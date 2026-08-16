#ifndef ECU_APP_H
#define ECU_APP_H

#include <stdint.h>

void ECU_Init(void);
void ECU_Loop(void);
void ECU_OnRxByte(uint8_t b);

#endif
