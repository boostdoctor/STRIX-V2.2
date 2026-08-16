/**
 * Runtime engine settings ↔ flash blob settings block.
 */
#ifndef ECU_SETTINGS_H
#define ECU_SETTINGS_H
#include "ecu_flash.h"
#ifdef __cplusplus
extern "C" {
#endif
void ECU_Settings_Pack(EcuFlashSettings *out);
void ECU_Settings_Apply(const EcuFlashSettings *in);
#ifdef __cplusplus
}
#endif
#endif
