/**
 * Map lookup (bilinear) — STRIX V2 12×22
 * Implementation currently in ecu_app.c; this header is the public API.
 */
#ifndef ECU_MAPS_H
#define ECU_MAPS_H
#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif
void ECU_LookupMaps(float load, float rpm, int8_t *advOut, float *injOutMs);
uint8_t ECU_MapCellR(void);
uint8_t ECU_MapCellC(void);
#ifdef __cplusplus
}
#endif
#endif
