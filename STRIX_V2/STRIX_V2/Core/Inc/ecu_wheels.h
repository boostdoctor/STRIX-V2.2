/**
 * Crank / cam wheel profiles for STRIX V2
 */
#ifndef ECU_WHEELS_H
#define ECU_WHEELS_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
  ECU_CAM_NONE = 0,
  ECU_CAM_SINGLE = 1,
  ECU_CAM_DUAL = 2
} EcuCamMode;

typedef struct {
  uint8_t     id;
  const char *name;
  uint8_t     teeth;
  uint8_t     missing;
  EcuCamMode  cam;
} EcuWheelProfile;

/** Lookup by id; returns NULL if unknown */
const EcuWheelProfile *ECU_WheelById(uint8_t id);

#ifdef __cplusplus
}
#endif

#endif /* ECU_WHEELS_H */
