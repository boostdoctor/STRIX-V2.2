/**
 * Crank wheel profiles + rusEFI-style trigger shape table (STRIX original).
 */
#ifndef ECU_WHEELS_H
#define ECU_WHEELS_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define ECU_TRIG_MAX 64

typedef enum {
  ECU_CAM_NONE = 0,
  ECU_CAM_SINGLE = 1,
  ECU_CAM_DUAL = 2
} EcuCamMode;

typedef struct {
  uint8_t     id;
  const char *name;
  uint8_t     teeth;    /* nominal slots including missing */
  uint8_t     missing;
  EcuCamMode  cam;
} EcuWheelProfile;

/**
 * Built shape: physical teeth after the gap are index 0..phys-1.
 * deg_x10[i] = crank angle ×10 of that tooth (0 = first edge after gap).
 * Ratios are Q8 (1.00 = 256, 2.00 = 512).
 */
typedef struct {
  uint8_t  id;
  uint8_t  count;
  uint8_t  missing;
  uint8_t  phys;
  uint8_t  cam;
  uint16_t deg_x10[ECU_TRIG_MAX];
  uint16_t gap_q8;
  uint16_t gap_lo_q8;
  uint16_t gap_hi_q8;
  uint16_t tooth_lo_q8;
  uint16_t tooth_hi_q8;
} EcuTriggerShape;

const EcuWheelProfile *ECU_WheelById(uint8_t id);
void ECU_Trigger_Rebuild(uint8_t teeth, uint8_t missing, uint8_t cam, uint8_t id);
const EcuTriggerShape *ECU_Trigger_Shape(void);
uint16_t ECU_Trigger_AngleX10(uint8_t toothIndex);
uint8_t  ECU_Trigger_ExpectGapAfter(uint8_t teethSinceGap);

#ifdef __cplusplus
}
#endif

#endif
