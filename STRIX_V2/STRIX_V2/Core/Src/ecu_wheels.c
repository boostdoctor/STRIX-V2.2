#include "ecu_wheels.h"

static const EcuWheelProfile kWheels[] = {
  { 0,  "12-1",     12, 1, ECU_CAM_NONE },
  { 1,  "12-1+cam", 12, 1, ECU_CAM_SINGLE },
  { 2,  "24-1",     24, 1, ECU_CAM_NONE },
  { 3,  "24-2",     24, 2, ECU_CAM_NONE },
  { 4,  "36-1",     36, 1, ECU_CAM_NONE },
  { 5,  "36-1+cam", 36, 1, ECU_CAM_SINGLE },
  { 6,  "36-1",     36, 1, ECU_CAM_NONE },
  { 7,  "36-2+cam", 36, 2, ECU_CAM_SINGLE },
  { 8,  "60-2",     60, 2, ECU_CAM_NONE },
  { 9,  "60-2+cam", 60, 2, ECU_CAM_SINGLE }, /* V2.2 default */
  { 10, "60-2+dual",60, 2, ECU_CAM_DUAL },
};

const EcuWheelProfile *ECU_WheelById(uint8_t id)
{
  for (unsigned i = 0; i < sizeof(kWheels) / sizeof(kWheels[0]); i++) {
    if (kWheels[i].id == id)
      return &kWheels[i];
  }
  /* fallback 36-1 */
  return &kWheels[6];
}
