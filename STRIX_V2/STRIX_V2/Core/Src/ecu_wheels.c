#include "ecu_wheels.h"

/* IDs must match tuner WHEEL_PROFILES 1:1. */
static const EcuWheelProfile kWheels[] = {
  { 0,  "Custom",     36, 1, ECU_CAM_NONE   },
  { 1,  "12-1",       12, 1, ECU_CAM_NONE   },
  { 2,  "24-1",       24, 1, ECU_CAM_NONE   },
  { 3,  "24-2",       24, 2, ECU_CAM_NONE   },
  { 4,  "36-1",       36, 1, ECU_CAM_NONE   },
  { 5,  "36-1+cam",   36, 1, ECU_CAM_SINGLE },
  { 6,  "36-1",       36, 1, ECU_CAM_NONE   },
  { 7,  "36-2",       36, 2, ECU_CAM_NONE   },
  { 8,  "60-2",       60, 2, ECU_CAM_NONE   },
  { 9,  "60-2+cam",   60, 2, ECU_CAM_SINGLE },
  { 10, "60-2+dual",  60, 2, ECU_CAM_DUAL   },
  { 11, "36-2+cam",   36, 2, ECU_CAM_SINGLE },
};

static EcuTriggerShape gShape;

const EcuWheelProfile *ECU_WheelById(uint8_t id)
{
  for (unsigned i = 0; i < sizeof(kWheels) / sizeof(kWheels[0]); i++) {
    if (kWheels[i].id == id)
      return &kWheels[i];
  }
  return 0;
}

void ECU_Trigger_Rebuild(uint8_t teeth, uint8_t missing, uint8_t cam, uint8_t id)
{
  if (teeth < 4) teeth = 4;
  if (teeth > ECU_TRIG_MAX) teeth = ECU_TRIG_MAX;
  if (missing < 1) missing = 0;
  if (missing >= teeth) missing = 1;

  gShape.id = id;
  gShape.count = teeth;
  gShape.missing = missing;
  gShape.phys = (uint8_t)(teeth - missing);
  if (gShape.phys < 2) gShape.phys = 2;
  gShape.cam = cam;

  /* Equal slots around 360°. Tooth i after the gap sits at i * 360/count. */
  for (uint8_t i = 0; i < ECU_TRIG_MAX; i++) {
    if (i < gShape.phys)
      gShape.deg_x10[i] = (uint16_t)((uint32_t)i * 3600u / (uint32_t)teeth);
    else
      gShape.deg_x10[i] = 0;
  }

  /* Gap spans (missing+1) tooth periods — rusEFI sync ratio. */
  uint16_t gq = (uint16_t)(((uint16_t)missing + 1u) * 256u);
  gShape.gap_q8 = gq;
  gShape.gap_lo_q8 = (uint16_t)((gq * 3u) / 4u);   /* 75% */
  gShape.gap_hi_q8 = (uint16_t)((gq * 4u) / 3u);   /* 133% */
  if (gShape.gap_lo_q8 < 300) gShape.gap_lo_q8 = 300;
  /* Normal tooth ~1.0× previous, wide enough for accel */
  gShape.tooth_lo_q8 = 160;  /* 0.63× */
  gShape.tooth_hi_q8 = 400;  /* 1.56× */
}

const EcuTriggerShape *ECU_Trigger_Shape(void)
{
  if (gShape.count < 4) {
    /* First use before Rebuild — 60-2 default */
    ECU_Trigger_Rebuild(60, 2, ECU_CAM_SINGLE, 9);
  }
  return &gShape;
}

uint16_t ECU_Trigger_AngleX10(uint8_t toothIndex)
{
  const EcuTriggerShape *s = ECU_Trigger_Shape();
  if (toothIndex >= s->phys)
    return 0;
  return s->deg_x10[toothIndex];
}

uint8_t ECU_Trigger_ExpectGapAfter(uint8_t teethSinceGap)
{
  const EcuTriggerShape *s = ECU_Trigger_Shape();
  /* After phys physical teeth the next interval is the missing-tooth gap */
  return (teethSinceGap + 1u >= s->phys) ? 1u : 0u;
}
