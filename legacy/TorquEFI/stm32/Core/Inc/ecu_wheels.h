/**
 * TorquEFI wheel profiles — derived from Ardu-Stim wheel_defs.h (David J. Andruczyk)
 * Used for decoder setup (teeth / missing / cam), not edge-stim arrays.
 */
#ifndef ECU_WHEELS_H
#define ECU_WHEELS_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Cam / phase modes the firmware understands today */
typedef enum {
  ECU_CAM_NONE = 0,       /* crank-only; 360° cycleHalf toggle */
  ECU_CAM_SINGLE = 1,     /* one pulse per 720° (sequential) */
  ECU_CAM_HALFMOON = 2,   /* 360° high / 360° low style */
  ECU_CAM_MULTI = 3       /* multi-tooth cam (informational) */
} EcuCamMode;

typedef struct {
  uint8_t     id;           /* stable ID for CFG:WHEEL,id */
  const char *name;         /* UI / GETCFG string */
  uint8_t     teeth;        /* nominal tooth count including missing slots */
  uint8_t     missing;      /* consecutive missing teeth at gap */
  EcuCamMode  cam;
  uint8_t     supported;    /* 1 = basic missing-tooth decoder OK */
} EcuWheelProfile;

/*
 * IDs align with common Ardu-Stim WheelType order where practical.
 * unsupported complex patterns still listed so tuner can show "needs advanced decoder".
 */
static const EcuWheelProfile ECU_WHEEL_TABLE[] = {
  /* id, name, teeth, missing, cam, supported */
  {  3, "60-2",              60, 2, ECU_CAM_NONE,     1 },
  {  4, "60-2 + cam",        60, 2, ECU_CAM_SINGLE,   1 },
  {  5, "60-2 + halfmoon",   60, 2, ECU_CAM_HALFMOON, 1 },
  {  6, "36-1",              36, 1, ECU_CAM_NONE,     1 },
  {  7, "24-1",              24, 1, ECU_CAM_NONE,     1 },
  {  8, "4-1 + cam",          4, 1, ECU_CAM_SINGLE,   1 },
  {  9, "8-1",                8, 1, ECU_CAM_NONE,     1 },
  { 10, "6-1 + cam",          6, 1, ECU_CAM_SINGLE,   1 },
  { 11, "12-1 + cam",        12, 1, ECU_CAM_SINGLE,   1 },
  { 12, "40-1 Ford V10",     40, 1, ECU_CAM_NONE,     1 },
  {  0, "Dizzy 4-cyl",        2, 0, ECU_CAM_NONE,     1 },
  {  1, "Dizzy 6-cyl",        3, 0, ECU_CAM_NONE,     1 },
  {  2, "Dizzy 8-cyl",        4, 0, ECU_CAM_NONE,     1 },
  { 16, "12-3",              12, 3, ECU_CAM_NONE,     1 },
  { 17, "36-2-2-2 H4",       36, 2, ECU_CAM_NONE,     0 }, /* multi-gap */
  { 18, "36-2-2-2 H6",       36, 2, ECU_CAM_NONE,     0 },
  { 24, "GM LS1 24x+cam",    24, 0, ECU_CAM_MULTI,    0 },
  { 25, "GM 58x + 4x cam",   60, 2, ECU_CAM_MULTI,    1 }, /* 58x ≈ 60-2 */
  { 28, "36-1 + 2nd trig",   36, 1, ECU_CAM_SINGLE,   1 },
  { 35, "24-2 + 2nd trig",   24, 2, ECU_CAM_SINGLE,   1 },
  { 48, "Miata 99-05",       36, 1, ECU_CAM_SINGLE,   1 },
  { 49, "12 even + cam",     12, 0, ECU_CAM_SINGLE,   1 },
  { 50, "24 even + cam",     24, 0, ECU_CAM_SINGLE,   1 },
  { 51, "Subaru 6/7",         6, 0, ECU_CAM_MULTI,    0 },
  { 52, "GM 7X",              7, 0, ECU_CAM_NONE,     0 },
  { 63, "BMW N20 58x",       60, 2, ECU_CAM_MULTI,    1 },
  { 65, "36-2 + 1 cam",      36, 2, ECU_CAM_SINGLE,   1 },
  { 66, "GM 40 OSS",         40, 0, ECU_CAM_NONE,     1 },
};

#define ECU_WHEEL_COUNT  (sizeof(ECU_WHEEL_TABLE)/sizeof(ECU_WHEEL_TABLE[0]))

static inline const EcuWheelProfile *ECU_WheelById(uint8_t id)
{
  for (unsigned i = 0; i < ECU_WHEEL_COUNT; i++) {
    if (ECU_WHEEL_TABLE[i].id == id)
      return &ECU_WHEEL_TABLE[i];
  }
  return 0;
}

static inline const EcuWheelProfile *ECU_WheelByIndex(unsigned idx)
{
  if (idx >= ECU_WHEEL_COUNT) return 0;
  return &ECU_WHEEL_TABLE[idx];
}

#ifdef __cplusplus
}
#endif
#endif /* ECU_WHEELS_H */
