#ifndef ECU_GOERTZEL_H
#define ECU_GOERTZEL_H

#include <stdint.h>

/** Knock intensity estimate — implemented in ecu_runtime.c */
float Goertzel_KnockIntensity(const float *x, int n, float fs, float f1, float f2);

#endif
