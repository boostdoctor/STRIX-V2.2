/**
 * Goertzel algorithm — single-bin DFT power for knock detection
 *
 * Detects energy near a target frequency (e.g. 6–8 kHz) in a short
 * ADC sample window after spark (ATDC).
 */
#ifndef ECU_GOERTZEL_H
#define ECU_GOERTZEL_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
  float coeff;      /* 2 * cos(2*pi*k/N) */
  float q1, q2;
  uint16_t n;       /* samples processed in this block */
  uint16_t nTarget; /* block length N */
  float kNorm;      /* target bin k = f_target * N / fs */
} GoertzelState;

void Goertzel_Init(GoertzelState *g, uint16_t N, float fs_hz, float f0_hz);
void Goertzel_Reset(GoertzelState *g);
void Goertzel_Push(GoertzelState *g, float sample);
float Goertzel_Power(const GoertzelState *g);

/** One-shot buffer process (mean-removed). */
float Goertzel_ProcessBuffer(const float *buf, uint16_t len,
                             float fs_hz, float f0_hz);

/**
 * Dual-frequency knock intensity: max power at f1 and f2 (normalized by N^2).
 * Returns relative intensity suitable for thresholding.
 */
float Goertzel_KnockIntensity(const float *buf, uint16_t len,
                              float fs_hz, float f1_hz, float f2_hz);

#ifdef __cplusplus
}
#endif

#endif
