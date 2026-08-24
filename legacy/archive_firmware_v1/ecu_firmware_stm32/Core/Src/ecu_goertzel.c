#include "ecu_goertzel.h"
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

void Goertzel_Init(GoertzelState *g, uint16_t N, float fs_hz, float f0_hz)
{
  if (!g || N < 4 || fs_hz < 1000.0f || f0_hz < 100.0f) return;

  g->nTarget = N;
  g->n = 0;
  g->q1 = 0.0f;
  g->q2 = 0.0f;

  float k = (float)N * (f0_hz / fs_hz);
  g->kNorm = k;
  float omega = 2.0f * (float)M_PI * k / (float)N;
  g->coeff = 2.0f * cosf(omega);
}

void Goertzel_Reset(GoertzelState *g)
{
  if (!g) return;
  g->q1 = 0.0f;
  g->q2 = 0.0f;
  g->n = 0;
}

void Goertzel_Push(GoertzelState *g, float sample)
{
  if (!g) return;
  float q0 = sample + g->coeff * g->q1 - g->q2;
  g->q2 = g->q1;
  g->q1 = q0;
  if (g->n < 65535u)
    g->n++;
}

float Goertzel_Power(const GoertzelState *g)
{
  if (!g) return 0.0f;
  float p = g->q1 * g->q1 + g->q2 * g->q2 - g->q1 * g->q2 * g->coeff;
  if (p < 0.0f) p = 0.0f;
  return p;
}

float Goertzel_ProcessBuffer(const float *buf, uint16_t len,
                             float fs_hz, float f0_hz)
{
  if (!buf || len < 4) return 0.0f;

  GoertzelState g;
  Goertzel_Init(&g, len, fs_hz, f0_hz);

  float mean = 0.0f;
  for (uint16_t i = 0; i < len; i++)
    mean += buf[i];
  mean /= (float)len;

  for (uint16_t i = 0; i < len; i++)
    Goertzel_Push(&g, buf[i] - mean);

  return Goertzel_Power(&g);
}

float Goertzel_KnockIntensity(const float *buf, uint16_t len,
                              float fs_hz, float f1_hz, float f2_hz)
{
  if (!buf || len < 8) return 0.0f;
  float p1 = Goertzel_ProcessBuffer(buf, len, fs_hz, f1_hz);
  float p2 = Goertzel_ProcessBuffer(buf, len, fs_hz, f2_hz);
  float p = (p1 > p2) ? p1 : p2;
  /* Normalize roughly by block length so thresholds are less N-dependent */
  float norm = (float)len * (float)len;
  if (norm < 1.0f) norm = 1.0f;
  return p / norm;
}
