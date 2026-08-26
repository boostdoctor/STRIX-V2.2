# STRIX V2.3

## Why
V2.2 crank path mixed Kalman leftovers, PLL stubs, tooth-count windows and
flash-on-SET. That dropped sync and could stall USB/CDC.

## Trigger (rusEFI idea, STRIX code)
rusEFI `TriggerDecoder` syncs on **gap ratios** of consecutive tooth times,
not absolute µs windows. V2.3 does the same in the TIM5 ISR:

- 36-1: accept this interval if it is ~1.5–2.7× the previous tooth
- 60-2: accept ~2.2–4.0× previous
- Integer only (no float / Kalman / PLL in IRQ)
- Cam lock still decided at the gap (3 hits / 5 misses)
- Unlock only after ~4 wheels with no gap-like interval, 6 times

We did **not** copy rusEFI sources (GPL-3). Same architecture, new code.
https://github.com/rusefi/rusefi/tree/338c50f007129bfb082238155055f2fd06f070c5/firmware/controllers/trigger

## Sensors
MAP and TPS: 8-sample ring buffer, published every 20 ms.

## Init stages (ECU_Init)
1. trigger  2. sensors  3. actuators  4. math/maps  5. core IRQs + IWDG

## Persist
Engine Settings OK does not SAVE (avoids CDC drop). Use Flash/Save at RPM 0.

## Wheel table (rusEFI-style shape)
`ECU_Trigger_Rebuild()` builds a per-wheel event table:
- `deg_x10[i]` — angle×10 of physical tooth i after the gap
- `gap_q8` — expected gap/tooth ratio (36-1 → 2.00, 60-2 → 3.00)
- While locked, a gap is only accepted at the table slot (after `phys` teeth)

Profiles: 12-1, 24-1, 24-2, 36-1, 36-1+cam, 36-2, 36-2+cam, 60-2, 60-2+cam, 60-2+dual, Custom.
