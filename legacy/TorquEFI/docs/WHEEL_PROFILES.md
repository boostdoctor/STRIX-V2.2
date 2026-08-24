# Wheel profiles (Ardu-Stim / wheel_defs.h)

TorquEFI uses **decoder parameters** (teeth, missing, cam mode) derived from
Ardu-Stim `wheel_defs.h` names — not the full stim edge arrays.

## Commands

| Command | Action |
|---------|--------|
| `SET:WHEEL,<id>` | Select profile by id |
| `CFG:WHEEL,<id>` | Same |
| `CFG:teeth,missing,angle` | Manual override |
| `GETWHEEL` | Reply `WHEEL:id,teeth,missing,cam,name` |

## Common IDs

| ID | Profile | Teeth | Missing |
|----|---------|-------|---------|
| 6 | 36-1 | 36 | 1 |
| 3 | 60-2 | 60 | 2 |
| 4 | 60-2 + cam | 60 | 2 |
| 7 | 24-1 | 24 | 1 |
| 25 | GM 58x | 60 | 2 |
| 48 | Miata 99-05 | 36 | 1 |

Complex multi-gap wheels (36-2-2-2, LS1 24x, Subaru 6/7) are listed but
marked unsupported for the basic missing-tooth decoder.

## Tuner

STATUS panel → **Wheel** dropdown → **Set**
Also sends `CFG:teeth,missing,trig` for compatibility.
