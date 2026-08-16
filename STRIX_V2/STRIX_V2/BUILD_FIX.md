# Build fix — 215 undeclared symbol errors

## Cause
The module split left many globals only in `ecu_runtime.c` without matching
`extern` declarations in `ecu_runtime.h`. Features/knock/boost/DTC code could
not see those symbols.

## Fixed in tree
1. **`Core/Inc/ecu_runtime.h`** — full `extern` list + DTC codes + KNK_* defines
2. **`Core/Src/ecu_runtime.c`** — coil/inj state, `cylTrimPct`, helpers:
   - `msRetardLookup`
   - `readClutch`
   - `Goertzel_KnockIntensity` (simple energy stub)

## CubeIDE — files to compile

**Must include**
- All `ecu_*.c` under `Core/Src` **except**:
  - `ecu_app_monolith.c.bak` (exclude / do not add)
- Cube-generated: `main.c`, `tim.c`, `adc.c`, `dma.c`, `gpio.c`, `usb_*`, HAL

**If you still get multiple-definition errors**
| Symbol | Action |
|--------|--------|
| `idleTargetFromEct` | Keep only in `ecu_idle.c` — remove body from `ecu_runtime.c` if duplicate |
| `IDLE_KP` etc. | Defined once in `ecu_runtime.c` only |
| `htim9` | Weak stub in `ecu_adc.c`; strong from `tim.c` after TIM9 enabled |

**If link still fails on one module**
Temporary recovery — compile **monolith only**:
1. Rename `ecu_app.c` → `ecu_app_split.c` (exclude)
2. Copy `ecu_app_monolith.c.bak` → `ecu_app.c`
3. Exclude: `ecu_decode`, `ecu_features`, `ecu_fuel`, `ecu_maps`, `ecu_spark`,
   `ecu_outputs`, `ecu_idle`, `ecu_sensors`, `ecu_cmd`, `ecu_util`, `ecu_runtime`
4. Keep: `ecu_flash`, `ecu_serial`, `ecu_adc`, `ecu_settings`

## After rebuild
1. Clean project
2. Rebuild
3. Flash; confirm `PROTO:2` and live MAP/TPS with DMA if configured
