# EOI-based injection

## Formula

- EOI angle = compression TDC − `gEoiBtdc` (° on 720° or 360° cycle)
- SOI angle = EOI − PW°  
- PW° = pulse_width_µs × 360 / revolution_µs  

## Config

| Symbol / command | Meaning |
|------------------|---------|
| `CFG_EOI_BTDC_DEG` | Default EOI (° BTDC), in `ecu_config.h` |
| `SET:EOI,<deg>` | Runtime EOI (10–400) |
| `OK:EOI,…` | Confirmation |

## Requirements

- Crank sync (`SYNC`)
- For full sequential: cam home (`CAM`) + `CFG_SEQUENTIAL 1`
- Without cam: 360° bank injection (1+4 / 2+3)

## Code

- `stm32/Core/Src/ecu_app.c` → `serviceInjection()`
- Pins: `ecu_pins.h` → `ECU_INJ_HI/LO`
