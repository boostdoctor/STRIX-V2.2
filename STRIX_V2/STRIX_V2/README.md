# STRIX V2 Firmware (STM32F411)

Archived V1 sources: `artifacts/archive/firmware_v1/`

## Changes vs V1

| Item | V2 |
|------|-----|
| Map size | **12 × 22** |
| Load bins | 0.20…2.40 (= **20…240 kPa** at engMap/100); max scale via tuner |
| Load clamp | up to **5.0** (500 kPa sensors) |
| MAP telemetry | Whole kPa (`MAP:%.0f`) |
| Sensor enables | `SET:SENS:ECT,0/1` · `IAT` · `O2` · `MAP` · `TPS` |
| Device ID | `GETUID` → `UID:STRIXV2…` |
| Flash blob | Version **7**, 12×22 maps |
| Parse | Integer/`parse_float` (nano-safe) |

## Files to copy into CubeIDE project

```
Core/Src/ecu_app.c
Core/Src/ecu_flash.c
Core/Src/ecu_serial.c
Core/Inc/ecu_flash.h
Core/Inc/ecu_config.h
Core/Inc/ecu_serial.h
STM32F411CEUX_FLASH.ld   # FLASH length 384K, NVM sector 7
```

Keep existing CubeMX-generated `main.c`, HAL, USB CDC.  
Do **not** leave duplicate `Src/ecu_app.c` outside `Core/Src`.

## Protocol (software match)

- `GETMAP` / `UPLOAD:ADV` / `UPLOAD:INJ` / `SET:A` / `SET:I` / `SAVE`
- `SET:SENS:ECT,1` etc.
- `GETUID`

## Note

V1 flash blobs (15×22, version ≤6) will not load into V2 — re-upload maps after flash.


## Pin change: INJ1 → PB15 (Cam2 on PB4)

| Function | Pin |
|----------|-----|
| INJ1 | **PB15** |
| INJ2–4 | PB5, PB6, PB7 |
| Cam2 | **PB4** TIM3_CH1 |

Copy `Core/Inc/ecu_pins.h` into the CubeIDE project. In CubeMX: PB15 = GPIO_Output, PB4 = TIM3_CH1.
