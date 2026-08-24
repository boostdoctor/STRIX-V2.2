# Legacy development (pre–STRIX V2.2)

Historical code kept for reference. **Active development is STRIX V2.2** (`STRIX_V2/` firmware + `STRIX_V2_py/` tuner).

| Path | Description |
|------|-------------|
| `TorquEFI/` | Early project package: Arduino sketch, first STM32 port, simple tuner, docs |
| `TorquEFI/arduino_legacy/` | Arduino `ecu_firmware.ino` + `ecu_config.h` |
| `TorquEFI/stm32/` | First Black Pill / CubeIDE application sources |
| `TorquEFI/tuner/` | Early Python tuner scripts |
| `TorquEFI_STM32F411/` | Alternate early CubeIDE snapshot |
| `ecu_firmware_stm32/` | Early STM32 notes / Core sketch |
| `arduino/` | Standalone Arduino firmware copy |

Do not flash these for new builds — use `STRIX_V2/STRIX_V2` and `STRIX_V2_py`.
