# STRIX V2 firmware modules (full split)

| File | Responsibility |
|------|----------------|
| `ecu_runtime.c/h` | Shared globals (maps, sync, sensors, config) |
| `ecu_internal.h` | Cross-module function prototypes |
| `ecu_util.c` | parse/clamp helpers |
| `ecu_decode.c` | Crank/cam capture, Kalman RPM, sync |
| `ecu_maps.c` | defaultMaps, lookupMaps, calcEngineLoad |
| `ecu_spark.c` | Ignition scheduling |
| `ecu_fuel.c` | Injection, DFCO, O2/trim, ASE/CSE |
| `ecu_features.c` | DTC, knock, motorsport, boost |
| `ecu_outputs.c` | FP/fan, VVT, ETB, start prime |
| `ecu_idle.c` | Closed-loop idle PID |
| `ecu_sensors.c` | Engineering units from ADC frame |
| `ecu_adc.c/h` | TIM9-triggered + DMA circular scan, `readAdc` |
| `ecu_cmd.c` | Serial protocol, telemetry, flash pack/save |
| `ecu_app.c` | GPIO init, ECU_Init, ECU_Loop, UART RX byte |
| `ecu_serial.c` | USB CDC rings |
| `ecu_flash.c` | NVM blob v8 |
| `ecu_settings.c` | Weak settings stubs (strong in cmd/app) |

## CubeIDE
Add all `Core/Src/ecu_*.c` to the build (including **ecu_adc.c**). Do **not** compile `ecu_app_monolith.c.bak`.  
ADC DMA / TIM9: see **CUBEMX_ADC_DMA.md**.

Original monolith backed up as `ecu_app_monolith.c.bak`.
