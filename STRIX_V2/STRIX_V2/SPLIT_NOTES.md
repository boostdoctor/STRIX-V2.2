# Full ecu_app split — notes

## What was done
Monolithic `ecu_app.c` (~4100 lines) was split into:

- `ecu_runtime.c/h` — shared globals + millis helpers
- `ecu_internal.h` — cross-module prototypes  
- `ecu_util.c`, `ecu_decode.c`, `ecu_maps.c`, `ecu_spark.c`, `ecu_fuel.c`
- `ecu_features.c`, `ecu_outputs.c`, `ecu_idle.c`, `ecu_sensors.c`, `ecu_cmd.c`
- `ecu_app.c` — Init / Loop / GPIO / UART byte

Original preserved as `ecu_app_monolith.c.bak` (do not add to build).

## CubeIDE
1. Exclude `ecu_app_monolith.c.bak` from build.
2. Add every `Core/Src/ecu_*.c` except the `.bak`.
3. Ensure include path has `Core/Inc`.
4. Build; fix any remaining undefined refs by declaring them in `ecu_runtime.h` / defining in `ecu_runtime.c`.

## If link errors
Most failures are missing `extern` globals. Copy the symbol from the `.bak` into `ecu_runtime.c` and declare in `ecu_runtime.h`.

## Rollback
```
cp Core/Src/ecu_app_monolith.c.bak Core/Src/ecu_app.c
# remove other ecu_*.c from build except flash/serial/settings
```
