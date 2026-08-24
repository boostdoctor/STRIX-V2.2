# STRIX V2.2

Open ECU firmware + tuner for **STM32F411CE Black Pill**.

| Component | Path | Description |
|-----------|------|-------------|
| **Firmware** | `STRIX_V2/STRIX_V2/` | STM32CubeIDE project (sequential inj/ign, crank PLL, USB CDC) |
| **Tuner** | `STRIX_V2_py/` | PySide6 live tuner (maps, VE / Injector Duty, motorsport, AE) |
| **Docs** | root | Pinout PDF, user manual |

## Features (Rev 2.2)

- Missing-tooth crank decode + PLL, optional cam home (720° sequential)
- Fuel map modes: **VE %** or **Injector Duty (ms)** — auto-detected from ECU on connect
- Wideband closed-loop, flex fuel (PA6), VSS (PC15), launch VSS decay
- Closed-loop idle target RPM vs ECT, acceleration enrichment (TPS-dot)
- NVM flash persistence (no battery), live strip with VE% + injection time
- DFCO, ASE/WUE, cylinder trims, AFR target map, motorsport (launch / ALS / FFS)

## Quick start — Tuner

```bash
cd STRIX_V2_py
pip install -r requirements.txt
python main.py
```

USB serial **115200** (CDC). On connect, settings and maps are **read from the ECU** (no local override prompt).

## Quick start — Firmware

1. Open `STRIX_V2/STRIX_V2` in **STM32CubeIDE**
2. Build → Flash Black Pill (SWD)
3. USB device enumerates as CDC ACM

Pinout: see `STRIX_V2_BlackPill_Pinout.pdf`.

## Protocol (summary)

- Telemetry text lines: `KEY:value` pairs (`RPM`, `MAP`, `TPS`, `PW`, `VEMODE`, …)
- Commands: `SET:…`, `CFG:teeth,missing,trig`, `GETCFG`, `GETMAP`, `SAVE`
- Fuel mode: `SET:VEMODE,0|1` and `VEMODE` in `GETCFG`

## License / project

Maintained as **STRIX V2.2** — https://github.com/boostdoctor/STRIX-V2.2

## Legacy (archived)

Pre–STRIX V2.2 development (TorquEFI, Arduino sketch, early STM32) lives under [`legacy/`](legacy/). See `legacy/README.md`. Not for new flashes.
