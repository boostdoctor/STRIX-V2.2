# STRIX V2 GUI

## Features (current)

- **12×22** ignition & fuel maps (fit window, no zoom/drag)
- Load axis **MAP kPa** (20–max, default max **240**, up to **500**) or **TPS %**
- MAP values displayed as **whole numbers**
- Live strip always: **RPM · TPS · MAP · ECT · IAT · SYNC** (+ optional fields in Program settings)
- Fixed-size live value boxes
- **Auto-connect** last COM port; local **device ID** (`~/.strix_v2/`)
- On connect: **auto GETMAP**
- Cell edits **push to ECU RAM** immediately — only **Flash** button (no Upload)
- Sensor calibration presets (Bosch/GM MAP, IAT, ECT) + enable toggles
- O2: Disabled / Narrowband (5 pts) / Wideband (10× AFR–V), 0–5 V controller scale
- Engine settings **.tcal** import/export; locked when RPM > 0
- Trigger wizard when RPM = 0
- Selection drag + **Ctrl+P** percent change + Interpolate
- Crosshair hysteresis (ignores brief RPM drops)

## Run

```bash
cd STRIX_V2
pip install -r requirements.txt
python main.py
```

## Note on firmware

Current STM32 firmware may still use **15×22** maps. V2 reads the first **12** rows.
A firmware update to 12×22 + sensor enable flags is recommended for full alignment.
