# TorquEFI Basic

STM32F411 (Black Pill) sequential ECU + Python tuner.

## Layout

```
TorquEFI/
  tuner/                 PC software (PySide6)
  stm32/                 CubeIDE application sources
  docs/                  Guides and troubleshooting
  arduino_legacy/        Previous Uno firmware (reference only)
```

## File reference

### Tuner (PC)

| File | Role |
|------|------|
| `tuner/ecu_tuner_aligned.py` | Main TorquEFI Basic tuner GUI |
| `tuner/engine_setup_tool.py` | Cylinders / trigger / sensors → config headers |
| `tuner/requirements.txt` | `PySide6`, `pyserial` |

### STM32 firmware — headers (`stm32/Core/Inc/`)

| File | Role |
|------|------|
| `ecu_app.h` | ECU API (`ECU_Init`, `ECU_Loop`, crank/cam capture) |
| `ecu_config.h` | Compile-time config (cyl, sequential, dwell, **EOI**) |
| `ecu_config_from_setup.h` | Generated overrides from engine setup tool |
| `ecu_pins.h` | GPIO / timer pin map (4-cyl USB-focused) |
| `ecu_serial.h` | USB CDC TX/RX API |
| `ecu_wheels.h` | Crank/cam **wheel profiles** (Ardu-Stim style IDs) |
| `ecu_flash.h` | Flash SAVE / map persistence |
| `ecu_goertzel.h` | Knock Goertzel helper |
| `main.h` / `stm32f4xx_it.h` | Cube IDE stubs / IRQ prototypes |

### STM32 firmware — sources (`stm32/Core/Src/`)

| File | Role |
|------|------|
| `ecu_app.c` | Core: crank sync, **EOI injection**, sequential coils, telemetry |
| `ecu_serial.c` | USB CDC write + `ECU_CDC_Receive` |
| `ecu_flash.c` | Flash read/write maps & trims |
| `ecu_goertzel.c` | Knock frequency detect |
| `main.c` | Init, `HAL_TIM_IC_Start_IT`, `ECU_Loop` |
| `stm32f4xx_it.c` | `TIM5` / `TIM2` / USB IRQs |
| `stm32f4xx_hal_msp.c` | Low-level MSP (prefer TIM MSP in Cube `tim.c`) |
| `cdc_hello_test.c` | Optional CDC “HELLO” diagnostic |
| `usbd_cdc_if_hook.c` | **Do not link** — use real `USB_DEVICE/App/usbd_cdc_if.c` |

### USB (from Cube project, not always in this tree)

| File | Role |
|------|------|
| `USB_DEVICE/App/usbd_cdc_if.c` | `CDC_Receive_FS` → `ECU_CDC_Receive` + `ReceivePacket` |
| `USB_DEVICE/App/usb_device.c` | `MX_USB_DEVICE_Init` |

### Docs

| File | Role |
|------|------|
| `docs/IMPORT_CUBEIDE.md` | Import / merge into CubeIDE |
| `docs/PINOUT.md` | Pin map |
| `docs/WHEEL_PROFILES.md` | Wheel IDs / `SET:WHEEL` |
| `docs/CDC_RX_DEBUG.md` | CDC buffer / RX debug |
| `docs/CDC_TROUBLESHOOT.md` | USB COM troubleshooting |
| `docs/NO_COM_DATA.md` | No telemetry on COM |
| `docs/CRANK_PA0.md` | PA0 / TIM5 crank |
| `stm32/IMPORT_CUBEIDE.md` | Copy of import notes |
| `stm32/PINOUT.md` | Copy of pinout |
| `stm32/Core/Src/crank_pa0_bench.md` | Bench checklist for RPM=0 |

### STRIX patch pack (artifacts root)

| File | Role |
|------|------|
| `STRIX_patched.zip` | Patched `main.c`, `tim.c`, `stm32f4xx_it.c`, `ecu_app.c`, … |
| `STRIX_patched/Core/Src/*` | Same files unpacked |

## Tuner

```bash
cd tuner
pip install -r requirements.txt
python ecu_tuner_aligned.py
```

Connect via **USB-C CDC** virtual COM after flashing STM32.

## STM32

1. CubeIDE: STM32F411CEU6, USB Device CDC, TIM1/2/4/5, ADC1  
2. Merge files from `stm32/Core` (and STRIX patches if used)  
3. See `docs/IMPORT_CUBEIDE.md` and `docs/PINOUT.md`  
4. Flash via SWD or DFU; tune over USB-C  

## Features

- 4-cyl sequential spark (720° with cam home)  
- **EOI-based injection** (`CFG_EOI_BTDC_DEG`, `SET:EOI,<deg>`)  
- Wheel profiles (`ecu_wheels.h`, `SET:WHEEL,<id>`)  
- USB CDC tuner, live maps, flash SAVE  
- Closed-loop ETB, boost, narrowband O2 STFT/LTFT  
- Dual VVT PWM, knock ADC + Goertzel helper  

## Serial protocol (short)

| Direction | Examples |
|-----------|----------|
| ECU → PC | `RPM:…`, `CFG:…`, `WHEEL:…`, `MAP:ADV` / rows / `MAP:END` |
| PC → ECU | `GETCFG`, `GETMAP`, `GETWHEEL`, `SET:WHEEL,id`, `SET:EOI,60`, `CFG:t,m,a`, `SAVE` |
