# CubeMX — Cam 2 (TIM3_CH1 / PB4) + related timers

Target: **STM32F411CEU6** Black Pill, project **STRIX / TorquEFI**.

---

## 1. Open the `.ioc`

Open your CubeMX / CubeIDE `.ioc` (e.g. `STRIX.ioc`).

---

## 2. Pinout view — assign Cam 2

| Pin | Mode | Function |
|-----|------|----------|
| **PB4** | TIM3_CH1 | Cam 2 input capture |

Steps:
1. Click **PB4**
2. Select **TIM3_CH1**
3. Confirm it is **not** still NJTRST (System → SYS → Debug = **Serial Wire** only so PB3/PB4 are free)

Already in use (do not change unless intentional):

| Pin | Function |
|-----|----------|
| PA0 | TIM5_CH1 — Crank |
| PA15 | TIM2_CH1 — Cam 1 |
| PA11/PA12 | USB_OTG_FS |
| PA8/PA9/PA10 | ETB / DIR / VVT1 |
| PB0–PB3 | IGN 1–4 |
| PB5–PB7, PB15 | INJ |
| PB8 | TIM4_CH3 Boost |
| PB9/PB10 | Fan / Fuel pump |
| PB14 | VVT2 |

---

## 3. TIM3 configuration (Cam 2)

**Timers → TIM3**

### Mode
- **Clock Source:** Internal Clock  
- **Channel 1:** Input Capture direct mode  

### Parameter Settings
| Parameter | Value | Notes |
|-----------|--------|------|
| Prescaler (PSC) | **95** | With 96 MHz timer clock → 1 MHz (1 µs tick). Adjust if your APB1 timer clock differs: `PSC = (TIMclk_Hz / 1e6) - 1` |
| Counter Period (ARR) | **65535** | 16-bit max |
| auto-reload preload | Disable | |
| **Input Capture Channel 1** | | |
| Polarity Selection | Rising Edge | Or Both if needed |
| IC Selection | Direct |
| Prescaler | No division |
| **Filter** | **10** | 8–12 OK; reduces noise |

### NVIC Settings
- **TIM3 global interrupt:** Enabled  
- Preemption priority: **1** (same as TIM2/TIM5)

---

## 4. Confirm Crank / Cam 1 (should already exist)

### TIM5 (Crank PA0)
- CH1 Input Capture  
- PSC ≈ 95 (1 µs), ARR = 0xFFFFFFFF (32-bit)  
- Filter ≈ 10  
- NVIC TIM5 enabled, priority 1  

### TIM2 (Cam 1 PA15)
- CH1 Input Capture  
- PSC ≈ 95, ARR = 65535  
- Filter ≈ 10  
- NVIC TIM2 enabled, priority 1  
- PA15 often needs **Disable JTAG** / Serial Wire only  

---

## 5. Clock tree (USB + timers)

Typical Black Pill **HSI → PLL 96 MHz**:
- SYSCLK = 96 MHz  
- AHB = 96 MHz  
- APB1 = 48 MHz → **timer clock on APB1 = 96 MHz** (x2 when APB1 prescaler ≠ 1)  
- USB: PLLQ so **48 MHz** to USB (e.g. PLLQ = 4 with N=192, M=16 on HSI)

Verify: **Clock Configuration** tab → USB frequency = **48 MHz**.

---

## 6. Generate code

1. **Project Manager → Code Generator**
   - Generate peripheral init as pair of `.c/.h` files per peripheral (optional)
   - Keep user code sections  
2. **GENERATE CODE**

---

## 7. After generation — firmware hooks

### Option A — Use CubeMX `MX_TIM3_Init()` (preferred once TIM3 is in `.ioc`)

In `main.c` USER CODE:
```c
MX_TIM3_Init();
HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_1); /* Cam2 PB4 */
```

Remove or comment out:
```c
ECU_TIM3_Cam2_Init();  /* only if htim3 is defined in ecu_app.c */
```

If both define `htim3`, you get a **duplicate symbol** link error. Keep **one** definition:
- CubeMX: `htim3` in `tim.c`  
- Delete `TIM_HandleTypeDef htim3;` and `ECU_TIM3_Cam2_Init()` body from `ecu_app.c`, leave `ECU_Cam2Capture()`  

### Option B — No TIM3 in CubeMX yet

Keep:
```c
ECU_TIM3_Cam2_Init();
HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_1);
```
MSP must still configure PB4 AF2 + TIM3 IRQ (see `stm32f4xx_hal_msp.c`).

### Capture callback (`main.c`)
```c
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM5) {
    ECU_CrankCapture(HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1));
  } else if (htim->Instance == TIM2) {
    ECU_CamCapture(HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1));
  } else if (htim->Instance == TIM3) {
    ECU_Cam2Capture(HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1));
  }
}
```

### IRQ (`stm32f4xx_it.c`)
```c
extern TIM_HandleTypeDef htim3;
void TIM3_IRQHandler(void) { HAL_TIM_IRQHandler(&htim3); }
```

---

## 8. Wiring

- Cam 2 Hall (or conditioned VR) → **3.3 V** logic into **PB4**  
- Common ground with MCU  
- Internal pull-up is enabled in MSP; external 3.3–10 kΩ to 3.3 V is fine if open-collector  

---

## 9. Verify

1. Build & flash  
2. Connect tuner / serial  
3. Spin engine or pulse PB4  
4. Telemetry should show **`CAM2:1`** after valid edges  

---

## Quick checklist

- [ ] SYS Debug = Serial Wire (PB4 free)  
- [ ] PB4 = TIM3_CH1  
- [ ] TIM3 IC CH1, filter ~10, PSC for 1 µs  
- [ ] TIM3 NVIC enabled  
- [ ] Code gen → Start_IT + callback + IRQ  
- [ ] Single `htim3` definition  
- [ ] Telemetry `CAM2:` present  
