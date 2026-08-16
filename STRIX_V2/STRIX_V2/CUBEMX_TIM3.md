# CubeMX — TIM3 Cam2 (PB4)

**MCU:** STM32F411CEU6 (Black Pill)  
**Role:** Cam 2 input capture  
**Pin:** PB4 = TIM3_CH1 (INJ1 is on **PB15**, not PB4)

---

## Prerequisites

1. Open the project `.ioc` in CubeMX or CubeIDE (Device Configuration Tool).
2. **System Core → SYS → Debug = Serial Wire**  
   - Required so PB4 is not NJTRST.

---

## Step 1 — Pinout view

1. Click **PB4**.
2. Choose **TIM3_CH1**.
3. Confirm pin label shows `TIM3_CH1`.
4. Confirm **PB15** is **GPIO_Output** (Injector 1).

---

## Step 2 — TIM3 mode

**Pinout & Configuration → Timers → TIM3**

| Setting | Value |
|---------|--------|
| Clock Source | **Internal Clock** |
| Channel 1 | **Input Capture direct mode** |

Leave CH2/CH3/CH4 disabled unless you use them elsewhere.

---

## Step 3 — Parameter Settings

| Parameter | Value |
|-----------|--------|
| Prescaler (PSC) | **95** |
| Counter Period (ARR) | **65535** |
| auto-reload preload | Disable |

**Input Capture Channel 1**

| Parameter | Value |
|-----------|--------|
| Polarity Selection | **Rising Edge** (or Both Edges if needed) |
| IC Selection | **Direct** |
| Prescaler | **No division** |
| Input Filter | **10** (8–12 acceptable) |

### Prescaler note

With SYSCLK 96 MHz and APB1 = /2, timer clock on APB1 is **96 MHz**.

```text
PSC = (TimerClock_Hz / 1_000_000) - 1 = 96e6/1e6 - 1 = 95
```

→ 1 µs ticks (same as TIM5/TIM2 crank/cam1).

If your APB1 timer clock differs, recalculate PSC.

---

## Step 4 — NVIC

**TIM3 → NVIC Settings** (or **System Core → NVIC**)

| IRQ | Enabled | Preemption priority |
|-----|---------|---------------------|
| TIM3 global interrupt | **Checked** | **1** (same band as TIM2/TIM5) |

---

## Step 5 — GPIO (auto from TIM3_CH1)

CubeMX sets PB4 AF. Verify:

| Pin | Mode | Pull | Speed |
|-----|------|------|--------|
| PB4 | Alternate Function Push-Pull | No pull (or Pull-up if open sensor) | High |

Hall sensors often need an external pull-up; VR needs a conditioner — not set in CubeMX alone.

---

## Step 6 — Generate code

1. **Project Manager → Project** — correct toolchain (STM32CubeIDE).
2. **Code Generator** — “Keep user code when re-generating”.
3. **GENERATE CODE**.

---

## Step 7 — Application hooks (`main.c`)

After `MX_TIM3_Init()`:

```c
HAL_TIM_Base_Start(&htim3);
if (HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_1) != HAL_OK) {
  Error_Handler();
}
```

In `HAL_TIM_IC_CaptureCallback`:

```c
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM5)
    ECU_CrankCapture(HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1));
  else if (htim->Instance == TIM2)
    ECU_CamCapture(HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1));
  else if (htim->Instance == TIM3)
    ECU_Cam2Capture(HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1));
}
```

### Avoid double `htim3`

- Prefer CubeMX `htim3` in `tim.c`.
- Do **not** also define `htim3` in `ecu_app.c` (link error: multiple definition).

---

## Checklist

- [ ] SYS Debug = Serial Wire  
- [ ] PB4 = TIM3_CH1  
- [ ] PB15 = GPIO_Output (INJ1)  
- [ ] TIM3 IC CH1, PSC=95, ARR=65535, Filter≈10  
- [ ] TIM3 NVIC enabled, priority 1  
- [ ] `HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_1)` in `main`  
- [ ] Callback routes TIM3 → `ECU_Cam2Capture`  
- [ ] Single `htim3` definition only  

---

## Related

- Crank: TIM5_CH1 on **PA0**  
- Cam 1: TIM2_CH1 on **PA15**  
- Full pin map: `TorquEFI/stm32/PINOUT.md`  
