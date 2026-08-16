# CubeMX — TIM2 Cam1 (PA15)

**MCU:** STM32F411CEU6 (Black Pill)  
**Role:** Cam 1 (phase / 720° home) input capture  
**Pin:** **PA15** = TIM2_CH1  

Cam2 is separate: TIM3_CH1 on **PB4** (see `CUBEMX_TIM3.md`).

---

## Prerequisites

1. Open the project `.ioc`.
2. **System Core → SYS → Debug = Serial Wire**  
   - Required so **PA15** is not JTDI and **PB3/PB4** are free.

---

## Step 1 — Pinout view

1. Click **PA15**.
2. Select **TIM2_CH1**.
3. Confirm the pin shows `TIM2_CH1`.

If PA15 only offers JTDI / System:
- Set **SYS → Debug → Serial Wire** first, then re-assign PA15.

---

## Step 2 — TIM2 mode

**Pinout & Configuration → Timers → TIM2**

| Setting | Value |
|---------|--------|
| Clock Source | **Internal Clock** |
| Channel 1 | **Input Capture direct mode** |

Leave other channels off unless used elsewhere.

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
| Polarity Selection | **Rising Edge** (Both Edges if needed) |
| IC Selection | **Direct** |
| Prescaler | **No division** |
| Input Filter | **10** (8–12 OK) |

### Prescaler

Same as TIM3/TIM5 when timer clock = 96 MHz:

```text
PSC = (TimerClock_Hz / 1_000_000) - 1 = 95  →  1 µs ticks
```

---

## Step 4 — NVIC

| IRQ | Enabled | Preemption priority |
|-----|---------|---------------------|
| TIM2 global interrupt | **Yes** | **1** |

Match crank (TIM5) priority band so cam edges are not starved by USB.

---

## Step 5 — GPIO (from TIM2_CH1)

| Pin | Mode | Pull | Speed |
|-----|------|------|--------|
| PA15 | Alternate Function Push-Pull | No pull* | High |

\*Hall often needs external pull-up; VR needs external conditioner.

---

## Step 6 — Generate code

**GENERATE CODE** with user-code sections preserved.

---

## Step 7 — Application hooks (`main.c`)

```c
/* After MX_TIM2_Init() */
HAL_TIM_Base_Start(&htim2);
if (HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1) != HAL_OK) {
  Error_Handler();
}
```

Capture callback:

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

Use **one** `htim2` (CubeMX `tim.c` only).

---

## Checklist

- [ ] SYS Debug = Serial Wire  
- [ ] PA15 = TIM2_CH1  
- [ ] TIM2 IC CH1, PSC=95, ARR=65535, Filter≈10  
- [ ] TIM2 NVIC enabled, priority 1  
- [ ] `HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1)`  
- [ ] Callback → `ECU_CamCapture`  
- [ ] No duplicate `htim2` in `ecu_app.c`  

---

## Related timers

| Signal | Timer | Pin |
|--------|-------|-----|
| Crank | TIM5_CH1 | PA0 |
| Cam 1 | TIM2_CH1 | **PA15** |
| Cam 2 | TIM3_CH1 | PB4 |

