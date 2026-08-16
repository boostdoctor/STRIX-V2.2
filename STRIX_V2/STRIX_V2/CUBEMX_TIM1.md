# CubeMX — TIM1 (ETB + VVT)

**MCU:** STM32F411CEU6 (Black Pill)  
**Role:** PWM outputs for drive-by-wire and cam solenoids  

| Channel | Pin | Label | Function |
|---------|-----|-------|----------|
| **CH1** | **PA8** | ETB_PWM | Electronic throttle / idle PWM |
| **CH2N** | **PB14** | VVT2 | Exhaust VVT (complementary) |
| **CH3** | **PA10** | VVT1 | Intake VVT |

Related GPIO (not TIM1 alternate):

| Pin | Label | Function |
|-----|-------|----------|
| **PA9** | ETB_DIR | H-bridge direction (GPIO output) |

---

## Prerequisites

- Clock tree: SYSCLK **96 MHz**, APB2 timers **96 MHz** (APB2 /1).
- **SYS → Debug = Serial Wire**.

---

## Step 1 — Pinout view

1. **PA8** → **TIM1_CH1**
2. **PA10** → **TIM1_CH3**
3. **PB14** → **TIM1_CH2N**
4. **PA9** → **GPIO_Output** (ETB_DIR)

---

## Step 2 — TIM1 mode

**Timers → TIM1**

| Setting | Value |
|---------|--------|
| Clock Source | **Internal Clock** |
| Channel 1 | **PWM Generation CH1** |
| Channel 2 | **PWM Generation CH2N** |
| Channel 3 | **PWM Generation CH3** |

---

## Step 3 — Parameter Settings

| Parameter | Value | Notes |
|-----------|--------|------|
| Prescaler (PSC) | **95** | 1 µs tick @ 96 MHz |
| Counter Period (ARR) | **999** | PWM ≈ **1 kHz** |
| auto-reload preload | Enable | |
| Counter Mode | Up | |

### Channel 1 (PA8 ETB)

| Parameter | Value |
|-----------|--------|
| Mode | PWM mode 1 |
| Pulse (CCR1) | 0 |
| CH Polarity | High |

### Channel 3 (PA10 VVT1)

| Parameter | Value |
|-----------|--------|
| Mode | PWM mode 1 |
| Pulse (CCR3) | 0 |
| CH Polarity | High |

### Channel 2N (PB14 VVT2)

| Parameter | Value |
|-----------|--------|
| Mode | PWM mode 1 |
| Pulse (CCR2) | 0 |
| CHN Polarity | High |

### Break / dead-time (CubeMX: TIM1 → Parameter Settings → Break and Dead Time)

TIM1 is an **advanced** timer. Dead time inserts a delay between turning off one side of a complementary pair and turning on the other (CH2 ↔ CH2N). ETB (CH1) and VVT1 (CH3) are single-ended; only **CH2/CH2N** uses complementary outputs if both were enabled — with **only CH2N** active, dead time has little effect unless you also enable CH2.

| Parameter | Recommended (STRIX VVT/ETB) | Notes |
|-----------|----------------------------|--------|
| **Dead Time** | **0** | Low-side solenoid / open-drain drivers — no half-bridge shoot-through risk |
| Off-State Selection (OSSR/OSSI) | Disable | Outputs forced idle when MOE=0 |
| Lock Level | Off | Keep configurable |
| Break State | **Disable** | No BKIN pin used on Black Pill pinout |
| Break Polarity | High | N/A if break disabled |
| Break Filter | 0 | |
| Automatic Output (AOE) | **Disable** | Software sets MOE via PWM start |
| Break & Break2 | Disable | |

#### When to use non-zero dead time

Use only if **both** TIM1_CH2 and TIM1_CH2N drive a **half-bridge** (high-side + low-side MOSFET):

| Target dead time | DTG (approx @ 96 MHz, CKD=DIV1) |
|------------------|----------------------------------|
| 0.5 µs | ~48 |
| 1 µs | ~96 |
| 2 µs | ~192 |

Formula (CKD = clock division 1):

```text
t_dt ≈ DTG / f_tim          for DTG ≤ 127
```

CubeMX field is **Dead Time** (0–255 DTG code). Start with **0** for VVT/ETB as wired in STRIX.

#### HAL equivalent (`MX_TIM1_Init`)

```c
TIM_BreakDeadTimeConfigTypeDef bd = {0};
bd.OffStateRunMode  = TIM_OSSR_DISABLE;
bd.OffStateIDLEMode = TIM_OSSI_DISABLE;
bd.LockLevel        = TIM_LOCKLEVEL_OFF;
bd.DeadTime         = 0;                    /* DTG — 0 for solenoid drivers */
bd.BreakState       = TIM_BREAK_DISABLE;
bd.BreakPolarity    = TIM_BREAKPOLARITY_HIGH;
bd.AutomaticOutput  = TIM_AUTOMATICOUTPUT_DISABLE;
if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &bd) != HAL_OK) {
  Error_Handler();
}
```

**Do not** pass a wrong type (e.g. `int *`) into `HAL_TIMEx_ConfigBreakDeadTime` — must be `TIM_BreakDeadTimeConfigTypeDef *`.


---

## Step 4 — NVIC

TIM1 IRQs optional — leave **disabled** unless you add callbacks.

---

## Step 5 — GPIO

| Pin | Mode | Function |
|-----|------|----------|
| PA8 | AF PP High | TIM1_CH1 |
| PA10 | AF PP High | TIM1_CH3 |
| PB14 | AF PP High | TIM1_CH2N |
| PA9 | GPIO Output | ETB_DIR |

---

## Step 6 — Generate code

**GENERATE CODE** (keep user sections).

---

## Step 7 — Application hooks (`main.c`)

```c
HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);      /* ETB PA8 */
HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);      /* VVT1 PA10 */
HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);   /* VVT2 PB14 */
```

Duty example (0.0–1.0):

```c
__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1,
    (uint32_t)(duty * (htim1.Init.Period)));
```

---

## PWM frequency

```text
f_pwm = 96e6 / ((PSC+1)*(ARR+1)) = 96e6/(96*1000) = 1 kHz
```

| Target | PSC | ARR |
|--------|-----|-----|
| 1 kHz | 95 | 999 |
| 2 kHz | 95 | 499 |

---

## Checklist

- [ ] PA8 TIM1_CH1 · PA10 TIM1_CH3 · PB14 TIM1_CH2N  
- [ ] PA9 GPIO ETB_DIR  
- [ ] PSC=95 ARR=999 · break off  
- [ ] PWM_Start CH1+CH3 · PWMN_Start CH2  

## Related guides

- `CUBEMX_TIM2.md` — Cam1 PA15  
- `CUBEMX_TIM3.md` — Cam2 PB4  
- `STRIX_V2.ioc` — full project  
