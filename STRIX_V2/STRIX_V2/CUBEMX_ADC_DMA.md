# CubeMX — Timer-triggered ADC + DMA (STRIX V2)

Target: **STM32F411CEU6**, 8-channel regular sequence, **TIM9 TRGO** @ 1 kHz, **DMA2** circular.

## 1. Enable DMA controller

- Pinout / System Core → **DMA**
- Click **Add** (or open DMA settings from ADC)
- No need to add streams by hand if you attach from ADC (step 3)

## 2. TIM9 — scan trigger (1 kHz)

TIM2/3/5 are crank/cam; TIM1/4 are PWM. Use **TIM9** (APB2).

| Setting | Value |
|---------|--------|
| Clock Source | Internal Clock |
| Prescaler | **95** (96 MHz timer clock → 1 MHz tick, same as other TIMs) |
| Counter Period | **999** (1 MHz / 1000 = **1 kHz** update) |
| auto-reload preload | Enable |
| **TRGO** | **Update Event** |
| IRQ | Optional (not required for ADC trigger) |

Formula:  
`scan_Hz = TIMCLK / ((PSC+1) * (ARR+1))`  
With PSC=95, ARR=999, TIMCLK=96 MHz → 1000 Hz.

## 3. ADC1 — regular sequence + DMA + external trigger

### Parameter settings

| Parameter | Value |
|-----------|--------|
| Clock Prescaler | PCLK2 div4 (or div2 if within F4 ADC limits) |
| Resolution | 12 bit |
| Data Alignment | Right |
| **Scan Conversion** | **Enabled** |
| **Continuous Conversion** | **Disabled** (one sequence per TRGO) |
| Discontinuous | Disabled |
| **DMA Continuous Requests** | **Enabled** |
| **External Trigger Conversion Source** | **Timer 9 Trigger Out event** |
| External Trigger Edge | Rising edge |
| Nbr Of Conversion | **8** |
| EOC Selection | EOC flag at end of sequence (optional) |

### Rank table (must match `ecu_adc.h` indices)

| Rank | Channel | Pin | Label |
|------|---------|-----|--------|
| 1 | IN1 | PA1 | MAP |
| 2 | IN2 | PA2 | TPS |
| 3 | IN3 | PA3 | CLT |
| 4 | IN4 | PA4 | IAT |
| 5 | IN5 | PA5 | O2 |
| 6 | IN6 | PA6 | KNOCK |
| 7 | IN7 | PA7 | VBATT |
| 8 | IN12 | PB12 | PEDAL |

Sampling time suggestion:

- MAP / TPS / O2 / Knock / Pedal / VBATT: **84 or 144 cycles**
- CLT / IAT (NTC): **480 cycles**

### DMA settings (from ADC1 → DMA Settings → Add)

| Setting | Value |
|---------|--------|
| DMA Request | ADC1 |
| Stream | **DMA2 Stream0** (or Stream4) |
| Direction | Peripheral to Memory |
| Priority | High |
| Mode | **Circular** |
| Peripheral Increment | Disable |
| Memory Increment | Enable |
| Peripheral Data Size | Half Word |
| Memory Data Size | Half Word |

## 4. NVIC

- DMA2 Stream0/4 global interrupt: **optional** (circular continuous needs no ISR for basic use)
- TIM9: not required for TRGO

## 5. Init order in `main.c`

```c
MX_GPIO_Init();
MX_DMA_Init();      /* BEFORE ADC */
MX_ADC1_Init();
MX_TIM9_Init();
/* ... other timers, USB ... */

ECU_Serial_Init();
ECU_Init();         /* calls ECU_Adc_Init() → Start_DMA + TIM9 base start */
```

## 6. Firmware API

```c
#include "ecu_adc.h"

ECU_Adc_Init();           /* once */
uint16_t map = readAdc(ECU_ADC_CH_MAP);  /* non-blocking if DMA up */
uint16_t raw = ECU_Adc_Raw(ECU_ADC_IX_TPS);
```

If DMA/TIM9 are missing from the Cube project, `readAdc()` falls back to **blocking poll** so the build still runs.

## 7. Alternative: continuous DMA (no timer)

If you skip TIM9:

- Continuous Conversion = **Enabled**
- External Trigger = Software start
- Still DMA circular, 8 ranks
- `HAL_ADC_Start_DMA` alone is enough; omit `HAL_TIM_Base_Start(&htim9)`

Timer trigger is preferred when you want a **fixed sample rate** (logging, knock window alignment, deterministic load).

## 8. Verify

1. After connect, live MAP/TPS should update smoothly.
2. Scope or debugger: `adcDmaBuf[]` changing at ~1 kHz.
3. Crank sync must remain solid (DMA priority ≤ TIM IC priority; keep TIM2/3/5 preemption higher than DMA if you enable DMA IRQ).
