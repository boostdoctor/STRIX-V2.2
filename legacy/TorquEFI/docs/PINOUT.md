# TorquEFI STM32F411 Black Pill — 4-cyl USB-only pinout

```
                    USB-C (CDC tuner)
                         │
                    PA11 DM / PA12 DP
                         │
    3V3  ─ [ Black Pill STM32F411CEU6 top view, USB up ] ─  GND
    B10  FP relay                          B12  Pedal ADC
    B1   IGN2                              B13  Clutch EXTI
    B0   IGN1                              A2   TPS
    R    NRST                              A1   MAP
    3V3                                    A0   CRANK (TIM5_CH1)
    G0   (free)                            B9   Fan relay
    …                                      B8   Boost
                                           5V
                                           G
                                           3V3
```

Exact silk varies by board vendor. Functional map below is authoritative.

---

## Left / right functional map

```
        ┌─────────────────────────────────────┐
        │           USB-C  (PA11/PA12 CDC)     │
        ├──────────────┬──────────────────────┤
   SWD  │ PA14 SWCLK   │  PA13 SWDIO     SWD  │
        ├──────────────┼──────────────────────┤
 Crank  │ PA0  TIM5_CH1│  PA1  MAP ADC        │
        │ PA2  TPS     │  PA3  CLT            │
        │ PA4  IAT     │  PA5  O2             │
        │ PA6  KNOCK   │  PA7  Vbatt          │
        │ PA8  ETB PWM │  PA9  ETB DIR        │
   VVT1 │ PA10 TIM1_CH3│  PA15 CAM TIM2_CH1   │
        ├──────────────┼──────────────────────┤
  IGN1  │ PB0          │  PB1  IGN2           │
  IGN3  │ PB2          │  PB3  IGN4           │
  INJ1  │ PB4          │  PB5  INJ2           │
  INJ3  │ PB6          │  PB7  INJ4           │
 Boost  │ PB8          │  PB9  FAN            │
   FP   │ PB10         │  PB12 PEDAL          │
        │              │  PB13 CLUTCH         │
   VVT2 │ PB14 TIM1_CH2N│  PB15 spare         │
        ├──────────────┼──────────────────────┤
  LED   │ PC13         │  PC14/15 free (LSE)  │
        │ PC6/7 USART6 optional fallback      │
        └──────────────┴──────────────────────┘
```

---

## By function

| Function | Pin | Resource |
|----------|-----|----------|
| **USB CDC DM/DP** | PA11 / PA12 | Tuner VCP |
| **Crank** | PA0 | TIM5_CH1 |
| **Cam** | PA15 | TIM2_CH1 |
| **IGN 1–4** | PB0–PB3 | GPIO / TIM3 |
| **INJ 1–4** | PB4–PB7 | GPIO / TIM3/4 |
| **Boost** | PB8 | TIM4_CH3 |
| **ETB PWM / DIR** | PA8 / PA9 | TIM1_CH1 + GPIO |
| **Fan / Fuel pump** | PB9 / PB10 | Relays |
| **MAP TPS CLT IAT** | PA1–PA4 | ADC1 |
| **O2 / Knock / Vbat** | PA5 / **PA6** / PA7 | ADC1 |
| **Pedal / Clutch** | PB12 / PB13 | ADC / EXTI |
| **LED** | PC13 | Status |
| **SWD** | PA13 / PA14 | Debug only |

## Open (spare)

| Pin | Suggestion |
|-----|------------|
| PB15 | Extra relay / tach |
| PA10 / PB14 | **VVT1 / VVT2** (assigned) |
| PC14, PC15 | Leave free if LSE fitted |
| PC6, PC7 | Optional USART6 |

## Do not reassign

- **PA11/PA12** — USB only (no CAN)  
- **PA13/PA14** — SWD  
- **PA0 / PA15** — crank / cam  

---

*Matches `Core/Inc/ecu_pins.h` (4-cyl USB-only).*
