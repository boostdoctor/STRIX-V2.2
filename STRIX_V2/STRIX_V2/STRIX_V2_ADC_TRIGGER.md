# STM32F411 ADC1 external triggers

**TIM9 TRGO is NOT available** on ADC1 for STM32F411 (RM0383).
CubeMX will not list it — that is correct.

## Recommended (STRIX V2)

| Setting | Value |
|---------|--------|
| Scan Conversion | Enabled |
| Continuous Conversion | **Enabled** |
| DMA Continuous Requests | **Enabled** |
| External Trigger | **Software start** / none |
| Nbr Of Conversion | 8 |

Firmware: `HAL_ADC_Start_DMA()` runs free continuous multi-rank scan into `adcDmaBuf[]`.

## If you need a hardware timer trigger instead

Valid **ADC1** sources on F411 include:

| CubeMX name | Notes for STRIX |
|-------------|-----------------|
| **Timer 3 Trigger Out** | TIM3 also used for Cam2 IC — set TRGO=Update OK |
| **Timer 2 Trigger Out** | TIM2 Cam1 — TRGO=Update OK |
| **Timer 4 CH4** | TIM4 CH3 is boost; CH4 free only if pin free |
| **Timer 1 TRGO** | Shares TIM1 with ETB/VVT |
| **Timer 5 CH1** | **Do not use** — PA0 crank IC |
| EXTI line 11 | Not useful for periodic scan |

**Do not use TIM9** for ADC on F411.

### Optional TIM4_TRGO setup
1. TIM4: Master → TRGO = Update Event (keep CH3 PWM for boost)
2. ADC1: Continuous = Disable, Trigger = Timer 4 Trigger Out, Rising
3. Ensure TIM4 is started (`HAL_TIM_PWM_Start` / base start)
