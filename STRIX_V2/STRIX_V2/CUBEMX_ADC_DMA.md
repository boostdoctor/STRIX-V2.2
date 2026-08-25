# CubeMX / firmware ADC + FLEX (STRIX V2.2)

## Analog inputs (ADC1 continuous scan + DMA2 Stream0)

| Rank | Channel | Pin | Signal |
|------|---------|-----|--------|
| 1 | IN1 | PA1 | MAP |
| 2 | IN2 | PA2 | TPS |
| 3 | IN3 | PA3 | CLT / ECT |
| 4 | IN4 | PA4 | IAT |
| 5 | IN5 | PA5 | O2 |
| 6 | IN7 | PA7 | VBATT |

**Settings required in `MX_ADC1_Init` (already patched in `main.c`):**

- Scan Conversion = **Enabled**
- Continuous Conversion = **Enabled**
- Nbr Of Conversion = **6**
- DMA Continuous Requests = **Enabled**
- External trigger = Software start
- Sampling: 84 cycles (MAP/TPS/O2/VBATT), 480 cycles (CLT/IAT)

DMA: DMA2 Stream0 Channel0, circular, half-word, memory increment. Linked via `ECU_DMA_ADC1_Config()` from MSP / `ECU_Adc_Init()`.

## FLEX — PA6 frequency (not ADC)

Ethanol / flex sensor is a **frequency** input:

| Hz | Ethanol |
|----|---------|
| 40 | 0% (E0) |
| 160 | 100% (E100) |

- Pin: **PA6** digital input (pull-up), edge timing via `micros()` in `serviceFlexFuel()`
- `ECU_Flex_Init()` called from `ECU_Init()`
- `engFlexHz` holds measured frequency; `engEthanol` maps 40–160 Hz → 0–100% when `gFlexEnable` is set

Do **not** configure PA6 as ADC_IN6.
