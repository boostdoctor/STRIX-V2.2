# TorquEFI Basic — STM32F411 Full Sequential (CubeIDE)

Per-cylinder ignition & injection using **crank (PA0)** + **cam (PA15)** for 720° phase.

## Pin map (as specified)

### Outputs
| Function | Pin | Resource |
|----------|-----|----------|
| Ignition 1–4 | PB0–PB3 | TIM3_CH3/4 + GPIO |
| Ignition 5 | PA10 | TIM1_CH3 |
| Ignition 6 | PC15 | GPIO |
| Injector 1–4 | PB4–PB7 | TIM3/TIM4 PWM-capable |
| Injector 5–6 | PB14–PB15 | TIM1 complementary |
| Boost | PB8 | TIM4_CH3 |
| ETB PWM / DIR | PA8 / PA9 | TIM1_CH1 + GPIO |
| Fan / Fuel pump | PB9 / PB10 | GPIO relays |
| LED | PC13 | Status |

### Inputs
| Function | Pin | Resource |
|----------|-----|----------|
| Crank | PA0 | TIM5_CH1 |
| Cam | PA15 | TIM2_CH1 |
| MAP TPS CLT IAT | PA1–PA4 | ADC1 |
| O2 / FuelP / Vbat | PA5–PA7 | ADC1 |
| Pedal | PB12 | ADC1_IN12 |
| Clutch | PB13 | EXTI |
| CAN | PA11/PA12 | CAN1 |

**Conflict:** supplied map listed PB14 as both Inj5 and fuel-pressure ADC — firmware uses **PB14 = Inj5**, fuel pressure on **PA6**.

**UART tuner:** PA2/PA3 are ADC — use **USART6 on PC6/PC7** (or USB CDC).

## CubeMX checklist
1. SYSCLK 100 MHz (HSE 25 MHz if fitted)
2. TIM5 CH1 IC rising → crank IRQ
3. TIM2 CH1 IC rising → cam IRQ
4. ADC1 channels on PA1–PA7, PB12
5. USART6 115200 on PC6/PC7
6. GPIO outs for IGN1–6, INJ1–6, FAN, FP
7. Optional: TIM PWM later for timed inject/dwell

## main.c hooks
```c
#include "ecu_app.h"
/* after MX_*_Init */
ECU_Init();
HAL_TIM_IC_Start_IT(&htim5, TIM_CHANNEL_1);
HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1);
HAL_UART_Receive_IT(&huart6, &uartRxByte, 1);

while (1) { ECU_Loop(); }

/* callbacks */
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) {
  if (htim->Instance == TIM5)
    ECU_CrankCapture(HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1));
  if (htim->Instance == TIM2)
    ECU_CamCapture(HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1));
}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
  if (huart->Instance == USART6) {
    ECU_UART_RxByte(uartRxByte);
    HAL_UART_Receive_IT(&huart6, &uartRxByte, 1);
  }
}
```

## Sequential behaviour
- **Cam sync** → `crankDeg` 0–720°, one spark & one inject event per cylinder per cycle
- **No cam** → falls back to 360° semi-sequential / wasted-spark style
- Firing order 4-cyl: 1-3-4-2; 6-cyl: 1-5-3-6-2-4
- `CFG_COIL_SMART 1` = short logic pulse (IGBT / smart coil)
- Same text protocol as Python tuner (115200)

## Files
```
Core/Inc/ecu_pins.h   ecu_config.h   ecu_app.h
Core/Src/ecu_app.c
```
