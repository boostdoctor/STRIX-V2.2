# No RPM from PA0 (crank)

## Signal path
```
Crank sensor → conditioner (0–3.3 V square) → PA0 → TIM5_CH1 IC IRQ → ECU_CrankCapture → rpmLive
```

## CubeMX must have
1. **TIM5** enabled, Channel 1 **Input Capture** on **PA0**
2. NVIC: **TIM5 global interrupt** enabled
3. GPIO PA0: AF **TIM5_CH1** (AF2), pull-up or pull-down per sensor
4. In `main` after init:
   ```c
   HAL_TIM_IC_Start_IT(&htim5, TIM_CHANNEL_1);
   ```
5. Callback (in `main.c` user section):
   ```c
   void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
   {
     if (htim->Instance == TIM5)
       ECU_CrankCapture(HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1));
   }
   ```
6. `stm32f4xx_it.c`: `TIM5_IRQHandler` → `HAL_TIM_IRQHandler(&htim5)`

## Hardware
| Item | Requirement |
|------|-------------|
| Level | **0–3.3 V** logic into PA0 (not raw VR AC) |
| Sensor | Hall/optical with open-collector needs **pull-up** |
| VR sensor | Needs VR conditioner IC → digital edge |
| Ground | Sensor ground shared with MCU GND |

## Config in software
- Default wheel **36-1** (`gTeeth=36`, `gMissing=1`)
- Send `CFG:36,1,30` or your wheel: `CFG:teeth,missing,trigAngle`
- 60-2: `CFG:60,2,0`

## Telemetry debug
| Field | Meaning |
|-------|---------|
| TOOTH | tooth index if SYNC=1; else edge count |
| TERR | rejected edges (noise / filter) |
| SYNC | 1 after missing-tooth gap found |
| RPM | from gap-to-gap, or provisional from tooth period |

If **TOOTH stays 0** while spinning: no IRQ (Cube pin/NVIC/callback).
If **TOOTH climbs, RPM=0**: edges seen, gap not detected — wrong teeth/missing or threshold.
If **TERR climbs fast**: noise or wrong polarity — try falling edge in CubeMX.

## Polarity
Cube: TIM5 CH1 polarity **Rising** (default) or **Falling** if your conditioner inverts.

## Quick scope check
PA0 should toggle clean 0/3.3 V with each tooth while cranking.
