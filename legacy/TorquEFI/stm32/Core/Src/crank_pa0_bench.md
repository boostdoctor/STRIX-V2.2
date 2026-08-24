# PA0 crank — why RPM stays 0

## 1. Hardware (do this first)

With a multimeter or scope on **PA0** while cranking / spinning the wheel:

| PA0 signal | Meaning |
|------------|---------|
| Stays 0 V or 3.3 V | No edges → sensor / wiring / power |
| Swings 0↔3.3 V | Good — MCU should see teeth |
| AC / ± volts | Raw VR — **will not work** on PA0; need conditioner |

Black Pill pin: **PA0** is the second pin from the bottom on many pinouts — confirm your board silk.

## 2. CubeMX TIM5 (must match)

1. Timers → **TIM5** → Channel1 → **Input Capture direct mode**
2. Pinout view: **PA0** = TIM5_CH1 (not ADC, not GPIO_Input only)
3. NVIC → **TIM5 global interrupt** = Enabled
4. Parameter: polarity Rising (or Falling if inverted sensor)
5. Generate code

## 3. Code that must exist in YOUR project

### main.c — after MX_TIM5_Init()
```c
HAL_TIM_IC_Start_IT(&htim5, TIM_CHANNEL_1);
```

### main.c — USER CODE
```c
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM5) {
    ECU_CrankCapture(HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1));
  }
}
```

### stm32f4xx_it.c
```c
void TIM5_IRQHandler(void)
{
  HAL_TIM_IRQHandler(&htim5);
}
```

### stm32f4xx_hal_msp.c (or Cube MSP)
PA0 AF2_TIM5, clock enable TIM5 + GPIOA, NVIC enable TIM5.

## 4. Read telemetry fields

| Field | If zero | If rising |
|-------|---------|-----------|
| TOOTH | No capture IRQ | Edges seen |
| TERR | — | Noise / rejected |
| SYNC | No missing-tooth gap yet | Gap found |
| RPM | No period yet | Calculated |

Spin the engine/wheel and watch **TOOTH**.  
If TOOTH never moves → **IRQ/path broken**, not the RPM formula.

## 5. 60-second bench without engine

Wire PA0 to a **3.3 V square wave** (function generator, or toggle from another MCU pin ~100–500 Hz).

Expected:
- TOOTH climbing
- RPM non-zero (provisional)

If still TOOTH=0 with a known good square wave → software/Cube config only.

## 6. Common mistakes

1. Callback never added to STRIX `main.c` after Cube regenerate
2. TIM5 IRQ not enabled in NVIC
3. PA0 left as GPIO_Input in gpio.c overwriting AF
4. Testing with ST-Link only — must run firmware that calls `HAL_TIM_IC_Start_IT`
5. Sensor not 3.3 V logic
