# No data on COM port (empty terminal)

If PuTTY/Tera Term on the STM32 COM port shows **nothing**, the problem is
**firmware / USB**, not the Python tuner.

## A. Does the COM port exist?

| Result | Meaning |
|--------|---------|
| No new COM when USB-C plugged | USB device not enumerating (clock, init, cable, bootloader) |
| COM appears, no text | Enum OK; TX path or `ECU_Loop` broken |
| Text only after opening port | Normal — CDC TX starts when host opens port |

**Black Pill:** use the **board USB-C** connector (PA11/PA12), not ST-Link USB.

## B. Clock (most common fail)

CubeMX **Clock Configuration**:
- USB clock = **48.000 MHz** (green)
- Typical 25 MHz HSE: PLLN=192, PLLP=2 → SYSCLK 96 MHz, PLLQ=4 → USB 48 MHz

If USB is not 48 MHz, Windows may show a COM port that never works correctly.

## C. `main` must do this order

```c
HAL_Init();
SystemClock_Config();
MX_GPIO_Init();
/* … other MX_ init … */
MX_USB_DEVICE_Init();   /* required */
ECU_Init();

while (1) {
  ECU_Loop();           /* required — sends RPM lines */
}
```

If you never call `ECU_Loop()`, there is **no** telemetry.

## D. Remove the weak stub

If `usbd_cdc_if_hook.c` is in the build, it can replace real USB TX.
**Exclude it from the project** (only use Cube’s `USB_DEVICE/App/usbd_cdc_if.c`).

## E. `CDC_Receive_FS` (RX) — does not create TX

RX wiring does not make the board send. TX is `CDC_Transmit_FS` inside
`ecu_serial.c` ← `sendTelemetry()` ← `ECU_Loop()`.

## F. 5-minute HELLO test

1. Add call in `while(1)` every 500 ms (see `cdc_hello_test.c`):

```c
extern void CDC_Hello_Tick(void);
// in loop:
CDC_Hello_Tick();
```

2. Build, flash, open COM in PuTTY.
3. Expect: `HELLO,1` `HELLO,2` …

| HELLO works | Then |
|-------------|------|
| Yes | USB OK — fix/ensure `ECU_Loop` + `sendTelemetry` |
| No | USB stack/clock/init — not ECU maps |

## G. ST-Link vs board USB

| Connector | Function |
|-----------|----------|
| SWD / ST-Link USB | Debug/flash only — **no** tuner CDC |
| Black Pill USB-C | CDC COM port for telemetry |

## H. Optional UART fallback

Wire USB-TTL to **PC6/PC7** (USART6), define `ECU_USE_USART6_FALLBACK`,
init USART6 at 115200 — telemetry also goes to UART while debugging CDC.
