# No telemetry / TX failed

## "TX failed: port not open" (tuner)
- Port closed after Connect (device reset, wrong COM).
- Pick the **STM32 Virtual COM Port** / `ttyACM0`, not ST-Link VCP.
- Close other apps using the port (serial monitor, Cube monitor).

## No RPM lines (firmware CDC)
1. Confirm Device Manager / `ls /dev/ttyACM*` shows a port when USB-C is plugged **after** firmware runs.
2. `main()` must call `MX_USB_DEVICE_Init()` then loop `ECU_Loop()`.
3. `CDC_Receive_FS` must call `ECU_CDC_Receive` (not a static local).
4. **Remove** `usbd_cdc_if_hook.c` from the build (weak stub does not send USB data).
5. Clock: USB must be **48 MHz**.
6. Telemetry only TX when host has opened the COM port (`USBD_STATE_CONFIGURED`).

## Quick test
Open the COM port in Tera Term / PuTTY (any baud). You should see lines:
`RPM:0,MAP:...,TPS:...`
If terminal works but tuner does not → tuner port selection.
If terminal is empty → firmware/USB/clock.
