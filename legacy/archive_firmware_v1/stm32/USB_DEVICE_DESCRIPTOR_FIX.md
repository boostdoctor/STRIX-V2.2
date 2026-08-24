# Fix: Windows “Device descriptor request failed”

## Meaning
The PC powered the device but could not read the USB device descriptor.
Almost never a COM-port software issue — the MCU is not enumerating.

## Fix checklist (Black Pill STM32F411)

1. **Clock — USB must be 48.000 MHz**
   - Prefer **HSE 25 MHz** (crystal on board):
     - PLLM=25, PLLN=192, PLLP=/2 → SYSCLK 96 MHz
     - PLLQ=4 → **USB 48 MHz**
   - CubeMX: Clock Configuration → USB frequency must show **48 MHz**
   - If you regenerate code, do **not** overwrite with a clock that leaves USB at 0 or ≠48

2. **Use updated `main.c`**
   - Tries HSE first, falls back to HSI
   - `MX_USB_DEVICE_Init()` after GPIO/timers, short delay before heavy ECU init

3. **CubeMX USB**
   - Connectivity → USB_OTG_FS → Device_Only
   - Middleware → USB_DEVICE → CDC
   - Pins PA11 / PA12
   - NVIC: OTG_FS interrupt enabled

4. **Hardware**
   - Data USB-C cable (not charge-only)
   - Try another port / hub
   - Device Manager: uninstall failed device → unplug → plug

5. **Bootloader stuck**
   - If only “STM32 BOOTLOADER” appears: flash app, BOOT0=0, reset

6. **Conflict**
   - Only **one** `SystemClock_Config` in the project
   - If CubeMX generates it in `main.c`, put the HSE/PLL settings in USER CODE or match .ioc

## Quick test
Flash ST’s USB CDC standalone example with the same clock.
- If that also fails → crystal / cable / board USB
- If ST example works → app clock or early HardFault
