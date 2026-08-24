# TorquEFI STM32F411 — Import + USB CDC tuning

## Binary type
**Executable**

## USB CDC (Black Pill USB-C)

1. CubeMX → **Connectivity → USB_OTG_FS** → Device Only (PA11/PA12)
2. **Middleware → USB_DEVICE** → Class: **Communication Device Class (CDC)**
3. **Do not** enable CAN on PA11/PA12
4. Generate code

### Wire RX into the ECU parser

Edit `USB_DEVICE/App/usbd_cdc_if.c` → `CDC_Receive_FS`:

```c
#include "ecu_serial.h"

static int8_t CDC_Receive_FS(uint8_t* Buf, uint32_t *Len)
{
  ECU_CDC_Receive(Buf, Len);
  USBD_CDC_SetRxBuffer(&hUsbDeviceFS, &Buf[0]);
  USBD_CDC_ReceivePacket(&hUsbDeviceFS);
  return (USBD_OK);
}
```

TX uses `CDC_Transmit_FS` via `ECU_Serial_Write()` in `ecu_serial.c`.

### main.c sequence
```c
MX_USB_DEVICE_Init();
ECU_Init();
ECU_Serial_Init();
HAL_TIM_IC_Start_IT(&htim5, TIM_CHANNEL_1);
HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1);
while (1) { ECU_Loop(); }
```

### Tuner
1. Flash firmware (SWD)
2. Plug **USB-C** into PC
3. Select the new **Virtual COM port** in `ecu_tuner_aligned.py`
4. Baud rate can stay 115200 (ignored by CDC; harmless)

## Merge these files into a Cube-generated F411 project
```
Core/Inc/ecu_*.h  ecu_serial.h  main.h
Core/Src/ecu_app.c  ecu_serial.c  main.c  stm32f4xx_it.c  stm32f4xx_hal_msp.c
```
Add `usbd_cdc_if.h` include path from USB middleware.

Weak `CDC_Transmit_FS` in `usbd_cdc_if_hook.c` allows linking before USB is generated; remove that file once real `usbd_cdc_if.c` exists.
