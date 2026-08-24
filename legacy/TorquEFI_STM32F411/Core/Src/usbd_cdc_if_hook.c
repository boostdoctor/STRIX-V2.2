/**
 * Instructions for Cube-generated USB/App/usbd_cdc_if.c
 *
 * In CDC_Receive_FS(), replace the default body with:
 *
 *   #include "ecu_serial.h"
 *   ...
 *   static int8_t CDC_Receive_FS(uint8_t* Buf, uint32_t *Len)
 *   {
 *     ECU_CDC_Receive(Buf, Len);
 *     USBD_CDC_SetRxBuffer(&hUsbDeviceFS, &Buf[0]);
 *     USBD_CDC_ReceivePacket(&hUsbDeviceFS);
 *     return (USBD_OK);
 *   }
 *
 * Keep CDC_Transmit_FS() as generated.
 *
 * This file is documentation-only when USB middleware is present.
 */
#include <stdint.h>

/* Weak stubs so the project links before USB middleware is added */
__attribute__((weak)) uint8_t CDC_Transmit_FS(uint8_t *Buf, uint16_t Len)
{
  (void)Buf; (void)Len;
  return 0; /* pretend OK */
}

#ifndef USBD_BUSY
#define USBD_BUSY 1
#endif
