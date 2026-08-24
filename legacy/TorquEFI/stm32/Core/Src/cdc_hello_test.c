/**
 * Minimal CDC TX test — optional temporary file
 *
 * Goal: prove USB CDC IN path works before full ECU telemetry.
 *
 * In main.c after MX_USB_DEVICE_Init():
 *   // call from loop every 500 ms:
 *   CDC_Hello_Tick();
 *
 * Or paste CDC_Hello_Tick body into main while(1).
 *
 * REMOVE this from the project once RPM telemetry works.
 */
#include "usbd_cdc_if.h"
#include "usbd_def.h"
#include <stdio.h>
#include <string.h>

extern USBD_HandleTypeDef hUsbDeviceFS;

void CDC_Hello_Tick(void)
{
  static uint32_t last;
  static uint32_t n;
  uint32_t now = HAL_GetTick();
  if ((now - last) < 500U)
    return;
  last = now;

  if (hUsbDeviceFS.dev_state != USBD_STATE_CONFIGURED)
    return; /* host has not opened COM yet */

  char msg[48];
  int len = snprintf(msg, sizeof msg, "HELLO,%lu\r\n", (unsigned long)++n);
  if (len > 0)
    (void)CDC_Transmit_FS((uint8_t *)msg, (uint16_t)len);
}
