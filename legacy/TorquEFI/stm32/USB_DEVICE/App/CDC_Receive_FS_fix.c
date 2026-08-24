/* Paste into USB_DEVICE/App/usbd_cdc_if.c
 *
 * 1. With the other includes at the top of usbd_cdc_if.c, add:
 *      #include "ecu_serial.h"
 *
 * 2. Replace CDC_Receive_FS with the function below.
 */

#include "ecu_serial.h"  /* <-- must be in usbd_cdc_if.c includes */

static int8_t CDC_Receive_FS(uint8_t* Buf, uint32_t *Len)
{
  if (Buf != NULL && Len != NULL && *Len > 0U) {
    ECU_CDC_Receive(Buf, Len);
  }
  USBD_CDC_SetRxBuffer(&hUsbDeviceFS, &Buf[0]);
  USBD_CDC_ReceivePacket(&hUsbDeviceFS);
  return (USBD_OK);
}
