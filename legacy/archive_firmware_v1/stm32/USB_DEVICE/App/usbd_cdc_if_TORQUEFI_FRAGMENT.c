/**
 * TorquEFI — paste into USB_DEVICE/App/usbd_cdc_if.c
 *
 * 1) Add near top of usbd_cdc_if.c (after Cube includes):
 *      #include "ecu_serial.h"
 *
 * 2) Replace the bodies of:
 *      CDC_Init_FS
 *      CDC_Control_FS
 *      CDC_Receive_FS
 *      CDC_TransmitCplt_FS
 *
 * 3) Optional in usbd_cdc_if.h:
 *      #define APP_RX_DATA_SIZE  2048
 *      #define APP_TX_DATA_SIZE  2048
 *
 * Do NOT compile this file by itself — it is a reference only.
 */

#include "ecu_serial.h"

/* -------------------------------------------------------------------------- */
/* CDC_Init_FS                                                                */
/* -------------------------------------------------------------------------- */
static int8_t CDC_Init_FS(void)
{
  /* Set Application Buffers */
  USBD_CDC_SetTxBuffer(&hUsbDeviceFS, UserTxBufferFS, 0);
  USBD_CDC_SetRxBuffer(&hUsbDeviceFS, UserRxBufferFS);
  return (USBD_OK);
}

/* -------------------------------------------------------------------------- */
/* CDC_Control_FS — ACM line coding + DTR/RTS                                 */
/* -------------------------------------------------------------------------- */
static int8_t CDC_Control_FS(uint8_t cmd, uint8_t* pbuf, uint16_t length)
{
  /* Forward all CDC class requests to TorquEFI serial layer */
  ECU_CDC_Control(cmd, pbuf, length);
  return (USBD_OK);
}

/* -------------------------------------------------------------------------- */
/* CDC_Receive_FS — USB OUT (host → device)                                   */
/*                                                                            */
/* Runs in USB / ISR context: keep short.                                     */
/* Must re-arm ReceivePacket or RX stops.                                     */
/* If RX ring is full, skip re-arm → host bulk OUT gets NAK (flow control).  */
/* -------------------------------------------------------------------------- */
static int8_t CDC_Receive_FS(uint8_t* Buf, uint32_t *Len)
{
  if (Buf != NULL && Len != NULL && *Len > 0U) {
    (void)ECU_CDC_Receive(Buf, Len);
  }

  if (ECU_CDC_RxCanAccept()) {
    USBD_CDC_SetRxBuffer(&hUsbDeviceFS, &Buf[0]);
    USBD_CDC_ReceivePacket(&hUsbDeviceFS);
  }
  /* else: ecu_serial will re-arm from ECU_Serial_Service when ring drains */

  return (USBD_OK);
}

/* -------------------------------------------------------------------------- */
/* CDC_TransmitCplt_FS — USB IN complete (device → host)                      */
/*                                                                            */
/* Clears TX busy and kicks next chunk from the TX ring.                      */
/* -------------------------------------------------------------------------- */
static int8_t CDC_TransmitCplt_FS(uint8_t *Buf, uint32_t *Len, uint8_t epnum)
{
  (void)Buf;
  (void)Len;
  (void)epnum;

  ECU_CDC_TxComplete();
  return (USBD_OK);
}
