/**
 * TorquEFI — correct CDC_Receive_FS buffer handling (CubeIDE)
 *
 * File to edit: USB_DEVICE/App/usbd_cdc_if.c
 *
 * ------------------------------------------------------------------
 * HOW ST CDC RX WORKS
 * ------------------------------------------------------------------
 * 1. Stack fills UserRxBufferFS[] (or Buf) with one USB OUT transfer.
 * 2. CDC_Receive_FS(Buf, Len) is called from USB interrupt context.
 * 3. You must copy/process Len bytes immediately (Buf may be reused).
 * 4. You MUST call USBD_CDC_ReceivePacket() again or RX stops forever.
 *
 * ------------------------------------------------------------------
 * REQUIRED EDITS IN usbd_cdc_if.c
 * ------------------------------------------------------------------
 */

#include "ecu_serial.h"

/* At top of usbd_cdc_if.c — after other includes.
 * DELETE any:  static int8_t ECU_CDC_Receive(...);
 */

/**
 * @brief  Data received over USB OUT endpoint.
 * @param  Buf: Receive buffer (usually UserRxBufferFS)
 * @param  Len: Number of bytes received this packet
 * @retval USBD_OK
 *
 * NOTE: Keep this SHORT — runs in IRQ / USB callback context.
 */
static int8_t CDC_Receive_FS(uint8_t* Buf, uint32_t *Len)
{
  if (Buf != NULL && Len != NULL && *Len > 0U) {
    /* Push into TorquEFI line parser (ecu_serial.c → ECU_UART_RxByte) */
    ECU_CDC_Receive(Buf, Len);
  }

  /* Re-arm RX: tell stack where to put the NEXT packet */
  USBD_CDC_SetRxBuffer(&hUsbDeviceFS, &Buf[0]);
  USBD_CDC_ReceivePacket(&hUsbDeviceFS);

  return (USBD_OK);
}

/*
 * Optional: in CDC_Init_FS ensure RX buffer is registered once:
 *
 * static int8_t CDC_Init_FS(void)
 * {
 *   USBD_CDC_SetTxBuffer(&hUsbDeviceFS, UserTxBufferFS, 0);
 *   USBD_CDC_SetRxBuffer(&hUsbDeviceFS, UserRxBufferFS);
 *   return (USBD_OK);
 * }
 *
 * APP_RX_DATA_SIZE / APP_TX_DATA_SIZE in usbd_cdc_if.h:
 *   #define APP_RX_DATA_SIZE  2048
 *   #define APP_TX_DATA_SIZE  2048
 */
