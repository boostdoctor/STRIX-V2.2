/**
 * USB CDC ACM flow control — patch Cube USB_DEVICE/App/usbd_cdc_if.c
 *
 * 1) CDC_Control_FS  (ACM DTR/RTS + line coding)
 * ------------------------------------------------
 *   #include "ecu_serial.h"
 *
 *   static int8_t CDC_Control_FS(uint8_t cmd, uint8_t* pbuf, uint16_t length)
 *   {
 *     ECU_CDC_Control(cmd, pbuf, length);
 *     return (USBD_OK);
 *   }
 *
 * 2) CDC_Receive_FS  (RX backpressure → USB OUT NAK)
 * ------------------------------------------------
 *   static int8_t CDC_Receive_FS(uint8_t* Buf, uint32_t *Len)
 *   {
 *     ECU_CDC_Receive(Buf, Len);
 *     if (ECU_CDC_RxCanAccept()) {
 *       USBD_CDC_SetRxBuffer(&hUsbDeviceFS, &Buf[0]);
 *       USBD_CDC_ReceivePacket(&hUsbDeviceFS);
 *     }
 *     return (USBD_OK);
 *   }
 *
 * 3) CDC_TransmitCplt_FS  (TX ring chain)
 * ------------------------------------------------
 *   static int8_t CDC_TransmitCplt_FS(uint8_t *Buf, uint32_t *Len, uint8_t epnum)
 *   {
 *     (void)Buf; (void)Len; (void)epnum;
 *     ECU_CDC_TxComplete();
 *     return (USBD_OK);
 *   }
 *
 * ACM mapping used by TorquEFI:
 *   DTR=1  host has the virtual COM open
 *   RTS=1  host ready for TX (optional gate: ECU_CDC_HONOR_RTS)
 *   RX full → stop ReceivePacket → bulk OUT NAK (hardware USB flow)
 *   TX busy → USBD_BUSY / ring → bulk IN NAK until complete
 *
 * Exclude this file from the CubeIDE build.
 */
