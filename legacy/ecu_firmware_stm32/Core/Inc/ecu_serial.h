/**
 * TorquEFI serial transport — USB CDC (Black Pill) primary
 * Optional USART6 fallback if ECU_USE_USART6_FALLBACK is defined.
 *
 * CubeMX: Middleware → USB_DEVICE → Communication Device Class (CDC)
 * Pins: PA11 USB_DM, PA12 USB_DP (not CAN)
 */
#ifndef ECU_SERIAL_H
#define ECU_SERIAL_H

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/** Init transport (CDC already started in MX_USB_DEVICE_Init) */
void ECU_Serial_Init(void);

/** Write null-terminated string to host (CDC) */
void ECU_Serial_Write(const char *s);

/** Write raw bytes */
void ECU_Serial_WriteBytes(const uint8_t *data, uint16_t len);

/**
 * Feed one RX byte into protocol parser (line-based).
 * Call from CDC_Receive_FS for each byte, or from UART IRQ.
 */
void ECU_UART_RxByte(uint8_t b);

/**
 * CDC_Receive_FS helper: push buffer into parser.
 * Returns USBD_OK (0) for Cube USB stack.
 */
int8_t ECU_CDC_Receive(uint8_t *buf, uint32_t *len);

#ifdef __cplusplus
}
#endif

#endif
