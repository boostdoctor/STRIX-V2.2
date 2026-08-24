/**
 * USB CDC transport for tuner (Black Pill USB-C)
 * Requires: MX_USB_DEVICE_Init(), real usbd_cdc_if.c (not weak stub)
 */
#include "ecu_serial.h"
#include "ecu_app.h"
#include <string.h>
#include "usbd_cdc_if.h"
#include "usbd_def.h"

extern USBD_HandleTypeDef hUsbDeviceFS;

#ifdef ECU_USE_USART6_FALLBACK
#include "main.h"
extern UART_HandleTypeDef huart6;
#endif

#ifndef USBD_OK
#define USBD_OK 0U
#endif
#ifndef USBD_BUSY
#define USBD_BUSY 1U
#endif
#ifndef USBD_FAIL
#define USBD_FAIL 2U
#endif
#ifndef USBD_STATE_CONFIGURED
#define USBD_STATE_CONFIGURED 3U
#endif

/* Debug counters — optional watch in debugger */
volatile uint32_t g_cdc_rx_packets = 0;
volatile uint32_t g_cdc_rx_bytes   = 0;
volatile uint32_t g_cdc_last_len   = 0;

void ECU_Serial_Init(void)
{
  /* USB stack started in MX_USB_DEVICE_Init() */
}

static int usb_configured(void)
{
  return (hUsbDeviceFS.dev_state == USBD_STATE_CONFIGURED) ? 1 : 0;
}

void ECU_Serial_WriteBytes(const uint8_t *data, uint16_t len)
{
  if (!data || !len) return;

  if (!usb_configured()) {
#ifdef ECU_USE_USART6_FALLBACK
    HAL_UART_Transmit(&huart6, (uint8_t *)data, len, 30);
#endif
    return;
  }

  uint16_t off = 0;
  while (off < len) {
    uint16_t n = (uint16_t)(len - off);
    if (n > 64) n = 64;

    uint32_t tries = 0;
    uint8_t st = USBD_BUSY;
    while (tries < 200) {
      st = CDC_Transmit_FS((uint8_t *)(data + off), n);
      if (st == USBD_OK)
        break;
      if (st == USBD_FAIL)
        break;
      tries++;
      for (volatile uint32_t d = 0; d < 200; d++) { }
    }
    if (st != USBD_OK)
      break;
    off = (uint16_t)(off + n);
  }

#ifdef ECU_USE_USART6_FALLBACK
  HAL_UART_Transmit(&huart6, (uint8_t *)data, len, 30);
#endif
}

void ECU_Serial_Write(const char *s)
{
  if (!s) return;
  size_t n = strlen(s);
  if (n > 0xFFFF) n = 0xFFFF;
  ECU_Serial_WriteBytes((const uint8_t *)s, (uint16_t)n);
}

/**
 * Called from CDC_Receive_FS with one USB OUT packet.
 * Buf is only valid during this call — process immediately.
 */
int8_t ECU_CDC_Receive(uint8_t *buf, uint32_t *len)
{
  if (!buf || !len) return 0;

  uint32_t n = *len;
  /* Guard against corrupt length */
  if (n > 2048U)
    n = 2048U;

  g_cdc_rx_packets++;
  g_cdc_last_len = n;
  g_cdc_rx_bytes += n;

  for (uint32_t i = 0; i < n; i++)
    ECU_UART_RxByte(buf[i]);

  return 0; /* USBD_OK */
}
