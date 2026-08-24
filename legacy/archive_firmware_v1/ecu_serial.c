/**
 * USB CDC transport with flow control and error tracking
 *
 * TX: app → ring → CDC_Transmit_FS (64 B); drop + count on full
 * RX: OUT → ring → ECU_UART_RxByte; pause ReceivePacket at HI water (OUT NAK)
 * Errors: sticky flags + counters; query via ECU_Serial_GetErrors / GETUART
 */
#include "ecu_serial.h"
#include "ecu_app.h"
#include <string.h>
#include "usbd_cdc_if.h"
#include "usbd_def.h"
#include "usbd_cdc.h"
#include "stm32f4xx_hal.h"

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

#ifndef ECU_CDC_REQUIRE_DTR
#define ECU_CDC_REQUIRE_DTR 0
#endif
#ifndef ECU_CDC_HONOR_RTS
#define ECU_CDC_HONOR_RTS 0
#endif

#define CDC_TX_RING   8192U
#define CDC_RX_RING   4096U
#define CDC_PKT         64U
#define CDC_RX_HI     (CDC_RX_RING * 3U / 4U)  /* ~3072 */
#define CDC_RX_LO     (CDC_RX_RING / 4U)       /* ~1024 */
#define CDC_TX_STUCK_MS  500U

static uint8_t  tx_ring[CDC_TX_RING];
static volatile uint16_t tx_head, tx_tail;
static volatile uint8_t  tx_busy;
static uint8_t  tx_pkt[CDC_PKT];
static uint32_t tx_busy_since_ms;

static uint8_t  rx_ring[CDC_RX_RING];
static volatile uint16_t rx_head, rx_tail;
static volatile uint8_t  rx_paused;

static volatile uint8_t cdc_dtr = 1;
static volatile uint8_t cdc_rts = 1;

/* Sticky error flags (ECU_SER_ERR_*) */
static volatile uint32_t ser_errors = 0;

volatile uint32_t g_cdc_rx_packets = 0;
volatile uint32_t g_cdc_rx_bytes   = 0;
volatile uint32_t g_cdc_tx_drop    = 0;
volatile uint32_t g_cdc_rx_drop    = 0;
volatile uint32_t g_cdc_tx_busy    = 0;
volatile uint32_t g_cdc_tx_fail    = 0;
volatile uint32_t g_cdc_last_len   = 0;
volatile uint32_t g_cdc_line_ovf   = 0;

static uint16_t tx_count(void)
{
  uint16_t h = tx_head, t = tx_tail;
  return (h >= t) ? (uint16_t)(h - t) : (uint16_t)(CDC_TX_RING - t + h);
}

static uint16_t rx_count(void)
{
  uint16_t h = rx_head, t = rx_tail;
  return (h >= t) ? (uint16_t)(h - t) : (uint16_t)(CDC_RX_RING - t + h);
}

static void ser_set_err(uint32_t bit)
{
  ser_errors |= bit;
}

static uint8_t usb_configured(void)
{
  return (hUsbDeviceFS.dev_state == USBD_STATE_CONFIGURED) ? 1u : 0u;
}

/* Returns bytes accepted (may be < len on overflow) */
static uint16_t tx_push(const uint8_t *data, uint16_t len)
{
  uint16_t accepted = 0;
  for (uint16_t i = 0; i < len; i++) {
    uint16_t next = (uint16_t)((tx_head + 1U) % CDC_TX_RING);
    if (next == tx_tail) {
      g_cdc_tx_drop++;
      ser_set_err(ECU_SER_ERR_TX_OVERFLOW);
      break;
    }
    tx_ring[tx_head] = data[i];
    tx_head = next;
    accepted++;
  }
  return accepted;
}

static void rx_push_byte(uint8_t b)
{
  uint16_t next = (uint16_t)((rx_head + 1U) % CDC_RX_RING);
  if (next == rx_tail) {
    g_cdc_rx_drop++;
    ser_set_err(ECU_SER_ERR_RX_OVERFLOW);
    return;
  }
  rx_ring[rx_head] = b;
  rx_head = next;
}

static int rx_pop_byte(uint8_t *out)
{
  if (rx_head == rx_tail)
    return 0;
  *out = rx_ring[rx_tail];
  rx_tail = (uint16_t)((rx_tail + 1U) % CDC_RX_RING);
  return 1;
}

static void tx_kick(void)
{
  if (tx_busy)
    return;
  if (!usb_configured()) {
    ser_set_err(ECU_SER_ERR_NOT_CFG);
    return;
  }
#if ECU_CDC_REQUIRE_DTR
  if (!cdc_dtr)
    return;
#endif
#if ECU_CDC_HONOR_RTS
  if (!cdc_rts)
    return;
#endif

  uint16_t n = tx_count();
  if (n == 0)
    return;
  if (n > CDC_PKT)
    n = CDC_PKT;

  for (uint16_t i = 0; i < n; i++) {
    tx_pkt[i] = tx_ring[tx_tail];
    tx_tail = (uint16_t)((tx_tail + 1U) % CDC_TX_RING);
  }

  tx_busy = 1;
  tx_busy_since_ms = HAL_GetTick();
  uint8_t st = CDC_Transmit_FS(tx_pkt, n);
  if (st == USBD_OK) {
    /* completion via ECU_CDC_TxComplete */
  } else if (st == USBD_BUSY) {
    g_cdc_tx_busy++;
    /* restore bytes to ring — simple path: drop kick, wait complete */
    /* Bytes already removed; if BUSY without complete, stuck recovery handles it */
    ser_set_err(ECU_SER_ERR_TX_BUSY);
  } else {
    g_cdc_tx_fail++;
    ser_set_err(ECU_SER_ERR_TX_FAIL);
    tx_busy = 0;
  }
}

/* Re-arm OUT endpoint when RX has room */
static void rx_try_rearm(void)
{
  if (!rx_paused)
    return;
  if (rx_count() > CDC_RX_LO)
    return;
  if (!usb_configured())
    return;
  rx_paused = 0;
  (void)USBD_CDC_ReceivePacket(&hUsbDeviceFS);
}

uint8_t ECU_Serial_HostReady(void)
{
  if (!usb_configured())
    return 0;
#if ECU_CDC_REQUIRE_DTR
  if (!cdc_dtr)
    return 0;
#endif
  return 1;
}

uint8_t ECU_CDC_RxCanAccept(void)
{
  return (rx_count() < CDC_RX_HI) ? 1u : 0u;
}

void ECU_CDC_SetControlLineState(uint8_t *pbuf)
{
  if (!pbuf)
    return;
  uint16_t v = (uint16_t)pbuf[0] | ((uint16_t)pbuf[1] << 8);
  cdc_dtr = (v & 0x01u) ? 1u : 0u;
  cdc_rts = (v & 0x02u) ? 1u : 0u;
}

uint8_t ECU_CDC_GetDTR(void) { return cdc_dtr; }
uint8_t ECU_CDC_GetRTS(void) { return cdc_rts; }

void ECU_CDC_Control(uint8_t cmd, uint8_t *pbuf, uint16_t length)
{
  (void)length;
  switch (cmd) {
  case CDC_SET_CONTROL_LINE_STATE:
    ECU_CDC_SetControlLineState(pbuf);
    break;
  case CDC_SET_LINE_CODING:
    /* baud ignored on USB */
    break;
  default:
    break;
  }
}

void ECU_Serial_Init(void)
{
  tx_head = tx_tail = 0;
  rx_head = rx_tail = 0;
  tx_busy = 0;
  rx_paused = 0;
  ser_errors = 0;
  cdc_dtr = 1;
  cdc_rts = 1;
  g_cdc_tx_drop = g_cdc_rx_drop = 0;
  g_cdc_tx_fail = g_cdc_line_ovf = 0;
}

uint16_t ECU_Serial_TxPending(void) { return tx_count(); }
uint16_t ECU_Serial_RxPending(void) { return rx_count(); }

uint32_t ECU_Serial_GetErrors(void)
{
  return ser_errors;
}

void ECU_Serial_ClearErrors(void)
{
  ser_errors = 0;
}

void ECU_CDC_TxComplete(void)
{
  tx_busy = 0;
  tx_kick();
}

void ECU_Serial_Service(void)
{
  /* Recover stuck TX (complete callback lost) */
  if (tx_busy) {
    uint32_t now = HAL_GetTick();
    if ((now - tx_busy_since_ms) > CDC_TX_STUCK_MS) {
      tx_busy = 0;
      ser_set_err(ECU_SER_ERR_TX_STUCK);
      g_cdc_tx_fail++;
    }
  }

  if (usb_configured())
    tx_kick();
  else if (tx_count() > 0)
    ser_set_err(ECU_SER_ERR_NOT_CFG);

  /* Drain RX ring into line parser */
  uint8_t b;
  uint16_t budget = 512;
  while (budget-- && rx_pop_byte(&b))
    ECU_UART_RxByte(b);

  rx_try_rearm();
}

int ECU_Serial_WriteBytes(const uint8_t *data, uint16_t len)
{
  if (!data || !len)
    return 0;

#ifdef ECU_USE_USART6_FALLBACK
  if (!usb_configured()) {
    HAL_StatusTypeDef st = HAL_UART_Transmit(&huart6, (uint8_t *)data, len, 30);
    if (st != HAL_OK) {
      ser_set_err(ECU_SER_ERR_TX_FAIL);
      return -1;
    }
    return (int)len;
  }
#endif

  if (!usb_configured()) {
    ser_set_err(ECU_SER_ERR_NOT_CFG);
    return -1;
  }

  uint16_t n = tx_push(data, len);
  tx_kick();
  if (n < len)
    return -2; /* partial / overflow */
  return (int)n;
}

int ECU_Serial_Write(const char *s)
{
  if (!s)
    return 0;
  size_t n = strlen(s);
  if (n > 0xFFFF)
    n = 0xFFFF;
  return ECU_Serial_WriteBytes((const uint8_t *)s, (uint16_t)n);
}

int8_t ECU_CDC_Receive(uint8_t *buf, uint32_t *len)
{
  if (!buf || !len) {
    ser_set_err(ECU_SER_ERR_RX_BADARG);
    return -1;
  }

  uint32_t n = *len;
  if (n > 2048U)
    n = 2048U;

  g_cdc_rx_packets++;
  g_cdc_last_len = n;
  g_cdc_rx_bytes += n;

  for (uint32_t i = 0; i < n; i++)
    rx_push_byte(buf[i]);

  if (!ECU_CDC_RxCanAccept())
    rx_paused = 1;

  return 0;
}

/* Optional weak line-overflow hook from ecu_app */
void ECU_Serial_NoteLineOverflow(void)
{
  g_cdc_line_ovf++;
  ser_set_err(ECU_SER_ERR_LINE_OVF);
}
