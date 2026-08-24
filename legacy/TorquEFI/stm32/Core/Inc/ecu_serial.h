/**
 * TorquEFI USB CDC ACM transport + flow control + error flags
 */
#ifndef ECU_SERIAL_H
#define ECU_SERIAL_H

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Sticky error bits (ECU_Serial_GetErrors) */
#define ECU_SER_ERR_TX_OVERFLOW  (1u << 0)
#define ECU_SER_ERR_RX_OVERFLOW  (1u << 1)
#define ECU_SER_ERR_NOT_CFG      (1u << 2)
#define ECU_SER_ERR_TX_BUSY      (1u << 3)
#define ECU_SER_ERR_TX_FAIL      (1u << 4)
#define ECU_SER_ERR_TX_STUCK     (1u << 5)
#define ECU_SER_ERR_LINE_OVF     (1u << 6)
#define ECU_SER_ERR_RX_BADARG    (1u << 7)

void ECU_Serial_Init(void);
void ECU_Serial_Service(void);

/** Returns bytes written, 0 if empty, -1 not configured / fail, -2 TX overflow */
int  ECU_Serial_Write(const char *s);
int  ECU_Serial_WriteBytes(const uint8_t *data, uint16_t len);

uint16_t ECU_Serial_TxPending(void);
uint16_t ECU_Serial_RxPending(void);
uint8_t  ECU_Serial_HostReady(void);
uint8_t  ECU_CDC_RxCanAccept(void);

uint32_t ECU_Serial_GetErrors(void);
void     ECU_Serial_ClearErrors(void);
void     ECU_Serial_NoteLineOverflow(void);

int8_t ECU_CDC_Receive(uint8_t *buf, uint32_t *len);
void ECU_CDC_TxComplete(void);
void ECU_CDC_Control(uint8_t cmd, uint8_t *pbuf, uint16_t length);
void ECU_CDC_SetControlLineState(uint8_t *pbuf);
uint8_t ECU_CDC_GetDTR(void);
uint8_t ECU_CDC_GetRTS(void);

void ECU_UART_RxByte(uint8_t b);

/* Debug counters (optional) */
extern volatile uint32_t g_cdc_rx_packets;
extern volatile uint32_t g_cdc_rx_bytes;
extern volatile uint32_t g_cdc_tx_drop;
extern volatile uint32_t g_cdc_rx_drop;
extern volatile uint32_t g_cdc_tx_busy;
extern volatile uint32_t g_cdc_tx_fail;
extern volatile uint32_t g_cdc_line_ovf;

#ifdef __cplusplus
}
#endif

#endif
