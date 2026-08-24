# Debug CDC_Receive_FS buffer handling

## Correct pattern

```c
static int8_t CDC_Receive_FS(uint8_t* Buf, uint32_t *Len)
{
  if (Buf && Len && *Len)
    ECU_CDC_Receive(Buf, Len);   /* copy/parse NOW */

  USBD_CDC_SetRxBuffer(&hUsbDeviceFS, &Buf[0]);
  USBD_CDC_ReceivePacket(&hUsbDeviceFS);  /* REQUIRED or RX dies */
  return (USBD_OK);
}
```

## Common bugs

| Bug | Symptom |
|-----|---------|
| No `USBD_CDC_ReceivePacket` | First packet only, then TX timeout forever |
| `static ECU_CDC_Receive` in usbd_cdc_if.c | Build error / wrong function |
| Long work in callback | USB stalls / host TX timeout |
| Using Buf after return | Corrupted data |
| APP_RX_DATA_SIZE too small | Truncation (use 2048) |
| Weak `usbd_cdc_if_hook.c` still linked | Wrong TX/RX symbols |

## Debugger watches

After PuTTY sends `GETCFG\r`:

| Variable | Expect |
|----------|--------|
| `g_cdc_rx_packets` | increments |
| `g_cdc_last_len` | ~6–8 |
| `g_cdc_rx_bytes` | climbs |

If packets stay 0 → callback never runs (USB not configured / wrong usbd_cdc_if).  
If packets climb but no CFG reply → `handleLine` / `GETCFG` path in `ecu_app.c`.

## Init

`CDC_Init_FS` must set RX buffer once:

```c
USBD_CDC_SetRxBuffer(&hUsbDeviceFS, UserRxBufferFS);
```

## Host test

PuTTY: `GETCFG` + Enter → expect `CFG:...`  
If host write times out → OUT endpoint not re-armed.
