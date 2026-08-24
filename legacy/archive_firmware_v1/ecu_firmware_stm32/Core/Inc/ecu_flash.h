/**
 * TorquEFI — non-volatile store in last flash sector (F411 512KB → sector 7)
 */
#ifndef ECU_FLASH_H
#define ECU_FLASH_H

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

#define ECU_FLASH_MAGIC   0xECAF4110u
#define ECU_FLASH_VERSION 4u   /* bumped: map 15x22 */

/* F411xE 512KB: sector 7 @ 0x08060000 (128KB). Change for 256KB parts. */
#ifndef ECU_FLASH_SECTOR
#define ECU_FLASH_SECTOR      FLASH_SECTOR_7
#define ECU_FLASH_SECTOR_ADDR 0x08060000u
#endif

#define ECU_FLASH_MAP_ROWS 15
#define ECU_FLASH_MAP_COLS 22

typedef struct __attribute__((packed)) {
  uint32_t magic;
  uint16_t version;
  uint16_t tpsClosed;
  uint16_t tpsOpen;
  uint16_t pedClosed;
  uint16_t pedOpen;
  uint16_t trigAngle;
  uint8_t  teeth;
  uint8_t  missing;
  int16_t  ltftCenti;  /* LTFT percent * 100, e.g. 12.5% → 1250 */
  int8_t   advMap[ECU_FLASH_MAP_ROWS][ECU_FLASH_MAP_COLS];
  uint8_t  injMap[ECU_FLASH_MAP_ROWS][ECU_FLASH_MAP_COLS];
  uint32_t crc32;  /* CRC of all bytes before this field */
} EcuFlashBlob;

/** Load blob from flash; returns 1 if magic+CRC OK */
int ECU_Flash_Load(EcuFlashBlob *out);

/** Erase sector and write blob (computes CRC). Returns 0 on success */
int ECU_Flash_Save(const EcuFlashBlob *in);

/** Quick valid check at fixed address */
int ECU_Flash_Present(void);

#ifdef __cplusplus
}
#endif

#endif
