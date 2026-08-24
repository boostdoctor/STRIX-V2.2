/**
 * TorquEFI NVM — one full flash sector for tune storage
 *
 *   F411CEUx 512 KB → SECTOR_7 @ 0x08060000 (128 KB)
 *   F411CCUx 256 KB → SECTOR_5 @ 0x08020000 (128 KB)
 *
 * Blob CRC: software reflected CRC-32 (zlib/ISO-HDLC) — stable across resets.
 * Hardware CRC unit available via ECU_Flash_CrcHw() for diagnostics.
 */
#ifndef ECU_FLASH_H
#define ECU_FLASH_H

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

#define ECU_FLASH_MAGIC     0xECAF4110u
/* v6 = software CRC-32 again (v5 HW CRC rejected on many boards after reset) */
#define ECU_FLASH_VERSION   6u

#define ECU_NVM_BASE_512K   0x08060000u
#define ECU_NVM_BASE_256K   0x08020000u
#define ECU_NVM_SECTOR_SIZE 0x00020000u

#define ECU_FLASH_MAP_ROWS  15
#define ECU_FLASH_MAP_COLS  22

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
  int16_t  ltftCenti;
  int8_t   advMap[ECU_FLASH_MAP_ROWS][ECU_FLASH_MAP_COLS];
  uint8_t  injMap[ECU_FLASH_MAP_ROWS][ECU_FLASH_MAP_COLS];
  uint32_t crc32;
} EcuFlashBlob;

void     ECU_Flash_CrcInit(void);
uint32_t ECU_Flash_CrcCalc(const EcuFlashBlob *blob);
uint32_t ECU_Flash_CrcBuffer(const void *data, size_t len);
uint32_t ECU_Flash_CrcHw(const void *data, size_t len); /* optional HW */

int      ECU_Flash_Present(void);
int      ECU_Flash_Load(EcuFlashBlob *out);
int      ECU_Flash_Save(const EcuFlashBlob *in);
uint32_t ECU_Flash_StoredCrc(void);
uint32_t ECU_Flash_SectorAddr(void);
uint32_t ECU_Flash_SectorIndex(void);
uint32_t ECU_Flash_SectorSize(void);

#ifdef __cplusplus
}
#endif

#endif
