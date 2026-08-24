/**
 * TorquEFI — flash store on last sector (STM32F411 512KB → sector 7 @ 0x08060000)
 *
 * SAVE flow:
 *   1. Copy caller blob, stamp magic/version, CRC32 over payload
 *   2. Unlock flash, clear flags, erase sector 7
 *   3. Program word-by-word (32-bit)
 *   4. Lock, verify magic+CRC
 *
 * Errors: -1 null, -2 erase, -3 program, -4 verify, -5 size
 */
#include "ecu_flash.h"
#include "stm32f4xx_hal.h"
#include <string.h>

/* Ensure blob fits in one sector with margin (128KB sector) */
_Static_assert(sizeof(EcuFlashBlob) < 65536u, "EcuFlashBlob too large for sector");

static uint32_t crc32_calc(const uint8_t *data, size_t len)
{
  uint32_t crc = 0xFFFFFFFFu;
  for (size_t i = 0; i < len; i++) {
    crc ^= data[i];
    for (int b = 0; b < 8; b++)
      crc = (crc >> 1) ^ (0xEDB88320u & (uint32_t)-(int32_t)(crc & 1u));
  }
  return ~crc;
}

static void flash_clear_flags(void)
{
  __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR |
                         FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR);
}

int ECU_Flash_Present(void)
{
  const EcuFlashBlob *p = (const EcuFlashBlob *)(uintptr_t)ECU_FLASH_SECTOR_ADDR;
  if (p->magic != ECU_FLASH_MAGIC || p->version != ECU_FLASH_VERSION)
    return 0;
  uint32_t expect = crc32_calc((const uint8_t *)p, offsetof(EcuFlashBlob, crc32));
  return (expect == p->crc32) ? 1 : 0;
}

int ECU_Flash_Load(EcuFlashBlob *out)
{
  if (!out)
    return 0;
  if (!ECU_Flash_Present())
    return 0;
  const EcuFlashBlob *p = (const EcuFlashBlob *)(uintptr_t)ECU_FLASH_SECTOR_ADDR;
  memcpy(out, p, sizeof(*out));
  return 1;
}

int ECU_Flash_Save(const EcuFlashBlob *in)
{
  if (!in)
    return -1;
  if (sizeof(EcuFlashBlob) > 120000u)
    return -5;

  /* Local aligned copy with header + CRC */
  EcuFlashBlob blob;
  memcpy(&blob, in, sizeof(blob));
  blob.magic   = ECU_FLASH_MAGIC;
  blob.version = ECU_FLASH_VERSION;
  blob.crc32   = crc32_calc((const uint8_t *)&blob, offsetof(EcuFlashBlob, crc32));

  /* Critical section: erase + program must not be interrupted by USB/ISRs
   * that could re-enter flash ops. */
  uint32_t primask = __get_PRIMASK();
  __disable_irq();

  HAL_FLASH_Unlock();
  flash_clear_flags();

  FLASH_EraseInitTypeDef er;
  memset(&er, 0, sizeof(er));
  uint32_t sectorError = 0xFFFFFFFFu;
  er.TypeErase    = FLASH_TYPEERASE_SECTORS;
  er.VoltageRange = FLASH_VOLTAGE_RANGE_3; /* 2.7–3.6 V */
  er.Sector       = ECU_FLASH_SECTOR;
  er.NbSectors    = 1;

  if (HAL_FLASHEx_Erase(&er, &sectorError) != HAL_OK) {
    HAL_FLASH_Lock();
    if (!primask)
      __enable_irq();
    return -2;
  }

  /* Program as 32-bit words; pad last word if size not multiple of 4 */
  const uint8_t *bytes = (const uint8_t *)&blob;
  size_t nbytes = sizeof(blob);
  uint32_t addr = ECU_FLASH_SECTOR_ADDR;

  for (size_t off = 0; off < nbytes; off += 4u) {
    uint32_t word = 0xFFFFFFFFu;
    size_t n = nbytes - off;
    if (n >= 4u)
      memcpy(&word, bytes + off, 4u);
    else
      memcpy(&word, bytes + off, n);

    if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, addr + (uint32_t)off, word) != HAL_OK) {
      HAL_FLASH_Lock();
      if (!primask)
        __enable_irq();
      return -3;
    }
  }

  HAL_FLASH_Lock();
  if (!primask)
    __enable_irq();

  /* Read-back verify */
  if (!ECU_Flash_Present())
    return -4;

  /* Byte-compare payload (optional strict check) */
  {
    const EcuFlashBlob *p = (const EcuFlashBlob *)(uintptr_t)ECU_FLASH_SECTOR_ADDR;
    if (memcmp(p, &blob, sizeof(blob)) != 0)
      return -4;
  }
  return 0;
}
