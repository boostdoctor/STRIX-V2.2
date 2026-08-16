/**
 * TorquEFI NVM — last flash sector
 * Errors: -1 null, -2 erase, -3 program, -4 verify, -5 size, -6 CRC
 */
#include "ecu_flash.h"
#include "stm32f4xx_hal.h"
#include <string.h>
#include <stddef.h>

/* Fallbacks if an older ecu_flash.h is still on the include path */
#ifndef ECU_NVM_BASE_512K
#define ECU_NVM_BASE_512K   0x08060000u
#endif
#ifndef ECU_NVM_BASE_256K
#define ECU_NVM_BASE_256K   0x08020000u
#endif
#ifndef ECU_NVM_SECTOR_SIZE
#define ECU_NVM_SECTOR_SIZE 0x00020000u
#endif
#ifndef ECU_FLASH_MAGIC
#define ECU_FLASH_MAGIC     0xECAF4110u
#endif
#ifndef ECU_FLASH_VERSION
#define ECU_FLASH_VERSION   9u
#endif

_Static_assert(sizeof(EcuFlashBlob) < 4096u, "blob too large");
_Static_assert(offsetof(EcuFlashBlob, crc32) + 4u == sizeof(EcuFlashBlob),
               "crc32 must be last field");

#define FLASH_SIZE_KB_REG  (*(const uint16_t *)0x1FFF7A22u)

/* Prefer compile-time chip ID — FLASHSIZE reg is wrong on some clones */
static int is_512k(void)
{
#if defined(STM32F411xE)
  return 1;   /* F411CE 512 KB → NVM sector 7 */
#elif defined(STM32F411xC)
  return 0;   /* F411CC 256 KB → NVM sector 5 */
#else
  return (FLASH_SIZE_KB_REG >= 512u) ? 1 : 0;
#endif
}

static uint32_t sector_addr(void)
{
  if (is_512k())
    return (uint32_t)ECU_NVM_BASE_512K;  /* 0x08060000 */
  return (uint32_t)ECU_NVM_BASE_256K;    /* 0x08020000 */
}

static uint32_t sector_index(void)
{
  if (is_512k())
    return (uint32_t)FLASH_SECTOR_7;
  return (uint32_t)FLASH_SECTOR_5;
}

uint32_t ECU_Flash_SectorAddr(void)  { return sector_addr(); }
uint32_t ECU_Flash_SectorIndex(void) { return sector_index(); }
uint32_t ECU_Flash_SectorSize(void)  { return (uint32_t)ECU_NVM_SECTOR_SIZE; }

/* ── CRC: software reflected CRC-32 (authoritative for NVM) ─────────
 * Hardware CRC unit kept for optional diagnostics (different poly/result).
 * ─────────────────────────────────────────────────────────────────── */

void ECU_Flash_CrcInit(void)
{
  RCC->AHB1ENR |= RCC_AHB1ENR_CRCEN;
  __DSB();
  CRC->CR = CRC_CR_RESET;
}

/** ISO-HDLC / zlib CRC-32 — used for blob.crc32 */
static uint32_t crc32_sw(const uint8_t *data, size_t len)
{
  uint32_t crc = 0xFFFFFFFFu;
  for (size_t i = 0; i < len; i++) {
    crc ^= data[i];
    for (int b = 0; b < 8; b++)
      crc = (crc >> 1) ^ (0xEDB88320u & (uint32_t)-(int32_t)(crc & 1u));
  }
  return ~crc;
}

uint32_t ECU_Flash_CrcBuffer(const void *data, size_t len)
{
  if (!data && len)
    return 0u;
  return crc32_sw((const uint8_t *)data, len);
}

/** STM32 F4 HW CRC (poly 0x04C11DB7) — diagnostic only, not stored in blob */
uint32_t ECU_Flash_CrcHw(const void *data, size_t len)
{
  if (!data && len)
    return 0u;
  RCC->AHB1ENR |= RCC_AHB1ENR_CRCEN;
  __DSB();
  CRC->CR = CRC_CR_RESET;
  const uint8_t *p = (const uint8_t *)data;
  size_t words = len / 4u;
  for (size_t i = 0; i < words; i++) {
    uint32_t w;
    memcpy(&w, p + i * 4u, 4u);
    CRC->DR = w;
  }
  size_t rem = len % 4u;
  if (rem) {
    uint32_t w = 0u;
    memcpy(&w, p + words * 4u, rem);
    CRC->DR = w;
  }
  return CRC->DR;
}

uint32_t ECU_Flash_CrcCalc(const EcuFlashBlob *blob)
{
  if (!blob)
    return 0u;
  return ECU_Flash_CrcBuffer(blob, offsetof(EcuFlashBlob, crc32));
}


static void flash_clear_flags(void)
{
  __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR |
                         FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR);
}

static void cache_off(void)
{
#if defined(__HAL_FLASH_DATA_CACHE_DISABLE)
  __HAL_FLASH_DATA_CACHE_DISABLE();
#endif
#if defined(__HAL_FLASH_INSTRUCTION_CACHE_DISABLE)
  __HAL_FLASH_INSTRUCTION_CACHE_DISABLE();
#endif
}

static void cache_on(void)
{
#if defined(__HAL_FLASH_INSTRUCTION_CACHE_RESET)
  __HAL_FLASH_INSTRUCTION_CACHE_RESET();
#endif
#if defined(__HAL_FLASH_DATA_CACHE_RESET)
  __HAL_FLASH_DATA_CACHE_RESET();
#endif
#if defined(__HAL_FLASH_INSTRUCTION_CACHE_ENABLE)
  __HAL_FLASH_INSTRUCTION_CACHE_ENABLE();
#endif
#if defined(__HAL_FLASH_DATA_CACHE_ENABLE)
  __HAL_FLASH_DATA_CACHE_ENABLE();
#endif
  __DSB();
  __ISB();
}

static int flash_program_blob(const EcuFlashBlob *blob)
{
  const uint32_t base = sector_addr();
  const uint32_t sec  = sector_index();

  HAL_FLASH_Unlock();
  flash_clear_flags();
  cache_off();

  FLASH_EraseInitTypeDef er;
  memset(&er, 0, sizeof(er));
  uint32_t sectorError = 0xFFFFFFFFu;
  er.TypeErase    = FLASH_TYPEERASE_SECTORS;
  er.VoltageRange = FLASH_VOLTAGE_RANGE_3;
  er.Sector       = sec;
  er.NbSectors    = 1;
#if defined(FLASH_BANK_1)
  er.Banks        = FLASH_BANK_1;
#endif

  if (HAL_FLASHEx_Erase(&er, &sectorError) != HAL_OK) {
    flash_clear_flags();
    cache_on();
    HAL_FLASH_Lock();
    return -2;
  }
  __DSB();
  __ISB();

  const uint8_t *bytes = (const uint8_t *)blob;
  size_t nbytes = sizeof(*blob);
  size_t words = (nbytes + 3u) / 4u;
  for (size_t w = 0; w < words; w++) {
    uint32_t word = 0xFFFFFFFFu;
    size_t off = w * 4u;
    size_t n = nbytes - off;
    if (n >= 4u)
      memcpy(&word, bytes + off, 4u);
    else if (n > 0u)
      memcpy(&word, bytes + off, n);

    if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, base + (uint32_t)off, word) != HAL_OK) {
      flash_clear_flags();
      cache_on();
      HAL_FLASH_Lock();
      return -3;
    }
  }

  __DSB();
  __ISB();
  cache_on();
  HAL_FLASH_Lock();
  return 0;
}

static int flash_byte_verify(const EcuFlashBlob *blob)
{
  const uint8_t *src = (const uint8_t *)blob;
  const volatile uint8_t *dst =
      (const volatile uint8_t *)(uintptr_t)sector_addr();
  for (size_t i = 0; i < sizeof(*blob); i++) {
    if (dst[i] != src[i])
      return -4;
  }
  return 0;
}

static int flash_crc_verify(void)
{
  const EcuFlashBlob *p = (const EcuFlashBlob *)(uintptr_t)sector_addr();
  if (p->magic != ECU_FLASH_MAGIC)
    return -6;
  /* Accept v4–v6; all use software CRC-32 for crc32 field.
   * v5 HW-CRC blobs will fail CRC and fall through to defaults. */
  if (p->version < 4u || p->version > ECU_FLASH_VERSION)
    return -6;
  uint32_t expect = ECU_Flash_CrcBuffer(p, offsetof(EcuFlashBlob, crc32));
  if (expect != p->crc32)
    return -6;
  return 0;
}

int ECU_Flash_Present(void)
{
  return (flash_crc_verify() == 0) ? 1 : 0;
}

uint32_t ECU_Flash_StoredCrc(void)
{
  if (!ECU_Flash_Present())
    return 0u;
  const EcuFlashBlob *p = (const EcuFlashBlob *)(uintptr_t)sector_addr();
  return p->crc32;
}

int ECU_Flash_Load(EcuFlashBlob *out)
{
  if (!out)
    return 0;
  if (!ECU_Flash_Present())
    return 0;
  const EcuFlashBlob *p = (const EcuFlashBlob *)(uintptr_t)sector_addr();
  memcpy(out, p, sizeof(*out));
  if (ECU_Flash_CrcCalc(out) != out->crc32)
    return 0;
  return 1;
}

int ECU_Flash_Save(const EcuFlashBlob *in)
{
  if (!in)
    return -1;
  if (sizeof(EcuFlashBlob) > (size_t)ECU_NVM_SECTOR_SIZE)
    return -5;

  EcuFlashBlob blob;
  memcpy(&blob, in, sizeof(blob));
  blob.magic   = ECU_FLASH_MAGIC;
  blob.version = ECU_FLASH_VERSION;
  blob.crc32   = 0;
  blob.crc32   = ECU_Flash_CrcCalc(&blob);

  uint32_t primask = __get_PRIMASK();
  __disable_irq();
  int err = flash_program_blob(&blob);
  if (!primask)
    __enable_irq();

  if (err == 0)
    err = flash_byte_verify(&blob);
  if (err == 0)
    err = flash_crc_verify();

  if (err == -4 || err == -6) {
    primask = __get_PRIMASK();
    __disable_irq();
    err = flash_program_blob(&blob);
    if (!primask)
      __enable_irq();
    if (err == 0)
      err = flash_byte_verify(&blob);
    if (err == 0)
      err = flash_crc_verify();
  }

  return err;
}
