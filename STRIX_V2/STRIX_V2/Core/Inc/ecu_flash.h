/**
 * TorquEFI / STRIX V2 NVM — full tune + engine settings
 *
 *   F411CEUx 512 KB → SECTOR_7 @ 0x08060000 (128 KB)
 *   F411CCUx 256 KB → SECTOR_5 @ 0x08020000 (128 KB)
 *
 * v8 = maps + engine settings block + CRC32
 */
#ifndef ECU_FLASH_H
#define ECU_FLASH_H

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

#define ECU_FLASH_MAGIC     0xECAF4110u
#define ECU_FLASH_VERSION   9u

#define ECU_NVM_BASE_512K   0x08060000u
#define ECU_NVM_BASE_256K   0x08020000u
#define ECU_NVM_SECTOR_SIZE 0x00020000u

#define ECU_FLASH_MAP_ROWS  12
#define ECU_FLASH_MAP_COLS  22

/** Engine settings persisted with maps (survives power cycle without PC). */
typedef struct __attribute__((packed)) {
  uint8_t  loadMode;      /* 0=MAP 1=TPS 2=Hybrid */
  uint8_t  injMode;       /* 1=batch 2=seq 3=hybrid (legacy 0=auto) */
  uint16_t batchAboveRpm;
  uint8_t  coilSmart;     /* 1=smart 0=dumb */
  uint8_t  cylinders;
  uint8_t  wheelId;
  uint8_t  dbwEnable;
  uint8_t  idleOutMode;   /* 0=2wire 1=1wire 2=stepper 0xFF=disabled */
  uint8_t  idleEnable;
  uint16_t idleTargetRpm;
  uint16_t fpPrimeMs;
  uint16_t injPrimeMs;
  uint8_t  injPrimeEn;
  uint16_t rpmLimit;
  uint8_t  rpmCutMode;    /* 0=hard 1=soft */
  uint8_t  fanEnable;
  uint8_t  fanOnC;
  uint8_t  o2Mode;        /* 0=off 1=NB 2=WB */
  uint8_t  boostMode;
  uint8_t  vvtMode;
  uint8_t  sensEctEn;
  uint8_t  sensIatEn;
  uint8_t  sensO2En;
  uint8_t  sensMapEn;
  uint8_t  sensTpsEn;
  uint16_t mapLoadRefKpa; /* kPa at load=1.0 */
  uint8_t  ignMode;       /* 0=wasted spark 1=sequential */
  uint8_t  coilType;      /* 0=smart 1=dumb 2=dist */
  uint8_t  coilChargeMode;/* 0=const duty 1=const charge time */
  uint8_t  reserved[13];
} EcuFlashSettings;

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
  EcuFlashSettings settings; /* v8+ */
  uint32_t crc32;
} EcuFlashBlob;

void     ECU_Flash_CrcInit(void);
uint32_t ECU_Flash_CrcCalc(const EcuFlashBlob *blob);
uint32_t ECU_Flash_CrcBuffer(const void *data, size_t len);
uint32_t ECU_Flash_CrcHw(const void *data, size_t len);

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
