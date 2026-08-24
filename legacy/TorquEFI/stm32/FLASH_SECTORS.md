# STM32F411 NVM sector map (TorquEFI)

## Flash layout (reference)

| Sector | Address        | Size   | Use                          |
|--------|----------------|--------|------------------------------|
| 0–3    | 0x08000000     | 4×16KB | Application                  |
| 4      | 0x08010000     | 64KB   | Application                  |
| 5      | 0x08020000     | 128KB  | App **or** NVM on 256KB part |
| 6      | 0x08040000     | 128KB  | Application (512KB only)     |
| 7      | 0x08060000     | 128KB  | **NVM maps** (512KB only)    |

## Selection in firmware (`ecu_flash.c`)

| Chip flash size | NVM base       | Sector macro    |
|-----------------|----------------|-----------------|
| ≥ 512 KB (CE)   | `0x08060000`   | `FLASH_SECTOR_7` |
| ≤ 256 KB (CC)   | `0x08020000`   | `FLASH_SECTOR_5` |

Size is read from `FLASHSIZE` register at `0x1FFF7A22`.

## Linker requirement

Application **must not** occupy the NVM sector:

- **F411CEUx (512KB):** `FLASH (rx) : ORIGIN = 0x08000000, LENGTH = 384K`
- **F411CCUx (256KB):** `FLASH (rx) : ORIGIN = 0x08000000, LENGTH = 128K`  
  (NVM is sector 5 @ 0x08020000 — app only sectors 0–4 = 128KB)

If the linker uses full 512K for code, erasing sector 7 is safe only if
`.text` ends below 0x08060000. Prefer explicit 384K LENGTH.

## Blob contents (`EcuFlashBlob`, version 4)

- Magic `0xECAF4110`, CRC32 over all bytes before `crc32`
- Advance map: `int8_t[15][22]` (degrees)
- Injector map: `uint8_t[15][22]` in **0.1 ms units** (25 = 2.5 ms)
- TPS/pedal cal, trigger angle, teeth, missing, LTFT

## Why injection can show while map is 0.0

`ECU_Loop` clamps pulse width to a **minimum 800 µs**.  
A zero inj map still produces PW ≈ 0.8 ms on the live strip.

## Debug commands

```
GETFLASH   → FLASH:OK,A0:..,I0:..,SUMA:..,SUMI:..,ADDR:..,SEC:..,SZ:..
GETMAPSUM  → MAPSUM:A:..,I:..,A00:..,I00:..
SAVE       → OK:SAVE,RAMA0:..,FLA0:..,RAMI0:..,FLI0:..
GETMAP     → dumps ADV then INJ (INJ as ms ×0.1 cells)
```

If `SUMI:0` after SAVE, the RAM inj table was empty before program —
re-upload INJ map then SAVE again.
