# STM32F411 flash layout (TorquEFI / STRIX)

## F411CE (512 KB) — default Black Pill

| Region | Address | Sector | Size | Use |
|--------|---------|--------|------|-----|
| App | `0x08000000` | S0–S6 | 384 KB | Code + const (linker FLASH) |
| NVM | `0x08060000` | **S7** | 128 KB | Maps / cal (`ecu_flash`) |

Linker: `STM32F411CEUX_FLASH.ld` → `FLASH LENGTH = 384K`

## F411CC (256 KB)

| Region | Address | Sector | Size | Use |
|--------|---------|--------|------|-----|
| App | `0x08000000` | S0–S4 | 128 KB | Code |
| NVM | `0x08020000` | **S5** | 128 KB | Maps / cal |

Linker: `STM32F411CCUX_FLASH.ld` → `FLASH LENGTH = 128K`

## Runtime

`ecu_flash.c` reads `0x1FFF7A22` (flash size KB) and selects S7 vs S5 automatically.
Blob sits at **offset 0** of the NVM sector; CRC-32 protects the payload.

## CubeIDE

Project → Properties → C/C++ Build → Settings → MCU GCC Linker → General  
→ Linker script: point to the matching `.ld` file above.
