# Linker script configuration — STRIX V2

## File

`STM32F411CEUX_FLASH.ld` — use with **STM32F411CEU6** (Black Pill, 512 KB flash).

## Memory layout

| Region   | Origin     | Length | Purpose                          |
|----------|------------|--------|----------------------------------|
| **RAM**  | 0x20000000 | 128 KB | `.data`, `.bss`, heap, stack     |
| **FLASH**| 0x08000000 | **384 KB** | Application code + rodata     |
| **ECU_NVM** | 0x08060000 | 128 KB | Sector 7 — maps/settings blob |

Sector 7 is **not** part of the linked application image. `ecu_flash.c` erases and programs it at runtime via HAL.

```
0x08000000  ┌─────────────────────┐
            │  App (S0–S6)        │  384 KB  ← LENGTH(FLASH)
0x08060000  ├─────────────────────┤
            │  ECU NVM (S7)       │  128 KB  ← ECU_Flash_* only
0x08080000  └─────────────────────┘
```

## CubeIDE / CubeMX steps

1. **After “Generate Code”**, replace the project’s  
   `STM32F411CEUX_FLASH.ld` with this file  
   **or** point the linker at it:

   - *Project → Properties → C/C++ Build → Settings*  
   - *MCU GCC Linker → General → Linker script (-T)*  
   - Browse to `STM32F411CEUX_FLASH.ld`

2. **Do not** set FLASH length to 512K — that allows `.text` into NVM and the next map save can erase code.

3. Clean + rebuild. Check the `.map` file:

   ```
   FLASH          0x08000000   0x00060000
   ```
   Highest app address must be **&lt; 0x08060000**.

4. Optional C symbols (already provided by the script):

   ```c
   extern uint32_t _ecu_nvm_start;
   extern uint32_t _ecu_nvm_size;
   ```

   Runtime code should still use `ECU_Flash_SectorAddr()` so F411CC (256 KB) keeps working.

## Heap / stack

| Symbol            | Default | Notes                |
|-------------------|---------|----------------------|
| `_Min_Heap_Size`  | 0x400   | 1 KB                 |
| `_Min_Stack_Size` | 0x800   | 2 KB                 |

Increase stack if deep ISR + USB + printf paths fault (`HardFault` near `_estack`).

## STM32F411CC (256 KB) variant

| Region | Origin     | Length |
|--------|------------|--------|
| FLASH  | 0x08000000 | 128 KB |
| NVM    | 0x08020000 | 128 KB (sector 5) |

Match `ECU_NVM_BASE_256K` in `ecu_flash.h`. Application must stay below 0x08020000.

## Verify after Flash from tuner

1. Link address of `_etext` / end of `.rodata` &lt; 0x08060000  
2. `OK:SAVE` + `GETMAPSUM` after power-cycle  
3. App still boots (S7 erase did not wipe code)

## Common mistakes

| Mistake | Result |
|---------|--------|
| FLASH length = 512K | Maps save may erase code in S7 |
| NVM in a linked section | Linker puts const data in S7; erase kills it |
| Wrong `.ld` after CubeMX regen | Overwrites custom script — re-copy after generate |
