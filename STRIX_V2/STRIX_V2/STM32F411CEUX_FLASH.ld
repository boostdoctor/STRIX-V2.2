/*
 * STRIX V2 / TorquEFI — STM32F411CEUx linker script
 * =================================================
 * MCU: STM32F411CEU6 (Black Pill) — 512 KB Flash, 128 KB SRAM
 *
 * Flash sector map (STM32F4 dual-bank style sectors on F411):
 *
 *   Address      Sector   Size     Use
 *   ───────────  ──────   ──────   ─────────────────────────────
 *   0x08000000   S0       16 KB    ISR vectors + early code
 *   0x08004000   S1       16 KB    code
 *   0x08008000   S2       16 KB    code
 *   0x0800C000   S3       16 KB    code
 *   0x08010000   S4       64 KB    code
 *   0x08020000   S5      128 KB    code
 *   0x08040000   S6      128 KB    code
 *   0x08060000   S7      128 KB    *** ECU NVM (maps + settings) ***
 *   0x08080000            —        end of 512 KB
 *
 * Application FLASH length is 384 KB (S0–S6 only) so the linker NEVER
 * places .text/.rodata in sector 7.  ecu_flash.c erases/programs S7 only.
 *
 * CubeIDE setup:
 *   Project → Properties → C/C++ Build → Settings → Tool Settings
 *     → MCU GCC Linker → General → Linker script (-T)
 *       → path to this file:  STM32F411CEUX_FLASH.ld
 *   Or replace the generated STM32F411CEUX_FLASH.ld in the project root
 *   after CubeMX code generation.
 *
 * For STM32F411CC (256 KB): use LENGTH(FLASH)=128K and NVM at 0x08020000
 * (sector 5). See LINKER.md.
 */

ENTRY(Reset_Handler)

/* Top of stack = end of SRAM */
_estack = ORIGIN(RAM) + LENGTH(RAM);

/* Minimum sizes checked at link time via ._user_heap_stack */
_Min_Heap_Size  = 0x400;   /* 1 KB heap  */
_Min_Stack_Size = 0x800;   /* 2 KB stack */

/* -------------------------------------------------------------------------- */
MEMORY
{
  /* 128 KB SRAM @ 0x20000000 */
  RAM    (xrw) : ORIGIN = 0x20000000, LENGTH = 128K

  /* 384 KB application flash — sectors 0..6 (leaves S7 free) */
  FLASH  (rx)  : ORIGIN = 0x08000000, LENGTH = 384K

  /* Reserved NVM — not allocated to any output section; documents S7 */
  ECU_NVM (r)  : ORIGIN = 0x08060000, LENGTH = 128K
}

/* Absolute symbols usable from C (optional):
 *   extern uint8_t _ecu_nvm_start[], _ecu_nvm_size[];
 * Prefer ECU_Flash_SectorAddr() in ecu_flash.c for runtime selection.
 */
_ecu_nvm_start = ORIGIN(ECU_NVM);
_ecu_nvm_size  = LENGTH(ECU_NVM);

/* -------------------------------------------------------------------------- */
SECTIONS
{
  /* Cortex-M vector table — must be first in flash */
  .isr_vector :
  {
    . = ALIGN(4);
    KEEP(*(.isr_vector))
    . = ALIGN(4);
  } >FLASH

  /* Executable code */
  .text :
  {
    . = ALIGN(4);
    *(.text)
    *(.text*)
    *(.glue_7)
    *(.glue_7t)
    *(.eh_frame)

    KEEP(*(.init))
    KEEP(*(.fini))

    . = ALIGN(4);
    _etext = .;
  } >FLASH

  /* Constants / maps baked into firmware (not NVM blob) */
  .rodata :
  {
    . = ALIGN(4);
    *(.rodata)
    *(.rodata*)
    . = ALIGN(4);
  } >FLASH

  /* ARM exception index (if any) */
  .ARM.extab :
  {
    *(.ARM.extab* .gnu.linkonce.armextab.*)
  } >FLASH

  .ARM :
  {
    __exidx_start = .;
    *(.ARM.exidx*)
    __exidx_end = .;
  } >FLASH

  .preinit_array :
  {
    PROVIDE_HIDDEN(__preinit_array_start = .);
    KEEP(*(.preinit_array*))
    PROVIDE_HIDDEN(__preinit_array_end = .);
  } >FLASH

  .init_array :
  {
    PROVIDE_HIDDEN(__init_array_start = .);
    KEEP(*(SORT(.init_array.*)))
    KEEP(*(.init_array*))
    PROVIDE_HIDDEN(__init_array_end = .);
  } >FLASH

  .fini_array :
  {
    PROVIDE_HIDDEN(__fini_array_start = .);
    KEEP(*(SORT(.fini_array.*)))
    KEEP(*(.fini_array*))
    PROVIDE_HIDDEN(__fini_array_end = .);
  } >FLASH

  /* Initialized data: load address in FLASH, run address in RAM */
  _sidata = LOADADDR(.data);

  .data :
  {
    . = ALIGN(4);
    _sdata = .;        /* start of .data in RAM */
    *(.data)
    *(.data*)
    *(.RamFunc)        /* functions copied to RAM if any */
    *(.RamFunc*)
    . = ALIGN(4);
    _edata = .;        /* end of .data in RAM */
  } >RAM AT> FLASH

  /* Zero-initialized BSS */
  .bss :
  {
    . = ALIGN(4);
    _sbss = .;
    __bss_start__ = _sbss;
    *(.bss)
    *(.bss*)
    *(COMMON)
    . = ALIGN(4);
    _ebss = .;
    __bss_end__ = _ebss;
  } >RAM

  /* Heap + stack reservation (link fails if RAM overflow) */
  ._user_heap_stack :
  {
    . = ALIGN(8);
    PROVIDE(end = .);
    PROVIDE(_end = .);
    . = . + _Min_Heap_Size;
    . = . + _Min_Stack_Size;
    . = ALIGN(8);
  } >RAM

  /* Remove library debug clutter from output */
  /DISCARD/ :
  {
    libc.a ( * )
    libm.a ( * )
    libgcc.a ( * )
  }

  .ARM.attributes 0 : { *(.ARM.attributes) }
}

/* End of linker script */
