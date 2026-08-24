/*
 * TorquEFI — STM32F411CCUx (256 KB flash, 128 KB RAM)
 *
 *   0x08000000  S0–S3   4×16 KB   code
 *   0x08010000  S4      64 KB     code
 *   0x08020000  S5     128 KB     *** ECU NVM (maps) ***
 *
 * Application FLASH = 128 KB (S0–S4). NVM = sector 5 @ 0x08020000.
 */
ENTRY(Reset_Handler)

_estack = ORIGIN(RAM) + LENGTH(RAM);
_Min_Heap_Size  = 0x400;
_Min_Stack_Size = 0x800;

MEMORY
{
  RAM    (xrw) : ORIGIN = 0x20000000, LENGTH = 128K
  FLASH  (rx)  : ORIGIN = 0x08000000, LENGTH = 128K
  EEU_NVM (r)  : ORIGIN = 0x08020000, LENGTH = 128K
}

_ecu_nvm_start = ORIGIN(EEU_NVM);
_ecu_nvm_size  = LENGTH(EEU_NVM);

SECTIONS
{
  .isr_vector :
  {
    . = ALIGN(4);
    KEEP(*(.isr_vector))
    . = ALIGN(4);
  } >FLASH

  .text :
  {
    . = ALIGN(4);
    *(.text) *(.text*) *(.glue_7) *(.glue_7t) *(.eh_frame)
    KEEP(*(.init)) KEEP(*(.fini))
    . = ALIGN(4);
    _etext = .;
  } >FLASH

  .rodata :
  {
    . = ALIGN(4);
    *(.rodata) *(.rodata*)
    . = ALIGN(4);
  } >FLASH

  ._user_heap_stack :
  {
    . = ALIGN(8);
    PROVIDE(end = .);
    PROVIDE(_end = .);
    . = . + _Min_Heap_Size;
    . = . + _Min_Stack_Size;
    . = ALIGN(8);
  } >RAM

  .data :
  {
    . = ALIGN(4);
    _sdata = .;
    *(.data) *(.data*)
    . = ALIGN(4);
    _edata = .;
  } >RAM AT> FLASH

  _sidata = LOADADDR(.data);

  .bss :
  {
    . = ALIGN(4);
    _sbss = .;
    __bss_start__ = _sbss;
    *(.bss) *(.bss*) *(COMMON)
    . = ALIGN(4);
    _ebss = .;
    __bss_end__ = _ebss;
  } >RAM
}
