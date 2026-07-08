OUTPUT_ARCH("riscv")
ENTRY(_start)
SECTIONS
{
  . = 0x00400000;                 /* USER_TEXT_START -> mem.text.bin */
  .text : {
    KEEP(*(.text.init))
    *(.text) *(.text.*)
    *(.rodata) *(.rodata.*) *(.srodata) *(.srodata.*)
    . = ALIGN(8);
  }
  . = 0x10000000;                 /* USER_DATA_START -> mem.data.bin */
  .data : {
    *(.data) *(.data.*)
    . = ALIGN(8);
    PROVIDE(__global_pointer$ = . + 0x800);
    *(.sdata) *(.sdata.*)
    . = ALIGN(8);
  }
  .bss : {
    *(.bss) *(.bss.*) *(.sbss) *(.sbss.*) *(COMMON)
    . = ALIGN(8);
  }
  _end = .;
}
