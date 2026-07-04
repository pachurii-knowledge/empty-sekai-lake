OUTPUT_ARCH("riscv")
ENTRY(rvtest_entry_point)

SECTIONS
{
  . = 0x00400000;
  .text.init : { *(.text.init) }
  .text.rvtest : { *(.text.rvtest) *(.text.rvtest.*) }
  . = ALIGN(0x1000);
  .text.rvmodel : { *(.text.rvmodel) *(.text.rvmodel.*) *(.text) *(.text.*) *(.rodata) *(.rodata.*) }

  . = 0x10000000;
  .data : { *(.data) *(.tohost) *(.sdata) *(.sdata.*) }
  .bss : { *(.bss) *(.sbss) *(.sbss.*) }

  . = ALIGN(0x1000);
  _end = .;
}
