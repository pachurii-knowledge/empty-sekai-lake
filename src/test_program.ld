/**
 * test_program.ld
 *
 * RISC-V 32-bit Processor
 *
 * ECE 18-447
 * Carnegie Mellon University
 *
 * This file contains the linker script used to layout RISC-V test programs.
 *
 * The linker script defines the layout and contents of all the sections that
 * that a RISC-V test program can have. It defines the starting addresses of
 * each section and also which subsections of which each section is composed.
 * The linker script is really only required for C programs, as the user can
 * manually defines sections in an assembly file.
 *
 * Authors:
 *  - 2017: Brandon Perez
 **/

/*----------------------------------------------------------------------------*
 *                          DO NOT MODIFY THIS FILE!                          *
 *          You should only add or change files in the src directory!         *
 *----------------------------------------------------------------------------*/

SECTIONS
{
    /* The user text section, which contains user code. To simplify loading the
     * program into memory for simulation, all sections which contain read-only
     * global variables are packed into the .text section. */
    .text 0x00400000:
    {
        *(.text .text.*)
        *(.rodata .rodata.*)
        . = ALIGN(4);
    }

    /* The user data section, which contains the writable user global variables.
     * To simplify loading the program into memory for simulation, all sections
     * which contain writable global variables are packed into the .data
     * section. */
    .data 0x10000000:
    {
        *(.data .data.*)
        *(.sdata .sdata.*)
        . = ALIGN(4);
    }

    /* The user bss section, which contains all uninitialized user global
     * variables. These are zero-initialized. The section is placed immediately
     * after the .data section. */
    .bss . :
    {
        *(.bss .bss.*)
        *(.sbss .sbss.*)
        . = ALIGN(4);
    }

    /* The kernel text section, which contains kernel code. */
    .ktext 0x80000000:
    {
        *(.ktext .ktext.*)
        . = ALIGN(4);
    }

    /* The kernel data section, which contains kernel global variables. */
    .kdata 0x90000000:
    {
        *(.kdata .kdata.*)
        . = ALIGN(4);
    }

    /* The kernel bss section, which contains uninitialized kernel global
     * variables. This is placed directly after the .kdata section. */
    .kbss . :
    {
        *(.kbss .kbss.*)
        *(.ksbss .ksbss.*)
        . = ALIGN(4);
    }
}
