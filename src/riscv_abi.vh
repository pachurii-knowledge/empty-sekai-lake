/**
 * riscv_abi.vh
 *
 * RISC-V 32-bit Processor
 *
 * ECE 18-447
 * Carnegie Mellon University
 *
 * This file contains the definitions for the RISC-V application binary
 * interface (ABI), which are namely the register aliases for application
 * registers, such as temporaries, the stack pointer, etc.
 *
 * Note that the names of the enumerations are based on the names and
 * assignments in chapter 20 of the RISC-V 2.2 ISA manual.
 *
 * Authors:
 *  - 2016 - 2017: Brandon Perez
 **/

/*----------------------------------------------------------------------------*
 *                          DO NOT MODIFY THIS FILE!                          *
 *          You should only add or change files in the src directory!         *
 *----------------------------------------------------------------------------*/

`ifndef RISCV_ABI_VH_
`define RISCV_ABI_VH_

// Local Includes
`include "riscv_isa.vh"     // RISC-V ISA register, register number width

/*----------------------------------------------------------------------------
 * Definitions
 *----------------------------------------------------------------------------*/

package RISCV_ABI;

    // Import the number of bits for a register number, and all ISA registers
    import RISCV_ISA::*;

    /* The value that must be passed in register a0 (x10) to the ECALL
     * instruction to halt the processor. */
    parameter ECALL_ARG_HALT    = 'ha;

    // Aliases for the registers in the application binary interface (ABI)
    typedef enum logic [REG_NUM_WIDTH-1:0] {
        ZERO    = X0,       // Zero register, hardwired to 0
        RA      = X1,       // Return address register (caller-saved)
        SP      = X2,       // Stack pointer register (callee-saved)
        GP      = X3,       // Global pointer register (points to data section)
        TP      = X4,       // Thread pointer (points to thread-local data)
        T0      = X5,       // Temporary register 0 (caller-saved)
        T1      = X6,       // Temporary register 0 (caller-saved)
        T2      = X7,       // Temporary register 0 (caller-saved)
        S0_FP   = X8,       // Saved 0/stack frame pointer (callee-saved)
        S1      = X9,       // Saved register 1 (callee-saved)
        A0      = X10,      // Function argument/return value 0 (caller-saved)
        A1      = X11,      // Function argument/return value 1 (caller-saved)
        A2      = X12,      // Function argument 2 (caller-saved)
        A3      = X13,      // Function argument 3 (caller-saved)
        A4      = X14,      // Function argument 4 (caller-saved)
        A5      = X15,      // Function argument 5 (caller-saved)
        A6      = X16,      // Function argument 6 (caller-saved)
        A7      = X17,      // Function argument 7 (caller-saved)
        S2      = X18,      // Saved register 2 (callee-saved)
        S3      = X19,      // Saved register 3 (callee-saved)
        S4      = X20,      // Saved register 4 (callee-saved)
        S5      = X21,      // Saved register 5 (callee-saved)
        S6      = X22,      // Saved register 6 (callee-saved)
        S7      = X23,      // Saved register 7 (callee-saved)
        S8      = X24,      // Saved register 8 (callee-saved)
        S9      = X25,      // Saved register 9 (callee-saved)
        S10     = X26,      // Saved register 10 (callee-saved)
        S11     = X27,      // Saved register 11 (callee-saved)
        T3      = X28,      // Temporary register 3 (caller-saved)
        T4      = X29,      // Temporary register 4 (caller-saved)
        T5      = X30,      // Temporary register 5 (caller-saved)
        T6      = X31       // Temporary register 6 (caller-saved)
    } abi_reg_t;

endpackage: RISCV_ABI

`endif /* RISCV_ABI_VH_ */
