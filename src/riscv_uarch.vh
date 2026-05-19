/**
 * riscv_uarch.vh
 *
 * RISC-V 32-bit Processor
 *
 * ECE 18-447
 * Carnegie Mellon University
 *
 * This file contains definitions for the RISC-V microarchitecture
 * implemented by this processor. Namely, this defines parameters that determine
 * the processor's underlying architecture.
 *
 * Authors:
 *  - 2017: Brandon Perez
 **/

/*----------------------------------------------------------------------------*
 *                          DO NOT MODIFY THIS FILE!                          *
 *          You should only add or change files in the src directory!         *
 *----------------------------------------------------------------------------*/

`ifndef RISCV_UARCH_VH_
`define RISCV_UARCH_VH_

// Local Includes
`include "riscv_isa.vh"                 // Definition of word width and bytes

/*----------------------------------------------------------------------------
 * Definitions
 *----------------------------------------------------------------------------*/

package RISCV_UArch;

    // Import the width of addresses in the architecture and byte per word
    import RISCV_ISA::XLEN, RISCV_ISA::XLEN_BYTES;

    // Half of the period of the clock for the processor (simulation only)
    parameter CLOCK_HALF_PERIOD         = 50;

    // Execution processing width (multiplies the number of regfile ports).
    // Phase 2 keeps the scalar core as the default and enables the 4-wide path
    // explicitly with +define+SUPERSCALAR_4WIDE/-DSUPERSCALAR_4WIDE.
`ifdef SUPERSCALAR_4WIDE
    parameter SUPERSCALAR_WAYS          = 4;
`else
    parameter SUPERSCALAR_WAYS          = 1;
`endif
    
    // Number of consecutive words returned by memory read
    parameter MEMORY_READ_WIDTH            = (`LAB_18447 == "4a") ? 4 :
                                             (`LAB_18447 == "4b") ? 4 : 1;
    
    // Number of clock edges in memory read delay
    parameter IMEMORY_READ_DELAY            = (`LAB_18447 == "4a") ? 2 :
                                              (`LAB_18447 == "4b") ? 2 :
                                              (`LAB_18447 == "3") ? 2 :
                                              (`LAB_18447 == "2") ? 2 :
                                              (`LAB_18447 == "1b") ? 0 : 0;
    
    parameter DMEMORY_READ_DELAY            = (`LAB_18447 == "4a") ? 8 :
                                              (`LAB_18447 == "4b") ? 8 :
                                              (`LAB_18447 == "3") ? 2 :
                                              (`LAB_18447 == "2") ? 2 :
                                              (`LAB_18447 == "1b") ? 0 : 0;

    // The width of words in the SRAM used for the branch target buffer (BTB)
    parameter BTB_WORD_WIDTH            = 62;

    // The number of words (entries) in the SRAM used for the BTB, by default
    parameter BTB_NUM_WORDS             = 128;

    /* The width of the addresses used by main memory. The addresses only need
     * to be wide enough to address words in memory. */
    localparam MEMORY_ADDR_WIDTH        = XLEN - $clog2(XLEN_BYTES);

    // The number of ports that the processor's main memory has
    parameter MEMORY_NUM_PORTS          = 2;

    /* The ad-hoc parameter used to control the combinational delay in model for
     * main memory (synthesis only). */
    parameter MEMORY_DELAY_WIDTH        = 8;

endpackage: RISCV_UArch

`endif /* RISCV_UARCH_VH_ */
