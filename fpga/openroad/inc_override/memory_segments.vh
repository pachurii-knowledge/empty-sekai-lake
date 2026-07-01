/**
 * memory_segments.vh
 *
 * RISC-V 32-bit Processor
 *
 * ECE 18-447
 * Carnegie Mellon University
 *
 * This file contains the definitions for the segments in the processor memory.
 *
 * This defines the metadata about each segment in memory, namely its starting
 * address and maximum size. This also defines an array that contains all the
 * segments that are present in the processor's memory.
 *
 * Authors:
 *  - 2017: Brandon Perez
 **/

/*----------------------------------------------------------------------------*
 *                          DO NOT MODIFY THIS FILE!                          *
 *          You should only add or change files in the src directory!         *
 *----------------------------------------------------------------------------*/

`ifndef MEMORY_SEGMENTS_VH_
`define MEMORY_SEGMENTS_VH_

`include "riscv_isa.vh"             // Definition of XLEN_BYTES

package MemorySegments;

/*----------------------------------------------------------------------------
 * Memory Segment Addresses
 *----------------------------------------------------------------------------*/

    // Import the number of bytes in a word/register
    import RISCV_ISA::XLEN_BYTES;

    // The number of memory segments in the processor
    parameter NUM_SEGMENTS          = 5;

    /* The size of each segment in bytes. The size of each segment is kept
     * fixed, so that its size is known at elaboration-time, allowing for memory
     * dumps for the DVE GUI. */
    parameter SEGMENT_SIZE          = 512 * 1024;
    localparam SEGMENT_WORDS        = SEGMENT_SIZE / XLEN_BYTES;

    // The starting addresses of the user's data and text segments
    parameter USER_TEXT_START       = 'h0040_0000;
    parameter USER_DATA_START       = 'h1000_0000;

    // The starting and ending addresses of the stack segment, and its size
    parameter STACK_END             = 'h7ff0_0000;
    localparam STACK_START          = STACK_END - SEGMENT_SIZE;

    // The starting addresses and sizes of the kernel's data, and text segments
    parameter KERNEL_TEXT_START     = 'h8000_0000;
    parameter KERNEL_DATA_START     = 'h9000_0000;

endpackage: MemorySegments

`endif /* MEMORY_SEGMENTS_VH_ */
