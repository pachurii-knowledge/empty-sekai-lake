/**
 * riscv_isa.vh
 *
 * RISC-V 32-bit Processor
 *
 * ECE 18-447
 * Carnegie Mellon University
 *
 * This file contains the definitions for the RISC-V instruction set
 * architecture (ISA). These are definitions for the RISC-V opcodes and
 * function codes for that must be implemented by the processor.
 *
 * Note that the names of the enumerations are based on the names given in
 * chapter 2 of the RISC-V 2.2 ISA manual.
 *
 * Authors:
 *  - 2016 - 2017: Brandon Perez
 **/

/*----------------------------------------------------------------------------*
 *                          DO NOT MODIFY THIS FILE!                          *
 *          You should only add or change files in the src directory!         *
 *----------------------------------------------------------------------------*/

`ifndef RISCV_ISA_VH_
`define RISCV_ISA_VH_

package RISCV_ISA;

/*----------------------------------------------------------------------------
 * Definitions
 *----------------------------------------------------------------------------*/

    /* The number of registers in the register file, and the number of bits
     * needed to represent it. */
    parameter NUM_REGS          = 32;
    localparam REG_NUM_WIDTH    = $clog2(NUM_REGS);

    /* The size of registers, in bits and bytes. This defines the architecture
     * as either 32-bit or 64-bit. */
    localparam BYTE_WIDTH       = 8;
    parameter XLEN              = 32;
    localparam XLEN_BYTES       = XLEN / BYTE_WIDTH;

    // The sizes of opcodes and function codes in bits
    parameter OPCODE_WIDTH       = 7;
    parameter FUNCT3_WIDTH       = 3;
    parameter FUNCT7_WIDTH       = 7;
    parameter FUNCT12_WIDTH      = 12;

/*----------------------------------------------------------------------------
 * Opcodes (All Instruction Types)
 *----------------------------------------------------------------------------*/

    // Opcodes for the RISC-V ISA, in the lowest 7 bits of the instruction
    typedef enum logic [OPCODE_WIDTH-1:0] {
        // Opcode that indicates an integer R-type instruction (register)
        OP_OP                   = 'h33,

        // Opcode that indicates an integer I-type instruction (immediate)
        OP_IMM                  = 'h13,

        // Opcodes for load and store instructions (I-type and S-type)
        OP_LOAD                 = 'h03,
        OP_STORE                = 'h23,

        // Opcodes for U-type instructions (unsigned immediate)
        OP_LUI                  = 'h37,
        OP_AUIPC                = 'h17,

        // Opcodes for jump instructions (UJ-type and I-type)
        OP_JAL                  = 'h6F,
        OP_JALR                 = 'h67,

        // Opcode that indicates a general SB-type instruction (branch)
        OP_BRANCH               = 'h63,

        // Opcode that indicates a special system instruction (I-type)
        OP_SYSTEM               = 'h73
    } opcode_t;

/*----------------------------------------------------------------------------
 * 7-bit Function Codes (R-type and I-type Instructions)
 *----------------------------------------------------------------------------*/

    // 7-bit function codes, the highest 7 bits of the instruction
    typedef enum logic [FUNCT7_WIDTH-1:0] {
        FUNCT7_INT              = 'h00,     // Typical integer instruction
        FUNCT7_MULDIV           = 'h01,     // M extension MULDIV
        FUNCT7_ALT_INT          = 'h20      // Alternate (sub/sra/srai)
    } funct7_t;

/*----------------------------------------------------------------------------
 * R-type Function Codes
 *----------------------------------------------------------------------------*/

    // 3-bit function codes for integer R-type instructions
    typedef enum logic [FUNCT3_WIDTH-1:0] {
        FUNCT3_ADD_SUB          = 'h0,      // Add/subtract
        FUNCT3_SLL              = 'h1,      // Shift left logical
        FUNCT3_SLT              = 'h2,      // Set on less than signed
        FUNCT3_SLTU             = 'h3,      // Set on less than unsigned
        FUNCT3_XOR              = 'h4,      // Bit-wise xor
        FUNCT3_SRL_SRA          = 'h5,      // Shift right logical/arithmetic
        FUNCT3_OR               = 'h6,      // Bit-wise or
        FUNCT3_AND              = 'h7       // Bit-wise and
    } rtype_funct3_t;

/*----------------------------------------------------------------------------
 * I-type Function Codes
 *----------------------------------------------------------------------------*/

    // 3-bit function codes for integer I-type instructions
    typedef enum logic [FUNCT3_WIDTH-1:0] {
        FUNCT3_ADDI             = 'h0,      // Add immediate
        FUNCT3_SLTI             = 'h2,      // Set on less than signed
        FUNCT3_SLTIU            = 'h3,      // Set on less than unsigned
        FUNCT3_XORI             = 'h4,      // Bit-wise xor immediate
        FUNCT3_ORI              = 'h6,      // Bit-wise or immediate
        FUNCT3_ANDI             = 'h7,      // Bit-wise and immediate
        FUNCT3_SLLI             = 'h1,      // Shift left logical immediate
        FUNCT3_SRLI_SRAI        = 'h5       // Shift right logical/arithmetic
    } itype_int_funct3_t;

    // 3-bit function codes for load instructions (I-type)
    typedef enum logic [FUNCT3_WIDTH-1:0] {
        FUNCT3_LB               = 'h0,      // Load byte (1 byte) signed
        FUNCT3_LH               = 'h1,      // Load halfword (2 bytes) signed
        FUNCT3_LW               = 'h2,      // Load word (4 bytes)
        FUNCT3_LBU              = 'h4,      // Load byte (1 byte) unsigned
        FUNCT3_LHU              = 'h5       // Load halfword (2 bytes) unsigned
    } itype_load_funct3_t;

    // 12-bit function codes for special system instructions (I-type)
    typedef enum logic [FUNCT12_WIDTH-1:0] {
        FUNCT12_ECALL           = 'h000     // Environment call
    } itype_funct12_t;

/*----------------------------------------------------------------------------
 * S-type Function Codes
 *----------------------------------------------------------------------------*/

    // 3-bit function codes for S-type instructions (store)
    typedef enum logic [FUNCT3_WIDTH-1:0] {
        FUNCT3_SB               = 'h0,      // Store byte (1 byte)
        FUNCT3_SH               = 'h1,      // Store halfword (2 bytes)
        FUNCT3_SW               = 'h2       // Store word (4 bytes)
    } stype_funct3_t;

/*----------------------------------------------------------------------------
 * SB-type Function Codes
 *----------------------------------------------------------------------------*/

    // 3-bit function codes for SB-type instructions (branch)
    typedef enum logic [FUNCT3_WIDTH-1:0] {
        FUNCT3_BEQ              = 'h0,      // Branch if equal
        FUNCT3_BNE              = 'h1,      // Branch if not equal
        FUNCT3_BLT              = 'h4,      // Branch if less than (signed)
        FUNCT3_BGE              = 'h5,      // Branch if greater than or equal
        FUNCT3_BLTU             = 'h6,      // Branch if less than (unsigned)
        FUNCT3_BGEU             = 'h7       // Branch if greater than or equal
    } sbtype_funct3_t;

/*----------------------------------------------------------------------------
 * ISA Register Names
 *----------------------------------------------------------------------------*/

    // Enumeration of the registers in the ISA
    typedef enum logic [REG_NUM_WIDTH-1:0] {
        X0                      = 'd0,      // ISA Register 0, hardwired to 0
        X1                      = 'd1,      // ISA Register 1
        X2                      = 'd2,      // ISA Register 2
        X3                      = 'd3,      // ISA Register 3
        X4                      = 'd4,      // ISA Register 4
        X5                      = 'd5,      // ISA Register 5
        X6                      = 'd6,      // ISA Register 6
        X7                      = 'd7,      // ISA Register 7
        X8                      = 'd8,      // ISA Register 8
        X9                      = 'd9,      // ISA Register 9
        X10                     = 'd10,     // ISA Register 10
        X11                     = 'd11,     // ISA Register 11
        X12                     = 'd12,     // ISA Register 12
        X13                     = 'd13,     // ISA Register 13
        X14                     = 'd14,     // ISA Register 14
        X15                     = 'd15,     // ISA Register 15
        X16                     = 'd16,     // ISA Register 16
        X17                     = 'd17,     // ISA Register 17
        X18                     = 'd18,     // ISA Register 18
        X19                     = 'd19,     // ISA Register 19
        X20                     = 'd20,     // ISA Register 20
        X21                     = 'd21,     // ISA Register 21
        X22                     = 'd22,     // ISA Register 22
        X23                     = 'd23,     // ISA Register 23
        X24                     = 'd24,     // ISA Register 24
        X25                     = 'd25,     // ISA Register 25
        X26                     = 'd26,     // ISA Register 26
        X27                     = 'd27,     // ISA Register 27
        X28                     = 'd28,     // ISA Register 28
        X29                     = 'd29,     // ISA Register 29
        X30                     = 'd30,     // ISA Register 30
        X31                     = 'd31      // ISA Register 31
    } isa_reg_t;

endpackage: RISCV_ISA

`endif /* RISCV_ISA_VH_ */
