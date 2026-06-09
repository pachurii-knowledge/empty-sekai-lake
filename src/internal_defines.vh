/**
 * internal_defines.vh
 *
 * RISC-V 32-bit Processor
 *
 * ECE 18-447
 * Carnegie Mellon University
 *
 * This contains the definitions of constants and types that are used by the
 * core of the RISC-V processor, such as control signals and ALU operations.
**/

/*----------------------------------------------------------------------------*
 *  You may edit this file and add or change any files in the src directory.  *
 *----------------------------------------------------------------------------*/

`ifndef INTERNAL_DEFINES_VH_
`define INTERNAL_DEFINES_VH_

// 2nd operand immediate mode
typedef enum logic [2:0] {
    IMM_I,
    IMM_S,
    IMM_SB,
    IMM_U,
    IMM_UJ,
    IMM_DC = 3'bxxx         // Don't care value
} imm_mode_t;

// Constants that specify which operation the ALU should perform
typedef enum logic [5:0] {
    ALU_ADD,                // Addition operation
    ALU_SUB,                // Subtraction/Compare operation
    ALU_XOR,                // XOR operation
    ALU_OR,                 // OR operation
    ALU_AND,                // AND operation
    ALU_SLL,                 // Shift left operation
    ALU_SRL,                // Shift right logical operation
    ALU_SRA,                // Shift right arithmetic operation
    ALU_BEQ,                // Branch equal operation
    ALU_BNE,                // Branch not equal operation
    ALU_BLT,                // Branch less than operation
    ALU_BGE,                // Branch greater than or equal operation
    ALU_BLTU,               // Branch less than unsigned operation
    ALU_BGEU,               // Branch greater than or equal unsigned operation
    ALU_SLT,                // Set less than operation
    ALU_SLTU,               // Set less than unsigned operation
    ALU_MUL,
    ALU_MULH,
    ALU_MULHSU,
    ALU_MULHU,
    ALU_DIV,
    ALU_DIVU,
    ALU_REM,
    ALU_REMU,
    // RV64 W-form ops: operate on the low 32 bits, sign-extend bit 31 to XLEN
    ALU_ADDW,
    ALU_SUBW,
    ALU_SLLW,
    ALU_SRLW,
    ALU_SRAW,
    ALU_MULW,               // RV64M: low 32 bits of the product, sign-extended
    ALU_DIVW,               // RV64M: 32-bit signed divide, sign-extended
    ALU_DIVUW,              // RV64M: 32-bit unsigned divide, sign-extended
    ALU_REMW,               // RV64M: 32-bit signed remainder, sign-extended
    ALU_REMUW,              // RV64M: 32-bit unsigned remainder, sign-extended
    ALU_DC = 6'bxxxxxx      // Don't care value
} alu_op_t;

// Load/store partial word mode
typedef enum logic [2:0] {
    LDST_W,
    LDST_H,
    LDST_HU,
    LDST_B,
    LDST_BU,
    LDST_D,                  // RV64 doubleword (LD/SD)
    LDST_WU,                 // RV64 word unsigned (LWU)
    LDST_DC = 3'bxxx         // Don't care value
} ldst_mode_t;

// Next PC source
typedef enum logic [1:0] {
    PC_plus4,               // non-control flow
    PC_cond,                // Branch
    PC_uncond,              // JAL
    PC_indirect,            // indirect jump (JALR)
    PC_DC = 2'bxx
} pc_source_t;

// Next RD source
typedef enum logic [2:0] {
    RD_MMU,                 // from memory
    RD_CMP,                 // from compare
    RD_PC4,                 // PC + 4
    RD_ALU,                 // ALU
    RD_IMM,                 // IMM
    RD_DC = 3'bxxx
} rd_source_t;

typedef enum logic [2:0] {
    EXEC_INT,
    EXEC_CSR,
    EXEC_FENCE,
    EXEC_AMO,
    EXEC_FP,
    EXEC_DC = 3'bxxx
} exec_class_t;

typedef enum logic [2:0] {
    CSR_NONE,
    CSR_RW,
    CSR_RS,
    CSR_RC,
    CSR_RWI,
    CSR_RSI,
    CSR_RCI,
    CSR_DC = 3'bxxx
} csr_op_t;

typedef enum logic [3:0] {
    AMO_NONE,
    AMO_LR,
    AMO_SC,
    AMO_SWAP,
    AMO_ADD,
    AMO_XOR,
    AMO_AND,
    AMO_OR,
    AMO_MIN,
    AMO_MAX,
    AMO_MINU,
    AMO_MAXU,
    AMO_DC = 4'bxxxx
} amo_op_t;

typedef enum logic [5:0] {
    FP_NONE,
    FP_ADD,
    FP_SUB,
    FP_MUL,
    FP_DIV,
    FP_SQRT,
    FP_SGNJ,
    FP_SGNJN,
    FP_SGNJX,
    FP_MIN,
    FP_MAX,
    FP_CVT_W,
    FP_CVT_WU,
    FP_CVT_F_W,
    FP_CVT_F_WU,
    FP_CVT_F_F,
    FP_MV_X,
    FP_MV_F_X,
    FP_EQ,
    FP_LT,
    FP_LE,
    FP_CLASS,
    FP_MADD,
    FP_MSUB,
    FP_NMSUB,
    FP_NMADD,
    FP_DC = 6'bxxxxxx
} fp_op_t;

// Hysteresis for branch prediction
typedef enum logic [1:0] {
    PRED_NOT_TAKEN_S, // 00
    PRED_NOT_TAKEN_W, // 01 
    PRED_TAKEN_W, // 10
    PRED_TAKEN_S, // 11
    PRED_DC = 2'bxx
} hys_state_t;

/* The definition of the control signal structure, which contains all
 * microarchitectural control signals for controlling the MIPS datapath. */
typedef struct packed {
    logic useImm;           // 2nd ALU input from immediate else GPR port
    logic usePC;            // 1st ALU input from PC else GPR port
    logic doCmp;             // do a comparison opperation
    logic rfWrite;          // write GPR
    logic mem2RF;           // memory load result write to GPR
    logic pc2RF;            // PC+4 write to GPR (link)
    logic memRead;          // load instruction
    logic memWrite;         // store instruction
    imm_mode_t imm_mode;    // immediate mode applied
    alu_op_t alu_op;        // The ALU operation to perform
    ldst_mode_t ldst_mode;  // load/store partial word mode;
    pc_source_t pc_source;  // next PC source
    rd_source_t rd_source;  // next RD source
    exec_class_t exec_class;
    csr_op_t csr_op;
    amo_op_t amo_op;
    fp_op_t fp_op;
    logic fp_double;
    logic fp_uses_rs1;
    logic fp_uses_rs2;
    logic fp_uses_rs3;
    logic fp_writes_fpr;
    logic fp_writes_gpr;
    logic csr_write;
    logic fence_i;
    logic serializing;
    logic [2:0] btype;      // branch FUNCT3
    logic syscall;          // Indicates if the current instruction is an ECALL
    logic is_ebreak;        // EBREAK
    logic is_mret;          // MRET (return from machine trap)
    logic is_sret;          // SRET (return from supervisor trap)
    logic is_wfi;           // WFI (wait for interrupt)
    logic is_sfence_vma;    // SFENCE.VMA (TLB shootdown)
    logic illegal_instr;    // Indicates if the current instruction is illegal
    logic isBranch;         // Indicates if the current instruction is a branch
    // Instruction-fetch fault injected by the core (not derived from the
    // instruction bits): when set, the lane is a forced NOP that the ALU pipe
    // converts into a precise instruction page/access fault at commit. The
    // faulting virtual address (= PC) is reported in m/stval.
    logic       fetch_fault;
    logic [4:0] fetch_fault_cause; // EXC_INSTR_PAGE_FAULT or EXC_INSTR_ACCESS
} ctrl_signals_t;

`endif /* INTERNAL_DEFINES_VH_ */