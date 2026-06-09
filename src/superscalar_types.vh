`ifndef SUPERSCALAR_TYPES_VH_
`define SUPERSCALAR_TYPES_VH_

`include "internal_defines.vh"
`include "riscv_isa.vh"

typedef struct packed {
    logic        valid;
    logic        kill;
    logic [RISCV_ISA::XLEN-1:0] pc;
    logic [31:0] instr;
} fetch_lane_t;

typedef struct packed {
    logic          valid;
    logic          kill;
    logic [RISCV_ISA::XLEN-1:0]   pc;
    logic [31:0]   instr;
    ctrl_signals_t ctrl;
    logic [4:0]    rs1;
    logic [4:0]    rs2;
    logic [4:0]    rd;
    logic [RISCV_ISA::XLEN-1:0]   imm;
    logic          uses_rs1;
    logic          uses_rs2;
} decode_lane_t;

typedef struct packed {
    logic          valid;
    logic          kill;
    logic [RISCV_ISA::XLEN-1:0]   pc;
    logic [31:0]   instr;
    ctrl_signals_t ctrl;
    logic [4:0]    rs1;
    logic [4:0]    rs2;
    logic [4:0]    rd;
    logic [RISCV_ISA::XLEN-1:0]   rs1_data;
    logic [RISCV_ISA::XLEN-1:0]   rs2_data;
    logic [RISCV_ISA::XLEN-1:0]   imm;
} execute_lane_t;

typedef struct packed {
    logic        valid;
    logic        kill;
    logic [RISCV_ISA::XLEN-1:0] pc;
    logic [4:0]  rd;
    logic        rd_we;
    logic [RISCV_ISA::XLEN-1:0] rd_data;
    logic        halted;
    logic        exception;
} writeback_lane_t;

`endif /* SUPERSCALAR_TYPES_VH_ */
