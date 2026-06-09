/**
 * riscv_core.sv
 *
 * Phase 2 wrapper. The known-good Phase 1 scalar implementation is preserved in
 * riscv_core_scalar. Defining SUPERSCALAR_4WIDE selects the conservative 4-wide
 * in-order implementation.
 */

`include "riscv_isa.vh"
`include "riscv_uarch.vh"

`default_nettype none

module riscv_core
    import RISCV_ISA::XLEN, RISCV_ISA::XLEN_BYTES;
    import RISCV_UArch::MEMORY_READ_WIDTH, RISCV_UArch::MEMORY_ADDR_WIDTH;
(
    input  logic             clk, rst_l, instr_mem_excpt, data_mem_excpt,
    input  logic [MEMORY_READ_WIDTH-1:0][XLEN-1:0] instr, data_load,
    input  logic [MEMORY_ADDR_WIDTH-1:0] data_load_addr,
    input  logic             data_load_valid,
    output logic             data_load_en, halted,
    output logic [XLEN_BYTES-1:0]        data_store_mask,
    output logic [MEMORY_ADDR_WIDTH-1:0] instr_addr, data_addr,
    output logic             instr_stall, data_stall,
    output logic [XLEN-1:0]  data_store,
    // MMU page-table-walk port
    output logic [MEMORY_ADDR_WIDTH-1:0] ptw_addr,
    output logic             ptw_we,
    output logic [XLEN-1:0]  ptw_wdata,
    input  logic [XLEN-1:0]  ptw_rdata
);

    generate
        if (RISCV_UArch::OOO_ENABLED != 0) begin : gen_ooo
            riscv_core_ooo OoOCore (
                .clk,
                .rst_l,
                .instr_mem_excpt,
                .data_mem_excpt,
                .instr,
                .data_load,
                .data_load_addr,
                .data_load_valid,
                .data_load_en,
                .halted,
                .data_store_mask,
                .instr_addr,
                .data_addr,
                .instr_stall,
                .data_stall,
                .data_store,
                .ptw_addr,
                .ptw_we,
                .ptw_wdata,
                .ptw_rdata
            );
        end else if (RISCV_UArch::SUPERSCALAR_WAYS == 4) begin : gen_4wide
            riscv_core_4wide Core4Wide (
                .clk,
                .rst_l,
                .instr_mem_excpt,
                .data_mem_excpt,
                .instr,
                .data_load,
                .data_load_addr,
                .data_load_valid,
                .data_load_en,
                .halted,
                .data_store_mask,
                .instr_addr,
                .data_addr,
                .instr_stall,
                .data_stall,
                .data_store,
                .ptw_addr,
                .ptw_we,
                .ptw_wdata,
                .ptw_rdata
            );
        end else begin : gen_scalar
            riscv_core_scalar ScalarCore (
                .clk,
                .rst_l,
                .instr_mem_excpt,
                .data_mem_excpt,
                .instr,
                .data_load,
                .data_load_addr,
                .data_load_valid,
                .data_load_en,
                .halted,
                .data_store_mask,
                .instr_addr,
                .data_addr,
                .instr_stall,
                .data_stall,
                .data_store,
                .ptw_addr,
                .ptw_we,
                .ptw_wdata,
                .ptw_rdata
            );
        end
    endgenerate

endmodule: riscv_core
