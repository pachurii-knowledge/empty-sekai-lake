/**
 * riscv_core.sv
 *
 * Phase 2 wrapper. The known-good Phase 1 scalar implementation is preserved in
 * riscv_core_scalar. Defining SUPERSCALAR_4WIDE selects the conservative 4-wide
 * in-order implementation.
 */

`include "riscv_uarch.vh"

`default_nettype none

module riscv_core (
    input  logic             clk, rst_l, instr_mem_excpt, data_mem_excpt,
    input  logic [3:0][31:0] instr, data_load,
    input  logic [29:0]      data_load_addr,
    input  logic             data_load_valid,
    output logic             data_load_en, halted,
    output logic [ 3:0]      data_store_mask,
    output logic [29:0]      instr_addr, data_addr,
    output logic             instr_stall, data_stall,
    output logic [31:0]      data_store
);

    generate
        if (RISCV_UArch::SUPERSCALAR_WAYS == 4) begin : gen_4wide
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
                .data_store
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
                .data_store
            );
        end
    endgenerate

endmodule: riscv_core
