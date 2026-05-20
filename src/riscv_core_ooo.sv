/**
 * Phase 3 selectable OoO core shell.
 *
 * The Phase 3 structures are developed as standalone modules first. Until the
 * final integration path replaces this compatibility core, the selectable OoO
 * build delegates architectural execution to the verified Phase 2 4-wide core so
 * regression behavior remains stable while the new structures compile together.
 */

`include "ooo_types.vh"

`default_nettype none

module riscv_core_ooo (
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

    riscv_core_4wide CompatibilityCore (
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

endmodule: riscv_core_ooo
