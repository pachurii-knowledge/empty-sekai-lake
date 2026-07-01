// Blackbox stub for the FakeRAM-generated single-port SRAM niigo_sram_64x8
// (64 words x 8 bits, 1RW). Real timing = niigo_sram_64x8.lib, phys = .lef.
// Used only in the ASAP7 macro-mapped synth path (NIIGO_SRAM_MACRO); the
// Verilator/functional build uses the inferred array in l1_data_array.sv.
`ifndef NIIGO_SRAM_64X8_BB
`define NIIGO_SRAM_64X8_BB
(* blackbox *)
module niigo_sram_64x8 (
   output [7:0] rd_out,
   input  [5:0] addr_in,
   input        we_in,
   input  [7:0] wd_in,
   input        clk,
   input        ce_in
);
endmodule
`endif
