// Blackbox stub for FakeRAM single-port SRAM niigo_sram_64x52 (64 words x 52 bits,
// 1RW) -- the RV64 L1 tag (L1_TAG_BITS=52). Synth-only (NIIGO_SRAM_MACRO); the
// Verilator build uses the inferred tag array.
`ifndef NIIGO_SRAM_64X52_BB
`define NIIGO_SRAM_64X52_BB
(* blackbox *)
module niigo_sram_64x52 (
   output [51:0] rd_out,
   input  [5:0]  addr_in,
   input         we_in,
   input  [51:0] wd_in,
   input         clk,
   input         ce_in
);
endmodule
`endif
