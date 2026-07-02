/**
 * l1_tag_array.sv
 *
 * Sync-read, single-write tag SRAM, shared by L1I and L1D. Holds only the
 * address tag per (set, way); valid and dirty bits live in flops inside the
 * cache so they can be flash-cleared (fence.i invalidate / halt flush) in one
 * cycle without walking the array. Each way is a separate single-port memory
 * (one read addr, one write addr) so it infers RAM rather than registers on
 * FPGA; the read returns all WAYS tags for the index, registered one cycle
 * later (aligns with l1_data_array).
 *
 * ONE read/write port per way (true 1RW). Read (fetch/replay/probe) and write
 * (install/store-hit) are disjoint cycles in both caches, so the single macro
 * address port is muxed. The L1I coherence snoop no longer needs a 2nd read
 * port: it interleaves onto this one port (see l1_icache.sv), so under the
 * ASAP7 macro build BOTH the L1I and L1D tag arrays map to the niigo_sram_64x52
 * single-port SRAM.
 */

`default_nettype none

module l1_tag_array #(
    parameter int SETS     = 64,
    parameter int WAYS     = 4,
    parameter int TAG_BITS = 20
) (
    input wire logic clk,
    input wire logic                       ren,
    input wire logic [$clog2(SETS)-1:0]    ridx,
    output logic [WAYS-1:0][TAG_BITS-1:0] rtag,
    input wire logic                       wen,
    input wire logic [$clog2(SETS)-1:0]    widx,
    input wire logic [$clog2(WAYS)-1:0]    wway,
    input wire logic [TAG_BITS-1:0]        wtag
);

    // Single-port SRAM macro under the ASAP7 macro build; inferred RAM otherwise
    // (functional/Verilator). Read (S_IDLE/S_SERVE/S_REPLAY/probe) and write
    // (store-hit/S_INSTALL) are disjoint cycles, so the one macro address port is
    // muxed. Guarded on the geometry the macro actually is (niigo_sram_64x52 =
    // 64 sets x 52-bit RV64 L1 tag); other geometries (RV32 L1, the L2) stay
    // inferred even under the macro build.
`ifdef NIIGO_SRAM_MACRO
    localparam bit USE_SRAM_TAG = (SETS == 64) && (TAG_BITS == 52);
`else
    localparam bit USE_SRAM_TAG = 1'b0;
`endif

    genvar w;
    generate
        if (USE_SRAM_TAG) begin : g_sram
            for (w = 0; w < WAYS; w += 1) begin : ways
                wire tag_wr = wen && (wway == w[$clog2(WAYS)-1:0]);
                niigo_sram_64x52 u_tag (
                    .clk    (clk),
                    .ce_in  (ren || tag_wr),
                    .we_in  (tag_wr),
                    .addr_in(tag_wr ? widx : ridx),
                    .wd_in  (wtag),
                    .rd_out (rtag[w])
                );
            end
        end else begin : g_infer
            for (w = 0; w < WAYS; w += 1) begin : ways
                logic [TAG_BITS-1:0] tags [SETS];
                always_ff @(posedge clk) begin
                    if (ren) rtag[w] <= tags[ridx];
                end
                always_ff @(posedge clk) begin
                    if (wen && (wway == w[$clog2(WAYS)-1:0])) tags[widx] <= wtag;
                end
            end
        end
    endgenerate

endmodule : l1_tag_array

`default_nettype wire
