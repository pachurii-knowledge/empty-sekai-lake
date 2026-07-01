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
 * A second, independent read port (ren2/ridx2 -> rtag2) is the coherence snoop
 * port (phase C4): the L1I uses it to look up store-snoop lines without ever
 * stalling the fetch read port. With one write port and two read ports Vivado
 * replicates the (tiny) tag memory per read port -- a "duplicate-tags" snoop
 * filter, the textbook structure -- each copy still a 1W1R BRAM. Consumers that
 * don't snoop tie ren2 = 0 (the replicated port is then optimized away).
 */

`default_nettype none

module l1_tag_array #(
    parameter int SETS     = 64,
    parameter int WAYS     = 4,
    parameter int TAG_BITS = 20,
    // SNOOP_PORT=1 (default, L1I): the 2nd read port (ren2) is live -> the
    // ASAP7 single-port SRAM map keeps the inferred dup-tags here, because a
    // fetch-read + snoop-read can be concurrent and a 1RW macro can't serve both
    // (that needs snoop arbitration -- deferred). SNOOP_PORT=0 (L1D): ren2 is
    // tied off, so under NIIGO_SRAM_MACRO this maps to a clean 1RW SRAM per way.
    parameter bit SNOOP_PORT = 1
) (
    input wire logic clk,
    input wire logic                       ren,
    input wire logic [$clog2(SETS)-1:0]    ridx,
    output logic [WAYS-1:0][TAG_BITS-1:0] rtag,
    // Second read port (C4 snoop): independent index, registered one cycle.
    input wire logic                       ren2,
    input wire logic [$clog2(SETS)-1:0]    ridx2,
    output logic [WAYS-1:0][TAG_BITS-1:0] rtag2,
    input wire logic                       wen,
    input wire logic [$clog2(SETS)-1:0]    widx,
    input wire logic [$clog2(WAYS)-1:0]    wway,
    input wire logic [TAG_BITS-1:0]        wtag
);

    // Map to a single-port SRAM only for the no-snoop (L1D) instance under the
    // ASAP7 macro build; L1I (SNOOP_PORT=1) and all functional builds infer.
`ifdef NIIGO_SRAM_MACRO
    localparam bit USE_SRAM_TAG = (SNOOP_PORT == 0);
`else
    localparam bit USE_SRAM_TAG = 1'b0;
`endif

    genvar w;
    generate
        if (USE_SRAM_TAG) begin : g_sram
            // L1D tag: read (S_IDLE/probe) and write (store-hit/S_INSTALL) are
            // disjoint cycles, so the one macro address port is muxed. Requires
            // TAG_BITS==52 (RV64 L1 tag) to match niigo_sram_64x52.
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
            assign rtag2 = '0;   // snoop port unused when SNOOP_PORT=0
        end else begin : g_infer
            for (w = 0; w < WAYS; w += 1) begin : ways
                logic [TAG_BITS-1:0] tags [SETS];
                always_ff @(posedge clk) begin
                    if (ren) rtag[w] <= tags[ridx];
                end
                always_ff @(posedge clk) begin
                    if (ren2) rtag2[w] <= tags[ridx2];
                end
                always_ff @(posedge clk) begin
                    if (wen && (wway == w[$clog2(WAYS)-1:0])) tags[widx] <= wtag;
                end
            end
        end
    endgenerate

endmodule : l1_tag_array

`default_nettype wire
