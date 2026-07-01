/**
 * l1_data_array.sv
 *
 * Sync-read, single-write SRAM array of cache line data, shared by L1I and
 * L1D. Each way is a separate single-read / single-write memory so it maps to
 * distributed/block RAM on FPGA (DD-10) rather than flip-flops: a 2-D
 * mem[SETS][WAYS] with a per-way parallel read infers a multi-read-port array,
 * which Vivado implements in registers. One read port (presents an index,
 * returns all WAYS lines registered one cycle later) and one byte-masked write
 * port (one way per access). Hit latency is 2 cycles.
 */

`default_nettype none

module l1_data_array #(
    parameter int SETS      = 64,
    parameter int WAYS      = 4,
    parameter int LINE_BITS = 512
) (
    input wire logic clk,
    // Read port: registered output (sync read).
    input wire logic                         ren,
    input wire logic [$clog2(SETS)-1:0]      ridx,
    output logic [WAYS-1:0][LINE_BITS-1:0] rdata,
    // Write port: one way, byte-masked.
    input wire logic                         wen,
    input wire logic [$clog2(SETS)-1:0]      widx,
    input wire logic [$clog2(WAYS)-1:0]      wway,
    input wire logic [LINE_BITS-1:0]         wdata,
    input wire logic [LINE_BITS/8-1:0]       wmask     // per-byte enable
);

    localparam int BYTES = LINE_BITS/8;

`ifdef NIIGO_SRAM_MACRO
    // ------------------------------------------------------------------ ASAP7
    // Single-port SRAM macro mapping. Each (way, byte-lane) is a niigo_sram_64x8
    // (64 words x 8 bits, 1RW; FakeRAM-generated, LIB/LEF under fpga/openroad/
    // sram/). Both L1 controllers drive READ and WRITE in DISJOINT cycles
    // (l1_icache: S_SERVE/S_REPLAY read vs S_INSTALL write; l1_dcache: S_IDLE/
    // S_FLUSH_READ read vs store-hit/S_INSTALL write), so the single shared
    // address port is muxed (this lane's write index when it is being written,
    // else the read index). Byte writes map to per-lane we_in (wmask[b]); a read
    // presents all WAYS*BYTES lanes at ridx. Real SRAM holds rd_out between
    // accesses, matching the inferred array's registered read. Logically
    // identical to the inferred array below, which the Verilator/functional
    // build (no NIIGO_SRAM_MACRO) uses. Requires SETS==64, LINE_BITS==512
    // (the L1 geometry; the niigo_sram_64x8 depth/width are fixed to match).
    genvar w, b;
    generate
        for (w = 0; w < WAYS; w += 1) begin : ways
            for (b = 0; b < BYTES; b += 1) begin : lanes
                wire lane_wr = wen && (wway == w[$clog2(WAYS)-1:0]) && wmask[b];
                niigo_sram_64x8 u_lane (
                    .clk    (clk),
                    .ce_in  (ren || lane_wr),
                    .we_in  (lane_wr),
                    .addr_in(lane_wr ? widx : ridx),
                    .wd_in  (wdata[b*8 +: 8]),
                    .rd_out (rdata[w][b*8 +: 8])
                );
            end
        end
    endgenerate
`else
    genvar w;
    generate
        for (w = 0; w < WAYS; w += 1) begin : ways
            logic [LINE_BITS-1:0] mem [SETS];
            always_ff @(posedge clk) begin
                if (ren) rdata[w] <= mem[ridx];
            end
            always_ff @(posedge clk) begin
                if (wen && (wway == w[$clog2(WAYS)-1:0])) begin
                    for (int b = 0; b < BYTES; b += 1) begin
                        if (wmask[b]) mem[widx][b*8 +: 8] <= wdata[b*8 +: 8];
                    end
                end
            end
        end
    endgenerate
`endif

endmodule : l1_data_array

`default_nettype wire
