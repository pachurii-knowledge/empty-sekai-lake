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

    // Single-port SRAM macro mapping (ASAP7). Each (way, byte-lane) is a
    // niigo_sram_64x8 (64 words x 8 bits, 1RW; FakeRAM-generated, LIB/LEF under
    // fpga/openroad/sram/). Both L1 controllers drive READ and WRITE in DISJOINT
    // cycles, so the single shared address port is muxed (write index when being
    // written, else read index). Logically identical to the inferred array below,
    // which the Verilator/functional build uses. Geometry-guarded exactly like
    // l1_tag_array: only the fixed 64x512 L1 geometry maps to the macro; other
    // geometries (RV32 L1, the L2) stay inferred even under the macro build.
`ifdef NIIGO_SRAM_MACRO
    localparam bit USE_SRAM_DATA = (SETS == 64) && (LINE_BITS == 512);
`else
    localparam bit USE_SRAM_DATA = 1'b0;
`endif

    genvar w, b;
    generate
        if (USE_SRAM_DATA) begin : g_sram
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
        end else begin : g_infer
            for (w = 0; w < WAYS; w += 1) begin : ways
                logic [LINE_BITS-1:0] mem [SETS];
                always_ff @(posedge clk) begin
                    if (ren) rdata[w] <= mem[ridx];
                end
                always_ff @(posedge clk) begin
                    if (wen && (wway == w[$clog2(WAYS)-1:0])) begin
                        for (int bb = 0; bb < BYTES; bb += 1) begin
                            if (wmask[bb]) mem[widx][bb*8 +: 8] <= wdata[bb*8 +: 8];
                        end
                    end
                end
            end
        end
    endgenerate

endmodule : l1_data_array

`default_nettype wire
