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

endmodule : l1_data_array

`default_nettype wire
