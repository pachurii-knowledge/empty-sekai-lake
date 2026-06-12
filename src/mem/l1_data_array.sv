/**
 * l1_data_array.sv
 *
 * Sync-read, single-write SRAM array of cache line data, shared by L1I and
 * L1D (DD-10: sync-read from day one so it infers BRAM on the FPGA). One read
 * port (presents an index, returns all WAYS lines registered one cycle later)
 * and one byte-masked write port (one way per access). Hit latency is 2
 * cycles: present index, compare/mux the registered output next cycle.
 */

`default_nettype none

module l1_data_array #(
    parameter int SETS      = 64,
    parameter int WAYS      = 4,
    parameter int LINE_BITS = 512
) (
    input  logic clk,
    // Read port: registered output (sync read).
    input  logic                         ren,
    input  logic [$clog2(SETS)-1:0]      ridx,
    output logic [WAYS-1:0][LINE_BITS-1:0] rdata,
    // Write port: one way, byte-masked.
    input  logic                         wen,
    input  logic [$clog2(SETS)-1:0]      widx,
    input  logic [$clog2(WAYS)-1:0]      wway,
    input  logic [LINE_BITS-1:0]         wdata,
    input  logic [LINE_BITS/8-1:0]       wmask     // per-byte enable
);

    localparam int BYTES = LINE_BITS/8;

    logic [LINE_BITS-1:0] mem [SETS][WAYS];

    always_ff @(posedge clk) begin
        if (ren) begin
            for (int w = 0; w < WAYS; w += 1) begin
                rdata[w] <= mem[ridx][w];
            end
        end
    end

    always_ff @(posedge clk) begin
        if (wen) begin
            for (int b = 0; b < BYTES; b += 1) begin
                if (wmask[b]) begin
                    mem[widx][wway][b*8 +: 8] <= wdata[b*8 +: 8];
                end
            end
        end
    end

endmodule : l1_data_array

`default_nettype wire
