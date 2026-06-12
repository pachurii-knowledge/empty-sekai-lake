/**
 * l1_tag_array.sv
 *
 * Sync-read, single-write tag SRAM, shared by L1I and L1D. Holds only the
 * address tag per (set, way); valid and dirty bits live in flops inside the
 * cache so they can be flash-cleared (fence.i invalidate / halt flush) in one
 * cycle without walking the array. Read returns all WAYS tags for the index,
 * registered one cycle later (aligns with l1_data_array).
 */

`default_nettype none

module l1_tag_array #(
    parameter int SETS     = 64,
    parameter int WAYS     = 4,
    parameter int TAG_BITS = 20
) (
    input  logic clk,
    input  logic                       ren,
    input  logic [$clog2(SETS)-1:0]    ridx,
    output logic [WAYS-1:0][TAG_BITS-1:0] rtag,
    input  logic                       wen,
    input  logic [$clog2(SETS)-1:0]    widx,
    input  logic [$clog2(WAYS)-1:0]    wway,
    input  logic [TAG_BITS-1:0]        wtag
);

    logic [TAG_BITS-1:0] tags [SETS][WAYS];

    always_ff @(posedge clk) begin
        if (ren) begin
            for (int w = 0; w < WAYS; w += 1) begin
                rtag[w] <= tags[ridx][w];
            end
        end
    end

    always_ff @(posedge clk) begin
        if (wen) begin
            tags[widx][wway] <= wtag;
        end
    end

endmodule : l1_tag_array

`default_nettype wire
