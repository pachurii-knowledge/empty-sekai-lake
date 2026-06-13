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
    parameter int TAG_BITS = 20
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

    genvar w;
    generate
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
    endgenerate

endmodule : l1_tag_array

`default_nettype wire
