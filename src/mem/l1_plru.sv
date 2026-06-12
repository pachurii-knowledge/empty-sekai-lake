/**
 * l1_plru.sv
 *
 * Combinational tree-PLRU helper for a 4-way set (3 state bits per set),
 * shared by L1I and L1D. The cache stores one 3-bit state per set in flops and
 * feeds it here to obtain the victim way and the post-access next state.
 *
 * Tree bits point toward the subtree to replace next:
 *   b0  selects the half (0 -> ways {0,1}, 1 -> ways {2,3})
 *   b1  selects within the low half  (victim way {0,1})
 *   b2  selects within the high half (victim way {2,3})
 * On an access to way A the bits are flipped to point away from A.
 * Invalid ways are always victimised first (lowest-numbered invalid way).
 */

`default_nettype none

module l1_plru #(
    parameter int WAYS = 4
) (
    input  logic [2:0]               state,        // current tree bits
    input  logic [WAYS-1:0]          valid,        // per-way valid mask
    output logic [$clog2(WAYS)-1:0]  victim,
    input  logic                     update_en,
    input  logic [$clog2(WAYS)-1:0]  access_way,
    output logic [2:0]               next_state
);

    // Victim selection: invalid-first, else follow the tree.
    logic [1:0] plru_victim;
    always_comb begin
        plru_victim[1] = state[0];
        plru_victim[0] = state[0] ? state[2] : state[1];
        victim = plru_victim;
        // Lowest-numbered invalid way wins (iterate high->low, last write sticks).
        for (int i = WAYS-1; i >= 0; i -= 1) begin
            if (!valid[i]) victim = i[$clog2(WAYS)-1:0];
        end
    end

    // On access to access_way, flip the bits along its path away from it.
    always_comb begin
        next_state = state;
        if (update_en) begin
            next_state[0] = ~access_way[1];
            if (access_way[1]) next_state[2] = ~access_way[0];
            else               next_state[1] = ~access_way[0];
        end
    end

endmodule : l1_plru

`default_nettype wire
