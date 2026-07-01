/**
 * l2_plru.sv
 *
 * Combinational tree-PLRU helper for an 8-way set (7 state bits per set), for
 * the shared write-back L2 (plans/multicore-ccd.md §2). The L2 stores one 7-bit
 * state per set in flops and feeds it here to obtain the victim way and the
 * post-access next state. This is the 8-way analogue of l1_plru.sv (which is
 * hardcoded to a 3-bit / 4-way tree and is not reusable at this width).
 *
 * Binary tree, root = state[0]; each bit points toward the subtree to replace
 * next, so the victim is read by walking root -> leaf:
 *                       state[0]                 -> victim way[2] (half {0-3}/{4-7})
 *                 0 /            \ 1
 *             state[1]          state[2]         -> victim way[1]
 *            0 /    \ 1        0 /    \ 1
 *       state[3]  state[4]  state[5]  state[6]   -> victim way[0]
 *        w0 w1     w2 w3     w4 w5     w6 w7
 *   leaf node index = 3 + {way[2], way[1]}  (nodes 3..6)
 *
 * On an access to way A the three bits on A's root->leaf path are flipped to
 * point away from A. Invalid ways are always victimised first (lowest-numbered
 * invalid way wins), matching l1_plru.
 */

`default_nettype none

module l2_plru #(
    parameter int WAYS = 8
) (
    input  wire logic [6:0]               state,        // current tree bits
    input  wire logic [WAYS-1:0]          valid,        // per-way valid mask
    output logic [$clog2(WAYS)-1:0]       victim,
    input  wire logic                     update_en,
    input  wire logic [$clog2(WAYS)-1:0]  access_way,
    output logic [6:0]                    next_state
);

    // Victim selection: follow the tree pointers, then invalid-first override.
    // Leaf node index = 3 + {vb2,vb1} (nodes 3..6), spelled as an explicit case.
    always_comb begin
        automatic logic vb2, vb1, vb0;
        vb2 = state[0];
        vb1 = vb2 ? state[2] : state[1];
        unique case ({vb2, vb1})
            2'b00: vb0 = state[3];
            2'b01: vb0 = state[4];
            2'b10: vb0 = state[5];
            2'b11: vb0 = state[6];
        endcase
        victim = {vb2, vb1, vb0};
        // Lowest-numbered invalid way wins (iterate high->low, last write sticks).
        for (int i = WAYS-1; i >= 0; i -= 1) begin
            if (!valid[i]) victim = i[$clog2(WAYS)-1:0];
        end
    end

    // On access to access_way, flip the bits along its root->leaf path away from it.
    always_comb begin
        next_state = state;
        if (update_en) begin
            next_state[0] = ~access_way[2];
            if (access_way[2]) next_state[2] = ~access_way[1];
            else               next_state[1] = ~access_way[1];
            unique case ({access_way[2], access_way[1]})
                2'b00: next_state[3] = ~access_way[0];
                2'b01: next_state[4] = ~access_way[0];
                2'b10: next_state[5] = ~access_way[0];
                2'b11: next_state[6] = ~access_way[0];
            endcase
        end
    end

`ifndef SYNTHESIS
    // This tree is fixed 8-way (7 state bits); a 4-way L1 uses l1_plru instead.
    initial if (WAYS != 8)
        $fatal(1, "l2_plru: WAYS(%0d) != 8 -- this tree-PLRU is fixed 8-way (7 state bits)", WAYS);
`endif

endmodule : l2_plru

`default_nettype wire
