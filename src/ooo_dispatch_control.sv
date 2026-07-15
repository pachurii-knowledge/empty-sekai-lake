`include "ooo_types.vh"

`default_nettype none

module ooo_dispatch_control
    import OOO_Types::*;
(
    input wire logic [OOO_WIDTH-1:0] lane_valid,
    input wire logic [OOO_WIDTH-1:0] lane_has_dest,
    input wire logic [OOO_WIDTH-1:0] lane_is_branch,
`ifdef JAL_NO_CKPT
    // Lanes that actually allocate a branch checkpoint (= lane_is_branch minus JAL,
    // which takes no checkpoint under JAL_NO_CKPT). Only these are subject to the
    // full-branch-stack dispatch stall; a JAL may dispatch into a full stack.
    input wire logic [OOO_WIDTH-1:0] lane_needs_ckpt,
`endif
    input wire logic [OOO_WIDTH-1:0] lane_is_memory,
    input wire logic [OOO_WIDTH-1:0] lane_is_terminal,
    input wire logic [OOO_WIDTH-1:0] lane_is_serializing,
    // P5b FP de-serialization (tied 0 unless -DFP_OOO drives them): lane_is_fp
    // marks any lane that touches an FPR, lane_fp_src_busy marks a lane whose FP
    // source (or WAW dest) FPR is still busy. Both are inert when 0.
    input wire logic [OOO_WIDTH-1:0] lane_is_fp,
    input wire logic [OOO_WIDTH-1:0] lane_fp_src_busy,
    input wire logic                 active_list_full,
    input wire logic                 int_iq_full,
    input wire logic                 mem_queue_full,
    input wire logic                 branch_stack_full,
    input wire logic                 free_list_can_allocate,
    input wire logic [$clog2(PHYS_REGS+1)-1:0] free_list_available,
    input wire logic                 suppress_dispatch,
    output logic [OOO_WIDTH-1:0] dispatch_valid,
    output logic                 dispatch_stall
);

    logic stop_prefix;
    logic branch_seen;
    logic memory_seen;
    logic fp_seen;
    logic prefix_dispatched;
    logic [$clog2(OOO_WIDTH+1)-1:0] dest_seen;

    always_comb begin
        dispatch_valid = '0;
        stop_prefix = 1'b0;
        branch_seen = 1'b0;
        memory_seen = 1'b0;
        fp_seen = 1'b0;
        prefix_dispatched = 1'b0;
        dest_seen = '0;
        dispatch_stall = suppress_dispatch || active_list_full || int_iq_full ||
            mem_queue_full;

        for (int i = 0; i < OOO_WIDTH; i += 1) begin
            if (lane_is_branch[i] && branch_seen) begin
                stop_prefix = 1'b1;
            end
            if (lane_is_memory[i] && memory_seen) begin
                stop_prefix = 1'b1;
            end
            // P5b: at most one FP op per group (avoids the intra-group by-value
            // FPR hazard, since FP operands are read by value at dispatch), and
            // an FP op holds here while a source/WAW FPR is still busy.
            if (lane_is_fp[i] && fp_seen) begin
                stop_prefix = 1'b1;
            end
            if (lane_fp_src_busy[i]) begin
                stop_prefix = 1'b1;
            end
            if (lane_valid[i] && lane_has_dest[i] &&
                    (dest_seen >= free_list_available)) begin
                stop_prefix = 1'b1;
            end
            if (i != 0 && lane_is_terminal[i - 1]) begin
                stop_prefix = 1'b1;
            end
            if (lane_is_terminal[i] && prefix_dispatched) begin
                stop_prefix = 1'b1;
            end
            if (i != 0 && lane_is_serializing[i - 1]) begin
                stop_prefix = 1'b1;
            end
`ifdef JAL_NO_CKPT
            if (lane_needs_ckpt[i] && branch_stack_full) begin
`else
            if (lane_is_branch[i] && branch_stack_full) begin
`endif
                stop_prefix = 1'b1;
                if (!prefix_dispatched) begin
                    dispatch_stall = 1'b1;
                end
            end
            if (lane_is_serializing[i] && prefix_dispatched) begin
                stop_prefix = 1'b1;
            end

            dispatch_valid[i] = lane_valid[i] && !dispatch_stall && !stop_prefix;
            prefix_dispatched |= dispatch_valid[i];

            if (dispatch_valid[i] && lane_is_branch[i]) begin
                branch_seen = 1'b1;
                stop_prefix = 1'b1;
            end
            if (dispatch_valid[i] && lane_is_memory[i]) begin
                memory_seen = 1'b1;
            end
            if (dispatch_valid[i] && lane_is_fp[i]) begin
                fp_seen = 1'b1;
            end
            if (dispatch_valid[i] && lane_has_dest[i]) begin
                dest_seen += 1'b1;
            end
        end
    end

endmodule: ooo_dispatch_control
