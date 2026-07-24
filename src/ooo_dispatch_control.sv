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
`ifdef FUSE_BRANCH
    // Fuse-master lanes whose folded slave is a branch (shared-infra §4b): the
    // atomic-pair stall keeps master+branch-slave dispatching together or not
    // at all (a split pair would double-execute the folded op).
    input wire logic [OOO_WIDTH-1:0] lane_fuse_pre_branch,
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
`ifdef DISPATCH_STATS
    // Instrumentation only -- combinational shadows of the ladder below, driven at
    // the exact point each condition fires. Nothing here feeds the datapath.
    output logic                 dstat_cut_valid,    // stop_prefix went 0->1 this cycle
    output logic [3:0]           dstat_cut_reason,   // DCUT_* -- why the group was cut
    output logic [$clog2(OOO_WIDTH+1)-1:0] dstat_cut_idx, // lane at which it was cut
    output logic [3:0]           dstat_stall_reason, // DSTL_* -- why dispatch_stall
`endif
    output logic                 dispatch_stall
);

    logic stop_prefix;
    logic branch_seen;
    logic memory_seen;
    logic fp_seen;
    logic prefix_dispatched;
    logic [$clog2(OOO_WIDTH+1)-1:0] dest_seen;
`ifdef DISPATCH_STATS
    logic bstack_first_cut;   // stats only: was the bstack arm itself the first cut?
`endif

`ifdef DISPATCH_STATS
    // Record the FIRST condition that sets stop_prefix. Guarded on !stop_prefix so
    // later conditions in the same cycle cannot overwrite the true cause. Placed
    // before each `stop_prefix = 1'b1` so it observes the pre-assignment value.
    `define DSTAT_CUT(code) \
        if (!stop_prefix) begin \
            dstat_cut_valid = 1'b1; \
            dstat_cut_reason = (code); \
            dstat_cut_idx = ($clog2(OOO_WIDTH+1))'(i); \
        end
`else
    `define DSTAT_CUT(code)
`endif

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
`ifdef DISPATCH_STATS
        dstat_cut_valid  = 1'b0;
        dstat_cut_reason = DCUT_NONE;
        dstat_cut_idx    = '0;
        bstack_first_cut = 1'b0;
        // Structural attribution of the line above, in its OR order. The core
        // decomposes DSTL_SUPPRESS further (it owns the 11 suppress_dispatch terms).
        dstat_stall_reason = suppress_dispatch ? DSTL_SUPPRESS  :
                             active_list_full  ? DSTL_ROB_FULL  :
                             int_iq_full       ? DSTL_IQ_FULL   :
                             mem_queue_full    ? DSTL_MEMQ_FULL : DSTL_NONE;
`endif

        for (int i = 0; i < OOO_WIDTH; i += 1) begin
            if (lane_is_branch[i] && branch_seen) begin
                `DSTAT_CUT(DCUT_2ND_BR)
                stop_prefix = 1'b1;
            end
            if (lane_is_memory[i] && memory_seen) begin
                `DSTAT_CUT(DCUT_2ND_MEM)
                stop_prefix = 1'b1;
            end
            // P5b: at most one FP op per group (avoids the intra-group by-value
            // FPR hazard, since FP operands are read by value at dispatch), and
            // an FP op holds here while a source/WAW FPR is still busy.
            if (lane_is_fp[i] && fp_seen) begin
                `DSTAT_CUT(DCUT_2ND_FP)
                stop_prefix = 1'b1;
            end
            if (lane_fp_src_busy[i]) begin
                `DSTAT_CUT(DCUT_FP_BUSY)
                stop_prefix = 1'b1;
            end
            if (lane_valid[i] && lane_has_dest[i] &&
                    (dest_seen >= free_list_available)) begin
                `DSTAT_CUT(DCUT_FREELIST)
                stop_prefix = 1'b1;
            end
            if (i != 0 && lane_is_terminal[i - 1]) begin
                `DSTAT_CUT(DCUT_TERM_PREV)
                stop_prefix = 1'b1;
            end
            if (lane_is_terminal[i] && prefix_dispatched) begin
                `DSTAT_CUT(DCUT_TERM_CUR)
                stop_prefix = 1'b1;
            end
            if (i != 0 && lane_is_serializing[i - 1]) begin
                `DSTAT_CUT(DCUT_SER_PREV)
                stop_prefix = 1'b1;
            end
`ifdef JAL_NO_CKPT
            if (lane_needs_ckpt[i] && branch_stack_full) begin
`else
            if (lane_is_branch[i] && branch_stack_full) begin
`endif
`ifdef DISPATCH_STATS
                bstack_first_cut = !stop_prefix;
`endif
                `DSTAT_CUT(DCUT_BSTACK)
                stop_prefix = 1'b1;
                if (!prefix_dispatched) begin
                    dispatch_stall = 1'b1;
`ifdef DISPATCH_STATS
                    // This arm has NO !stop_prefix guard, so it also fires on a lane
                    // that was already dead from an earlier cut (e.g. lane0 held by
                    // lane_fp_src_busy). In that case the branch stack was irrelevant
                    // -- the lane could not have dispatched anyway -- so only claim
                    // the stall when this arm is itself the first cut, else leave
                    // DSTL_NONE and let the core credit dstat_cut_reason.
                    if (dstat_stall_reason == DSTL_NONE && bstack_first_cut)
                        dstat_stall_reason = DSTL_BSTACK;
`endif
                end
            end
            if (lane_is_serializing[i] && prefix_dispatched) begin
                `DSTAT_CUT(DCUT_SER_CUR)
                stop_prefix = 1'b1;
            end
`ifdef FUSE_BRANCH
            // Keep the fused pair atomic (shared-infra §4b): if the master's
            // slave-branch could not get a checkpoint this cycle, stall the
            // master too (so master+slave dispatch together or not at all).
            // Mirrors the lane_needs_ckpt branch-stack stall above.
            if (lane_fuse_pre_branch[i] && branch_stack_full) begin
`ifdef DISPATCH_STATS
                bstack_first_cut = !stop_prefix;
`endif
                `DSTAT_CUT(DCUT_FUSE_BST)
                stop_prefix = 1'b1;
                if (!prefix_dispatched) begin
                    dispatch_stall = 1'b1;
`ifdef DISPATCH_STATS
                    if (dstat_stall_reason == DSTL_NONE && bstack_first_cut)
                        dstat_stall_reason = DSTL_BSTACK;
`endif
                end
            end
`endif

            dispatch_valid[i] = lane_valid[i] && !dispatch_stall && !stop_prefix;
            prefix_dispatched |= dispatch_valid[i];

            if (dispatch_valid[i] && lane_is_branch[i]) begin
                branch_seen = 1'b1;
                // A dispatched branch unconditionally ends the group. This is the
                // dominant truncation source and is NOT one of the ladder tests
                // above -- it fires after this lane already dispatched.
                `DSTAT_CUT(DCUT_BR_TERM)
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

`undef DSTAT_CUT

endmodule: ooo_dispatch_control
