`include "ooo_types.vh"

`default_nettype none

module int_issue_queue
    import OOO_Types::*;
(
    input wire logic                 clk,
    input wire logic                 rst_l,
    input wire logic [OOO_WIDTH-1:0] insert_valid,
    input wire issue_entry_t         insert_entry [OOO_WIDTH],
    input wire logic [OOO_WIDTH-1:0] wakeup_valid,
    input wire phys_reg_t            wakeup_prd [OOO_WIDTH],
    // Speculative wakeup: an ALU producer in EXECUTE (S2) broadcasts its dest one
    // cycle before its writeback bus appears, so a dependent ALU consumer can issue
    // back-to-back (zero bubble) across the new select->execute pipeline register.
    // Applied to FU_ALU consumers ONLY (see the wakeup loop) -- they read operands
    // in their own S2 (select+1), where the producer's writeback is bypassed.
    input wire logic [ALU_ISSUE_PORTS-1:0] spec_wake_valid,
    input wire phys_reg_t            spec_wake_prd [ALU_ISSUE_PORTS],
    input wire logic [FU_ISSUE_PORTS-1:0] issue_ready,
    input wire branch_mask_t         reset_mask,
    input wire branch_mask_t         abort_mask,
    // Full pipeline flush on a precise trap / interrupt / trap-return: discard
    // every queued instruction (all are younger than the trapping instruction).
    input wire logic                 flush,
    output logic                 full,
    output logic [FU_ISSUE_PORTS-1:0] issue_valid,
    output issue_entry_t         issue_entry [FU_ISSUE_PORTS]
);

    issue_entry_t entries_q [INT_IQ_SIZE];
    issue_entry_t entries_next [INT_IQ_SIZE];
    logic [$clog2(INT_IQ_SIZE+1)-1:0] count_q, count_next;
    logic [$clog2(OOO_WIDTH+1)-1:0] insert_count;

    // Parallel per-FU-class issue select (replaces the serial 5-port x 16-entry
    // scan). Each entry belongs to exactly one fu_class, so the port classes never
    // contend for the same entry -- pick each class independently. Only the two ALU
    // ports share a class, and only ALU ops can be control flow, so the branch
    // constraints live entirely in the ALU 2-pick.
    typedef logic [$clog2(INT_IQ_SIZE)-1:0] iq_idx_t;
    logic [INT_IQ_SIZE-1:0] sel_rdy, sel_cf, sel_alu_cand, sel_alu1_cand;
    logic [INT_IQ_SIZE-1:0] sel_mul, sel_div, sel_fp;
    iq_idx_t alu0_idx, alu1_idx, mul_idx, div_idx, fp_idx;
    logic alu0_found, alu1_found, alu0_is_cf;

    assign full = (count_q > INT_IQ_SIZE - OOO_WIDTH);

    always_comb begin
        insert_count = '0;
        for (int i = 0; i < OOO_WIDTH; i += 1) begin
            insert_count += insert_valid[i];
        end
    end

    always_comb begin
        entries_next = entries_q;
        issue_valid = '0;
        for (int i = 0; i < FU_ISSUE_PORTS; i += 1) begin
            issue_entry[i] = '0;
        end

        for (int i = 0; i < INT_IQ_SIZE; i += 1) begin
            if ((entries_next[i].branch_mask & abort_mask) != '0) begin
                entries_next[i] = '0;
            end else if (entries_next[i].valid) begin
                entries_next[i].branch_mask &= ~reset_mask;
                for (int w = 0; w < OOO_WIDTH; w += 1) begin
                    if (wakeup_valid[w]) begin
                        if (entries_next[i].prs1 == wakeup_prd[w]) begin
                            entries_next[i].src1_ready = 1'b1;
                        end
                        if (entries_next[i].prs2 == wakeup_prd[w]) begin
                            entries_next[i].src2_ready = 1'b1;
                        end
                    end
                end
                // Speculative ALU wakeup -> FU_ALU consumers only. An ALU consumer
                // reads its operands in its own execute stage (select+1), where the
                // spec-woken producer's writeback is already on the bus (bypassed in
                // the phys_reg_file). MUL/DIV/FP read operands at select (no execute
                // register) so the value is NOT yet available -> they wake only on
                // the completion wakeup above. spec_wake_prd is a freshly allocated
                // dest (has_dest, prd != 0), so it never spuriously matches prs == 0.
                if (entries_next[i].fu_class == FU_ALU) begin
                    for (int w = 0; w < ALU_ISSUE_PORTS; w += 1) begin
                        if (spec_wake_valid[w]) begin
                            if (entries_next[i].prs1 == spec_wake_prd[w]) begin
                                entries_next[i].src1_ready = 1'b1;
                            end
                            if (entries_next[i].prs2 == spec_wake_prd[w]) begin
                                entries_next[i].src2_ready = 1'b1;
                            end
                        end
                    end
                end
            end
        end

        // ---- Per-FU-class eligibility (parallel over the post-wakeup entries) ----
        for (int i = 0; i < INT_IQ_SIZE; i += 1) begin
            sel_rdy[i] = entries_next[i].valid &&
                entries_next[i].src1_ready && entries_next[i].src2_ready;
            sel_cf[i] = is_control_flow(entries_next[i]);
            // A control-flow op may issue only once every older branch it is
            // speculative under has resolved (branch_mask == 0).
            sel_alu_cand[i] = sel_rdy[i] && (entries_next[i].fu_class == FU_ALU) &&
                !(sel_cf[i] && (entries_next[i].branch_mask != '0));
            sel_mul[i] = sel_rdy[i] && (entries_next[i].fu_class == FU_MUL);
            sel_div[i] = sel_rdy[i] && (entries_next[i].fu_class == FU_DIV);
            sel_fp[i]  = sel_rdy[i] && (entries_next[i].fu_class == FU_FP);
        end

        // ---- ALU 2-pick (ports 0,1) ----  a0 = lowest candidate; a1 = next lowest
        // excluding a0, and -- if a0 is a control transfer -- excluding any other
        // control transfer (at most one branch/jump issues per cycle).
        alu0_idx   = lowest_idx(sel_alu_cand);
        alu0_found = issue_ready[ISSUE_ALU0] && (sel_alu_cand != '0);
        alu0_is_cf = alu0_found && sel_cf[alu0_idx];
        for (int i = 0; i < INT_IQ_SIZE; i += 1) begin
            sel_alu1_cand[i] = sel_alu_cand[i] && (iq_idx_t'(i) != alu0_idx) &&
                !(alu0_is_cf && sel_cf[i]);
        end
        alu1_idx   = lowest_idx(sel_alu1_cand);
        alu1_found = alu0_found && issue_ready[ISSUE_ALU1] && (sel_alu1_cand != '0);

        // ---- MUL/DIV/FP single picks (independent; never control flow) ----
        mul_idx = lowest_idx(sel_mul);
        div_idx = lowest_idx(sel_div);
        fp_idx  = lowest_idx(sel_fp);

        // ---- Apply picks: drive the FU ports, clear the issued entries. The picked
        // indices are distinct (a1 != a0; MUL/DIV/FP are different fu_class), so the
        // clears never collide. ----
        if (alu0_found) begin
            issue_valid[ISSUE_ALU0] = 1'b1;
            issue_entry[ISSUE_ALU0] = entries_next[alu0_idx];
            entries_next[alu0_idx] = '0;
        end
        if (alu1_found) begin
            issue_valid[ISSUE_ALU1] = 1'b1;
            issue_entry[ISSUE_ALU1] = entries_next[alu1_idx];
            entries_next[alu1_idx] = '0;
        end
        if (issue_ready[ISSUE_MUL] && (sel_mul != '0)) begin
            issue_valid[ISSUE_MUL] = 1'b1;
            issue_entry[ISSUE_MUL] = entries_next[mul_idx];
            entries_next[mul_idx] = '0;
        end
        if (issue_ready[ISSUE_DIV] && (sel_div != '0)) begin
            issue_valid[ISSUE_DIV] = 1'b1;
            issue_entry[ISSUE_DIV] = entries_next[div_idx];
            entries_next[div_idx] = '0;
        end
        if (issue_ready[ISSUE_FP] && (sel_fp != '0)) begin
            issue_valid[ISSUE_FP] = 1'b1;
            issue_entry[ISSUE_FP] = entries_next[fp_idx];
            entries_next[fp_idx] = '0;
        end

        if (!full) begin
            for (int lane = 0; lane < OOO_WIDTH; lane += 1) begin
                if (insert_valid[lane]) begin
                    for (int i = 0; i < INT_IQ_SIZE; i += 1) begin
                        if (!entries_next[i].valid) begin
                            entries_next[i] = insert_entry[lane];
                            entries_next[i].valid = 1'b1;
                            break;
                        end
                    end
                end
            end
        end

        if (flush) begin
            for (int i = 0; i < INT_IQ_SIZE; i += 1) begin
                entries_next[i] = '0;
            end
            issue_valid = '0;
            for (int i = 0; i < FU_ISSUE_PORTS; i += 1) begin
                issue_entry[i] = '0;
            end
        end

        // Queue occupancy = popcount of the valid bits in the final next-state
        // (flush zeroes every entry -> 0). Replaces the serial -1/+1 RMW that was
        // threaded through squash/issue/insert, which put count_next on the deep
        // serial select->count chain (the abort_mask -> count_q worst path). The
        // sum of 1-bit valids synthesizes to a shallow popcount adder tree.
        count_next = '0;
        for (int i = 0; i < INT_IQ_SIZE; i += 1) begin
            count_next += entries_next[i].valid;
        end
    end

    function automatic logic is_control_flow(issue_entry_t entry);
        is_control_flow = (entry.ctrl.pc_source == PC_cond) ||
            (entry.ctrl.pc_source == PC_uncond) ||
            (entry.ctrl.pc_source == PC_indirect);
    endfunction

    // Lowest set index in a per-entry mask (a priority encoder over the 16 IQ
    // slots). The reverse iteration leaves index 0 assigned last, so the lowest
    // set bit wins; returns 0 when the mask is empty (callers gate on mask != 0).
    function automatic iq_idx_t lowest_idx(input logic [INT_IQ_SIZE-1:0] mask);
        lowest_idx = '0;
        for (int i = INT_IQ_SIZE-1; i >= 0; i -= 1) begin
            if (mask[i]) lowest_idx = iq_idx_t'(i);
        end
    endfunction

    always_ff @(posedge clk or negedge rst_l) begin
        if (!rst_l) begin
            count_q <= '0;
            for (int i = 0; i < INT_IQ_SIZE; i += 1) begin
                entries_q[i] <= '0;
            end
        end else begin
            entries_q <= entries_next;
            count_q <= count_next;
        end
    end

endmodule: int_issue_queue
