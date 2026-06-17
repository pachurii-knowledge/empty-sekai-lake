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
    // FB2b false-loop break: the former monolithic always_comb read insert_entry
    // (insert) AND wrote issue_entry (select) -- a whole-block alias that drew the
    // false dispatch_issue_entries -> int_issue_entry loop edge on EVERY UNOPTFLAT
    // loop. Split into A: squash+wakeup (-> entries_wake), B: select (-> issue_entry
    // + entries_sel), C: insert+flush+count (-> entries_next). Select runs before
    // insert, so issue_entry never depended on insert_entry; this is pure code
    // motion (value-identical), making the acyclicity structural.
    issue_entry_t entries_wake [INT_IQ_SIZE]; // post squash + wakeup
    issue_entry_t entries_sel  [INT_IQ_SIZE]; // post select (issued slots cleared)
    logic [$clog2(INT_IQ_SIZE+1)-1:0] count_q, count_next;
    // FB2b depth cut: incremental occupancy (count_q - squashed - issued + inserted)
    // instead of popcount(entries_next.valid), which put count_next at the tail of the
    // deep abort_mask -> squash -> select -> insert -> entries_next chain (the -12.8 ns
    // worst path). These three popcounts read the shallow squash/issue/insert masks.
    logic [$clog2(INT_IQ_SIZE+1)-1:0] sq_count;   // entries squashed by abort this cycle
    logic [$clog2(INT_IQ_SIZE+1)-1:0] iss_count;  // entries issued (selected) this cycle
    logic [$clog2(INT_IQ_SIZE+1)-1:0] ins_count;  // entries inserted this cycle
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

    // Parallel insert: the (<= OOO_WIDTH) incoming ops fill the lowest free slots.
    logic [INT_IQ_SIZE-1:0] ins_free_mask;
    iq_idx_t ins_free [OOO_WIDTH];
    logic [$clog2(OOO_WIDTH+1)-1:0] ins_rank [OOO_WIDTH];

    assign full = (count_q > INT_IQ_SIZE - OOO_WIDTH);

    always_comb begin
        insert_count = '0;
        for (int i = 0; i < OOO_WIDTH; i += 1) begin
            insert_count += insert_valid[i];
        end
    end

    // ---- A: squash + branch-mask reset + wakeup (post-wakeup snapshot) ----
    always_comb begin
        entries_wake = entries_q;
        for (int i = 0; i < INT_IQ_SIZE; i += 1) begin
            // FB2b R3: the deep squash (zeroing every ~100-bit wrong-path entry) is
            // DEFERRED to block C (applied before the free-slot insert), taking it OFF
            // the select's input cone -- the front of the -12.x branch-recovery worst
            // path. Wakeup + reset_mask still apply to all valid entries here; the
            // select excludes wrong-path entries via a cheap 2-level abort gate
            // (block B), and a wrong-path entry's stale fields are harmless (it is
            // never picked, and is zeroed in block C before it can be reused by insert
            // or counted). entries_next, count, and the issued set are bit-identical.
            if (entries_wake[i].valid) begin
                entries_wake[i].branch_mask &= ~reset_mask;
                for (int w = 0; w < OOO_WIDTH; w += 1) begin
                    if (wakeup_valid[w]) begin
                        if (entries_wake[i].prs1 == wakeup_prd[w]) begin
                            entries_wake[i].src1_ready = 1'b1;
                        end
                        if (entries_wake[i].prs2 == wakeup_prd[w]) begin
                            entries_wake[i].src2_ready = 1'b1;
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
                if (entries_wake[i].fu_class == FU_ALU) begin
                    for (int w = 0; w < ALU_ISSUE_PORTS; w += 1) begin
                        if (spec_wake_valid[w]) begin
                            if (entries_wake[i].prs1 == spec_wake_prd[w]) begin
                                entries_wake[i].src1_ready = 1'b1;
                            end
                            if (entries_wake[i].prs2 == spec_wake_prd[w]) begin
                                entries_wake[i].src2_ready = 1'b1;
                            end
                        end
                    end
                end
            end
        end
    end

    // ---- B: per-FU-class select -> issue ports + post-select entries ----
    // Reads ONLY entries_wake (NOT insert_entry); this is what severs the false
    // dispatch_issue_entries -> int_issue_entry loop edge.
    always_comb begin
        entries_sel = entries_wake;
        issue_valid = '0;
        for (int i = 0; i < FU_ISSUE_PORTS; i += 1) begin
            issue_entry[i] = '0;
        end

        // ---- Per-FU-class eligibility (parallel over the post-wakeup entries) ----
        for (int i = 0; i < INT_IQ_SIZE; i += 1) begin
            // FB2b R3 abort gate: exclude wrong-path entries from selection without
            // the deep squash (block A no longer zeroes them). Uses the ORIGINAL
            // entries_q.branch_mask (pre-reset_mask) to match block A/C's squash
            // decision exactly -- reset_mask clears the resolved-branch bit, which
            // abort_mask also carries, so a post-reset mask could miss a wrong-path
            // entry whose only abort bit was that branch. ~2 levels, parallel to wakeup.
            sel_rdy[i] = entries_wake[i].valid &&
                entries_wake[i].src1_ready && entries_wake[i].src2_ready &&
                ((entries_q[i].branch_mask & abort_mask) == '0);
            sel_cf[i] = is_control_flow(entries_wake[i]);
            // A control-flow op may issue only once every older branch it is
            // speculative under has resolved (branch_mask == 0).
            sel_alu_cand[i] = sel_rdy[i] && (entries_wake[i].fu_class == FU_ALU) &&
                !(sel_cf[i] && (entries_wake[i].branch_mask != '0));
            sel_mul[i] = sel_rdy[i] && (entries_wake[i].fu_class == FU_MUL);
            sel_div[i] = sel_rdy[i] && (entries_wake[i].fu_class == FU_DIV);
            sel_fp[i]  = sel_rdy[i] && (entries_wake[i].fu_class == FU_FP);
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

        // ---- Apply picks: drive the FU ports, clear the issued entries in the
        // post-select snapshot. The picked indices are distinct (a1 != a0;
        // MUL/DIV/FP are different fu_class), so the clears never collide. ----
        if (alu0_found) begin
            issue_valid[ISSUE_ALU0] = 1'b1;
            issue_entry[ISSUE_ALU0] = entries_wake[alu0_idx];
            entries_sel[alu0_idx] = '0;
        end
        if (alu1_found) begin
            issue_valid[ISSUE_ALU1] = 1'b1;
            issue_entry[ISSUE_ALU1] = entries_wake[alu1_idx];
            entries_sel[alu1_idx] = '0;
        end
        if (issue_ready[ISSUE_MUL] && (sel_mul != '0)) begin
            issue_valid[ISSUE_MUL] = 1'b1;
            issue_entry[ISSUE_MUL] = entries_wake[mul_idx];
            entries_sel[mul_idx] = '0;
        end
        if (issue_ready[ISSUE_DIV] && (sel_div != '0)) begin
            issue_valid[ISSUE_DIV] = 1'b1;
            issue_entry[ISSUE_DIV] = entries_wake[div_idx];
            entries_sel[div_idx] = '0;
        end
        if (issue_ready[ISSUE_FP] && (sel_fp != '0)) begin
            issue_valid[ISSUE_FP] = 1'b1;
            issue_entry[ISSUE_FP] = entries_wake[fp_idx];
            entries_sel[fp_idx] = '0;
        end

        if (flush) begin
            issue_valid = '0;
            for (int i = 0; i < FU_ISSUE_PORTS; i += 1) begin
                issue_entry[i] = '0;
            end
        end
    end

    // ---- C: parallel insert (lowest free slots) + flush + occupancy ----
    always_comb begin
        entries_next = entries_sel;

        // FB2b R3: apply the deferred branch squash here (moved from block A), BEFORE
        // the free-slot insert -- so a wrong-path slot reads free for ins_free exactly
        // as when the squash ran in block A. entries_sel already has the issued slots
        // cleared by the select (which never picked a wrong-path entry, via the abort
        // gate), so zeroing the wrong-path entries here yields a bit-identical
        // entries_next. Wrong-path entries are zeroed before they can be counted
        // (count reads entries_q + abort_mask directly) or reused.
        for (int i = 0; i < INT_IQ_SIZE; i += 1) begin
            if ((entries_q[i].branch_mask & abort_mask) != '0) begin
                entries_next[i] = '0;
            end
        end

        // The incoming ops fill the lowest free slots, in lane order. ins_free[k] is
        // the k-th lowest free slot (priority encoders, each excluding the prior). A
        // valid lane takes ins_free[its prefix rank] = the (#valid lanes before it)-th
        // lowest free slot. !full guarantees >= OOO_WIDTH free slots, so all ins_free[]
        // are valid.
        //
        // FB2b R2': the free mask reads the REGISTERED occupancy (entries_q.valid), NOT
        // the post-squash/post-select entries_next.valid. This takes the ins_free
        // priority encoders OFF the abort_mask -> squash -> select -> ins_free worst
        // path -- they now read registered state, parallel to the select. Correct
        // because !full => count_q <= INT_IQ_SIZE - OOO_WIDTH => >= OOO_WIDTH registered
        // -free slots, so the insert never fails; the inserted ops just land in slots
        // that were free LAST cycle instead of also-this-cycle-freed (squashed/issued)
        // slots. NOT bit-identical -- slot assignment differs -> issue priority among
        // co-ready ops differs (architecturally correct OoO scheduling) -> needs
        // usertests stress. No IPC (a dispatched op is still selectable the next cycle).
        if (!full) begin
            for (int i = 0; i < INT_IQ_SIZE; i += 1) begin
                ins_free_mask[i] = !entries_q[i].valid;
            end
            ins_free[0] = lowest_idx(ins_free_mask);
            for (int k = 1; k < OOO_WIDTH; k += 1) begin
                logic [INT_IQ_SIZE-1:0] taken;
                taken = '0;
                for (int p = 0; p < k; p += 1) begin
                    taken[ins_free[p]] = 1'b1;
                end
                ins_free[k] = lowest_idx(ins_free_mask & ~taken);
            end
            for (int lane = 0; lane < OOO_WIDTH; lane += 1) begin
                ins_rank[lane] = '0;
                for (int j = 0; j < OOO_WIDTH; j += 1) begin
                    if ((j < lane) && insert_valid[j]) begin
                        ins_rank[lane] += 1'b1;
                    end
                end
            end
            for (int lane = 0; lane < OOO_WIDTH; lane += 1) begin
                if (insert_valid[lane]) begin
                    entries_next[ins_free[ins_rank[lane]]] = insert_entry[lane];
                    entries_next[ins_free[ins_rank[lane]]].valid = 1'b1;
                end
            end
        end

        if (flush) begin
            for (int i = 0; i < INT_IQ_SIZE; i += 1) begin
                entries_next[i] = '0;
            end
        end

        // Queue occupancy, computed INCREMENTALLY (value-identical to the former
        // popcount(entries_next.valid), but off the deep select/insert chain): the
        // surviving entries are count_q minus the squashed (abort) and issued
        // (selected) entries -- which are DISJOINT, since a squashed entry reads
        // valid=0 in entries_wake and so cannot be selected -- plus the inserted ops.
        // count_q >= squashed + issued (both are subsets of the valid entries), so the
        // running value never underflows. squashed reads entries_q + abort_mask
        // directly (shallow, parallel to the select); issued/inserted are popcounts of
        // the fire / insert_valid masks. flush -> 0 (matches the entry zeroing above).
        sq_count = '0;
        for (int i = 0; i < INT_IQ_SIZE; i += 1) begin
            if (entries_q[i].valid &&
                    ((entries_q[i].branch_mask & abort_mask) != '0)) begin
                sq_count += 1'b1;
            end
        end
        iss_count = '0;
        for (int i = 0; i < FU_ISSUE_PORTS; i += 1) begin
            iss_count += issue_valid[i];
        end
        ins_count = '0;
        if (!full) begin
            for (int lane = 0; lane < OOO_WIDTH; lane += 1) begin
                ins_count += insert_valid[lane];
            end
        end
        count_next = flush ? '0 : (count_q - sq_count - iss_count + ins_count);
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
