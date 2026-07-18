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
`ifdef LOAD_SPEC_WAKE
    ,
    // LOAD_SPEC_WAKE (plans/dhry-attack-plan/load-spec-wake.md): a hit-predicted
    // load's dest, broadcast by the LSQ at issue (sole-outstanding, plain
    // single-beat cacheable INT, non-device). spec_rdy below is a COMBINATIONAL
    // select-time readiness boost -- never written into entries_q.src_ready --
    // so a missed consumer re-arms on the refill writeback. ld_spec_hit/miss is
    // the core's one-cycle verdict for the broadcast; issue_ld_spec poisons the
    // per-ALU-port pick so the core can squash the consumer's S2 on a miss.
    input  wire logic            load_spec_wake_valid,
    input  wire phys_reg_t       load_spec_wake_prd,
    input  wire logic            ld_spec_hit,
    input  wire logic            ld_spec_miss,
    output logic [ALU_ISSUE_PORTS-1:0] issue_ld_spec
`endif
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

`ifdef LOAD_SPEC_WAKE
    // LOAD_SPEC_WAKE retain-in-IQ state (IQ-local parallel arrays; no
    // issue_entry_t change, so the OFF packet width is byte-identical):
    //  - spec_rdy1/2: combinational select-time readiness boost off the
    //    hit-predicted load broadcast (FU_ALU consumers only, exactly like the
    //    ALU spec_wake in block A). NEVER persisted into entries_q.src_ready,
    //    so a not-picked consumer simply loses the boost next cycle, and a
    //    missed pick re-arms only on the real completion wakeup.
    //  - spec_issued_q: the entry was picked while spec-reliant and is running
    //    its speculative S2. It is RETAINED (not freed at select) and excluded
    //    from re-select until the one-cycle verdict: freed on ld_spec_hit,
    //    re-armed (bit cleared, committed src_ready still 0) on ld_spec_miss.
    //  - issued_ld_spec_slot: this cycle's pick is spec-reliant (a source was
    //    ready ONLY via the boost) -- poisons the S2 op via issue_ld_spec.
    logic [INT_IQ_SIZE-1:0] spec_issued_q, spec_issued_next;
    logic [INT_IQ_SIZE-1:0] spec_rdy1, spec_rdy2;
    logic [INT_IQ_SIZE-1:0] issued_ld_spec_slot;
    // Occupancy correction: a spec-reliant pick asserts issue_valid but does
    // NOT free its slot (spec_retain); a hit-verified spec_issued slot is freed
    // one cycle later (spec_free, disjoint from sq_count via the !squash gate).
    logic [$clog2(INT_IQ_SIZE+1)-1:0] spec_retain;
    logic [$clog2(INT_IQ_SIZE+1)-1:0] spec_free;
`endif

    // Parallel per-FU-class issue select (replaces the serial 5-port x 16-entry
    // scan). Each entry belongs to exactly one fu_class, so the port classes never
    // contend for the same entry -- pick each class independently. Only the two ALU
    // ports share a class, and only ALU ops can be control flow, so the branch
    // constraints live entirely in the ALU 2-pick.
    typedef logic [$clog2(INT_IQ_SIZE)-1:0] iq_idx_t;
    logic [INT_IQ_SIZE-1:0] sel_rdy, sel_cf, sel_alu_cand, sel_csr;
    logic [INT_IQ_SIZE-1:0] sel_mul, sel_div, sel_fp;
`ifdef CF_OOO
    logic                   cf_head_ready;  // the oldest unresolved branch can issue now
`endif
    // Generalized ALU N-pick (scales with ALU_ISSUE_PORTS; at ALU_ISSUE_PORTS==2 it
    // is value-identical to the former 2-pick). alu_taken tracks the running union
    // of picked slots (distinctness); cf_prefix latches once any earlier port takes
    // a control-flow op so later ports exclude CF (<=1 CF issue/cycle). CSR ops
    // confine to ALU0/ALU1 (only CSR_RD_PORTS=2 csr read ports exist), so ports
    // >= CSR_RD_PORTS exclude sel_csr.
    localparam int CSR_RD_PORTS = 2;
    logic [INT_IQ_SIZE-1:0]     alu_cand [ALU_ISSUE_PORTS];
    iq_idx_t                    alu_idx  [ALU_ISSUE_PORTS];
    logic [ALU_ISSUE_PORTS-1:0] alu_found;
    logic [INT_IQ_SIZE-1:0]     alu_taken;
    logic                       cf_prefix;
    iq_idx_t mul_idx, div_idx, fp_idx;

    // Parallel insert: the (<= OOO_WIDTH) incoming ops fill the lowest free slots.
    logic [INT_IQ_SIZE-1:0] ins_free_mask;
    iq_idx_t ins_free [OOO_WIDTH];
    logic [$clog2(OOO_WIDTH+1)-1:0] ins_rank [OOO_WIDTH];

`ifdef AGE_ORDER
    // AGE_ORDER (plans/dhry-direct-attacks.md Stage 4): the NxN relative-age
    // matrix. age_q[i][j]=1 <=> slot i is OLDER than slot j. Written ONLY at
    // insert (a full row+col scrub of the target slot) and cleared on flush;
    // never touched on issue/squash/abort -- a dead slot's stale age bits are
    // don't-care (validity gates candidacy) and are erased wholesale when the
    // slot is reused. The "clear on free" therefore keys on the REAL free
    // (slot reuse at insert), not on raw issue_valid: a future LOAD_SPEC_WAKE
    // retained pick (ld_spec_hit re-issue of a spec-issued entry) keeps its
    // age until its slot is genuinely recycled, so a spec-wake replay cannot
    // corrupt the order. ins_target[lane] is the slot block C writes for lane.
    logic [INT_IQ_SIZE-1:0][INT_IQ_SIZE-1:0] age_q, age_next;
    iq_idx_t ins_target [OOO_WIDTH];
`endif

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
`ifdef LOAD_SPEC_WAKE
        issue_ld_spec = '0;
        // Combinational spec-readiness boost (reads entries_wake, FU_ALU-only
        // exactly like the ALU spec-wake in block A): a load broadcast makes a
        // dependent ALU consumer SELECTABLE this cycle; the boost is never
        // written into entries_q (see the decl note). A spec_issued entry is
        // already running its speculative S2, so it is never re-boosted.
        spec_rdy1 = '0;
        spec_rdy2 = '0;
        for (int i = 0; i < INT_IQ_SIZE; i += 1) begin
            spec_rdy1[i] = load_spec_wake_valid && entries_wake[i].valid &&
                (entries_wake[i].fu_class == FU_ALU) && !spec_issued_q[i] &&
                (entries_wake[i].prs1 == load_spec_wake_prd);
            spec_rdy2[i] = load_spec_wake_valid && entries_wake[i].valid &&
                (entries_wake[i].fu_class == FU_ALU) && !spec_issued_q[i] &&
                (entries_wake[i].prs2 == load_spec_wake_prd);
            // A pick is spec-reliant iff a source was ready ONLY via the boost.
            issued_ld_spec_slot[i] =
                (spec_rdy1[i] && !entries_wake[i].src1_ready) ||
                (spec_rdy2[i] && !entries_wake[i].src2_ready);
        end
`endif

        // ---- Per-FU-class eligibility (parallel over the post-wakeup entries) ----
`ifdef CF_OOO
        // CF_OOO -- out-of-order control-flow issue.
        //
        // The baseline lets a CF op issue only once EVERY older branch it is
        // speculative under has resolved (branch_mask == 0), so only the OLDEST
        // unresolved branch may ever execute: branch resolve -- and hence branch
        // checkpoint reclamation -- is fully serialized. That was never a correctness
        // requirement. It entered as an admitted stop-gap ("conservatively handle
        // nested branches for now", 7264445, 2026-05-24) bundled with the real bug of
        // that day (the free_list restore-distance cast), and branch_stack has always
        // been an order-agnostic POOL whose frees are keyed by resolve_id
        // (branch_stack.sv:63,85-112,137-154). No recorded bug depends on it.
        //
        // CF_OOO demotes the rule from a BAN to a PRIORITY: a younger CF op may take
        // the (<=1/cycle, cf_prefix) CF issue slot ONLY on a cycle when the oldest
        // unresolved branch is not itself ready to take it. The oldest branch -- the
        // one that unblocks the ROB head and drains the checkpoint pool -- is thus
        // never delayed by a younger one, and younger branches only fill CF slots that
        // would otherwise idle. A plain ban-removal would NOT be safe here: the ALU
        // pick is index-priority (lowest_idx, :182), not age-ordered, so it could
        // starve the oldest branch in a machine that is already commit-starved.
        //
        // At most ONE entry can have branch_mask == '0 (any CF dispatched while a
        // branch is unresolved carries that branch's bit), so cf_head_ready identifies
        // exactly the oldest unresolved branch. The sel_rdy terms are repeated verbatim
        // here because sel_rdy[] is only assigned in the loop below.
        cf_head_ready = 1'b0;
        for (int i = 0; i < INT_IQ_SIZE; i += 1) begin
            if (entries_wake[i].valid &&
                    entries_wake[i].src1_ready && entries_wake[i].src2_ready &&
                    ((entries_q[i].branch_mask & abort_mask) == '0) &&
                    (entries_wake[i].fu_class == FU_ALU) &&
                    is_control_flow(entries_wake[i]) &&
`ifdef LOAD_SPEC_WAKE
                    // A spec-issued branch is already running its speculative
                    // S2 -- never mistake it for the oldest-ready branch.
                    !spec_issued_q[i] &&
`endif
                    (entries_wake[i].branch_mask == '0)) begin
                cf_head_ready = 1'b1;
            end
        end
`endif
        for (int i = 0; i < INT_IQ_SIZE; i += 1) begin
            // FB2b R3 abort gate: exclude wrong-path entries from selection without
            // the deep squash (block A no longer zeroes them). Uses the ORIGINAL
            // entries_q.branch_mask (pre-reset_mask) to match block A/C's squash
            // decision exactly -- reset_mask clears the resolved-branch bit, which
            // abort_mask also carries, so a post-reset mask could miss a wrong-path
            // entry whose only abort bit was that branch. ~2 levels, parallel to wakeup.
`ifdef LOAD_SPEC_WAKE
            // LOAD_SPEC_WAKE: the hit-predicted load's boost ORs into the
            // single readiness wire -- FU_ALU-only by construction, so the
            // MUL/DIV/FP class masks below are unaffected. A spec-issued entry
            // is running its speculative S2: excluded from re-select (also
            // from the AGE_ORDER age pick, which ranks this same sel_rdy).
            sel_rdy[i] = entries_wake[i].valid &&
                (entries_wake[i].src1_ready || spec_rdy1[i]) &&
                (entries_wake[i].src2_ready || spec_rdy2[i]) &&
                ((entries_q[i].branch_mask & abort_mask) == '0) &&
                !spec_issued_q[i];
`else
            sel_rdy[i] = entries_wake[i].valid &&
                entries_wake[i].src1_ready && entries_wake[i].src2_ready &&
                ((entries_q[i].branch_mask & abort_mask) == '0);
`endif
            sel_cf[i] = is_control_flow(entries_wake[i]);
            // CSR ops read from the priv CSR file's 2 read ports (wired to ALU0/ALU1
            // only), so they must confine to the first CSR_RD_PORTS ALU ports.
            sel_csr[i] = (entries_wake[i].ctrl.exec_class == EXEC_CSR);
`ifdef CF_OOO
            // CF_OOO: hold a younger CF op ONLY while the oldest unresolved branch is
            // itself ready to issue this cycle (see the note above). Otherwise the CF
            // slot would idle, so let the younger branch resolve early and recycle its
            // checkpoint.
            sel_alu_cand[i] = sel_rdy[i] && (entries_wake[i].fu_class == FU_ALU) &&
                !(sel_cf[i] && (entries_wake[i].branch_mask != '0) && cf_head_ready);
`else
            // A control-flow op may issue only once every older branch it is
            // speculative under has resolved (branch_mask == 0).
            sel_alu_cand[i] = sel_rdy[i] && (entries_wake[i].fu_class == FU_ALU) &&
                !(sel_cf[i] && (entries_wake[i].branch_mask != '0));
`endif
            sel_mul[i] = sel_rdy[i] && (entries_wake[i].fu_class == FU_MUL);
            sel_div[i] = sel_rdy[i] && (entries_wake[i].fu_class == FU_DIV);
            sel_fp[i]  = sel_rdy[i] && (entries_wake[i].fu_class == FU_FP);
        end

        // ---- ALU N-pick (ports 0..ALU_ISSUE_PORTS-1) ----  each port picks the
        // lowest remaining candidate, excluding already-picked slots (alu_taken),
        // excluding any further control transfer once one is taken (cf_prefix, so
        // <=1 CF/cycle), and excluding CSR ops on ports >= CSR_RD_PORTS. alu_found
        // is monotone (port p requires p-1) so a failed port makes later idx
        // don't-cares. At ALU_ISSUE_PORTS==2 this is value-identical to the 2-pick.
        alu_taken = '0;
        cf_prefix = 1'b0;
        for (int p = 0; p < ALU_ISSUE_PORTS; p += 1) begin
            alu_cand[p] = sel_alu_cand & ~alu_taken &
                (cf_prefix ? ~sel_cf : {INT_IQ_SIZE{1'b1}}) &
                ((p < CSR_RD_PORTS) ? {INT_IQ_SIZE{1'b1}} : ~sel_csr);
`ifdef AGE_ORDER
            // AGE_ORDER: oldest-ready instead of index-priority. alu_cand[p]
            // derives from sel_rdy, the single readiness wire -- Stage 5
            // (LOAD_SPEC_WAKE) ORs its spec-rdy boost into sel_rdy and is
            // ranked by the same age matrix with no further change here.
            alu_idx[p]   = oldest_idx(alu_cand[p]);
`else
            alu_idx[p]   = lowest_idx(alu_cand[p]);
`endif
            alu_found[p] = issue_ready[ISSUE_ALU0 + p] && (alu_cand[p] != '0) &&
                ((p == 0) ? 1'b1 : alu_found[p-1]);
            if (alu_found[p]) begin
                alu_taken[alu_idx[p]] = 1'b1;
                if (sel_cf[alu_idx[p]]) cf_prefix = 1'b1;
            end
        end

        // ---- MUL/DIV/FP single picks (independent; never control flow) ----
`ifdef AGE_ORDER
        mul_idx = oldest_idx(sel_mul);
        div_idx = oldest_idx(sel_div);
        fp_idx  = oldest_idx(sel_fp);
`else
        mul_idx = lowest_idx(sel_mul);
        div_idx = lowest_idx(sel_div);
        fp_idx  = lowest_idx(sel_fp);
`endif

        // ---- Apply picks: drive the FU ports, clear the issued entries in the
        // post-select snapshot. The picked indices are distinct (a1 != a0;
        // MUL/DIV/FP are different fu_class), so the clears never collide. ----
        for (int p = 0; p < ALU_ISSUE_PORTS; p += 1) begin
            if (alu_found[p]) begin
                issue_valid[ISSUE_ALU0 + p] = 1'b1;
                issue_entry[ISSUE_ALU0 + p] = entries_wake[alu_idx[p]];
`ifdef LOAD_SPEC_WAKE
                // LOAD_SPEC_WAKE: poison the per-port S2 op, and RETAIN a
                // spec-reliant pick (do not free the slot); spec_issued_next is
                // set in block C, and the slot is freed on ld_spec_hit
                // (free-on-verify, block C) or re-armed on ld_spec_miss.
                // alu_idx[p] is whichever slot the pick selected (lowest_idx,
                // or the AGE_ORDER oldest_idx under that flag) -- the
                // retain-vs-free gate applies to the same index either way.
                issue_ld_spec[p] = issued_ld_spec_slot[alu_idx[p]];
                if (!issued_ld_spec_slot[alu_idx[p]]) begin
                    entries_sel[alu_idx[p]] = '0;   // normal free
                end
`else
                entries_sel[alu_idx[p]] = '0;
`endif
            end
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

`ifdef LOAD_SPEC_WAKE
        // LOAD_SPEC_WAKE spec_issued next-state. Set: this cycle's spec-reliant
        // ALU picks (disjoint from the resolve below -- a spec_issued_q entry is
        // excluded from select, so no slot is set and cleared in one cycle).
        // Resolve: ONE global verdict (ld_spec_hit/miss) covers every entry
        // spec-issued last cycle -- at most one load broadcasts per cycle and
        // its resolution window is exactly one cycle.
        spec_issued_next = spec_issued_q;
        for (int p = 0; p < ALU_ISSUE_PORTS; p += 1) begin
            if (alu_found[p] && issued_ld_spec_slot[alu_idx[p]]) begin
                spec_issued_next[alu_idx[p]] = 1'b1;
            end
        end
        for (int i = 0; i < INT_IQ_SIZE; i += 1) begin
            if (spec_issued_q[i]) begin
                if ((entries_q[i].branch_mask & abort_mask) != '0) begin
                    spec_issued_next[i] = 1'b0;   // squashed anyway (zeroed below)
                end else if (ld_spec_hit) begin
                    spec_issued_next[i] = 1'b0;   // verified; entry freed below
                end else if (ld_spec_miss) begin
                    // Re-arm: keep the entry, drop the bit. Its committed
                    // src_ready still has the spec source = 0 (the boost was
                    // never persisted), so it re-selects only when the real
                    // load completion wakeup arrives.
                    spec_issued_next[i] = 1'b0;
                end
            end
        end
        if (flush) begin
            spec_issued_next = '0;
        end
`endif
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

`ifdef LOAD_SPEC_WAKE
        // LOAD_SPEC_WAKE free-on-verify (folded into block C AFTER the abort
        // squash + insert, per the adversarial hardening): a spec-issued entry
        // verified by ld_spec_hit retires from the IQ now (its S2 result
        // registers at X+2; the slot is released one cycle after a normal
        // issue). Gated on !abort-squashed so it is disjoint from the squash
        // above (no double-free in the occupancy count). Placed after insert
        // so ins_free (registered occupancy) can never have targeted the slot
        // this cycle, and so the AGE_ORDER Phase-1 column read of
        // entries_next.valid below sees the slot already free. On ld_spec_miss
        // the entry is UNTOUCHED here (re-arm, see spec_issued_next above).
        for (int i = 0; i < INT_IQ_SIZE; i += 1) begin
            if (spec_issued_q[i] && ld_spec_hit &&
                    ((entries_q[i].branch_mask & abort_mask) == '0)) begin
                entries_next[i] = '0;
            end
        end
`endif

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
`ifdef LOAD_SPEC_WAKE
        // LOAD_SPEC_WAKE occupancy correction: a spec-reliant pick asserts
        // issue_valid (counted in iss_count) but RETAINS its slot, so back it
        // out (spec_retain <= iss_count, no underflow); a hit-verified
        // spec_issued slot frees HERE (spec_free), one cycle after its pick.
        // spec_free is gated on !abort-squashed so it is disjoint from
        // sq_count, and a spec_issued slot is a valid occupied entry so
        // spec_free <= count_q - sq_count. On a miss nothing frees (re-arm).
        spec_retain = '0;
        for (int p = 0; p < ALU_ISSUE_PORTS; p += 1) begin
            if (alu_found[p] && issued_ld_spec_slot[alu_idx[p]]) begin
                spec_retain += 1'b1;
            end
        end
        spec_free = '0;
        for (int i = 0; i < INT_IQ_SIZE; i += 1) begin
            if (spec_issued_q[i] && ld_spec_hit &&
                    ((entries_q[i].branch_mask & abort_mask) == '0)) begin
                spec_free += 1'b1;
            end
        end
        count_next = flush ? '0 :
            (count_q - sq_count - (iss_count - spec_retain) - spec_free + ins_count);
`else
        count_next = flush ? '0 : (count_q - sq_count - iss_count + ins_count);
`endif

`ifdef AGE_ORDER
        // ---- AGE matrix maintenance (AGE_ORDER) ----
        // Carry by default; insert is the ONLY writer. Phase 1: an inserted op
        // lands younger than every entry that survives the cycle -- a full
        // row+col scrub of its target slot, which also erases the stale bits
        // of the slot's previous occupant wholesale (the real "clear on free":
        // keyed on slot reuse, NOT on issue_valid, so a Stage-5 spec-woken
        // retained pick keeps its age until the slot is genuinely recycled).
        // The column reads entries_next.valid (post squash/select/insert), so
        // co-inserted slots see each other as valid. Phase 2 (AFTER Phase 1,
        // blocking) then orders same-cycle co-inserts by lane (= program)
        // order, overwriting Phase 1's both-directions bits between their
        // targets. ins_free reads REGISTERED occupancy (R2'), so a this-cycle-
        // freed slot is not a target and its scrub cannot race a live entry.
        // Issue/squash/abort never write age: validity gates candidacy, and
        // the diagonal stays 0 (row scrub covers j==target, col skips it).
        // flush -> empty queue, no order.
        for (int lane = 0; lane < OOO_WIDTH; lane += 1) begin
            ins_target[lane] = ins_free[ins_rank[lane]];
        end
        age_next = age_q;
        if (!full) begin
            for (int lane = 0; lane < OOO_WIDTH; lane += 1) begin
                if (insert_valid[lane]) begin
                    for (int j = 0; j < INT_IQ_SIZE; j += 1) begin
                        age_next[ins_target[lane]][j] = 1'b0;
                        age_next[j][ins_target[lane]] =
                            entries_next[j].valid && (j != ins_target[lane]);
                    end
                end
            end
            for (int l1 = 0; l1 < OOO_WIDTH; l1 += 1) begin
                for (int l2 = l1 + 1; l2 < OOO_WIDTH; l2 += 1) begin
                    if (insert_valid[l1] && insert_valid[l2]) begin
                        age_next[ins_target[l1]][ins_target[l2]] = 1'b1;
                        age_next[ins_target[l2]][ins_target[l1]] = 1'b0;
                    end
                end
            end
        end
        if (flush) begin
            age_next = '0;
        end
`endif
    end

    function automatic logic is_control_flow(issue_entry_t entry);
        is_control_flow = (entry.ctrl.pc_source == PC_cond) ||
            (entry.ctrl.pc_source == PC_uncond) ||
`ifdef FUSE_BRANCH
            // A fused branch-fusion master (FUSE_CMPBR) RESOLVES a branch on
            // its ALU writeback, so it must join the CF issue discipline: the
            // <=1-CF-issue/cycle cf_prefix exclusion (the writeback bus's
            // branch_writeback carries only ONE resolve per cycle — a 2nd
            // same-cycle resolve would be silently dropped, checkpoint and
            // mispredict redirect lost) and the CF_OOO oldest-branch priority
            // hold (it is a branch resolve for pool-drain purposes; on a
            // non-CF_OOO build this is the conservative hard ban — correct,
            // just less speculative). It is NOT held by its own folded slave's
            // checkpoint: its branch_mask predates the slave's allocation.
            ((entry.ctrl.pc_source == PC_indirect) || entry.fuse_is_branch);
`else
            (entry.ctrl.pc_source == PC_indirect);
`endif
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

`ifdef AGE_ORDER
    // Oldest set index in a mask (AGE_ORDER): the candidate no OTHER candidate
    // is older than. age_q[j][i]=1 <=> j older than i; the mask[j] term on the
    // suppressor is load-bearing -- a non-candidate (not-ready / wrong-path /
    // already-taken slot) must never suppress. Seeded with lowest_idx so that
    // if totality ever breaks the pick degrades to a real in-mask candidate
    // instead of silently firing slot 0 (invalid / wrong-path). With the
    // insert maintenance above the live entries form a strict total order, so
    // exactly one candidate has no older rival and the result is one-hot-safe.
    // Reads only the REGISTERED age_q: no new combinational loop into block B.
    function automatic iq_idx_t oldest_idx(input logic [INT_IQ_SIZE-1:0] mask);
        oldest_idx = lowest_idx(mask);
        for (int i = 0; i < INT_IQ_SIZE; i += 1) begin
            logic any_older;
            any_older = 1'b0;
            for (int j = 0; j < INT_IQ_SIZE; j += 1) begin
                if ((j != i) && mask[j] && age_q[j][i]) any_older = 1'b1;
            end
            if (mask[i] && !any_older) oldest_idx = iq_idx_t'(i);
        end
    endfunction
`endif

    always_ff @(posedge clk or negedge rst_l) begin
        if (!rst_l) begin
            count_q <= '0;
`ifdef AGE_ORDER
            age_q <= '0;
`endif
`ifdef LOAD_SPEC_WAKE
            spec_issued_q <= '0;
`endif
            for (int i = 0; i < INT_IQ_SIZE; i += 1) begin
                entries_q[i] <= '0;
            end
        end else begin
            // Element-wise (not whole-array `entries_q <= entries_next`): a whole
            // unpacked-array NBA trips a Verilator V3Delayed internal error at the
            // larger INT_IQ_SIZE=24 (BIG_IQ). Behaviourally identical, so the default
            // 16-entry build is unchanged.
            for (int i = 0; i < INT_IQ_SIZE; i += 1)
                entries_q[i] <= entries_next[i];
            count_q <= count_next;
`ifdef AGE_ORDER
            // age_q is a packed NxN vector (not an unpacked array), so a
            // whole-vector NBA is safe from the V3Delayed issue above.
            age_q <= age_next;
`endif
`ifdef LOAD_SPEC_WAKE
            spec_issued_q <= spec_issued_next;
`endif
        end
    end

endmodule: int_issue_queue
