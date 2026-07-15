`include "ooo_types.vh"

`default_nettype none

module active_list
    import OOO_Types::*;
(
    input wire logic                  clk,
    input wire logic                  rst_l,
    input wire logic                  restore_valid,
    input wire active_id_t            restore_tail,
    // Full pipeline flush on a precise trap / interrupt / trap-return. Unlike a
    // branch restore (which rolls back to a checkpoint), this squashes every
    // entry still in flight behind the committing (trapping) instruction. The
    // trapping instruction itself still commits this cycle via the loop above;
    // `flush` then discards everything younger.
    input wire logic                  flush,
    input wire logic [OOO_WIDTH-1:0]  allocate_valid,
    input wire rename_packet_t        allocate_packet [OOO_WIDTH],
    input wire logic [OOO_WIDTH-1:0]  writeback_valid,
    input wire active_id_t            writeback_id [OOO_WIDTH],
    input wire logic [OOO_WIDTH-1:0][XLEN-1:0] writeback_data,
    input wire logic [OOO_WIDTH-1:0]  writeback_exception,
    input wire logic [OOO_WIDTH-1:0][4:0] writeback_exc_cause,
    input wire logic [OOO_WIDTH-1:0]  writeback_halted,
    input wire logic [OOO_WIDTH-1:0]  writeback_fp_write,
    input wire arch_reg_t             writeback_fp_rd [OOO_WIDTH],
    input wire fp_reg_data_t          writeback_fp_data [OOO_WIDTH],
    input wire logic [OOO_WIDTH-1:0]  writeback_csr_write,
    input wire logic [OOO_WIDTH-1:0][11:0] writeback_csr_addr,
    input wire logic [OOO_WIDTH-1:0][XLEN-1:0] writeback_csr_wdata,
    input wire logic [OOO_WIDTH-1:0]  writeback_fp_fflags_valid,
    input wire logic [OOO_WIDTH-1:0][4:0] writeback_fp_fflags,
    input wire branch_mask_t          reset_mask,
    input wire branch_mask_t          abort_mask,
    // Which presented commit lanes the commit unit actually retired this
    // cycle (a prefix of commit_valid). Only taken entries are popped; a
    // held lane -- e.g. a store whose write the memory port cannot accept
    // this cycle -- stays at the head and is re-presented next cycle.
    // Without this feedback the list popped every presented entry, so a
    // held store silently vanished from the ROB while the LSQ kept waiting
    // for its commit_store pulse (a deadlock the N2 memory-latency fuzzer
    // exposed; unreachable before N1 because store_port_busy could never
    // coincide with a store at the ROB head under the fixed-latency port).
`ifndef COMMIT_1STAGE
    // In the 2-stage commit (below) commit_taken refers to the REGISTERED
    // window commit_valid_q/commit_packet_q, which describes entries at the
    // CURRENT head_q -- so the pop reads commit_taken[i] and retires entry
    // head_q + i.
`endif /* COMMIT_1STAGE */
    input wire logic [OOO_WIDTH-1:0]  commit_taken,
    output logic                  full,
    output logic                  empty,
    output active_id_t            tail,
    output logic [OOO_WIDTH-1:0]  commit_valid,
    output commit_packet_t        commit_packet [OOO_WIDTH],
    output logic [OOO_WIDTH-1:0]  free_valid,
    output phys_reg_t             free_prd [OOO_WIDTH]
);

`ifndef COMMIT_1STAGE
    // ====================================================================
    // FB2b wall #1 -- the 2-stage (registered) commit (ROB-recovery root).
    // ====================================================================
    // The ROB commit OUTPUT (commit_valid/commit_packet) is the single
    // broadest combinational root in the core: it fans into the commit unit,
    // the precise-trap/return detection, the architectural map + free-list,
    // the CSR/FP commit, the frontend redirect, and the full-pipeline flush
    // -- 30+ consumer chains, all in one cycle, all rooted at this present.
    // Registering it (commit_valid_q/commit_packet_q) cuts every consumer
    // chain at once (measured +1.17 ns placed, -9.666 -> -8.496).
    //
    // To keep 4/cycle commit throughput across the new register, the present
    // is moved out by one cycle as a SLIDING WINDOW: each cycle we (1) POP the
    // registered window at head_q on commit_taken, (2) head-skip leading
    // squashed/invalid entries, (3) PRESENT the next <=OOO_WIDTH contiguous
    // valid+done entries from head_NEXT (post-pop, post-skip) and REGISTER
    // that present, and set head_q <= head_next. So commit_valid_q(N) always
    // describes entries starting exactly at head_q(N) -- lane i <=> entry
    // head_q + i. See plans/rob-2stage-commit-audit.md for the proof and the
    // R1-R5 coding contracts enforced below.
    //
    // Invariant I1 (load-bearing): entries in the registered window are the
    // OLDEST in flight, so no same-cycle branch abort can squash them (the
    // aborting branch is strictly younger); they always retire unless held by
    // the store port (re-presented, I3) or they ARE the trap (window
    // suppressed via the flush gate on commit_valid_q, I4). A held store
    // re-presents automatically: stop_prefix => commit_taken[j..]=0 =>
    // head_next = head_q + j, and the head-skip never passes the valid held
    // store, so PRESENT re-includes it at lane 0 next cycle.

`endif /* COMMIT_1STAGE */
    typedef struct packed {
        logic valid;
        logic done;
        logic exception;
        logic [4:0] exc_cause;
        logic halted;
        logic [XLEN-1:0] pc;
        logic [31:0] instr;
`ifdef RVC
        logic is_compressed;
        logic [15:0] rvc_parcel;
`endif
        logic [XLEN-1:0] data;
        logic fp_write;
        arch_reg_t fp_rd;
        fp_reg_data_t fp_data;
        logic csr_write;
        logic [11:0] csr_addr;
        logic [XLEN-1:0] csr_wdata;
        logic fp_fflags_valid;
        logic [4:0] fp_fflags;
        logic serializing;
        arch_reg_t rd;
        phys_reg_t prd;
        phys_reg_t old_prd;
        logic has_dest;
        logic is_store;
        logic is_sc;          // M4-S5b: store-conditional (memWrite + EXEC_AMO + AMO_SC)
        logic is_amo;         // M4 #3: RMW atomic (EXEC_AMO, amo_op not LR/SC)
        branch_mask_t branch_mask;
    } active_entry_t;

    typedef logic [$clog2(ACTIVE_LIST_SIZE+1)-1:0] active_count_t;

    active_entry_t entries_q [ACTIVE_LIST_SIZE];
    active_entry_t entries_next [ACTIVE_LIST_SIZE];
`ifndef COMMIT_1STAGE
    active_entry_t entries_squash [ACTIVE_LIST_SIZE];   // entries_q + reset_mask clear
    active_entry_t entries_premerge [ACTIVE_LIST_SIZE]; // + wrong-path squash + retired-pop zero
`else /* COMMIT_1STAGE */
    // FB2b false-loop break: the main always_comb read alloc_count (= this cycle's
    // allocate_valid, line "count_next += alloc_count") AND wrote commit_valid -- a
    // whole-block alias drawing the false dispatch_valid -> active_commit_valid edge
    // (the commit/recovery cycle). commit_valid is presented from the PRE-alloc
    // count, so it is independent of alloc_count. Split via entries_premerge: block
    // C (squash/head-skip/commit-present/pop -> entries_premerge + count_next_c +
    // commit_valid, reads NO alloc_count) and block A (reserve/Q-write/writeback/
    // flush -> entries_next). Value-identical -- the alloc reserve/write was already
    // placed after the commit presentation.
    active_entry_t entries_premerge [ACTIVE_LIST_SIZE];
`endif /* COMMIT_1STAGE */
    active_id_t head_q, head_next;
    active_id_t tail_q, tail_next;
    active_count_t count_q, count_next;
`ifndef COMMIT_1STAGE

    // Stage-2 registered commit window (the ROB-recovery-root cut). The
    // module OUTPUTS commit_valid/commit_packet are these registers; the
    // combinational PRESENT below drives commit_valid_next/commit_packet_next,
    // which are flopped into them (gated to 0 on flush -- R3/I4).
    logic [OOO_WIDTH-1:0]  commit_valid_q;
    commit_packet_t        commit_packet_q [OOO_WIDTH];
    logic [OOO_WIDTH-1:0]  commit_valid_next;
    commit_packet_t        commit_packet_next [OOO_WIDTH];

`else /* COMMIT_1STAGE */
    active_count_t count_next_c;   // post-commit count handed to the allocate block
    // FB2b false-loop break (retire_valid): the commit block read commit_taken (the
    // pop) AND wrote commit_valid (the present) -- a whole-block alias that closed the
    // last false loop commit_valid -> commit_unit -> retire_valid(=commit_taken) -> pop.
    // commit_valid is computed (present) BEFORE the pop reads commit_taken, so the
    // alias is false. Split: block C1 (squash + head-skip + present) writes commit_valid
    // + the post-squash/head-skip state below, reading NO commit_taken; block C2 (pop)
    // reads commit_taken and produces the final entries_premerge/head_next/count_next_c.
    active_entry_t entries_squash [ACTIVE_LIST_SIZE]; // post-reset (pre-pop) entries
    active_id_t    head_postskip;                     // head after the leading-skip
    active_count_t count_postskip;                    // count after the leading-skip
`endif /* COMMIT_1STAGE */
    // FB2b R3: per-slot post-squash validity (= entries_q.valid && not-wrong-path),
`ifndef COMMIT_1STAGE
    // a shallow function of registered state + abort_mask. The head-skip and the
    // present read THIS instead of the deep-zeroed entries_premerge[i].valid, so
    // the ~100-bit-per-entry squash leaves their input cones; the zeroing is
    // deferred to entries_premerge (POP block). Equals the old squash validity.
`else /* COMMIT_1STAGE */
    // a shallow function of registered state + abort_mask. The head-skip and commit
    // present read THIS instead of the deep-zeroed entries_squash[i].valid, so the
    // ~100-bit-per-entry squash leaves their input cones; the zeroing is deferred to
    // entries_premerge (block C2). Equals the old entries_squash[i].valid exactly.
`endif /* COMMIT_1STAGE */
    logic [ACTIVE_LIST_SIZE-1:0] sq_valid;
    logic [$clog2(OOO_WIDTH+1)-1:0] alloc_count;
    logic [$clog2(OOO_WIDTH+1)-1:0] commit_pop_count;
    // Dispatch->insert (D->Q) pipeline register: the tail/count RESERVE advances
    // combinationally from this cycle's allocate_valid (D stage), but the wide
    // entry-array WRITE is deferred one cycle and driven from these registered
    // packets (Q stage). Takes the rename cone off the entry-fill input path.
    logic [OOO_WIDTH-1:0] allocate_valid_q;
    rename_packet_t       allocate_packet_q [OOO_WIDTH];
    logic [ACTIVE_LIST_SIZE-1:0] inflight_mask;
    int commit_count;
`ifdef COMMIT_1STAGE
    active_id_t walk_id;
    active_count_t walk_left;
`endif /* COMMIT_1STAGE */
    logic commit_go;   // running prefix for the parallel commit presentation
    active_id_t commit_idx;
    logic commit_ready;
`ifndef COMMIT_1STAGE
    // Post-pop (pre-skip) head + count, handed from the POP block to the
    // SKIP+PRESENT block. head_after_pop = head_q + commit_pop_count; the
    // restore distance is measured from THIS cycle's head_q, then the pop is
    // subtracted (R5 -- do not key the restore off head_next).
    active_id_t    head_after_pop;
    active_count_t count_after_pop;
    // Parallel leading-invalid-head skip: a constant-offset leading count
    // (reverse scan), capped at count_after_pop. base = first valid entry
    // after the pop; the present starts there and head_q <= base = head_next.
`else /* COMMIT_1STAGE */
    // Parallel leading-invalid-head skip (see the head-advance block): replaces a
    // 32-deep sequential ripple with a constant-offset leading count.
`endif /* COMMIT_1STAGE */
    active_count_t head_skip_n;
`ifndef COMMIT_1STAGE
    active_id_t    base_head;
    active_count_t count_after_skip;
`endif /* COMMIT_1STAGE */

    assign tail = tail_q;
    assign full = (count_q > ACTIVE_LIST_SIZE - OOO_WIDTH);
    assign empty = (count_q == '0);
`ifndef COMMIT_1STAGE
    assign commit_valid  = commit_valid_q;   // REGISTERED (the recovery-root cut)
    assign commit_packet = commit_packet_q;
`endif /* COMMIT_1STAGE */

    always_comb begin
        alloc_count = '0;
        commit_pop_count = '0;
        for (int i = 0; i < OOO_WIDTH; i += 1) begin
            alloc_count += allocate_valid[i];
            commit_pop_count += commit_taken[i];
        end
    end

`ifndef COMMIT_1STAGE
    // ---- Block POP: retire the REGISTERED window at head_q (commit_taken). ----
    // Reads commit_taken + the registered entries at head_q + i; produces the
    // entry-state after reset_mask clear + wrong-path squash + retired-pop zero
    // (entries_premerge), the freed physregs, and the post-pop head/count handed
    // to SKIP+PRESENT. commit_taken is a registered-derived input (= retire_valid
    // from the commit unit reading commit_valid_q), so the present->commit->pop
    // cycle is broken by the commit_valid_q register -- no combinational loop.
    // The pop indexes from head_q (NOT a re-run head-skip): head_q already points
    // at the first valid entry (last cycle's present base), and per I1 it is never
    // squashed, so commit_taken[i] retires exactly entry head_q + i (R2).
`else /* COMMIT_1STAGE */
    // ---- Block C1: squash + reset + head-skip + commit PRESENT. Writes commit_valid
    // / commit_packet + the post-squash/head-skip state (entries_squash / head_postskip
    // / count_postskip). Reads entries_q (registered) + abort_mask/reset_mask +
    // allocate_*_q (registered) ONLY -- crucially NO commit_taken -> commit_valid is
    // severed from the pop, breaking the last false loop (commit_valid -> commit_unit
    // -> retire_valid(=commit_taken) -> pop). The present already ran BEFORE the pop,
    // so this is value-identical pure code motion. ----
`endif /* COMMIT_1STAGE */
    always_comb begin
`ifndef COMMIT_1STAGE
        // reset_mask: clear the resolved branch's checkpoint bit from survivors.
`endif /* COMMIT_1STAGE */
        entries_squash = entries_q;
`ifndef COMMIT_1STAGE
        for (int i = 0; i < ACTIVE_LIST_SIZE; i += 1) begin
            if (entries_squash[i].valid) begin
                entries_squash[i].branch_mask &= ~reset_mask;
            end
        end

        // Deferred branch squash: zero every wrong-path entry (R3 squash-defer --
        // off the head-skip / present input cones, which read sq_valid instead).
        entries_premerge = entries_squash;
        for (int i = 0; i < ACTIVE_LIST_SIZE; i += 1) begin
            if ((entries_q[i].branch_mask & abort_mask) != '0) begin
                entries_premerge[i] = '0;
            end
        end

        // Pop the retired prefix at head_q. commit_taken is a contiguous prefix
        // (the commit unit's stop_prefix halts all younger lanes once one is
        // held/halted/excepted), so retired lane i pops entry head_q + i at a
        // CONSTANT offset. An excepting instruction discards its result: its rd
        // keeps the old arch mapping (restored on flush), so old_prd must NOT be
        // freed here; its freshly allocated prd is reclaimed by the free-list
        // head rollback. Reads entries_squash (reset_mask only touches
        // branch_mask, so old_prd/has_dest/exception are the registered values).
        free_valid = '0;
        for (int i = 0; i < OOO_WIDTH; i += 1) begin
            free_prd[i] = '0;
        end
        for (int i = 0; i < OOO_WIDTH; i += 1) begin
            if (commit_taken[i]) begin
                free_valid[i] = entries_squash[head_q + active_id_t'(i)].has_dest &&
                    !entries_squash[head_q + active_id_t'(i)].exception;
                free_prd[i] = entries_squash[head_q + active_id_t'(i)].old_prd;
                entries_premerge[head_q + active_id_t'(i)] = '0;
            end
        end

        head_after_pop  = head_q + active_id_t'(commit_pop_count);
        // Restore distance measured from THIS cycle's head_q, pop subtracted
        // after (R5). active_distance(head_q, restore_tail) excludes the
        // wrong-path tail the misprediction rolls back.
        count_after_pop = (restore_valid ? restore_count(head_q, restore_tail)
                                         : count_q) - active_count_t'(commit_pop_count);
    end

    // ---- Block SKIP + PRESENT: head-skip leading squashed/invalid entries from
    // head_after_pop, then present (and REGISTER) the next <=OOO_WIDTH contiguous
    // valid+done entries from base_head = head_next. ----
    // Reads entries_q (registered, payload + done) + abort_mask (sq_valid) +
    // allocate_*_q (inflight_mask) + head_after_pop/count_after_pop (POP). Writes
    // commit_valid_next/commit_packet_next (-> registered) + base_head/head_next/
    // count_after_skip. Everything it reads is registered or registered-derived,
    // and its commit_valid_next output is registered -> no combinational loop.
    always_comb begin
        commit_valid_next = '0;
`else /* COMMIT_1STAGE */
        head_postskip = head_q;
        count_postskip = restore_valid ? restore_count(head_q, restore_tail) : count_q;
        commit_valid = '0;
`endif /* COMMIT_1STAGE */
        commit_count = 0;
        for (int i = 0; i < OOO_WIDTH; i += 1) begin
`ifndef COMMIT_1STAGE
            commit_packet_next[i] = '0;
`else /* COMMIT_1STAGE */
            commit_packet[i] = '0;
`endif /* COMMIT_1STAGE */
        end

`ifndef COMMIT_1STAGE
        // sq_valid (post-squash validity) from the ORIGINAL entries_q.branch_mask
        // (pre-reset), matching the squash decision; head-skip / present read it.
`else /* COMMIT_1STAGE */
        // FB2b R3: compute sq_valid (post-squash validity) shallowly and apply
        // reset_mask, but DEFER the deep wrong-path zeroing to entries_premerge
        // (block C2). sq_valid[i] uses the ORIGINAL entries_q.branch_mask (pre-reset),
        // matching the old squash decision; the head-skip / commit-present read it.
`endif /* COMMIT_1STAGE */
        for (int i = 0; i < ACTIVE_LIST_SIZE; i += 1) begin
            sq_valid[i] = entries_q[i].valid &&
                ((entries_q[i].branch_mask & abort_mask) == '0);
`ifdef COMMIT_1STAGE
            if (entries_squash[i].valid) begin
                entries_squash[i].branch_mask &= ~reset_mask;
            end
`endif /* COMMIT_1STAGE */
        end

`ifndef COMMIT_1STAGE
        // In-flight reserved mask for the head-skip. The registered dispatch
        // group (allocate_valid_q) reserved its ROB slots last cycle (count_q
        // includes them), but the wide entry write is deferred to the Q stage
        // (ALLOC block below). For the one cycle before that write a reserved
        // slot reads valid=0; mark those slots occupied here so the head-skip
        // does not mistake a reserved-but-unwritten slot for a squashed one and
        // skip it (the near-empty deadlock). Built from REGISTERED signals only.
`else /* COMMIT_1STAGE */
        // In-flight reserved mask for the head-skip. The registered dispatch group
        // (allocate_valid_q) reserved its ROB slots last cycle (count_q includes
        // them), but the wide entry write is deferred to the Q stage BELOW (after
        // the commit presentation, to keep that write out of the commit cone). So
        // for the one cycle before the deferred write, a reserved slot reads
        // valid=0. Mark those slots occupied here so the head-skip does not mistake
        // a reserved-but-unwritten head slot for a squashed (=zeroed) one and skip
        // it -- the near-empty deadlock. Built from REGISTERED signals only
        // (allocate_valid_q / allocate_packet_q), so it adds NO path into the
        // commit cone (writing the entry early here instead would close a
        // valid -> commit_valid -> trap_take/commit_taken -> entry loop). Same gate
        // as the Q write below, so mask membership == will-be-written.
`endif /* COMMIT_1STAGE */
        inflight_mask = '0;
        for (int i = 0; i < OOO_WIDTH; i += 1) begin
            if (allocate_valid_q[i] &&
                    ((allocate_packet_q[i].branch_mask & abort_mask) == '0)) begin
                inflight_mask[allocate_packet_q[i].active_id] = 1'b1;
            end
        end

`ifndef COMMIT_1STAGE
        // Advance over leading invalid (squashed) slots after the pop in one
        // shot: head_skip_n = number of leading invalid slots at head_after_pop,
        // capped at count_after_pop. Reverse scan (lowest in-range valid k wins);
        // defaults to count_after_pop when every in-range slot is invalid. A slot
        // counts as occupied (not skippable) if its entry is valid OR it is a
        // reserved-but-unwritten in-flight slot (inflight_mask).
        head_skip_n = count_after_pop;
`else /* COMMIT_1STAGE */
        // Advance the head over leading invalid (squashed) slots in one shot.
        // Behaviorally identical to the former 32-deep sequential ripple (which
        // re-indexed entries_premerge[head_next] with a rippling index each
        // iteration), but each slot is read at a CONSTANT offset (head_next + k)
        // so the validity reads are independent and only the leading-count
        // accumulates -- a far shallower cone feeding the commit/pop logic.
        // head_skip_n = number of leading invalid (squashed) slots at the head,
        // capped at count_next_c = the first VALID slot's offset from head_next. A
        // priority mux (reverse scan, lowest in-range valid k wins) instead of the
        // former 32-deep serial +1 accumulation; defaults to count_next_c when every
        // in-range slot is invalid. count_next_c is folded into the parallel-sum below.
        // A slot counts as occupied (not skippable) if its entry is valid OR it is
        // a reserved-but-unwritten in-flight slot (inflight_mask) -- only a genuine
        // squash (zeroed, not in-flight) is skipped.
        head_skip_n = count_postskip;
`endif /* COMMIT_1STAGE */
        for (int k = ACTIVE_LIST_SIZE-1; k >= 0; k -= 1) begin
`ifndef COMMIT_1STAGE
            if ((active_count_t'(k) < count_after_pop) &&
                    (sq_valid[head_after_pop +
`else /* COMMIT_1STAGE */
            if ((active_count_t'(k) < count_postskip) &&
                    (sq_valid[head_postskip +
`endif /* COMMIT_1STAGE */
                        ($clog2(ACTIVE_LIST_SIZE))'(k)] ||
`ifndef COMMIT_1STAGE
                     inflight_mask[head_after_pop +
`else /* COMMIT_1STAGE */
                     inflight_mask[head_postskip +
`endif /* COMMIT_1STAGE */
                        ($clog2(ACTIVE_LIST_SIZE))'(k)])) begin
                head_skip_n = active_count_t'(k);
            end
        end
`ifndef COMMIT_1STAGE
        base_head        = head_after_pop + head_skip_n[$clog2(ACTIVE_LIST_SIZE)-1:0];
        count_after_skip = count_after_pop - head_skip_n;
        head_next        = base_head;

        // Present up to OOO_WIDTH completed entries (oldest first), in PARALLEL,
        // from base_head. Contiguous from base_head, so lane i reads entry
        // (base_head + i) at a CONSTANT offset. commit_go is a 4-deep prefix: a
        // lane presents iff every older lane presented and none halted/excepted.
        // Condition reads sq_valid (this-cycle squash honored) + entries_q.done
        // (registered/stable); payload reads entries_q (done a prior cycle). The
        // present output is REGISTERED into commit_valid_q/commit_packet_q below,
        // so a completion becomes RETIRABLE two cycles after its writeback (one
        // for done, one for the present register -- the +1 commit-latency).
`else /* COMMIT_1STAGE */
        head_postskip = head_postskip + head_skip_n[$clog2(ACTIVE_LIST_SIZE)-1:0];
        count_postskip = count_postskip - head_skip_n;

        // (Writeback marking moved to AFTER the Q-stage allocate write below, so a
        // store/op that writes back in the SAME cycle as its deferred allocate
        // write keeps its done/data instead of being clobbered by the alloc reset.
        // Safe to move: the commit presentation reads the REGISTERED entries_q for
        // done/payload, so it never observes this cycle's writeback regardless.)

        // Present up to OOO_WIDTH completed entries (oldest first) WITHOUT
        // popping them. The commit unit decides which prefix actually retires
        // this cycle (commit_taken) -- e.g. a store is held while the memory
        // port cannot accept its write -- and only that prefix is popped
        // below; a held entry is re-presented next cycle. Presentation reads
        // entries_premerge so same-cycle aborts and writeback payloads are
        // visible, but `done` comes from the registered entry (a completion
        // becomes eligible to retire one cycle after its writeback).
        // Present up to OOO_WIDTH completed entries (oldest first), in PARALLEL.
        // The presented entries are CONTIGUOUS from head_next, so lane i reads
        // entry (head_next + i) at a CONSTANT offset -- 4 parallel 32:1 muxes,
        // not a data-dependent walk_id that serialized the 4 wide commit_packet
        // muxes (the FB2b ActiveList worst-path cone: commit_valid/commit_packet,
        // ~259 levels). commit_go is a 4-deep prefix: a lane presents iff every
        // older lane presented and none halted/excepted (matches the former
        // serial walk's break-on-stop and stop-on-gap). Condition reads
        // entries_premerge[idx].valid (this-cycle squash honored); payload reads
        // entries_q[idx] (done a prior cycle -> registered/stable, value-equiv).
`endif /* COMMIT_1STAGE */
        commit_go = 1'b1;
        commit_count = 0;
`ifdef COMMIT_1STAGE
        walk_id = head_postskip;
        walk_left = count_postskip;
`endif /* COMMIT_1STAGE */
        for (int i = 0; i < OOO_WIDTH; i += 1) begin
`ifndef COMMIT_1STAGE
            commit_idx = base_head + active_id_t'(i);
            commit_ready = (active_count_t'(i) < count_after_skip) &&
`else /* COMMIT_1STAGE */
            commit_idx = head_postskip + active_id_t'(i);
            commit_ready = (active_count_t'(i) < count_postskip) &&
`endif /* COMMIT_1STAGE */
                sq_valid[commit_idx] && entries_q[commit_idx].done;
            if (commit_go && commit_ready) begin
`ifndef COMMIT_1STAGE
                commit_valid_next[i] = 1'b1;
                commit_packet_next[i].valid = 1'b1;
                commit_packet_next[i].active_id = commit_idx;
                commit_packet_next[i].rd = entries_q[commit_idx].rd;
                commit_packet_next[i].prd = entries_q[commit_idx].prd;
                commit_packet_next[i].old_prd = entries_q[commit_idx].old_prd;
                commit_packet_next[i].has_dest = entries_q[commit_idx].has_dest;
                commit_packet_next[i].pc = entries_q[commit_idx].pc;
                commit_packet_next[i].instr = entries_q[commit_idx].instr;
`else /* COMMIT_1STAGE */
                commit_valid[i] = 1'b1;
                commit_packet[i].valid = 1'b1;
                commit_packet[i].active_id = commit_idx;
                commit_packet[i].rd = entries_q[commit_idx].rd;
                commit_packet[i].prd = entries_q[commit_idx].prd;
                commit_packet[i].old_prd = entries_q[commit_idx].old_prd;
                commit_packet[i].has_dest = entries_q[commit_idx].has_dest;
                commit_packet[i].pc = entries_q[commit_idx].pc;
                commit_packet[i].instr = entries_q[commit_idx].instr;
`endif /* COMMIT_1STAGE */
`ifdef RVC
`ifndef COMMIT_1STAGE
                commit_packet_next[i].is_compressed =
`else /* COMMIT_1STAGE */
                commit_packet[i].is_compressed =
`endif /* COMMIT_1STAGE */
                    entries_q[commit_idx].is_compressed;
`ifndef COMMIT_1STAGE
                commit_packet_next[i].rvc_parcel =
`else /* COMMIT_1STAGE */
                commit_packet[i].rvc_parcel =
`endif /* COMMIT_1STAGE */
                    entries_q[commit_idx].rvc_parcel;
`endif
`ifndef COMMIT_1STAGE
                commit_packet_next[i].data = entries_q[commit_idx].data;
                commit_packet_next[i].fp_write = entries_q[commit_idx].fp_write;
                commit_packet_next[i].fp_rd = entries_q[commit_idx].fp_rd;
                commit_packet_next[i].fp_data = entries_q[commit_idx].fp_data;
                commit_packet_next[i].csr_write = entries_q[commit_idx].csr_write;
                commit_packet_next[i].csr_addr = entries_q[commit_idx].csr_addr;
                commit_packet_next[i].csr_wdata = entries_q[commit_idx].csr_wdata;
                commit_packet_next[i].fp_fflags_valid =
`else /* COMMIT_1STAGE */
                commit_packet[i].data = entries_q[commit_idx].data;
                commit_packet[i].fp_write = entries_q[commit_idx].fp_write;
                commit_packet[i].fp_rd = entries_q[commit_idx].fp_rd;
                commit_packet[i].fp_data = entries_q[commit_idx].fp_data;
                commit_packet[i].csr_write = entries_q[commit_idx].csr_write;
                commit_packet[i].csr_addr = entries_q[commit_idx].csr_addr;
                commit_packet[i].csr_wdata = entries_q[commit_idx].csr_wdata;
                commit_packet[i].fp_fflags_valid =
`endif /* COMMIT_1STAGE */
                    entries_q[commit_idx].fp_fflags_valid;
`ifndef COMMIT_1STAGE
                commit_packet_next[i].fp_fflags = entries_q[commit_idx].fp_fflags;
                commit_packet_next[i].serializing = entries_q[commit_idx].serializing;
                commit_packet_next[i].is_store = entries_q[commit_idx].is_store;
                commit_packet_next[i].is_sc = entries_q[commit_idx].is_sc;
                commit_packet_next[i].is_amo = entries_q[commit_idx].is_amo;
                commit_packet_next[i].halted = entries_q[commit_idx].halted;
                commit_packet_next[i].exception = entries_q[commit_idx].exception;
                commit_packet_next[i].exc_cause = entries_q[commit_idx].exc_cause;
`else /* COMMIT_1STAGE */
                commit_packet[i].fp_fflags = entries_q[commit_idx].fp_fflags;
                commit_packet[i].serializing = entries_q[commit_idx].serializing;
                commit_packet[i].is_store = entries_q[commit_idx].is_store;
                commit_packet[i].is_sc = entries_q[commit_idx].is_sc;
                commit_packet[i].is_amo = entries_q[commit_idx].is_amo;
                commit_packet[i].halted = entries_q[commit_idx].halted;
                commit_packet[i].exception = entries_q[commit_idx].exception;
                commit_packet[i].exc_cause = entries_q[commit_idx].exc_cause;
`endif /* COMMIT_1STAGE */
                commit_count += 1;
                // Stop younger lanes after a halt/exception (precise commit).
`ifndef COMMIT_1STAGE
                if (commit_packet_next[i].halted || commit_packet_next[i].exception) begin
`else /* COMMIT_1STAGE */
                if (commit_packet[i].halted || commit_packet[i].exception) begin
`endif /* COMMIT_1STAGE */
                    commit_go = 1'b0;
                end
            end else begin
                // Gap (entry not yet committable): no younger lane may present.
                commit_go = 1'b0;
            end
        end
`ifdef COMMIT_1STAGE

    end

    // ---- Block C2: POP the retired prefix (commit_taken). Reads commit_taken +
    // commit_pop_count + C1's post-squash/head-skip state (entries_squash /
    // head_postskip / count_postskip); writes the FINAL entries_premerge / head_next /
    // count_next_c (consumed by block A + the head/count registers) + free_valid /
    // free_prd. This is the ONLY commit_taken reader on the path, and it is strictly
    // downstream of commit_valid (C1) -> no commit_valid <-> commit_taken loop. ----
    always_comb begin
        entries_premerge = entries_squash;
        head_next = head_postskip;
        count_next_c = count_postskip;
        free_valid = '0;
        for (int i = 0; i < OOO_WIDTH; i += 1) begin
            free_prd[i] = '0;
        end

        // FB2b R3: apply the deferred branch squash (moved out of C1) -- zero every
        // wrong-path entry. Off the head-skip / commit-present input cones (which now
        // read sq_valid); committed entries are never wrong-path (the present gates on
        // sq_valid), so the pop's per-lane reads of entries_squash above are
        // unaffected. entries_premerge (-> entries_next -> entries_q) is bit-identical
        // to the old C1 squash.
        for (int i = 0; i < ACTIVE_LIST_SIZE; i += 1) begin
            if ((entries_q[i].branch_mask & abort_mask) != '0) begin
                entries_premerge[i] = '0;
            end
        end

        // Pop exactly the retired prefix. commit_taken is a contiguous prefix of the
        // presented lanes (the commit unit's stop_prefix halts every younger lane once
        // one is held/halted/excepted), so retired lane i pops the entry at the CONSTANT
        // offset (head_postskip + i) -- frees and zeroes are independent across lanes,
        // and head_next/count_next_c advance by the popcount in one step. An excepting
        // instruction discards its result: its rd keeps the old mapping (restored from
        // the architectural map on flush), so old_prd must NOT be freed here; its
        // freshly allocated prd is reclaimed by rolling the free-list head back to the
        // committed head instead. The per-lane reads come from entries_squash (C1, never
        // popped), so they are independent of this block's zeroing.
        for (int i = 0; i < OOO_WIDTH; i += 1) begin
            if (commit_taken[i]) begin
                free_valid[i] = entries_squash[head_postskip + active_id_t'(i)].has_dest &&
                    !entries_squash[head_postskip + active_id_t'(i)].exception;
                free_prd[i] = entries_squash[head_postskip + active_id_t'(i)].old_prd;
                entries_premerge[head_postskip + active_id_t'(i)] = '0;
            end
        end
        head_next = head_postskip + active_id_t'(commit_pop_count);
        count_next_c = count_postskip - commit_pop_count;
`endif /* COMMIT_1STAGE */
    end

`ifndef COMMIT_1STAGE
    // ---- Block ALLOC: reserve tail/count from this cycle's dispatch + Q-stage
    // entry write (from the REGISTERED dispatch group) + writeback marking + trap
    // flush. Reads entries_premerge (POP) + count_after_skip (SKIP) + base_head
    // (head_next) + allocate_*_q + writeback + flush; writes entries_next /
    // count_next / tail_next. Off the commit cone. ----
`else /* COMMIT_1STAGE */
    // ---- Block A: reserve (tail/count from this cycle's dispatch) + Q-stage entry
    // write (from the REGISTERED dispatch group) + writeback marking + trap flush.
    // Reads alloc_count / allocate_packet_q (registered) / writeback; writes the real
    // entries_next / count_next / tail_next from entries_premerge / count_next_c.
    // Off the commit cone, so reading alloc_count here no longer aliases into
    // commit_valid. Value-identical: the reserve/write already followed the commit
    // presentation. ----
`endif /* COMMIT_1STAGE */
    always_comb begin
        entries_next = entries_premerge;
`ifndef COMMIT_1STAGE
        count_next = count_after_skip;
`else /* COMMIT_1STAGE */
        count_next = count_next_c;
`endif /* COMMIT_1STAGE */
        tail_next = restore_valid ? restore_tail : tail_q;

`ifndef COMMIT_1STAGE
        // Reserve (D stage): advance tail/count from THIS cycle's dispatch group
        // so tail_q/count_q/full are unchanged from the 1-stage design. Only the
        // wide entry-array write is deferred to the Q stage (below).
`else /* COMMIT_1STAGE */
        // Reserve (D stage): advance the tail/count from THIS cycle's dispatch
        // group, exactly as before -- so tail_q/count_q/full are unchanged. Only
        // the wide entry-array write is deferred to the Q stage (below); the tail
        // pointer still names where the reserved slots are.
`endif /* COMMIT_1STAGE */
        if (!restore_valid && !full) begin
            tail_next = tail_next + active_id_t'(alloc_count);
`ifndef COMMIT_1STAGE
            count_next = count_next + active_count_t'(alloc_count);
`else /* COMMIT_1STAGE */
            count_next = count_next + alloc_count;
`endif /* COMMIT_1STAGE */
        end

`ifndef COMMIT_1STAGE
        // Write (Q stage): fill the reserved ROB slots from the registered
        // dispatch group (reserved last cycle in D). active_id, computed in D
        // from the then-current tail, names the slot, now behind tail and within
        // [head, tail). Placed AFTER the pop/squash so this write is out of the
        // commit cone; the head-skip already treats these slots as occupied via
        // inflight_mask. The reserved slot read valid=0 in the gap (present saw a
        // not-yet-committable gap, correct); the earliest writeback is dispatch+2
        // (> this dispatch+1 write), so .done is never marked before the entry
        // exists. Gate off a branch-aborted in-flight group; `flush` is NOT gated
        // here (it is a commit output -> would re-close the loop) -- the trailing
        // flush block zeroes every entry on a trap anyway.
`else /* COMMIT_1STAGE */
        // Write (Q stage of the dispatch->insert pipeline): fill the reserved ROB
        // slots from the registered dispatch group (reserved last cycle in D; the
        // wide rename-packet fields land here one cycle later -- this is the timing
        // cut, taking the rename cone off the entry-fill input). active_id, computed
        // in D from the then-current tail, names the slot, now behind tail and
        // within [head, tail). Placed AFTER the commit presentation so this write
        // is OUT of the commit cone (no entry -> commit_valid -> commit_taken/
        // trap_take -> entry loop); the head-skip already treats these slots as
        // occupied via inflight_mask. The reserved slot read valid=0 in the gap, so
        // the commit presentation saw it as a not-yet-committable gap (correct); the
        // earliest writeback is dispatch+2 (> this dispatch+1 write), so .done is
        // never marked before the entry exists. Gate off a branch-aborted in-flight
        // group; `flush` is NOT gated here (it is a commit output -> would re-close
        // the loop) -- the trailing flush block zeroes every entry on a trap anyway.
        // Initialize the FULL entry (static + dynamic-field reset). The writeback
        // marking below runs AFTER this, so a same-cycle writeback re-asserts
        // done/data on top of this reset (no clobber).
`endif /* COMMIT_1STAGE */
        for (int i = 0; i < OOO_WIDTH; i += 1) begin
            if (allocate_valid_q[i] &&
                    ((allocate_packet_q[i].branch_mask & abort_mask) == '0)) begin
                entries_next[allocate_packet_q[i].active_id].valid = 1'b1;
                entries_next[allocate_packet_q[i].active_id].done = 1'b0;
                entries_next[allocate_packet_q[i].active_id].exception = 1'b0;
                entries_next[allocate_packet_q[i].active_id].exc_cause = 5'd0;
                entries_next[allocate_packet_q[i].active_id].halted = 1'b0;
                entries_next[allocate_packet_q[i].active_id].data = '0;
                entries_next[allocate_packet_q[i].active_id].fp_write = 1'b0;
                entries_next[allocate_packet_q[i].active_id].fp_rd =
                    allocate_packet_q[i].fp_rd;
                entries_next[allocate_packet_q[i].active_id].fp_data = '0;
                entries_next[allocate_packet_q[i].active_id].csr_write = 1'b0;
                entries_next[allocate_packet_q[i].active_id].csr_addr =
                    allocate_packet_q[i].instr[31:20];
                entries_next[allocate_packet_q[i].active_id].csr_wdata = '0;
                entries_next[allocate_packet_q[i].active_id].fp_fflags_valid = 1'b0;
                entries_next[allocate_packet_q[i].active_id].fp_fflags = '0;
                entries_next[allocate_packet_q[i].active_id].serializing =
                    allocate_packet_q[i].ctrl.serializing;
                entries_next[allocate_packet_q[i].active_id].pc =
                    allocate_packet_q[i].pc;
                entries_next[allocate_packet_q[i].active_id].instr =
                    allocate_packet_q[i].instr;
`ifdef RVC
                entries_next[allocate_packet_q[i].active_id].is_compressed =
                    allocate_packet_q[i].ctrl.is_compressed;
                entries_next[allocate_packet_q[i].active_id].rvc_parcel =
                    allocate_packet_q[i].ctrl.rvc_parcel;
`endif
                entries_next[allocate_packet_q[i].active_id].rd =
                    allocate_packet_q[i].rd;
                entries_next[allocate_packet_q[i].active_id].prd =
                    allocate_packet_q[i].prd;
                entries_next[allocate_packet_q[i].active_id].old_prd =
                    allocate_packet_q[i].old_prd;
                entries_next[allocate_packet_q[i].active_id].has_dest =
                    allocate_packet_q[i].has_dest;
                entries_next[allocate_packet_q[i].active_id].is_store =
                    allocate_packet_q[i].ctrl.memWrite;
                entries_next[allocate_packet_q[i].active_id].is_sc =
                    allocate_packet_q[i].ctrl.memWrite &&
                    (allocate_packet_q[i].ctrl.exec_class == EXEC_AMO) &&
                    (allocate_packet_q[i].ctrl.amo_op == AMO_SC);
                // M4 #3: an RMW atomic (EXEC_AMO, not LR/SC). The commit unit holds it
                // at the ROB head until the agent-authoritative COP_AMO resolves.
                entries_next[allocate_packet_q[i].active_id].is_amo =
                    (allocate_packet_q[i].ctrl.exec_class == EXEC_AMO) &&
                    (allocate_packet_q[i].ctrl.amo_op != AMO_LR) &&
                    (allocate_packet_q[i].ctrl.amo_op != AMO_SC);
                entries_next[allocate_packet_q[i].active_id].branch_mask =
                    allocate_packet_q[i].branch_mask & ~reset_mask;
            end
        end

`ifndef COMMIT_1STAGE
        // Writeback marking (after the Q-stage allocate write so a same-cycle
        // writeback re-asserts done/data over the allocate's done=0 reset). The
        // present reads the REGISTERED entries_q for done/payload, so this
        // cycle's marking only affects a future cycle's commit -- value-neutral.
`else /* COMMIT_1STAGE */
        // Writeback marking (moved here from before the commit presentation so it
        // runs AFTER the Q-stage allocate write above): a completing op marks its
        // ROB entry done + records its result. Placed after the allocate write so a
        // same-cycle writeback (its dispatch+1 write coinciding with a fast AGU
        // store/op completion) wins over the allocate's done=0 reset. The commit
        // presentation reads the REGISTERED entries_q for done/payload, so this
        // cycle's marking only affects NEXT cycle's commit -- moving it past the
        // presentation is value-neutral.
`endif /* COMMIT_1STAGE */
        for (int i = 0; i < OOO_WIDTH; i += 1) begin
            if (writeback_valid[i]) begin
                entries_next[writeback_id[i]].done = 1'b1;
                entries_next[writeback_id[i]].data = writeback_data[i];
                entries_next[writeback_id[i]].fp_write = writeback_fp_write[i];
                entries_next[writeback_id[i]].fp_rd = writeback_fp_rd[i];
                entries_next[writeback_id[i]].fp_data = writeback_fp_data[i];
                entries_next[writeback_id[i]].csr_write = writeback_csr_write[i];
                entries_next[writeback_id[i]].csr_addr = writeback_csr_addr[i];
                entries_next[writeback_id[i]].csr_wdata = writeback_csr_wdata[i];
                entries_next[writeback_id[i]].fp_fflags_valid =
                    writeback_fp_fflags_valid[i];
                entries_next[writeback_id[i]].fp_fflags = writeback_fp_fflags[i];
                if (writeback_exception[i] && !entries_next[writeback_id[i]].exception)
                    entries_next[writeback_id[i]].exc_cause = writeback_exc_cause[i];
                entries_next[writeback_id[i]].exception |= writeback_exception[i];
                entries_next[writeback_id[i]].halted |= writeback_halted[i];
            end
        end

        // Trap flush: discard everything still in flight behind the trapping
`ifndef COMMIT_1STAGE
        // instruction (which already committed via the registered window this
        // cycle, advancing head_next past it). Reset the queue to empty at the
        // post-commit head. The matching ahead-present suppression
        // (commit_valid_q <= 0 on flush) is in the register block (R3/I4).
`else /* COMMIT_1STAGE */
        // instruction (which already committed via the loop above, advancing
        // head_next past it). Reset the queue to empty at the post-commit head.
`endif /* COMMIT_1STAGE */
        if (flush) begin
            for (int i = 0; i < ACTIVE_LIST_SIZE; i += 1) begin
                entries_next[i] = '0;
            end
            tail_next = head_next;
            count_next = '0;
        end
    end

    function automatic active_count_t active_distance(input active_id_t from_id,
            input active_id_t to_id);
        if (to_id >= from_id) begin
            active_distance = active_count_t'(to_id - from_id);
        end else begin
            active_distance = active_count_t'(ACTIVE_LIST_SIZE - from_id + to_id);
        end
    endfunction

    // Occupancy of [head_id, rtail) on a branch-mispredict restore.
    //
    // active_distance() is mod-ACTIVE_LIST_SIZE, so it returns 0 -- never SIZE --
    // when rtail == head_id: a FULL ring aliases onto an EMPTY one. That state is
    // reachable: `full` (:222) is (count_q > SIZE - OOO_WIDTH), so a 4-wide group may
    // dispatch at count_q == SIZE-4 and drive count_q to exactly SIZE (head_q ==
    // tail_q, full); and because a dispatched branch terminates its group
    // (ooo_dispatch_control.sv:89-92) a branch can be the YOUNGEST ROB entry, whose
    // checkpoint tail (riscv_core_ooo.sv:2158) is then == head_q. If that branch
    // mispredicts, recomputing count as active_distance(head_q, head_q) = 0 declares
    // the ROB empty while every entry is still live -- a silent wipe that orphans the
    // in-flight physregs/LSQ entries and hangs the machine.
    //
    // On a restore the resolving branch's OWN entry is always still in the window (it
    // is only marked done by this cycle's writeback), so the true distance is >= 1.
    // rtail == head_id therefore unambiguously means SIZE, not 0.
    //
    // Latent on the in-order-CF-issue baseline (the restoring branch must also be the
    // only unresolved branch), but -DCF_OOO lets any younger branch restore and makes
    // it reachable in ordinary branch-dense code. Fixed unconditionally: the guarded
    // case cannot occur today, so this is behaviourally inert on the current baseline.
    function automatic active_count_t restore_count(input active_id_t head_id,
            input active_id_t rtail);
        restore_count = (rtail == head_id) ? active_count_t'(ACTIVE_LIST_SIZE)
                                           : active_distance(head_id, rtail);
    endfunction

    always_ff @(posedge clk or negedge rst_l) begin
        if (!rst_l) begin
            head_q <= '0;
            tail_q <= '0;
            count_q <= '0;
            allocate_valid_q <= '0;
`ifndef COMMIT_1STAGE
            // R1: the new stage-2 commit register MUST reset to 0. Verilator's
            // 2-state X->0 masks a missing reset, so an omission here is an
            // FPGA-ONLY phantom-retire at reset. commit_packet_q needs no reset
            // (every consumer gates on commit_valid_q).
            commit_valid_q <= '0;
`endif /* COMMIT_1STAGE */
            for (int i = 0; i < ACTIVE_LIST_SIZE; i += 1) begin
                entries_q[i] <= '0;
            end
        end else begin
            // Element-wise (not whole-array `entries_q <= entries_next`): a whole
            // unpacked-array NBA trips a Verilator V3Delayed internal error at the
            // larger ACTIVE_LIST_SIZE=64 (BIG_ROB). Behaviourally identical, so the
            // default 32-entry build is unchanged. (commit_packet_q/allocate_packet_q
            // below stay whole-array: they are OOO_WIDTH-sized and never grow.)
            for (int i = 0; i < ACTIVE_LIST_SIZE; i += 1)
                entries_q[i] <= entries_next[i];
            head_q <= head_next;
            tail_q <= tail_next;
            count_q <= count_next;
`ifndef COMMIT_1STAGE
            // R3/I4: suppress the ahead-present on a trap flush -- the present
            // this cycle shows entries YOUNGER than the trapping instruction
            // (all wrong-path), so they must never become next cycle's window.
            commit_valid_q  <= flush ? '0 : commit_valid_next;
            commit_packet_q <= commit_packet_next;
`endif /* COMMIT_1STAGE */
            // D->Q dispatch register: carry this cycle's allocate group to the Q
            // stage. Dispatch is suppressed on a flush/abort cycle (allocate_valid
            // is then 0), so the register drains naturally across recovery; a
            // wrong-path group that slips in is gated off at the Q write above.
            allocate_valid_q <= allocate_valid;
            allocate_packet_q <= allocate_packet;
        end
    end
`ifndef COMMIT_1STAGE

`ifndef SYNTHESIS
    // Invariant checks for the 2-stage commit (sim-only; see R2/R3/R5 contracts
    // in plans/rob-2stage-commit-audit.md). These convert the load-bearing
    // pointer/window invariants into runtime assertions.
    always_ff @(posedge clk) begin
        if (rst_l) begin
            // R5: ROB occupancy is consistent with the head/tail pointers.
            // active_distance wraps to 0 when head==tail, which is BOTH empty
            // (count 0) and full (count ACTIVE_LIST_SIZE) -- so the full case is
            // checked separately (head must equal tail), not via the distance.
            if (count_q > active_count_t'(ACTIVE_LIST_SIZE)) begin
                $error("active_list R5: count_q=%0d overflows ACTIVE_LIST_SIZE", count_q);
            end else if (count_q == active_count_t'(ACTIVE_LIST_SIZE)) begin
                if (head_q != tail_q)
                    $error("active_list R5: full (count=%0d) but head=%0d != tail=%0d",
                        count_q, head_q, tail_q);
            end else if (count_q != active_distance(head_q, tail_q)) begin
                $error("active_list R5: count_q=%0d != distance(head=%0d,tail=%0d)=%0d",
                    count_q, head_q, tail_q, active_distance(head_q, tail_q));
            end
            // R2: the registered window describes entries starting at head_q,
            // so a presented/retired lane i must name ROB entry head_q + i.
            for (int i = 0; i < OOO_WIDTH; i += 1) begin
                if (commit_valid_q[i] &&
                        (commit_packet_q[i].active_id != (head_q + active_id_t'(i)))) begin
                    $error("active_list R2: lane %0d active_id=%0d != head_q+%0d=%0d",
                        i, commit_packet_q[i].active_id, i,
                        (head_q + active_id_t'(i)));
                end
            end
            // R3: no retire the cycle after a flush (the ahead-present was
            // suppressed). $past is safe here (guarded by rst_l).
            if ($past(flush) && (commit_valid_q != '0)) begin
                $error("active_list R3: commit_valid_q=%b nonzero after flush",
                    commit_valid_q);
            end
        end
    end
`endif
`endif /* COMMIT_1STAGE */

endmodule: active_list
