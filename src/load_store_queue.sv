`include "ooo_types.vh"
`include "riscv_priv.vh"

`default_nettype none

module load_store_queue
    import OOO_Types::*;
    import RISCV_ISA::XLEN_BYTES;
    import RISCV_UArch::MEMORY_ADDR_WIDTH;
(
    input wire logic                 clk,
    input wire logic                 rst_l,
    input wire logic [OOO_WIDTH-1:0] insert_valid,
    input wire issue_entry_t         insert_entry [OOO_WIDTH],
    input wire logic [OOO_WIDTH-1:0][XLEN-1:0] insert_rs1_data,
    input wire logic [OOO_WIDTH-1:0][XLEN-1:0] insert_rs2_data,
    input wire logic [OOO_WIDTH-1:0] wakeup_valid,
    input wire phys_reg_t            wakeup_prd [OOO_WIDTH],
    input wire logic [OOO_WIDTH-1:0][XLEN-1:0] wakeup_data,
    input wire branch_mask_t         reset_mask,
    input wire branch_mask_t         abort_mask,
    // Full pipeline flush on a precise trap / interrupt / trap-return: discard
    // every queued memory op (all are younger than the trapping instruction).
    input wire logic                 flush,
    // Data-port load response (delivered by the memory subsystem after an
    // arbitrary >= 1 cycle latency; device reads are muxed into data_load by
    // the core). The queue has at most one load outstanding, so a response
    // always belongs to the single in-flight request (mem_inflight below);
    // no address matching is needed.
    input wire logic                 data_load_valid,
    input wire logic [XLEN-1:0]      data_load,
    // The memory subsystem can accept a data request (load issue or store
    // write beat) this cycle. Registered upstream; never depends on the
    // request being presented.
    input wire logic                 dmem_req_ready,
    input wire logic                 commit_store,
    input wire active_id_t           commit_store_id,
    // Sv32 data-side translation (driven by the core's MMU). When paging_data is
    // low the queue behaves exactly as before (identity mapping). When high, the
    // head's virtual address is exposed for the DTLB lookup, and the core feeds
    // back whether the translation is still walking (xlate_stall) or faulted.
    input wire logic                 paging_data,
    input wire logic                 xlate_stall,
    input wire logic                 xlate_fault,
    input wire logic [4:0]           xlate_cause,
    // Resolved physical address for the current mem_req_vaddr (driven by the
    // core MMU). Captured during the high-word probe of a cross-word store so
    // the second (fire-and-forget) write beat can target the correct PA after
    // the entry has retired.
    input wire logic [XLEN-1:0]      xlate_pa,
    output logic                 mem_req_valid,
    output logic [XLEN-1:0]      mem_req_vaddr,
    output logic                 mem_req_store,
    output logic                 full,
    output logic                 data_load_en,
    output logic [MEMORY_ADDR_WIDTH-1:0] data_addr,
    output logic [XLEN-1:0]      data_store,
    output logic [XLEN_BYTES-1:0] data_store_mask,
    // High while driving the second beat of a split store: the data_addr output
    // already carries the captured physical word address, so the core port must
    // bypass the (head-VA based) translation mux.
    output logic                 store_second_beat,
    // Hold the commit stage off for one cycle while the second store beat drains
    // so a younger store cannot collide on the single memory write port.
    output logic                 store_port_busy,
    // Byte offset of the in-order head load within the bus word, for
    // memory-mapped device read side effects (which 32-bit register is read).
    output logic [ADDR_SHIFT-1:0] head_load_off,
    output writeback_packet_t    load_writeback
);

    typedef struct packed {
        issue_entry_t entry;
        logic addr_ready;
        logic data_ready;
        logic issued_load;
        logic load_complete;
        logic double_low_valid;
        logic [XLEN-1:0] addr;
        logic [XLEN-1:0] load_low_word;
        logic [XLEN-1:0] store_data;
        logic [XLEN-1:0] store_data_upper;
        logic [XLEN_BYTES-1:0] store_mask;
        // Raw (unshifted) rs2 value for integer stores. The byte-offset shift
        // baked into store_data/store_mask depends on addr[1:0], which may not
        // be resolved when the data operand arrives; keeping the raw value lets
        // the formatted fields be re-derived once the address is known.
        logic [XLEN-1:0] store_raw;
        // High word of a two-beat store (cross-word misaligned, or the upper
        // word of an FP double). store_mask_hi is zero for single-beat stores.
        logic [XLEN-1:0] store_data_hi;
        logic [XLEN_BYTES-1:0]  store_mask_hi;
        // Captured physical word address of the high beat (from xlate_pa during
        // the high-word probe). Used to drive the second write after retire.
        logic [XLEN-1:0] store_hi_pa;
        // Captured physical address of the (low) beat, latched when the store
        // is proven translatable at the completion probe. The store then writes
        // memory at commit using THIS PA -- not a re-evaluated live translation,
        // which can have been evicted from the small DTLB by the time the store
        // reaches commit (a later cycle), orphaning the committed store.
        logic [XLEN-1:0] store_lo_pa;
    } mem_entry_t;

    // Byte-address -> word-address shift for the word-granular memory bus.
    localparam int ADDR_SHIFT = $clog2(XLEN_BYTES);

    mem_entry_t entries_q [MEM_Q_SIZE];
    mem_entry_t entries_next [MEM_Q_SIZE];
    logic [$clog2(MEM_Q_SIZE+1)-1:0] count_q, count_next;
    logic [$clog2(MEM_Q_SIZE)-1:0] head_q, head_next;
    logic [$clog2(MEM_Q_SIZE)-1:0] tail_q, tail_next;
    logic head_match, head_xlate_ok, head_xlate_flt;
    // Parallel leading-invalid-head skip (see the head-advance block): how many
    // consecutive squashed/empty slots sit at the head this cycle.
    logic [$clog2(MEM_Q_SIZE+1)-1:0] head_skip_n;
    logic head_skip_done;
    // Deferred head retire: the per-op head blocks (fault/misaligned-AMO/SC/
    // load-complete) set this instead of advancing head_next mid-block, so the
    // whole load/fault/issue/complete group indexes entries_next at a CONSTANT
    // head_next -> one shared 16:1 struct mux instead of one rebuilt per block.
    // The advance is applied once below, before the store-commit block (which
    // must see the post-retire head to allow a load-complete + store-commit in
    // the same cycle). At most one of those blocks fires per cycle (they share
    // the single load_writeback port and a fault gates head_xlate_ok off), so
    // deferring the advance is correctness-preserving.
    logic head_retire;
    logic reservation_valid_q, reservation_valid_next;
    logic [XLEN-1:0] reservation_addr_q, reservation_addr_next;
    // Unused high-word outputs of the AMO result repositioning (atomics are
    // aligned, so they never split across memory words).
    logic [XLEN-1:0] amo_split_hi_data;
    logic [XLEN_BYTES-1:0] amo_split_hi_mask;
    logic double_store_pending_q, double_store_pending_next;
    logic [MEMORY_ADDR_WIDTH-1:0] double_store_addr_q, double_store_addr_next;
    logic [XLEN-1:0] double_store_data_q, double_store_data_next;
    logic [XLEN_BYTES-1:0]  double_store_mask_q, double_store_mask_next;
    // One load may be outstanding at the memory port. mem_inflight tracks it;
    // mem_inflight_kill marks an outstanding response whose requesting entry
    // was squashed (branch abort / trap flush) so the response is discarded on
    // arrival instead of being delivered to an unrelated newer head. No new
    // load issues until the stale response drains (mem_inflight stays set).
    logic mem_inflight_q, mem_inflight_next;
    logic mem_inflight_kill_q, mem_inflight_kill_next;
    // High while the head store is probing its high-word translation (cross-word
    // store / FP double). Steers mem_req_vaddr up one word so the second page is
    // translated and verified before either word is written (store atomicity).
    logic        store_probe_hi_q, store_probe_hi_next;

    assign full = (count_q > MEM_Q_SIZE - OOO_WIDTH);
    assign store_port_busy = double_store_pending_q;

    // Expose the registered head's virtual address so the core can translate it
    // (DTLB lookup / PTW). Only meaningful once the address operand is resolved.
    assign mem_req_valid = entries_q[head_q].entry.valid &&
        entries_q[head_q].addr_ready &&
        (entries_q[head_q].entry.ctrl.memRead ||
         entries_q[head_q].entry.ctrl.memWrite);
    // While probing the high word of a two-beat store, expose the high-word VA
    // (one word up) so the MMU translates/faults the second page. Adding 4
    // increments the word index and preserves the byte offset.
    assign mem_req_vaddr = entries_q[head_q].addr +
        (store_probe_hi_q ? XLEN'(XLEN_BYTES) : '0);
    // AMOs read and write; treat as a store so the walker checks W permission.
    assign mem_req_store = entries_q[head_q].entry.ctrl.memWrite;

    // The in-order head load's byte offset (devices snoop the returned load
    // address, which lines up with the head load this matches).
    assign head_load_off  = entries_q[head_q].addr[ADDR_SHIFT-1:0];


    always_comb begin
        entries_next = entries_q;
        count_next = count_q;
        head_next = head_q;
        tail_next = tail_q;
        data_load_en = 1'b0;
        data_addr = '0;
        data_store = '0;
        data_store_mask = '0;
        load_writeback = '0;
        store_second_beat = 1'b0;
        reservation_valid_next = reservation_valid_q;
        reservation_addr_next = reservation_addr_q;
        double_store_pending_next = 1'b0;
        double_store_addr_next = '0;
        double_store_data_next = '0;
        double_store_mask_next = '0;
        store_probe_hi_next = 1'b0;
        mem_inflight_next = mem_inflight_q;
        mem_inflight_kill_next = mem_inflight_kill_q;
        head_retire = 1'b0;

        // A delivered response frees the single outstanding-load slot whether
        // it is consumed (match block below) or discarded as stale (killed /
        // squashed entry -- the match conditions then simply fail).
        if (data_load_valid) begin
            mem_inflight_next = 1'b0;
            mem_inflight_kill_next = 1'b0;
        end

        if (double_store_pending_q) begin
            data_addr = double_store_addr_q;
            data_store = double_store_data_q;
            data_store_mask = double_store_mask_q;
            // data_addr already holds the captured physical word address; tell
            // the core port to use it directly (skip the head-VA translation).
            store_second_beat = 1'b1;
            // Hold the (fire-and-forget) second beat until the memory port
            // accepts it.
            if (!dmem_req_ready) begin
                double_store_pending_next = 1'b1;
                double_store_addr_next = double_store_addr_q;
                double_store_data_next = double_store_data_q;
                double_store_mask_next = double_store_mask_q;
            end
        end

        for (int i = 0; i < MEM_Q_SIZE; i += 1) begin
            if ((entries_next[i].entry.branch_mask & abort_mask) != '0) begin
                entries_next[i] = '0;
            end else if (entries_next[i].entry.valid) begin
                entries_next[i].entry.branch_mask &= ~reset_mask;
                for (int w = 0; w < OOO_WIDTH; w += 1) begin
                    if (wakeup_valid[w]) begin
                        if (entries_next[i].entry.prs1 == wakeup_prd[w]) begin
                            entries_next[i].entry.src1_ready = 1'b1;
                            entries_next[i].addr_ready = 1'b1;
                            entries_next[i].addr = wakeup_data[w] +
                                ((entries_next[i].entry.ctrl.exec_class == EXEC_AMO) ?
                                 '0 : entries_next[i].entry.imm);
                        end
                        if ((entries_next[i].entry.prs2 == wakeup_prd[w]) &&
                                !((entries_next[i].entry.ctrl.exec_class == EXEC_FP) &&
                                  entries_next[i].entry.ctrl.memWrite)) begin
                            entries_next[i].entry.src2_ready = 1'b1;
                            entries_next[i].data_ready = 1'b1;
                            entries_next[i].store_raw = wakeup_data[w];
                            format_store_split(
                                entries_next[i].entry.ctrl.ldst_mode,
                                entries_next[i].addr[ADDR_SHIFT-1:0],
                                wakeup_data[w],
                                entries_next[i].store_data,
                                entries_next[i].store_mask,
                                entries_next[i].store_data_hi,
                                entries_next[i].store_mask_hi);
                        end
                    end
                end
            end
        end

        // The single in-flight load is always owned by the registered head
        // entry (the head cannot advance past an issued, incomplete load). If
        // a branch abort squashes that entry while its response is still
        // outstanding, mark the response stale so it is discarded on arrival.
        // (A response arriving in the same cycle already freed the slot above
        // and fails the match conditions against the squashed entry.)
        if (mem_inflight_q && !data_load_valid &&
                ((entries_q[head_q].entry.branch_mask & abort_mask) != '0)) begin
            mem_inflight_kill_next = 1'b1;
        end

        // Re-derive the byte-offset-dependent store fields for plain stores
        // once both the address and data operands are resolved. This is
        // idempotent and corrects the case where the data arrived (and was
        // formatted) before the address, so the byte offset was not yet known.
        // AMO/SC stores are word-aligned (offset always 0, and their
        // store_data may be an amo_result) and are left alone. On RV32, FP
        // stores are full-word (offset-independent) and also skipped; on RV64
        // they go through the same byte-split path (their data -- store_raw =
        // fp_src2_data -- is always present from dispatch, so only the address
        // needs to have resolved).
        for (int i = 0; i < MEM_Q_SIZE; i += 1) begin
            if (entries_next[i].entry.valid &&
                    entries_next[i].entry.ctrl.memWrite &&
                    !entries_next[i].entry.ctrl.memRead &&
                    (entries_next[i].entry.ctrl.exec_class != EXEC_AMO) &&
                    entries_next[i].addr_ready &&
`ifdef RV64
                    ((entries_next[i].entry.ctrl.exec_class == EXEC_FP) ||
                     entries_next[i].data_ready)
`else
                    (entries_next[i].entry.ctrl.exec_class != EXEC_FP) &&
                    entries_next[i].data_ready
`endif
                    ) begin
                format_store_split(
                    entries_next[i].entry.ctrl.ldst_mode,
                    entries_next[i].addr[ADDR_SHIFT-1:0],
                    entries_next[i].store_raw,
                    entries_next[i].store_data,
                    entries_next[i].store_mask,
                    entries_next[i].store_data_hi,
                    entries_next[i].store_mask_hi);
            end
        end

        // Advance the head over leading invalid (squashed/empty) slots in one
        // shot. Behaviorally identical to the former sequential ripple (which
        // re-indexed entries_next[head_next] with a rippling index, rebuilding a
        // 16:1 struct mux and a head_next add every iteration -- a ~16-deep
        // combinational chain), but here each slot is read at a CONSTANT offset
        // (head_next + k), so the 16 validity reads are independent and only the
        // leading-count accumulates -- a far shallower cone. head_next is still
        // == head_q at this point, so the skip must reach the first valid head
        // THIS cycle (a store committed by the ROB the same cycle its predecessor
        // hole is skipped relies on it -- a single-step skip would drop it).
        head_skip_n = '0;
        head_skip_done = 1'b0;
        for (int k = 0; k < MEM_Q_SIZE; k += 1) begin
            if (!head_skip_done && (head_skip_n < count_next) &&
                    !entries_next[head_next +
                        ($clog2(MEM_Q_SIZE))'(k)].entry.valid) begin
                head_skip_n = head_skip_n + 1'b1;
            end else begin
                head_skip_done = 1'b1;
            end
        end
        head_next  = head_next  + head_skip_n[$clog2(MEM_Q_SIZE)-1:0];
        count_next = count_next - head_skip_n;

        // ---- Sv32 data translation gating ----
        // Under paging, a head memory op may only touch memory once its
        // translation is established (DTLB hit, walk done, no fault) and the
        // registered head matches the entry currently processed (so the DTLB
        // lookup driven from entries_q[head_q] corresponds to this access).
        // xlate_fault now covers both translation page faults (under paging) and
        // PMP access faults (any mode, including bare), so the fault path is no
        // longer gated on paging_data. mem_req_valid (the registered head
        // address) is now required even in bare mode: it is what the core
        // translates / PMP-checks, so a memory op must not complete off the
        // combinational src-ready a cycle before the fault becomes visible. With
        // paging off and no fault this matches the original (proceed to memory).
        head_match     = (head_next == head_q);
        head_xlate_ok  = !xlate_fault && mem_req_valid &&
            (!paging_data || (head_match && !xlate_stall));
        head_xlate_flt = head_match && mem_req_valid && xlate_fault;

        // Faulting access: retire it with an exception instead of touching memory.
        if (head_xlate_flt && !double_store_pending_q &&
                entries_next[head_next].entry.valid &&
                (entries_next[head_next].entry.ctrl.memRead ||
                 entries_next[head_next].entry.ctrl.memWrite) &&
                entries_next[head_next].entry.src1_ready &&
                !entries_next[head_next].issued_load) begin
            load_writeback.valid = 1'b1;
            load_writeback.active_id = entries_next[head_next].entry.active_id;
            load_writeback.prd = entries_next[head_next].entry.prd;
            load_writeback.has_dest = entries_next[head_next].entry.has_dest;
            load_writeback.branch_mask = entries_next[head_next].entry.branch_mask;
            load_writeback.exception = 1'b1;
            load_writeback.exc_cause = xlate_cause;
            load_writeback.data = entries_next[head_next].addr;   // mtval = VA
            entries_next[head_next] = '0;
            head_retire = 1'b1;
        end

        // Misaligned AMO / LR / SC: the implementation does not support
        // misaligned atomics (MISALIGNED_AMO=false, LR/SC "always raise access
        // fault"), so an unaligned address retires with an access fault and
        // never touches memory or the reservation. Gated by head_match so it
        // only acts on the registered head (and never clobbers a writeback the
        // fault block above already produced for a different entry).
        if (head_match && !double_store_pending_q &&
                entries_next[head_next].entry.valid &&
                (entries_next[head_next].entry.ctrl.exec_class == EXEC_AMO) &&
                entries_next[head_next].entry.src1_ready &&
                !entries_next[head_next].issued_load &&
                ((entries_next[head_next].addr[1:0] != 2'b00) ||
                 ((entries_next[head_next].entry.ctrl.ldst_mode == LDST_D) &&
                  entries_next[head_next].addr[2]))) begin
            load_writeback.valid = 1'b1;
            load_writeback.active_id = entries_next[head_next].entry.active_id;
            load_writeback.prd = entries_next[head_next].entry.prd;
            load_writeback.has_dest = entries_next[head_next].entry.has_dest;
            load_writeback.branch_mask = entries_next[head_next].entry.branch_mask;
            load_writeback.exception = 1'b1;
            load_writeback.exc_cause =
                (entries_next[head_next].entry.ctrl.amo_op == AMO_LR) ?
                    RISCV_Priv::EXC_LOAD_ACCESS : RISCV_Priv::EXC_STORE_ACCESS;
            load_writeback.data = entries_next[head_next].addr;   // mtval = VA
            entries_next[head_next] = '0;
            head_retire = 1'b1;
        end

        if (head_xlate_ok && !double_store_pending_q && entries_next[head_next].entry.valid &&
                (entries_next[head_next].entry.ctrl.exec_class == EXEC_AMO) &&
                (entries_next[head_next].entry.ctrl.amo_op == AMO_SC) &&
                entries_next[head_next].entry.src1_ready &&
                entries_next[head_next].entry.src2_ready &&
                !entries_next[head_next].issued_load) begin
            load_writeback.valid = 1'b1;
            load_writeback.active_id = entries_next[head_next].entry.active_id;
            load_writeback.prd = entries_next[head_next].entry.prd;
            load_writeback.has_dest = entries_next[head_next].entry.has_dest;
            load_writeback.branch_mask = entries_next[head_next].entry.branch_mask;
            load_writeback.data = (reservation_valid_q &&
                (reservation_addr_q == entries_next[head_next].addr)) ? '0 : XLEN'(1);
            if (load_writeback.data == '0) begin
                // SC succeeds: capture its PA for the commit write-back.
                entries_next[head_next].store_lo_pa = xlate_pa;
                entries_next[head_next].issued_load = 1'b1;
                reservation_valid_next = 1'b0;
            end else begin
                entries_next[head_next] = '0;
                head_retire = 1'b1;
            end
        end

        if (head_xlate_ok && !double_store_pending_q && entries_next[head_next].entry.valid &&
                entries_next[head_next].entry.ctrl.memRead &&
                entries_next[head_next].entry.src1_ready && !entries_next[head_next].issued_load &&
                !mem_inflight_q && dmem_req_ready) begin
            data_load_en = 1'b1;
            data_addr = entries_next[head_next].addr[XLEN-1:ADDR_SHIFT];
            // Capture the PA for an AMO's write-back beat (same address it read);
            // harmless for a pure load.
            entries_next[head_next].store_lo_pa = xlate_pa;
            entries_next[head_next].issued_load = 1'b1;
            mem_inflight_next = 1'b1;
        end

        // Pure-store completion / cross-word probe. A single-word store marks
        // itself complete as soon as its (low) translation resolves. A two-beat
        // store (cross-word misaligned, or FP double) must additionally verify
        // the high word's translation BEFORE it is allowed to commit, so that a
        // page fault on the second word is reported as the store's exception and
        // no partial write is ever performed (store atomicity).
        if (head_xlate_ok && !double_store_pending_q && entries_next[head_next].entry.valid &&
                entries_next[head_next].entry.ctrl.memWrite &&
                entries_next[head_next].entry.src1_ready &&
                entries_next[head_next].entry.src2_ready &&
                !entries_next[head_next].entry.ctrl.memRead &&
                !entries_next[head_next].issued_load) begin
            if (!needs_two_beats(entries_next[head_next].entry.ctrl,
                    entries_next[head_next].addr)) begin
                // Single-beat store proven translatable: latch its PA and mark
                // it complete. The commit write below uses this latched PA.
                entries_next[head_next].store_lo_pa = xlate_pa;
                load_writeback.valid = 1'b1;
                load_writeback.active_id = entries_next[head_next].entry.active_id;
                load_writeback.branch_mask = entries_next[head_next].entry.branch_mask;
                load_writeback.has_dest = 1'b0;
                entries_next[head_next].issued_load = 1'b1;
            end else if (!store_probe_hi_q) begin
                // Low word translated OK: capture its PA, then probe the high
                // word next cycle.
                entries_next[head_next].store_lo_pa = xlate_pa;
                store_probe_hi_next = 1'b1;
            end else begin
                // High word translated OK; capture its PA and complete.
                entries_next[head_next].store_hi_pa = xlate_pa;
                load_writeback.valid = 1'b1;
                load_writeback.active_id = entries_next[head_next].entry.active_id;
                load_writeback.branch_mask = entries_next[head_next].entry.branch_mask;
                load_writeback.has_dest = 1'b0;
                entries_next[head_next].issued_load = 1'b1;
                store_probe_hi_next = 1'b0;
            end
        end else if (!double_store_pending_q && entries_next[head_next].entry.valid &&
                entries_next[head_next].entry.ctrl.memWrite &&
                entries_next[head_next].entry.src1_ready &&
                entries_next[head_next].entry.src2_ready &&
                !entries_next[head_next].entry.ctrl.memRead &&
                !entries_next[head_next].issued_load &&
                store_probe_hi_q) begin
            // High-word translation still walking: hold the probe phase.
            store_probe_hi_next = 1'b1;
        end

        if (!double_store_pending_q && entries_next[head_next].entry.valid &&
                entries_next[head_next].entry.ctrl.memRead &&
                entries_next[head_next].issued_load && data_load_valid &&
                !entries_next[head_next].load_complete &&
                mem_inflight_q && !mem_inflight_kill_q) begin
            load_writeback.valid = 1'b1;
            load_writeback.active_id = entries_next[head_next].entry.active_id;
            load_writeback.prd = entries_next[head_next].entry.prd;
            load_writeback.has_dest = entries_next[head_next].entry.has_dest;
            load_writeback.branch_mask = entries_next[head_next].entry.branch_mask;
            if (entries_next[head_next].entry.ctrl.exec_class == EXEC_AMO) begin
                // rd gets the old memory value at the access width (an AMO.W on
                // RV64 sign-extends bit 31, and may sit at offset 4 of the
                // 8-byte memory word).
                load_writeback.data = format_load(data_load,
                    entries_next[head_next].addr[ADDR_SHIFT-1:0],
                    entries_next[head_next].entry.ctrl.ldst_mode);
                if (entries_next[head_next].entry.ctrl.amo_op == AMO_LR) begin
                    reservation_valid_next = 1'b1;
                    reservation_addr_next = entries_next[head_next].addr;
                    entries_next[head_next] = '0;
                    head_retire = 1'b1;
                end else begin
                    // Compute on the raw (unshifted) operands at the access
                    // width, then position the result back into its memory
                    // word with the matching byte-enable mask.
                    format_store_split(
                        entries_next[head_next].entry.ctrl.ldst_mode,
                        entries_next[head_next].addr[ADDR_SHIFT-1:0],
                        amo_result(entries_next[head_next].entry.ctrl.amo_op,
                            load_writeback.data,
                            entries_next[head_next].store_raw,
                            entries_next[head_next].entry.ctrl.ldst_mode != LDST_D),
                        entries_next[head_next].store_data,
                        entries_next[head_next].store_mask,
                        amo_split_hi_data,
                        amo_split_hi_mask);
                    entries_next[head_next].load_complete = 1'b1;
                end
            end else begin
                if (needs_two_beats(entries_next[head_next].entry.ctrl,
                        entries_next[head_next].addr) &&
                        !entries_next[head_next].double_low_valid) begin
                    // First beat of a two-beat load (cross-word misaligned, or
                    // the low word of an FP double): stash it, advance one word,
                    // and re-issue. Adding 4 preserves the byte offset used for
                    // the final extraction.
                    load_writeback = '0;
                    entries_next[head_next].load_low_word = data_load;
                    entries_next[head_next].double_low_valid = 1'b1;
                    entries_next[head_next].issued_load = 1'b0;
                    entries_next[head_next].addr =
                        entries_next[head_next].addr + XLEN'(XLEN_BYTES);
                end else begin
                    if (entries_next[head_next].double_low_valid) begin
                        // Final beat: combine {high, low} and extract the
                        // requested bytes at the original byte offset.
                        load_writeback.data = format_load_wide(
                            {data_load, entries_next[head_next].load_low_word},
                            entries_next[head_next].addr[ADDR_SHIFT-1:0],
                            entries_next[head_next].entry.ctrl.ldst_mode);
                    end else begin
                        load_writeback.data = format_load(data_load,
                            entries_next[head_next].addr[ADDR_SHIFT-1:0],
                            entries_next[head_next].entry.ctrl.ldst_mode);
                    end
                    if (entries_next[head_next].entry.ctrl.exec_class == EXEC_FP) begin
                    load_writeback.fp_write =
                        entries_next[head_next].entry.ctrl.fp_writes_fpr;
                    load_writeback.fp_rd = entries_next[head_next].entry.fp_rd;
`ifdef RV64
                    // load_writeback.data was just extracted at the access
                    // width/offset (FLD decodes LDST_D, FLW LDST_W), so an FLD
                    // is the full value and an FLW NaN-boxes its low word --
                    // correct for any byte offset and for misaligned splits.
                    load_writeback.fp_data = entries_next[head_next].entry.ctrl.fp_double ?
                        load_writeback.data :
                        {32'hffff_ffff, load_writeback.data[31:0]};
`else
                    load_writeback.fp_data = entries_next[head_next].entry.ctrl.fp_double ?
                        {data_load, entries_next[head_next].load_low_word} :
                        {32'hffff_ffff, data_load};
`endif
                    load_writeback.has_dest = 1'b0;
                    end
                    entries_next[head_next] = '0;
                    head_retire = 1'b1;
                end
            end
        end

        // Apply the deferred head advance from the per-op blocks above (fault /
        // misaligned-AMO / SC-fail / load-complete -- at most one fires) before
        // the store-commit block, so store-commit sees the post-retire head and
        // the load-complete + store-commit two-retire-per-cycle case still works.
        // Keeping head_next constant through all the per-op blocks lets them share
        // a single entries_next[head_next] mux instead of rebuilding one each.
        if (head_retire) begin
            head_next = head_next + 1'b1;
            count_next -= 1'b1;
        end

        // Store commit: the committing store was already proven translatable at
        // its completion probe (which latched store_lo_pa), so write it using
        // that captured PA -- NOT a re-evaluated live translation (head_xlate_ok),
        // whose DTLB entry can have been evicted between the probe and commit,
        // leaving the committed store orphaned and wedging the queue. store_lo_pa
        // is a physical address, so assert store_second_beat to tell the core
        // port to use data_addr directly (skip the head-VA translation mux).
        if (!double_store_pending_q && commit_store && entries_next[head_next].entry.valid &&
                entries_next[head_next].entry.ctrl.memWrite &&
                (entries_next[head_next].entry.active_id == commit_store_id)) begin
            // Read the committing store's data/PA/mask from the REGISTERED entry
            // (entries_q), not entries_next. entries_next recomputes store_data /
            // store_data_hi via format_store_split combinationally every cycle
            // (the re-derive loop), which dragged that 128-bit shift + the wakeup
            // chain into the commit-data / double_store cone -- the FB2b worst
            // path (abort_mask -> double_store_data_q). For a committing store
            // these fields were finalized cycles ago (operands long resolved, at
            // the ROB head so branch_mask==0 -> no squash), so entries_q ==
            // entries_next for them; the registered read is value-equivalent and
            // shallow (just the head mux, no recompute). The CONDITION still uses
            // entries_next so a same-cycle squash/zeroing is honored.
            // First (low) beat: written at the captured physical address.
            data_addr = entries_q[head_next].store_lo_pa[XLEN-1:ADDR_SHIFT];
            data_store = entries_q[head_next].store_data;
            data_store_mask = entries_q[head_next].store_mask;
            store_second_beat = 1'b1;
            // Second (high) beat of a two-beat store: queue a fire-and-forget
            // write at the high word's captured physical address. The high page
            // was already proven fault-free during the probe above, so this
            // write cannot fault. store_second_beat tells the core port to use
            // this PA directly (the entry will have retired by then).
            if (needs_two_beats(entries_q[head_next].entry.ctrl,
                    entries_q[head_next].addr)) begin
                double_store_pending_next = 1'b1;
                double_store_addr_next = entries_q[head_next].store_hi_pa[XLEN-1:ADDR_SHIFT];
                double_store_data_next = entries_q[head_next].store_data_hi;
                double_store_mask_next = entries_q[head_next].store_mask_hi;
            end
            if (reservation_valid_next &&
                    (reservation_addr_next == entries_q[head_next].addr)) begin
                reservation_valid_next = 1'b0;
            end
            entries_next[head_next] = '0;
            head_next = head_next + 1'b1;
            count_next -= 1'b1;
        end

        if (!full) begin
            for (int lane = 0; lane < OOO_WIDTH; lane += 1) begin
                if (insert_valid[lane]) begin
                    entries_next[tail_next].entry = insert_entry[lane];
                    entries_next[tail_next].entry.valid = 1'b1;
                    entries_next[tail_next].addr_ready = insert_entry[lane].src1_ready;
                    entries_next[tail_next].data_ready = insert_entry[lane].src2_ready;
                    entries_next[tail_next].issued_load = 1'b0;
                    entries_next[tail_next].load_complete = 1'b0;
                    entries_next[tail_next].double_low_valid = 1'b0;
                    entries_next[tail_next].load_low_word = '0;
                    entries_next[tail_next].store_hi_pa = '0;
                    entries_next[tail_next].store_lo_pa = '0;
                    entries_next[tail_next].store_raw =
                        (insert_entry[lane].ctrl.exec_class == EXEC_FP) ?
`ifdef RV64
                            // FP store data is positioned by the same
                            // format_store_split path as integer stores.
                            XLEN'(insert_entry[lane].fp_src2_data)
`else
                            '0
`endif
                            : insert_rs2_data[lane];
                    if (insert_entry[lane].src1_ready) begin
                        entries_next[tail_next].addr = insert_rs1_data[lane] +
                            ((insert_entry[lane].ctrl.exec_class == EXEC_AMO) ?
                             '0 : insert_entry[lane].imm);
                    end else begin
                        entries_next[tail_next].addr = '0;
                    end
                    if (insert_entry[lane].src2_ready ||
                            insert_entry[lane].ctrl.fp_uses_rs2) begin
                        if (insert_entry[lane].ctrl.exec_class == EXEC_FP) begin
`ifdef RV64
                            // FSD decodes LDST_D / FSW LDST_W, so FP stores
                            // position through the generic byte-split path
                            // exactly like integer stores.
                            format_store_split(insert_entry[lane].ctrl.ldst_mode,
                                entries_next[tail_next].addr[ADDR_SHIFT-1:0],
                                XLEN'(insert_entry[lane].fp_src2_data),
                                entries_next[tail_next].store_data,
                                entries_next[tail_next].store_mask,
                                entries_next[tail_next].store_data_hi,
                                entries_next[tail_next].store_mask_hi);
                            entries_next[tail_next].store_data_upper = '0;
`else
                            // FP store: each word is full-width (FSW = low word
                            // only; FSD = both words). No byte-level splitting.
                            entries_next[tail_next].store_data =
                                insert_entry[lane].fp_src2_data[31:0];
                            entries_next[tail_next].store_mask = 4'b1111;
                            entries_next[tail_next].store_data_hi =
                                insert_entry[lane].fp_src2_data[63:32];
                            entries_next[tail_next].store_mask_hi =
                                insert_entry[lane].ctrl.fp_double ? 4'b1111 : 4'b0000;
                            entries_next[tail_next].store_data_upper =
                                insert_entry[lane].fp_src2_data[63:32];
`endif
                        end else begin
                            format_store_split(insert_entry[lane].ctrl.ldst_mode,
                                entries_next[tail_next].addr[ADDR_SHIFT-1:0],
                                insert_rs2_data[lane],
                                entries_next[tail_next].store_data,
                                entries_next[tail_next].store_mask,
                                entries_next[tail_next].store_data_hi,
                                entries_next[tail_next].store_mask_hi);
                            entries_next[tail_next].store_data_upper = '0;
                        end
                    end else begin
                        entries_next[tail_next].store_data = '0;
                        entries_next[tail_next].store_data_upper = '0;
                        entries_next[tail_next].store_mask = '0;
                        entries_next[tail_next].store_data_hi = '0;
                        entries_next[tail_next].store_mask_hi = '0;
                    end
                    tail_next = tail_next + 1'b1;
                    count_next += 1'b1;
                end
            end
        end

        // Precise-trap flush: every queued op is younger than the trapping
        // instruction, so discard them all. Suppress any speculative
        // memory/writeback effects driven above this cycle. The LR reservation
        // is committed architectural state and is intentionally preserved.
        //
        // Store WRITES driven this cycle are NOT suppressed: they belong to
        // stores that already committed (a pending/held second beat from an
        // earlier commit, or a store retiring in the same commit group as --
        // and architecturally older than -- the trapping instruction), so the
        // write outputs and any scheduled/held second-beat drain state are
        // left exactly as driven above.
        if (flush) begin
            for (int i = 0; i < MEM_Q_SIZE; i += 1) begin
                entries_next[i] = '0;
            end
            head_next = '0;
            tail_next = '0;
            count_next = '0;
            data_load_en = 1'b0;
            load_writeback = '0;
            store_probe_hi_next = 1'b0;
            // A load issue suppressed by this flush (data_load_en cleared
            // above) must not leave a phantom in-flight slot; a load that was
            // already outstanding before this cycle stays outstanding and its
            // eventual response is discarded as stale.
            mem_inflight_next = mem_inflight_q && !data_load_valid;
            mem_inflight_kill_next = mem_inflight_q && !data_load_valid;
        end
    end

    // Access size in bytes implied by the load/store mode.
    function automatic logic [4:0] mem_size(input ldst_mode_t mode);
        unique case (mode)
            LDST_D:          mem_size = 5'd8;
            LDST_W, LDST_WU: mem_size = 5'd4;
            LDST_H, LDST_HU: mem_size = 5'd2;
            LDST_B, LDST_BU: mem_size = 5'd1;
            default:         mem_size = 5'(XLEN_BYTES);
        endcase
    endfunction

    // True when an access at byte offset byte_sel spills past the end of its
    // containing memory word (XLEN_BYTES) and must be split into two word beats.
    function automatic logic mem_crosses(input ldst_mode_t mode,
            input logic [ADDR_SHIFT-1:0] byte_sel);
        mem_crosses = (({1'b0, byte_sel} + mem_size(mode)) > 5'(XLEN_BYTES));
    endfunction

    // A memory op needs a second word beat when it is an FP double (FSD/FLD) or
    // an integer access that crosses a word boundary. AMO/LR/SC never split
    // (a misaligned atomic raises an access fault instead).
    function automatic logic needs_two_beats(input ctrl_signals_t ctrl,
            input logic [XLEN-1:0] addr);
        // An 8-byte FP double (FLD/FSD) crosses the memory word whenever the
        // word is narrower than 8 bytes (RV32, always) or the access is not
        // 8-byte aligned within an 8-byte word (RV64).
        needs_two_beats =
            ((ctrl.exec_class == EXEC_FP) && ctrl.fp_double &&
             ((XLEN_BYTES < 8) || (addr[ADDR_SHIFT-1:0] != '0))) ||
            ((ctrl.exec_class != EXEC_AMO) &&
             mem_crosses(ctrl.ldst_mode, addr[ADDR_SHIFT-1:0]));
    endfunction

    // Position a store value and byte-enable mask across (up to) two memory words
    // at an arbitrary byte offset. The high word's mask is zero for accesses that
    // fit within a single word (including within-word misaligned ones).
    task automatic format_store_split(input ldst_mode_t mode,
            input logic [ADDR_SHIFT-1:0] byte_sel,
            input logic [XLEN-1:0] value,
            output logic [XLEN-1:0] store_value_lo,
            output logic [XLEN_BYTES-1:0]  store_mask_lo,
            output logic [XLEN-1:0] store_value_hi,
            output logic [XLEN_BYTES-1:0]  store_mask_hi);
        logic [2*XLEN-1:0] shifted;
        logic [2*XLEN_BYTES-1:0] maskw;
        logic [XLEN_BYTES-1:0]   full_mask;
        unique case (mode)
            LDST_D:          full_mask = '1;
            LDST_W, LDST_WU: full_mask = XLEN_BYTES'('hF);
            LDST_H, LDST_HU: full_mask = XLEN_BYTES'('h3);
            LDST_B, LDST_BU: full_mask = XLEN_BYTES'('h1);
            default:         full_mask = '0;
        endcase
        shifted = ({{XLEN{1'b0}}, value}) << {byte_sel, 3'b0};
        maskw   = ({{XLEN_BYTES{1'b0}}, full_mask}) << byte_sel;
        store_value_lo = shifted[XLEN-1:0];
        store_value_hi = shifted[2*XLEN-1:XLEN];
        store_mask_lo  = maskw[XLEN_BYTES-1:0];
        store_mask_hi  = maskw[2*XLEN_BYTES-1:XLEN_BYTES];
    endtask

    // Extract and sign/zero-extend a value contained within a single memory word
    // at byte offset byte_sel.
    function automatic logic [XLEN-1:0] format_load(input logic [XLEN-1:0] raw_word,
            input logic [ADDR_SHIFT-1:0] byte_sel, input ldst_mode_t mode);
        logic [XLEN-1:0] sh;
        sh = raw_word >> {byte_sel, 3'b0};
        unique case (mode)
            LDST_D:  format_load = raw_word;
            LDST_W:  format_load = {{(XLEN-32){sh[31]}}, sh[31:0]};
            LDST_WU: format_load = {{(XLEN-32){1'b0}},   sh[31:0]};
            LDST_H:  format_load = {{(XLEN-16){sh[15]}}, sh[15:0]};
            LDST_HU: format_load = {{(XLEN-16){1'b0}},   sh[15:0]};
            LDST_B:  format_load = {{(XLEN-8){sh[7]}},   sh[7:0]};
            LDST_BU: format_load = {{(XLEN-8){1'b0}},    sh[7:0]};
            default: format_load = '0;
        endcase
    endfunction

    // Extract a value that straddles a word boundary from the {high,low} word
    // pair at byte offset byte_sel.
    function automatic logic [XLEN-1:0] format_load_wide(input logic [2*XLEN-1:0] pair,
            input logic [ADDR_SHIFT-1:0] byte_sel, input ldst_mode_t mode);
        logic [2*XLEN-1:0] sh;
        sh = pair >> {byte_sel, 3'b0};
        unique case (mode)
            LDST_D:  format_load_wide = sh[XLEN-1:0];
            LDST_W:  format_load_wide = {{(XLEN-32){sh[31]}}, sh[31:0]};
            LDST_WU: format_load_wide = {{(XLEN-32){1'b0}},   sh[31:0]};
            LDST_H:  format_load_wide = {{(XLEN-16){sh[15]}}, sh[15:0]};
            LDST_HU: format_load_wide = {{(XLEN-16){1'b0}},   sh[15:0]};
            LDST_B:  format_load_wide = {{(XLEN-8){sh[7]}},   sh[7:0]};
            LDST_BU: format_load_wide = {{(XLEN-8){1'b0}},    sh[7:0]};
            default: format_load_wide = '0;
        endcase
    endfunction

    // word_op: a 32-bit AMO (AMO*.W) — compute on the low 32 bits of the
    // operands. Sign- or zero-extending both to XLEN first makes the XLEN-wide
    // compare/arithmetic produce the correct 32-bit result in the low bits
    // (only the low 32 are stored back). Identity at XLEN=32.
    function automatic logic [XLEN-1:0] amo_result(input amo_op_t op,
            input logic [XLEN-1:0] old_value, input logic [XLEN-1:0] operand,
            input logic word_op);
        logic [XLEN-1:0] a, b;
        if (word_op && ((op == AMO_MINU) || (op == AMO_MAXU))) begin
            a = XLEN'(old_value[31:0]);
            b = XLEN'(operand[31:0]);
        end else if (word_op) begin
            a = XLEN'($signed(old_value[31:0]));
            b = XLEN'($signed(operand[31:0]));
        end else begin
            a = old_value;
            b = operand;
        end
        unique case (op)
            AMO_SWAP: amo_result = b;
            AMO_ADD:  amo_result = a + b;
            AMO_XOR:  amo_result = a ^ b;
            AMO_AND:  amo_result = a & b;
            AMO_OR:   amo_result = a | b;
            AMO_MIN:  amo_result = (signed'(a) < signed'(b)) ? a : b;
            AMO_MAX:  amo_result = (signed'(a) > signed'(b)) ? a : b;
            AMO_MINU: amo_result = (a < b) ? a : b;
            AMO_MAXU: amo_result = (a > b) ? a : b;
            default:  amo_result = a;
        endcase
    endfunction








    always_ff @(posedge clk or negedge rst_l) begin
        if (!rst_l) begin
            count_q <= '0;
            head_q <= '0;
            tail_q <= '0;
            reservation_valid_q <= 1'b0;
            reservation_addr_q <= '0;
            double_store_pending_q <= 1'b0;
            double_store_addr_q <= '0;
            double_store_data_q <= '0;
            double_store_mask_q <= '0;
            store_probe_hi_q <= 1'b0;
            mem_inflight_q <= 1'b0;
            mem_inflight_kill_q <= 1'b0;
            for (int i = 0; i < MEM_Q_SIZE; i += 1) begin
                entries_q[i] <= '0;
            end
        end else begin
            count_q <= count_next;
            head_q <= head_next;
            tail_q <= tail_next;
            reservation_valid_q <= reservation_valid_next;
            reservation_addr_q <= reservation_addr_next;
            double_store_pending_q <= double_store_pending_next;
            double_store_addr_q <= double_store_addr_next;
            double_store_data_q <= double_store_data_next;
            double_store_mask_q <= double_store_mask_next;
            store_probe_hi_q <= store_probe_hi_next;
            mem_inflight_q <= mem_inflight_next;
            mem_inflight_kill_q <= mem_inflight_kill_next;
            entries_q <= entries_next;
        end
    end

endmodule: load_store_queue
