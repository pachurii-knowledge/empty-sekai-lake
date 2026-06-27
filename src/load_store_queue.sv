`include "ooo_types.vh"
`include "riscv_priv.vh"

`default_nettype none

module load_store_queue
    import OOO_Types::*;
    import RISCV_ISA::XLEN_BYTES;
    import RISCV_UArch::MEMORY_ADDR_WIDTH;
#(
    // M4 B9: in a coherent (multi-core) build the SC defers its success/fail
    // decision and rd writeback to the commit-store cycle, re-checking the
    // (snoop-killed) reservation there. Default 0 => single-core builds elaborate
    // the verbatim head-decision path, so the netlist is bit-identical.
    parameter bit COHERENT = 1'b0
)
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
    // M3d Stage 3: snoop-kill from the CCD agent -- a remote write (FwdGetM/INV) to this line.
    // S2 uses it for the reservation coherence-kill; a future S5 for spec-load squash. Constant 0
    // in non-CCD + single-core CCD builds, so all consumers are inert and the baseline is identical.
    input wire logic                 snoop_kill_valid,
    input wire logic [MEMORY_ADDR_WIDTH-1:0] snoop_kill_laddr,
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
    // M3d Stage 2: typed memory-op code for the current data-port transaction.
    // Consumed by the CCD L1D agent adapter in niigo_memsys (mapped to l1_core_op_e);
    // ignored by the L1D/passthrough arms (they derive load/store from the write mask).
    // A load issue carries LOAD/LR/AMO_RD; a store write (commit / second beat) carries
    // STORE. The 3-bit contract MUST match the decode in niigo_memsys.sv's CCD arm.
    output logic [2:0]           dmem_req_op,
    // M4 #3: the fine AMO sub-op (amo_op_t ordinal) accompanying a COP_AMO beat,
    // so the CCD agent applies the right atomic RMW. Don't-care on non-AMO beats /
    // non-CCD builds (sunk by the memsys).
    output logic [3:0]           dmem_req_amo,
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
    // M4-S5b: pulses the cycle the LSQ resolves a coherent SC (the agent's COP_SC
    // sc_ok arrived + rd written). The commit unit holds the SC at the ROB head
    // until this fires, then retires it. Constant 0 when COHERENT=0.
    output logic                 sc_commit_done,
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

    // M3d Stage 2: dmem_req_op codes (see the port comment). The CCD adapter maps
    // these to l1_core_op_e; keep the two in lockstep.
    localparam logic [2:0] DMEM_OP_LOAD   = 3'd0;
    localparam logic [2:0] DMEM_OP_STORE  = 3'd1;
    localparam logic [2:0] DMEM_OP_LR     = 3'd2;
    localparam logic [2:0] DMEM_OP_AMO_RD = 3'd3;
    localparam logic [2:0] DMEM_OP_SC     = 3'd4;   // M4-S5b: agent-authoritative SC (COP_SC)
    localparam logic [2:0] DMEM_OP_AMO    = 3'd5;   // M4 #3: agent-authoritative AMO (COP_AMO)

    // M3d Stage 3: coherence line = 64 B (the fixed PIPT line). LINE_OFF_W = the word-offset
    // bits within a line, so [MEMORY_ADDR_WIDTH-1:LINE_OFF_W] of a word address is its line.
    // The CCD agent's snoop_kill_laddr is already line-aligned; masking the reservation PA's low
    // bits gives a line-granular compare (the RISC-V reservation set is a cache line).
    localparam int LINE_BYTES = 64;
    localparam int LINE_OFF_W = $clog2(LINE_BYTES) - ADDR_SHIFT;

    // FB2b false-loop break (LSQ wakeup<->load_writeback): the per-op head blocks
    // (HD) write the single head entry via sparse per-field write-enables instead
    // of indexing the shared entries_premerge array directly, so load_writeback's
    // cone never reads the wakeup-written array. The merge block (M) layers these
    // writes onto entries_wake (post squash/wakeup/rederive) -- exactly as the
    // original block layered the head-op writes onto post-wakeup entries_premerge.
    // Per-field we_* (not whole-entry replacement) preserves any same-cycle wakeup
    // update to fields the firing op does not touch -> robustly value-identical.
    typedef struct packed {
        logic                  zero;            // retire: write whole head entry = 0
        logic                  we_store_lo_pa;  logic [XLEN-1:0]       store_lo_pa;
        logic                  we_store_hi_pa;  logic [XLEN-1:0]       store_hi_pa;
        logic                  we_issued_load;  logic                 issued_load;
        // store_data and store_mask are always written together (AMO result).
        logic                  we_store_data;   logic [XLEN-1:0]       store_data;
                                                logic [XLEN_BYTES-1:0] store_mask;
        logic                  we_load_complete;   logic              load_complete;
        logic                  we_load_low_word;   logic [XLEN-1:0]    load_low_word;
        logic                  we_double_low_valid; logic             double_low_valid;
        logic                  we_addr;         logic [XLEN-1:0]       addr;
    } head_delta_t;

    mem_entry_t entries_q [MEM_Q_SIZE];
    mem_entry_t entries_next [MEM_Q_SIZE];
    // FB2b false-loop break: the main always_comb formed load_writeback (from the
    // REGISTERED head) AND read the insert operands (insert_rs*_data) -- a whole-
    // block alias that drew the false phys_rs_data -> load_writeback loop edge.
    // Split: block 1 computes entries_premerge (post squash/wakeup/head-ops/store-
    // commit, pre-insert) + load_writeback from registered state only; block 2 (the
    // insert, the sole insert_rs*_data reader) fills entries_premerge's free slots
    // into entries_next. Value-identical (insert never touches the occupied head).
    mem_entry_t entries_premerge [MEM_Q_SIZE];
    // FB2b LSQ split: entries_wake = post squash/wakeup/store-rederive, the ONLY
    // wakeup-reading product (block W). The head-op block (HD) and the head-skip
    // read registered state only, so load_writeback is severed from the wakeup
    // array (the false wakeup<->load_writeback loop edge).
    mem_entry_t entries_wake [MEM_Q_SIZE];
    head_delta_t head_delta;
    logic [$clog2(MEM_Q_SIZE+1)-1:0] count_q, count_next;
    logic [$clog2(MEM_Q_SIZE+1)-1:0] count_next_pre;
    logic [$clog2(MEM_Q_SIZE+1)-1:0] count_after_skip;
    logic [$clog2(MEM_Q_SIZE)-1:0] head_q, head_next;
    logic [$clog2(MEM_Q_SIZE)-1:0] head_next_skip;
    logic [$clog2(MEM_Q_SIZE)-1:0] tail_q, tail_next;
    // HD load-issue drive (muxed into the data port by M, behind double-store /
    // store-commit), and HD's post-head-op reservation (M applies store-commit clear).
    logic                          head_data_load_en;
    logic [MEMORY_ADDR_WIDTH-1:0]  load_data_addr;
    // M3d Stage 2: op code for the head load issue (LOAD / LR / AMO_RD), forwarded
    // by the data-port mux in block M when a load (not a store write) is driven.
    logic [2:0]                    head_load_op;
    logic                          reservation_valid_mid;
    logic [XLEN-1:0]               reservation_addr_mid;
    // M locals: head after the deferred retire advance, and whether a store
    // commits this cycle (drives the data port + the second-beat schedule).
    logic [$clog2(MEM_Q_SIZE)-1:0] head_after_retire;
    logic                          store_commit_fires;
    // M4-S5b (COHERENT only): agent-authoritative SC. The SC defers to commit, then
    // issues a COP_SC (atomic check-and-store at the agent) and awaits the agent's
    // sc_ok; rd is written from that response. sc_issue (block HD) drives the COP_SC
    // data beat in block M; sc_inflight_q tracks the outstanding COP_SC; sc_commit_done
    // (to the commit unit) releases the SC's ROB retire once the agent has answered.
    // All inert when COHERENT=0.
    logic                          sc_issue;
    logic                          sc_inflight_q, sc_inflight_next;
    // M4 #3 (COHERENT only): agent-authoritative AMO. Mirrors the SC -- the AMO
    // defers to commit, then issues a COP_AMO (the agent does acquire-M + atomic
    // RMW + return-OLD in one action, with snoop-replay during acquire), awaits the
    // OLD word, writes rd, and pulses sc_commit_done. amo_issue drives the COP_AMO
    // data beat in block M; amo_inflight_q tracks the outstanding COP_AMO. Inert
    // when COHERENT=0 (the AMO uses the Stage-2 COP_AMO_RD + commit-store path).
    logic                          amo_issue;
    logic                          amo_inflight_q, amo_inflight_next;
    // HD local: post-squash validity per slot (head-skip input, no wakeup).
    logic [MEM_Q_SIZE-1:0]         sq_valid;
    // HD locals: AMO result byte-split (low into head_delta, high unused).
    logic [XLEN-1:0]               amo_store_data;
    logic [XLEN_BYTES-1:0]         amo_store_mask;
    logic head_match, head_xlate_ok, head_xlate_flt;
    // Parallel leading-invalid-head skip (see the head-advance block): how many
    // consecutive squashed/empty slots sit at the head this cycle.
    logic [$clog2(MEM_Q_SIZE+1)-1:0] head_skip_n;
    // Registered snapshot of the (post-find-first) head entry. The per-op head
    // blocks (fault / misaligned-AMO / SC / load-issue / store-complete /
    // load-complete) read their conditions + data from THIS (entries_q) rather
    // than the combinationally-updated entries_next -- so the load AGU / data_addr
    // / store fields come from a shallow registered mux instead of the deep
    // abort->squash->wakeup->format chain (the FB2b LSQ worst path). Cost: a head
    // op fires one cycle after its operand is registered-ready (+1 load-use cycle);
    // the LSQ is latency-tolerant (mem_inflight) so this is functionally safe. The
    // head-op WRITES still target entries_next[head_next] (merging with this
    // cycle's wakeup/squash). The translation gating (mem_req_valid/head_xlate_ok)
    // is ALREADY registered-head based, so this makes the head op self-consistent.
    mem_entry_t headq;
    // At-most-one head-op per cycle. The per-op blocks formerly excluded each
    // other implicitly: an earlier block's entries_next write (retire ->
    // valid=0, or issued_load=1) was visible to a later block reading
    // entries_next. Now that they read the REGISTERED headq, that same-cycle
    // visibility is gone, so this explicit flag enforces the same single-fire
    // (a faulted/retired/handled head must not also be picked up by a later
    // block). store-commit is OUTSIDE this group (it runs on the post-retire
    // head, allowing the load-complete + store-commit two-retire-per-cycle case).
    logic head_done;
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
    // M3d Stage 3: a PHYSICAL line-address shadow of the reservation (word address), latched
    // alongside reservation_addr at LR completion from the LR's captured PA (store_lo_pa). The SC
    // success/fail test stays the VA compare (reservation_addr_q == headq.addr, byte-identical);
    // this shadow is read ONLY by the snoop coherence-kill (a remote write -> the reservation dies).
    logic [MEMORY_ADDR_WIDTH-1:0] reservation_pa_q, reservation_pa_next, reservation_pa_mid;
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

    // FB2b wall #2 -- registered head translation (the translate pipeline stage).
    // The core translates mem_req_vaddr (= entries_q[head_q].addr, registered)
    // combinationally through the DTLB + DataPMP; that result (xlate_pa/fault/
    // stall/cause) feeding the head fault/issue/retire decision in the SAME cycle
    // was the binding placed path (LSQ head -> DTLB -> DataPMP -> head_retire).
    // Register the translate result here and consume it ONE cycle later, with a
    // (head_q, store_probe_hi_q) validity tag so the registered translate is only
    // used when it belongs to the head request currently presented -- never a
    // stale translate from a different head or a walk transition. Cost: +1 cycle
    // on the head translate (load issue / store complete / fault); the LSQ is
    // latency-tolerant (single outstanding load via mem_inflight). mem_req_vaddr
    // is a pure function of (head_q, store_probe_hi_q), so matching both is exact.
    logic [XLEN-1:0] xlate_pa_q;
    logic            xlate_fault_q;
    logic            xlate_stall_q;
    logic [4:0]      xlate_cause_q;
    logic [$clog2(MEM_Q_SIZE)-1:0] xlate_head_q;  // head_q the translate was for
    logic            xlate_probe_q;               // store_probe_hi_q ditto
    logic            xlate_reqv_q;                // mem_req_valid ditto
    logic            xlate_ready;                 // registered translate is current

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


    // ==== Block HD: head-skip + registered head snapshot + per-op head blocks ====
    // Reads entries_q + abort_mask (inline squash-valid) + registered control
    // state ONLY. Produces load_writeback + head_delta (sparse per-field head-entry
    // writes) + the load-issue drive + mem_inflight/reservation_mid/store_probe.
    // It does NOT read wakeup or entries_wake, so load_writeback's cone is severed
    // from the wakeup-written array -- breaking the false wakeup<->load_writeback
    // loop. The merge block (M) layers head_delta onto entries_wake.
    always_comb begin
        load_writeback = '0;
        head_delta = '0;
        head_data_load_en = 1'b0;
        load_data_addr = '0;
        head_load_op = DMEM_OP_LOAD;
        store_probe_hi_next = 1'b0;
        mem_inflight_next = mem_inflight_q;
        mem_inflight_kill_next = mem_inflight_kill_q;
        reservation_valid_mid = reservation_valid_q;
        reservation_addr_mid = reservation_addr_q;
        reservation_pa_mid = reservation_pa_q;
        head_retire = 1'b0;
        head_done = 1'b0;
        sc_issue = 1'b0;
        sc_commit_done = 1'b0;
        sc_inflight_next = sc_inflight_q;
        amo_issue = 1'b0;
        amo_inflight_next = amo_inflight_q;
        amo_store_data = '0;
        amo_store_mask = '0;
        amo_split_hi_data = '0;
        amo_split_hi_mask = '0;

        // A delivered response frees the single outstanding-load slot whether
        // it is consumed (match block below) or discarded as stale (killed /
        // squashed entry -- the match conditions then simply fail).
        if (data_load_valid) begin
            mem_inflight_next = 1'b0;
            mem_inflight_kill_next = 1'b0;
        end

        // Post-squash validity per slot. Reads entries_q + abort_mask only; wakeup
        // never touches .entry.valid, so this equals the post-wakeup entries_premerge
        // validity the original head-skip read -- but without aliasing the wakeup
        // array, which is what keeps head_next_skip (-> headq -> load_writeback) out
        // of the wakeup cone.
        for (int i = 0; i < MEM_Q_SIZE; i += 1) begin
            sq_valid[i] = ((entries_q[i].entry.branch_mask & abort_mask) == '0) &&
                          entries_q[i].entry.valid;
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

        // Head-skip over leading invalid (squashed/empty) slots in one shot,
        // capped at count_q (the first VALID slot's offset from head_q). Priority
        // mux (reverse scan, lowest in-range valid k wins); defaults to count_q when
        // every in-range slot is invalid. Reads only sq_valid (registered-derived),
        // not the wakeup array. Multi-step skip preserved (a store committed the
        // same cycle its predecessor hole is skipped is not dropped).
        head_skip_n = count_q;
        for (int k = MEM_Q_SIZE-1; k >= 0; k -= 1) begin
            if ((($clog2(MEM_Q_SIZE+1))'(k) < count_q) &&
                    sq_valid[(head_q + ($clog2(MEM_Q_SIZE))'(k))]) begin
                head_skip_n = ($clog2(MEM_Q_SIZE+1))'(k);
            end
        end
        head_next_skip   = head_q + head_skip_n[$clog2(MEM_Q_SIZE)-1:0];
        count_after_skip = count_q - head_skip_n;

        // Registered snapshot of the head entry for the per-op blocks below.
        // The head-skip lands head_next_skip on a non-squashed valid entry, so
        // entries_q[head_next_skip] is its registered (this-cycle-stable) view;
        // reading it (not entries_wake) keeps the load AGU / store data / readiness
        // off the deep squash->wakeup->format chain. A head op fires one cycle
        // after its operand registers; the LSQ tolerates the extra latency.
        headq = entries_q[head_next_skip];

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
        head_match     = (head_next_skip == head_q);
        // The registered translate (xlate_*_q) belongs to the current head request
        // iff a request was valid both when it was registered (last cycle) and now,
        // for the SAME head entry and probe phase. When the head advances or the
        // probe toggles -- or on the first cycle a new head presents -- the tag
        // mismatches and the head op waits one cycle for the fresh translate to
        // register (the +1 translate latency). This also guarantees a stale
        // translate from a different head (or a walk transition) is never consumed.
        xlate_ready = xlate_reqv_q && mem_req_valid &&
            (xlate_head_q == head_q) && (xlate_probe_q == store_probe_hi_q);
        head_xlate_ok  = !xlate_fault_q && xlate_ready &&
            (!paging_data || (head_match && !xlate_stall_q));
        head_xlate_flt = head_match && xlate_ready && xlate_fault_q;

        // The per-op head blocks below write the single head entry via head_delta
        // (sparse per-field write-enables) instead of indexing entries_premerge,
        // so they read/write only registered/headq state. M applies head_delta.

        // Faulting access: retire it with an exception instead of touching memory.
        if (head_xlate_flt && !head_done && !double_store_pending_q &&
                headq.entry.valid &&
                (headq.entry.ctrl.memRead ||
                 headq.entry.ctrl.memWrite) &&
                headq.entry.src1_ready &&
                !headq.issued_load) begin
            head_done = 1'b1;
            load_writeback.valid = 1'b1;
            load_writeback.active_id = headq.entry.active_id;
            load_writeback.prd = headq.entry.prd;
            load_writeback.has_dest = headq.entry.has_dest;
            load_writeback.branch_mask = headq.entry.branch_mask;
            load_writeback.exception = 1'b1;
            load_writeback.exc_cause = xlate_cause_q;
            load_writeback.data = headq.addr;   // mtval = VA
            head_delta.zero = 1'b1;
            head_retire = 1'b1;
        end

        // Misaligned AMO / LR / SC: the implementation does not support
        // misaligned atomics (MISALIGNED_AMO=false, LR/SC "always raise access
        // fault"), so an unaligned address retires with an access fault and
        // never touches memory or the reservation. Gated by head_match so it
        // only acts on the registered head (and never clobbers a writeback the
        // fault block above already produced for a different entry).
        if (head_match && !head_done && !double_store_pending_q &&
                headq.entry.valid &&
                (headq.entry.ctrl.exec_class == EXEC_AMO) &&
                headq.entry.src1_ready &&
                !headq.issued_load &&
                ((headq.addr[1:0] != 2'b00) ||
                 ((headq.entry.ctrl.ldst_mode == LDST_D) &&
                  headq.addr[2]))) begin
            head_done = 1'b1;
            load_writeback.valid = 1'b1;
            load_writeback.active_id = headq.entry.active_id;
            load_writeback.prd = headq.entry.prd;
            load_writeback.has_dest = headq.entry.has_dest;
            load_writeback.branch_mask = headq.entry.branch_mask;
            load_writeback.exception = 1'b1;
            load_writeback.exc_cause =
                (headq.entry.ctrl.amo_op == AMO_LR) ?
                    RISCV_Priv::EXC_LOAD_ACCESS : RISCV_Priv::EXC_STORE_ACCESS;
            load_writeback.data = headq.addr;   // mtval = VA
            head_delta.zero = 1'b1;
            head_retire = 1'b1;
        end

        if (head_xlate_ok && !head_done && !double_store_pending_q && headq.entry.valid &&
                (headq.entry.ctrl.exec_class == EXEC_AMO) &&
                (headq.entry.ctrl.amo_op == AMO_SC) &&
                headq.entry.src1_ready &&
                headq.entry.src2_ready &&
                !headq.issued_load) begin
            head_done = 1'b1;
            if (!COHERENT) begin
                // Single-core: the SC decides success/fail and writes rd here, at
                // the head (the verbatim pre-B9 path). Bit-identical when COHERENT=0.
                load_writeback.valid = 1'b1;
                load_writeback.active_id = headq.entry.active_id;
                load_writeback.prd = headq.entry.prd;
                load_writeback.has_dest = headq.entry.has_dest;
                load_writeback.branch_mask = headq.entry.branch_mask;
                load_writeback.data = (reservation_valid_q &&
                    (reservation_addr_q == headq.addr)) ? '0 : XLEN'(1);
                if (load_writeback.data == '0) begin
                    // SC succeeds: capture its PA for the commit write-back.
                    head_delta.we_store_lo_pa = 1'b1;
                    head_delta.store_lo_pa = xlate_pa_q;
                    head_delta.we_issued_load = 1'b1;
                    head_delta.issued_load = 1'b1;
                    reservation_valid_mid = 1'b0;
                end else begin
                    head_delta.zero = 1'b1;
                    head_retire = 1'b1;
                end
            end else begin
                // M4 B9 (coherent): DEFER the decision. Mark the SC ROB-done with a
                // completion-only writeback (has_dest=0 => no rd, no wakeup, no
                // phys-write, so the dependent op sleeps until commit), capture the
                // PA, and PARK the SC at the head (issued_load=1) WITHOUT clearing
                // the reservation and WITHOUT retiring. Both would-succeed and
                // would-fail park; the success/fail decision + rd are taken at the
                // commit-store cycle (commit-resolve block below), so a remote
                // snoop-kill anywhere in [head, commit] forces the SC to fail.
                load_writeback.valid = 1'b1;
                load_writeback.active_id = headq.entry.active_id;
                load_writeback.branch_mask = headq.entry.branch_mask;
                load_writeback.has_dest = 1'b0;
                head_delta.we_store_lo_pa = 1'b1;
                head_delta.store_lo_pa = xlate_pa_q;
                head_delta.we_issued_load = 1'b1;
                head_delta.issued_load = 1'b1;
            end
        end

        // M4-S5b (COHERENT only): agent-authoritative SC at the commit point. The
        // provisional head op above parked the SC at the head (issued_load=1) with no
        // rd. Two phases, both driven off registered state + module inputs only (the
        // HD cone -- no wakeup<->load_writeback loop):
        //  (1) ISSUE: when the commit unit signals this SC (commit_store + matching
        //      active_id) and no COP_SC is outstanding, issue a COP_SC on the data
        //      port (driven in block M) -- the agent does the reservation
        //      check-and-store ATOMICALLY at the directory serialization point. Mark
        //      mem_inflight + sc_inflight so the response is awaited (single-outstanding).
        //  (2) COMPLETE: when the agent's response arrives (data_load carries the
        //      formatted rd: 0=success / 1=fail), write rd, retire the SC from the LSQ,
        //      and pulse sc_commit_done so the commit unit retires it from the ROB.
        // Correct under tight SMP contention (unlike an LSQ-local re-check): the
        // success decision and the store are one atomic agent action ordered by the
        // directory, so exactly one contender wins. Constant-folds when COHERENT=0.
        if (COHERENT && sc_inflight_q && data_load_valid) begin
            // (2) COMPLETE
            head_done = 1'b1;
            load_writeback.valid = 1'b1;
            load_writeback.active_id = headq.entry.active_id;
            load_writeback.prd = headq.entry.prd;
            load_writeback.has_dest = headq.entry.has_dest;
            load_writeback.branch_mask = '0;   // SC is the non-speculative ROB head
            load_writeback.data = data_load;   // agent sc_ok, formatted to 0/1 by the adapter
            head_delta.zero = 1'b1;
            head_retire = 1'b1;
            reservation_valid_mid = 1'b0;      // RVWMO: SC clears the reservation either way
            sc_inflight_next = 1'b0;
            sc_commit_done = 1'b1;
        end else if (COHERENT && !sc_inflight_q && !head_done && !double_store_pending_q &&
                headq.entry.valid && headq.issued_load &&
                (headq.entry.ctrl.exec_class == EXEC_AMO) &&
                (headq.entry.ctrl.amo_op == AMO_SC) &&
                commit_store && (commit_store_id == headq.entry.active_id) &&
                !mem_inflight_q && dmem_req_ready) begin
            // (1) ISSUE the COP_SC. Block M drives the data port (a write at the
            // captured PA tagged DMEM_OP_SC); here we reserve the single outstanding
            // slot and wait for the agent's verdict.
            head_done = 1'b1;
            sc_issue = 1'b1;
            mem_inflight_next = 1'b1;
            sc_inflight_next = 1'b1;
        end

        // M4 #3 (COHERENT only): agent-authoritative AMO. Mirrors the SC. The AMO
        // does NOT read at the head; it PARKS (head-park block below) and at commit
        // issues a COP_AMO -- the agent acquires M, atomically reads OLD, writes
        // amo(op, OLD, operand), and returns OLD (snoop-replay during the acquire).
        // The OLD word arrives like a load response; rd is written from it. This
        // closes the Stage-2 read->commit-store window (AT_AMO_RD has no replay).
        if (COHERENT && amo_inflight_q && data_load_valid) begin
            // (2) COMPLETE: the agent's OLD word arrived -> write rd, retire, release.
            head_done = 1'b1;
            load_writeback.valid = 1'b1;
            load_writeback.active_id = headq.entry.active_id;
            load_writeback.prd = headq.entry.prd;
            load_writeback.has_dest = headq.entry.has_dest;
            load_writeback.branch_mask = '0;     // the AMO is the non-speculative ROB head
            load_writeback.data = format_load(data_load,
                headq.addr[ADDR_SHIFT-1:0], headq.entry.ctrl.ldst_mode);
            head_delta.zero = 1'b1;
            head_retire = 1'b1;
            amo_inflight_next = 1'b0;
            sc_commit_done = 1'b1;               // shared "atomic op at head resolved" pulse
        end else if (COHERENT && !amo_inflight_q && !head_done && !double_store_pending_q &&
                headq.entry.valid && headq.issued_load &&
                (headq.entry.ctrl.exec_class == EXEC_AMO) &&
                (headq.entry.ctrl.amo_op != AMO_LR) && (headq.entry.ctrl.amo_op != AMO_SC) &&
                commit_store && (commit_store_id == headq.entry.active_id) &&
                !mem_inflight_q && dmem_req_ready) begin
            // (1) ISSUE the COP_AMO. Block M drives the data beat (captured PA,
            // operand=store_raw, dmem_req_amo=amo_op, tagged DMEM_OP_AMO).
            head_done = 1'b1;
            amo_issue = 1'b1;
            mem_inflight_next = 1'b1;
            amo_inflight_next = 1'b1;
        end

        // M4 #3 (COHERENT only): AMO head-park. Like the COHERENT SC head-decision,
        // mark the AMO ROB-done with a completion-only writeback (has_dest=0 => the
        // dependent op sleeps), capture the PA, and PARK at the head (issued_load=1)
        // WITHOUT issuing a read. The atomic RMW + rd happen at the commit COP_AMO
        // above. Placed before the load-issue so the AMO never issues a COP_AMO_RD.
        if (COHERENT && head_xlate_ok && !head_done && !double_store_pending_q &&
                headq.entry.valid &&
                (headq.entry.ctrl.exec_class == EXEC_AMO) &&
                (headq.entry.ctrl.amo_op != AMO_LR) && (headq.entry.ctrl.amo_op != AMO_SC) &&
                headq.entry.src1_ready && headq.entry.src2_ready &&
                !headq.issued_load) begin
            head_done = 1'b1;
            load_writeback.valid = 1'b1;
            load_writeback.active_id = headq.entry.active_id;
            load_writeback.branch_mask = headq.entry.branch_mask;
            load_writeback.has_dest = 1'b0;
            head_delta.we_store_lo_pa = 1'b1;
            head_delta.store_lo_pa = xlate_pa_q;
            head_delta.we_issued_load = 1'b1;
            head_delta.issued_load = 1'b1;
        end

        // Load issue: reads/conditions from the REGISTERED head snapshot (headq)
        // so load_data_addr is a shallow registered mux, not the deep AGU chain.
        // A load issues one cycle after its address operand registers; the LSQ
        // tolerates the latency. Writes (store_lo_pa, issued_load) go via head_delta.
        if (head_xlate_ok && !head_done && !double_store_pending_q && headq.entry.valid &&
                headq.entry.ctrl.memRead &&
                headq.entry.src1_ready && !headq.issued_load &&
                !mem_inflight_q && dmem_req_ready) begin
            head_done = 1'b1;
            head_data_load_en = 1'b1;
            load_data_addr = headq.addr[XLEN-1:ADDR_SHIFT];
            // M3d Stage 2: tag the read beat so the CCD agent acquires the right
            // coherence state -- AMO_RD acquires M (held for the commit write),
            // LR sets the agent reservation, a plain load gets S/E.
            head_load_op = (headq.entry.ctrl.exec_class == EXEC_AMO)
                ? ((headq.entry.ctrl.amo_op == AMO_LR) ? DMEM_OP_LR : DMEM_OP_AMO_RD)
                : DMEM_OP_LOAD;
            // Capture the PA for an AMO's write-back beat (same address it read);
            // harmless for a pure load.
            head_delta.we_store_lo_pa = 1'b1;
            head_delta.store_lo_pa = xlate_pa_q;
            head_delta.we_issued_load = 1'b1;
            head_delta.issued_load = 1'b1;
            mem_inflight_next = 1'b1;
        end

        // Pure-store completion / cross-word probe. A single-word store marks
        // itself complete as soon as its (low) translation resolves. A two-beat
        // store (cross-word misaligned, or FP double) must additionally verify
        // the high word's translation BEFORE it is allowed to commit, so that a
        // page fault on the second word is reported as the store's exception and
        // no partial write is ever performed (store atomicity).
        if (head_xlate_ok && !head_done && !double_store_pending_q && headq.entry.valid &&
                headq.entry.ctrl.memWrite &&
                headq.entry.src1_ready &&
                headq.entry.src2_ready &&
                !headq.entry.ctrl.memRead &&
                !headq.issued_load) begin
            head_done = 1'b1;
            if (!needs_two_beats(headq.entry.ctrl,
                    headq.addr)) begin
                // Single-beat store proven translatable: latch its PA and mark
                // it complete. The commit write below uses this latched PA.
                head_delta.we_store_lo_pa = 1'b1;
                head_delta.store_lo_pa = xlate_pa_q;
                load_writeback.valid = 1'b1;
                load_writeback.active_id = headq.entry.active_id;
                load_writeback.branch_mask = headq.entry.branch_mask;
                load_writeback.has_dest = 1'b0;
                head_delta.we_issued_load = 1'b1;
                head_delta.issued_load = 1'b1;
            end else if (!store_probe_hi_q) begin
                // Low word translated OK: capture its PA, then probe the high
                // word next cycle.
                head_delta.we_store_lo_pa = 1'b1;
                head_delta.store_lo_pa = xlate_pa_q;
                store_probe_hi_next = 1'b1;
            end else begin
                // High word translated OK; capture its PA and complete.
                head_delta.we_store_hi_pa = 1'b1;
                head_delta.store_hi_pa = xlate_pa_q;
                load_writeback.valid = 1'b1;
                load_writeback.active_id = headq.entry.active_id;
                load_writeback.branch_mask = headq.entry.branch_mask;
                load_writeback.has_dest = 1'b0;
                head_delta.we_issued_load = 1'b1;
                head_delta.issued_load = 1'b1;
                store_probe_hi_next = 1'b0;
            end
        end else if (!head_done && !double_store_pending_q && headq.entry.valid &&
                headq.entry.ctrl.memWrite &&
                headq.entry.src1_ready &&
                headq.entry.src2_ready &&
                !headq.entry.ctrl.memRead &&
                !headq.issued_load &&
                store_probe_hi_q) begin
            // High-word translation still walking: hold the probe phase.
            head_done = 1'b1;
            store_probe_hi_next = 1'b1;
        end

        if (!head_done && !double_store_pending_q && headq.entry.valid &&
                headq.entry.ctrl.memRead &&
                headq.issued_load && data_load_valid &&
                !headq.load_complete &&
                mem_inflight_q && !mem_inflight_kill_q) begin
            head_done = 1'b1;
            load_writeback.valid = 1'b1;
            load_writeback.active_id = headq.entry.active_id;
            load_writeback.prd = headq.entry.prd;
            load_writeback.has_dest = headq.entry.has_dest;
            load_writeback.branch_mask = headq.entry.branch_mask;
            if (headq.entry.ctrl.exec_class == EXEC_AMO) begin
                // rd gets the old memory value at the access width (an AMO.W on
                // RV64 sign-extends bit 31, and may sit at offset 4 of the
                // 8-byte memory word).
                load_writeback.data = format_load(data_load,
                    headq.addr[ADDR_SHIFT-1:0],
                    headq.entry.ctrl.ldst_mode);
                if (headq.entry.ctrl.amo_op == AMO_LR) begin
                    reservation_valid_mid = 1'b1;
                    reservation_addr_mid = headq.addr;
                    // M3d Stage 3: capture the LR's PHYSICAL line for the snoop coherence-kill.
                    // store_lo_pa was latched with the LR's translated PA at issue (load-issue block).
                    reservation_pa_mid = headq.store_lo_pa[XLEN-1:ADDR_SHIFT];
                    head_delta.zero = 1'b1;
                    head_retire = 1'b1;
                end else begin
                    // Compute on the raw (unshifted) operands at the access
                    // width, then position the result back into its memory
                    // word with the matching byte-enable mask.
                    format_store_split(
                        headq.entry.ctrl.ldst_mode,
                        headq.addr[ADDR_SHIFT-1:0],
                        amo_result(headq.entry.ctrl.amo_op,
                            load_writeback.data,
                            headq.store_raw,
                            headq.entry.ctrl.ldst_mode != LDST_D),
                        amo_store_data,
                        amo_store_mask,
                        amo_split_hi_data,
                        amo_split_hi_mask);
                    head_delta.we_store_data = 1'b1;
                    head_delta.store_data = amo_store_data;
                    head_delta.store_mask = amo_store_mask;
                    head_delta.we_load_complete = 1'b1;
                    head_delta.load_complete = 1'b1;
                end
            end else begin
                if (needs_two_beats(headq.entry.ctrl,
                        headq.addr) &&
                        !headq.double_low_valid) begin
                    // First beat of a two-beat load (cross-word misaligned, or
                    // the low word of an FP double): stash it, advance one word,
                    // and re-issue. Adding 4 preserves the byte offset used for
                    // the final extraction.
                    load_writeback = '0;
                    head_delta.we_load_low_word = 1'b1;
                    head_delta.load_low_word = data_load;
                    head_delta.we_double_low_valid = 1'b1;
                    head_delta.double_low_valid = 1'b1;
                    head_delta.we_issued_load = 1'b1;
                    head_delta.issued_load = 1'b0;
                    head_delta.we_addr = 1'b1;
                    head_delta.addr = headq.addr + XLEN'(XLEN_BYTES);
                end else begin
                    if (headq.double_low_valid) begin
                        // Final beat: combine {high, low} and extract the
                        // requested bytes at the original byte offset.
                        load_writeback.data = format_load_wide(
                            {data_load, headq.load_low_word},
                            headq.addr[ADDR_SHIFT-1:0],
                            headq.entry.ctrl.ldst_mode);
                    end else begin
                        load_writeback.data = format_load(data_load,
                            headq.addr[ADDR_SHIFT-1:0],
                            headq.entry.ctrl.ldst_mode);
                    end
                    if (headq.entry.ctrl.exec_class == EXEC_FP) begin
                    load_writeback.fp_write =
                        headq.entry.ctrl.fp_writes_fpr;
                    load_writeback.fp_rd = headq.entry.fp_rd;
`ifdef RV64
                    // load_writeback.data was just extracted at the access
                    // width/offset (FLD decodes LDST_D, FLW LDST_W), so an FLD
                    // is the full value and an FLW NaN-boxes its low word --
                    // correct for any byte offset and for misaligned splits.
                    load_writeback.fp_data = headq.entry.ctrl.fp_double ?
                        load_writeback.data :
                        {32'hffff_ffff, load_writeback.data[31:0]};
`else
                    load_writeback.fp_data = headq.entry.ctrl.fp_double ?
                        {data_load, headq.load_low_word} :
                        {32'hffff_ffff, data_load};
`endif
                    load_writeback.has_dest = 1'b0;
                    end
                    head_delta.zero = 1'b1;
                    head_retire = 1'b1;
                end
            end
        end

        // Flush: suppress this cycle's load issue + writeback + probe, and the
        // outstanding-load bookkeeping. Committed store WRITES are handled (and
        // intentionally preserved) by M. head_delta is moot under flush -- M / block
        // 2 zero the whole queue.
        if (flush) begin
            load_writeback = '0;
            head_data_load_en = 1'b0;
            store_probe_hi_next = 1'b0;
            // A load issue suppressed by this flush must not leave a phantom
            // in-flight slot; an already-outstanding load stays outstanding and its
            // eventual response is discarded as stale.
            mem_inflight_next = mem_inflight_q && !data_load_valid;
            mem_inflight_kill_next = mem_inflight_q && !data_load_valid;
        end
    end

    // ==== Block W: squash + reset_mask + wakeup + store-rederive -> entries_wake ===
    // The ONLY wakeup reader. Its product (entries_wake) feeds M (merge) but never
    // load_writeback (HD), so the wakeup<->load_writeback loop stays broken.
    always_comb begin
        entries_wake = entries_q;
        for (int i = 0; i < MEM_Q_SIZE; i += 1) begin
            // FB2b R3: defer the abort squash (the deep ~entry-wide zeroing) to block M,
            // off the wakeup/rederive input cone (the rederive's format_store_split is the
            // store_data worst-path tail -- it must read entries_q-based state, not the
            // post-squash array). reset/wakeup apply to all valid entries; aborted entries
            // are zeroed in M before the next state. The head-skip (HD) already excludes
            // them via the shallow sq_valid, and the store-commit head is non-aborted, so
            // an un-zeroed wrong-path entry here can neither issue/commit nor be counted.
            if (entries_wake[i].entry.valid) begin
                entries_wake[i].entry.branch_mask &= ~reset_mask;
                for (int w = 0; w < OOO_WIDTH; w += 1) begin
                    if (wakeup_valid[w]) begin
                        if (entries_wake[i].entry.prs1 == wakeup_prd[w]) begin
                            entries_wake[i].entry.src1_ready = 1'b1;
                            entries_wake[i].addr_ready = 1'b1;
                            entries_wake[i].addr = wakeup_data[w] +
                                ((entries_wake[i].entry.ctrl.exec_class == EXEC_AMO) ?
                                 '0 : entries_wake[i].entry.imm);
                        end
                        if ((entries_wake[i].entry.prs2 == wakeup_prd[w]) &&
                                !((entries_wake[i].entry.ctrl.exec_class == EXEC_FP) &&
                                  entries_wake[i].entry.ctrl.memWrite)) begin
                            entries_wake[i].entry.src2_ready = 1'b1;
                            entries_wake[i].data_ready = 1'b1;
                            entries_wake[i].store_raw = wakeup_data[w];
                            format_store_split(
                                entries_wake[i].entry.ctrl.ldst_mode,
                                entries_wake[i].addr[ADDR_SHIFT-1:0],
                                wakeup_data[w],
                                entries_wake[i].store_data,
                                entries_wake[i].store_mask,
                                entries_wake[i].store_data_hi,
                                entries_wake[i].store_mask_hi);
                        end
                    end
                end
            end
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
            if (entries_wake[i].entry.valid &&
                    entries_wake[i].entry.ctrl.memWrite &&
                    !entries_wake[i].entry.ctrl.memRead &&
                    (entries_wake[i].entry.ctrl.exec_class != EXEC_AMO) &&
                    entries_wake[i].addr_ready &&
`ifdef RV64
                    ((entries_wake[i].entry.ctrl.exec_class == EXEC_FP) ||
                     entries_wake[i].data_ready)
`else
                    (entries_wake[i].entry.ctrl.exec_class != EXEC_FP) &&
                    entries_wake[i].data_ready
`endif
                    ) begin
                format_store_split(
                    entries_wake[i].entry.ctrl.ldst_mode,
                    entries_wake[i].addr[ADDR_SHIFT-1:0],
                    entries_wake[i].store_raw,
                    entries_wake[i].store_data,
                    entries_wake[i].store_mask,
                    entries_wake[i].store_data_hi,
                    entries_wake[i].store_mask_hi);
            end
        end
    end

    // ==== Block M: merge entries_wake + head_delta, advance head, data-port mux ====
    // Layers HD's head-op writes (head_delta) onto entries_wake (post squash/wakeup/
    // rederive), advances the head (retire from HD + store-commit), muxes the data
    // port, and finalizes count/reservation/double-store/flush. It reads entries_wake
    // (wakeup) but drives entries_premerge / head_next / the data port -- NOT
    // load_writeback -- so no new wakeup<->load_writeback path is created. Store-commit
    // data is read from REGISTERED entries_q (value-equivalent for a committing store,
    // and shallow -- no format_store_split recompute in the commit cone).
    always_comb begin
        entries_premerge = entries_wake;

        // FB2b R3: apply the deferred abort squash (moved out of block W) -- zero every
        // wrong-path entry. Off block W's wakeup/rederive input cones (the store_data
        // worst-path tail). head_next_skip is non-aborted (head-skip uses sq_valid), so
        // this never collides with the head_delta write below; entries_premerge
        // (-> entries_next -> entries_q) is bit-identical to the old block-W squash.
        for (int i = 0; i < MEM_Q_SIZE; i += 1) begin
            if ((entries_q[i].entry.branch_mask & abort_mask) != '0) begin
                entries_premerge[i] = '0;
            end
        end

        // Apply HD's per-op head writes onto the head entry. Per-field write-enables
        // preserve any same-cycle wakeup update to fields the firing op does not
        // touch -> identical to the original (which layered the head-op writes onto
        // the post-wakeup entries_premerge[head_next]).
        if (head_delta.zero) begin
            entries_premerge[head_next_skip] = '0;
        end else begin
            if (head_delta.we_store_lo_pa)
                entries_premerge[head_next_skip].store_lo_pa = head_delta.store_lo_pa;
            if (head_delta.we_store_hi_pa)
                entries_premerge[head_next_skip].store_hi_pa = head_delta.store_hi_pa;
            if (head_delta.we_issued_load)
                entries_premerge[head_next_skip].issued_load = head_delta.issued_load;
            if (head_delta.we_store_data) begin
                entries_premerge[head_next_skip].store_data = head_delta.store_data;
                entries_premerge[head_next_skip].store_mask = head_delta.store_mask;
            end
            if (head_delta.we_load_complete)
                entries_premerge[head_next_skip].load_complete = head_delta.load_complete;
            if (head_delta.we_load_low_word)
                entries_premerge[head_next_skip].load_low_word = head_delta.load_low_word;
            if (head_delta.we_double_low_valid)
                entries_premerge[head_next_skip].double_low_valid = head_delta.double_low_valid;
            if (head_delta.we_addr)
                entries_premerge[head_next_skip].addr = head_delta.addr;
        end

        // Apply the deferred head advance from a retiring head op (fault /
        // misaligned-AMO / SC-fail / load-complete) before store-commit, so
        // store-commit sees the post-retire head -> the load-complete + store-commit
        // two-retire-per-cycle case still works.
        head_after_retire = head_next_skip;
        count_next_pre = count_after_skip;
        if (head_retire) begin
            head_after_retire = head_next_skip + 1'b1;
            count_next_pre -= 1'b1;
        end

        // Reservation: HD's post-head-op value, then store-commit clears on match.
        reservation_valid_next = reservation_valid_mid;
        reservation_addr_next = reservation_addr_mid;
        reservation_pa_next = reservation_pa_mid;
        // M3d Stage 3 (S2): coherence-kill -- a remote write (FwdGetM/INV) to the reserved line
        // clears the reservation, so a later SC fails (RVWMO LR/SC atomicity). Conservative-early,
        // line-granular, mirrors the agent's own rsv-kill (niigo_l1d_gg.sv block C). The SC compare
        // itself is unchanged (VA-based). Inert single-core (snoop_kill_valid is constant 0).
        if (snoop_kill_valid &&
            (reservation_pa_q[MEMORY_ADDR_WIDTH-1:LINE_OFF_W] ==
             snoop_kill_laddr[MEMORY_ADDR_WIDTH-1:LINE_OFF_W]))
            reservation_valid_next = 1'b0;
        double_store_pending_next = 1'b0;
        double_store_addr_next = '0;
        double_store_data_next = '0;
        double_store_mask_next = '0;
        head_next = head_after_retire;

        // Store commit: the committing store was already proven translatable at its
        // completion probe (which latched store_lo_pa), so write it using that
        // captured PA -- NOT a re-evaluated live translation, whose DTLB entry can
        // have been evicted between the probe and commit. The CONDITION reads
        // entries_premerge (post squash/wakeup/head_delta) so a same-cycle squash is
        // honored; the DATA reads the REGISTERED entries_q (value-equivalent for a
        // committing store, and shallow).
        // M4-S5b / M4 #3: a coherent SC or RMW-AMO is NOT a plain committed store --
        // it is resolved by the COP_SC / COP_AMO issue/complete path (block HD + the
        // sc_issue / amo_issue data beat below), so it must never fire the
        // unconditional store-commit write. Exclude both here (LR is a read, never
        // a committed store, so amo_op != AMO_LR covers SC + every RMW AMO).
        store_commit_fires = !double_store_pending_q && commit_store &&
            entries_premerge[head_after_retire].entry.valid &&
            entries_premerge[head_after_retire].entry.ctrl.memWrite &&
            (entries_premerge[head_after_retire].entry.active_id == commit_store_id) &&
            !(COHERENT &&
              (entries_premerge[head_after_retire].entry.ctrl.exec_class == EXEC_AMO) &&
              (entries_premerge[head_after_retire].entry.ctrl.amo_op != AMO_LR));

        if (store_commit_fires) begin
            // Second (high) beat of a two-beat store: queue a fire-and-forget write
            // at the high word's captured physical address (already proven fault-free
            // during the probe).
            if (needs_two_beats(entries_q[head_after_retire].entry.ctrl,
                    entries_q[head_after_retire].addr)) begin
                double_store_pending_next = 1'b1;
                double_store_addr_next = entries_q[head_after_retire].store_hi_pa[XLEN-1:ADDR_SHIFT];
                double_store_data_next = entries_q[head_after_retire].store_data_hi;
                double_store_mask_next = entries_q[head_after_retire].store_mask_hi;
            end
            if (reservation_valid_next &&
                    (reservation_addr_next == entries_q[head_after_retire].addr)) begin
                reservation_valid_next = 1'b0;
            end
            entries_premerge[head_after_retire] = '0;
            head_next = head_after_retire + 1'b1;
            count_next_pre -= 1'b1;
        end

        // Held second beat of a prior double store re-arms while the port is busy.
        // Mutually exclusive with store-commit (which is gated !double_store_pending_q).
        if (double_store_pending_q && !dmem_req_ready) begin
            double_store_pending_next = 1'b1;
            double_store_addr_next = double_store_addr_q;
            double_store_data_next = double_store_data_q;
            double_store_mask_next = double_store_mask_q;
        end

        // ---- Data-port mux. Priority (matching the original sequence): a held
        // double-store beat overrides store-commit overrides the HD load issue.
        // data_load_en tracks the HD load issue independently (a store-commit does
        // not clear it), reproducing the original where load-issue set data_load_en
        // unconditionally and store-commit only overrode the address/store fields.
        data_load_en = head_data_load_en;
        data_addr = '0;
        data_store = '0;
        data_store_mask = '0;
        store_second_beat = 1'b0;
        // M3d Stage 2: the typed op follows the same priority as the data fields
        // (double-store beat / store-commit are STORE; a head load issue carries its
        // LOAD/LR/AMO_RD tag). Idle -> don't-care (the port is invalid).
        dmem_req_op = DMEM_OP_LOAD;
        dmem_req_amo = 4'd0;            // M4 #3: only meaningful on the COP_AMO beat
        if (double_store_pending_q) begin
            data_addr = double_store_addr_q;
            data_store = double_store_data_q;
            data_store_mask = double_store_mask_q;
            // data_addr already holds the captured physical word address; tell
            // the core port to use it directly (skip the head-VA translation).
            store_second_beat = 1'b1;
            dmem_req_op = DMEM_OP_STORE;
        end else if (sc_issue) begin
            // M4-S5b: coherent SC -- drive the conditional store beat (COP_SC) at the
            // captured PA. The agent does the atomic check-and-store and returns sc_ok
            // (awaited via mem_inflight, like a load). Same captured-PA / store_second_beat
            // shape as a committed store, but tagged DMEM_OP_SC.
            data_addr = entries_q[head_after_retire].store_lo_pa[XLEN-1:ADDR_SHIFT];
            data_store = entries_q[head_after_retire].store_data;
            data_store_mask = entries_q[head_after_retire].store_mask;
            store_second_beat = 1'b1;
            dmem_req_op = DMEM_OP_SC;
        end else if (amo_issue) begin
            // M4 #3: coherent AMO -- drive the COP_AMO beat at the captured PA. The
            // raw operand (rs2) rides data_store and the fine op rides dmem_req_amo;
            // the agent reads OLD, applies amo(op, OLD, operand) atomically, and
            // returns OLD (awaited via mem_inflight, like a load). Full-word write.
            data_addr = entries_q[head_after_retire].store_lo_pa[XLEN-1:ADDR_SHIFT];
            data_store = entries_q[head_after_retire].store_raw;
            data_store_mask = '1;
            store_second_beat = 1'b1;
            dmem_req_op = DMEM_OP_AMO;
            dmem_req_amo = entries_q[head_after_retire].entry.ctrl.amo_op;
        end else if (store_commit_fires) begin
            // First (low) beat: written at the captured physical address.
            data_addr = entries_q[head_after_retire].store_lo_pa[XLEN-1:ADDR_SHIFT];
            data_store = entries_q[head_after_retire].store_data;
            data_store_mask = entries_q[head_after_retire].store_mask;
            store_second_beat = 1'b1;
            dmem_req_op = DMEM_OP_STORE;
        end else if (head_data_load_en) begin
            data_addr = load_data_addr;
            dmem_req_op = head_load_op;
        end

        // Precise-trap flush: discard the queued head; suppress the load issue.
        // Committed store WRITES (held double-store beat / store-commit) are left
        // exactly as driven above (they belong to already-committed stores).
        if (flush) begin
            head_next = '0;
            data_load_en = 1'b0;
        end
    end

    // ---- Block 2: parallel insert (the only insert_rs*_data reader) + occupancy +
    // entries/tail/count flush. Splitting load_writeback (block 1) from the insert
    // here is what breaks the false phys_rs_data -> load_writeback loop edge.
    // entries_premerge is the post-squash/wakeup/head/store-commit state; insert
    // fills its free (tail) slots, which the head block never touches. ----
    always_comb begin
        entries_next = entries_premerge;
        count_next = count_next_pre;
        tail_next = tail_q;

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
            tail_next = '0;
            count_next = '0;
            // head_next / data_load_en / load_writeback / in-flight are flushed in
            // the block-1 flush above.
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
            reservation_pa_q <= '0;
            double_store_pending_q <= 1'b0;
            double_store_addr_q <= '0;
            double_store_data_q <= '0;
            double_store_mask_q <= '0;
            store_probe_hi_q <= 1'b0;
            mem_inflight_q <= 1'b0;
            mem_inflight_kill_q <= 1'b0;
            sc_inflight_q <= 1'b0;
            amo_inflight_q <= 1'b0;
            xlate_pa_q <= '0;
            xlate_fault_q <= 1'b0;
            xlate_stall_q <= 1'b0;
            xlate_cause_q <= 5'd0;
            xlate_head_q <= '0;
            xlate_probe_q <= 1'b0;
            xlate_reqv_q <= 1'b0;
            for (int i = 0; i < MEM_Q_SIZE; i += 1) begin
                entries_q[i] <= '0;
            end
        end else begin
            count_q <= count_next;
            head_q <= head_next;
            tail_q <= tail_next;
            reservation_valid_q <= reservation_valid_next;
            reservation_addr_q <= reservation_addr_next;
            reservation_pa_q <= reservation_pa_next;
            double_store_pending_q <= double_store_pending_next;
            double_store_addr_q <= double_store_addr_next;
            double_store_data_q <= double_store_data_next;
            double_store_mask_q <= double_store_mask_next;
            store_probe_hi_q <= store_probe_hi_next;
            mem_inflight_q <= mem_inflight_next;
            mem_inflight_kill_q <= mem_inflight_kill_next;
            sc_inflight_q <= sc_inflight_next;
            amo_inflight_q <= amo_inflight_next;
            // Register this cycle's head translation (combinational from the core's
            // DTLB+DataPMP on mem_req_vaddr) + the (head_q, store_probe_hi_q,
            // mem_req_valid) tag identifying which head request it is for. Consumed
            // next cycle via xlate_ready (the translate pipeline stage).
            xlate_pa_q <= xlate_pa;
            xlate_fault_q <= xlate_fault;
            xlate_stall_q <= xlate_stall;
            xlate_cause_q <= xlate_cause;
            xlate_head_q <= head_q;
            xlate_probe_q <= store_probe_hi_q;
            xlate_reqv_q <= mem_req_valid;
            entries_q <= entries_next;
        end
    end

endmodule: load_store_queue
