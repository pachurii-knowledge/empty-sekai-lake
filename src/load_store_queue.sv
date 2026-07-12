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
`ifdef LSQ_MLP2
    // Track A: transaction id of the returning data-port response (the outstanding
    // slot it belongs to). Const-0 at P3a (single-outstanding); load-bearing at P3c
    // (parked out-of-order completion) where a younger hit can return before an
    // older miss. Threaded LSQ->core->memsys->l1_dcache and back. See track-a-mlp.md.
    input wire logic [LSQ_ID_W-1:0]  dmem_resp_id,
`endif
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
`ifdef LSQ_MLP2
    // Track A: transaction id tagging the data-port request (the outstanding slot
    // it allocates). Const-0 at P3a (single-outstanding); the allocated inflight
    // slot at P3b+. Round-trips back as dmem_resp_id. See track-a-mlp.md.
    output logic [LSQ_ID_W-1:0]  dmem_req_id,
`endif
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
    output writeback_packet_t    load_writeback,
    // P3-M0 (display-only, MLP diagnosis): categorizes why the in-order head is
    // blocked this cycle (see the LSQ_HR_* codes). Pure combinational hint off the
    // head state -- feeds only the SIMULATION perf counters, never functional logic;
    // optimized away in synthesis. See plans/ooo-perf.md P3-M0.
    output logic [2:0]           lsq_head_reason,
    // P3 L2a (display-only, store->load forwarding MEASURE-FIRST): on a store-park
    // cycle, classifies whether a younger load could forward from the parked HEAD
    // store (see the LSQ_FWD_* codes), modeling the L2b stalls (device PA, single-word,
    // no-intervening-store, byte full-cover) so FWD_FULL is not over-counted. Pure
    // combinational; feeds only the SIM perf counters. See plans/ooo-perf.md P3 lever 2.
    output logic [2:0]           lsq_fwd_class
);
    // P3-M0 head-blocking-reason codes.
    localparam logic [2:0] LSQ_HR_EMPTY     = 3'd0; // no valid entry at head (idle)
    localparam logic [2:0] LSQ_HR_READY     = 3'd1; // head drains/progresses this cycle
    localparam logic [2:0] LSQ_HR_LOADWAIT  = 3'd2; // load outstanding (mem_inflight): MLP=1 tax
    localparam logic [2:0] LSQ_HR_STOREPARK = 3'd3; // completed store parked awaiting commit
    localparam logic [2:0] LSQ_HR_XLATE     = 3'd4; // head mem-op, translation not ready
    localparam logic [2:0] LSQ_HR_MEMPORT   = 3'd5; // load ready but dmem port not ready
    localparam logic [2:0] LSQ_HR_OTHER     = 3'd6; // operand-wait / other
    // P3 L2a store->load forwarding-opportunity codes.
    localparam logic [2:0] LSQ_FWD_NA      = 3'd0; // not a store-park cycle
    localparam logic [2:0] LSQ_FWD_NOLOAD  = 3'd1; // store-park, no younger eligible int load
    localparam logic [2:0] LSQ_FWD_NOMATCH = 3'd2; // younger load, no byte overlap w/ head store
    localparam logic [2:0] LSQ_FWD_PARTIAL = 3'd3; // overlap but partial/two-beat/intervening/device
    localparam logic [2:0] LSQ_FWD_FULL    = 3'd4; // younger int load, full-cover from single-word RAM head store

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

    // Track A (plans/track-a-mlp.md): LSQ_MLP = the number of loads allowed
    // outstanding at the memory port. -DLSQ_MLP2 raises it to 2 (non-blocking L1D).
    // CCD/COHERENT PIN (correctness-critical, compile-time): a coherent build is
    // FORCED back to 1 -- two in-flight loads open an RVWMO load-load reorder window
    // with no 9.11-style inflight-load squash. CCD_AGENT is a macro, COHERENT a
    // param, so both are folded here (COHERENT can only be checked at elaboration).
    // LSQ_MLP==1 => the whole Track-A path is bit-identical to the single-outstanding
    // design (id fields const-0, issue_ptr==head).
`ifdef LSQ_MLP2
  `ifdef CCD_AGENT
    localparam int LSQ_MLP_REQ = 1;
  `else
    localparam int LSQ_MLP_REQ = 2;
  `endif
`else
    localparam int LSQ_MLP_REQ = 1;
`endif
    localparam int LSQ_MLP  = COHERENT ? 1 : LSQ_MLP_REQ;
    localparam int LSQ_ID_W = (LSQ_MLP > 1) ? $clog2(LSQ_MLP) : 1;
    // Compile-time guard: the coherent path must never elaborate MLP>1.
    initial begin
        if (COHERENT && (LSQ_MLP != 1))
            $fatal(1, "load_store_queue: COHERENT build must pin LSQ_MLP=1 (got %0d)", LSQ_MLP);
    end

    // M3d Stage 2: dmem_req_op codes (see the port comment). The CCD adapter maps
    // these to l1_core_op_e; keep the two in lockstep.
    localparam logic [2:0] DMEM_OP_LOAD   = 3'd0;
    localparam logic [2:0] DMEM_OP_STORE  = 3'd1;
    localparam logic [2:0] DMEM_OP_LR     = 3'd2;
    localparam logic [2:0] DMEM_OP_AMO_RD = 3'd3;
    localparam logic [2:0] DMEM_OP_SC     = 3'd4;   // M4-S5b: agent-authoritative SC (COP_SC)
    localparam logic [2:0] DMEM_OP_AMO    = 3'd5;   // M4 #3: agent-authoritative AMO (COP_AMO)

`ifdef LSQ_MLP2
    // P3b delivers IN ORDER (oldest slot first); the memsys returns responses oldest-
    // first, so a response's id must be the FIFO oldest slot. Assert it (P3c relaxes to
    // out-of-order parked completion and makes dmem_resp_id load-bearing). Sink the id
    // outside the assertion window so there is no UNUSED lint.
    logic _unused_dmem_resp_id;
    assign _unused_dmem_resp_id = ^dmem_resp_id;
    always_ff @(posedge clk) begin
        if (rst_l && data_load_valid && (inflight_count_q != '0))
            assert (dmem_resp_id == inflight_head_q)
                else $fatal(1, "LSQ P3b: out-of-order response id=%0d, expected oldest=%0d",
                            dmem_resp_id, inflight_head_q);
    end
`endif

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
`ifdef LSQ_MLP2
    // ===================== Track A P3b: non-blocking LSQ =====================
    // Up to LSQ_MLP loads outstanding. The outstanding loads form a FIFO of slots
    // in issue/age order (the memsys returns responses oldest-first -- passthrough
    // d_q FIFO or blocking-L1D single-outstanding -- and the head retires in order),
    // so delivery stays head-first (the existing head-complete block); the slot table
    // only supplies the issue-gate COUNT and a per-slot LATCHED KILL that discards a
    // squashed load's stale response (a mispredict can kill a suffix of the run).
    localparam int MEM_PTR_W  = $clog2(MEM_Q_SIZE);
    localparam int CNT_W      = $clog2(LSQ_MLP + 1);
    // issue_ptr: the next entry the 2nd (younger-than-head) issue considers. Advances
    // over a plain single-beat cacheable non-device load; reset to head on head pass.
    logic [MEM_PTR_W-1:0]  issue_ptr_q, issue_ptr_next;
    // Per-slot inflight descriptor FIFO (age order). owner = the issuing entry's ring
    // index; kill = monotonic latch (owner squashed while outstanding); gen = owner
    // active_id (reallocation guard). head/tail are the FIFO read/write pointers.
    logic                  inflight_valid_q [LSQ_MLP], inflight_valid_next [LSQ_MLP];
    logic [MEM_PTR_W-1:0]  inflight_owner_q [LSQ_MLP], inflight_owner_next [LSQ_MLP];
    logic                  inflight_kill_q  [LSQ_MLP], inflight_kill_next  [LSQ_MLP];
    active_id_t            inflight_gen_q   [LSQ_MLP], inflight_gen_next   [LSQ_MLP];
    logic [LSQ_ID_W-1:0]   inflight_head_q, inflight_head_next;   // FIFO read ptr (oldest slot)
    logic [LSQ_ID_W-1:0]   inflight_tail_q, inflight_tail_next;   // FIFO write ptr (next free slot)
    logic [CNT_W-1:0]      inflight_count_q, inflight_count_next;
    // Registered time-mux translate target: which entry mem_req_vaddr aims at. Driven
    // off a REGISTER so the DTLB index stays register-driven (no FB2b path regression);
    // head-priority keeps it == head_q whenever the head wants the port.
    logic [MEM_PTR_W-1:0]  xlate_target_q, xlate_target_next;
    logic [MEM_PTR_W-1:0]  xlate_tgt_q;   // which target the registered translate is for
    // Combinational issue_ptr issue event + its per-cycle helpers (driven in block HD).
    logic                  ip_issue_fire;
    // Address WIDTH, not ring-pointer width: this holds a physical WORD address
    // (xlate_pa_q[XLEN-1:ADDR_SHIFT]), so it must match load_data_addr / the data_addr
    // port. At MEM_PTR_W it truncated the PA to its low bits -> the ip load read a garbage
    // word near 0x0 (latent in committed P3b too; inert only while ip never fired).
    logic [MEMORY_ADDR_WIDTH-1:0] ip_load_addr_word;
    // "the head still needs the translate port" -- head-priority arbitration input.
    logic                  head_wants_xlate;
    // ---- P3b review fixes (wf_71a9e035-6d8) ----
    // MF2/MF5: issue_ptr's forward distance from head_q as its OWN register (kills the
    // mod-ring teleport). Invariant: issue_ptr_q == (head_q + ip_off_q) mod MEM_Q_SIZE.
    // Sized at count width so it can hold [0..MEM_Q_SIZE] (the full-ring park sentinel).
    logic [$clog2(MEM_Q_SIZE+1)-1:0] ip_off_q, ip_off_next;
    // A1: step issue_ptr over the head's own issued single-beat load (module-level so the
    // stat counters can see it). Computed in the FIFO next-state block.
    logic                  ip_step;
    // MF3: active_id tag of the registered translate's target -- ip consumes the
    // registered PA only when the target ring index STILL holds the same op (guards a
    // squash+reuse in the translate->issue window; mirrors the delivery gen-match).
    active_id_t            xlate_tgt_aid_q;
    // MF1/A4: the head load issues at its registered/effective PA (store_second_beat
    // pattern) EXCEPT a two-beat load's beats, which keep the live-translate path.
    logic                  head_load_pa_direct;
    // A3: write issued_load=1 on the issue_ptr-issued entry (else it re-issues at head).
    logic                  ip_delta_we_issued;
    logic [MEM_PTR_W-1:0]  ip_delta_idx;
    // MF6: the PHYSICAL 64B line each outstanding load was ISSUED at (A4 issues at the
    // PA, so the same-line guard must compare physical lines, not virtual entry addrs).
    logic [XLEN-1:ADDR_SHIFT+LINE_OFF_W] inflight_line_q   [LSQ_MLP];
    logic [XLEN-1:ADDR_SHIFT+LINE_OFF_W] inflight_line_next [LSQ_MLP];
`endif
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
    // P3 lever 1 (XLATE_BYPASS): the "effective" translate the head op consumes --
    // the registered result when it is current, else (bypass) the LIVE combinational
    // DTLB/DataPMP result when it resolves THIS cycle (a DTLB hit / bare / a fault),
    // so a hit issues WITHOUT the FB2b +1-cycle register bubble. Off => == xlate_*_q.
    logic [XLEN-1:0] xlate_pa_eff;
    logic            xlate_fault_eff;
    logic            xlate_stall_eff;
    logic            xlate_ready_eff;

    assign full = (count_q > MEM_Q_SIZE - OOO_WIDTH);
    assign store_port_busy = double_store_pending_q;

    // Expose the registered head's virtual address so the core can translate it
    // (DTLB lookup / PTW). Only meaningful once the address operand is resolved.
`ifdef LSQ_MLP2
    // Track A time-mux: the translate port aims at the REGISTERED xlate_target_q
    // (head_q, or issue_ptr_q when the head does not want the port). Indexing off a
    // register keeps the DTLB address-index path register-driven exactly as the
    // head-only design (no FB2b lengthening); head-priority (below) keeps
    // xlate_target_q == head_q whenever the head has a mem-op to translate, so the
    // head translate + XLATE_BYPASS are unchanged in the common case. When aimed at
    // issue_ptr it is always a plain single-beat cacheable load (carve-outs), so no
    // store/probe offset applies.
    wire mlp_xt_head = (xlate_target_q == head_q);
    // Head's OWN request validity (independent of the time-mux target) -- the head
    // translate-ready + XLATE_BYPASS gates must key on this, not the muxed mem_req_valid
    // (which reflects issue_ptr on its port turns).
    wire mlp_head_req_valid = entries_q[head_q].entry.valid && entries_q[head_q].addr_ready &&
        (entries_q[head_q].entry.ctrl.memRead || entries_q[head_q].entry.ctrl.memWrite);
    // issue_ptr's request validity: a plain single-beat cacheable load, address ready.
    // Used by the ip ISSUE gate (ip_ready_c) -- keyed on issue_ptr (the entry to issue).
    wire mlp_ip_req_valid = entries_q[issue_ptr_q].entry.valid && entries_q[issue_ptr_q].addr_ready &&
        entries_q[issue_ptr_q].entry.ctrl.memRead && !entries_q[issue_ptr_q].entry.ctrl.memWrite;
    // CRITICAL: the port TRANSLATES the entry it is registered-AIMED at (xlate_target_q),
    // NOT the current issue_ptr. When issue_ptr advances, xlate_target_q (set from a past
    // issue_ptr) and issue_ptr_q diverge; mem_req_valid/vaddr must follow the AIM so the
    // registered translate (xlate_pa_q) is the aimed entry's PA -- consumed next cycle only
    // when its tag (xlate_tgt_q==xlate_target_q) still equals issue_ptr (ip_ready). Keying
    // these on issue_ptr_q instead translates one entry while tagging it as another -> the
    // ip load reads the WRONG word (silent wrong-data; no assert fires).
    wire mlp_xt_req_valid = entries_q[xlate_target_q].entry.valid && entries_q[xlate_target_q].addr_ready &&
        entries_q[xlate_target_q].entry.ctrl.memRead && !entries_q[xlate_target_q].entry.ctrl.memWrite;
    assign mem_req_valid = mlp_xt_head ? mlp_head_req_valid : mlp_xt_req_valid;
    // BLOCKER 1: device-ness of the registered translate's resolved PA (xlate_pa_q).
    // issue_ptr must NOT emit a request to a device (irreversible response snoop), so
    // its issue is gated on this being 0. Same byte-PA device hole the core decodes.
    wire xlate_ip_is_device =
        ((xlate_pa_q >= XLEN'('h0200_0000)) && (xlate_pa_q < XLEN'('h0201_0000))) ||
        ((xlate_pa_q >= XLEN'('h0C00_0000)) && (xlate_pa_q < XLEN'('h1000_0000))) ||
        ((xlate_pa_q >= XLEN'('h0D00_0000)) && (xlate_pa_q < XLEN'('h0D00_1000)));
    // Translate the AIMED entry (xlate_target_q). When aimed at head (mlp_xt_head),
    // xlate_target_q==head_q so this is the head's addr (+ the two-beat-store probe
    // offset); when aimed at issue_ptr it is the aimed entry's addr -- NOT issue_ptr_q,
    // which may have since advanced (the wrong-word bug above).
    assign mem_req_vaddr = entries_q[xlate_target_q].addr +
        ((mlp_xt_head && store_probe_hi_q) ? XLEN'(XLEN_BYTES) : '0);
    assign mem_req_store = mlp_xt_head ? entries_q[head_q].entry.ctrl.memWrite : 1'b0;
`else
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
`endif

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
`ifdef LSQ_MLP2
        head_load_pa_direct = 1'b0;   // MF1: default live-translate path
        ip_delta_we_issued = 1'b0;     // A3: no issued_load write unless ip issues
        ip_delta_idx = '0;
`endif
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
`ifdef LSQ_MLP2
        // Track A: the head consumes the registered translate only when it was aimed
        // AT the head (xlate_tgt_q==head_q -- else it is issue_ptr's translate) and the
        // head still has a valid request (mlp_head_req_valid, not the muxed mem_req_valid
        // which reflects issue_ptr on its port turns).
        xlate_ready = xlate_reqv_q && mlp_head_req_valid &&
            (xlate_head_q == head_q) && (xlate_probe_q == store_probe_hi_q) &&
            (xlate_tgt_q == head_q);
`else
        xlate_ready = xlate_reqv_q && mem_req_valid &&
            (xlate_head_q == head_q) && (xlate_probe_q == store_probe_hi_q);
`endif
        // Effective translate = registered when current, else (XLATE_BYPASS) the live
        // combinational DTLB/DataPMP result when it resolves this cycle (a hit or a
        // fault -- i.e. NOT a walk-stall), letting the head op issue/fault the SAME
        // cycle it presents. A DTLB miss (xlate_stall) or a not-yet-live translate
        // falls back to the registered/PTW path. Off (no -DXLATE_BYPASS) the _eff
        // signals are exactly xlate_*_q => bit-identical. The bypass re-opens the
        // LSQ-head -> DTLB -> DataPMP -> issue placed path (FB2b Fmax cost; Quick-place
        // before FPGA use). mem_req_vaddr already carries the probe offset, so the
        // two-beat store's high-word probe bypasses correctly too (atomicity preserved
        // by store_probe_hi sequencing, not by the translate stage).
        xlate_pa_eff    = xlate_pa_q;
        xlate_fault_eff = xlate_fault_q;
        xlate_stall_eff = xlate_stall_q;
        xlate_ready_eff = xlate_ready;
`ifdef XLATE_BYPASS
        // Restrict the bypass to PLAIN LOADS (not stores / AMO / LR / SC): those carry
        // the ordering-sensitive commit-write + reservation state the FB2b register
        // stage was implicitly sequencing. A plain load at the head has all older ops
        // drained, so issuing it a cycle early cannot reorder against an older store.
        if (!xlate_ready && mem_req_valid && head_match && !xlate_stall &&
                headq.entry.ctrl.memRead && !headq.entry.ctrl.memWrite &&
                (headq.entry.ctrl.exec_class != EXEC_AMO)
`ifdef LSQ_MLP2
                // BLOCKER 2: only bypass when the live translate is the HEAD's (the port
                // is aimed at the head this cycle); an issue_ptr-aimed cycle carries a
                // different entry's PA. mem_req_valid == mlp_head_req_valid when mlp_xt_head.
                && mlp_xt_head
`endif
                ) begin
            xlate_pa_eff    = xlate_pa;
            xlate_fault_eff = xlate_fault;
            xlate_stall_eff = 1'b0;
            xlate_ready_eff = 1'b1;
        end
`endif
        head_xlate_ok  = !xlate_fault_eff && xlate_ready_eff &&
            (!paging_data || (head_match && !xlate_stall_eff));
        head_xlate_flt = head_match && xlate_ready_eff && xlate_fault_eff;

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
                    head_delta.store_lo_pa = xlate_pa_eff;
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
                head_delta.store_lo_pa = xlate_pa_eff;
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
            head_delta.store_lo_pa = xlate_pa_eff;
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
                !mem_inflight_q && dmem_req_ready
`ifdef LSQ_MLP2
                // Track A: the head load's PA is the LIVE translate of mem_req_vaddr,
                // so the port must be aimed at the head this cycle (mlp_xt_head); and a
                // free inflight slot must exist. (mlp_xt_head is true here by strict
                // head-priority whenever !issued_load, so this only bulletproofs the
                // 1-cycle target-register transition.)
                && mlp_xt_head && (inflight_count_q < LSQ_MLP)
`endif
                ) begin
            head_done = 1'b1;
            head_data_load_en = 1'b1;
`ifdef LSQ_MLP2
            // A4/MF1: issue the load at the SAME translate the decision consumed
            // (store_second_beat=PA-direct in block M) -- removes the registered-
            // decision vs live-PA desync on a DTLB fill. A two-beat load's beats keep
            // the LIVE path: their xlate_ready tag carries no beat term, so xlate_pa_eff
            // still holds beat-1's PA and PA-direct would re-read the first word.
            if (needs_two_beats(headq.entry.ctrl, headq.addr) || headq.double_low_valid) begin
                load_data_addr = headq.addr[XLEN-1:ADDR_SHIFT];
                head_load_pa_direct = 1'b0;
            end else begin
                load_data_addr = xlate_pa_eff[XLEN-1:ADDR_SHIFT];
                head_load_pa_direct = 1'b1;
            end
`else
            load_data_addr = headq.addr[XLEN-1:ADDR_SHIFT];
`endif
            // M3d Stage 2: tag the read beat so the CCD agent acquires the right
            // coherence state -- AMO_RD acquires M (held for the commit write),
            // LR sets the agent reservation, a plain load gets S/E.
            head_load_op = (headq.entry.ctrl.exec_class == EXEC_AMO)
                ? ((headq.entry.ctrl.amo_op == AMO_LR) ? DMEM_OP_LR : DMEM_OP_AMO_RD)
                : DMEM_OP_LOAD;
            // Capture the PA for an AMO's write-back beat (same address it read);
            // harmless for a pure load.
            head_delta.we_store_lo_pa = 1'b1;
            head_delta.store_lo_pa = xlate_pa_eff;
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
                head_delta.store_lo_pa = xlate_pa_eff;
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
                head_delta.store_lo_pa = xlate_pa_eff;
                store_probe_hi_next = 1'b1;
            end else begin
                // High word translated OK; capture its PA and complete.
                head_delta.we_store_hi_pa = 1'b1;
                head_delta.store_hi_pa = xlate_pa_eff;
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
`ifdef LSQ_MLP2
                // Track A: deliver the OLDEST outstanding slot only when it IS the current
                // head's load: owner index == the post-skip head (A3, closes active_id-recycle
                // collision), gen/active_id match (robust to ring reuse), and not killed
                // (owner squashed => the response is stale => discard, not deliver).
                (inflight_count_q != '0) && !inflight_kill_q[inflight_head_q] &&
                (inflight_owner_q[inflight_head_q] == head_next_skip) &&
                (inflight_gen_q[inflight_head_q] == headq.entry.active_id)
`else
                mem_inflight_q && !mem_inflight_kill_q
`endif
                ) begin
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

`ifdef LSQ_MLP2
        // ---- Track A: issue_ptr issues a 2nd (younger-than-head) plain load ----
        // Lowest port priority (only when the head is not using the data port and no
        // store/AMO/SC commits). Requires: issue_ptr's registered translate resolved
        // and aimed at issue_ptr (non-fault/stall/device); issue_ptr is a valid, in-
        // range, plain single-beat cacheable load, addr+operand ready, not yet issued;
        // a free inflight slot; not the same 64B line as any outstanding load.
        ip_issue_fire = 1'b0;
        ip_load_addr_word = '0;
        begin
            logic ip_ready_c, ip_carve_c, ip_same_line_c, ip_inrange_c, ip_notsquash_c;
            // MF3: consume the registered translate only if the target ring index STILL
            // holds the SAME op (active_id) -- guards a squash+reuse in the translate->
            // issue window that index-match alone would miss (A4 removes the live-PA net).
            ip_ready_c = xlate_reqv_q && (xlate_tgt_q == issue_ptr_q) &&
                         (xlate_tgt_aid_q == entries_q[issue_ptr_q].entry.active_id) &&
                         !xlate_fault_q && !xlate_stall_q && !xlate_ip_is_device &&
                         mlp_ip_req_valid;
            ip_carve_c = entries_q[issue_ptr_q].entry.valid &&
                         entries_q[issue_ptr_q].addr_ready &&
                         entries_q[issue_ptr_q].entry.src1_ready &&
                         !entries_q[issue_ptr_q].issued_load &&
                         entries_q[issue_ptr_q].entry.ctrl.memRead &&
                         !entries_q[issue_ptr_q].entry.ctrl.memWrite &&
                         (entries_q[issue_ptr_q].entry.ctrl.exec_class != EXEC_AMO) &&
                         !needs_two_beats(entries_q[issue_ptr_q].entry.ctrl,
                                          entries_q[issue_ptr_q].addr);
            // MF2/MF5: in-range via the offset register (no mod-ring compare). Strictly
            // younger than head_q (ip_off!=0) AND within the valid region (ip_off<count).
            // MF4: never the post-skip head entry -- the head path owns it (makes the
            // ip/head no-collision invariant true by construction, not by interlock luck).
            ip_inrange_c = (ip_off_q != '0) && (ip_off_q < count_q) &&
                           (issue_ptr_q != head_next_skip);
            // A3: never issue (nor write issued_load) for an entry being squashed this
            // cycle -- it would resurrect an entry block M zeroes (entries_premerge).
            ip_notsquash_c =
                ((entries_q[issue_ptr_q].entry.branch_mask & abort_mask) == '0);
            // MF6: the issuing ip load goes to xlate_pa_q's PHYSICAL 64B line (A4), so
            // the same-line guard compares the issued PHYSICAL line, not virtual addrs.
            ip_same_line_c = 1'b0;
            for (int k = 0; k < LSQ_MLP; k += 1) begin
                if (inflight_valid_q[k] &&
                    (inflight_line_q[k] ==
                     xlate_pa_q[XLEN-1:ADDR_SHIFT+LINE_OFF_W]))
                    ip_same_line_c = 1'b1;
            end
            if (ip_ready_c && ip_carve_c && ip_inrange_c && ip_notsquash_c &&
                    !ip_same_line_c &&
                    // this cycle the port must ALSO be aimed at issue_ptr, so xlate_pa_q
                    // is issue_ptr's registered PA (the issued load's actual address).
                    (xlate_target_q == issue_ptr_q) &&
                    (inflight_count_q < CNT_W'(LSQ_MLP)) && dmem_req_ready &&
                    !head_data_load_en && !double_store_pending_q &&
                    !sc_issue && !amo_issue && !commit_store) begin
                ip_issue_fire = 1'b1;
                // A4: issue at the registered PA (store_second_beat=1 in block M).
                ip_load_addr_word = xlate_pa_q[XLEN-1:ADDR_SHIFT];
                // A3: mark the ip-issued entry issued (else it re-issues at the head).
                ip_delta_we_issued = 1'b1;
                ip_delta_idx = issue_ptr_q;
            end
        end
`endif

        // Flush: suppress this cycle's load issue + writeback + probe, and the
        // outstanding-load bookkeeping. Committed store WRITES are handled (and
        // intentionally preserved) by M. head_delta is moot under flush -- M / block
        // 2 zero the whole queue.
        if (flush) begin
            load_writeback = '0;
            head_data_load_en = 1'b0;
            store_probe_hi_next = 1'b0;
`ifdef LSQ_MLP2
            ip_issue_fire = 1'b0;    // no new issue under flush (drain only)
`endif
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

`ifdef LSQ_MLP2
        // A3: mark the issue_ptr-issued entry as issued (so it does not re-issue when it
        // reaches the head). Applied after the squash-zero; ip_issue_fire is gated on the
        // entry NOT being squashed this cycle (ip_notsquash_c), so this never resurrects a
        // zeroed entry. ip_delta_idx != head_next_skip (ip_inrange_c) => no head_delta clash.
        if (ip_delta_we_issued)
            entries_premerge[ip_delta_idx].issued_load = 1'b1;
`endif

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
`ifdef LSQ_MLP2
        data_load_en = head_data_load_en || ip_issue_fire;
`else
        data_load_en = head_data_load_en;
`endif
        data_addr = '0;
        data_store = '0;
        data_store_mask = '0;
        store_second_beat = 1'b0;
        // M3d Stage 2: the typed op follows the same priority as the data fields
        // (double-store beat / store-commit are STORE; a head load issue carries its
        // LOAD/LR/AMO_RD tag). Idle -> don't-care (the port is invalid).
        dmem_req_op = DMEM_OP_LOAD;
        dmem_req_amo = 4'd0;            // M4 #3: only meaningful on the COP_AMO beat
`ifdef LSQ_MLP2
        // The issuing load (head OR issue_ptr) is allocated the tail (next-free) slot.
        dmem_req_id = inflight_tail_q;
`endif
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
`ifdef LSQ_MLP2
            // A4/MF1: single-beat head loads issue at the effective PA (PA-direct); a
            // two-beat load's beats keep the live-translate path (head_load_pa_direct=0).
            store_second_beat = head_load_pa_direct;
        end else if (ip_issue_fire) begin
            // Track A: lowest-priority port user. A plain cacheable load issued at
            // issue_ptr's REGISTERED PA (A4: PA-direct, decision and address from the
            // same translate -- no live-DTLB-evict desync).
            data_addr = ip_load_addr_word;
            dmem_req_op = DMEM_OP_LOAD;
            store_second_beat = 1'b1;
`endif
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








`ifdef LSQ_MLP2
    // ==== Track A P3b: issue_ptr + inflight-FIFO next-state ====
    // Reads registered state + this cycle's issue/free events (head_data_load_en,
    // ip_issue_fire, data_load_valid) + the head-skip/retire results. NB reads only
    // registered entries_q (never the wakeup array), so it adds no false loop edge.
    always_comb begin
        int unsigned k;
        logic head_issue, ip_issue, resp_free, ip_want_c;
        logic [MEM_PTR_W-1:0] head_owner;
        logic [$clog2(MEM_Q_SIZE+1)-1:0] head_adv_k, ip_off_raw;
        logic ip_advance, head_is_device;
        head_issue = head_data_load_en && !flush;
        ip_issue   = ip_issue_fire;              // already 0 under flush
        resp_free  = data_load_valid;
        head_owner = head_next_skip;             // the effective head's ring index

        // --- xlate_target arbitration (registered next aim; strict head-priority) ---
        // MF4: ip only wants the port for a strictly-younger entry that is NOT the
        // post-skip head (the head path owns head_next_skip).
        head_wants_xlate = headq.entry.valid &&
            (headq.entry.ctrl.memRead || headq.entry.ctrl.memWrite) && !headq.issued_load;
        ip_want_c = entries_q[issue_ptr_q].entry.valid && entries_q[issue_ptr_q].addr_ready &&
            entries_q[issue_ptr_q].entry.ctrl.memRead && !entries_q[issue_ptr_q].entry.ctrl.memWrite &&
            (entries_q[issue_ptr_q].entry.ctrl.exec_class != EXEC_AMO) &&
            !entries_q[issue_ptr_q].issued_load && (issue_ptr_q != head_q) &&
            (issue_ptr_q != head_next_skip) &&
            (inflight_count_q < CNT_W'(LSQ_MLP));
        if (head_wants_xlate)   xlate_target_next = head_q;
        else if (ip_want_c)     xlate_target_next = issue_ptr_q;
        else                    xlate_target_next = head_q;

        // --- A1: step issue_ptr OVER the head's own outstanding load ---
        // issue_ptr advances only on ip_issue, which requires ip_off!=0; nothing steps it
        // past the HEAD's issued load, so absent this it stays pinned at head (MLP never
        // engages). Step iff issue_ptr is at head (ip_off==0), no squash-skip is pending,
        // and the head holds a currently-outstanding PLAIN SINGLE-BEAT CACHEABLE NON-DEVICE
        // load (never a store/AMO/two-beat/device -- those must stay unreorderable). No FIFO
        // alloc: the head's load slot was allocated when the head issued it.
        head_is_device =
            ((entries_q[head_q].store_lo_pa >= XLEN'('h0200_0000)) && (entries_q[head_q].store_lo_pa < XLEN'('h0201_0000))) ||
            ((entries_q[head_q].store_lo_pa >= XLEN'('h0C00_0000)) && (entries_q[head_q].store_lo_pa < XLEN'('h1000_0000))) ||
            ((entries_q[head_q].store_lo_pa >= XLEN'('h0D00_0000)) && (entries_q[head_q].store_lo_pa < XLEN'('h0D00_1000)));
        ip_step = !flush && (ip_off_q == '0) && (head_q == head_next_skip) &&
                  (inflight_count_q >= CNT_W'(1)) && (inflight_count_q < CNT_W'(LSQ_MLP)) &&
                  entries_q[head_q].entry.valid && entries_q[head_q].issued_load &&
                  !entries_q[head_q].load_complete &&
                  entries_q[head_q].entry.ctrl.memRead && !entries_q[head_q].entry.ctrl.memWrite &&
                  (entries_q[head_q].entry.ctrl.exec_class != EXEC_AMO) &&
                  !needs_two_beats(entries_q[head_q].entry.ctrl, entries_q[head_q].addr) &&
                  !head_is_device;
        ip_advance = ip_issue || ip_step;

        // --- A2/MF2/MF5: issue_ptr as an OFFSET from head_q (kills the mod-ring teleport).
        // head_adv_k = how far head advanced this cycle -- NOT 0/1/2: a burst head-skip can
        // retire many at once, so it is the full modular (head_next-head_q). All-unsigned
        // arithmetic: floor to head_next when head passes issue_ptr, else clamp to count_next.
        // Size-cast forces the subtract to MEM_PTR_W self-determined width so it wraps
        // mod MEM_Q_SIZE (a bare `head_next - head_q` would context-extend to count width
        // and NOT wrap at the ring top, over-counting the advance by W on every wrap and
        // yanking a legitimately detached issue_ptr back to the head). [0..W-1].
        head_adv_k = ($clog2(MEM_Q_SIZE))'(head_next - head_q);
        ip_off_raw = '0;   // default (assigned only in the else branch below)
        if (flush) begin
            ip_off_next    = '0;
            issue_ptr_next = head_next;
        end else if ((ip_off_q + CNT_W'(ip_advance)) <= head_adv_k) begin
            ip_off_next    = '0;                               // head passed issue_ptr
            issue_ptr_next = head_next;
        end else begin
            ip_off_raw     = ip_off_q + CNT_W'(ip_advance) - head_adv_k;
            ip_off_next    = (ip_off_raw > count_next) ? count_next : ip_off_raw;
            issue_ptr_next = head_next + ip_off_next[MEM_PTR_W-1:0];
        end

        // --- inflight FIFO next-state: copy, then kill / free / alloc ---
        inflight_head_next  = inflight_head_q;
        inflight_tail_next  = inflight_tail_q;
        inflight_count_next = inflight_count_q;
        for (k = 0; k < LSQ_MLP; k = k + 1) begin
            inflight_valid_next[k] = inflight_valid_q[k];
            inflight_owner_next[k] = inflight_owner_q[k];
            inflight_kill_next[k]  = inflight_kill_q[k];
            inflight_gen_next[k]   = inflight_gen_q[k];
            inflight_line_next[k]  = inflight_line_q[k];
        end
        // (a) monotonic per-slot kill: latch when the owner (while STILL the load) is
        // squashed -- read the already-aged entries_q[owner].branch_mask this cycle
        // (the load being squashed, pre-reallocation). Held until the slot drains.
        for (k = 0; k < LSQ_MLP; k = k + 1) begin
            if (inflight_valid_q[k] &&
                ((entries_q[inflight_owner_q[k]].entry.branch_mask & abort_mask) != '0))
                inflight_kill_next[k] = 1'b1;
        end
        if (flush) begin
            for (k = 0; k < LSQ_MLP; k = k + 1)
                if (inflight_valid_q[k]) inflight_kill_next[k] = 1'b1;
        end
        // (b) free the oldest slot on a response (deliver/discard decided in block HD).
        if (resp_free && (inflight_count_q != '0)) begin
            inflight_valid_next[inflight_head_q] = 1'b0;
            inflight_head_next  = inflight_head_q + LSQ_ID_W'(1);
            inflight_count_next = inflight_count_next - 1'b1;
        end
        // (c) allocate the tail slot on an issue (head OR issue_ptr, mutually exclusive).
        // MF6: latch the PHYSICAL line the load was ISSUED at (head: effective PA; ip:
        // registered PA) so the same-line guard compares physical lines (A4 issues at PA).
        if (head_issue || ip_issue) begin
            inflight_valid_next[inflight_tail_q] = 1'b1;
            inflight_owner_next[inflight_tail_q] = head_issue ? head_owner : issue_ptr_q;
            inflight_gen_next[inflight_tail_q]   =
                entries_q[head_issue ? head_owner : issue_ptr_q].entry.active_id;
            inflight_kill_next[inflight_tail_q]  =
                ((entries_q[head_issue ? head_owner : issue_ptr_q].entry.branch_mask & abort_mask) != '0);
            inflight_line_next[inflight_tail_q]  = head_issue
                ? xlate_pa_eff[XLEN-1:ADDR_SHIFT+LINE_OFF_W]
                : xlate_pa_q[XLEN-1:ADDR_SHIFT+LINE_OFF_W];
            inflight_tail_next  = inflight_tail_q + LSQ_ID_W'(1);
            inflight_count_next = inflight_count_next + 1'b1;
        end
    end
`endif

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
`ifdef LSQ_MLP2
            issue_ptr_q      <= '0;
            ip_off_q         <= '0;
            inflight_head_q  <= '0;
            inflight_tail_q  <= '0;
            inflight_count_q <= '0;
            xlate_target_q   <= '0;
            xlate_tgt_q      <= '0;
            xlate_tgt_aid_q  <= '0;
            for (int k = 0; k < LSQ_MLP; k += 1) begin
                inflight_valid_q[k] <= 1'b0;
                inflight_owner_q[k] <= '0;
                inflight_kill_q[k]  <= 1'b0;
                inflight_gen_q[k]   <= '0;
                inflight_line_q[k]  <= '0;
            end
`endif
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
`ifdef LSQ_MLP2
            // Track A: register this cycle's translate target as the tag for the
            // registered translate result (xlate_pa_q is the translate of mem_req_vaddr,
            // which is aimed at xlate_target_q this cycle).
            issue_ptr_q      <= issue_ptr_next;
            ip_off_q         <= ip_off_next;
            inflight_head_q  <= inflight_head_next;
            inflight_tail_q  <= inflight_tail_next;
            inflight_count_q <= inflight_count_next;
            xlate_target_q   <= xlate_target_next;
            xlate_tgt_q      <= xlate_target_q;
            // MF3: the active_id of the entry this cycle's translate is aimed at, tagging
            // xlate_pa_q so ip can reject a stale registered PA after a squash+reuse.
            xlate_tgt_aid_q  <= entries_q[xlate_target_q].entry.active_id;
            for (int k = 0; k < LSQ_MLP; k += 1) begin
                inflight_valid_q[k] <= inflight_valid_next[k];
                inflight_owner_q[k] <= inflight_owner_next[k];
                inflight_kill_q[k]  <= inflight_kill_next[k];
                inflight_gen_q[k]   <= inflight_gen_next[k];
                inflight_line_q[k]  <= inflight_line_next[k];
            end
`endif
            // Element-wise (not whole-array `entries_q <= entries_next`): a whole
            // unpacked-array NBA trips a Verilator V3Delayed internal error at the
            // larger MEM_Q_SIZE=32 (BIG_LSQ). Behaviourally identical, so the default
            // 16-entry build is unchanged.
            for (int i = 0; i < MEM_Q_SIZE; i += 1)
                entries_q[i] <= entries_next[i];
        end
    end

`ifdef LSQ_MLP2
    // MF2 safety net (sim-only): no valid non-squashed STORE may lie in [head_q, issue_ptr)
    // (offsets 0..ip_off_q-1). issue_ptr only ever advances over the head's own load
    // (ip_step) or plain loads (ip_issue), so a store in that region is a memory-ordering
    // violation (a younger load issued past an older store). issue_ptr ITSELF (offset
    // ip_off_q) may legitimately park at a store, so it is excluded. This catches an A2
    // teleport regression that the pure-load overlap gate cannot expose.
    always_ff @(posedge clk) begin
        if (rst_l && (ip_off_q != '0)) begin
            for (int j = 0; j < MEM_Q_SIZE; j += 1) begin
                if (j < int'(ip_off_q)) begin
                    automatic logic [MEM_PTR_W-1:0] chk = head_q + MEM_PTR_W'(j);
                    if (entries_q[chk].entry.valid &&
                        ((entries_q[chk].entry.branch_mask & abort_mask) == '0) &&
                        entries_q[chk].entry.ctrl.memWrite &&
                        !entries_q[chk].entry.ctrl.memRead)
                        $fatal(1, "LSQ P3b MF2: store at head+%0d inside (head, issue_ptr) ip_off=%0d",
                               j, ip_off_q);
                end
            end
        end
    end
`endif

`ifdef LSQ_MLP_STAT
    // Instrumented overlap gate (the mandatory P3b-fix PASS criterion): PROVES MLP=2
    // engages. On passthrough+fuzz mlp_stream, ip_fires>0 AND two_out>0 => the 2nd load
    // genuinely issues while the head's is outstanding. Separate macro so PERF stays lean.
    longint unsigned mlp_ip_fires, mlp_head_fires, mlp_two_out, mlp_steps;
    always_ff @(posedge clk or negedge rst_l) begin
        if (!rst_l) begin
            mlp_ip_fires <= '0; mlp_head_fires <= '0; mlp_two_out <= '0; mlp_steps <= '0;
        end else begin
            if (ip_issue_fire)                 mlp_ip_fires   <= mlp_ip_fires + 1;
            if (head_data_load_en && !flush)   mlp_head_fires <= mlp_head_fires + 1;
            if (inflight_count_q == CNT_W'(2)) mlp_two_out    <= mlp_two_out + 1;
            if (ip_step)                       mlp_steps      <= mlp_steps + 1;
        end
    end
    final $display("LSQ-MLP-STAT: ip_fires=%0d head_fires=%0d two_out_cycles=%0d steps=%0d",
                   mlp_ip_fires, mlp_head_fires, mlp_two_out, mlp_steps);
`endif

    // P3-M0: categorize why the in-order head is blocked this cycle (display-only;
    // reads only the head state already computed above -> no functional effect).
    // Priority: idle > progressing > load-response-wait > store-park > translate >
    // dmem-port > other (operand-wait). LOADWAIT (mem_inflight) is the MLP=1 tax
    // that P3b/P3c target; STOREPARK is the store->load-forwarding tax (ext. opt A).
    always_comb begin
        if (count_q == '0)
            lsq_head_reason = LSQ_HR_EMPTY;
        else if (head_retire)
            lsq_head_reason = LSQ_HR_READY;
`ifdef LSQ_MLP2
        // A5: under MLP the head's tracker (mem_inflight_q) misses the issue_ptr-issued
        // load's wait; count LOADWAIT whenever ANY slot is outstanding (display-only).
        else if (inflight_count_q != '0)
            lsq_head_reason = LSQ_HR_LOADWAIT;
`else
        else if (mem_inflight_q)
            lsq_head_reason = LSQ_HR_LOADWAIT;
`endif
        else if (headq.entry.valid && headq.entry.ctrl.memWrite && headq.issued_load)
            lsq_head_reason = LSQ_HR_STOREPARK;
        else if (headq.entry.valid && !headq.issued_load &&
                 (headq.entry.ctrl.memRead || headq.entry.ctrl.memWrite) &&
                 !head_xlate_ok)
            lsq_head_reason = LSQ_HR_XLATE;
        else if (headq.entry.valid && !headq.issued_load &&
                 headq.entry.ctrl.memRead && head_xlate_ok && !dmem_req_ready)
            lsq_head_reason = LSQ_HR_MEMPORT;
        else
            lsq_head_reason = LSQ_HR_OTHER;
    end

    // P3 L2a (MEASURE-FIRST, display-only): on a store-park cycle, would a younger
    // load forward from the parked HEAD store? Reads registered entries_q only (the
    // FB2b torn-read rule) and models the L2b stalls so FWD_FULL is not over-counted:
    // device-PA (the store's captured store_lo_pa in the device hole < 0x8000_0000),
    // single-word store/load, no intervening store between head and the load, and
    // byte full-cover. Purely combinational -> optimized away in synthesis.
    logic                          fwd_found_load, fwd_intervening_store;
    logic [$clog2(MEM_Q_SIZE)-1:0] fwd_load_idx;
    logic [XLEN_BYTES-1:0]         fwd_load_full, fwd_load_mask;
    always_comb begin
        lsq_fwd_class         = LSQ_FWD_NA;
        fwd_found_load        = 1'b0;
        fwd_intervening_store = 1'b0;
        fwd_load_idx          = head_next_skip;
        fwd_load_full         = '0;
        fwd_load_mask         = '0;
        if (lsq_head_reason == LSQ_HR_STOREPARK) begin
            // oldest younger eligible plain-integer load; flag any store before it.
            for (int j = 1; j < MEM_Q_SIZE; j += 1) begin
                automatic logic [$clog2(MEM_Q_SIZE)-1:0] fi =
                    head_next_skip + ($clog2(MEM_Q_SIZE))'(j);
                if (!fwd_found_load &&
                        (($clog2(MEM_Q_SIZE+1))'(j) < count_after_skip) &&
                        entries_q[fi].entry.valid) begin
                    if (entries_q[fi].entry.ctrl.memRead &&
                            !entries_q[fi].entry.ctrl.memWrite &&
                            (entries_q[fi].entry.ctrl.exec_class == EXEC_INT) &&
                            entries_q[fi].addr_ready && !entries_q[fi].issued_load) begin
                        fwd_found_load = 1'b1;
                        fwd_load_idx   = fi;
                    end else if (entries_q[fi].entry.ctrl.memWrite) begin
                        fwd_intervening_store = 1'b1;
                    end
                end
            end
            if (!fwd_found_load) begin
                lsq_fwd_class = LSQ_FWD_NOLOAD;
            end else begin
                unique case (entries_q[fwd_load_idx].entry.ctrl.ldst_mode)
                    LDST_D:          fwd_load_full = '1;
                    LDST_W, LDST_WU: fwd_load_full = XLEN_BYTES'('hF);
                    LDST_H, LDST_HU: fwd_load_full = XLEN_BYTES'('h3);
                    LDST_B, LDST_BU: fwd_load_full = XLEN_BYTES'('h1);
                    default:         fwd_load_full = '0;
                endcase
                fwd_load_mask = fwd_load_full <<
                    entries_q[fwd_load_idx].addr[ADDR_SHIFT-1:0];
                if ((headq.addr[XLEN-1:ADDR_SHIFT] !=
                         entries_q[fwd_load_idx].addr[XLEN-1:ADDR_SHIFT]) ||
                        ((fwd_load_mask & headq.store_mask) == '0)) begin
                    lsq_fwd_class = LSQ_FWD_NOMATCH;   // no byte overlap w/ head store
                end else if (((fwd_load_mask & ~headq.store_mask) == '0) &&  // full-cover
                        (headq.store_lo_pa >= XLEN'(64'h8000_0000)) &&        // RAM, not device
                        (headq.store_mask_hi == '0) &&                        // store single-word
                        !needs_two_beats(entries_q[fwd_load_idx].entry.ctrl,
                                         entries_q[fwd_load_idx].addr) &&      // load single-word
                        !fwd_intervening_store) begin
                    lsq_fwd_class = LSQ_FWD_FULL;
                end else begin
                    lsq_fwd_class = LSQ_FWD_PARTIAL;
                end
            end
        end
    end

endmodule: load_store_queue
