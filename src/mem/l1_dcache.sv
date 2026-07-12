/**
 * l1_dcache.sv  (phase C2)
 *
 * Blocking, write-back / write-allocate L1 data cache: 16 KiB, 4-way, 64 B
 * lines, PIPT, per-line dirty, tree-PLRU. One unified word-granular requester
 * (the niigo_memsys front arbiter muxes the LSQ data port and the PTW onto it,
 * LSQ priority). Devices are split off before they reach here, so this cache
 * only ever sees cacheable RAM.
 *
 * Per op: present index (sync arrays) -> compare. Hit load returns the word;
 * hit store byte-writes the word and sets dirty. A miss writes the dirty victim
 * back first (WR_LINE / DWB), refills (RD_LINE / DFILL), then installs and
 * completes -- fully serialised, so no writeback buffer or refill/writeback
 * hazard exists (correctness-first; the pipelined variant is a later perf
 * lever). A flush request walks every set/way and writes back all dirty lines
 * (fence.i ordering + the halt/tohost signature dump); dirty is cleared, lines
 * stay valid.
 */

`include "niigo_mem.vh"

`default_nettype none

module l1_dcache
    import RISCV_ISA::XLEN, RISCV_ISA::XLEN_BYTES;
    import RISCV_UArch::MEMORY_ADDR_WIDTH;
    import NIIGO_Mem::*;
(
    input wire logic clk,
    input wire logic rst_l,

    // ---- Unified D request (from the front arbiter) ----
    input wire logic                          req_valid,
    output logic                          req_ready,
    input wire logic                          req_write,
    input wire logic [MEMORY_ADDR_WIDTH-1:0]  req_waddr,
    input wire logic [XLEN-1:0]               req_wdata,
    input wire logic [XLEN_BYTES-1:0]         req_wmask,
`ifdef LSQ_MLP2
    // Track A: transaction id sidecar. Latched with the op and echoed on resp_valid
    // so the memsys/LSQ can match a load response to its outstanding slot (P3c).
    input wire logic [DMEM_ID_W-1:0]          req_id,
    output logic [DMEM_ID_W-1:0]              resp_id,
    // P3c-3 hit-under-miss: a 2nd op (op2) is accepted during a primary miss (S_WB_WAIT/
    // S_FILL_WAIT), read-only, and if it HITS its data returns on this dedicated resp2 lane;
    // a miss/store/same-set op2 is held and promoted to the next primary. op2_accepting tells
    // the memsys THIS req_fire landed in op2 (not S_IDLE) so it does not clobber owner_ptw_q;
    // op2_promote pulses when a parked op2 is promoted to primary (dmem-only) so the memsys
    // resets owner_ptw_q (B2). All read-only / dmem-only by construction.
    output logic                          resp2_valid,
    output logic [XLEN-1:0]               resp2_data,
    output logic [MEMORY_ADDR_WIDTH-1:0]  resp2_addr,
    output logic [DMEM_ID_W-1:0]          resp2_id,
    output logic                          op2_accepting,
    output logic                          op2_promote_o,
`endif
    output logic                          resp_valid,    // load data this cycle
    output logic [XLEN-1:0]               resp_data,
    output logic [MEMORY_ADDR_WIDTH-1:0]  resp_addr,     // echo (word addr)
    output logic                          wr_accept,     // store accepted (PTW write ack)

    // ---- Flush: write back all dirty lines (fence.i + halt) ----
    input wire logic                          flush_req,
    output logic                          flush_done,

    // ---- C4a coherence probe (clean-before-refill) ----
    // niigo_memsys asserts probe_valid (level) with the line word address an
    // L1I refill is about to fetch. If this cache holds that line dirty, it is
    // written back here first; probe_clean then rises and the memsys releases
    // the held I-refill so it reads the now-current line from memory. A probe
    // miss / clean hit raises probe_clean immediately. The line stays valid.
    input wire logic                          probe_valid,
    input wire logic [MEMORY_ADDR_WIDTH-1:0]  probe_waddr,
    output logic                          probe_clean,

    // ---- NMI master ----
    output nmi_req_t                      nmi_req,
    input wire logic                          nmi_req_ready,
    input wire nmi_resp_t                     nmi_resp,

    // ---- C3 observability (pulses) ----
    output logic                          ev_access,
    output logic                          ev_miss,
    output logic                          ev_wb
);

    localparam int LB    = LINE_BITS;
    localparam int LBY   = LINE_BITS/8;
    localparam int WB_W  = XLEN;
    localparam int WBY   = XLEN_BYTES;
`ifdef LSQ_MLP2
    localparam int DMEM_ID_W = 1;   // Track A dmem txn-id width (LSQ_MLP<=2 => 1 bit)
`endif

    // ---------------- arrays + flop metadata ----------------
    logic                           tag_ren, tag_wen;
    logic [L1_INDEX_BITS-1:0]       tag_ridx, tag_widx;
    logic [L1_WAY_BITS-1:0]         tag_wway;
    logic [L1_TAG_BITS-1:0]         tag_wtag;
    logic [L1_WAYS-1:0][L1_TAG_BITS-1:0] tag_rdata;

    logic                           dat_ren, dat_wen;
    logic [L1_INDEX_BITS-1:0]       dat_ridx, dat_widx;
    logic [L1_WAY_BITS-1:0]         dat_wway;
    logic [LB-1:0]                  dat_wdata;
    logic [LBY-1:0]                 dat_wmask;
    logic [L1_WAYS-1:0][LB-1:0]     dat_rdata;

    // The L1D probe (C4a) reuses the main read port in S_PROBE_LOOK, so the tag
    // array is a clean single-port (1RW) SRAM under the ASAP7 macro build.
    l1_tag_array #(.SETS(L1_SETS), .WAYS(L1_WAYS), .TAG_BITS(L1_TAG_BITS)) Tags (
        .clk, .ren(tag_ren), .ridx(tag_ridx), .rtag(tag_rdata),
        .wen(tag_wen), .widx(tag_widx), .wway(tag_wway), .wtag(tag_wtag)
    );
    l1_data_array #(.SETS(L1_SETS), .WAYS(L1_WAYS), .LINE_BITS(LB)) Data (
        .clk, .ren(dat_ren), .ridx(dat_ridx), .rdata(dat_rdata),
        .wen(dat_wen), .widx(dat_widx), .wway(dat_wway),
        .wdata(dat_wdata), .wmask(dat_wmask)
    );

    logic [L1_WAYS-1:0] valid_q [L1_SETS];
    logic [L1_WAYS-1:0] valid_n [L1_SETS];
    logic [L1_WAYS-1:0] dirty_q [L1_SETS];
    logic [L1_WAYS-1:0] dirty_n [L1_SETS];
    logic [2:0]         plru_q  [L1_SETS];
    logic [1:0]         gen_q;

    // ---------------- request latch ----------------
    logic                          op_valid_q;
    logic                          op_write_q;
    logic [MEMORY_ADDR_WIDTH-1:0]  op_addr_q;
    logic [WB_W-1:0]               op_wdata_q;
    logic [WBY-1:0]                op_wmask_q;
`ifdef LSQ_MLP2
    logic [DMEM_ID_W-1:0]          op_id_q;   // Track A: id of the in-flight op, echoed on resp
    // ---------------- P3c-3 op2 latch + phase (accept-and-park hit-under-miss) ----------------
    // P3d-2 extends the phase enum with a CONCURRENT MSHR fill leg (OP2_MISS_*): a different-set
    // LOAD MISS whose victim is invalid/CLEAN launches its own fill concurrently with the primary
    // miss instead of parking. Clean-victim only (dirty/same-set/store keep the P3c PARK+promote):
    // this is coherence-safe by construction (a load fill only copies memory into a CLEAN line, so
    // it never creates dirty state and cannot violate the C4a clean-before-refill / SMC contract).
    typedef enum logic [2:0] {
        OP2_EMPTY, OP2_PEND, OP2_READ, OP2_PARK,
        OP2_MISS_FILLREQ, OP2_MISS_FILLWAIT, OP2_MISS_INSTALL
    } op2_phase_e;
    op2_phase_e                    op2_phase_q, op2_phase_n2;
    logic                          op2_write_q;
    logic [MEMORY_ADDR_WIDTH-1:0]  op2_addr_q;
    logic [WB_W-1:0]               op2_wdata_q;
    logic [WBY-1:0]                op2_wmask_q;
    logic [DMEM_ID_W-1:0]          op2_id_q;
    wire op2_occupied = (op2_phase_q != OP2_EMPTY);   // review's "op2_valid" (single source of truth)
    wire [L1_INDEX_BITS-1:0]  op2_idx  = l1_index(op2_addr_q);
    wire [L1_TAG_BITS-1:0]    op2_tag  = l1_tag(op2_addr_q);
    wire [LINE_WORD_BITS-1:0] op2_woff = l1_word_off(op2_addr_q);
    // P3d-2 concurrent op2-MSHR fill-leg context (clean-victim different-set LOAD miss only).
    wire op2_mshr_active = (op2_phase_q == OP2_MISS_FILLREQ) ||
                           (op2_phase_q == OP2_MISS_FILLWAIT) ||
                           (op2_phase_q == OP2_MISS_INSTALL);
    logic [LB-1:0]          op2_refill_line_q, op2_refill_line_n;
    logic [L1_WAY_BITS-1:0] op2_fill_way_q, op2_fill_way_n;
    logic [3:0]            op2_exp_fill_id_q;   // {DFILL,gen} the op2 fill was issued with (P3d-0 id-match)
`endif

    // VIPT seam (M2; plans/multicore-ccd.md §V): the set index is taken from the page-offset
    // bits, which are translation-invariant (VA[idx] == PA[idx], alias-free per L1_VIPT_ALIAS_FREE
    // below), so it is a VA-sourced index in all but name; the tag compare is physical (op_tag, a PA
    // tag). A coherence snoop/probe indexes with the PA over the same bits -> the SAME set, no synonym
    // search (see the C4a probe port). In the single core, translation is upstream of the cache so the
    // request address already carries the resolved PA; true VA-early indexing (TLB-overlap) is a
    // descoped FPGA-perf lever, not needed here. The L1D coherence state (MOESI) + a 2nd snoop-data
    // port + the CMI interface are the M3 coherence-integration step.
    logic [L1_INDEX_BITS-1:0]  op_idx;   // VIPT index  (page-offset bits -> translation-invariant)
    logic [L1_TAG_BITS-1:0]    op_tag;   // physical tag (PA)
    logic [LINE_WORD_BITS-1:0] op_woff;
    assign op_idx  = l1_index(op_addr_q);
    assign op_tag  = l1_tag(op_addr_q);
    assign op_woff = l1_word_off(op_addr_q);

    // The whole VIPT/PA-snoop correctness rests on the index lying within the page offset.
    initial assert (L1_VIPT_ALIAS_FREE)
        else $fatal(1, "l1_dcache: VIPT alias-free invariant violated (way_size %0d B > page %0d B)",
                    L1_WAY_BYTES, PAGE_BYTES);

    typedef enum logic [3:0] {
        S_IDLE, S_LOOKUP, S_WB_REQ, S_WB_WAIT, S_FILL_REQ, S_FILL_WAIT, S_INSTALL,
        S_FLUSH_READ, S_FLUSH_SCAN, S_FLUSH_WB_REQ, S_FLUSH_WB_WAIT, S_FLUSH_DONE,
        S_PROBE_LOOK, S_PROBE_WB_REQ, S_PROBE_WB_WAIT   // 0..14 (encoding frozen for OFF byte-identity)
`ifdef LSQ_MLP2
        , S_OP2_PROMOTE                                 // 15: re-lookup a promoted parked op2 (P3c-3)
`endif
    } state_e;
    state_e state_q, state_n;

    logic [LB-1:0]             refill_line_q, refill_line_n;
    logic [L1_WAY_BITS-1:0]    fill_way_q, fill_way_n;
    logic [LB-1:0]             wb_line_q,  wb_line_n;
    logic [MEMORY_ADDR_WIDTH-1:0] wb_addr_q, wb_addr_n;
    logic [L1_INDEX_BITS-1:0]  flush_set_q, flush_set_n;
    logic [L1_WAY_BITS-1:0]    flush_way_q, flush_way_n;
`ifdef LSQ_MLP2
    // P3d-0: the {src,gen} NMI id the in-flight WB / FILL was ISSUED with, latched at the
    // S_WB_REQ / S_FILL_REQ fire edge (co-located with the gen_q bump below). The WAIT exits
    // consume nmi_resp ONLY when its id matches -- so once the fabric is multi-outstanding
    // (P3d-1/P3d-3) a mismatched response can never be mistaken for this MSHR's (silent
    // corruption). Cycle-identical single-outstanding (the sole in-flight op's id always matches).
    logic [3:0]  exp_wb_id_q;
    logic [3:0]  exp_fill_id_q;
`endif

    // ---------------- C4a probe state ----------------
    logic [L1_INDEX_BITS-1:0]  pidx_q, pidx_n;
    logic [L1_TAG_BITS-1:0]    ptag_q, ptag_n;
    logic [L1_WAY_BITS-1:0]    pway_q, pway_n;
    logic                      probe_done_q, probe_done_n;  // this probe resolved
    logic                      probe_start;                 // begin a probe in S_IDLE
    // P3d-2: hold a probe off while the op2-MSHR is filling/installing -- keeps the C4a probe read
    // from racing the op2 install on the single-port array, and the probe then snoops a quiescent
    // array. (op2 fills are clean-only so this is a latency guard, not a coherence requirement.)
    assign probe_start = probe_valid && !probe_done_q && !flush_req
`ifdef LSQ_MLP2
                         && !op2_mshr_active
`endif
                         ;
    assign probe_clean = probe_done_q;

    // ---------------- hit detection ----------------
    logic [L1_WAYS-1:0]     hit_oh;
    logic                   any_hit;
    logic [L1_WAY_BITS-1:0] hit_way;
    always_comb begin
        for (int w = 0; w < L1_WAYS; w += 1)
            hit_oh[w] = valid_q[op_idx][w] && (tag_rdata[w] == op_tag);
        any_hit = |hit_oh;
        hit_way = '0;
        for (int w = 0; w < L1_WAYS; w += 1) if (hit_oh[w]) hit_way = L1_WAY_BITS'(w);
    end

    // ---------------- C4a probe hit detection (S_PROBE_LOOK) ----------------
    // Reuses the main tag/data read port: S_IDLE presents the probe index, the
    // result lands in S_PROBE_LOOK. A dirty hit is written back; the line stays
    // valid-clean. Probes only run from S_IDLE, so they never clash with a live
    // load/store op (req acceptance is blocked while a probe is pending).
    logic [L1_WAYS-1:0]     probe_hit_oh;
    logic                   probe_any_hit, probe_dirty;
    logic [L1_WAY_BITS-1:0] probe_hit_way;
    always_comb begin
        for (int w = 0; w < L1_WAYS; w += 1)
            probe_hit_oh[w] = valid_q[pidx_q][w] && (tag_rdata[w] == ptag_q);
        probe_any_hit = |probe_hit_oh;
        probe_hit_way = '0;
        for (int w = 0; w < L1_WAYS; w += 1) if (probe_hit_oh[w]) probe_hit_way = L1_WAY_BITS'(w);
        probe_dirty = probe_any_hit && dirty_q[pidx_q][probe_hit_way];
    end

    // ---------------- PLRU ----------------
    logic [L1_WAY_BITS-1:0] victim;
    logic                   plru_upd_en;
    logic [L1_WAY_BITS-1:0] plru_acc_way;
    logic [2:0]             plru_next;
    l1_plru #(.WAYS(L1_WAYS)) Plru (
        .state(plru_q[op_idx]), .valid(valid_q[op_idx]), .victim(victim),
        .update_en(plru_upd_en), .access_way(plru_acc_way), .next_state(plru_next)
    );
`ifdef LSQ_MLP2
    // P3d-2: a 2nd PLRU for the op2-MSHR's victim in op2_idx (a DIFFERENT set from op_idx, so the
    // two never contend). Its update rides the op2-MSHR install (plru_q[op2_idx] <= op2_plru_next).
    logic [L1_WAY_BITS-1:0] op2_victim;
    logic                   op2_plru_upd_en;
    logic [2:0]             op2_plru_next;
    l1_plru #(.WAYS(L1_WAYS)) Plru2 (
        .state(plru_q[op2_idx]), .valid(valid_q[op2_idx]), .victim(op2_victim),
        .update_en(op2_plru_upd_en), .access_way(op2_fill_way_q), .next_state(op2_plru_next)
    );
    assign op2_plru_upd_en = op2_install_fire;   // touch the op2 way on its install
    // The op2-MSHR launches only when its victim needs no writeback (invalid or CLEAN) -- keeps the
    // leg WB-free (coherence-safe, no C2 cross-MSHR WB) so a dirty victim keeps the P3c PARK fallback.
    wire op2_victim_clean = !(valid_q[op2_idx][op2_victim] && dirty_q[op2_idx][op2_victim]);
    // op2 load-miss to a different set (the only op2 kind that launches an MSHR; else PARK).
    wire op2_is_diffset_load_miss = (op2_phase_q == OP2_READ) && !op2_write_q &&
                                    !op2_any_hit && (op2_idx != op_idx);
    // The concurrent-MSHR launch edge (1-cycle, at OP2_READ): a genuine access + line miss.
    wire op2_mshr_launch = op2_is_diffset_load_miss && op2_victim_clean;
`endif

    // Reconstruct the byte (word) address of a (tag, set) line.
    function automatic logic [MEMORY_ADDR_WIDTH-1:0]
            line_addr(input logic [L1_TAG_BITS-1:0] t, input logic [L1_INDEX_BITS-1:0] s);
        line_addr = {t, s, {LINE_WORD_BITS{1'b0}}};
    endfunction

    // Merge the store word into a full line image (for a store miss install).
    logic [LB-1:0] install_line;
    always_comb begin
        install_line = refill_line_q;
        if (op_write_q) begin
            for (int b = 0; b < WBY; b += 1) begin
                if (op_wmask_q[b])
                    install_line[(op_woff*WBY + b)*8 +: 8] = op_wdata_q[b*8 +: 8];
            end
        end
    end

    // ---------------- control ----------------
    logic req_fire, serve_hit, serve_miss, store_hit;
`ifdef LSQ_MLP2
    // P3c-3: the read port is idle throughout a primary miss's WAIT states, so a 2nd HIT-ONLY
    // op (op2) can borrow it. Accept op2 when: in a WAIT state, op2 slot empty, no probe pending,
    // no flush queued. Every term is REGISTERED state / an out-of-band input -- NONE touches
    // req_write/req_waddr/req_id -- so req_ready never depends on op kind (same loop-freedom as the
    // memsys dmem_req_ready = dev_ready && l1d_req_ready). op2_accept_ready is disjoint from the
    // S_IDLE term (op2 is only accepted in a WAIT state, S_IDLE is not a WAIT state).
    wire op2_accept_ready = (state_q == S_WB_WAIT || state_q == S_FILL_WAIT) &&
                            !op2_occupied && !probe_valid && !flush_req;
    // P3d-2 (B-NEWPRIM): while the op2-MSHR is filling, the primary FSM may sit in S_IDLE but must
    // NOT start a FRESH primary (nor a probe) -- that would exceed the LSQ_MLP=2 budget and let a
    // probe race the op2 install on the single-port array. So the S_IDLE accept term also gates on
    // !op2_mshr_active.
    assign req_ready = ((state_q == S_IDLE) && !flush_req && !probe_start && !op2_mshr_active)
                       || op2_accept_ready;
    assign op2_accepting = op2_accept_ready;
    wire op2_latch_fire = req_fire && op2_accept_ready;
    // op2 borrows the read port ONE cycle inside a WAIT state (the !nmi_resp.valid gate keeps the
    // resulting OP2_READ cycle STILL inside that WAIT state -- the B3-review no-skid proof: resp2
    // then fires only in {S_WB_WAIT,S_FILL_WAIT}, disjoint from resp_valid's {S_LOOKUP,S_INSTALL}).
    wire op2_read_present = (state_q == S_WB_WAIT || state_q == S_FILL_WAIT) &&
                            (op2_phase_q == OP2_PEND) && !nmi_resp.valid;
    // op2 promotes to primary when the primary install completes and the op2 still needs primary
    // processing -- i.e. it is occupied but NOT an active op2-MSHR (which self-completes on resp2).
    // This is the P3c-3 promote (any occupied op2: PEND that never got to read, or PARK) MINUS the
    // MSHR phases. Critically it MUST include OP2_PEND: an op2 accepted late in the primary's miss
    // can be stuck in PEND when the primary installs (op2_read_present is blocked by nmi_resp.valid
    // on the transition cycle); orphaning it (never promoting) freezes its LSQ slot -> memq deadlock.
    wire op2_promote = (state_q == S_INSTALL) && op2_occupied && !op2_mshr_active;
    assign op2_promote_o = op2_promote;
    // ---- P3d-2 concurrent op2-MSHR shared-resource arbitration (primary always wins) ----
    // NMI master port: the primary requests in its REQ states; the op2-MSHR fill issues only when
    // the primary is not requesting (its FILL_WAIT window). Single tag/data write port: the op2-MSHR
    // install fires only when neither a store-hit byte-write nor the primary S_INSTALL is writing.
    wire primary_nmi_req = (state_q == S_WB_REQ) || (state_q == S_FILL_REQ) ||
                           (state_q == S_FLUSH_WB_REQ) || (state_q == S_PROBE_WB_REQ);
    wire op2_fill_present = (op2_phase_q == OP2_MISS_FILLREQ) && !primary_nmi_req;
    wire op2_fill_req_fire = op2_fill_present && nmi_req_ready;
    wire primary_install   = store_hit || (state_q == S_INSTALL);
    wire op2_install_fire  = (op2_phase_q == OP2_MISS_INSTALL) && !primary_install;
`else
    // A pending probe takes priority over a new request in S_IDLE.
    assign req_ready = (state_q == S_IDLE) && !flush_req && !probe_start;
`endif
    assign req_fire  = req_valid && req_ready;
    assign serve_hit  = (state_q == S_LOOKUP) && op_valid_q &&  any_hit;
    assign serve_miss = (state_q == S_LOOKUP) && op_valid_q && !any_hit;
    assign store_hit  = serve_hit && op_write_q;

`ifdef LSQ_MLP2
    // ---- op2 hit detection (valid only in OP2_READ, when tag_rdata holds op2's set) ----
    logic [L1_WAYS-1:0]     op2_hit_oh;
    logic                   op2_any_hit;
    logic [L1_WAY_BITS-1:0] op2_hit_way;
    always_comb begin
        for (int w = 0; w < L1_WAYS; w += 1)
            op2_hit_oh[w] = valid_q[op2_idx][w] && (tag_rdata[w] == op2_tag);
        op2_any_hit = |op2_hit_oh;
        op2_hit_way = '0;
        for (int w = 0; w < L1_WAYS; w += 1) if (op2_hit_oh[w]) op2_hit_way = L1_WAY_BITS'(w);
    end
    // SERVE (read-only): an op2 that is a cacheable LOAD HIT to a DIFFERENT SET than the in-flight
    // miss (op_idx). op2_idx==op_idx PARKS (same-set interlock, C1: forecloses same-line-refill /
    // victim-way / S_INSTALL-alias in one compare). A store op2 (op2_write_q) never serves -> parks.
    wire op2_serve_hit = (op2_phase_q == OP2_READ) && !op2_write_q &&
                         op2_any_hit && (op2_idx != op_idx);
    // resp2 returns EITHER an early op2 HIT (read from the array) OR the op2-MSHR's INSTALL word
    // (read from its captured refill line). Both are mutually exclusive with the primary resp_valid:
    // op2_serve_hit fires in {S_WB_WAIT,S_FILL_WAIT}; op2_install_fire fires only while op2_mshr_active,
    // during which req_ready's S_IDLE term is gated off so the primary can never be at S_LOOKUP/S_INSTALL
    // (asserted by !(resp_valid && resp2_valid) below).
    assign resp2_valid = op2_serve_hit || op2_install_fire;
    assign resp2_data  = op2_serve_hit ? dat_rdata[op2_hit_way][op2_woff*XLEN +: XLEN]
                                       : op2_refill_line_q[op2_woff*XLEN +: XLEN];
    assign resp2_addr  = op2_addr_q;
    assign resp2_id    = op2_id_q;
`endif

    // Load response: a hit in S_LOOKUP, or the install of a load miss.
    logic load_resp_lookup, load_resp_install;
    assign load_resp_lookup  = serve_hit && !op_write_q;
    assign load_resp_install = (state_q == S_INSTALL) && !op_write_q;
    assign resp_valid = load_resp_lookup || load_resp_install;
    assign resp_addr  = op_addr_q;
`ifdef LSQ_MLP2
    assign resp_id    = op_id_q;
`endif
    always_comb begin
        if (load_resp_install) resp_data = refill_line_q[op_woff*XLEN +: XLEN];
        else                   resp_data = dat_rdata[hit_way][op_woff*XLEN +: XLEN];
    end
    // Store accept (PTW write ack): hit store in S_LOOKUP, or install of a store
    // miss. (The LSQ ignores this; the PTW uses it as its write ack.)
    assign wr_accept = store_hit || ((state_q == S_INSTALL) && op_write_q);

    // M7: ev_access is a continuous assign, so an op2 hit (a genuine access+hit) is OR-ed into
    // the single driver -- NOT a procedural |= (which would be a 2nd driver, illegal).
    // P3d-2 (F3): the op2-MSHR launch is a genuine access+MISS -- count it in ev_access/ev_miss so
    // the C3 HPM counters stay consistent with the op2-hit case (observability; the launch is a
    // 1-cycle OP2_READ edge). op2_mshr_launch is defined with the op2 control below.
    assign ev_access = serve_hit || serve_miss
`ifdef LSQ_MLP2
                       || op2_serve_hit || op2_mshr_launch
`endif
                       ;
    assign ev_miss   = serve_miss
`ifdef LSQ_MLP2
                       || op2_mshr_launch
`endif
                       ;
    assign ev_wb     = (state_q == S_WB_REQ && nmi_req_ready) ||
                       (state_q == S_FLUSH_WB_REQ && nmi_req_ready) ||
                       (state_q == S_PROBE_WB_REQ && nmi_req_ready);

    assign flush_done = (state_q == S_FLUSH_DONE);

    // NMI request.
    always_comb begin
        nmi_req = '0;
        unique case (state_q)
            S_WB_REQ, S_FLUSH_WB_REQ, S_PROBE_WB_REQ: begin
                nmi_req.valid = 1'b1;
                nmi_req.op    = NMI_WR_LINE;
                nmi_req.waddr = wb_addr_q;
                nmi_req.id    = {NMI_SRC_DWB, gen_q};
                nmi_req.wdata = wb_line_q;
            end
            S_FILL_REQ: begin
                nmi_req.valid = 1'b1;
                nmi_req.op    = NMI_RD_LINE;
                nmi_req.waddr = l1_line_base(op_addr_q);
                nmi_req.id    = {NMI_SRC_DFILL, gen_q};
            end
            default: ;
        endcase
`ifdef LSQ_MLP2
        // P3d-2: the concurrent op2-MSHR fill borrows the NMI port ONLY when the primary is not
        // requesting (op2_fill_present already ANDs !primary_nmi_req), with a DISTINCT gen (gen_q
        // increments on this fire too, below) so its {DFILL,gen} id never aliases the primary's.
        if (op2_fill_present) begin
            nmi_req.valid = 1'b1;
            nmi_req.op    = NMI_RD_LINE;
            nmi_req.waddr = l1_line_base(op2_addr_q);
            nmi_req.id    = {NMI_SRC_DFILL, gen_q};
        end
`endif
    end

    // Array read presentation.
    always_comb begin
        tag_ren = 1'b0; tag_ridx = op_idx;
        dat_ren = 1'b0; dat_ridx = op_idx;
        if ((state_q == S_IDLE) && probe_start) begin
            // Probe lookup has priority over a new request (req_ready is low).
            tag_ren = 1'b1; tag_ridx = l1_index(probe_waddr);
            dat_ren = 1'b1; dat_ridx = l1_index(probe_waddr);
        end else if ((state_q == S_IDLE) && req_fire) begin
            tag_ren = 1'b1; tag_ridx = l1_index(req_waddr);
            dat_ren = 1'b1; dat_ridx = l1_index(req_waddr);
        end else if (state_q == S_FLUSH_READ) begin
            tag_ren = 1'b1; tag_ridx = flush_set_q;
            dat_ren = 1'b1; dat_ridx = flush_set_q;
`ifdef LSQ_MLP2
        // P3c-3: op2 borrows the idle read port for one cycle inside a WAIT state (op2_read_present),
        // and again for the promoted op2's re-lookup (S_OP2_PROMOTE). Both are in cycles with NO array
        // write (dat_wen/tag_wen assert only in store_hit@S_LOOKUP and S_INSTALL), so read and write are
        // always in disjoint cycles (B7; asserted below).
        end else if (op2_read_present) begin
            tag_ren = 1'b1; tag_ridx = op2_idx;
            dat_ren = 1'b1; dat_ridx = op2_idx;
        end else if (state_q == S_OP2_PROMOTE) begin
            tag_ren = 1'b1; tag_ridx = op_idx;   // op_idx is now the promoted op2's index
            dat_ren = 1'b1; dat_ridx = op_idx;
`endif
        end
    end

    // Array writes.
    always_comb begin
        tag_wen   = 1'b0; tag_widx = op_idx; tag_wway = fill_way_q; tag_wtag = op_tag;
        dat_wen   = 1'b0; dat_widx = op_idx; dat_wway = fill_way_q;
        dat_wdata = install_line; dat_wmask = '1;
        if (store_hit) begin
            // Byte-write the store word into the hit way.
            dat_wen   = 1'b1;
            dat_widx  = op_idx;
            dat_wway  = hit_way;
            dat_wdata = '0;
            dat_wmask = '0;
            for (int b = 0; b < WBY; b += 1) begin
                if (op_wmask_q[b]) begin
                    dat_wdata[(op_woff*WBY + b)*8 +: 8] = op_wdata_q[b*8 +: 8];
                    dat_wmask[op_woff*WBY + b]          = 1'b1;
                end
            end
        end else if (state_q == S_INSTALL) begin
            tag_wen = 1'b1;
            dat_wen = 1'b1;       // full-line install (store bytes merged in)
`ifdef LSQ_MLP2
        end else if (op2_install_fire) begin
            // P3d-2: the op2-MSHR installs its clean load fill at op2_idx/op2_fill_way. Fixed
            // priority (store_hit > primary S_INSTALL > op2 install; op2_install_fire already ANDs
            // !primary_install), and op2_idx != op_idx so the two never target the same set.
            tag_wen   = 1'b1; tag_widx = op2_idx; tag_wway = op2_fill_way_q; tag_wtag = op2_tag;
            dat_wen   = 1'b1; dat_widx = op2_idx; dat_wway = op2_fill_way_q;
            dat_wdata = op2_refill_line_q; dat_wmask = '1;
`endif
        end
    end

    // PLRU update.
    always_comb begin
        plru_upd_en  = 1'b0;
        plru_acc_way = hit_way;
        if (serve_hit) begin
            plru_upd_en  = 1'b1;
            plru_acc_way = hit_way;
        end else if (state_q == S_INSTALL) begin
            plru_upd_en  = 1'b1;
            plru_acc_way = fill_way_q;
        end
    end

    // ---------------- next-state ----------------
    always_comb begin
        state_n       = state_q;
        refill_line_n = refill_line_q;
        fill_way_n    = fill_way_q;
        wb_line_n     = wb_line_q;
        wb_addr_n     = wb_addr_q;
        flush_set_n   = flush_set_q;
        flush_way_n   = flush_way_q;
        pidx_n        = pidx_q;
        ptag_n        = ptag_q;
        pway_n        = pway_q;
        probe_done_n  = probe_done_q;

        unique case (state_q)
            S_IDLE: begin
                if (flush_req
`ifdef LSQ_MLP2
                    // P3d-2 (F1): drain the op2-MSHR before the flush walk -- else the flush read
                    // could race the op2 install on the single-port array. Today masked (fence.i is
                    // serializing so op2_mshr_active is already 0), but gated + asserted defensively.
                    && !op2_mshr_active
`endif
                    ) begin
                    flush_set_n = '0;
                    state_n     = S_FLUSH_READ;
                end else if (probe_start) begin
                    // Snoop the line the held I-refill wants (read presented above).
                    pidx_n  = l1_index(probe_waddr);
                    ptag_n  = l1_tag(probe_waddr);
                    state_n = S_PROBE_LOOK;
                end else if (req_fire) begin
                    state_n = S_LOOKUP;
                end
            end

            S_PROBE_LOOK: begin
                if (probe_dirty) begin
                    wb_line_n = dat_rdata[probe_hit_way];
                    wb_addr_n = line_addr(tag_rdata[probe_hit_way], pidx_q);
                    pway_n    = probe_hit_way;
                    state_n   = S_PROBE_WB_REQ;
                end else begin
                    probe_done_n = 1'b1;   // miss / clean hit: nothing to drain
                    state_n      = S_IDLE;
                end
            end
            S_PROBE_WB_REQ:  if (nmi_req_ready)  state_n = S_PROBE_WB_WAIT;
            S_PROBE_WB_WAIT: if (nmi_resp.valid) begin
                probe_done_n = 1'b1;       // dirty line now in memory; refill may go
                state_n      = S_IDLE;
            end

            S_LOOKUP: begin
                if (any_hit) begin
                    state_n = S_IDLE;            // hit completes here
                end else begin
                    fill_way_n = victim;
                    if (valid_q[op_idx][victim] && dirty_q[op_idx][victim]) begin
                        wb_line_n = dat_rdata[victim];
                        wb_addr_n = line_addr(tag_rdata[victim], op_idx);
                        state_n   = S_WB_REQ;
                    end else begin
                        state_n = S_FILL_REQ;
                    end
                end
            end

            S_WB_REQ:   if (nmi_req_ready)  state_n = S_WB_WAIT;
`ifdef LSQ_MLP2
            // P3d-0: id-matched WAIT consumption. Cycle-identical here (single-outstanding =>
            // nmi_resp.id always == exp_*), load-bearing once the fabric reorders (P3d-1/3).
            S_WB_WAIT:  if (nmi_resp.valid && (nmi_resp.id == exp_wb_id_q)) state_n = S_FILL_REQ;
            S_FILL_REQ: if (nmi_req_ready)  state_n = S_FILL_WAIT;
            S_FILL_WAIT: if (nmi_resp.valid && (nmi_resp.id == exp_fill_id_q)) begin
                refill_line_n = nmi_resp.rdata;
                state_n       = S_INSTALL;
            end
`else
            S_WB_WAIT:  if (nmi_resp.valid) state_n = S_FILL_REQ;
            S_FILL_REQ: if (nmi_req_ready)  state_n = S_FILL_WAIT;
            S_FILL_WAIT: if (nmi_resp.valid) begin
                refill_line_n = nmi_resp.rdata;
                state_n       = S_INSTALL;
            end
`endif
`ifdef LSQ_MLP2
            // P3c-3: drain a parked op2 (promote it to primary) before returning idle. The op2
            // registers into op_*_q on this edge (op-latch below), so S_OP2_PROMOTE presents its
            // set read against the POST-INSTALL array and S_LOOKUP resolves it fresh (M8: this arm
            // is mandatory -- default would fall to S_IDLE and silently drop the promoted op).
            // P3d-2: promote any occupied op2 EXCEPT an active op2-MSHR (which keeps filling; the
            // primary returns to S_IDLE, whose fresh-primary accept is gated on !op2_mshr_active).
            // Must match op2_promote (includes OP2_PEND, else a late op2 orphans -> memq deadlock).
            S_INSTALL: state_n = (op2_occupied && !op2_mshr_active) ? S_OP2_PROMOTE : S_IDLE;
            S_OP2_PROMOTE: state_n = S_LOOKUP;
`else
            S_INSTALL: state_n = S_IDLE;
`endif

            // ---- flush walk ----
            S_FLUSH_READ: state_n = S_FLUSH_SCAN;
            S_FLUSH_SCAN: begin
                if (dirty_q[flush_set_q][flush_way_q]) begin
                    wb_line_n = dat_rdata[flush_way_q];
                    wb_addr_n = line_addr(tag_rdata[flush_way_q], flush_set_q);
                    state_n   = S_FLUSH_WB_REQ;
                end else if (flush_way_q == L1_WAY_BITS'(L1_WAYS-1)) begin
                    if (flush_set_q == L1_INDEX_BITS'(L1_SETS-1)) begin
                        state_n = S_FLUSH_DONE;
                    end else begin
                        flush_set_n = flush_set_q + 1'b1;
                        flush_way_n = '0;
                        state_n     = S_FLUSH_READ;
                    end
                end else begin
                    flush_way_n = flush_way_q + 1'b1;
                end
            end
            S_FLUSH_WB_REQ:  if (nmi_req_ready)  state_n = S_FLUSH_WB_WAIT;
            S_FLUSH_WB_WAIT: if (nmi_resp.valid) begin
                if (flush_way_q == L1_WAY_BITS'(L1_WAYS-1)) begin
                    if (flush_set_q == L1_INDEX_BITS'(L1_SETS-1)) begin
                        state_n = S_FLUSH_DONE;
                    end else begin
                        flush_set_n = flush_set_q + 1'b1;
                        flush_way_n = '0;
                        state_n     = S_FLUSH_READ;
                    end
                end else begin
                    flush_way_n = flush_way_q + 1'b1;
                    state_n     = S_FLUSH_SCAN;   // dat_rdata still holds the set
                end
            end
            S_FLUSH_DONE: if (!flush_req) state_n = S_IDLE;

            default: state_n = S_IDLE;
        endcase

        // The probe handshake resolved-flag clears once memsys drops probe_valid
        // (i.e. the I-refill it gated has been released). One probe per refill.
        if (!probe_valid) probe_done_n = 1'b0;
    end

    // op latch.
    logic op_valid_n2, op_write_n2;
    logic [MEMORY_ADDR_WIDTH-1:0] op_addr_n2;
    logic [WB_W-1:0] op_wdata_n2;
    logic [WBY-1:0]  op_wmask_n2;
`ifdef LSQ_MLP2
    logic [DMEM_ID_W-1:0] op_id_n2;
`endif
    always_comb begin
        op_valid_n2 = op_valid_q;
        op_write_n2 = op_write_q;
        op_addr_n2  = op_addr_q;
        op_wdata_n2 = op_wdata_q;
        op_wmask_n2 = op_wmask_q;
`ifdef LSQ_MLP2
        op_id_n2    = op_id_q;
`endif
        if ((state_q == S_IDLE) && req_fire) begin
            op_valid_n2 = 1'b1;
            op_write_n2 = req_write;
            op_addr_n2  = req_waddr;
            op_wdata_n2 = req_wdata;
            op_wmask_n2 = req_wmask;
`ifdef LSQ_MLP2
            op_id_n2    = req_id;
`endif
        end else if (state_q == S_LOOKUP && any_hit) begin
            op_valid_n2 = 1'b0;
        end else if (state_q == S_INSTALL) begin
`ifdef LSQ_MLP2
            // M10 (replace-in-place): if a parked op2 is waiting, PROMOTE it into the primary op
            // latch on the S_INSTALL->S_OP2_PROMOTE edge (op2 is dmem-only + cacheable). Its own
            // resp fires later with op2's id; the primary's resp already fired this cycle with the
            // primary's id. P3d-2: promote any occupied op2 EXCEPT an active op2-MSHR (self-completes);
            // must match op2_promote above (includes OP2_PEND, else a late op2 orphans -> memq deadlock).
            if (op2_occupied && !op2_mshr_active) begin
                op_valid_n2 = 1'b1;
                op_write_n2 = op2_write_q;
                op_addr_n2  = op2_addr_q;
                op_wdata_n2 = op2_wdata_q;
                op_wmask_n2 = op2_wmask_q;
                op_id_n2    = op2_id_q;
            end else begin
                op_valid_n2 = 1'b0;
            end
`else
            op_valid_n2 = 1'b0;
`endif
        end
    end

`ifdef LSQ_MLP2
    // ---- op2 phase next-state + field latches ----
    logic op2_write_n2;
    logic [MEMORY_ADDR_WIDTH-1:0] op2_addr_n2;
    logic [WB_W-1:0] op2_wdata_n2;
    logic [WBY-1:0]  op2_wmask_n2;
    logic [DMEM_ID_W-1:0] op2_id_n2;
    always_comb begin
        op2_phase_n2 = op2_phase_q;
        op2_write_n2 = op2_write_q; op2_addr_n2 = op2_addr_q;
        op2_wdata_n2 = op2_wdata_q; op2_wmask_n2 = op2_wmask_q; op2_id_n2 = op2_id_q;
        op2_fill_way_n    = op2_fill_way_q;
        op2_refill_line_n = op2_refill_line_q;
        if (op2_latch_fire) begin
            // Accept-and-park: latch ANY presented op (address-/kind-agnostic, so req_ready has no
            // comb dependence on req_write -> loop-free). A store / miss / same-set op parks; a
            // different-set load HIT serves early (resp2) then leaves.
            op2_phase_n2 = OP2_PEND;
            op2_write_n2 = req_write; op2_addr_n2 = req_waddr;
            op2_wdata_n2 = req_wdata; op2_wmask_n2 = req_wmask; op2_id_n2 = req_id;
        end else if (op2_promote) begin
            op2_phase_n2 = OP2_EMPTY;   // moved into the primary op latch (above)
        end else begin
            unique case (op2_phase_q)
                OP2_PEND: if (op2_read_present) op2_phase_n2 = OP2_READ;
                OP2_READ: begin
                    if (op2_serve_hit) begin
                        op2_phase_n2 = OP2_EMPTY;                     // hit -> served on resp2, done
                    end else if (op2_is_diffset_load_miss && op2_victim_clean) begin
                        // P3d-2 LAUNCH the concurrent MSHR: capture the (clean) victim way now, then
                        // run the WB-free fill leg. Different-set (op2_idx!=op_idx) so no contention.
                        op2_fill_way_n = op2_victim;
                        op2_phase_n2   = OP2_MISS_FILLREQ;
                    end else begin
                        op2_phase_n2 = OP2_PARK;    // dirty-victim / same-set / store -> P3c fallback
                    end
                end
                OP2_PARK: ;                                                     // held until promote
                OP2_MISS_FILLREQ:  if (op2_fill_req_fire) op2_phase_n2 = OP2_MISS_FILLWAIT;
                OP2_MISS_FILLWAIT: if (nmi_resp.valid && (nmi_resp.id == op2_exp_fill_id_q)) begin
                    op2_refill_line_n = nmi_resp.rdata;
                    op2_phase_n2      = OP2_MISS_INSTALL;
                end
                OP2_MISS_INSTALL:  if (op2_install_fire) op2_phase_n2 = OP2_EMPTY;
                default: ;
            endcase
        end
    end
`endif

    // valid / dirty next state.
    always_comb begin
        for (int s = 0; s < L1_SETS; s += 1) begin
            valid_n[s] = valid_q[s];
            dirty_n[s] = dirty_q[s];
        end
        if (store_hit) begin
            dirty_n[op_idx][hit_way] = 1'b1;
        end
        if (state_q == S_INSTALL) begin
            valid_n[op_idx][fill_way_q] = 1'b1;
            dirty_n[op_idx][fill_way_q] = op_write_q;   // store miss installs dirty
        end
        if (state_q == S_FLUSH_WB_WAIT && nmi_resp.valid) begin
            dirty_n[flush_set_q][flush_way_q] = 1'b0;
        end
        // C4a: a probe writeback leaves the line valid-clean.
        if (state_q == S_PROBE_WB_WAIT && nmi_resp.valid) begin
            dirty_n[pidx_q][pway_q] = 1'b0;
        end
`ifdef LSQ_MLP2
        // P3d-2: the op2-MSHR installs a valid-CLEAN line (a load fill; never dirty -> no new
        // coherence/SMC state). Different set from the primary install (op2_idx != op_idx).
        if (op2_install_fire) begin
            valid_n[op2_idx][op2_fill_way_q] = 1'b1;
            dirty_n[op2_idx][op2_fill_way_q] = 1'b0;
        end
`endif
    end

    always_ff @(posedge clk or negedge rst_l) begin
        if (!rst_l) begin
            state_q       <= S_IDLE;
            op_valid_q    <= 1'b0;
            op_write_q    <= 1'b0;
            op_addr_q     <= '0;
            op_wdata_q    <= '0;
            op_wmask_q    <= '0;
`ifdef LSQ_MLP2
            op_id_q       <= '0;
            op2_phase_q   <= OP2_EMPTY;
            op2_write_q   <= 1'b0;
            op2_addr_q    <= '0;
            op2_wdata_q   <= '0;
            op2_wmask_q   <= '0;
            op2_id_q      <= '0;
            op2_refill_line_q <= '0;
            op2_fill_way_q    <= '0;
            op2_exp_fill_id_q <= '0;
            exp_wb_id_q   <= '0;
            exp_fill_id_q <= '0;
`endif
            refill_line_q <= '0;
            fill_way_q    <= '0;
            wb_line_q     <= '0;
            wb_addr_q     <= '0;
            flush_set_q   <= '0;
            flush_way_q   <= '0;
            pidx_q        <= '0;
            ptag_q        <= '0;
            pway_q        <= '0;
            probe_done_q  <= 1'b0;
            gen_q         <= '0;
            for (int s = 0; s < L1_SETS; s += 1) begin
                valid_q[s] <= '0;
                dirty_q[s] <= '0;
                plru_q[s]  <= '0;
            end
        end else begin
            state_q       <= state_n;
            op_valid_q    <= op_valid_n2;
            op_write_q    <= op_write_n2;
            op_addr_q     <= op_addr_n2;
            op_wdata_q    <= op_wdata_n2;
            op_wmask_q    <= op_wmask_n2;
`ifdef LSQ_MLP2
            op_id_q       <= op_id_n2;
            op2_phase_q   <= op2_phase_n2;
            op2_write_q   <= op2_write_n2;
            op2_addr_q    <= op2_addr_n2;
            op2_wdata_q   <= op2_wdata_n2;
            op2_wmask_q   <= op2_wmask_n2;
            op2_id_q      <= op2_id_n2;
            op2_refill_line_q <= op2_refill_line_n;
            op2_fill_way_q    <= op2_fill_way_n;
`endif
            refill_line_q <= refill_line_n;
            fill_way_q    <= fill_way_n;
            wb_line_q     <= wb_line_n;
            wb_addr_q     <= wb_addr_n;
            flush_set_q   <= flush_set_n;
            flush_way_q   <= flush_way_n;
            pidx_q        <= pidx_n;
            ptag_q        <= ptag_n;
            pway_q        <= pway_n;
            probe_done_q  <= probe_done_n;
`ifdef LSQ_MLP2
            // P3d-2: the op2-MSHR fill also consumes a distinct gen (kept in an ifdef/else so the
            // OFF token stream is byte-identical to the pre-P3d condition -- no leaked parens).
            if (((state_q == S_WB_REQ || state_q == S_FILL_REQ ||
                 state_q == S_FLUSH_WB_REQ || state_q == S_PROBE_WB_REQ) && nmi_req_ready)
                || op2_fill_req_fire)
                gen_q <= gen_q + 2'd1;
`else
            if ((state_q == S_WB_REQ || state_q == S_FILL_REQ ||
                 state_q == S_FLUSH_WB_REQ || state_q == S_PROBE_WB_REQ) && nmi_req_ready)
                gen_q <= gen_q + 2'd1;
`endif
`ifdef LSQ_MLP2
            // P3d-0: latch the id each miss leg was issued with (same edge as the gen_q bump, so
            // it captures the gen_q value the outgoing nmi_req.id used, before the increment).
            if ((state_q == S_WB_REQ)   && nmi_req_ready) exp_wb_id_q   <= {NMI_SRC_DWB,   gen_q};
            if ((state_q == S_FILL_REQ) && nmi_req_ready) exp_fill_id_q <= {NMI_SRC_DFILL, gen_q};
            // P3d-2: the op2-MSHR fill's expected response id (its own gen, distinct from primary).
            if (op2_fill_req_fire) op2_exp_fill_id_q <= {NMI_SRC_DFILL, gen_q};
`endif
            for (int s = 0; s < L1_SETS; s += 1) begin
                valid_q[s] <= valid_n[s];
                dirty_q[s] <= dirty_n[s];
            end
            if (plru_upd_en) plru_q[op_idx] <= plru_next;
`ifdef LSQ_MLP2
            // P3d-2: the op2-MSHR install touches its (different-set) way. Distinct index from
            // plru_q[op_idx] above, so both can update the same cycle without conflict.
            if (op2_plru_upd_en) plru_q[op2_idx] <= op2_plru_next;
`endif
        end
    end

`ifdef LSQ_MLP2
    // ---- P3c-3 assertions (sim-only) ----
    always_ff @(posedge clk) begin
        if (rst_l) begin
            // B3 no-skid proof: resp (primary, in {S_LOOKUP,S_INSTALL}) and resp2 (op2 hit, in a
            // WAIT state via the !nmi_resp gate) are in disjoint state sets -> never both valid.
            assert (!(resp_valid && resp2_valid))
                else $fatal(1, "l1_dcache: resp/resp2 same-cycle collision (B3 invariant broken)");
            // B7: op2's read is presented only in cycles with NO array write -> disjoint.
            assert (!(dat_ren && dat_wen))
                else $fatal(1, "l1_dcache: dat_ren && dat_wen same cycle (B7)");
            assert (!(tag_ren && tag_wen))
                else $fatal(1, "l1_dcache: tag_ren && tag_wen same cycle (B7)");
            // M11: the same-set interlock, asserted on the DECOMPOSED precondition (op2_serve_hit
            // already ANDs op2_idx!=op_idx, so asserting it there is tautological). A same-set op2
            // that would otherwise hit must NEVER produce resp2 -- it must park.
            assert (!((op2_phase_q == OP2_READ) && !op2_write_q && op2_any_hit &&
                      (op2_idx == op_idx) && resp2_valid))
                else $fatal(1, "l1_dcache: same-set op2 served early (interlock bypassed)");
            // P3d-2: the op2-MSHR always operates on a DIFFERENT set than the primary (disjoint =>
            // no victim/PLRU/tag/valid collision, C1/C2 closed) -- a new primary can't start while it
            // is active (req_ready S_IDLE gated on !op2_mshr_active), so op_idx is stable and distinct.
            assert (!(op2_mshr_active && (op2_idx == op_idx)))
                else $fatal(1, "l1_dcache: op2-MSHR same set as primary (C1/C2 invariant broken)");
            // P3d-2 (C4): two concurrent DFILL waits must carry DISTINCT ids (else a response would
            // install into the wrong context = silent corruption).
            assert (!((state_q == S_FILL_WAIT) && (op2_phase_q == OP2_MISS_FILLWAIT) &&
                      (exp_fill_id_q == op2_exp_fill_id_q)))
                else $fatal(1, "l1_dcache: primary + op2 fill id alias (C4)");
            // P3d-2 (install serialize): at most one writer of the array per cycle.
            assert (!(op2_install_fire && primary_install))
                else $fatal(1, "l1_dcache: op2 install races primary install (write-port)");
        end
    end
`endif

`ifdef LSQ_MLP_STAT
    // P3d-2 activation: cycles the L1D holds TWO miss contexts at once (a primary miss S_{WB,FILL}_*
    // AND the concurrent op2-MSHR). == 0 at P3c-3, > 0 at P3d-2 on mlp_stream => the concurrent-MSHR
    // structure engages. (IPC is still flat at P3d-2 -- the single-outstanding backend serializes the
    // two fills; P3d-3's concurrent adapter is what overlaps them.)
    wire l1d_primary_miss = (state_q == S_WB_REQ) || (state_q == S_WB_WAIT) ||
                            (state_q == S_FILL_REQ) || (state_q == S_FILL_WAIT) || (state_q == S_INSTALL);
    longint unsigned l1d_two_miss_cyc, l1d_op2_mshr_launches;
    always_ff @(posedge clk or negedge rst_l) begin
        if (!rst_l) begin l1d_two_miss_cyc <= '0; l1d_op2_mshr_launches <= '0; end
        else begin
            if (l1d_primary_miss && op2_mshr_active) l1d_two_miss_cyc <= l1d_two_miss_cyc + 1;
            if ((op2_phase_q == OP2_READ) && op2_is_diffset_load_miss && op2_victim_clean)
                l1d_op2_mshr_launches <= l1d_op2_mshr_launches + 1;
        end
    end
    final $display("L1D-P3D-STAT: two_miss_cyc=%0d op2_mshr_launches=%0d",
                   l1d_two_miss_cyc, l1d_op2_mshr_launches);
`endif

`ifdef AGENT_DEBUG
    always_ff @(posedge clk) begin
        if (rst_l) begin
            if ((state_q == S_IDLE) && req_fire)
                $display("[L1D] accept %s waddr=%08h idx=%0d tag=%05h woff=%0d wmask=%0h wdata=%016h",
                    req_write ? "ST" : "LD", req_waddr, l1_index(req_waddr),
                    l1_tag(req_waddr), l1_word_off(req_waddr), req_wmask, req_wdata);
            if (serve_hit)
                $display("[L1D] HIT way=%0d %s resp=%016h", hit_way,
                    op_write_q ? "ST" : "LD", resp_data);
            if (serve_miss)
                $display("[L1D] MISS idx=%0d tag=%05h victim=%0d vld=%b drt=%b",
                    op_idx, op_tag, victim, valid_q[op_idx][victim], dirty_q[op_idx][victim]);
            if ((state_q == S_WB_REQ) && nmi_req_ready)
                $display("[L1D] WB   waddr=%08h line=%0128h", wb_addr_q, wb_line_q);
            if ((state_q == S_FILL_WAIT) && nmi_resp.valid)
                $display("[L1D] FILL line=%0128h", nmi_resp.rdata);
            if (state_q == S_INSTALL)
                $display("[L1D] INST way=%0d drt=%b resp=%016h line=%0128h",
                    fill_way_q, op_write_q, resp_data, install_line);
            if (state_q == S_PROBE_LOOK)
                $display("[L1D] PROBE waddr=%08h idx=%0d tag=%05h hit=%b dirty=%b",
                    {ptag_q, pidx_q, {LINE_WORD_BITS{1'b0}}}, pidx_q, ptag_q,
                    probe_any_hit, probe_dirty);
            if ((state_q == S_PROBE_WB_REQ) && nmi_req_ready)
                $display("[L1D] PROBE-WB waddr=%08h line=%0128h", wb_addr_q, wb_line_q);
        end
    end
`endif

endmodule : l1_dcache

`default_nettype wire
