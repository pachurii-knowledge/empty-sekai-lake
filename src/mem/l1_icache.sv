/**
 * l1_icache.sv  (phase C1)
 *
 * Blocking, read-only L1 instruction cache: 16 KiB, 4-way, 64 B lines, PIPT
 * (the ITLB/FetchPMP translate before the request reaches here, so the
 * incoming word address is already physical). Sync-read tag+data arrays give a
 * 2-cycle hit; hits pipeline at one 16 B fetch group per cycle. A miss issues a
 * single NMI RD_LINE, installs into the PLRU victim way, and replays. fence.i
 * (and the halt/flush path) flash-invalidate every line.
 *
 * The core's instruction port consumes exactly one 16 B group (4 words on
 * RV32, low 2 words on RV64) which is always contained in one 64 B line, so a
 * group never spans a line (C1 task 0, resolved from the frontend lane use in
 * riscv_core_ooo.sv decode_fetch_instr).
 */

`include "niigo_mem.vh"

`default_nettype none

module l1_icache
    import RISCV_ISA::XLEN, RISCV_ISA::XLEN_BYTES;
    import RISCV_UArch::MEMORY_READ_WIDTH, RISCV_UArch::MEMORY_ADDR_WIDTH;
    import NIIGO_Mem::*;
(
    input wire logic clk,
    input wire logic rst_l,

    // ---- Core instruction-fetch port (handshaked; combinational accept) ----
    input wire logic                                    ifetch_req_valid,
    output logic                                    ifetch_req_ready,
    input wire logic [MEMORY_ADDR_WIDTH-1:0]            ifetch_req_addr,
    output logic                                    ifetch_resp_valid,
    output logic [MEMORY_READ_WIDTH-1:0][XLEN-1:0]  ifetch_resp_data,
    output logic                                    ifetch_resp_excpt,

    // ---- fence.i / flush: flash-invalidate all lines (single-cycle pulse) ----
    input wire logic                                    inval_all,

    // ---- C4b coherence snoop: a committed D-store to snoop_waddr invalidates
    //      any L1I copy of that line (single-cycle pulse; word address). The
    //      snoop read interleaves onto the single tag port (occasionally
    //      deferring a fetch accept by a cycle -- the fetch path is handshaked
    //      and tolerates variable latency). ----
    input wire logic                                    snoop_valid,
    input wire logic [MEMORY_ADDR_WIDTH-1:0]            snoop_waddr,

    // ---- NMI master ----
    output nmi_req_t                                nmi_req,
    input wire logic                                    nmi_req_ready,
    input wire nmi_resp_t                               nmi_resp,

    // ---- C3 observability (pulses; unused until C3 wires them) ----
    output logic                                    ev_access,
    output logic                                    ev_miss
);

    // ---------------- arrays + flop metadata ----------------
    logic                           tag_ren, tag_wen;
    logic [L1_INDEX_BITS-1:0]       tag_ridx, tag_widx;
    logic [L1_WAY_BITS-1:0]         tag_wway;
    logic [L1_TAG_BITS-1:0]         tag_wtag;
    logic [L1_WAYS-1:0][L1_TAG_BITS-1:0] tag_rdata;

    logic                           dat_ren, dat_wen;
    logic [L1_INDEX_BITS-1:0]       dat_ridx, dat_widx;
    logic [L1_WAY_BITS-1:0]         dat_wway;
    logic [LINE_BITS-1:0]           dat_wdata;
    logic [LINE_BITS/8-1:0]         dat_wmask;
    logic [L1_WAYS-1:0][LINE_BITS-1:0] dat_rdata;

    l1_tag_array #(.SETS(L1_SETS), .WAYS(L1_WAYS), .TAG_BITS(L1_TAG_BITS)) Tags (
        .clk, .ren(tag_ren), .ridx(tag_ridx), .rtag(tag_rdata),
        .wen(tag_wen), .widx(tag_widx), .wway(tag_wway), .wtag(tag_wtag)
    );
    l1_data_array #(.SETS(L1_SETS), .WAYS(L1_WAYS), .LINE_BITS(LINE_BITS)) Data (
        .clk, .ren(dat_ren), .ridx(dat_ridx), .rdata(dat_rdata),
        .wen(dat_wen), .widx(dat_widx), .wway(dat_wway),
        .wdata(dat_wdata), .wmask(dat_wmask)
    );

    logic [L1_WAYS-1:0] valid_q [L1_SETS];
    logic [L1_WAYS-1:0] valid_n [L1_SETS];
    logic [2:0]         plru_q  [L1_SETS];
    logic [1:0]         gen_q;

    // ---------------- request pipeline (1 in flight + replay) ----------------
    logic                          p_valid_q, p_valid_n;
    logic [MEMORY_ADDR_WIDTH-1:0]  p_addr_q,  p_addr_n;

    // VIPT seam (M2; plans/multicore-ccd.md §V): the set index uses page-offset bits, which are
    // translation-invariant (VA[idx] == PA[idx], alias-free per L1_VIPT_ALIAS_FREE) -> a VA-sourced
    // index in all but name; the tag (p_tag) is physical. The C4b snoop port indexes with the PA over
    // the same bits, hitting the SAME set with no synonym search. True VA-early indexing (TLB-overlap)
    // is a descoped FPGA-perf lever; per-line coherence state + the CMI interface are the M3 step.
    logic [L1_INDEX_BITS-1:0]  p_idx;   // VIPT index  (page-offset bits -> translation-invariant)
    logic [L1_TAG_BITS-1:0]    p_tag;   // physical tag (PA)
    logic [LINE_WORD_BITS-1:0] p_loff;
    assign p_idx  = l1_index(p_addr_q);
    assign p_tag  = l1_tag(p_addr_q);
    assign p_loff = l1_word_off(p_addr_q);

    initial assert (L1_VIPT_ALIAS_FREE)
        else $fatal(1, "l1_icache: VIPT alias-free invariant violated (way_size %0d B > page %0d B)",
                    L1_WAY_BYTES, PAGE_BYTES);

    typedef enum logic [2:0] {
        S_SERVE, S_MISS_REQ, S_MISS_WAIT, S_INSTALL, S_REPLAY
    } state_e;
    state_e state_q, state_n;

    logic [LINE_BITS-1:0] refill_line_q, refill_line_n;
    logic                 refill_err_q,  refill_err_n;
    logic                 miss_inval_q,  miss_inval_n;  // inval seen mid-refill

    // ---------------- C4b snoop pipeline (single-port interleave) -------------
    // The coherence snoop shares the ONE 1RW tag port with fetch, priority
    // install-write > snoop-read > fetch/replay-read. A snoop read launched in
    // cycle X changes tag_rdata only at X+1, and servicing a snoop deasserts
    // ifetch_req_ready at X so no fetch consumes tag_rdata at X+1 (the hit served
    // at X already used the X-1 read) -- so a steal never corrupts an in-flight
    // fetch and never reorders/drops a response (the core pairs responses to its
    // in-order fmeta FIFO). A depth-1 skid holds a snoop blocked by an install
    // write; it is empty entering S_INSTALL because the I-miss that leads there
    // could only be accepted on a snoop-gap cycle (req_fire needs !snp_service),
    // which zeroes the skid, and it stays zero through the port-free miss.
    // Stage 1: present the serviced snoop index to the shared read port.
    // Stage 2 (next cycle): compare the registered tag and clear the valid bit.
    logic                      snp_v_q,   snp_v_n;
    logic [L1_INDEX_BITS-1:0]  snp_idx_q, snp_idx_n;
    logic [L1_TAG_BITS-1:0]    snp_tag_q, snp_tag_n;
    // Depth-1 skid: a snoop that arrived while the port was doing an install write.
    logic                          snp_pend_v_q,    snp_pend_v_n;
    logic [MEMORY_ADDR_WIDTH-1:0]  snp_pend_addr_q, snp_pend_addr_n;

    // A snoop wants the port this cycle (fresh pulse or skidded); pending wins.
    logic                          snp_want_v;
    logic [MEMORY_ADDR_WIDTH-1:0]  snp_svc_a;
    assign snp_want_v = snp_pend_v_q || snoop_valid;
    assign snp_svc_a  = snp_pend_v_q ? snp_pend_addr_q : snoop_waddr;

    // Stage 2 hit detection against the shared-port registered tags, plus the
    // combinational refill-race drop.
    logic [L1_WAYS-1:0] snp_hit_oh;
    logic               snp_hit_refill;  // snoop targets the line being refilled
    always_comb begin
        for (int w = 0; w < L1_WAYS; w += 1)
            snp_hit_oh[w] = snp_v_q && valid_q[snp_idx_q][w] &&
                            (tag_rdata[w] == snp_tag_q);
        // A store to the line currently in refill makes the pending install
        // stale (it may predate the store) -> drop it, just like inval_all. This
        // is COMBINATIONAL over two terms so a snoop landing exactly at S_INSTALL
        // still drops the install: (1) an earlier snoop now registered in stage 2
        // while state != S_SERVE, and (2) a fresh/pending snoop arriving at
        // S_INSTALL (before it can reach stage 2).
        snp_hit_refill =
            ( snp_v_q    && (state_q != S_SERVE)   &&
              (snp_idx_q == p_idx)                 && (snp_tag_q == p_tag) ) ||
            ( snp_want_v && (state_q == S_INSTALL) &&
              (l1_index(snp_svc_a) == p_idx)       && (l1_tag(snp_svc_a) == p_tag) );
    end

    // Port arbitration: install-write > snoop-read > fetch/replay-read.
    logic install_go;   // the install write actually happens this cycle
    logic snp_service;  // the snoop reads the shared tag port this cycle
    assign install_go  = (state_q == S_INSTALL) && !miss_inval_q && !snp_hit_refill;
    assign snp_service = snp_want_v && !install_go;

    // Stage-1 capture: register the serviced snoop for next-cycle compare.
    assign snp_v_n   = snp_service;
    assign snp_idx_n = l1_index(snp_svc_a);
    assign snp_tag_n = l1_tag(snp_svc_a);

    // Depth-1 skid update: hold a snoop blocked by an install write, or a fresh
    // snoop arriving the same cycle a pending one is serviced.
    always_comb begin
        snp_pend_v_n    = snp_pend_v_q;
        snp_pend_addr_n = snp_pend_addr_q;
        if (snp_service) begin
            snp_pend_v_n = 1'b0;
            if (snp_pend_v_q && snoop_valid) begin
                snp_pend_v_n    = 1'b1;
                snp_pend_addr_n = snoop_waddr;
            end
        end else if (snoop_valid) begin
            snp_pend_v_n    = 1'b1;
            snp_pend_addr_n = snoop_waddr;
        end
    end

    // ---------------- hit detection (stage 2) ----------------
    logic [L1_WAYS-1:0]     hit_way_oh;
    logic                   any_hit;
    logic [L1_WAY_BITS-1:0] hit_way;
    always_comb begin
        for (int w = 0; w < L1_WAYS; w += 1) begin
            hit_way_oh[w] = valid_q[p_idx][w] && (tag_rdata[w] == p_tag);
        end
        any_hit = |hit_way_oh;
        hit_way = '0;
        for (int w = 0; w < L1_WAYS; w += 1) begin
            if (hit_way_oh[w]) hit_way = L1_WAY_BITS'(w);
        end
    end

    // Selected line and 16 B group extraction (wrap within the line; on RV64
    // the upper two response lanes are unused by the frontend, so wrap is
    // harmless).
    logic [LINE_BITS-1:0] sel_line;
    assign sel_line = dat_rdata[hit_way];
    always_comb begin
        for (int j = 0; j < MEMORY_READ_WIDTH; j += 1) begin
            logic [LINE_WORD_BITS-1:0] wsel;
            wsel = p_loff + LINE_WORD_BITS'(j);
            ifetch_resp_data[j] = sel_line[wsel*XLEN +: XLEN];
        end
    end

    // ---------------- PLRU victim + update ----------------
    logic [L1_WAY_BITS-1:0] victim;
    logic                   plru_upd_en;
    logic [L1_WAY_BITS-1:0] plru_acc_way;
    logic [2:0]             plru_next;
    l1_plru #(.WAYS(L1_WAYS)) Plru (
        .state(plru_q[p_idx]),
        .valid(valid_q[p_idx]),
        .victim(victim),
        .update_en(plru_upd_en),
        .access_way(plru_acc_way),
        .next_state(plru_next)
    );

    // ---------------- control ----------------
    logic serve_hit;     // a buffered request hits this cycle
    logic serve_miss;    // a buffered request misses this cycle
    logic req_fire;
    assign serve_hit  = (state_q == S_SERVE) && p_valid_q &&  any_hit;
    assign serve_miss = (state_q == S_SERVE) && p_valid_q && !any_hit;

    // A snoop stealing the shared tag read port this cycle blocks a new fetch
    // accept (the fetch requester simply waits -- variable latency tolerated).
    assign ifetch_req_ready = (state_q == S_SERVE) && (!p_valid_q || any_hit) && !snp_service;
    assign req_fire = ifetch_req_valid && ifetch_req_ready;

    assign ifetch_resp_valid = serve_hit;
    // Flat sim memory never raises a fetch access error, so NMI err is always
    // 0 (refill_err_q is captured for faithfulness but never set in practice).
    logic unused_refill_err;
    assign unused_refill_err = refill_err_q;
    assign ifetch_resp_excpt = 1'b0;

    assign ev_access = serve_hit || serve_miss;
    assign ev_miss   = serve_miss;

    // NMI request (RD_LINE on a miss).
    always_comb begin
        nmi_req       = '0;
        nmi_req.valid = (state_q == S_MISS_REQ);
        nmi_req.op    = NMI_RD_LINE;
        nmi_req.waddr = l1_line_base(p_addr_q);
        nmi_req.id    = {NMI_SRC_IFILL, gen_q};
    end

    // Array read presentation. A snoop service steals the tag read port (tag
    // only -- the data array stays idle; the snoop just needs the tags). It has
    // priority over a fetch accept (gated out of ifetch_req_ready) and over a
    // replay read (which is deferred by staying in S_REPLAY, below).
    always_comb begin
        tag_ren  = 1'b0;
        tag_ridx = p_idx;
        dat_ren  = 1'b0;
        dat_ridx = p_idx;
        if (snp_service) begin
            tag_ren  = 1'b1; tag_ridx = l1_index(snp_svc_a);
        end else if ((state_q == S_SERVE) && req_fire) begin
            tag_ren  = 1'b1; tag_ridx = l1_index(ifetch_req_addr);
            dat_ren  = 1'b1; dat_ridx = l1_index(ifetch_req_addr);
        end else if (state_q == S_REPLAY) begin
            tag_ren  = 1'b1; tag_ridx = p_idx;
            dat_ren  = 1'b1; dat_ridx = p_idx;
        end
    end

    // Array writes (install).
    always_comb begin
        tag_wen   = 1'b0;
        tag_widx  = p_idx;
        tag_wway  = victim;
        tag_wtag  = p_tag;
        dat_wen   = 1'b0;
        dat_widx  = p_idx;
        dat_wway  = victim;
        dat_wdata = refill_line_q;
        dat_wmask = '1;
        if (install_go) begin
            tag_wen = 1'b1;
            dat_wen = 1'b1;
        end
    end

    // PLRU update target.
    always_comb begin
        plru_upd_en  = 1'b0;
        plru_acc_way = hit_way;
        if (serve_hit) begin
            plru_upd_en  = 1'b1;
            plru_acc_way = hit_way;
        end else if (install_go) begin
            plru_upd_en  = 1'b1;
            plru_acc_way = victim;
        end
    end

    // Next-state / pipeline.
    always_comb begin
        state_n       = state_q;
        p_valid_n     = p_valid_q;
        p_addr_n      = p_addr_q;
        refill_line_n = refill_line_q;
        refill_err_n  = refill_err_q;
        miss_inval_n  = miss_inval_q;

        unique case (state_q)
            S_SERVE: begin
                if (serve_hit) begin
                    // Consumed this cycle; take a new request if one fires.
                    if (req_fire) begin p_valid_n = 1'b1; p_addr_n = ifetch_req_addr; end
                    else          begin p_valid_n = 1'b0; end
                end else if (serve_miss) begin
                    // Freeze the missing request; start the refill.
                    miss_inval_n = 1'b0;
                    state_n      = S_MISS_REQ;
                end else begin
                    // Empty buffer: latch a new request if one fires.
                    if (req_fire) begin p_valid_n = 1'b1; p_addr_n = ifetch_req_addr; end
                end
            end

            S_MISS_REQ: begin
                if (nmi_req_ready) state_n = S_MISS_WAIT;
            end

            S_MISS_WAIT: begin
                if (nmi_resp.valid) begin
                    refill_line_n = nmi_resp.rdata;
                    refill_err_n  = nmi_resp.err;
                    state_n       = S_INSTALL;
                end
            end

            S_INSTALL: begin
                // Install happens through the array/plru/valid writes above
                // (suppressed when an invalidate raced the refill); then replay.
                miss_inval_n = 1'b0;
                state_n      = S_REPLAY;
            end

            S_REPLAY: begin
                // Re-present the (now installed, or re-missing) index -- unless a
                // snoop stole the tag read port this cycle (the replay read did
                // not happen), in which case stay one more cycle to redo it.
                state_n = snp_service ? S_REPLAY : S_SERVE;
            end

            default: state_n = S_SERVE;
        endcase

        // A fence.i / flush during a refill must drop the install (the line may
        // predate the store the fence.i orders); the request then re-misses and
        // re-reads memory. A C4b snoop to the in-refill line does the same.
        if (inval_all && (state_q != S_SERVE)) miss_inval_n = 1'b1;
        if (snp_hit_refill)                    miss_inval_n = 1'b1;
    end

    // valid-bit next state (flash invalidate + install set).
    always_comb begin
        for (int s = 0; s < L1_SETS; s += 1) valid_n[s] = valid_q[s];
        // Flash invalidate: fence.i in SERVE, or the dropped-install case (a
        // fence.i or a snoop to the in-refill line raced the refill).
        if ((inval_all) ||
            ((state_q == S_INSTALL) && (miss_inval_q || snp_hit_refill))) begin
            for (int s = 0; s < L1_SETS; s += 1) valid_n[s] = '0;
        end
        // C4b snoop: clear any way whose tag matches a committed-store line.
        for (int w = 0; w < L1_WAYS; w += 1)
            if (snp_hit_oh[w]) valid_n[snp_idx_q][w] = 1'b0;
        // Install sets the victim way valid (only on a clean, non-dropped install).
        if (install_go) begin
            valid_n[p_idx][victim] = 1'b1;
        end
    end

    always_ff @(posedge clk or negedge rst_l) begin
        if (!rst_l) begin
            state_q       <= S_SERVE;
            p_valid_q     <= 1'b0;
            p_addr_q      <= '0;
            refill_line_q <= '0;
            refill_err_q  <= 1'b0;
            miss_inval_q  <= 1'b0;
            snp_v_q       <= 1'b0;
            snp_idx_q     <= '0;
            snp_tag_q     <= '0;
            snp_pend_v_q    <= 1'b0;
            snp_pend_addr_q <= '0;
            gen_q         <= '0;
            for (int s = 0; s < L1_SETS; s += 1) begin
                valid_q[s] <= '0;
                plru_q[s]  <= '0;
            end
        end else begin
            state_q       <= state_n;
            p_valid_q     <= p_valid_n;
            p_addr_q      <= p_addr_n;
            refill_line_q <= refill_line_n;
            refill_err_q  <= refill_err_n;
            miss_inval_q  <= miss_inval_n;
            snp_v_q       <= snp_v_n;
            snp_idx_q     <= snp_idx_n;
            snp_tag_q     <= snp_tag_n;
            snp_pend_v_q    <= snp_pend_v_n;
            snp_pend_addr_q <= snp_pend_addr_n;
            if (state_q == S_MISS_REQ && nmi_req_ready) gen_q <= gen_q + 2'd1;
            for (int s = 0; s < L1_SETS; s += 1) valid_q[s] <= valid_n[s];
            if (plru_upd_en) plru_q[p_idx] <= plru_next;
            // Depth-1 skid invariant: the skid is empty entering S_INSTALL (the
            // I-miss that leads there could only be accepted on a snoop-gap cycle,
            // which zeroes the skid, and it stays zero through the port-free
            // miss). If this fires, a second concurrent snoop could be dropped ->
            // the skid must be deepened.
            // synopsys translate_off
            assert (!(snp_pend_v_q && (state_q == S_INSTALL)))
                else $error("l1_icache: snoop skid non-empty at S_INSTALL (depth-1 invariant violated)");
            // synopsys translate_on
        end
    end

endmodule : l1_icache

`default_nettype wire
