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
    input  logic clk,
    input  logic rst_l,

    // ---- Core instruction-fetch port (handshaked; combinational accept) ----
    input  logic                                    ifetch_req_valid,
    output logic                                    ifetch_req_ready,
    input  logic [MEMORY_ADDR_WIDTH-1:0]            ifetch_req_addr,
    output logic                                    ifetch_resp_valid,
    output logic [MEMORY_READ_WIDTH-1:0][XLEN-1:0]  ifetch_resp_data,
    output logic                                    ifetch_resp_excpt,

    // ---- fence.i / flush: flash-invalidate all lines (single-cycle pulse) ----
    input  logic                                    inval_all,

    // ---- C4b coherence snoop: a committed D-store to snoop_waddr invalidates
    //      any L1I copy of that line (single-cycle pulse; word address). Uses
    //      the tag array's 2nd read port, so it never stalls the fetch port. ----
    input  logic                                    snoop_valid,
    input  logic [MEMORY_ADDR_WIDTH-1:0]            snoop_waddr,

    // ---- NMI master ----
    output nmi_req_t                                nmi_req,
    input  logic                                    nmi_req_ready,
    input  nmi_resp_t                               nmi_resp,

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
    // 2nd tag read port (C4b snoop).
    logic                           tag_ren2;
    logic [L1_INDEX_BITS-1:0]       tag_ridx2;
    logic [L1_WAYS-1:0][L1_TAG_BITS-1:0] tag_rdata2;

    logic                           dat_ren, dat_wen;
    logic [L1_INDEX_BITS-1:0]       dat_ridx, dat_widx;
    logic [L1_WAY_BITS-1:0]         dat_wway;
    logic [LINE_BITS-1:0]           dat_wdata;
    logic [LINE_BITS/8-1:0]         dat_wmask;
    logic [L1_WAYS-1:0][LINE_BITS-1:0] dat_rdata;

    l1_tag_array #(.SETS(L1_SETS), .WAYS(L1_WAYS), .TAG_BITS(L1_TAG_BITS)) Tags (
        .clk, .ren(tag_ren), .ridx(tag_ridx), .rtag(tag_rdata),
        .ren2(tag_ren2), .ridx2(tag_ridx2), .rtag2(tag_rdata2),
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

    logic [L1_INDEX_BITS-1:0]  p_idx;
    logic [L1_TAG_BITS-1:0]    p_tag;
    logic [LINE_WORD_BITS-1:0] p_loff;
    assign p_idx  = l1_index(p_addr_q);
    assign p_tag  = l1_tag(p_addr_q);
    assign p_loff = l1_word_off(p_addr_q);

    typedef enum logic [2:0] {
        S_SERVE, S_MISS_REQ, S_MISS_WAIT, S_INSTALL, S_REPLAY
    } state_e;
    state_e state_q, state_n;

    logic [LINE_BITS-1:0] refill_line_q, refill_line_n;
    logic                 refill_err_q,  refill_err_n;
    logic                 miss_inval_q,  miss_inval_n;  // inval seen mid-refill

    // ---------------- C4b snoop pipeline (2 stages, always-running) ----------
    // Stage 1 (this cycle): present the snoop index to the 2nd tag read port.
    // Stage 2 (next cycle): compare the registered tag and clear the matching
    // valid bit. Decoupled from the fetch read port, so it never bubbles fetch.
    logic                      snp_v_q,   snp_v_n;
    logic [L1_INDEX_BITS-1:0]  snp_idx_q, snp_idx_n;
    logic [L1_TAG_BITS-1:0]    snp_tag_q, snp_tag_n;
    assign tag_ren2  = snoop_valid;
    assign tag_ridx2 = l1_index(snoop_waddr);
    assign snp_v_n   = snoop_valid;
    assign snp_idx_n = l1_index(snoop_waddr);
    assign snp_tag_n = l1_tag(snoop_waddr);

    // Stage 2 hit detection against the registered (port-2) tags.
    logic [L1_WAYS-1:0] snp_hit_oh;
    logic               snp_hit_refill;  // snoop targets the line being refilled
    always_comb begin
        for (int w = 0; w < L1_WAYS; w += 1)
            snp_hit_oh[w] = snp_v_q && valid_q[snp_idx_q][w] &&
                            (tag_rdata2[w] == snp_tag_q);
        // A store to the line currently in refill makes the pending install
        // stale (it may predate the store) -> drop it, just like inval_all.
        snp_hit_refill = snp_v_q && (state_q != S_SERVE) &&
                         (snp_idx_q == p_idx) && (snp_tag_q == p_tag);
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

    assign ifetch_req_ready = (state_q == S_SERVE) && (!p_valid_q || any_hit);
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

    // Array read presentation.
    always_comb begin
        tag_ren  = 1'b0;
        tag_ridx = p_idx;
        dat_ren  = 1'b0;
        dat_ridx = p_idx;
        if ((state_q == S_SERVE) && req_fire) begin
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
        if ((state_q == S_INSTALL) && !miss_inval_q) begin
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
        end else if ((state_q == S_INSTALL) && !miss_inval_q) begin
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
                // Re-present the (now installed, or re-missing) index.
                state_n = S_SERVE;
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
        // Flash invalidate: fence.i in SERVE, or the dropped-install case.
        if ((inval_all) ||
            ((state_q == S_INSTALL) && miss_inval_q)) begin
            for (int s = 0; s < L1_SETS; s += 1) valid_n[s] = '0;
        end
        // C4b snoop: clear any way whose tag matches a committed-store line.
        for (int w = 0; w < L1_WAYS; w += 1)
            if (snp_hit_oh[w]) valid_n[snp_idx_q][w] = 1'b0;
        // Install sets the victim way valid (overrides a same-cycle clear only
        // when this is a clean install).
        if ((state_q == S_INSTALL) && !miss_inval_q) begin
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
            if (state_q == S_MISS_REQ && nmi_req_ready) gen_q <= gen_q + 2'd1;
            for (int s = 0; s < L1_SETS; s += 1) valid_q[s] <= valid_n[s];
            if (plru_upd_en) plru_q[p_idx] <= plru_next;
        end
    end

endmodule : l1_icache

`default_nettype wire
