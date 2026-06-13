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
    input  logic clk,
    input  logic rst_l,

    // ---- Unified D request (from the front arbiter) ----
    input  logic                          req_valid,
    output logic                          req_ready,
    input  logic                          req_write,
    input  logic [MEMORY_ADDR_WIDTH-1:0]  req_waddr,
    input  logic [XLEN-1:0]               req_wdata,
    input  logic [XLEN_BYTES-1:0]         req_wmask,
    output logic                          resp_valid,    // load data this cycle
    output logic [XLEN-1:0]               resp_data,
    output logic [MEMORY_ADDR_WIDTH-1:0]  resp_addr,     // echo (word addr)
    output logic                          wr_accept,     // store accepted (PTW write ack)

    // ---- Flush: write back all dirty lines (fence.i + halt) ----
    input  logic                          flush_req,
    output logic                          flush_done,

    // ---- C4a coherence probe (clean-before-refill) ----
    // niigo_memsys asserts probe_valid (level) with the line word address an
    // L1I refill is about to fetch. If this cache holds that line dirty, it is
    // written back here first; probe_clean then rises and the memsys releases
    // the held I-refill so it reads the now-current line from memory. A probe
    // miss / clean hit raises probe_clean immediately. The line stays valid.
    input  logic                          probe_valid,
    input  logic [MEMORY_ADDR_WIDTH-1:0]  probe_waddr,
    output logic                          probe_clean,

    // ---- NMI master ----
    output nmi_req_t                      nmi_req,
    input  logic                          nmi_req_ready,
    input  nmi_resp_t                     nmi_resp,

    // ---- C3 observability (pulses) ----
    output logic                          ev_access,
    output logic                          ev_miss,
    output logic                          ev_wb
);

    localparam int LB    = LINE_BITS;
    localparam int LBY   = LINE_BITS/8;
    localparam int WB_W  = XLEN;
    localparam int WBY   = XLEN_BYTES;

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

    l1_tag_array #(.SETS(L1_SETS), .WAYS(L1_WAYS), .TAG_BITS(L1_TAG_BITS)) Tags (
        .clk, .ren(tag_ren), .ridx(tag_ridx), .rtag(tag_rdata),
        // 2nd read port unused here: the L1D probe (C4a) reuses the main read
        // port in S_PROBE_LOOK, so the snoop port is tied off (optimized away).
        .ren2(1'b0), .ridx2('0), .rtag2(),
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

    logic [L1_INDEX_BITS-1:0]  op_idx;
    logic [L1_TAG_BITS-1:0]    op_tag;
    logic [LINE_WORD_BITS-1:0] op_woff;
    assign op_idx  = l1_index(op_addr_q);
    assign op_tag  = l1_tag(op_addr_q);
    assign op_woff = l1_word_off(op_addr_q);

    typedef enum logic [3:0] {
        S_IDLE, S_LOOKUP, S_WB_REQ, S_WB_WAIT, S_FILL_REQ, S_FILL_WAIT, S_INSTALL,
        S_FLUSH_READ, S_FLUSH_SCAN, S_FLUSH_WB_REQ, S_FLUSH_WB_WAIT, S_FLUSH_DONE,
        S_PROBE_LOOK, S_PROBE_WB_REQ, S_PROBE_WB_WAIT
    } state_e;
    state_e state_q, state_n;

    logic [LB-1:0]             refill_line_q, refill_line_n;
    logic [L1_WAY_BITS-1:0]    fill_way_q, fill_way_n;
    logic [LB-1:0]             wb_line_q,  wb_line_n;
    logic [MEMORY_ADDR_WIDTH-1:0] wb_addr_q, wb_addr_n;
    logic [L1_INDEX_BITS-1:0]  flush_set_q, flush_set_n;
    logic [L1_WAY_BITS-1:0]    flush_way_q, flush_way_n;

    // ---------------- C4a probe state ----------------
    logic [L1_INDEX_BITS-1:0]  pidx_q, pidx_n;
    logic [L1_TAG_BITS-1:0]    ptag_q, ptag_n;
    logic [L1_WAY_BITS-1:0]    pway_q, pway_n;
    logic                      probe_done_q, probe_done_n;  // this probe resolved
    logic                      probe_start;                 // begin a probe in S_IDLE
    assign probe_start = probe_valid && !probe_done_q && !flush_req;
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
    // A pending probe takes priority over a new request in S_IDLE.
    assign req_ready = (state_q == S_IDLE) && !flush_req && !probe_start;
    assign req_fire  = req_valid && req_ready;
    assign serve_hit  = (state_q == S_LOOKUP) && op_valid_q &&  any_hit;
    assign serve_miss = (state_q == S_LOOKUP) && op_valid_q && !any_hit;
    assign store_hit  = serve_hit && op_write_q;

    // Load response: a hit in S_LOOKUP, or the install of a load miss.
    logic load_resp_lookup, load_resp_install;
    assign load_resp_lookup  = serve_hit && !op_write_q;
    assign load_resp_install = (state_q == S_INSTALL) && !op_write_q;
    assign resp_valid = load_resp_lookup || load_resp_install;
    assign resp_addr  = op_addr_q;
    always_comb begin
        if (load_resp_install) resp_data = refill_line_q[op_woff*XLEN +: XLEN];
        else                   resp_data = dat_rdata[hit_way][op_woff*XLEN +: XLEN];
    end
    // Store accept (PTW write ack): hit store in S_LOOKUP, or install of a store
    // miss. (The LSQ ignores this; the PTW uses it as its write ack.)
    assign wr_accept = store_hit || ((state_q == S_INSTALL) && op_write_q);

    assign ev_access = serve_hit || serve_miss;
    assign ev_miss   = serve_miss;
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
                if (flush_req) begin
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
            S_WB_WAIT:  if (nmi_resp.valid) state_n = S_FILL_REQ;
            S_FILL_REQ: if (nmi_req_ready)  state_n = S_FILL_WAIT;
            S_FILL_WAIT: if (nmi_resp.valid) begin
                refill_line_n = nmi_resp.rdata;
                state_n       = S_INSTALL;
            end
            S_INSTALL: state_n = S_IDLE;

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
    always_comb begin
        op_valid_n2 = op_valid_q;
        op_write_n2 = op_write_q;
        op_addr_n2  = op_addr_q;
        op_wdata_n2 = op_wdata_q;
        op_wmask_n2 = op_wmask_q;
        if ((state_q == S_IDLE) && req_fire) begin
            op_valid_n2 = 1'b1;
            op_write_n2 = req_write;
            op_addr_n2  = req_waddr;
            op_wdata_n2 = req_wdata;
            op_wmask_n2 = req_wmask;
        end else if (state_q == S_LOOKUP && any_hit) begin
            op_valid_n2 = 1'b0;
        end else if (state_q == S_INSTALL) begin
            op_valid_n2 = 1'b0;
        end
    end

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
    end

    always_ff @(posedge clk or negedge rst_l) begin
        if (!rst_l) begin
            state_q       <= S_IDLE;
            op_valid_q    <= 1'b0;
            op_write_q    <= 1'b0;
            op_addr_q     <= '0;
            op_wdata_q    <= '0;
            op_wmask_q    <= '0;
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
            if ((state_q == S_WB_REQ || state_q == S_FILL_REQ ||
                 state_q == S_FLUSH_WB_REQ || state_q == S_PROBE_WB_REQ) && nmi_req_ready)
                gen_q <= gen_q + 2'd1;
            for (int s = 0; s < L1_SETS; s += 1) begin
                valid_q[s] <= valid_n[s];
                dirty_q[s] <= dirty_n[s];
            end
            if (plru_upd_en) plru_q[op_idx] <= plru_next;
        end
    end

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
