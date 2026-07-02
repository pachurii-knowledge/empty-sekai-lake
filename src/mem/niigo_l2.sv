/**
 * niigo_l2.sv  --  shared write-back / write-allocate NINE L2 (transparent).
 *
 * A line-granular L2 cache that interposes on the grant-and-go directory's NMI
 * memory leg (plans/multicore-ccd.md §2; see plans/l2-integration.md). It is an
 * NMI SLAVE upward (facing niigo_dir_gg's mem master) and an NMI MASTER downward
 * (facing nmi_mem_adapter / nmi_axi_bridge -> main_memory). The directory is
 * OBLIVIOUS to it: it still issues NMI_RD_LINE (S_MEMRD) / NMI_WR_LINE (S_MEMWR),
 * which now hit the L2 first. Cache-to-cache owner forwarding never reaches this
 * seam, so the owner > L2 > memory data-source priority falls out for free.
 *
 * 512 KiB, 8-way, 64 B lines, PIPT, per-line dirty, tree-PLRU (l2_plru). Both
 * faces are line-granular, so this is a strict simplification of l1_dcache:
 * NO word-merge, NO byte-mask store install, NO probe port, NO flush walk.
 *   - NMI_RD_LINE: hit -> return the line; miss -> (write back the dirty victim,)
 *     fill from memory, install clean, return the line.
 *   - NMI_WR_LINE: hit -> overwrite the line, mark dirty (no memory op); miss ->
 *     (write back the dirty victim,) write-allocate the supplied line dirty with
 *     NO fill read (the whole 64 B line is supplied).
 * A dirty victim is ALWAYS written back before it is dropped, so memory+L2 always
 * hold a usable copy -- the directory's "always data_present" invariant (NINE,
 * niigo_dir_gg.sv:17) is preserved and the cache is value-transparent (only
 * latency differs). NINE: an L2 eviction never back-invalidates any L1.
 *
 * Full-size SETS/WAYS default to the package geometry; SETS may be shrunk for
 * Verilator sim (the module derives its own index/tag from SETS). WAYS is fixed
 * at 8 (l2_plru is an 8-way tree).
 */

`include "niigo_mem.vh"

`default_nettype none

module niigo_l2
    import RISCV_UArch::MEMORY_ADDR_WIDTH;
    import NIIGO_Mem::*;
#(
    parameter int SETS = L2_SETS,     // 1024 full-size; shrinkable for sim
    parameter int WAYS = L2_WAYS      // 8 (fixed; l2_plru is an 8-way tree)
)(
    input  wire logic clk,
    input  wire logic rst_l,

    // ---- NMI slave (up, facing the directory's mem master) ----
    input  wire nmi_req_t  s_req,
    output logic           s_req_ready,
    output nmi_resp_t      s_resp,

    // ---- NMI master (down, facing nmi_mem_adapter / nmi_axi_bridge) ----
    output nmi_req_t       m_req,
    input  wire logic      m_req_ready,
    input  nmi_resp_t      m_resp
);

    localparam int LB       = LINE_BITS;                 // 512
    localparam int IDX_BITS = $clog2(SETS);
    localparam int WAY_BITS = $clog2(WAYS);
    localparam int TAG_BITS = MEMORY_ADDR_WIDTH - IDX_BITS - LINE_WORD_BITS;

    // Word-address field extraction from this L2's own geometry (SETS may differ
    // from the package L2_SETS when shrunk for sim).
    function automatic logic [IDX_BITS-1:0] l2i(input logic [MEMORY_ADDR_WIDTH-1:0] wa);
        l2i = wa[LINE_WORD_BITS +: IDX_BITS];
    endfunction
    function automatic logic [TAG_BITS-1:0] l2t(input logic [MEMORY_ADDR_WIDTH-1:0] wa);
        l2t = wa[MEMORY_ADDR_WIDTH-1 : LINE_WORD_BITS + IDX_BITS];
    endfunction
    // Reconstruct the line-base word address of a (tag, set).
    function automatic logic [MEMORY_ADDR_WIDTH-1:0]
            line_addr(input logic [TAG_BITS-1:0] t, input logic [IDX_BITS-1:0] s);
        line_addr = {t, s, {LINE_WORD_BITS{1'b0}}};
    endfunction

    // ---------------- arrays + flop metadata ----------------
    logic                          tag_ren, tag_wen;
    logic [IDX_BITS-1:0]           tag_ridx, tag_widx;
    logic [WAY_BITS-1:0]           tag_wway;
    logic [TAG_BITS-1:0]           tag_wtag;
    logic [WAYS-1:0][TAG_BITS-1:0] tag_rdata;

    logic                          dat_ren, dat_wen;
    logic [IDX_BITS-1:0]           dat_ridx, dat_widx;
    logic [WAY_BITS-1:0]           dat_wway;
    logic [LB-1:0]                 dat_wdata;
    logic [LB/8-1:0]               dat_wmask;
    logic [WAYS-1:0][LB-1:0]       dat_rdata;

    l1_tag_array #(.SETS(SETS), .WAYS(WAYS), .TAG_BITS(TAG_BITS)) Tags (  // NINE: no back-invalidate
        .clk, .ren(tag_ren), .ridx(tag_ridx), .rtag(tag_rdata),
        .wen(tag_wen), .widx(tag_widx), .wway(tag_wway), .wtag(tag_wtag)
    );
    l1_data_array #(.SETS(SETS), .WAYS(WAYS), .LINE_BITS(LB)) Data (
        .clk, .ren(dat_ren), .ridx(dat_ridx), .rdata(dat_rdata),
        .wen(dat_wen), .widx(dat_widx), .wway(dat_wway),
        .wdata(dat_wdata), .wmask(dat_wmask)
    );

    logic [WAYS-1:0] valid_q [SETS];
    logic [WAYS-1:0] valid_n [SETS];
    logic [WAYS-1:0] dirty_q [SETS];
    logic [WAYS-1:0] dirty_n [SETS];
    logic [6:0]      plru_q  [SETS];
    logic [1:0]      gen_q;

    // ---------------- request latch ----------------
    logic                          op_write_q;
    logic [MEMORY_ADDR_WIDTH-1:0]  op_addr_q;
    logic [LB-1:0]                 op_wdata_q;
    logic [3:0]                    op_id_q;
    logic                          err_q;

    logic [IDX_BITS-1:0] op_idx;
    logic [TAG_BITS-1:0] op_tag;
    assign op_idx = l2i(op_addr_q);
    assign op_tag = l2t(op_addr_q);

    // ---------------- states ----------------
    typedef enum logic [2:0] {
        S_IDLE, S_LOOKUP, S_WB_REQ, S_WB_WAIT, S_FILL_REQ, S_FILL_WAIT, S_INSTALL, S_RESP
    } state_e;
    state_e state_q, state_n;

    logic [LB-1:0]                refill_line_q, refill_line_n;
    logic [LB-1:0]                resp_line_q,   resp_line_n;
    logic [WAY_BITS-1:0]          fill_way_q,    fill_way_n;
    logic [LB-1:0]                wb_line_q,     wb_line_n;
    logic [MEMORY_ADDR_WIDTH-1:0] wb_addr_q,     wb_addr_n;

    // ---------------- accept ----------------
    logic accept;
    assign s_req_ready = (state_q == S_IDLE);
    assign accept      = s_req.valid && s_req_ready;

    // ---------------- hit detection (valid in S_LOOKUP) ----------------
    logic [WAYS-1:0]     hit_oh;
    logic                any_hit;
    logic [WAY_BITS-1:0] hit_way;
    always_comb begin
        for (int w = 0; w < WAYS; w += 1)
            hit_oh[w] = valid_q[op_idx][w] && (tag_rdata[w] == op_tag);
        any_hit = |hit_oh;
        hit_way = '0;
        for (int w = 0; w < WAYS; w += 1) if (hit_oh[w]) hit_way = WAY_BITS'(w);
    end

    // ---------------- PLRU ----------------
    logic [WAY_BITS-1:0] victim;
    logic                plru_upd_en;
    logic [WAY_BITS-1:0] plru_acc_way;
    logic [6:0]          plru_next;
    l2_plru #(.WAYS(WAYS)) Plru (
        .state(plru_q[op_idx]), .valid(valid_q[op_idx]), .victim(victim),
        .update_en(plru_upd_en), .access_way(plru_acc_way), .next_state(plru_next)
    );

    // ---------------- NMI master request ----------------
    always_comb begin
        m_req = '0;
        unique case (state_q)
            S_WB_REQ: begin
                m_req.valid = 1'b1;
                m_req.op    = NMI_WR_LINE;
                m_req.waddr = wb_addr_q;
                m_req.id    = {NMI_SRC_DWB, gen_q};
                m_req.wdata = wb_line_q;
            end
            S_FILL_REQ: begin
                m_req.valid = 1'b1;
                m_req.op    = NMI_RD_LINE;
                m_req.waddr = l1_line_base(op_addr_q);
                m_req.id    = {NMI_SRC_DFILL, gen_q};
            end
            default: ;
        endcase
    end

    // ---------------- slave response (single pulse in S_RESP) ----------------
    always_comb begin
        s_resp       = '0;
        s_resp.valid = (state_q == S_RESP);
        s_resp.id    = op_id_q;
        s_resp.rdata = resp_line_q;
        s_resp.err   = err_q;
    end

    // ---------------- array read presentation ----------------
    always_comb begin
        tag_ren = 1'b0; tag_ridx = op_idx;
        dat_ren = 1'b0; dat_ridx = op_idx;
        if ((state_q == S_IDLE) && accept) begin
            tag_ren = 1'b1; tag_ridx = l2i(s_req.waddr);
            dat_ren = 1'b1; dat_ridx = l2i(s_req.waddr);
        end
    end

    // ---------------- array writes ----------------
    always_comb begin
        tag_wen = 1'b0; tag_widx = op_idx; tag_wway = fill_way_q; tag_wtag = op_tag;
        dat_wen = 1'b0; dat_widx = op_idx; dat_wway = fill_way_q;
        dat_wdata = op_wdata_q; dat_wmask = '1;
        if ((state_q == S_LOOKUP) && any_hit && op_write_q) begin
            // WR_LINE hit: overwrite the resident line in place (no tag change).
            dat_wen  = 1'b1;
            dat_wway = hit_way;
        end else if (state_q == S_INSTALL) begin
            // Fill (read miss) or write-allocate (write miss) into the victim way.
            tag_wen   = 1'b1;
            dat_wen   = 1'b1;
            dat_wdata = op_write_q ? op_wdata_q : refill_line_q;
        end
    end

    // ---------------- PLRU update ----------------
    always_comb begin
        plru_upd_en  = 1'b0;
        plru_acc_way = hit_way;
        if ((state_q == S_LOOKUP) && any_hit) begin
            plru_upd_en  = 1'b1;
            plru_acc_way = hit_way;
        end else if (state_q == S_INSTALL) begin
            plru_upd_en  = 1'b1;
            plru_acc_way = fill_way_q;
        end
    end

    // ---------------- next-state / datapath ----------------
    always_comb begin
        state_n       = state_q;
        refill_line_n = refill_line_q;
        resp_line_n   = resp_line_q;
        fill_way_n    = fill_way_q;
        wb_line_n     = wb_line_q;
        wb_addr_n     = wb_addr_q;

        unique case (state_q)
            S_IDLE: begin
                if (accept) state_n = S_LOOKUP;    // op latched below; index presented above
            end

            S_LOOKUP: begin
                if (any_hit) begin
                    // Read hit: return the resident line. Write hit: overwrite (above).
                    resp_line_n = dat_rdata[hit_way];
                    state_n     = S_RESP;
                end else begin
                    fill_way_n = victim;
                    if (valid_q[op_idx][victim] && dirty_q[op_idx][victim]) begin
                        // Evict the dirty victim first.
                        wb_line_n = dat_rdata[victim];
                        wb_addr_n = line_addr(tag_rdata[victim], op_idx);
                        state_n   = S_WB_REQ;
                    end else if (op_write_q) begin
                        state_n = S_INSTALL;       // write miss, clean victim: no fill read
                    end else begin
                        state_n = S_FILL_REQ;      // read miss, clean victim: fill from memory
                    end
                end
            end

            S_WB_REQ:  if (m_req_ready)  state_n = S_WB_WAIT;
            S_WB_WAIT: if (m_resp.valid) state_n = op_write_q ? S_INSTALL : S_FILL_REQ;

            S_FILL_REQ:  if (m_req_ready)  state_n = S_FILL_WAIT;
            S_FILL_WAIT: if (m_resp.valid) begin
                refill_line_n = m_resp.rdata;
                state_n       = S_INSTALL;
            end

            S_INSTALL: begin
                resp_line_n = op_write_q ? op_wdata_q : refill_line_q;
                state_n     = S_RESP;
            end

            S_RESP: state_n = S_IDLE;

            default: state_n = S_IDLE;
        endcase
    end

    // ---------------- op latch ----------------
    logic                          op_write_n;
    logic [MEMORY_ADDR_WIDTH-1:0]  op_addr_n;
    logic [LB-1:0]                 op_wdata_n;
    logic [3:0]                    op_id_n;
    logic                          err_n;
    always_comb begin
        op_write_n = op_write_q;
        op_addr_n  = op_addr_q;
        op_wdata_n = op_wdata_q;
        op_id_n    = op_id_q;
        err_n      = err_q;
        if ((state_q == S_IDLE) && accept) begin
            op_write_n = (s_req.op == NMI_WR_LINE);
            op_addr_n  = s_req.waddr;
            op_wdata_n = s_req.wdata;
            op_id_n    = s_req.id;
            err_n      = 1'b0;
        end else if ((state_q == S_FILL_WAIT) && m_resp.valid) begin
            err_n = m_resp.err;
        end
    end

    // ---------------- valid / dirty next state ----------------
    always_comb begin
        for (int s = 0; s < SETS; s += 1) begin
            valid_n[s] = valid_q[s];
            dirty_n[s] = dirty_q[s];
        end
        if ((state_q == S_LOOKUP) && any_hit && op_write_q) begin
            dirty_n[op_idx][hit_way] = 1'b1;                 // WR_LINE hit: dirty in place
        end
        if (state_q == S_INSTALL) begin
            valid_n[op_idx][fill_way_q] = 1'b1;
            dirty_n[op_idx][fill_way_q] = op_write_q;        // write-allocate installs dirty
        end
    end

    always_ff @(posedge clk or negedge rst_l) begin
        if (!rst_l) begin
            state_q       <= S_IDLE;
            op_write_q    <= 1'b0;
            op_addr_q     <= '0;
            op_wdata_q    <= '0;
            op_id_q       <= '0;
            err_q         <= 1'b0;
            refill_line_q <= '0;
            resp_line_q   <= '0;
            fill_way_q    <= '0;
            wb_line_q     <= '0;
            wb_addr_q     <= '0;
            gen_q         <= '0;
            for (int s = 0; s < SETS; s += 1) begin
                valid_q[s] <= '0;
                dirty_q[s] <= '0;
                plru_q[s]  <= '0;
            end
        end else begin
            state_q       <= state_n;
            op_write_q    <= op_write_n;
            op_addr_q     <= op_addr_n;
            op_wdata_q    <= op_wdata_n;
            op_id_q       <= op_id_n;
            err_q         <= err_n;
            refill_line_q <= refill_line_n;
            resp_line_q   <= resp_line_n;
            fill_way_q    <= fill_way_n;
            wb_line_q     <= wb_line_n;
            wb_addr_q     <= wb_addr_n;
            if ((state_q == S_WB_REQ || state_q == S_FILL_REQ) && m_req_ready)
                gen_q <= gen_q + 2'd1;
            for (int s = 0; s < SETS; s += 1) begin
                valid_q[s] <= valid_n[s];
                dirty_q[s] <= dirty_n[s];
            end
            if (plru_upd_en) plru_q[op_idx] <= plru_next;
        end
    end

`ifndef SYNTHESIS
    // The directory only ever issues line ops; catch any other op loudly.
    always_ff @(posedge clk) if (rst_l && accept &&
            !(s_req.op == NMI_RD_LINE || s_req.op == NMI_WR_LINE))
        $error("niigo_l2: unexpected NMI op %0d (only RD_LINE/WR_LINE supported)", s_req.op);
`endif

`ifdef AGENT_DEBUG
    always_ff @(posedge clk) if (rst_l) begin
        if ((state_q == S_IDLE) && accept)
            $display("[L2] accept %s waddr=%0h idx=%0d tag=%0h", s_req.op==NMI_WR_LINE?"WR":"RD",
                     s_req.waddr, l2i(s_req.waddr), l2t(s_req.waddr));
        if ((state_q == S_LOOKUP) && any_hit)
            $display("[L2] HIT way=%0d %s", hit_way, op_write_q?"WR":"RD");
        if ((state_q == S_LOOKUP) && !any_hit)
            $display("[L2] MISS idx=%0d victim=%0d vld=%b drt=%b", op_idx, victim,
                     valid_q[op_idx][victim], dirty_q[op_idx][victim]);
        if ((state_q == S_WB_REQ) && m_req_ready)
            $display("[L2] WB   waddr=%0h", wb_addr_q);
        if ((state_q == S_FILL_WAIT) && m_resp.valid)
            $display("[L2] FILL rdata[0]=%0h", m_resp.rdata[63:0]);
    end
`endif

endmodule : niigo_l2

`default_nettype wire
