/**
 * niigo_dir.sv  --  M1 dual-core MOESI directory / coherence point
 *
 * The serialisation point of the 4-core CCD (plans/multicore-ccd.md §3/§9.5), brought up
 * for the M1 milestone: 2 cores (parameterised), one directory, no mesh — cores talk to the
 * directory over the M1 full-line CMI links (niigo_ccd_m1.vh). Backing store is memory via
 * the existing NMI bus (niigo_mem.vh); there is no L2 *data* array in M1 (NINE: data lives
 * in the owner cache or in memory — the L2 URAM is an M2+ area concern).
 *
 * Implements the §9 directory FSM the formal models (formal/moesi_ccd*.m) verified, in a
 * CORRECTNESS-FIRST form: GLOBALLY SERIALISED (one transaction at a time, like l1_dcache)
 * and DIR-MEDIATED (the dir forwards/invalidates, COLLECTS the Inv-Acks itself, sources the
 * data, and grants — no requester-side ack-counting or Unblock). The verified grant-and-go +
 * ack-to-requester + Unblock optimisation (D6/§13.4) is the M3 perf refinement. The 3-valued
 * owner-next (R-a/§13.9d) IS modelled: on a FwdGetS the owner reports ON_O/ON_S so the dir
 * resolves EM/O -> O vs S without telling E from M.
 *
 * Data sourcing (NINE, §13.9a): a dirty owner (EM/O) forwards cache-to-cache; otherwise
 * (DIR_I/DIR_S, memory current) the dir reads the line from memory. PutM/PutO flush to memory.
 *
 * Two-process FSM: one always_comb (outputs + *_n next values), one always_ff (register).
 */
`include "niigo_mem.vh"
`include "niigo_cmi.vh"
`include "niigo_ccd_m1.vh"
`default_nettype none
module niigo_dir
    import RISCV_UArch::MEMORY_ADDR_WIDTH;
    import NIIGO_Mem::*;
    import NIIGO_CMI::cmi_state_e, NIIGO_CMI::dir_state_e,
           NIIGO_CMI::CMI_S, NIIGO_CMI::CMI_E, NIIGO_CMI::CMI_M,
           NIIGO_CMI::DIR_I, NIIGO_CMI::DIR_S, NIIGO_CMI::DIR_EM, NIIGO_CMI::DIR_O,
           NIIGO_CMI::OP_GETS, NIIGO_CMI::OP_GETM, NIIGO_CMI::OP_UPGRADE,
           NIIGO_CMI::OP_PUTM, NIIGO_CMI::OP_PUTO, NIIGO_CMI::OP_PUTS, NIIGO_CMI::OP_PUTE,
           NIIGO_CMI::OP_FWD_GETS, NIIGO_CMI::OP_FWD_GETM, NIIGO_CMI::OP_INV,
           NIIGO_CMI::OP_DATA, NIIGO_CMI::OP_INV_ACK, NIIGO_CMI::OP_ACK;
    import NIIGO_CCD_M1::*;
#(
    // CORES = the package's NUM_CORES (4) so core-id widths match ccd_msg_t; M1 drives 2 of them.
    parameter int CORES    = NIIGO_CMI::NUM_CORES,
    parameter int DIR_SETS = 8
)(
    input  wire logic clk,
    input  wire logic rst_l,
    // per-core links (M1 full-line CMI): up = core->dir, down = dir->core
    input  wire ccd_chan_t  up_i        [CORES],
    output logic            up_ready_o  [CORES],
    output ccd_chan_t       down_o      [CORES],
    input  wire logic       down_ready_i[CORES],
    // backing memory (NMI master)
    output nmi_req_t   mem_req_o,
    input  wire logic  mem_req_ready_i,
    input  nmi_resp_t  mem_resp_i
);
    localparam int CW = NIIGO_CMI::CORE_ID_W;   // core-id width (matches ccd_msg_t)
    localparam int DIDX_W = $clog2(DIR_SETS);
    /* verilator lint_off ENUMVALUE */           // '{default:'0} hits enum members whose 0 IS valid
                                                 // (CMI_I/DIR_I/OP_GETS/NMI_RD_LINE/ON_NA all = 0)
    // laddr is a line-aligned WORD address (NMI waddr semantics) -> the dir set index + tag skip
    // the low LINE_WORD_BITS word-offset bits (else distinct lines alias on the offset).
    localparam int LWB    = NIIGO_Mem::LINE_WORD_BITS;
    localparam int DTAG_W = MEMORY_ADDR_WIDTH - DIDX_W - LWB;

    typedef struct packed {
        logic              valid;
        logic [DTAG_W-1:0] tag;
        dir_state_e        dstate;
        logic [CORES-1:0]  sharers;   // d_sharers (L1D)
        logic [CW-1:0]     owner;     // valid in EM/O
    } dent_t;

    typedef enum logic [2:0] { S_IDLE, S_FWD, S_INV, S_MEMRD, S_MEMWR, S_GRANT } fsm_e;

    function automatic logic [DIDX_W-1:0] didx(input logic [MEMORY_ADDR_WIDTH-1:0] la);
        didx = la[LWB +: DIDX_W];
    endfunction
    function automatic logic [DTAG_W-1:0] dtag(input logic [MEMORY_ADDR_WIDTH-1:0] la);
        dtag = la[MEMORY_ADDR_WIDTH-1 : LWB + DIDX_W];
    endfunction
    function automatic logic [CW-1:0] count_other(input logic [CORES-1:0] sh, input logic [CW-1:0] r);
        logic [CW-1:0] n; n = '0;
        for (int c = 0; c < CORES; c++) if (sh[c] && (c[CW-1:0] != r)) n = n + 1'b1;
        return n;
    endfunction

    // ---- registers ----
    fsm_e                        st_q;
    dent_t                       dir_q [DIR_SETS];
    logic [CW-1:0]               rr_q;
    ccd_msg_t                    cur_q;          // the request being served
    logic [CW-1:0]               cur_core_q;
    logic [DIDX_W-1:0]           cur_idx_q;
    cmi_state_e                  gst_q;          // grant state
    logic [LINE_BITS-1:0]        gline_q;        // grant line
    logic                        gdata_q;        // grant carries data (else ack-only)
    logic [CW:0]                 acks_q;         // outstanding Inv-Acks (CW+1 bits: 0..CORES)
    logic                        snoop_sent_q;   // outbound Fwd/Inv has been sent
    dir_state_e                  nds_q;          // dir state to commit at finalize
    logic [CORES-1:0]            nsh_q;
    logic [CW-1:0]               nown_q;
    logic                        nset_q;         // commit owner field

    // ---- next-state combinational ----
    fsm_e                        st_n;
    logic [CW-1:0]               rr_n;
    ccd_msg_t                    cur_n;
    logic [CW-1:0]               cur_core_n;
    logic [DIDX_W-1:0]           cur_idx_n;
    cmi_state_e                  gst_n;
    logic [LINE_BITS-1:0]        gline_n;
    logic                        gdata_n;
    logic [CW:0]                 acks_n;
    logic                        snoop_sent_n;
    dir_state_e                  nds_n;
    logic [CORES-1:0]            nsh_n;
    logic [CW-1:0]               nown_n;
    logic                        nset_n;
    logic                        dir_we;
    logic [DIDX_W-1:0]           dir_widx;
    dent_t                       dir_wval;

    nmi_req_t                    mem_req_c;
    ccd_chan_t                   down_c   [CORES];
    logic                        up_rdy_c [CORES];

    // request pick (round-robin) in S_IDLE
    logic          pick_v;
    logic [CW-1:0] pick_core;
    always_comb begin
        pick_v = 1'b0; pick_core = '0;
        for (int k = 0; k < CORES; k++) begin
            automatic int c = (int'(rr_q) + k) % CORES;
            automatic ccd_msg_t mm = up_i[c].msg;
            if (!pick_v && up_i[c].valid &&
                (mm.op==OP_GETS || mm.op==OP_GETM || mm.op==OP_UPGRADE ||
                 mm.op==OP_PUTM || mm.op==OP_PUTO || mm.op==OP_PUTS || mm.op==OP_PUTE))
            begin pick_v = 1'b1; pick_core = c[CW-1:0]; end
        end
    end

    // current dir entry (NINE: miss => DIR_I)
    dent_t cur_ent;
    always_comb begin
        cur_ent = dir_q[cur_idx_q];
        if (!cur_ent.valid || (cur_ent.tag != dtag(cur_q.laddr))) begin
            cur_ent.dstate = DIR_I; cur_ent.sharers = '0; cur_ent.owner = '0;
        end
    end

    // an Inv-Ack addressed to the current requester is present this cycle
    logic invack_v;
    always_comb begin
        invack_v = 1'b0;
        for (int c = 0; c < CORES; c++)
            if (up_i[c].valid && up_i[c].msg.op==OP_INV_ACK && (up_i[c].msg.req==cur_core_q))
                invack_v = 1'b1;
    end

    // ================= combinational FSM =================
    always_comb begin
        // defaults: hold all regs, idle outputs
        st_n = st_q; rr_n = rr_q; cur_n = cur_q; cur_core_n = cur_core_q; cur_idx_n = cur_idx_q;
        gst_n = gst_q; gline_n = gline_q; gdata_n = gdata_q; acks_n = acks_q;
        snoop_sent_n = snoop_sent_q; nds_n = nds_q; nsh_n = nsh_q; nown_n = nown_q; nset_n = nset_q;
        dir_we = 1'b0; dir_widx = cur_idx_q; dir_wval = '{default:'0};
        mem_req_c = '{default:'0};
        for (int c = 0; c < CORES; c++) begin down_c[c] = '{default:'0}; up_rdy_c[c] = 1'b0; end

        unique case (st_q)
        // -------------------------------------------------------------
        S_IDLE: if (pick_v) begin
            automatic ccd_msg_t       m  = up_i[pick_core].msg;
            automatic dent_t          e  = dir_q[didx(m.laddr)];
            automatic logic           hit= e.valid && (e.tag == dtag(m.laddr));
            automatic dir_state_e     ds = hit ? e.dstate  : DIR_I;
            automatic logic [CORES-1:0] sh = hit ? e.sharers : '0;
            automatic logic [CW-1:0]  ow = e.owner;
            automatic logic [CORES-1:0] bit_r = '0; bit_r[pick_core] = 1'b1;
            up_rdy_c[pick_core] = 1'b1;                 // consume the request
            cur_n = m; cur_core_n = pick_core; cur_idx_n = didx(m.laddr);
            rr_n = pick_core + 1'b1;             // CORES is a power of 2 -> CW-bit add wraps
            snoop_sent_n = 1'b0;

            unique case (m.op)
            OP_GETS: begin
                if (ds==DIR_EM || ds==DIR_O) begin
                    st_n = S_FWD;                       // owner forwards cache-to-cache
                end else begin
                    gdata_n = 1'b1;
                    gst_n   = (ds==DIR_I) ? CMI_E : CMI_S;
                    if (ds==DIR_I) begin nds_n=DIR_EM; nset_n=1'b1; nown_n=pick_core; nsh_n=bit_r; end
                    else           begin nds_n=DIR_S;  nset_n=1'b0; nsh_n = sh | bit_r;            end
                    st_n = S_MEMRD;
                end
            end
            OP_GETM: begin
                gdata_n=1'b1; gst_n=CMI_M; nds_n=DIR_EM; nset_n=1'b1; nown_n=pick_core; nsh_n=bit_r;
                if (ds==DIR_EM || ds==DIR_O) st_n = S_FWD;
                else if (ds==DIR_S && count_other(sh,pick_core)!=0) begin
                    acks_n = count_other(sh,pick_core); st_n = S_INV;
                end else st_n = S_MEMRD;
            end
            OP_UPGRADE: begin
                nds_n=DIR_EM; nset_n=1'b1; nown_n=pick_core; nsh_n=bit_r; gst_n=CMI_M;
                if ((ds==DIR_S || ds==DIR_O) && sh[pick_core]) begin
                    gdata_n = 1'b0;                     // requester keeps its own data
                    if (count_other(sh,pick_core)!=0) begin acks_n=count_other(sh,pick_core); st_n=S_INV; end
                    else st_n = S_GRANT;
                end else begin                          // lost the copy -> treat as GetM
                    gdata_n = 1'b1;
                    if (ds==DIR_EM || ds==DIR_O) st_n = S_FWD;
                    else if (ds==DIR_S && count_other(sh,pick_core)!=0) begin
                        acks_n=count_other(sh,pick_core); st_n=S_INV;
                    end else st_n = S_MEMRD;
                end
            end
            OP_PUTM, OP_PUTO: begin
                if ((ds==DIR_EM || ds==DIR_O) && (ow==pick_core)) begin
                    nsh_n = sh & ~bit_r; nset_n=1'b0;
                    nds_n = ((sh & ~bit_r)!=0) ? DIR_S : DIR_I;
                    st_n  = S_MEMWR;                    // flush dirty line
                end else begin
                    nsh_n = sh & ~bit_r; nset_n=1'b0; nds_n=ds; st_n=S_GRANT;
                end
            end
            OP_PUTE, OP_PUTS: begin
                automatic logic [CORES-1:0] sh2 = sh & ~bit_r;
                nsh_n = sh2; nset_n=1'b0;
                if (ds==DIR_EM && ow==pick_core) nds_n=DIR_I;
                else if (sh2==0)                 nds_n=DIR_I;
                else                             nds_n=DIR_S;
                st_n = S_GRANT;
            end
            default: ;
            endcase
        end

        // -------------------------------------------------------------
        S_FWD: begin
            if (!snoop_sent_q) begin
                down_c[cur_ent.owner].valid     = 1'b1;
                down_c[cur_ent.owner].msg.op    = (cur_q.op==OP_GETS) ? OP_FWD_GETS : OP_FWD_GETM;
                down_c[cur_ent.owner].msg.req   = cur_core_q;
                down_c[cur_ent.owner].msg.laddr = cur_q.laddr;
                if (down_ready_i[cur_ent.owner]) snoop_sent_n = 1'b1;
            end else if (up_i[cur_ent.owner].valid && up_i[cur_ent.owner].msg.op==OP_DATA) begin
                automatic logic [CORES-1:0] bit_r = '0; bit_r[cur_core_q]=1'b1;
                up_rdy_c[cur_ent.owner] = 1'b1;
                gline_n = up_i[cur_ent.owner].msg.line;
                snoop_sent_n = 1'b0;
                if (cur_q.op==OP_GETS) begin
                    gst_n = CMI_S;
                    if (up_i[cur_ent.owner].msg.onext == ON_O) begin
                        nds_n=DIR_O; nset_n=1'b1; nown_n=cur_ent.owner; nsh_n=cur_ent.sharers|bit_r;
                    end else begin                     // ON_S: E->S, no owner
                        nds_n=DIR_S; nset_n=1'b0; nsh_n=cur_ent.sharers|bit_r;
                    end
                end else begin                          // GETM / lost-Upgrade: requester -> M
                    gst_n=CMI_M; nds_n=DIR_EM; nset_n=1'b1; nown_n=cur_core_q; nsh_n=bit_r;
                end
                gdata_n = 1'b1;
                st_n = S_GRANT;
            end
        end

        // -------------------------------------------------------------
        S_INV: begin
            if (!snoop_sent_q) begin
                automatic logic all_rdy = 1'b1;
                for (int c = 0; c < CORES; c++)
                    if (cur_ent.sharers[c] && (c[CW-1:0]!=cur_core_q)) begin
                        down_c[c].valid     = 1'b1;
                        down_c[c].msg.op    = OP_INV;
                        down_c[c].msg.req   = cur_core_q;
                        down_c[c].msg.laddr = cur_q.laddr;
                        if (!down_ready_i[c]) all_rdy = 1'b0;
                    end
                if (all_rdy) snoop_sent_n = 1'b1;
            end else if (invack_v) begin
                for (int c = 0; c < CORES; c++)
                    if (up_i[c].valid && up_i[c].msg.op==OP_INV_ACK && (up_i[c].msg.req==cur_core_q))
                        up_rdy_c[c] = 1'b1;
                acks_n = acks_q - 1'b1;
                if (acks_q <= 1) begin
                    snoop_sent_n = 1'b0;
                    st_n = gdata_q ? S_MEMRD : S_GRANT;
                end
            end
        end

        // -------------------------------------------------------------
        S_MEMRD: begin
            mem_req_c.valid = 1'b1; mem_req_c.op = NMI_RD_LINE; mem_req_c.waddr = cur_q.laddr;
            if (mem_resp_i.valid) begin gline_n = mem_resp_i.rdata; st_n = S_GRANT; end
        end
        S_MEMWR: begin
            mem_req_c.valid = 1'b1; mem_req_c.op = NMI_WR_LINE; mem_req_c.waddr = cur_q.laddr;
            mem_req_c.wdata = cur_q.line;
            if (mem_resp_i.valid) st_n = S_GRANT;
        end

        // -------------------------------------------------------------
        S_GRANT: begin
            automatic logic is_put = (cur_q.op==OP_PUTM || cur_q.op==OP_PUTO ||
                                      cur_q.op==OP_PUTS || cur_q.op==OP_PUTE);
            down_c[cur_core_q].valid    = 1'b1;
            down_c[cur_core_q].msg.op   = is_put ? OP_ACK : OP_DATA;
            down_c[cur_core_q].msg.gst  = gst_q;
            down_c[cur_core_q].msg.req  = cur_core_q;
            down_c[cur_core_q].msg.line = gline_q;
            down_c[cur_core_q].msg.laddr= cur_q.laddr;
            if (down_ready_i[cur_core_q]) begin
                dir_we   = 1'b1;
                dir_widx = cur_idx_q;
                dir_wval.valid   = (nds_q != DIR_I);
                dir_wval.tag     = dtag(cur_q.laddr);
                dir_wval.dstate  = nds_q;
                dir_wval.sharers = nsh_q;
                dir_wval.owner   = nset_q ? nown_q : cur_ent.owner;
                st_n = S_IDLE;
            end
        end
        endcase
    end

    // ================= sequential =================
    always_ff @(posedge clk or negedge rst_l) begin
        if (!rst_l) begin
            st_q <= S_IDLE; rr_q <= '0; snoop_sent_q <= 1'b0; acks_q <= '0;
            for (int s = 0; s < DIR_SETS; s++) dir_q[s] <= '{default:'0};
        end else begin
            st_q <= st_n; rr_q <= rr_n; cur_q <= cur_n; cur_core_q <= cur_core_n;
            cur_idx_q <= cur_idx_n; gst_q <= gst_n; gline_q <= gline_n; gdata_q <= gdata_n;
            acks_q <= acks_n; snoop_sent_q <= snoop_sent_n;
            nds_q <= nds_n; nsh_q <= nsh_n; nown_q <= nown_n; nset_q <= nset_n;
            if (dir_we) dir_q[dir_widx] <= dir_wval;
        end
    end

    // ---- outputs ----
    assign mem_req_o = mem_req_c;
    always_comb for (int c = 0; c < CORES; c++) begin
        down_o[c]     = down_c[c];
        up_ready_o[c] = up_rdy_c[c];
    end

    /* verilator lint_on ENUMVALUE */
endmodule
`default_nettype wire
