/**
 * niigo_l1d_gg.sv  --  M3c non-blocking grant-and-go L1D MOESI agent
 *
 * The verified non-blocking L1D agent (plans/multicore-ccd.md §4; formal/moesi_ccd_v2/v3/v4b.m).
 * Replaces the M1 blocking niigo_l1d_moesi.sv. Fabric-agnostic clean message channels.
 *
 * One outstanding demand transaction (MSHR, D4). Acquire transients IS_D/IM_AD/IM_A/SM_AD/OM_A,
 * a completion phase (T_UNBLK/T_UNBLK_WB) that emits the UNBLOCK (+ paired refresh WB_DATA for an
 * ON_S/ON_I cache-to-cache read) so the directory finalises, and evict transients (the victim Put
 * is sent on the demand uplink, then the fill re-issues). Ack-to-requester down-counter (signed):
 * a GetM/Upgrade requester reaches M only when acks==0 && data_present (write-atomicity I3). A
 * separate snoop path services inbound Fwd/Inv off its own port; in an acquire transient it DEFERS
 * (sticky 1-slot) and ServeDeferred fires on reaching stable. A pre-switch step (rsv-kill on
 * FwdGetM/Inv, AMO-squash on any snoop) runs at both service and defer time.
 *
 * UNBLOCK is sent unconditionally after every acquire; the directory finalises a busy
 * (cache-to-cache / Upgrade) line and silently drops an UNBLOCK for a grant-and-go (non-busy)
 * line -- functionally identical to "UNBLOCK iff peer-sourced or Upgrade", simpler to drive.
 */
`include "niigo_mem.vh"
`include "niigo_cmi.vh"
`include "niigo_ccd_m1.vh"
`default_nettype none
module niigo_l1d_gg
    import RISCV_ISA::XLEN;
    import RISCV_UArch::MEMORY_ADDR_WIDTH;
    import NIIGO_Mem::LINE_BITS, NIIGO_Mem::LINE_WORD_BITS;
    import NIIGO_CMI::*;
    import NIIGO_CCD_M1::*;
#(
    parameter int unsigned CORE_ID = 0,
    parameter int          SETS    = 16
)(
    input  wire logic clk,
    input  wire logic rst_l,
    // ---- core LSQ port (blocking handshake) ----
    input  wire logic                          c_req_valid,
    output logic                               c_req_ready,
    input  wire l1_core_op_e                   c_req_op,
    input  wire l1_amo_op_e                    c_req_amo,
    input  wire logic [MEMORY_ADDR_WIDTH-1:0]  c_req_waddr,
    input  wire logic [XLEN-1:0]               c_req_wdata,
    output logic [XLEN-1:0]                     c_resp_rdata,
    output logic                               c_resp_sc_ok,
    // ---- demand uplink (to dir): GetS/GetM/Upgrade/Put*/UNBLOCK/WB_DATA ----
    output logic       dmd_valid,  output ccd_msg_t dmd_msg,  input wire logic dmd_ready,
    // ---- snoop-response uplink (to a peer core): Data-forward / InvAck ----
    output logic       snp_valid,  output ccd_msg_t snp_msg,  output logic [NODE_ID_W-1:0] snp_dst,
    input  wire logic  snp_ready,
    // ---- downlinks ----
    input  wire logic  snoop_valid, input wire ccd_msg_t snoop_msg, output logic snoop_ready, // C1 fwd/inv
    input  wire logic  resp_valid,  input wire ccd_msg_t resp_msg,  output logic resp_ready,   // C2 data
    input  wire logic  ack_valid,   input wire ccd_msg_t ack_msg,   output logic ack_ready     // C4 InvAck/WB_ACK
);
    localparam int IDX = $clog2(SETS);
    localparam int TAG = MEMORY_ADDR_WIDTH - IDX - LINE_WORD_BITS;
    localparam int AKW = ACK_CNT_W+2;   // signed ack down-counter with slack
    /* verilator lint_off ENUMVALUE */

    function automatic logic [IDX-1:0] ixf(input logic [MEMORY_ADDR_WIDTH-1:0] wa); ixf=wa[LINE_WORD_BITS +: IDX]; endfunction
    function automatic logic [TAG-1:0] tgf(input logic [MEMORY_ADDR_WIDTH-1:0] wa); tgf=wa[MEMORY_ADDR_WIDTH-1:LINE_WORD_BITS+IDX]; endfunction
    function automatic logic [LINE_WORD_BITS-1:0] off(input logic [MEMORY_ADDR_WIDTH-1:0] wa); off=wa[LINE_WORD_BITS-1:0]; endfunction
    function automatic logic [MEMORY_ADDR_WIDTH-1:0] lbase(input logic [MEMORY_ADDR_WIDTH-1:0] wa);
        lbase={wa[MEMORY_ADDR_WIDTH-1:LINE_WORD_BITS],{LINE_WORD_BITS{1'b0}}}; endfunction
    function automatic logic [XLEN-1:0] wrd(input logic [LINE_BITS-1:0] l, input logic [LINE_WORD_BITS-1:0] o); wrd=l[o*XLEN +: XLEN]; endfunction
    function automatic logic [LINE_BITS-1:0] wmrg(input logic [LINE_BITS-1:0] l, input logic [LINE_WORD_BITS-1:0] o, input logic [XLEN-1:0] w);
        wmrg=l; wmrg[o*XLEN +: XLEN]=w; endfunction
    function automatic logic [XLEN-1:0] amo(input l1_amo_op_e a, input logic [XLEN-1:0] od, input logic [XLEN-1:0] o);
        unique case(a) AMO_ADD:amo=od+o; AMO_SWAP:amo=o; AMO_OR:amo=od|o; AMO_AND:amo=od&o; AMO_XOR:amo=od^o; default:amo=o; endcase
    endfunction
    function automatic cmi_op_e put_for(input cmi_state_e s);
        unique case(s) CMI_M:put_for=OP_PUTM; CMI_O:put_for=OP_PUTO; CMI_E:put_for=OP_PUTE; default:put_for=OP_PUTS; endcase
    endfunction
    function automatic logic [NODE_ID_W-1:0] cnode(input logic [CORE_ID_W-1:0] c); cnode={{(NODE_ID_W-CORE_ID_W){1'b0}},c}; endfunction

    typedef enum logic [3:0] {
        T_NONE, T_IS_D, T_IM_AD, T_IM_A, T_SM_AD, T_OM_A,
        T_UNBLK, T_UNBLK_WB, T_EVICT, T_EI_WAIT
    } tstate_e;
    typedef enum logic [2:0] { AT_NONE, AT_SC, AT_AMO_ACQ, AT_AMO_LOCKED, AT_AMO_REPLAY } atom_e;

    // ---- cache + agent registers ----
    cmi_state_e            state_q [SETS];
    logic [TAG-1:0]        tag_q   [SETS];
    logic [LINE_BITS-1:0]  data_q  [SETS];
    // MSHR (single)
    logic                  m_val_q;
    tstate_e               m_ts_q;
    logic [MEMORY_ADDR_WIDTH-1:0] m_lad_q;        // demanded line base
    logic [MEMORY_ADDR_WIDTH-1:0] m_vlad_q;       // victim line base
    cmi_state_e            m_vst_q;               // victim state
    logic signed [AKW-1:0] m_acks_q;
    logic                  m_data_q;
    l1_core_op_e           m_rop_q; l1_amo_op_e m_ramo_q;
    logic [LINE_WORD_BITS-1:0] m_woff_q; logic [XLEN-1:0] m_wd_q;
    atom_e                 m_atom_q;
    logic                  m_issued_q;
    cmi_owner_next_e       m_unon_q;              // onext to echo in UNBLOCK
    logic [LINE_BITS-1:0]  m_uwb_q;               // refresh line for the paired WB_DATA
    // deferred snoop (sticky)
    logic                  d_val_q; cmi_op_e d_op_q; logic [CORE_ID_W-1:0] d_req_q; logic [ACK_CNT_W-1:0] d_acks_q;
    // reservation
    logic                  rsv_v_q; logic [MEMORY_ADDR_WIDTH-1:0] rsv_l_q; logic [XLEN-1:0] rsv_val_q;
    // pending snoop-response (consume snoop now, drive answer next cycle)
    logic                  sr_pend_q; ccd_msg_t sr_msg_q; logic [NODE_ID_W-1:0] sr_dst_q;

    // ---- next-state vars ----
    logic cw_we; logic [IDX-1:0] cw_idx; cmi_state_e cw_st; logic [TAG-1:0] cw_tag; logic [LINE_BITS-1:0] cw_line;
    logic cww_we; logic [IDX-1:0] cww_idx; logic [LINE_WORD_BITS-1:0] cww_off; logic [XLEN-1:0] cww_val;
    logic cs_we; logic [IDX-1:0] cs_idx; cmi_state_e cs_val;       // demand-side state write
    logic css_we; logic [IDX-1:0] css_idx; cmi_state_e css_val;    // DLK-1: snoop-side state write (2nd port)
    logic m_val_n; tstate_e m_ts_n;
    logic [MEMORY_ADDR_WIDTH-1:0] m_lad_n, m_vlad_n; cmi_state_e m_vst_n;
    logic signed [AKW-1:0] m_acks_n; logic m_data_n;
    l1_core_op_e m_rop_n; l1_amo_op_e m_ramo_n; logic [LINE_WORD_BITS-1:0] m_woff_n; logic [XLEN-1:0] m_wd_n;
    atom_e m_atom_n; logic m_issued_n; cmi_owner_next_e m_unon_n; logic [LINE_BITS-1:0] m_uwb_n;
    logic d_val_n; cmi_op_e d_op_n; logic [CORE_ID_W-1:0] d_req_n; logic [ACK_CNT_W-1:0] d_acks_n;
    logic rsv_v_n; logic [MEMORY_ADDR_WIDTH-1:0] rsv_l_n; logic [XLEN-1:0] rsv_val_n;
    logic sr_pend_n; ccd_msg_t sr_msg_n; logic [NODE_ID_W-1:0] sr_dst_n;
    logic dmd_v_c; ccd_msg_t dmd_m_c;
    logic creq_rdy_c; logic [XLEN-1:0] crd_c; logic csc_c;
    logic snoop_rdy_c, resp_rdy_c, ack_rdy_c;

    function automatic ccd_msg_t umsg(input cmi_op_e op, input cmi_state_e gst, input cmi_owner_next_e on,
                                      input logic [CORE_ID_W-1:0] req, input logic [ACK_CNT_W-1:0] acks,
                                      input logic [LINE_BITS-1:0] line, input logic [MEMORY_ADDR_WIDTH-1:0] la);
        ccd_msg_t m; m='{default:'0};
        m.op=op; m.src=CORE_ID[CORE_ID_W-1:0]; m.gst=gst; m.onext=on; m.req=req; m.acks=acks; m.line=line; m.laddr=la;
        return m;
    endfunction

    // serve a deferred snoop on reaching stable. `line_val` is the CURRENT/just-written line
    // (the install/RMW cache writes this cycle are nonblocking, so data_q[ix] is still stale).
    task automatic serve_deferred(input logic [IDX-1:0] ix, input cmi_state_e nowst,
                                  input logic [MEMORY_ADDR_WIDTH-1:0] la,
                                  input logic [LINE_BITS-1:0] line_val);
        if (d_val_q && !sr_pend_q) begin
            d_val_n = 1'b0;
            sr_pend_n = 1'b1; sr_dst_n = cnode(d_req_q);
            unique case (d_op_q)
                OP_FWD_GETS: begin
                    sr_msg_n = umsg(OP_DATA, CMI_S, (nowst==CMI_E)?ON_S:ON_O, d_req_q, '0, line_val, la);
                    cs_we=1; cs_idx=ix; cs_val=(nowst==CMI_E)?CMI_S:CMI_O;
                end
                OP_FWD_GETM: begin
                    sr_msg_n = umsg(OP_DATA, CMI_M, ON_NA, d_req_q, d_acks_q, line_val, la);
                    cs_we=1; cs_idx=ix; cs_val=CMI_I;
                end
                OP_INV: begin
                    sr_msg_n = umsg(OP_INV_ACK, CMI_I, ON_NA, d_req_q, '0, '0, la);
                    cs_we=1; cs_idx=ix; cs_val=CMI_I;
                end
                default: sr_pend_n=1'b0;
            endcase
        end
    endtask

    always_comb begin
        cw_we=0; cw_idx='0; cw_st=CMI_I; cw_tag='0; cw_line='0;
        cww_we=0; cww_idx='0; cww_off='0; cww_val='0;
        cs_we=0; cs_idx='0; cs_val=CMI_I;
        css_we=0; css_idx='0; css_val=CMI_I;
        m_val_n=m_val_q; m_ts_n=m_ts_q; m_lad_n=m_lad_q; m_vlad_n=m_vlad_q; m_vst_n=m_vst_q;
        m_acks_n=m_acks_q; m_data_n=m_data_q; m_rop_n=m_rop_q; m_ramo_n=m_ramo_q;
        m_woff_n=m_woff_q; m_wd_n=m_wd_q; m_atom_n=m_atom_q; m_issued_n=m_issued_q;
        m_unon_n=m_unon_q; m_uwb_n=m_uwb_q;
        d_val_n=d_val_q; d_op_n=d_op_q; d_req_n=d_req_q; d_acks_n=d_acks_q;
        rsv_v_n=rsv_v_q; rsv_l_n=rsv_l_q; rsv_val_n=rsv_val_q;
        sr_pend_n=sr_pend_q; sr_msg_n=sr_msg_q; sr_dst_n=sr_dst_q;
        dmd_v_c=0; dmd_m_c='{default:'0};
        snp_valid=0; snp_msg='{default:'0}; snp_dst='0;
        creq_rdy_c=0; crd_c='0; csc_c=0;
        snoop_rdy_c=0; resp_rdy_c=0; ack_rdy_c=0;

        // (A) drive a pending snoop-response on the snoop-resp uplink
        if (sr_pend_q) begin
            snp_valid=1'b1; snp_msg=sr_msg_q; snp_dst=sr_dst_q;
            if (snp_ready) sr_pend_n=1'b0;
        end

        // (B) demand uplink: issue the request / Put / UNBLOCK / WB depending on tstate
        if (m_val_q && !m_issued_q) begin
            unique case (m_ts_q)
                T_EVICT: begin
                    automatic logic [IDX-1:0] vi=ixf(m_vlad_q);
                    dmd_v_c=1; dmd_m_c=umsg(put_for(m_vst_q),CMI_I,ON_NA,CORE_ID[CORE_ID_W-1:0],'0,
                                            (m_vst_q==CMI_M||m_vst_q==CMI_O)?data_q[vi]:'0, m_vlad_q);
                    if (dmd_ready) m_issued_n=1;
                end
                T_IS_D:  begin dmd_v_c=1; dmd_m_c=umsg(OP_GETS,   CMI_I,ON_NA,CORE_ID[CORE_ID_W-1:0],'0,'0,m_lad_q); if(dmd_ready)m_issued_n=1; end
                T_IM_AD: begin dmd_v_c=1; dmd_m_c=umsg(OP_GETM,   CMI_I,ON_NA,CORE_ID[CORE_ID_W-1:0],'0,'0,m_lad_q); if(dmd_ready)m_issued_n=1; end
                T_SM_AD: begin dmd_v_c=1; dmd_m_c=umsg(OP_UPGRADE,CMI_I,ON_NA,CORE_ID[CORE_ID_W-1:0],'0,'0,m_lad_q); if(dmd_ready)m_issued_n=1; end
                T_OM_A:  begin dmd_v_c=1; dmd_m_c=umsg(OP_UPGRADE,CMI_I,ON_NA,CORE_ID[CORE_ID_W-1:0],'0,'0,m_lad_q); if(dmd_ready)m_issued_n=1; end
                T_UNBLK: begin dmd_v_c=1; dmd_m_c=umsg(OP_UNBLOCK,CMI_I,m_unon_q,CORE_ID[CORE_ID_W-1:0],'0,'0,m_lad_q); if(dmd_ready)m_issued_n=1; end
                T_UNBLK_WB: begin dmd_v_c=1; dmd_m_c=umsg(OP_WB_DATA,CMI_I,ON_NA,CORE_ID[CORE_ID_W-1:0],'0,m_uwb_q,m_lad_q); if(dmd_ready)m_issued_n=1; end
                default: ;
            endcase
        end
        // (B2) advance the completion phases once their message has been accepted
        if (m_val_q && m_issued_q) begin
            unique case (m_ts_q)
                T_UNBLK: begin
                    // after UNBLOCK: if a refresh WB is owed (ON_S/ON_I read), send it; else done
                    if (m_unon_q==ON_S || m_unon_q==ON_I) begin m_ts_n=T_UNBLK_WB; m_issued_n=0; end
                    else begin m_val_n=0; m_ts_n=T_NONE; m_issued_n=0; end
                end
                T_UNBLK_WB: begin m_val_n=0; m_ts_n=T_NONE; m_issued_n=0; end
                default: ;
            endcase
        end

        // (C) snoop FSM: service or defer (pre-switch rsv-kill + AMO-squash)
        if (snoop_valid && !sr_pend_q) begin
            automatic logic [IDX-1:0] si=ixf(snoop_msg.laddr);
            automatic cmi_state_e cs=state_q[si];
            automatic logic in_txn = m_val_q && (m_lad_q==snoop_msg.laddr) &&
                                     (m_ts_q==T_IS_D||m_ts_q==T_IM_AD||m_ts_q==T_IM_A||m_ts_q==T_SM_AD||m_ts_q==T_OM_A);
            automatic logic defer  = m_val_q && (m_lad_q==snoop_msg.laddr) &&
                                     (m_ts_q==T_IS_D||m_ts_q==T_IM_AD||m_ts_q==T_IM_A);
            automatic logic kill   = (snoop_msg.op==OP_FWD_GETM||snoop_msg.op==OP_INV) && rsv_v_q && (rsv_l_q==snoop_msg.laddr);
            automatic logic sq     = in_txn && (m_atom_q==AT_AMO_ACQ||m_atom_q==AT_AMO_LOCKED);
            if (defer) begin
                if (!d_val_q) begin
                    snoop_rdy_c=1; d_val_n=1; d_op_n=snoop_msg.op; d_req_n=snoop_msg.req; d_acks_n=snoop_msg.acks;
                    if (kill) rsv_v_n=0; if (sq) m_atom_n=AT_AMO_REPLAY;
                    // SM_AD/OM_A INV demotes to IM_AD (re-fetch) -- handled in service branch below; defer only for IS_D/IM_*
                end
            end else begin
                snoop_rdy_c=1;
                if (kill) rsv_v_n=0; if (sq) m_atom_n=AT_AMO_REPLAY;
                unique case (snoop_msg.op)
                    OP_FWD_GETS: begin
                        sr_pend_n=1; sr_dst_n=cnode(snoop_msg.req);
                        sr_msg_n=umsg(OP_DATA,CMI_S,(cs==CMI_E)?ON_S:ON_O,snoop_msg.req,'0,data_q[si],snoop_msg.laddr);
                        css_we=1; css_idx=si; css_val=(cs==CMI_E)?CMI_S:CMI_O;
                    end
                    OP_FWD_GETM: begin
                        sr_pend_n=1; sr_dst_n=cnode(snoop_msg.req);
                        sr_msg_n=umsg(OP_DATA,CMI_M,ON_NA,snoop_msg.req,snoop_msg.acks,data_q[si],snoop_msg.laddr);
                        css_we=1; css_idx=si; css_val=CMI_I;
                    end
                    OP_INV: begin
                        sr_pend_n=1; sr_dst_n=cnode(snoop_msg.req);
                        sr_msg_n=umsg(OP_INV_ACK,CMI_I,ON_NA,snoop_msg.req,'0,'0,snoop_msg.laddr);
                        // INV in SM_AD/OM_A demotes to IM_AD (lost the shared copy); else invalidate
                        if (m_val_q && m_lad_q==snoop_msg.laddr && (m_ts_q==T_SM_AD||m_ts_q==T_OM_A)) begin
                            m_ts_n=T_IM_AD; m_data_n=0; m_issued_n=0;   // re-fetch as GetM
                        end else begin css_we=1; css_idx=si; css_val=CMI_I; end
                    end
                    default: ;
                endcase
            end
        end

        // (D) inbound DATA -> install / ack-accumulate / ReachM / completion
        if (resp_valid && m_val_q && (resp_msg.laddr==m_lad_q) &&
            (m_ts_q==T_IS_D||m_ts_q==T_IM_AD||m_ts_q==T_SM_AD||m_ts_q==T_OM_A)) begin
            automatic logic [IDX-1:0] ix=ixf(m_lad_q);
            automatic logic [LINE_WORD_BITS-1:0] o=m_woff_q;
            resp_rdy_c=1;
            unique case (m_ts_q)
            T_IS_D: begin
                cw_we=1; cw_idx=ix; cw_tag=tgf(m_lad_q); cw_line=resp_msg.line; cw_st=(resp_msg.gst==CMI_E)?CMI_E:CMI_S;
                creq_rdy_c=1; crd_c=wrd(resp_msg.line,o);
                if (m_rop_q==COP_LR) begin rsv_v_n=1; rsv_l_n=m_lad_q; rsv_val_n=wrd(resp_msg.line,o); end
                serve_deferred(ix, (resp_msg.gst==CMI_E)?CMI_E:CMI_S, m_lad_q, resp_msg.line);
                // a deferred snoop served this cycle downgrades the just-installed line: fold its
                // state into the install (cw_we has seq priority over cs_we, so cs_we would be lost)
                if (cs_we) begin
                    cw_st=cs_val; cs_we=1'b0;
                    // AGT-1: a deferred FwdGetM/INV drove the line non-readable -> the LR reservation
                    // just set above must be killed (post-ServeDeferred pre-switch, §4.5/§4.10), else a
                    // later SC to a line handed to a remote writer would wrongly succeed.
                    if (cs_val==CMI_I) rsv_v_n=1'b0;
                end
                // completion: UNBLOCK (echo onext from the data); refresh WB if ON_S/ON_I
                m_unon_n=resp_msg.onext; m_uwb_n=resp_msg.line;
                m_ts_n=T_UNBLK; m_issued_n=0;
            end
            T_IM_AD: begin
                cw_we=1; cw_idx=ix; cw_tag=tgf(m_lad_q); cw_line=resp_msg.line; cw_st=CMI_M;
                m_data_n=1; m_acks_n=m_acks_n+signed'({1'b0,resp_msg.acks});
                m_ts_n=T_IM_A;
            end
            T_SM_AD, T_OM_A: begin
                // no-data grant: keep clean value (do NOT install payload)
                m_data_n=1; m_acks_n=m_acks_n+signed'({1'b0,resp_msg.acks});
                m_ts_n=T_IM_A;
            end
            default: ;
            endcase
        end

        // (E) ReachM: in T_IM_A with acks==0 && data -> M, do the op, ServeDeferred, then UNBLOCK
        if (m_val_q && m_ts_q==T_IM_A && m_data_q && (m_acks_q==0) && !creq_rdy_c) begin
            automatic logic [IDX-1:0] ix=ixf(m_lad_q);
            automatic logic [LINE_WORD_BITS-1:0] o=m_woff_q;
            automatic logic [LINE_BITS-1:0] postline=data_q[ix];   // line after this cycle's write
            cs_we=1; cs_idx=ix; cs_val=CMI_M;
            unique case (m_atom_q)
                AT_NONE: begin cww_we=1; cww_idx=ix; cww_off=o; cww_val=m_wd_q; creq_rdy_c=1; rsv_v_n=0;
                               postline=wmrg(data_q[ix],o,m_wd_q); end
                AT_SC: begin
                    creq_rdy_c=1;
                    if (rsv_v_q && rsv_l_q==m_lad_q) begin cww_we=1; cww_idx=ix; cww_off=o; cww_val=m_wd_q; csc_c=1;
                                                            postline=wmrg(data_q[ix],o,m_wd_q); end else csc_c=0;
                    rsv_v_n=0; m_atom_n=AT_NONE;
                end
                AT_AMO_ACQ, AT_AMO_REPLAY: begin
                    automatic logic [XLEN-1:0] old=wrd(data_q[ix],o);
                    cww_we=1; cww_idx=ix; cww_off=o; cww_val=amo(m_ramo_q,old,m_wd_q);
                    crd_c=old; creq_rdy_c=1; rsv_v_n=0; m_atom_n=AT_NONE;
                    postline=wmrg(data_q[ix],o,amo(m_ramo_q,old,m_wd_q));
                end
                default: ;
            endcase
            serve_deferred(ix, CMI_M, m_lad_q, postline);
            m_unon_n=ON_NA; m_ts_n=T_UNBLK; m_issued_n=0;
        end

        // (G) inbound INV_ACK / WB_ACK
        if (ack_valid) begin
            ack_rdy_c=1;
            if (ack_msg.op==OP_INV_ACK) begin
                if (m_val_q && (m_ts_q==T_IM_A||m_ts_q==T_IM_AD||m_ts_q==T_SM_AD||m_ts_q==T_OM_A))
                    m_acks_n=m_acks_n-1;
            end else if (ack_msg.op==OP_ACK) begin
                if (m_val_q && m_ts_q==T_EVICT && m_issued_q) begin
                    automatic logic [IDX-1:0] vi=ixf(m_vlad_q);
                    cs_we=1; cs_idx=vi; cs_val=CMI_I;
                    if (rsv_v_q && rsv_l_q==m_vlad_q) rsv_v_n=0;
                    m_ts_n=T_IS_D; m_issued_n=0;          // re-issue (the fill); m_ts set by allocate below? no: set fill tstate
                end
            end
        end

        // (H) core request: only when no MSHR busy + no pending snoop-resp + no inbound snoop for this line
        if (!m_val_q && c_req_valid && !sr_pend_q &&
            !(snoop_valid && snoop_msg.laddr==lbase(c_req_waddr))) begin
            automatic logic [IDX-1:0] ix=ixf(c_req_waddr);
            automatic logic [LINE_WORD_BITS-1:0] o=off(c_req_waddr);
            automatic cmi_state_e cs=state_q[ix];
            automatic logic hit=(cs!=CMI_I)&&(tag_q[ix]==tgf(c_req_waddr));
            automatic logic wrable=(cs==CMI_M)||(cs==CMI_E);
            automatic logic occ=(cs!=CMI_I)&&(tag_q[ix]!=tgf(c_req_waddr));
            // common MSHR allocate fields
            m_lad_n=lbase(c_req_waddr); m_woff_n=o; m_wd_n=c_req_wdata; m_rop_n=c_req_op; m_ramo_n=c_req_amo;
            m_issued_n=0; m_data_n=0; m_acks_n=0; d_val_n=0;
            // fill tstate selection helper
            unique case (c_req_op)
            COP_LOAD, COP_LR: begin
                if (hit) begin
                    creq_rdy_c=1; crd_c=wrd(data_q[ix],o);
                    if (c_req_op==COP_LR) begin rsv_v_n=1; rsv_l_n=lbase(c_req_waddr); rsv_val_n=wrd(data_q[ix],o); end
                end else begin
                    m_val_n=1; m_atom_n=AT_NONE;
                    if (occ) begin m_ts_n=T_EVICT; m_vlad_n=lbase({tag_q[ix],ix,{LINE_WORD_BITS{1'b0}}}); m_vst_n=cs; end
                    else m_ts_n=T_IS_D;
                end
            end
            COP_STORE: begin
                if (hit && cs==CMI_M) begin creq_rdy_c=1; cww_we=1; cww_idx=ix; cww_off=o; cww_val=c_req_wdata; if(rsv_v_q&&rsv_l_q==lbase(c_req_waddr))rsv_v_n=0; end
                else if (hit && cs==CMI_E) begin creq_rdy_c=1; cww_we=1; cww_idx=ix; cww_off=o; cww_val=c_req_wdata; cs_we=1; cs_idx=ix; cs_val=CMI_M; if(rsv_v_q&&rsv_l_q==lbase(c_req_waddr))rsv_v_n=0; end
                else if (hit && (cs==CMI_S||cs==CMI_O)) begin m_val_n=1; m_data_n=1; m_atom_n=AT_NONE; m_ts_n=(cs==CMI_O)?T_OM_A:T_SM_AD; end
                else begin
                    m_val_n=1; m_atom_n=AT_NONE;
                    if (occ) begin m_ts_n=T_EVICT; m_vlad_n=lbase({tag_q[ix],ix,{LINE_WORD_BITS{1'b0}}}); m_vst_n=cs; end
                    else m_ts_n=T_IM_AD;
                end
            end
            COP_SC: begin
                automatic logic resv=rsv_v_q && rsv_l_q==lbase(c_req_waddr);
                if (!resv) begin creq_rdy_c=1; csc_c=0; end
                else if (hit && wrable) begin creq_rdy_c=1; csc_c=1; cww_we=1; cww_idx=ix; cww_off=o; cww_val=c_req_wdata; cs_we=1; cs_idx=ix; cs_val=CMI_M; rsv_v_n=0; end
                else if (hit && (cs==CMI_S||cs==CMI_O)) begin m_val_n=1; m_data_n=1; m_atom_n=AT_SC; m_ts_n=(cs==CMI_O)?T_OM_A:T_SM_AD; end
                else begin creq_rdy_c=1; csc_c=0; rsv_v_n=0; end
            end
            COP_AMO: begin
                if (hit && wrable) begin
                    automatic logic [XLEN-1:0] old=wrd(data_q[ix],o);
                    creq_rdy_c=1; crd_c=old; cww_we=1; cww_idx=ix; cww_off=o; cww_val=amo(c_req_amo,old,c_req_wdata);
                    cs_we=1; cs_idx=ix; cs_val=CMI_M; if(rsv_v_q&&rsv_l_q==lbase(c_req_waddr))rsv_v_n=0;
                end
                else if (hit && (cs==CMI_S||cs==CMI_O)) begin m_val_n=1; m_data_n=1; m_atom_n=AT_AMO_ACQ; m_ts_n=(cs==CMI_O)?T_OM_A:T_SM_AD; end
                else begin
                    m_val_n=1; m_atom_n=AT_AMO_ACQ;
                    if (occ) begin m_ts_n=T_EVICT; m_vlad_n=lbase({tag_q[ix],ix,{LINE_WORD_BITS{1'b0}}}); m_vst_n=cs; end
                    else m_ts_n=T_IM_AD;
                end
            end
            default: creq_rdy_c=1;
            endcase
        end

        // after an evict completes (ack handler set m_ts_n=T_IS_D as a placeholder), pick the real fill tstate
        if (m_val_q && m_ts_q==T_EVICT && ack_valid && ack_msg.op==OP_ACK && m_issued_q) begin
            // choose fill op from the saved core op
            unique case (m_rop_q)
                COP_LOAD, COP_LR: m_ts_n=T_IS_D;
                default:          m_ts_n=T_IM_AD;   // store/amo/sc-miss
            endcase
        end
    end

    always_ff @(posedge clk or negedge rst_l) begin
        if (!rst_l) begin
            m_val_q<=0; m_issued_q<=0; d_val_q<=0; rsv_v_q<=0; sr_pend_q<=0; m_atom_q<=AT_NONE; m_ts_q<=T_NONE;
            for (int s=0;s<SETS;s++) begin state_q[s]<=CMI_I; tag_q[s]<='0; end
        end else begin
            m_val_q<=m_val_n; m_ts_q<=m_ts_n; m_lad_q<=m_lad_n; m_vlad_q<=m_vlad_n; m_vst_q<=m_vst_n;
            m_acks_q<=m_acks_n; m_data_q<=m_data_n; m_rop_q<=m_rop_n; m_ramo_q<=m_ramo_n;
            m_woff_q<=m_woff_n; m_wd_q<=m_wd_n; m_atom_q<=m_atom_n; m_issued_q<=m_issued_n; m_unon_q<=m_unon_n; m_uwb_q<=m_uwb_n;
            d_val_q<=d_val_n; d_op_q<=d_op_n; d_req_q<=d_req_n; d_acks_q<=d_acks_n;
            rsv_v_q<=rsv_v_n; rsv_l_q<=rsv_l_n; rsv_val_q<=rsv_val_n;
            sr_pend_q<=sr_pend_n; sr_msg_q<=sr_msg_n; sr_dst_q<=sr_dst_n;
            if (cw_we) begin state_q[cw_idx]<=cw_st; tag_q[cw_idx]<=cw_tag; data_q[cw_idx]<=cw_line; end
            else if (cs_we) state_q[cs_idx]<=cs_val;
            // DLK-1: the snoop FSM has its own state-write port; apply it independently of the
            // demand-side write unless they target the SAME set (demand write wins that collision).
            if (css_we && !(cw_we && css_idx==cw_idx) && !(cs_we && !cw_we && css_idx==cs_idx))
                state_q[css_idx]<=css_val;
            if (cww_we) data_q[cww_idx][cww_off*XLEN +: XLEN]<=cww_val;
        end
    end

    assign dmd_valid=dmd_v_c; assign dmd_msg=dmd_m_c;
    assign c_req_ready=creq_rdy_c; assign c_resp_rdata=crd_c; assign c_resp_sc_ok=csc_c;
    assign snoop_ready=snoop_rdy_c; assign resp_ready=resp_rdy_c; assign ack_ready=ack_rdy_c;
    /* verilator lint_on ENUMVALUE */
endmodule
`default_nettype wire
