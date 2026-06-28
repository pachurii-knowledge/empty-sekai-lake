/**
 * niigo_dir_gg.sv  --  M3c GRANT-AND-GO MOESI directory / coherence point
 *
 * The verified grant-and-go directory (plans/multicore-ccd.md §3; formal/moesi_ccd_v2.m + v3.m).
 * Replaces the M1 dir-mediated/blocking niigo_dir.sv. Fabric-AGNOSTIC: it speaks clean message
 * channels (req/unblk/wb in, one msg+dst out) so the SAME module runs over the direct test
 * interconnect (protocol validation) and the wheel hub-funnel (fabric validation).
 *
 * Grant-and-go (§3.3): an L2/memory-sourced grant (GetS/GetM to DIR_I/DIR_S) commits the new
 * stable directory state and returns to ready IN THE SAME STEP -- no transient, no wait. The
 * requester collects its own Inv-Acks (D6 ack-to-requester); the dir never counts acks. Only a
 * cache-to-cache forward (GetS/GetM to a peer owner) or an Upgrade goes transient (DIR_S_D/DIR_M_D)
 * and waits for the requester's UNBLOCK to finalize. ownerNext (R-a) is resolved from the UNBLOCK's
 * onext. Lost-copy Upgrade (R-b): requester not a sharer -> served as GetM. Per-line serialised
 * (one busy line; a same-line request while busy chains); grant-and-go grants are NOT busy.
 *
 * Backing store = memory over the NMI bus (NINE; no L2 data array -- always data_present).
 */
`include "niigo_mem.vh"
`include "niigo_cmi.vh"
`include "niigo_ccd_m1.vh"
`default_nettype none
module niigo_dir_gg
    import RISCV_UArch::MEMORY_ADDR_WIDTH;
    import NIIGO_Mem::*;
    import NIIGO_CMI::*;
    import NIIGO_CCD_M1::*;
#(
    parameter int CORES    = NIIGO_CMI::NUM_CORES,   // 4 (id widths match ccd_msg_t)
    parameter int DIR_SETS = 16,
    // S1 (plans/smp-4core-bug-surface.md): set-associative directory. With DIR_WAYS >= the active
    // core count the dir can track EVERY line each core caches in a given dir set without eviction
    // -- each core holds <=1 line per dir set (the dir index determines the L1 index), so at most
    // CORES distinct lines ever collide there. It is therefore inclusive and never silently forgets
    // a live owner/sharer on an aliasing admit; the old direct-mapped (1-way) dir dropped the
    // resident entry whenever a different-tag line hit the same set -> stale read / lost dirty WB.
    parameter int DIR_WAYS = NIIGO_CMI::NUM_CORES
)(
    input  wire logic clk,
    input  wire logic rst_l,
    // ---- inbound message channels (class-separated; the fabric demuxes by VC) ----
    input  wire logic      req_valid,    input  wire ccd_msg_t req_msg,    output logic req_ready,   // C0
    input  wire logic      unblk_valid,  input  wire ccd_msg_t unblk_msg,  output logic unblk_ready, // C4 UNBLOCK
    input  wire logic      wb_valid,     input  wire ccd_msg_t wb_msg,     output logic wb_ready,    // C3 WB_DATA
    // ---- outbound message channel (one fwd/inv/data/ack at a time, addressed to a core) ----
    output logic                     out_valid,
    output ccd_msg_t                 out_msg,
    output logic [NODE_ID_W-1:0]     out_dst,    // destination core node id
    input  wire logic                out_ready,
    // ---- backing memory (NMI master) ----
    output nmi_req_t   mem_req_o,
    input  wire logic  mem_req_ready_i,
    input  nmi_resp_t  mem_resp_i
);
    localparam int CW   = CORE_ID_W;                       // 2
    localparam int LWB  = NIIGO_Mem::LINE_WORD_BITS;
    localparam int DIDX = $clog2(DIR_SETS);
    localparam int DTAG = MEMORY_ADDR_WIDTH - DIDX - LWB;
    localparam int WW   = (DIR_WAYS<=1) ? 1 : $clog2(DIR_WAYS);   // way-index width
    /* verilator lint_off ENUMVALUE */

    function automatic logic [DIDX-1:0] didx(input logic [MEMORY_ADDR_WIDTH-1:0] la);
        didx = la[LWB +: DIDX];
    endfunction
    function automatic logic [DTAG-1:0] dtag(input logic [MEMORY_ADDR_WIDTH-1:0] la);
        dtag = la[MEMORY_ADDR_WIDTH-1 : LWB+DIDX];
    endfunction
    function automatic logic [NODE_ID_W-1:0] cnode(input logic [CW-1:0] c);
        cnode = {{(NODE_ID_W-CW){1'b0}}, c};
    endfunction
    function automatic logic [ACK_CNT_W-1:0] nother(input logic [CORES-1:0] sh, input logic [CW-1:0] r);
        logic [ACK_CNT_W-1:0] n; n='0;
        for (int c=0;c<CORES;c++) if (sh[c] && (c[CW-1:0]!=r)) n=n+1'b1;
        return n;
    endfunction

    // ---- stable directory entry (NINE: miss => DIR_I) ----
    typedef struct packed {
        logic              valid;
        logic [DTAG-1:0]   tag;
        dir_state_e        dstate;
        logic [CORES-1:0]  sharers;   // d_sharers
        logic [CW-1:0]     owner;     // valid in EM/O
    } dent_t;

    // ---- per-line busy tracker (single in this bring-up; multi-line MSHR file is a later step) ----
    typedef enum logic [1:0] { K_NONE, K_SD, K_MD, K_DRAINPUT } bkind_e;

    // ---- FSM for the outbound-message sequencer (drives fwd/inv/data/ack one at a time) ----
    typedef enum logic [2:0] { S_IDLE, S_MEMRD, S_MEMWR, S_EMIT, S_INVSEQ, S_DRAINACK } seq_e;

    dent_t              dir_q [DIR_SETS][DIR_WAYS];
    seq_e               st_q;
    // busy tracker
    logic               busy_q;
    bkind_e             bkind_q;
    logic [MEMORY_ADDR_WIDTH-1:0] blad_q;
    logic [CW-1:0]      breq_q, bowner_q;
    logic               brefresh_pend_q;       // S_D ON_S/ON_I: awaiting paired WB_DATA
    logic [LINE_BITS-1:0] brefresh_val_q;
    logic               bwb_seen_q, bub_seen_q; // paired arrivals for S_D refresh
    cmi_owner_next_e    bub_onext_q;
    // pending emission (the message to send out, possibly multi: INV fan then DATA)
    logic               em_data_pend_q;        // a DATA grant to emit after INVs
    ccd_msg_t           em_data_msg_q;  logic [NODE_ID_W-1:0] em_data_dst_q;
    logic [CORES-1:0]   inv_todo_q;            // remaining sharers to INV
    logic [CW-1:0]      inv_req_q;             // requester to carry on INV
    logic [MEMORY_ADDR_WIDTH-1:0] inv_lad_q;
    // single pending out msg (for memrd/memwr completion path)
    ccd_msg_t           gmsg_q;  logic [NODE_ID_W-1:0] gdst_q;
    logic [LINE_BITS-1:0] gline_q;
    // committed stable-update staged for the busy line's finalize
    dir_state_e         nds_q;  logic [CORES-1:0] nsh_q;  logic [CW-1:0] nown_q;  logic nset_q;
    logic [DIDX-1:0]    bidx_q;
    logic [WW-1:0]      bway_q;     // way of the busy line (set-associative)

    // next-state
    dent_t dir_wval; logic dir_we; logic [DIDX-1:0] dir_widx; logic [WW-1:0] dir_wway;
    // (the bulk of next-state is held in the same-named *_n below)

    // ---- current stable lookup for a given laddr (search all ways; miss => DIR_I) ----
    function automatic dent_t look(input logic [MEMORY_ADDR_WIDTH-1:0] la);
        dent_t e; e='{default:'0}; e.dstate=DIR_I;
        for (int w=0; w<DIR_WAYS; w++)
            if (dir_q[didx(la)][w].valid && dir_q[didx(la)][w].tag==dtag(la)) e=dir_q[didx(la)][w];
        return e;
    endfunction
    // way to write for `la`: the matching-tag way if resident, else the lowest invalid way. With
    // DIR_WAYS>=CORES a free way always exists for a new line (<=CORES lines per dir set), so the
    // '0 fallback is unreachable in correct operation (would be an inclusion overflow).
    function automatic logic [WW-1:0] look_way(input logic [MEMORY_ADDR_WIDTH-1:0] la);
        logic [WW-1:0] mw, fw; logic found_m, found_f;
        mw='0; fw='0; found_m=1'b0; found_f=1'b0;
        for (int w=DIR_WAYS-1; w>=0; w--) begin
            if (dir_q[didx(la)][w].valid && dir_q[didx(la)][w].tag==dtag(la)) begin mw=w[WW-1:0]; found_m=1'b1; end
            if (!dir_q[didx(la)][w].valid) begin fw=w[WW-1:0]; found_f=1'b1; end
        end
        return found_m ? mw : (found_f ? fw : '0);
    endfunction

    // ====================================================================
    // Combinational outputs + next-state
    // ====================================================================
    seq_e               st_n;
    logic               busy_n;  bkind_e bkind_n;
    logic [MEMORY_ADDR_WIDTH-1:0] blad_n;
    logic [CW-1:0]      breq_n, bowner_n;
    logic               brefresh_pend_n; logic [LINE_BITS-1:0] brefresh_val_n;
    logic               bwb_seen_n, bub_seen_n; cmi_owner_next_e bub_onext_n;
    logic               em_data_pend_n; ccd_msg_t em_data_msg_n; logic [NODE_ID_W-1:0] em_data_dst_n;
    logic [CORES-1:0]   inv_todo_n; logic [CW-1:0] inv_req_n; logic [MEMORY_ADDR_WIDTH-1:0] inv_lad_n;
    ccd_msg_t           gmsg_n; logic [NODE_ID_W-1:0] gdst_n; logic [LINE_BITS-1:0] gline_n;
    dir_state_e         nds_n; logic [CORES-1:0] nsh_n; logic [CW-1:0] nown_n; logic nset_n;
    logic [DIDX-1:0]    bidx_n;
    logic [WW-1:0]      bway_n;

    nmi_req_t mem_req_c;
    logic     ov_c; ccd_msg_t om_c; logic [NODE_ID_W-1:0] od_c;
    logic     req_rdy_c, unblk_rdy_c, wb_rdy_c;

    // helper: build an outbound message
    function automatic ccd_msg_t mkmsg(input cmi_op_e op, input logic [CW-1:0] req,
                                       input cmi_state_e gst, input logic [ACK_CNT_W-1:0] acks,
                                       input logic [LINE_BITS-1:0] line,
                                       input logic [MEMORY_ADDR_WIDTH-1:0] la);
        ccd_msg_t m; m='{default:'0};
        m.op=op; m.req=req; m.gst=gst; m.acks=acks; m.line=line; m.laddr=la;
        return m;
    endfunction

    // pick first set bit of a CORES-vector (returns index, valid only if any)
    function automatic logic [CW-1:0] first_bit(input logic [CORES-1:0] v);
        logic [CW-1:0] idx; idx='0;
        for (int c=CORES-1;c>=0;c--) if (v[c]) idx=c[CW-1:0];
        return idx;
    endfunction

    always_comb begin
        // hold
        st_n=st_q; busy_n=busy_q; bkind_n=bkind_q; blad_n=blad_q; breq_n=breq_q; bowner_n=bowner_q;
        brefresh_pend_n=brefresh_pend_q; brefresh_val_n=brefresh_val_q;
        bwb_seen_n=bwb_seen_q; bub_seen_n=bub_seen_q; bub_onext_n=bub_onext_q;
        em_data_pend_n=em_data_pend_q; em_data_msg_n=em_data_msg_q; em_data_dst_n=em_data_dst_q;
        inv_todo_n=inv_todo_q; inv_req_n=inv_req_q; inv_lad_n=inv_lad_q;
        gmsg_n=gmsg_q; gdst_n=gdst_q; gline_n=gline_q;
        nds_n=nds_q; nsh_n=nsh_q; nown_n=nown_q; nset_n=nset_q; bidx_n=bidx_q; bway_n=bway_q;
        dir_we=1'b0; dir_widx=bidx_q; dir_wway=bway_q; dir_wval='{default:'0};
        mem_req_c='{default:'0};
        ov_c=1'b0; om_c='{default:'0}; od_c='0;
        req_rdy_c=1'b0; unblk_rdy_c=1'b0; wb_rdy_c=1'b0;

        unique case (st_q)
        // =====================================================================
        S_IDLE: begin
            // ---- finalize outranks admit: WB sink, then UNBLOCK, then new request ----
            // (1) WB_DATA: eviction drain OR S_D refresh pairing
            if (wb_valid) begin
                wb_rdy_c = 1'b1;
                // match the busy transaction by REQUESTER (src==breq_q) AND line, not line alone:
                // a stale same-line WB from a different core must not pair with this transaction.
                if (busy_q && bkind_q==K_DRAINPUT && wb_msg.laddr==blad_q && wb_msg.src==breq_q) begin
                    gline_n = wb_msg.line; st_n = S_MEMWR;             // eviction writeback -> memory
                end else if (busy_q && bkind_q==K_SD && wb_msg.laddr==blad_q && wb_msg.src==breq_q) begin
                    brefresh_val_n = wb_msg.line; bwb_seen_n = 1'b1;  // pair for the S_D refresh
                end
                // (stray WB to a non-busy / wrong-requester line: accept + drop)
            end
            // (2) UNBLOCK finalize -- must come from THIS transaction's requester (breq_q), else a
            //     stale same-line UNBLOCK from another core (e.g. a delayed grant-and-go UNBLOCK)
            //     would corrupt the finalize (wrong onext / premature).
            else if (unblk_valid) begin
                unblk_rdy_c = 1'b1;
                if (busy_q && unblk_msg.laddr==blad_q && unblk_msg.src==breq_q) begin
                    bub_seen_n  = 1'b1;
                    bub_onext_n = unblk_msg.onext;
                end
            end
            // ---- complete a busy S_D/M_D finalize once its conditions are met ----
            // (handled below, after admit, so a same-cycle UNBLOCK is registered first)

            // ---- admit a new request only if not busy (per-line serialise) ----
            else if (req_valid) begin
                automatic ccd_msg_t   m  = req_msg;
                automatic logic [CW-1:0] R = m.src;
                automatic dent_t      e  = look(m.laddr);
                automatic logic [CORES-1:0] bitR = '0;
                automatic logic       same_busy = busy_q && (m.laddr==blad_q);
                bitR[R] = 1'b1;
                if (!busy_q && !same_busy) begin
                    req_rdy_c = 1'b1;
                    bidx_n = didx(m.laddr);
                    bway_n = look_way(m.laddr);   // matching-tag way if resident, else a free way
                    unique case (m.op)
                    // ---------- GetS ----------
                    OP_GETS: begin
                        if (e.dstate==DIR_I) begin
                            // grant-and-go: Data(E), -> DIR_EM(owner=R)
                            gmsg_n = mkmsg(OP_DATA, R, CMI_E, '0, '0, m.laddr); gdst_n=cnode(R);
                            nds_n=DIR_EM; nsh_n=bitR; nown_n=R; nset_n=1'b1;
                            st_n = S_MEMRD;   // fetch line from memory, then emit
                        end else if (e.dstate==DIR_S) begin
                            gmsg_n = mkmsg(OP_DATA, R, CMI_S, '0, '0, m.laddr); gdst_n=cnode(R);
                            nds_n=DIR_S; nsh_n=e.sharers|bitR; nown_n=e.owner; nset_n=1'b0;
                            st_n = S_MEMRD;
                        end else begin
                            // DIR_EM/DIR_O: cache-to-cache -> FwdGetS to owner, go busy S_D
                            gmsg_n = mkmsg(OP_FWD_GETS, R, CMI_I, '0, '0, m.laddr); gdst_n=cnode(e.owner);
                            busy_n=1'b1; bkind_n=K_SD; blad_n=m.laddr; breq_n=R; bowner_n=e.owner;
                            brefresh_pend_n=1'b0; bwb_seen_n=1'b0; bub_seen_n=1'b0;
                            st_n = S_EMIT;
                        end
                    end
                    // ---------- GetM ----------
                    OP_GETM: begin
                        if (e.dstate==DIR_I) begin
                            gmsg_n = mkmsg(OP_DATA, R, CMI_M, '0, '0, m.laddr); gdst_n=cnode(R);
                            nds_n=DIR_EM; nsh_n=bitR; nown_n=R; nset_n=1'b1;
                            st_n = S_MEMRD;
                        end else if (e.dstate==DIR_S) begin
                            // grant-and-go w/ ack-to-requester: INV sharers, Data(M, acks=nother)
                            inv_todo_n = e.sharers & ~bitR; inv_req_n=R; inv_lad_n=m.laddr;
                            em_data_msg_n = mkmsg(OP_DATA, R, CMI_M, nother(e.sharers,R), '0, m.laddr);
                            em_data_dst_n = cnode(R); em_data_pend_n=1'b1;
                            nds_n=DIR_EM; nsh_n=bitR; nown_n=R; nset_n=1'b1;
                            st_n = S_INVSEQ;   // (S_INVSEQ then memrd-data for the grant)
                        end else begin
                            // DIR_EM/DIR_O: cache-to-cache write -> FwdGetM to owner, go busy M_D
                            automatic logic [CORES-1:0] others = e.sharers & ~bitR;
                            others[e.owner]=1'b0;
                            gmsg_n = mkmsg(OP_FWD_GETM, R, CMI_M, nother(e.sharers & ~(CORES'(1)<<e.owner), R), '0, m.laddr);
                            gdst_n = cnode(e.owner);
                            inv_todo_n = others; inv_req_n=R; inv_lad_n=m.laddr;
                            busy_n=1'b1; bkind_n=K_MD; blad_n=m.laddr; breq_n=R;
                            st_n = (others!=0) ? S_INVSEQ : S_EMIT;
                        end
                    end
                    // ---------- Upgrade ----------
                    OP_UPGRADE: begin
                        automatic logic R_is_sharer = e.sharers[R];
                        if ((e.dstate==DIR_S || e.dstate==DIR_O) && R_is_sharer) begin
                            // no-data grant: INV others, Data(M,acks) don't-care payload
                            automatic logic [CORES-1:0] others = e.sharers & ~bitR;
                            inv_todo_n = others; inv_req_n=R; inv_lad_n=m.laddr;
                            em_data_msg_n = mkmsg(OP_DATA, R, CMI_M, nother(e.sharers,R), '0, m.laddr);
                            em_data_dst_n = cnode(R); em_data_pend_n=1'b1;
                            busy_n=1'b1; bkind_n=K_MD; blad_n=m.laddr; breq_n=R;
                            nds_n=DIR_EM; nsh_n=bitR; nown_n=R; nset_n=1'b1;
                            st_n = S_INVSEQ;   // S_INVSEQ emits the no-data grant even when others==0
                        end else begin
                            // lost-copy (R-b): serve as GetM
                            if (e.dstate==DIR_I) begin
                                gmsg_n=mkmsg(OP_DATA,R,CMI_M,'0,'0,m.laddr); gdst_n=cnode(R);
                                nds_n=DIR_EM; nsh_n=bitR; nown_n=R; nset_n=1'b1; st_n=S_MEMRD;
                            end else if (e.dstate==DIR_S) begin
                                inv_todo_n=e.sharers & ~bitR; inv_req_n=R; inv_lad_n=m.laddr;
                                em_data_msg_n=mkmsg(OP_DATA,R,CMI_M,nother(e.sharers,R),'0,m.laddr);
                                em_data_dst_n=cnode(R); em_data_pend_n=1'b1;
                                nds_n=DIR_EM; nsh_n=bitR; nown_n=R; nset_n=1'b1; st_n=S_INVSEQ;
                            end else begin // EM/O
                                automatic logic [CORES-1:0] others2 = e.sharers & ~bitR; others2[e.owner]=1'b0;
                                gmsg_n=mkmsg(OP_FWD_GETM,R,CMI_M,nother(e.sharers & ~(CORES'(1)<<e.owner),R),'0,m.laddr);
                                gdst_n=cnode(e.owner);
                                inv_todo_n=others2; inv_req_n=R; inv_lad_n=m.laddr;
                                busy_n=1'b1; bkind_n=K_MD; blad_n=m.laddr; breq_n=R;
                                st_n=(others2!=0)?S_INVSEQ:S_EMIT;
                            end
                        end
                    end
                    // ---------- PutM / PutO ----------
                    OP_PUTM, OP_PUTO: begin
                        if ((e.dstate==DIR_EM || e.dstate==DIR_O) && e.owner==R) begin
                            // dirty line is embedded in the PutM/PutO -> write it back to memory
                            gline_n = m.line; bidx_n = didx(m.laddr);
                            busy_n=1'b1; bkind_n=K_DRAINPUT; blad_n=m.laddr; breq_n=R;
                            // S3 (plans/smp-4core-bug-surface.md): honor remaining sharers for PUTM too,
                            // not just PUTO. A concurrent FwdGetS that demoted this owner M->O creates a
                            // sharer the evictor's captured m_vst doesn't reflect (it still sends PUTM);
                            // forcing DIR_I would DROP that live sharer and a later writer would grant M
                            // with no INV -> stale copy. The WB makes memory current, so the surviving
                            // sharer's clean copy stays consistent. Constant-folds to DIR_I single-core.
                            nds_n = ((e.sharers & ~bitR)!=0) ? DIR_S : DIR_I;
                            nsh_n = e.sharers & ~bitR; nown_n=e.owner; nset_n=1'b0;
                            st_n = S_MEMWR;
                        end else begin
                            // superseded: ownership moved. drop (no WB will be matched) + ack.
                            gmsg_n=mkmsg(OP_ACK,R,CMI_I,'0,'0,m.laddr); gdst_n=cnode(R);
                            nds_n=e.dstate; nsh_n=e.sharers & ~bitR; nown_n=e.owner; nset_n=1'b0;
                            st_n=S_EMIT;   // (the matching WB_DATA, if any, is dropped in S_IDLE)
                        end
                    end
                    // ---------- PutE / PutS (clean, no line) ----------
                    // DIR-1 fix: a clean Put only clears R's sharer bit; it must NOT demote a line
                    // that has a dirty owner (DIR_O, or DIR_EM owned by some core != R) -- that would
                    // drop the owner and a later cold reader would get stale memory (v3 PutE/PutS;
                    // I1/I2/I5). Only DIR_EM&&owner==R (PutE of the sole copy) or the last DIR_S
                    // sharer leaving collapses the line to DIR_I; everything else is unchanged.
                    OP_PUTE, OP_PUTS: begin
                        automatic logic [CORES-1:0] sh2 = e.sharers & ~bitR;
                        gmsg_n=mkmsg(OP_ACK,R,CMI_I,'0,'0,m.laddr); gdst_n=cnode(R);
                        if (e.dstate==DIR_EM && e.owner==R)    nds_n=DIR_I;   // sole clean owner left
                        else if (e.dstate==DIR_S && sh2==0)    nds_n=DIR_I;   // last shared sharer left
                        else                                   nds_n=e.dstate;// DIR_O / DIR_EM(other) unchanged
                        nsh_n=sh2; nown_n=e.owner; nset_n=1'b0;
                        st_n=S_EMIT;
                    end
                    default: req_rdy_c=1'b1; // drop unknown
                    endcase
                end
            end

            // ---- finalize a busy line whose conditions are now met (lower priority than consume) ----
            if (st_n==S_IDLE && busy_q && (bkind_q==K_SD || bkind_q==K_MD)) begin
                automatic dent_t eb = look(blad_q);
                automatic logic [CORES-1:0] bitR='0; bitR[breq_q]=1'b1;
                if (bkind_q==K_MD && bub_seen_q) begin
                    // GetM/Upgrade cache-to-cache finalize: requester -> M (EM owner=R)
                    dir_we=1'b1; dir_widx=didx(blad_q);
                    dir_wval.valid=1'b1; dir_wval.tag=dtag(blad_q);
                    dir_wval.dstate=DIR_EM; dir_wval.sharers=bitR; dir_wval.owner=breq_q;
                    busy_n=1'b0; bkind_n=K_NONE; bub_seen_n=1'b0;
                end else if (bkind_q==K_SD && bub_seen_q) begin
                    unique case (bub_onext_q)
                    ON_O: begin // owner keeps dirty + ownership (M->O / O stays O)
                        dir_we=1'b1; dir_widx=didx(blad_q);
                        dir_wval.valid=1'b1; dir_wval.tag=dtag(blad_q);
                        dir_wval.dstate=DIR_O; dir_wval.sharers=eb.sharers|bitR; dir_wval.owner=bowner_q;
                        busy_n=1'b0; bkind_n=K_NONE; bub_seen_n=1'b0;
                    end
                    default: begin // ON_S / ON_I: need the paired WB_DATA to refresh memory first
                        if (bwb_seen_q) begin
                            gline_n = brefresh_val_q;
                            // commit DIR_S (clear leaving owner on ON_I), refresh memory
                            nds_n=DIR_S;
                            nsh_n = (bub_onext_q==ON_I) ? ((eb.sharers & ~(CORES'(1)<<bowner_q))|bitR)
                                                        : (eb.sharers|bitR);
                            nown_n=eb.owner; nset_n=1'b0; bidx_n=didx(blad_q);
                            st_n = S_MEMWR;     // write refresh_val to memory, then finalize-commit
                        end
                        // else: still awaiting the paired WB_DATA; stay busy
                    end
                    endcase
                end
            end
        end

        // =====================================================================
        // S_MEMRD: read the line for an L2-sourced grant, then emit the DATA
        S_MEMRD: begin
            mem_req_c.valid=1'b1; mem_req_c.op=NMI_RD_LINE; mem_req_c.waddr=gmsg_q.laddr;
            if (mem_resp_i.valid) begin gline_n=mem_resp_i.rdata; st_n=S_EMIT; end
        end
        // S_MEMWR: write a line to memory (eviction drain OR S_D refresh), then finalize+ack
        S_MEMWR: begin
            mem_req_c.valid=1'b1; mem_req_c.op=NMI_WR_LINE; mem_req_c.waddr=blad_q; mem_req_c.wdata=gline_q;
            if (mem_resp_i.valid) begin
                // commit the staged stable update
                dir_we=1'b1; dir_widx=bidx_q;
                dir_wval.valid=(nds_q!=DIR_I); dir_wval.tag=dtag(blad_q);
                dir_wval.dstate=nds_q; dir_wval.sharers=nsh_q; dir_wval.owner=nown_q;
                if (bkind_q==K_DRAINPUT) begin
                    st_n=S_DRAINACK;   // ack the evictor (no re-commit), then clear busy
                end else begin
                    // S_D refresh finalize -> done
                    busy_n=1'b0; bkind_n=K_NONE; bub_seen_n=1'b0; bwb_seen_n=1'b0; st_n=S_IDLE;
                end
            end
        end

        // S_DRAINACK: send WB_ACK to the evictor, then release the busy line
        S_DRAINACK: begin
            ov_c=1'b1; om_c=mkmsg(OP_ACK,breq_q,CMI_I,'0,'0,blad_q); od_c=cnode(breq_q);
            if (out_ready) begin busy_n=1'b0; bkind_n=K_NONE; st_n=S_IDLE; end
        end

        // =====================================================================
        // S_INVSEQ: fan INV to each remaining sharer (one per cycle, carrying req)
        S_INVSEQ: begin
            if (inv_todo_q != 0) begin
                automatic logic [CW-1:0] t = first_bit(inv_todo_q);
                ov_c=1'b1; od_c=cnode(t);
                om_c=mkmsg(OP_INV, inv_req_q, CMI_I, '0, '0, inv_lad_q);
                if (out_ready) inv_todo_n = inv_todo_q & ~(CORES'(1)<<t);
            end else begin
                // INVs done: emit the pending DATA (if any) via memrd, else finalize/emit
                if (em_data_pend_q) begin
                    // need the line for a real Data grant (GetM-from-S / lost-Upgrade-from-S);
                    // a no-data Upgrade grant also rides OP_DATA (payload don't-care) -> still memrd ok
                    gmsg_n = em_data_msg_q; gdst_n = em_data_dst_q; em_data_pend_n=1'b0;
                    // commit the staged stable update now (grant-and-go) if not busy(M_D)
                    if (!busy_q) begin
                        dir_we=1'b1; dir_widx=bidx_q;
                        dir_wval.valid=1'b1; dir_wval.tag=dtag(em_data_msg_q.laddr);
                        dir_wval.dstate=nds_q; dir_wval.sharers=nsh_q; dir_wval.owner=nown_q;
                    end
                    st_n=S_MEMRD;
                end else begin
                    // M_D cache-to-cache (owner forwards data): just go EMIT the FwdGetM already done?
                    // FwdGetM for M_D was emitted via gmsg before S_INVSEQ? No -- emit it now:
                    st_n=S_EMIT;
                end
            end
        end

        // =====================================================================
        // S_EMIT: drive the single staged outbound message until accepted
        S_EMIT: begin
            ov_c=1'b1; om_c=gmsg_q; od_c=gdst_q;
            if (gmsg_q.op==OP_DATA) om_c.line=gline_q;   // L2-sourced data carries the memory line
            if (out_ready) begin
                // for grant-and-go L2 grants, commit the stable update here
                if (!busy_q && (gmsg_q.op==OP_DATA || gmsg_q.op==OP_ACK)) begin
                    dir_we=1'b1; dir_widx=bidx_q;
                    dir_wval.valid=(nds_q!=DIR_I); dir_wval.tag=dtag(gmsg_q.laddr);
                    dir_wval.dstate=nds_q; dir_wval.sharers=nsh_q; dir_wval.owner=nown_q;
                end
                st_n=S_IDLE;
            end
        end
        default: st_n=S_IDLE;
        endcase
    end

    // ====================================================================
    always_ff @(posedge clk or negedge rst_l) begin
        if (!rst_l) begin
            st_q<=S_IDLE; busy_q<=1'b0; bkind_q<=K_NONE; brefresh_pend_q<=1'b0;
            bwb_seen_q<=1'b0; bub_seen_q<=1'b0; em_data_pend_q<=1'b0; inv_todo_q<='0;
            for (int s=0;s<DIR_SETS;s++) for (int w=0;w<DIR_WAYS;w++) dir_q[s][w]<='{default:'0};
        end else begin
            st_q<=st_n; busy_q<=busy_n; bkind_q<=bkind_n; blad_q<=blad_n; breq_q<=breq_n; bowner_q<=bowner_n;
            brefresh_pend_q<=brefresh_pend_n; brefresh_val_q<=brefresh_val_n;
            bwb_seen_q<=bwb_seen_n; bub_seen_q<=bub_seen_n; bub_onext_q<=bub_onext_n;
            em_data_pend_q<=em_data_pend_n; em_data_msg_q<=em_data_msg_n; em_data_dst_q<=em_data_dst_n;
            inv_todo_q<=inv_todo_n; inv_req_q<=inv_req_n; inv_lad_q<=inv_lad_n;
            gmsg_q<=gmsg_n; gdst_q<=gdst_n; gline_q<=gline_n;
            nds_q<=nds_n; nsh_q<=nsh_n; nown_q<=nown_n; nset_q<=nset_n; bidx_q<=bidx_n; bway_q<=bway_n;
            if (dir_we) dir_q[dir_widx][dir_wway]<=dir_wval;
        end
    end

    assign mem_req_o   = mem_req_c;
    assign out_valid   = ov_c;
    assign out_msg     = om_c;
    assign out_dst     = od_c;
    assign req_ready   = req_rdy_c;
    assign unblk_ready = unblk_rdy_c;
    assign wb_ready    = wb_rdy_c;

    // S1 inclusion guards (sim-only): the set-associative directory is inclusive ONLY while
    // DIR_WAYS >= the active core count AND the dir index is finer-or-equal to the L1 index (so a
    // core holds <=1 line per dir set). If either is violated, look_way()'s '0 fallback would
    // silently clobber way 0 -- re-introducing the exact stale-read/lost-WB class S1 killed. Make
    // both failure modes LOUD instead of silent (no functional effect when the invariant holds).
`ifndef SYNTHESIS
    initial if (DIR_WAYS < CORES)
        $fatal(1, "niigo_dir_gg: DIR_WAYS(%0d) < CORES(%0d) -- directory not inclusive (S1 regression)", DIR_WAYS, CORES);
    always_ff @(posedge clk) if (rst_l && dir_we && dir_wval.valid &&
        dir_q[dir_widx][dir_wway].valid && dir_q[dir_widx][dir_wway].tag != dir_wval.tag)
        $error("niigo_dir_gg: dir set %0d way %0d clobbers a live different-tag line -- inclusion overflow (S1)",
               dir_widx, dir_wway);
`endif
    /* verilator lint_on ENUMVALUE */
endmodule
`default_nettype wire
