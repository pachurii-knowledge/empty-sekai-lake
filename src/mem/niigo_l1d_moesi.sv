/**
 * niigo_l1d_moesi.sv  --  M1 dual-core L1D MOESI coherence agent
 *
 * A private write-back L1D with the full MOESI state machine + snoop FSM, the L1D side of
 * the M1 CCD (plans/multicore-ccd.md §2/§9). Talks to its core over a simplified word-granular
 * LSQ port (load/store/LR/SC/AMO) and to the directory (niigo_dir.sv) over the M1 full-line CMI
 * link (niigo_ccd_m1.vh). Correctness-first to match the directory: BLOCKING — one outstanding
 * coherence transaction at a time (like l1_dcache). Because the directory is serialised, a
 * requester in an acquire/evict transient never receives a conflicting snoop, so the deferred-
 * snoop matrix + snoop-drain-before-reissue (v2/v3, grant-and-go) and the op+is_icache routing
 * (v4a, L1I) are M3/v4c concerns, NOT needed here. What M1 DOES implement:
 *   - MOESI {I,S,E,O,M}; cache-to-cache forwarding as the owner.
 *   - R-a 3-valued owner-next on a FwdGetS: M->O (ON_O), E->S (ON_S), O->O (ON_O).
 *   - LR/SC reservation with the coherence-kill (a remote FwdGetM/Inv to the reserved line
 *     clears the reservation; an own store/AMO/eviction clears it) — formal v4b.
 *   - AMO as acquire-M-then-RMW (atomic here because blocking+serialised; the snoop-squash-
 *     replay of formal v4b/§13.9c is only needed under grant-and-go, M3).
 *
 * Two-process FSM. Snoops (in A_IDLE) take priority over starting a new core request.
 */
`include "niigo_mem.vh"
`include "niigo_cmi.vh"
`include "niigo_ccd_m1.vh"
`default_nettype none
module niigo_l1d_moesi
    import RISCV_ISA::XLEN;
    import RISCV_UArch::MEMORY_ADDR_WIDTH;
    import NIIGO_Mem::LINE_BITS, NIIGO_Mem::LINE_WORDS, NIIGO_Mem::LINE_WORD_BITS;
    import NIIGO_CMI::cmi_state_e, NIIGO_CMI::CMI_I, NIIGO_CMI::CMI_S, NIIGO_CMI::CMI_E,
           NIIGO_CMI::CMI_O, NIIGO_CMI::CMI_M, NIIGO_CMI::CORE_ID_W,
           NIIGO_CMI::OP_GETS, NIIGO_CMI::OP_GETM, NIIGO_CMI::OP_UPGRADE,
           NIIGO_CMI::OP_PUTM, NIIGO_CMI::OP_PUTO, NIIGO_CMI::OP_PUTS, NIIGO_CMI::OP_PUTE,
           NIIGO_CMI::OP_FWD_GETS, NIIGO_CMI::OP_FWD_GETM, NIIGO_CMI::OP_INV,
           NIIGO_CMI::OP_DATA, NIIGO_CMI::OP_INV_ACK, NIIGO_CMI::OP_ACK;
    import NIIGO_CCD_M1::*;
#(
    parameter int unsigned CORE_ID = 0,
    parameter int          SETS    = 8
)(
    input  wire logic clk,
    input  wire logic rst_l,
    // ---- core LSQ port (blocking: hold the request until c_req_ready) ----
    input  wire logic                          c_req_valid,
    output logic                               c_req_ready,
    input  wire l1_core_op_e                   c_req_op,
    input  wire l1_amo_op_e                    c_req_amo,
    input  wire logic [MEMORY_ADDR_WIDTH-1:0]  c_req_waddr,   // WORD address
    input  wire logic [XLEN-1:0]               c_req_wdata,
    output logic [XLEN-1:0]                     c_resp_rdata,  // load / LR / AMO-old (valid when c_req_ready)
    output logic                               c_resp_sc_ok,  // SC success (valid when c_req_ready)
    // ---- CMI link to the directory ----
    output ccd_chan_t  up_o,
    input  wire logic  up_ready_i,
    input  wire ccd_chan_t down_i,
    output logic       down_ready_o
);
    localparam int IDX_W = $clog2(SETS);
    localparam int TAG_W = MEMORY_ADDR_WIDTH - IDX_W - LINE_WORD_BITS;
    /* verilator lint_off ENUMVALUE */           // '{default:'0}: 0 is a valid value for these enums

    function automatic logic [IDX_W-1:0] idxf(input logic [MEMORY_ADDR_WIDTH-1:0] wa);
        idxf = wa[LINE_WORD_BITS +: IDX_W];
    endfunction
    function automatic logic [TAG_W-1:0] tagf(input logic [MEMORY_ADDR_WIDTH-1:0] wa);
        tagf = wa[MEMORY_ADDR_WIDTH-1 : LINE_WORD_BITS + IDX_W];
    endfunction
    function automatic logic [LINE_WORD_BITS-1:0] offf(input logic [MEMORY_ADDR_WIDTH-1:0] wa);
        offf = wa[LINE_WORD_BITS-1:0];
    endfunction
    function automatic logic [MEMORY_ADDR_WIDTH-1:0] linebase(input logic [MEMORY_ADDR_WIDTH-1:0] wa);
        linebase = {wa[MEMORY_ADDR_WIDTH-1 : LINE_WORD_BITS], {LINE_WORD_BITS{1'b0}}};
    endfunction
    function automatic logic [XLEN-1:0] wordrd(input logic [LINE_BITS-1:0] line,
                                               input logic [LINE_WORD_BITS-1:0] off);
        wordrd = line[off*XLEN +: XLEN];
    endfunction
    function automatic logic [LINE_BITS-1:0] wordmerge(input logic [LINE_BITS-1:0] line,
                                                       input logic [LINE_WORD_BITS-1:0] off,
                                                       input logic [XLEN-1:0] w);
        wordmerge = line; wordmerge[off*XLEN +: XLEN] = w;
    endfunction
    function automatic logic [XLEN-1:0] amo_apply(input l1_amo_op_e a,
                                                  input logic [XLEN-1:0] old, input logic [XLEN-1:0] op);
        unique case (a)
            AMO_ADD:  amo_apply = old + op;
            AMO_SWAP: amo_apply = op;
            AMO_OR:   amo_apply = old | op;
            AMO_AND:  amo_apply = old & op;
            AMO_XOR:  amo_apply = old ^ op;
            default:  amo_apply = op;
        endcase
    endfunction

    typedef enum logic [2:0] { A_IDLE, A_WB, A_FILL, A_UPG } astate_e;

    // ---- cache + agent registers ----
    cmi_state_e            state_q [SETS];
    logic [TAG_W-1:0]      tag_q   [SETS];
    logic [LINE_BITS-1:0]  data_q  [SETS];
    astate_e               st_q;
    logic                  sent_q;        // outbound request/Put driven & accepted
    l1_core_op_e           rop_q;
    l1_amo_op_e            ramo_q;
    logic [MEMORY_ADDR_WIDTH-1:0] rwa_q;
    logic [XLEN-1:0]       rwd_q;
    logic                  rsv_valid_q;
    logic [MEMORY_ADDR_WIDTH-1:0] rsv_line_q;
    // pending snoop response (consume the snoop immediately, drive the answer next cycle, so the
    // dir's "snoop accepted -> now send me the data" handshake can't deadlock against our up port)
    logic                  snp_pend_q;
    NIIGO_CMI::cmi_op_e    snp_op_q;
    logic [LINE_BITS-1:0]  snp_line_q;
    cmi_owner_next_e       snp_onext_q;
    logic [CORE_ID_W-1:0]  snp_req_q;
    logic [MEMORY_ADDR_WIDTH-1:0] snp_laddr_q;

    // ---- next-state combinational ----
    astate_e               st_n;     logic sent_n;
    l1_core_op_e           rop_n;    l1_amo_op_e ramo_n;
    logic [MEMORY_ADDR_WIDTH-1:0] rwa_n;  logic [XLEN-1:0] rwd_n;
    logic                  rsv_valid_n; logic [MEMORY_ADDR_WIDTH-1:0] rsv_line_n;
    logic                  snp_pend_n;  NIIGO_CMI::cmi_op_e snp_op_n;
    logic [LINE_BITS-1:0]  snp_line_n;  cmi_owner_next_e snp_onext_n;
    logic [CORE_ID_W-1:0]  snp_req_n;   logic [MEMORY_ADDR_WIDTH-1:0] snp_laddr_n;
    // cache writes (applied in seq): full-line install, single-word write, state-only
    logic                  inst_we;  logic [IDX_W-1:0] inst_idx; cmi_state_e inst_st;
    logic [TAG_W-1:0]      inst_tag; logic [LINE_BITS-1:0] inst_line;
    logic                  word_we;  logic [IDX_W-1:0] word_idx; logic [LINE_WORD_BITS-1:0] word_off;
    logic [XLEN-1:0]       word_val;
    logic                  st_we;    logic [IDX_W-1:0] st_idx;   cmi_state_e st_val;

    ccd_chan_t up_c; logic down_rdy_c, creq_rdy_c; logic [XLEN-1:0] crdata_c; logic csc_c;

    // a snoop is pending on the down link (only in A_IDLE under serialised dir)
    logic snoop_v;
    always_comb snoop_v = down_i.valid &&
        (down_i.msg.op==OP_FWD_GETS || down_i.msg.op==OP_FWD_GETM || down_i.msg.op==OP_INV);

    // helper: build a Put op for evicting a line in state `s`
    function automatic NIIGO_CMI::cmi_op_e put_for(input cmi_state_e s);
        unique case (s)
            CMI_M:   put_for = OP_PUTM;
            CMI_O:   put_for = OP_PUTO;
            CMI_E:   put_for = OP_PUTE;
            default: put_for = OP_PUTS;
        endcase
    endfunction

    always_comb begin
        // defaults
        st_n=st_q; sent_n=sent_q; rop_n=rop_q; ramo_n=ramo_q; rwa_n=rwa_q; rwd_n=rwd_q;
        rsv_valid_n=rsv_valid_q; rsv_line_n=rsv_line_q;
        snp_pend_n=snp_pend_q; snp_op_n=snp_op_q; snp_line_n=snp_line_q; snp_onext_n=snp_onext_q;
        snp_req_n=snp_req_q; snp_laddr_n=snp_laddr_q;
        inst_we=0; inst_idx='0; inst_st=CMI_I; inst_tag='0; inst_line='0;
        word_we=0; word_idx='0; word_off='0; word_val='0;
        st_we=0;   st_idx='0;   st_val=CMI_I;
        up_c='{default:'0}; down_rdy_c=1'b0; creq_rdy_c=1'b0; crdata_c='0; csc_c=1'b0;

        unique case (st_q)
        // =========================================================
        A_IDLE: begin
            if (snp_pend_q) begin
                // ---- drive the latched snoop response up to the directory ----
                up_c.valid     = 1'b1;
                up_c.msg.op    = snp_op_q;
                up_c.msg.src   = CORE_ID[CORE_ID_W-1:0];
                up_c.msg.line  = snp_line_q;
                up_c.msg.onext = snp_onext_q;
                up_c.msg.req   = snp_req_q;
                up_c.msg.laddr = snp_laddr_q;
                if (up_ready_i) snp_pend_n = 1'b0;
            end else if (snoop_v) begin
                // ---- consume the snoop NOW, latch the response + downgrade the line ----
                automatic logic [IDX_W-1:0] si = idxf(down_i.msg.laddr);
                down_rdy_c   = 1'b1;
                snp_pend_n   = 1'b1;
                snp_laddr_n  = down_i.msg.laddr;
                snp_req_n    = down_i.msg.req;
                snp_onext_n  = ON_NA;
                unique case (down_i.msg.op)
                    OP_FWD_GETS: begin
                        snp_op_n=OP_DATA; snp_line_n=data_q[si];
                        snp_onext_n=(state_q[si]==CMI_E)?ON_S:ON_O;     // E->S clean, M/O->O
                        st_we=1; st_idx=si; st_val=(state_q[si]==CMI_E)?CMI_S:CMI_O;
                    end
                    OP_FWD_GETM: begin
                        snp_op_n=OP_DATA; snp_line_n=data_q[si];
                        st_we=1; st_idx=si; st_val=CMI_I;
                        if (rsv_valid_q && rsv_line_q==down_i.msg.laddr) rsv_valid_n=1'b0;
                    end
                    OP_INV: begin
                        snp_op_n=OP_INV_ACK;
                        st_we=1; st_idx=si; st_val=CMI_I;
                        if (rsv_valid_q && rsv_line_q==down_i.msg.laddr) rsv_valid_n=1'b0;
                    end
                    default: ;
                endcase
            end else if (c_req_valid) begin
                // ---- process a core request ----
                automatic logic [IDX_W-1:0]   ix = idxf(c_req_waddr);
                automatic logic [LINE_WORD_BITS-1:0] off = offf(c_req_waddr);
                automatic cmi_state_e         cs = state_q[ix];
                automatic logic               hit = (cs!=CMI_I) && (tag_q[ix]==tagf(c_req_waddr));
                automatic logic               writable = (cs==CMI_M)||(cs==CMI_E);
                rop_n=c_req_op; ramo_n=c_req_amo; rwa_n=c_req_waddr; rwd_n=c_req_wdata;
                sent_n=1'b0;

                unique case (c_req_op)
                // ---- LOAD / LR ----
                COP_LOAD, COP_LR: begin
                    if (hit) begin
                        creq_rdy_c=1'b1; crdata_c=wordrd(data_q[ix],off);
                        if (c_req_op==COP_LR) begin rsv_valid_n=1'b1; rsv_line_n=linebase(c_req_waddr); end
                    end else if (cs!=CMI_I) begin
                        st_n=A_WB;                          // conflict victim -> writeback first
                    end else st_n=A_FILL;                   // GETS
                end
                // ---- STORE ----
                COP_STORE: begin
                    if (hit && cs==CMI_M) begin
                        creq_rdy_c=1'b1; word_we=1; word_idx=ix; word_off=off; word_val=c_req_wdata;
                        if (rsv_valid_q && rsv_line_q==linebase(c_req_waddr)) rsv_valid_n=1'b0;
                    end else if (hit && cs==CMI_E) begin
                        creq_rdy_c=1'b1; word_we=1; word_idx=ix; word_off=off; word_val=c_req_wdata;
                        st_we=1; st_idx=ix; st_val=CMI_M;
                        if (rsv_valid_q && rsv_line_q==linebase(c_req_waddr)) rsv_valid_n=1'b0;
                    end else if (hit) begin                 // S/O -> Upgrade
                        st_n=A_UPG;
                    end else if (cs!=CMI_I) begin
                        st_n=A_WB;                          // evict victim, then GetM
                    end else st_n=A_FILL;                   // GetM
                end
                // ---- SC ----
                COP_SC: begin
                    automatic logic resv = rsv_valid_q && (rsv_line_q==linebase(c_req_waddr));
                    if (!resv) begin
                        creq_rdy_c=1'b1; csc_c=1'b0;        // SC fail (no reservation)
                    end else if (hit && writable) begin
                        creq_rdy_c=1'b1; csc_c=1'b1;
                        word_we=1; word_idx=ix; word_off=off; word_val=c_req_wdata;
                        st_we=1; st_idx=ix; st_val=CMI_M; rsv_valid_n=1'b0;
                    end else if (hit) begin                 // S/O + reserved -> Upgrade
                        st_n=A_UPG;
                    end else begin
                        creq_rdy_c=1'b1; csc_c=1'b0;        // lost the line -> fail
                    end
                end
                // ---- AMO ----
                COP_AMO: begin
                    if (hit && writable) begin
                        automatic logic [XLEN-1:0] old = wordrd(data_q[ix],off);
                        creq_rdy_c=1'b1; crdata_c=old;
                        word_we=1; word_idx=ix; word_off=off; word_val=amo_apply(c_req_amo,old,c_req_wdata);
                        st_we=1; st_idx=ix; st_val=CMI_M;
                        if (rsv_valid_q && rsv_line_q==linebase(c_req_waddr)) rsv_valid_n=1'b0;
                    end else if (hit) begin
                        st_n=A_UPG;
                    end else if (cs!=CMI_I) begin
                        st_n=A_WB;
                    end else st_n=A_FILL;
                end
                default: creq_rdy_c=1'b1;
                endcase
            end
        end

        // =========================================================
        // A_WB: writeback the conflict victim, then go fill the requested line
        A_WB: begin
            automatic logic [IDX_W-1:0] vi = idxf(rwa_q);
            if (!sent_q) begin
                up_c.valid     = 1'b1;
                up_c.msg.op    = put_for(state_q[vi]);
                up_c.msg.src   = CORE_ID[CORE_ID_W-1:0];
                up_c.msg.laddr = {tag_q[vi], vi, {LINE_WORD_BITS{1'b0}}};   // victim line addr
                up_c.msg.line  = data_q[vi];
                if (up_ready_i) sent_n=1'b1;
            end else if (down_i.valid && down_i.msg.op==OP_ACK) begin
                down_rdy_c=1'b1;
                st_we=1; st_idx=vi; st_val=CMI_I;            // victim now invalid
                if (rsv_valid_q && rsv_line_q==up_c.msg.laddr) rsv_valid_n=1'b0;
                sent_n=1'b0; st_n=A_FILL;                    // now fetch the requested line
            end
        end

        // =========================================================
        // A_FILL: issue GETS (load/LR) or GETM (store/amo) and install on DATA
        A_FILL: begin
            automatic logic [IDX_W-1:0] ix = idxf(rwa_q);
            automatic logic [LINE_WORD_BITS-1:0] off = offf(rwa_q);
            automatic logic is_rd = (rop_q==COP_LOAD)||(rop_q==COP_LR);
            if (!sent_q) begin
                up_c.valid     = 1'b1;
                up_c.msg.op    = is_rd ? OP_GETS : OP_GETM;
                up_c.msg.src   = CORE_ID[CORE_ID_W-1:0];
                up_c.msg.laddr = linebase(rwa_q);
                if (up_ready_i) sent_n=1'b1;
            end else if (down_i.valid && down_i.msg.op==OP_DATA) begin
                down_rdy_c=1'b1; sent_n=1'b0; st_n=A_IDLE; creq_rdy_c=1'b1;
                inst_we=1; inst_idx=ix; inst_tag=tagf(rwa_q);
                unique case (rop_q)
                    COP_LOAD, COP_LR: begin
                        inst_st=down_i.msg.gst; inst_line=down_i.msg.line;
                        crdata_c=wordrd(down_i.msg.line,off);
                        if (rop_q==COP_LR) begin rsv_valid_n=1'b1; rsv_line_n=linebase(rwa_q); end
                    end
                    COP_STORE: begin
                        inst_st=CMI_M; inst_line=wordmerge(down_i.msg.line,off,rwd_q);
                        if (rsv_valid_q && rsv_line_q==linebase(rwa_q)) rsv_valid_n=1'b0;
                    end
                    COP_AMO: begin
                        automatic logic [XLEN-1:0] old = wordrd(down_i.msg.line,off);
                        inst_st=CMI_M; inst_line=wordmerge(down_i.msg.line,off,amo_apply(ramo_q,old,rwd_q));
                        crdata_c=old;
                        if (rsv_valid_q && rsv_line_q==linebase(rwa_q)) rsv_valid_n=1'b0;
                    end
                    default: begin inst_st=down_i.msg.gst; inst_line=down_i.msg.line; end
                endcase
            end
        end

        // =========================================================
        // A_UPG: issue UPGRADE (S/O -> M) and complete the write on the grant
        A_UPG: begin
            automatic logic [IDX_W-1:0] ix = idxf(rwa_q);
            automatic logic [LINE_WORD_BITS-1:0] off = offf(rwa_q);
            if (!sent_q) begin
                up_c.valid     = 1'b1;
                up_c.msg.op    = OP_UPGRADE;
                up_c.msg.src   = CORE_ID[CORE_ID_W-1:0];
                up_c.msg.laddr = linebase(rwa_q);
                if (up_ready_i) sent_n=1'b1;
            end else if (down_i.valid && (down_i.msg.op==OP_DATA || down_i.msg.op==OP_ACK)) begin
                down_rdy_c=1'b1; sent_n=1'b0; st_n=A_IDLE; creq_rdy_c=1'b1;
                st_we=1; st_idx=ix; st_val=CMI_M;
                unique case (rop_q)
                    COP_STORE: begin
                        word_we=1; word_idx=ix; word_off=off; word_val=rwd_q;
                        if (rsv_valid_q && rsv_line_q==linebase(rwa_q)) rsv_valid_n=1'b0;
                    end
                    COP_SC: begin
                        if (rsv_valid_q && rsv_line_q==linebase(rwa_q)) begin
                            word_we=1; word_idx=ix; word_off=off; word_val=rwd_q; csc_c=1'b1;
                        end else csc_c=1'b0;
                        rsv_valid_n=1'b0;
                    end
                    COP_AMO: begin
                        automatic logic [XLEN-1:0] old = wordrd(data_q[ix],off);
                        word_we=1; word_idx=ix; word_off=off; word_val=amo_apply(ramo_q,old,rwd_q);
                        crdata_c=old;
                        if (rsv_valid_q && rsv_line_q==linebase(rwa_q)) rsv_valid_n=1'b0;
                    end
                    default: ;
                endcase
            end
        end
        endcase
    end

    // ================= sequential =================
    always_ff @(posedge clk or negedge rst_l) begin
        if (!rst_l) begin
            st_q<=A_IDLE; sent_q<=1'b0; rsv_valid_q<=1'b0; rsv_line_q<='0; snp_pend_q<=1'b0;
            for (int s=0;s<SETS;s++) begin state_q[s]<=CMI_I; tag_q[s]<='0; end
        end else begin
            st_q<=st_n; sent_q<=sent_n; rop_q<=rop_n; ramo_q<=ramo_n; rwa_q<=rwa_n; rwd_q<=rwd_n;
            rsv_valid_q<=rsv_valid_n; rsv_line_q<=rsv_line_n;
            snp_pend_q<=snp_pend_n; snp_op_q<=snp_op_n; snp_line_q<=snp_line_n;
            snp_onext_q<=snp_onext_n; snp_req_q<=snp_req_n; snp_laddr_q<=snp_laddr_n;
            if (inst_we) begin
                state_q[inst_idx]<=inst_st; tag_q[inst_idx]<=inst_tag; data_q[inst_idx]<=inst_line;
            end else if (st_we) state_q[st_idx]<=st_val;
            if (word_we) data_q[word_idx][word_off*XLEN +: XLEN] <= word_val;
        end
    end

    assign up_o         = up_c;
    assign down_ready_o = down_rdy_c;
    assign c_req_ready  = creq_rdy_c;
    assign c_resp_rdata = crdata_c;
    assign c_resp_sc_ok = csc_c;
    /* verilator lint_on ENUMVALUE */
endmodule
`default_nettype wire
