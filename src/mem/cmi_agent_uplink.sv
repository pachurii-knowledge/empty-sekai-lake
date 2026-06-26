/**
 * cmi_agent_uplink.sv  --  M3c-D agent Local-in serialiser (2 sources -> flits, shared credit)
 *
 * Merges the agent's two outbound message channels (plans/multicore-ccd.md §5.4) onto one core
 * router Local-in flit stream:
 *   - DEMAND slot: dmd (dst = HUB)        -- GetS/GetM/Upgrade/Put/UNBLOCK/WB_DATA (VC0/VC3/VC4)
 *   - SNOOP slot:  snp (dst = a peer core)-- Data-forward (VC2, ring) / INV_ACK (VC3, spoke->hub->spoke)
 *
 * One shared per-VC credit pool + a snoop-priority arbiter with PER-VC WORMHOLE ATOMICITY: a
 * multi-flit packet locks ITS vc head-to-tail, but the other slot may send on a DIFFERENT vc
 * (cross-vc interleave is legal). This is the load-bearing deadlock-freedom property: a snoop
 * response on VC2/VC3 can always make progress regardless of a demand multi-flit (e.g. an
 * embedded-line PutM on VC0) in flight -- so the snoop-during-miss progress rule (R1) holds and a
 * credit-starved demand never blocks a creditable snoop. One flit leaves per cycle (loc_in is one
 * physical channel); a registered output keeps the link a flop boundary.
 */
`include "niigo_cmi.vh"
`include "niigo_ccd_m1.vh"
`default_nettype none
module cmi_agent_uplink
    import NIIGO_CMI::*;
    import NIIGO_CCD_M1::*;
    import NIIGO_Mem::LINE_BITS;
#(
    parameter int unsigned CORE_ID = 0
)(
    input  wire logic              clk,
    input  wire logic              rst_l,
    // demand slot (to HUB)
    input  wire logic              dmd_valid, input wire ccd_msg_t dmd_msg, output logic dmd_ready,
    // snoop-response slot (to a peer core)
    input  wire logic              snp_valid, input wire ccd_msg_t snp_msg,
    input  wire logic [NODE_ID_W-1:0] snp_dst, output logic snp_ready,
    // core router Local-in (flits out) + credit back
    output cmi_link_t              loc_in,
    input  wire logic [NUM_VC_PHYS-1:0] loc_in_cr
);
    localparam logic [2:0] OC_MAX = 3'(VC_DEPTH);
    localparam int NBODY = LINE_FLITS;   // 4

    // ---- two serialiser slots: D (demand, dst=HUB), S (snoop, dst=peer) ----
    logic        dbusy_q; ccd_msg_t dmsg_q; logic [2:0] dbeat_q;
    logic        sbusy_q; ccd_msg_t smsg_q; logic [NODE_ID_W-1:0] sdst_q; logic [2:0] sbeat_q;
    logic [2:0]  ocredit_q [NUM_VC_PHYS];
    logic [1:0]  vlock_q [NUM_VC_PHYS];   // 0=free, 1=D, 2=S (per-vc wormhole lock)

    logic        dbusy_n; ccd_msg_t dmsg_n; logic [2:0] dbeat_n;
    logic        sbusy_n; ccd_msg_t smsg_n; logic [NODE_ID_W-1:0] sdst_n; logic [2:0] sbeat_n;
    logic [2:0]  ocredit_n [NUM_VC_PHYS];
    logic [1:0]  vlock_n [NUM_VC_PHYS];
    cmi_link_t   out_n;

    cmi_link_t   out_q;

    // build a HEAD flit word { ccd_ctrl, rhdr }
    function automatic logic [CMI_FLIT_W-1:0] head_word(input ccd_msg_t m, input logic [NODE_ID_W-1:0] dst);
        ccd_ctrl_t ctrl; cmi_rhdr_t rh; logic [CMI_FLIT_W-1:0] d;
        ctrl.op=m.op; ctrl.src=m.src; ctrl.is_icache=m.is_icache; ctrl.gst=m.gst;
        ctrl.onext=m.onext; ctrl.acks=m.acks; ctrl.req=m.req; ctrl.laddr=m.laddr;
        rh.dst=dst; rh.src_core=m.src; rh.mclass=cmi_op_class(m.op);
        d='0; d[CMI_RHDR_W-1:0]=rh; d[CMI_RHDR_W +: CCD_CTRL_W]=ctrl;
        return d;
    endfunction

    // per-slot current-flit descriptors
    logic [VC_ID_W-1:0] dvc, svc;
    logic [2:0] dnb, snb;
    logic dhas, shas, d_can, s_can, dleg, sleg;
    flit_kind_e dkind, skind;
    logic [CMI_FLIT_W-1:0] ddata, sdata;
    always_comb begin
        dhas = ccd_has_line(dmsg_q.op); dnb = dhas?3'd5:3'd1; dvc = cmi_vc(cmi_op_class(dmsg_q.op));
        shas = ccd_has_line(smsg_q.op); snb = shas?3'd5:3'd1; svc = cmi_vc(cmi_op_class(smsg_q.op));
        // D current flit
        if (dbeat_q==3'd0) begin dkind = dhas?FLIT_HEAD:FLIT_HEADTAIL; ddata = head_word(dmsg_q, NODE_ID_W'(CMI_HUB_ID)); end
        else begin dkind = (dbeat_q==dnb-3'd1)?FLIT_TAIL:FLIT_BODY; ddata = dmsg_q.line[(dbeat_q-3'd1)*CMI_FLIT_W +: CMI_FLIT_W]; end
        if (sbeat_q==3'd0) begin skind = shas?FLIT_HEAD:FLIT_HEADTAIL; sdata = head_word(smsg_q, sdst_q); end
        else begin skind = (sbeat_q==snb-3'd1)?FLIT_TAIL:FLIT_BODY; sdata = smsg_q.line[(sbeat_q-3'd1)*CMI_FLIT_W +: CMI_FLIT_W]; end
        dleg = (vlock_q[dvc]==2'd0) || (vlock_q[dvc]==2'd1);
        sleg = (vlock_q[svc]==2'd0) || (vlock_q[svc]==2'd2);
        d_can = dbusy_q && (ocredit_q[dvc]!=3'd0) && dleg;
        s_can = sbusy_q && (ocredit_q[svc]!=3'd0) && sleg;
    end

    always_comb begin
        dbusy_n=dbusy_q; dmsg_n=dmsg_q; dbeat_n=dbeat_q;
        sbusy_n=sbusy_q; smsg_n=smsg_q; sdst_n=sdst_q; sbeat_n=sbeat_q;
        for (int v=0;v<NUM_VC_PHYS;v++) begin
            ocredit_n[v]=ocredit_q[v];
            if (loc_in_cr[v] && ocredit_n[v]<OC_MAX) ocredit_n[v]=ocredit_n[v]+3'd1;
            vlock_n[v]=vlock_q[v];
        end
        out_n='{default:'0};
        dmd_ready = !dbusy_q;  snp_ready = !sbusy_q;

        // accept new messages into idle slots
        if (!dbusy_q && dmd_valid) begin dbusy_n=1'b1; dmsg_n=dmd_msg; dbeat_n=3'd0; end
        if (!sbusy_q && snp_valid) begin sbusy_n=1'b1; smsg_n=snp_msg; sdst_n=snp_dst; sbeat_n=3'd0; end

        // arbiter: snoop priority, one flit/cycle, per-vc wormhole legal + creditable
        if (s_can) begin
            out_n.valid=1'b1; out_n.ctrl.kind=skind; out_n.ctrl.vc=svc; out_n.data=sdata;
            ocredit_n[svc]=ocredit_n[svc]-3'd1;
            if (skind==FLIT_HEAD)       vlock_n[svc]=2'd2;                 // lock svc to S
            if (sbeat_q==snb-3'd1) begin sbusy_n=1'b0; sbeat_n=3'd0; if (vlock_n[svc]==2'd2) vlock_n[svc]=2'd0; end
            else                         sbeat_n=sbeat_q+3'd1;
        end else if (d_can) begin
            out_n.valid=1'b1; out_n.ctrl.kind=dkind; out_n.ctrl.vc=dvc; out_n.data=ddata;
            ocredit_n[dvc]=ocredit_n[dvc]-3'd1;
            if (dkind==FLIT_HEAD)       vlock_n[dvc]=2'd1;
            if (dbeat_q==dnb-3'd1) begin dbusy_n=1'b0; dbeat_n=3'd0; if (vlock_n[dvc]==2'd1) vlock_n[dvc]=2'd0; end
            else                         dbeat_n=dbeat_q+3'd1;
        end
    end

    always_ff @(posedge clk or negedge rst_l) begin
        if (!rst_l) begin
            dbusy_q<=1'b0; sbusy_q<=1'b0; dbeat_q<=3'd0; sbeat_q<=3'd0; out_q<='{default:'0};
            for (int v=0;v<NUM_VC_PHYS;v++) begin ocredit_q[v]<=OC_MAX; vlock_q[v]<=2'd0; end
        end else begin
            dbusy_q<=dbusy_n; dmsg_q<=dmsg_n; dbeat_q<=dbeat_n;
            sbusy_q<=sbusy_n; smsg_q<=smsg_n; sdst_q<=sdst_n; sbeat_q<=sbeat_n;
            out_q<=out_n;
            for (int v=0;v<NUM_VC_PHYS;v++) begin ocredit_q[v]<=ocredit_n[v]; vlock_q[v]<=vlock_n[v]; end
        end
    end

    assign loc_in = out_q;
endmodule
`default_nettype wire
