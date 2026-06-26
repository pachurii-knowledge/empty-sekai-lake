/**
 * cmi_msg_tx.sv  --  M3 message serialiser: ccd_msg_t -> CMI flit train (the wheel SerDes, TX)
 *
 * Serialises one M1 full-line coherence message (niigo_ccd_m1.vh) into the canonical 128 b
 * multi-flit CMI form (plans/multicore-ccd.md §W.2): a HEAD flit carrying the ccd control
 * (ccd_ctrl_t) packed above the router-visible cmi_rhdr_t, followed by 4 body flits (the 512 b
 * line) when the op carries data (DATA/WB/dirty Put), else a single HEADTAIL. Credit-paced:
 * a flit leaves only when the downstream input buffer has a credit on that VC. One message in
 * flight (1-deep) -- matches the blocking M1 agent/serialised directory (each endpoint sources
 * <=1 message at a time). The driven flit word is a registered function of credit/beat state,
 * so it is stable for the cycle the downstream router samples it.
 */
`include "niigo_cmi.vh"
`include "niigo_ccd_m1.vh"
`default_nettype none
module cmi_msg_tx
    import NIIGO_CMI::*;
    import NIIGO_CCD_M1::*;
(
    input  wire logic                   clk,
    input  wire logic                   rst_l,
    // message input (agent up_o, or directory down_o[c]) -- 1-deep accept
    input  wire ccd_chan_t              msg_in,
    output logic                        msg_ready,
    input  wire logic [NODE_ID_W-1:0]   dst_node,     // routing target (HUB for up, core c for down)
    // CMI flit output port
    output cmi_link_t                   out,
    input  wire logic [NUM_VC_PHYS-1:0] credit_in     // credits returned by downstream input buffer
);
    localparam int NBODY = LINE_FLITS;                 // 4
    localparam logic [2:0] OC_MAX = 3'(VC_DEPTH);

    logic                  busy_q;
    ccd_msg_t              msg_q;
    logic [2:0]            beat_q;                      // 0..4 (0=head, 1..4=body/tail)
    logic [2:0]            ocredit_q [NUM_VC_PHYS];

    logic                  busy_n;
    ccd_msg_t              msg_n;
    logic [2:0]            beat_n;
    logic [2:0]            ocredit_n [NUM_VC_PHYS];

    // current message shape
    logic                  hasln;
    logic [2:0]            nbeats;
    logic [VC_ID_W-1:0]    curvc;
    cmi_class_e            mclass;
    always_comb begin
        hasln  = ccd_has_line(msg_q.op);
        nbeats = hasln ? 3'd5 : 3'd1;
        mclass = cmi_op_class(msg_q.op);
        curvc  = cmi_vc(mclass);
    end

    // build the HEAD flit word: { ccd_ctrl, rhdr } zero-extended
    function automatic logic [CMI_FLIT_W-1:0] head_word();
        ccd_ctrl_t ctrl; cmi_rhdr_t rh; logic [CMI_FLIT_W-1:0] d;
        ctrl.op=msg_q.op; ctrl.src=msg_q.src; ctrl.is_icache=msg_q.is_icache;
        ctrl.gst=msg_q.gst; ctrl.onext=msg_q.onext; ctrl.acks=msg_q.acks;
        ctrl.req=msg_q.req; ctrl.laddr=msg_q.laddr;
        rh.dst=dst_node; rh.src_core=msg_q.src; rh.mclass=mclass;
        d = '0;
        d[CMI_RHDR_W-1:0]              = rh;
        d[CMI_RHDR_W +: CCD_CTRL_W]    = ctrl;
        return d;
    endfunction

    logic send;
    always_comb send = busy_q && (ocredit_q[curvc] != 3'd0);

    // ---- output flit (combinational from registered state) ----
    always_comb begin
        out = '{default:'0};
        if (send) begin
            out.valid    = 1'b1;
            out.ctrl.vc  = curvc;
            if (beat_q == 3'd0) begin
                out.ctrl.kind = hasln ? FLIT_HEAD : FLIT_HEADTAIL;
                out.data      = head_word();
            end else begin
                out.ctrl.kind = (beat_q == nbeats-3'd1) ? FLIT_TAIL : FLIT_BODY;
                out.data      = msg_q.line[(beat_q-3'd1)*CMI_FLIT_W +: CMI_FLIT_W];
            end
        end
    end

    // ---- accept handshake ----
    always_comb msg_ready = !busy_q;

    always_comb begin
        busy_n=busy_q; msg_n=msg_q; beat_n=beat_q;
        for (int v=0;v<NUM_VC_PHYS;v++) begin
            ocredit_n[v]=ocredit_q[v];
            if (credit_in[v] && ocredit_n[v] < OC_MAX) ocredit_n[v]=ocredit_n[v]+3'd1;
        end
        if (!busy_q) begin
            if (msg_in.valid) begin busy_n=1'b1; msg_n=msg_in.msg; beat_n=3'd0; end
        end else if (send) begin
            ocredit_n[curvc] = ocredit_n[curvc] - 3'd1;
            if (beat_q == nbeats-3'd1) begin busy_n=1'b0; beat_n=3'd0; end
            else                          beat_n = beat_q + 3'd1;
        end
    end

    always_ff @(posedge clk or negedge rst_l) begin
        if (!rst_l) begin
            busy_q<=1'b0; beat_q<=3'd0;
            for (int v=0;v<NUM_VC_PHYS;v++) ocredit_q[v]<=OC_MAX;
        end else begin
            busy_q<=busy_n; msg_q<=msg_n; beat_q<=beat_n;
            for (int v=0;v<NUM_VC_PHYS;v++) ocredit_q[v]<=ocredit_n[v];
        end
    end
endmodule
`default_nettype wire
