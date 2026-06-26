/**
 * cmi_msg_rx.sv  --  M3 message deserialiser: CMI flit train -> ccd_msg_t (the wheel SerDes, RX)
 *
 * Reassembles a CMI flit train (plans/multicore-ccd.md §W.2) back into one M1 full-line message
 * (niigo_ccd_m1.vh): a HEAD/HEADTAIL flit yields the ccd control (ccd_ctrl_t, unpacked from above
 * the cmi_rhdr_t); the 4 body flits refill the 512 b line. Presents one completed message
 * (out_valid/out_ready) plus its source core (from the head). Single in-flight reassembly:
 * each wheel endpoint sources <=1 message at a time (blocking M1 agent / serialised directory),
 * so per-endpoint traffic is already serialised -- no per-VC interleaving to untangle (that, and
 * the hub funnel, are M3c). Credit-returned per consumed flit; a COMPLETING flit (TAIL/HEADTAIL)
 * is held off (credit withheld -> upstream backpressure) until the previous completed message is
 * drained, so a slow consumer can never clobber an undelivered message.
 */
`include "niigo_cmi.vh"
`include "niigo_ccd_m1.vh"
`default_nettype none
module cmi_msg_rx
    import NIIGO_CMI::*;
    import NIIGO_CCD_M1::*;
    import NIIGO_Mem::LINE_BITS;
(
    input  wire logic                    clk,
    input  wire logic                    rst_l,
    // CMI flit input port
    input  wire cmi_link_t               in,
    output logic [NUM_VC_PHYS-1:0]       credit_out,
    // reassembled message output (1-deep)
    output logic                         out_valid,
    output ccd_msg_t                     out_msg,
    output logic [CORE_ID_W-1:0]         out_src,
    input  wire logic                    out_ready
);
    // ---- reassembly state ----
    logic                  busy_q;          // mid multi-flit packet
    logic [2:0]            beat_q;          // body beat index 0..3
    ccd_ctrl_t             ctrl_q;          // latched at HEAD
    logic [LINE_BITS-1:0]  line_q;
    // ---- completed-message holding register ----
    logic                  outv_q;
    ccd_msg_t              outm_q;
    logic [CORE_ID_W-1:0]  outs_q;
    logic [NUM_VC_PHYS-1:0] cr_q;

    logic                  busy_n;  logic [2:0] beat_n;
    ccd_ctrl_t             ctrl_n;  logic [LINE_BITS-1:0] line_n;
    logic                  outv_n;  ccd_msg_t outm_n;  logic [CORE_ID_W-1:0] outs_n;
    logic [NUM_VC_PHYS-1:0] cr_n;

    function automatic ccd_msg_t mk_msg(input ccd_ctrl_t c, input logic [LINE_BITS-1:0] ln);
        ccd_msg_t m;
        m.op=c.op; m.src=c.src; m.is_icache=c.is_icache; m.gst=c.gst;
        m.onext=c.onext; m.acks=c.acks; m.req=c.req; m.line=ln; m.laddr=c.laddr;
        return m;
    endfunction

    logic        is_complete, can_complete, accept;
    ccd_ctrl_t   hctrl;
    always_comb begin
        is_complete  = in.valid && (in.ctrl.kind==FLIT_TAIL || in.ctrl.kind==FLIT_HEADTAIL);
        can_complete = !outv_q || out_ready;
        accept       = in.valid && (!is_complete || can_complete);
        hctrl        = in.data[CMI_RHDR_W +: CCD_CTRL_W];
    end

    always_comb begin
        busy_n=busy_q; beat_n=beat_q; ctrl_n=ctrl_q; line_n=line_q;
        outv_n = outv_q && !out_ready;             // drain on handshake
        outm_n=outm_q; outs_n=outs_q;
        cr_n = '0;
        if (accept) begin
            cr_n[in.ctrl.vc] = 1'b1;               // return one credit for the consumed flit
            unique case (in.ctrl.kind)
                FLIT_HEADTAIL: begin
                    outv_n=1'b1; outm_n=mk_msg(hctrl, '0); outs_n=hctrl.src;
                end
                FLIT_HEAD: begin
                    ctrl_n=hctrl; line_n='0; beat_n=3'd0; busy_n=1'b1;
                end
                FLIT_BODY: begin
                    line_n[beat_q*CMI_FLIT_W +: CMI_FLIT_W]=in.data; beat_n=beat_q+3'd1;
                end
                FLIT_TAIL: begin
                    line_n[beat_q*CMI_FLIT_W +: CMI_FLIT_W]=in.data;
                    outv_n=1'b1; outm_n=mk_msg(ctrl_q, line_n); outs_n=ctrl_q.src; busy_n=1'b0;
                end
                default: ;
            endcase
        end
    end

    always_ff @(posedge clk or negedge rst_l) begin
        if (!rst_l) begin
            busy_q<=1'b0; beat_q<=3'd0; line_q<='0; outv_q<=1'b0; cr_q<='0;
        end else begin
            busy_q<=busy_n; beat_q<=beat_n; ctrl_q<=ctrl_n; line_q<=line_n;
            outv_q<=outv_n; outm_q<=outm_n; outs_q<=outs_n; cr_q<=cr_n;
        end
    end

    assign credit_out = cr_q;
    assign out_valid  = outv_q;
    assign out_msg    = outm_q;
    assign out_src    = outs_q;
endmodule
`default_nettype wire
