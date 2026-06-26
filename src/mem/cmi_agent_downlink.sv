/**
 * cmi_agent_downlink.sv  --  M3c-D agent Local-out demux (flits -> snoop / resp / ack channels)
 *
 * Splits a core router's Local-out flit stream (plans/multicore-ccd.md §5.4) by VC into the three
 * message channels niigo_l1d_gg consumes: C1 snoop (VC1, fwd/inv from the dir), C2 response data
 * (VC2 + the ring dateline sub-VC VC2B, from the dir L2-source OR a peer owner over the ring), and
 * C4 ack (VC3, InvAck from a sharer + WB_ACK from the dir). Each channel is one VC-gated
 * single-message reassembler (cmi_msg_rx): per-endpoint per-VC traffic is already serialised (the
 * agent has 1 demand + 1 snoop outstanding), so contiguous-per-VC packets need no interleave logic.
 */
`include "niigo_cmi.vh"
`include "niigo_ccd_m1.vh"
`default_nettype none
module cmi_agent_downlink
    import NIIGO_CMI::*;
    import NIIGO_CCD_M1::*;
(
    input  wire logic              clk,
    input  wire logic              rst_l,
    // core router Local-out (flits in) + credit back
    input  wire cmi_link_t         loc_out,
    output logic [NUM_VC_PHYS-1:0] loc_out_cr,
    // demuxed message channels to the agent
    output logic     snoop_valid, output ccd_msg_t snoop_msg, input wire logic snoop_ready,  // VC1
    output logic     resp_valid,  output ccd_msg_t resp_msg,  input wire logic resp_ready,    // VC2/VC2B
    output logic     ack_valid,   output ccd_msg_t ack_msg,   input wire logic ack_ready      // VC3
);
    // per-channel VC-gated input link
    cmi_link_t in_sn, in_rs, in_ak;
    logic [NUM_VC_PHYS-1:0] cr_sn, cr_rs, cr_ak;
    always_comb begin
        in_sn = '{default:'0}; in_rs = '{default:'0}; in_ak = '{default:'0};
        if (loc_out.valid) begin
            unique case (loc_out.ctrl.vc)
                3'd1:    begin in_sn = loc_out; end                      // C1 snoop
                3'd2,
                CMI_VC2B:begin in_rs = loc_out; end                      // C2 data (+ ring VC2B)
                3'd3:    begin in_ak = loc_out; end                      // C4 ack
                default: ;                                              // (VC0/VC4 never arrive here)
            endcase
        end
    end

    cmi_msg_rx RXS (.clk, .rst_l, .in(in_sn), .credit_out(cr_sn),
        .out_valid(snoop_valid), .out_msg(snoop_msg), .out_src(), .out_ready(snoop_ready));
    cmi_msg_rx RXR (.clk, .rst_l, .in(in_rs), .credit_out(cr_rs),
        .out_valid(resp_valid),  .out_msg(resp_msg),  .out_src(), .out_ready(resp_ready));
    cmi_msg_rx RXA (.clk, .rst_l, .in(in_ak), .credit_out(cr_ak),
        .out_valid(ack_valid),   .out_msg(ack_msg),   .out_src(), .out_ready(ack_ready));

    assign loc_out_cr = cr_sn | cr_rs | cr_ak;   // disjoint VCs -> simple OR
endmodule
`default_nettype wire
