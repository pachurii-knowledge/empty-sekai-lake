/**
 * cmi_hub_dir_rx.sv  --  M3c-D hub internal-out demux (flits -> req / unblk / wb channels)
 *
 * Splits the wheel hub's Internal-out flit stream (plans/multicore-ccd.md §5.3) by VC into the
 * three inbound channels niigo_dir_gg consumes: C0 request (VC0, from any core via the hub 4->1
 * arbiter -- that serialisation IS the directory's admission order), C4 UNBLOCK (VC3), and C3
 * writeback data (VC4). Three VC-gated single-message reassemblers (cmi_msg_rx); the per-VC
 * separation lets an UNBLOCK/WB reach the dir even while a request is held back (the dir is busy),
 * which is what makes finalize-outranks-admit work.
 */
`include "niigo_cmi.vh"
`include "niigo_ccd_m1.vh"
`default_nettype none
module cmi_hub_dir_rx
    import NIIGO_CMI::*;
    import NIIGO_CCD_M1::*;
(
    input  wire logic              clk,
    input  wire logic              rst_l,
    input  wire cmi_link_t         int_out,
    output logic [NUM_VC_PHYS-1:0] int_out_cr,
    output logic     req_valid,   output ccd_msg_t req_msg,   input wire logic req_ready,   // VC0
    output logic     unblk_valid, output ccd_msg_t unblk_msg, input wire logic unblk_ready, // VC3
    output logic     wb_valid,    output ccd_msg_t wb_msg,    input wire logic wb_ready     // VC4
);
    cmi_link_t in_rq, in_ub, in_wb;
    logic [NUM_VC_PHYS-1:0] cr_rq, cr_ub, cr_wb;
    always_comb begin
        in_rq = '{default:'0}; in_ub = '{default:'0}; in_wb = '{default:'0};
        if (int_out.valid) begin
            unique case (int_out.ctrl.vc)
                3'd0:    in_rq = int_out;    // C0 request
                3'd3:    in_ub = int_out;    // C4 UNBLOCK
                3'd4:    in_wb = int_out;    // C3 WB data
                default: ;
            endcase
        end
    end

    cmi_msg_rx RXQ (.clk, .rst_l, .in(in_rq), .credit_out(cr_rq),
        .out_valid(req_valid),   .out_msg(req_msg),   .out_src(), .out_ready(req_ready));
    cmi_msg_rx RXU (.clk, .rst_l, .in(in_ub), .credit_out(cr_ub),
        .out_valid(unblk_valid), .out_msg(unblk_msg), .out_src(), .out_ready(unblk_ready));
    cmi_msg_rx RXW (.clk, .rst_l, .in(in_wb), .credit_out(cr_wb),
        .out_valid(wb_valid),    .out_msg(wb_msg),    .out_src(), .out_ready(wb_ready));

    assign int_out_cr = cr_rq | cr_ub | cr_wb;
endmodule
`default_nettype wire
