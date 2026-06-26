/**
 * niigo_ccd_gg_wheel.sv  --  M3c-D grant-and-go MOESI CCD over the real wheel NoC
 *
 * The fabric-agnostic grant-and-go directory (niigo_dir_gg) + non-blocking agents (niigo_l1d_gg)
 * wired onto the real wheel fabric (cmi_wheel: 4 radix-4 core routers in the ring + radix-5 hub)
 * through the M3c-D funnel modules (plans/multicore-ccd.md §5):
 *   - each agent <-> its core router Local port via cmi_agent_uplink (dmd+snp -> flits, shared
 *     per-VC credit, snoop-priority, per-VC wormhole) + cmi_agent_downlink (flits -> snoop/resp/ack);
 *   - the directory <-> the hub Internal port via cmi_hub_dir_rx (flits -> req/unblk/wb) +
 *     cmi_msg_tx (the dir's one out msg -> flits, dst-routed; the hub 4->1 arbiter is the
 *     serialisation point).
 * Cache-to-cache C2 data rides the ring (owner->requester); InvAcks (C4) go spoke->hub->spoke;
 * directory traffic rides the spokes. Drop-in port-compatible with niigo_ccd_top so the S1-S6
 * coherence program (and the concurrency tests) run unchanged -- if green, the grant-and-go
 * protocol is transparent over the real NoC.
 */
`include "niigo_mem.vh"
`include "niigo_cmi.vh"
`include "niigo_ccd_m1.vh"
`default_nettype none
module niigo_ccd_gg_wheel
    import RISCV_ISA::XLEN;
    import RISCV_UArch::MEMORY_ADDR_WIDTH;
    import NIIGO_Mem::*;
    import NIIGO_CMI::*;
    import NIIGO_CCD_M1::*;
#(
    parameter int NACTIVE  = 2,
    parameter int DIR_SETS = 64,
    parameter int L1_SETS  = 8
)(
    input  wire logic clk,
    input  wire logic rst_l,
    input  wire logic                          c_req_valid [NACTIVE],
    output logic                               c_req_ready [NACTIVE],
    input  wire l1_core_op_e                   c_req_op    [NACTIVE],
    input  wire l1_amo_op_e                    c_req_amo   [NACTIVE],
    input  wire logic [MEMORY_ADDR_WIDTH-1:0]  c_req_waddr [NACTIVE],
    input  wire logic [XLEN-1:0]               c_req_wdata [NACTIVE],
    output logic [XLEN-1:0]                     c_resp_rdata[NACTIVE],
    output logic                               c_resp_sc_ok[NACTIVE],
    output nmi_req_t   mem_req_o,
    input  wire logic  mem_req_ready_i,
    input  nmi_resp_t  mem_resp_i
);
    localparam int CORES = NUM_CORES;

    // ---- wheel fabric endpoints ----
    cmi_link_t          loc_in  [CORES]; logic [NUM_VC_PHYS-1:0] loc_in_cr [CORES];
    cmi_link_t          loc_out [CORES]; logic [NUM_VC_PHYS-1:0] loc_out_cr[CORES];
    cmi_link_t          int_in;          logic [NUM_VC_PHYS-1:0] int_in_cr;
    cmi_link_t          int_out;         logic [NUM_VC_PHYS-1:0] int_out_cr;

    cmi_wheel WHEEL (.clk, .rst_l,
        .loc_in, .loc_in_cr, .loc_out, .loc_out_cr,
        .int_in, .int_in_cr, .int_out, .int_out_cr);

    // ---- directory + its hub-side funnel ----
    logic      dreq_v;  ccd_msg_t dreq_m;  logic dreq_r;
    logic      dunb_v;  ccd_msg_t dunb_m;  logic dunb_r;
    logic      dwb_v;   ccd_msg_t dwb_m;   logic dwb_r;
    logic      dout_v;  ccd_msg_t dout_m;  logic [NODE_ID_W-1:0] dout_d; logic dout_r;

    niigo_dir_gg #(.CORES(CORES), .DIR_SETS(DIR_SETS)) DIR (.clk, .rst_l,
        .req_valid(dreq_v), .req_msg(dreq_m), .req_ready(dreq_r),
        .unblk_valid(dunb_v), .unblk_msg(dunb_m), .unblk_ready(dunb_r),
        .wb_valid(dwb_v), .wb_msg(dwb_m), .wb_ready(dwb_r),
        .out_valid(dout_v), .out_msg(dout_m), .out_dst(dout_d), .out_ready(dout_r),
        .mem_req_o(mem_req_o), .mem_req_ready_i(mem_req_ready_i), .mem_resp_i(mem_resp_i));

    cmi_hub_dir_rx HRX (.clk, .rst_l, .int_out(int_out), .int_out_cr(int_out_cr),
        .req_valid(dreq_v), .req_msg(dreq_m), .req_ready(dreq_r),
        .unblk_valid(dunb_v), .unblk_msg(dunb_m), .unblk_ready(dunb_r),
        .wb_valid(dwb_v), .wb_msg(dwb_m), .wb_ready(dwb_r));

    ccd_chan_t dtx_in; assign dtx_in.valid = dout_v; assign dtx_in.msg = dout_m;
    cmi_msg_tx HTX (.clk, .rst_l, .msg_in(dtx_in), .msg_ready(dout_r),
        .dst_node(dout_d), .out(int_in), .credit_in(int_in_cr));

    // ---- core tiles (active cores get an agent + uplink/downlink; idle cores tie off) ----
    genvar gi;
    generate
        for (gi=0; gi<NACTIVE; gi++) begin : G_AGENT
            logic     a_dmd_v; ccd_msg_t a_dmd_m; logic a_dmd_r;
            logic     a_snp_v; ccd_msg_t a_snp_m; logic [NODE_ID_W-1:0] a_snp_d; logic a_snp_r;
            logic     a_sn_v;  ccd_msg_t a_sn_m;  logic a_sn_r;
            logic     a_rs_v;  ccd_msg_t a_rs_m;  logic a_rs_r;
            logic     a_ak_v;  ccd_msg_t a_ak_m;  logic a_ak_r;

            niigo_l1d_gg #(.CORE_ID(gi), .SETS(L1_SETS)) L1D (.clk, .rst_l,
                .c_req_valid(c_req_valid[gi]), .c_req_ready(c_req_ready[gi]),
                .c_req_op(c_req_op[gi]), .c_req_amo(c_req_amo[gi]),
                .c_req_waddr(c_req_waddr[gi]), .c_req_wdata(c_req_wdata[gi]),
                .c_resp_rdata(c_resp_rdata[gi]), .c_resp_sc_ok(c_resp_sc_ok[gi]),
                .dmd_valid(a_dmd_v), .dmd_msg(a_dmd_m), .dmd_ready(a_dmd_r),
                .snp_valid(a_snp_v), .snp_msg(a_snp_m), .snp_dst(a_snp_d), .snp_ready(a_snp_r),
                .snoop_valid(a_sn_v), .snoop_msg(a_sn_m), .snoop_ready(a_sn_r),
                .resp_valid(a_rs_v),  .resp_msg(a_rs_m),  .resp_ready(a_rs_r),
                .ack_valid(a_ak_v),   .ack_msg(a_ak_m),   .ack_ready(a_ak_r));

            cmi_agent_uplink #(.CORE_ID(gi)) UL (.clk, .rst_l,
                .dmd_valid(a_dmd_v), .dmd_msg(a_dmd_m), .dmd_ready(a_dmd_r),
                .snp_valid(a_snp_v), .snp_msg(a_snp_m), .snp_dst(a_snp_d), .snp_ready(a_snp_r),
                .loc_in(loc_in[gi]), .loc_in_cr(loc_in_cr[gi]));

            cmi_agent_downlink DL (.clk, .rst_l,
                .loc_out(loc_out[gi]), .loc_out_cr(loc_out_cr[gi]),
                .snoop_valid(a_sn_v), .snoop_msg(a_sn_m), .snoop_ready(a_sn_r),
                .resp_valid(a_rs_v),  .resp_msg(a_rs_m),  .resp_ready(a_rs_r),
                .ack_valid(a_ak_v),   .ack_msg(a_ak_m),   .ack_ready(a_ak_r));
        end
        for (gi=NACTIVE; gi<CORES; gi++) begin : G_IDLE
            assign loc_in[gi]    = '{default:'0};
            assign loc_out_cr[gi]= '0;
        end
    endgenerate
endmodule
`default_nettype wire
