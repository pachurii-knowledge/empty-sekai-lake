/**
 * niigo_ccd_wheel.sv  --  M3b two-core MOESI CCD over the real wheel NoC
 *
 * The M1 coherence subsystem (niigo_l1d_moesi agents + niigo_dir) re-floored onto the M3 wheel
 * fabric (plans/multicore-ccd.md §W): the agents and the directory no longer share direct
 * full-line ccd links -- they exchange the canonical 128 b multi-flit CMI form across the radix-4
 * core routers (ring C0-C1-C3-C2) via the cmi_msg_tx/cmi_msg_rx SerDes. A drop-in replacement for
 * niigo_ccd_top (identical port list), so tb_niigo_ccd_wheel runs the SAME S1-S6 coherence program
 * -- if it stays green the fabric is transparent to the protocol (the load-bearing M3 check, §V/CT-10).
 *
 * Topology here: each agent attaches to its core router's Local port; the directory attaches to the
 * four Spoke ports through per-spoke SerDes (the dir's round-robin pick IS the hub's 4->1
 * serialisation point, §W.3b). The radix-5 hub router + ring cache-to-cache + the grant-and-go
 * directory that drives data over the ring are M3c; M1 forwarding stays directory-mediated, so all
 * traffic rides the spokes (the ring is exercised at the flit level by tb_cmi_wheel). M1 is
 * blocking + serialised, so every wheel endpoint sources <=1 message at a time -> the simple
 * single-message SerDes suffices.
 */
`include "niigo_mem.vh"
`include "niigo_cmi.vh"
`include "niigo_ccd_m1.vh"
`default_nettype none
module niigo_ccd_wheel
    import RISCV_ISA::XLEN;
    import RISCV_UArch::MEMORY_ADDR_WIDTH;
    import NIIGO_Mem::*;
    import NIIGO_CMI::*;
    import NIIGO_CCD_M1::*;
#(
    parameter int NACTIVE  = 2,
    parameter int DIR_SETS = 8,
    parameter int L1_SETS  = 8
)(
    input  wire logic clk,
    input  wire logic rst_l,
    // ---- per-core LSQ ports (word-granular; blocking) ----
    input  wire logic                          c_req_valid [NACTIVE],
    output logic                               c_req_ready [NACTIVE],
    input  wire l1_core_op_e                   c_req_op    [NACTIVE],
    input  wire l1_amo_op_e                    c_req_amo   [NACTIVE],
    input  wire logic [MEMORY_ADDR_WIDTH-1:0]  c_req_waddr [NACTIVE],
    input  wire logic [XLEN-1:0]               c_req_wdata [NACTIVE],
    output logic [XLEN-1:0]                     c_resp_rdata[NACTIVE],
    output logic                               c_resp_sc_ok[NACTIVE],
    // ---- backing memory (NMI master) ----
    output nmi_req_t   mem_req_o,
    input  wire logic  mem_req_ready_i,
    input  nmi_resp_t  mem_resp_i
);
    localparam int CORES = NUM_CORES;     // ring is a 4-node cycle; dir sized to 4
    localparam int NV    = NUM_VC_PHYS;

    // ---- core-router link nets [core][port] (0=W,1=E,2=Spoke,3=Local) ----
    cmi_link_t       ci  [CORES][4];   logic [NV-1:0] cicr[CORES][4];
    cmi_link_t       co  [CORES][4];   logic [NV-1:0] cocr[CORES][4];

    // ---- directory <-> per-spoke SerDes channels ----
    ccd_chan_t dir_up   [CORES]; logic dir_up_rdy  [CORES];
    ccd_chan_t dir_down [CORES]; logic dir_down_rdy[CORES];

    niigo_dir #(.CORES(CORES), .DIR_SETS(DIR_SETS)) DIR (
        .clk, .rst_l,
        .up_i(dir_up), .up_ready_o(dir_up_rdy),
        .down_o(dir_down), .down_ready_i(dir_down_rdy),
        .mem_req_o(mem_req_o), .mem_req_ready_i(mem_req_ready_i), .mem_resp_i(mem_resp_i)
    );

    genvar gc;
    generate
        for (gc = 0; gc < CORES; gc++) begin : G_NODE
            // ---- the core router (radix-4) ----
            cmi_router #(.NP(4), .NODE_ID(gc),
                         .P_RW(0), .P_RE(1), .P_SP(2), .P_LO(3), .P_IN(4))
            RT (.clk, .rst_l, .in(ci[gc]), .in_cr(cicr[gc]), .out(co[gc]), .out_cr(cocr[gc]));

            // ---- ring links ----
            assign ci[gc][0]   = co[cmi_ring_w(gc[CORE_ID_W-1:0])][1];
            assign ci[gc][1]   = co[cmi_ring_e(gc[CORE_ID_W-1:0])][0];
            assign cocr[gc][0] = cicr[cmi_ring_w(gc[CORE_ID_W-1:0])][1];
            assign cocr[gc][1] = cicr[cmi_ring_e(gc[CORE_ID_W-1:0])][0];

            // ---- Spoke <-> directory SerDes (all cores) ----
            cmi_msg_rx DRX (.clk, .rst_l, .in(co[gc][2]), .credit_out(cocr[gc][2]),
                .out_valid(dir_up[gc].valid), .out_msg(dir_up[gc].msg),
                .out_src(), .out_ready(dir_up_rdy[gc]));
            cmi_msg_tx DTX (.clk, .rst_l, .msg_in(dir_down[gc]), .msg_ready(dir_down_rdy[gc]),
                .dst_node(cmi_core_node(gc[CORE_ID_W-1:0])),
                .out(ci[gc][2]), .credit_in(cicr[gc][2]));
        end

        // ---- agents on the Local ports of the active cores ----
        for (gc = 0; gc < NACTIVE; gc++) begin : G_AGENT
            ccd_chan_t a_up, a_down; logic a_up_rdy, a_down_rdy;
            niigo_l1d_moesi #(.CORE_ID(gc), .SETS(L1_SETS)) L1D (
                .clk, .rst_l,
                .c_req_valid (c_req_valid [gc]), .c_req_ready (c_req_ready [gc]),
                .c_req_op    (c_req_op    [gc]), .c_req_amo   (c_req_amo   [gc]),
                .c_req_waddr (c_req_waddr [gc]), .c_req_wdata (c_req_wdata [gc]),
                .c_resp_rdata(c_resp_rdata[gc]), .c_resp_sc_ok(c_resp_sc_ok[gc]),
                .up_o(a_up), .up_ready_i(a_up_rdy),
                .down_i(a_down), .down_ready_o(a_down_rdy)
            );
            // up: agent -> Local-in (dst = HUB)
            cmi_msg_tx ATX (.clk, .rst_l, .msg_in(a_up), .msg_ready(a_up_rdy),
                .dst_node(CMI_HUB_ID), .out(ci[gc][3]), .credit_in(cicr[gc][3]));
            // down: Local-out -> agent
            cmi_msg_rx ARX (.clk, .rst_l, .in(co[gc][3]), .credit_out(cocr[gc][3]),
                .out_valid(a_down.valid), .out_msg(a_down.msg),
                .out_src(), .out_ready(a_down_rdy));
        end

        // ---- idle cores: no agent on the Local port ----
        for (gc = NACTIVE; gc < CORES; gc++) begin : G_IDLE
            assign ci[gc][3]   = '0;
            assign cocr[gc][3] = '0;
        end
    endgenerate
endmodule
`default_nettype wire
