/**
 * axi512_mux.sv  (FB1)
 *
 * A 2:1 AXI4 (512-bit) mux feeding the single sh_ddr DDR4 slave from either the
 * niigo core's AXI master (m0) or the host DMA_PCIS preload path (m1), selected
 * by `sel_m1`. This replaces the example's cl_axi_sc_2x2_wrapper SmartConnect IP
 * with from-source RTL, which is sound here because the two masters are mutually
 * exclusive in time: the host preloads DRAM over PCIS while the core is held in
 * reset (sel_m1=1), then releases reset and the core owns DRAM (sel_m1=0). The
 * non-selected master is held off (its A-channels see *ready=0) and its B/R
 * returns are gated, so there is never overlapping traffic across the switch.
 *
 * DDR-side widths (per the F2 shell sh_ddr slave): ID=16, ADDR=64, DATA=512,
 * LEN=8. The core master's narrower ID is zero-extended by the caller.
 *
 * NOTE (FB1 scope): structurally complete and synthesis-targeted. Full AXI
 * protocol correctness across the reset-edge switch is validated under the AWS
 * HDK simulation (FB1/FB3 gate), not in Verilator.
 */
`default_nettype none

module axi512_mux #(
    parameter int ID_W   = 16,
    parameter int ADDR_W = 64,
    parameter int DATA_W = 512,
    parameter int LEN_W  = 8
) (
    input  wire logic              sel_m1,   // 0: master 0 (core); 1: master 1 (PCIS)

    // ---- master 0 (core) ----
    input  wire logic [ID_W-1:0]   m0_awid,   input wire logic [ADDR_W-1:0] m0_awaddr,
    input  wire logic [LEN_W-1:0]  m0_awlen,  input wire logic [2:0] m0_awsize,
    input  wire logic [1:0]        m0_awburst, input wire logic m0_awvalid, output logic m0_awready,
    input  wire logic [DATA_W-1:0] m0_wdata,  input wire logic [DATA_W/8-1:0] m0_wstrb,
    input  wire logic              m0_wlast,  input wire logic m0_wvalid, output logic m0_wready,
    output logic [ID_W-1:0]        m0_bid,    output logic [1:0] m0_bresp,
    output logic                   m0_bvalid, input wire logic m0_bready,
    input  wire logic [ID_W-1:0]   m0_arid,   input wire logic [ADDR_W-1:0] m0_araddr,
    input  wire logic [LEN_W-1:0]  m0_arlen,  input wire logic [2:0] m0_arsize,
    input  wire logic [1:0]        m0_arburst, input wire logic m0_arvalid, output logic m0_arready,
    output logic [ID_W-1:0]        m0_rid,    output logic [DATA_W-1:0] m0_rdata,
    output logic [1:0]             m0_rresp,  output logic m0_rlast,
    output logic                   m0_rvalid, input wire logic m0_rready,

    // ---- master 1 (PCIS preload) ----
    input  wire logic [ID_W-1:0]   m1_awid,   input wire logic [ADDR_W-1:0] m1_awaddr,
    input  wire logic [LEN_W-1:0]  m1_awlen,  input wire logic [2:0] m1_awsize,
    input  wire logic [1:0]        m1_awburst, input wire logic m1_awvalid, output logic m1_awready,
    input  wire logic [DATA_W-1:0] m1_wdata,  input wire logic [DATA_W/8-1:0] m1_wstrb,
    input  wire logic              m1_wlast,  input wire logic m1_wvalid, output logic m1_wready,
    output logic [ID_W-1:0]        m1_bid,    output logic [1:0] m1_bresp,
    output logic                   m1_bvalid, input wire logic m1_bready,
    input  wire logic [ID_W-1:0]   m1_arid,   input wire logic [ADDR_W-1:0] m1_araddr,
    input  wire logic [LEN_W-1:0]  m1_arlen,  input wire logic [2:0] m1_arsize,
    input  wire logic [1:0]        m1_arburst, input wire logic m1_arvalid, output logic m1_arready,
    output logic [ID_W-1:0]        m1_rid,    output logic [DATA_W-1:0] m1_rdata,
    output logic [1:0]             m1_rresp,  output logic m1_rlast,
    output logic                   m1_rvalid, input wire logic m1_rready,

    // ---- slave (sh_ddr) ----
    output logic [ID_W-1:0]        s_awid,    output logic [ADDR_W-1:0] s_awaddr,
    output logic [LEN_W-1:0]       s_awlen,   output logic [2:0] s_awsize,
    output logic [1:0]             s_awburst, output logic s_awvalid, input wire logic s_awready,
    output logic [DATA_W-1:0]      s_wdata,   output logic [DATA_W/8-1:0] s_wstrb,
    output logic                   s_wlast,   output logic s_wvalid, input wire logic s_wready,
    input  wire logic [ID_W-1:0]   s_bid,     input wire logic [1:0] s_bresp,
    input  wire logic              s_bvalid,  output logic s_bready,
    output logic [ID_W-1:0]        s_arid,    output logic [ADDR_W-1:0] s_araddr,
    output logic [LEN_W-1:0]       s_arlen,   output logic [2:0] s_arsize,
    output logic [1:0]             s_arburst, output logic s_arvalid, input wire logic s_arready,
    input  wire logic [ID_W-1:0]   s_rid,     input wire logic [DATA_W-1:0] s_rdata,
    input  wire logic [1:0]        s_rresp,   input wire logic s_rlast,
    input  wire logic              s_rvalid,  output logic s_rready
);
    // A-channels and W: drive the slave from the selected master; the other
    // master sees *ready=0. B/R: route the slave's response to the selected
    // master only.
    always_comb begin
        // ---- write address ----
        s_awid    = sel_m1 ? m1_awid    : m0_awid;
        s_awaddr  = sel_m1 ? m1_awaddr  : m0_awaddr;
        s_awlen   = sel_m1 ? m1_awlen   : m0_awlen;
        s_awsize  = sel_m1 ? m1_awsize  : m0_awsize;
        s_awburst = sel_m1 ? m1_awburst : m0_awburst;
        s_awvalid = sel_m1 ? m1_awvalid : m0_awvalid;
        m0_awready = !sel_m1 && s_awready;
        m1_awready =  sel_m1 && s_awready;
        // ---- write data ----
        s_wdata   = sel_m1 ? m1_wdata   : m0_wdata;
        s_wstrb   = sel_m1 ? m1_wstrb   : m0_wstrb;
        s_wlast   = sel_m1 ? m1_wlast   : m0_wlast;
        s_wvalid  = sel_m1 ? m1_wvalid  : m0_wvalid;
        m0_wready = !sel_m1 && s_wready;
        m1_wready =  sel_m1 && s_wready;
        // ---- write response ----
        m0_bid = s_bid;  m1_bid = s_bid;
        m0_bresp = s_bresp;  m1_bresp = s_bresp;
        m0_bvalid = !sel_m1 && s_bvalid;
        m1_bvalid =  sel_m1 && s_bvalid;
        s_bready  = sel_m1 ? m1_bready : m0_bready;
        // ---- read address ----
        s_arid    = sel_m1 ? m1_arid    : m0_arid;
        s_araddr  = sel_m1 ? m1_araddr  : m0_araddr;
        s_arlen   = sel_m1 ? m1_arlen   : m0_arlen;
        s_arsize  = sel_m1 ? m1_arsize  : m0_arsize;
        s_arburst = sel_m1 ? m1_arburst : m0_arburst;
        s_arvalid = sel_m1 ? m1_arvalid : m0_arvalid;
        m0_arready = !sel_m1 && s_arready;
        m1_arready =  sel_m1 && s_arready;
        // ---- read data ----
        m0_rid = s_rid;  m1_rid = s_rid;
        m0_rdata = s_rdata;  m1_rdata = s_rdata;
        m0_rresp = s_rresp;  m1_rresp = s_rresp;
        m0_rlast = s_rlast;  m1_rlast = s_rlast;
        m0_rvalid = !sel_m1 && s_rvalid;
        m1_rvalid =  sel_m1 && s_rvalid;
        s_rready  = sel_m1 ? m1_rready : m0_rready;
    end

endmodule : axi512_mux

`default_nettype wire
