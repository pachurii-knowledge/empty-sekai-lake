/**
 * cl_niigo.sv  (FB1) -- AWS F2 Custom Logic top for the niigo SoC.
 *
 * Wraps niigo_soc (the 4-wide OoO RV64G core + L1I/L1D + NMI bus + AXI4-512
 * master) into the F2 shell. Connects:
 *   - clk_main_a0 / rst_main_n            -> the SoC clock + a host-gated reset
 *   - OCL AXI-Lite (AppPF BAR0)           -> ocl_csr (control + vUART + debug)
 *   - DMA_PCIS 512b slave (host->DRAM)    -> reset-gated AXI mux -> sh_ddr
 *   - the core's AXI4-512 master          -> the same mux -> sh_ddr (DDR4)
 *   - vUART byte streams                  -> TX/RX FIFOs <-> ocl_csr
 *   - the commit-stage debug probe        -> ocl_csr debug block
 *
 * Bring-up model (host runtime, see fpga/README.md): hold the core in reset
 * (CTRL.go=0 => soc_rst_l low), preload the kernel+fs image into DRAM over
 * DMA_PCIS, then write CTRL.go=1. The AXI mux hands DDR to PCIS while the core
 * is in reset and to the core afterwards -- the two never overlap, matching the
 * plan's "host loads memory only while the core is in reset" DMA-coherence
 * non-goal. Device space (CLINT/PLIC/UART) stays inside the core and never
 * reaches AXI; only cacheable traffic does.
 *
 * SCOPE / VERIFICATION: this top instantiates F2 shell library blocks (sh_ddr,
 * lib_pipe, xpm_cdc_async_rst) and can only be elaborated/synthesized inside the
 * AWS HDK build (fpga/README.md). niigo_soc itself -- with the new vUART/debug
 * ports -- is covered by the generic OOC synth flow (fpga/synth/run_soc.sh), and
 * ocl_csr + uart_host_fifo by a standalone Verilator unit test
 * (tests/fpga/tb_ocl_csr.sv). The unused-interface tie-offs below cover the set
 * this design needs; cross-check the HBM-APB / PCIe-transceiver tie-offs against
 * the HDK CL_TEMPLATE when wiring the actual AFI build.
 */
`include "ooo_types.vh"
`include "niigo_mem.vh"
`include "cl_id_defines.vh"

`default_nettype none

module cl_niigo #(
    parameter EN_DDR = 1
) (
`include "cl_ports.vh"
);
    import NIIGO_Mem::AXI_ADDR_W, NIIGO_Mem::AXI_DATA_W;
    import NIIGO_Mem::AXI_ID_W,   NIIGO_Mem::AXI_STRB_W;
    import OOO_Types::debug_probe_t;

    localparam int NUM_CFG_STGS_CL_DDR_ATG = 8;
    localparam int DDR_ID_W = 16;   // sh_ddr slave AXI ID width

    // ================= core reset control =================
    // ocl_csr drives go / soft_reset (levels). The SoC runs only when the shell
    // is out of reset, the host has asserted go, and soft-reset is clear.
    logic core_go, core_soft_reset, soc_rst_l, core_in_reset, core_halted;
    assign soc_rst_l     = rst_main_n & core_go & ~core_soft_reset;
    assign core_in_reset = ~soc_rst_l;

    // ================= vUART FIFOs =================
    logic       vuart_tx_valid; logic [7:0] vuart_tx_byte;
    logic       vuart_rx_valid; logic [7:0] vuart_rx_byte; logic vuart_rx_pop;
    logic       tx_empty, tx_full, tx_pop;  logic [7:0] tx_dout;  logic [8:0] tx_count;
    logic       rx_empty, rx_full, rx_push;  logic [7:0] rx_dout, rx_din;  logic [8:0] rx_count;

    uart_host_fifo #(.WIDTH(8), .DEPTH(256)) TX_FIFO (
        .clk(clk_main_a0), .rst_l(rst_main_n),
        .wr_en(vuart_tx_valid), .wr_data(vuart_tx_byte), .full(tx_full),
        .rd_en(tx_pop), .rd_data(tx_dout), .empty(tx_empty), .count(tx_count));
    uart_host_fifo #(.WIDTH(8), .DEPTH(256)) RX_FIFO (
        .clk(clk_main_a0), .rst_l(rst_main_n),
        .wr_en(rx_push), .wr_data(rx_din), .full(rx_full),
        .rd_en(vuart_rx_pop), .rd_data(rx_dout), .empty(rx_empty), .count(rx_count));
    assign vuart_rx_valid = !rx_empty;
    assign vuart_rx_byte  = rx_dout;

    // ================= OCL control plane =================
    debug_probe_t dbg_probe;

    ocl_csr OCL (
        .clk(clk_main_a0), .rst_main_n(rst_main_n),
        .s_awaddr(ocl_cl_awaddr), .s_awvalid(ocl_cl_awvalid), .s_awready(cl_ocl_awready),
        .s_wdata(ocl_cl_wdata), .s_wstrb(ocl_cl_wstrb), .s_wvalid(ocl_cl_wvalid),
        .s_wready(cl_ocl_wready),
        .s_bresp(cl_ocl_bresp), .s_bvalid(cl_ocl_bvalid), .s_bready(ocl_cl_bready),
        .s_araddr(ocl_cl_araddr), .s_arvalid(ocl_cl_arvalid), .s_arready(cl_ocl_arready),
        .s_rdata(cl_ocl_rdata), .s_rresp(cl_ocl_rresp), .s_rvalid(cl_ocl_rvalid),
        .s_rready(ocl_cl_rready),
        .core_go(core_go), .core_soft_reset(core_soft_reset),
        .core_in_reset(core_in_reset), .core_halted(core_halted),
        .tx_fifo_empty(tx_empty), .tx_fifo_full(tx_full), .tx_fifo_dout(tx_dout),
        .tx_fifo_count(16'(tx_count)), .tx_fifo_pop(tx_pop),
        .rx_fifo_empty(rx_empty), .rx_fifo_full(rx_full), .rx_fifo_count(16'(rx_count)),
        .rx_fifo_push(rx_push), .rx_fifo_din(rx_din),
        .dbg_probe(dbg_probe)
    );

    // ================= niigo_soc + its AXI master =================
    logic [AXI_ADDR_W-1:0] c_awaddr, c_araddr;
    logic [AXI_ID_W-1:0]   c_awid, c_arid, c_bid, c_rid;
    logic [7:0]            c_awlen, c_arlen;
    logic [2:0]            c_awsize, c_arsize;
    logic [1:0]            c_awburst, c_arburst, c_bresp, c_rresp;
    logic                  c_awvalid, c_awready, c_wvalid, c_wready, c_wlast;
    logic                  c_bvalid, c_bready, c_arvalid, c_arready, c_rvalid, c_rready, c_rlast;
    logic [AXI_DATA_W-1:0] c_wdata, c_rdata;
    logic [AXI_STRB_W-1:0] c_wstrb;

    niigo_soc SOC (
        .clk(clk_main_a0), .rst_l(soc_rst_l), .halted(core_halted),
        .m_axi_awvalid(c_awvalid), .m_axi_awready(c_awready), .m_axi_awaddr(c_awaddr),
        .m_axi_awid(c_awid), .m_axi_awlen(c_awlen), .m_axi_awsize(c_awsize),
        .m_axi_awburst(c_awburst),
        .m_axi_wvalid(c_wvalid), .m_axi_wready(c_wready), .m_axi_wdata(c_wdata),
        .m_axi_wstrb(c_wstrb), .m_axi_wlast(c_wlast),
        .m_axi_bvalid(c_bvalid), .m_axi_bready(c_bready), .m_axi_bid(c_bid), .m_axi_bresp(c_bresp),
        .m_axi_arvalid(c_arvalid), .m_axi_arready(c_arready), .m_axi_araddr(c_araddr),
        .m_axi_arid(c_arid), .m_axi_arlen(c_arlen), .m_axi_arsize(c_arsize),
        .m_axi_arburst(c_arburst),
        .m_axi_rvalid(c_rvalid), .m_axi_rready(c_rready), .m_axi_rid(c_rid),
        .m_axi_rdata(c_rdata), .m_axi_rresp(c_rresp), .m_axi_rlast(c_rlast),
        .vuart_tx_valid(vuart_tx_valid), .vuart_tx_byte(vuart_tx_byte),
        .vuart_rx_valid(vuart_rx_valid), .vuart_rx_byte(vuart_rx_byte),
        .vuart_rx_pop(vuart_rx_pop),
        .dbg_probe(dbg_probe)
    );

    // ================= AXI mux: {core, PCIS} -> sh_ddr =================
    // DDR-side AXI (mux slave port) wires.
    logic [DDR_ID_W-1:0]   d_awid, d_arid, d_bid, d_rid;
    logic [AXI_ADDR_W-1:0] d_awaddr, d_araddr;
    logic [7:0]            d_awlen, d_arlen;
    logic [2:0]            d_awsize, d_arsize;
    logic [1:0]            d_awburst, d_arburst, d_bresp, d_rresp;
    logic                  d_awvalid, d_awready, d_wvalid, d_wready, d_wlast;
    logic                  d_bvalid, d_bready, d_arvalid, d_arready, d_rvalid, d_rready, d_rlast;
    logic [AXI_DATA_W-1:0] d_wdata, d_rdata;
    logic [AXI_STRB_W-1:0] d_wstrb;
    logic                  ddr_ready;

    axi512_mux #(.ID_W(DDR_ID_W), .ADDR_W(AXI_ADDR_W), .DATA_W(AXI_DATA_W), .LEN_W(8)) DDR_MUX (
        .sel_m1(core_in_reset),
        // ---- m0: core master (ID zero-extended to the DDR width) ----
        .m0_awid({{(DDR_ID_W-AXI_ID_W){1'b0}}, c_awid}), .m0_awaddr(c_awaddr),
        .m0_awlen(c_awlen), .m0_awsize(c_awsize), .m0_awburst(c_awburst),
        .m0_awvalid(c_awvalid), .m0_awready(c_awready),
        .m0_wdata(c_wdata), .m0_wstrb(c_wstrb), .m0_wlast(c_wlast),
        .m0_wvalid(c_wvalid), .m0_wready(c_wready),
        .m0_bid(/*open*/), .m0_bresp(c_bresp), .m0_bvalid(c_bvalid), .m0_bready(c_bready),
        .m0_arid({{(DDR_ID_W-AXI_ID_W){1'b0}}, c_arid}), .m0_araddr(c_araddr),
        .m0_arlen(c_arlen), .m0_arsize(c_arsize), .m0_arburst(c_arburst),
        .m0_arvalid(c_arvalid), .m0_arready(c_arready),
        .m0_rid(/*open*/), .m0_rdata(c_rdata), .m0_rresp(c_rresp), .m0_rlast(c_rlast),
        .m0_rvalid(c_rvalid), .m0_rready(c_rready),
        // ---- m1: host DMA_PCIS preload ----
        .m1_awid(sh_cl_dma_pcis_awid), .m1_awaddr(sh_cl_dma_pcis_awaddr),
        .m1_awlen(sh_cl_dma_pcis_awlen), .m1_awsize(sh_cl_dma_pcis_awsize),
        .m1_awburst(sh_cl_dma_pcis_awburst), .m1_awvalid(sh_cl_dma_pcis_awvalid),
        .m1_awready(cl_sh_dma_pcis_awready),
        .m1_wdata(sh_cl_dma_pcis_wdata), .m1_wstrb(sh_cl_dma_pcis_wstrb),
        .m1_wlast(sh_cl_dma_pcis_wlast), .m1_wvalid(sh_cl_dma_pcis_wvalid),
        .m1_wready(cl_sh_dma_pcis_wready),
        .m1_bid(cl_sh_dma_pcis_bid), .m1_bresp(cl_sh_dma_pcis_bresp),
        .m1_bvalid(cl_sh_dma_pcis_bvalid), .m1_bready(sh_cl_dma_pcis_bready),
        .m1_arid(sh_cl_dma_pcis_arid), .m1_araddr(sh_cl_dma_pcis_araddr),
        .m1_arlen(sh_cl_dma_pcis_arlen), .m1_arsize(sh_cl_dma_pcis_arsize),
        .m1_arburst(sh_cl_dma_pcis_arburst), .m1_arvalid(sh_cl_dma_pcis_arvalid),
        .m1_arready(cl_sh_dma_pcis_arready),
        .m1_rid(cl_sh_dma_pcis_rid), .m1_rdata(cl_sh_dma_pcis_rdata),
        .m1_rresp(cl_sh_dma_pcis_rresp), .m1_rlast(cl_sh_dma_pcis_rlast),
        .m1_rvalid(cl_sh_dma_pcis_rvalid), .m1_rready(sh_cl_dma_pcis_rready),
        // ---- slave: sh_ddr ----
        .s_awid(d_awid), .s_awaddr(d_awaddr), .s_awlen(d_awlen), .s_awsize(d_awsize),
        .s_awburst(d_awburst), .s_awvalid(d_awvalid), .s_awready(d_awready),
        .s_wdata(d_wdata), .s_wstrb(d_wstrb), .s_wlast(d_wlast), .s_wvalid(d_wvalid),
        .s_wready(d_wready),
        .s_bid(d_bid), .s_bresp(d_bresp), .s_bvalid(d_bvalid), .s_bready(d_bready),
        .s_arid(d_arid), .s_araddr(d_araddr), .s_arlen(d_arlen), .s_arsize(d_arsize),
        .s_arburst(d_arburst), .s_arvalid(d_arvalid), .s_arready(d_arready),
        .s_rid(d_rid), .s_rdata(d_rdata), .s_rresp(d_rresp), .s_rlast(d_rlast),
        .s_rvalid(d_rvalid), .s_rready(d_rready)
    );
    assign cl_sh_dma_pcis_ruser = 64'b0;

    // ================= DDR4 controller (sh_ddr) =================
    logic        ddr_sync_rst_n;
    logic  [7:0] sh_ddr_stat_addr_q;  logic sh_ddr_stat_wr_q, sh_ddr_stat_rd_q;
    logic [31:0] sh_ddr_stat_wdata_q; logic ddr_sh_stat_ack_q; logic [31:0] ddr_sh_stat_rdata_q;
    logic  [7:0] ddr_sh_stat_int_q;

    xpm_cdc_async_rst CDC_ASYNC_RST_N_DDR (
        .src_arst(rst_main_n), .dest_clk(clk_main_a0), .dest_arst(ddr_sync_rst_n));

    lib_pipe #(.WIDTH(1+1+8+32), .STAGES(NUM_CFG_STGS_CL_DDR_ATG)) PIPE_DDR_STAT0 (
        .clk(clk_main_a0), .rst_n(ddr_sync_rst_n),
        .in_bus ({sh_cl_ddr_stat_wr, sh_cl_ddr_stat_rd, sh_cl_ddr_stat_addr, sh_cl_ddr_stat_wdata}),
        .out_bus({sh_ddr_stat_wr_q, sh_ddr_stat_rd_q, sh_ddr_stat_addr_q, sh_ddr_stat_wdata_q}));
    lib_pipe #(.WIDTH(1+8+32), .STAGES(NUM_CFG_STGS_CL_DDR_ATG)) PIPE_DDR_STAT_ACK0 (
        .clk(clk_main_a0), .rst_n(ddr_sync_rst_n),
        .in_bus ({ddr_sh_stat_ack_q, ddr_sh_stat_int_q, ddr_sh_stat_rdata_q}),
        .out_bus({cl_sh_ddr_stat_ack, cl_sh_ddr_stat_int, cl_sh_ddr_stat_rdata}));

    sh_ddr #(.DDR_PRESENT(EN_DDR)) SH_DDR (
        .clk(clk_main_a0), .rst_n(ddr_sync_rst_n),
        .stat_clk(clk_main_a0), .stat_rst_n(ddr_sync_rst_n),
        .CLK_DIMM_DP(CLK_DIMM_DP), .CLK_DIMM_DN(CLK_DIMM_DN),
        .M_ACT_N(M_ACT_N), .M_MA(M_MA), .M_BA(M_BA), .M_BG(M_BG), .M_CKE(M_CKE),
        .M_ODT(M_ODT), .M_CS_N(M_CS_N), .M_CLK_DN(M_CLK_DN), .M_CLK_DP(M_CLK_DP),
        .M_PAR(M_PAR), .M_DQ(M_DQ), .M_ECC(M_ECC), .M_DQS_DP(M_DQS_DP), .M_DQS_DN(M_DQS_DN),
        .cl_RST_DIMM_N(RST_DIMM_N),
        .cl_sh_ddr_axi_awid(d_awid), .cl_sh_ddr_axi_awaddr(d_awaddr),
        .cl_sh_ddr_axi_awlen(d_awlen), .cl_sh_ddr_axi_awsize(d_awsize),
        .cl_sh_ddr_axi_awvalid(d_awvalid), .cl_sh_ddr_axi_awburst(d_awburst),
        .cl_sh_ddr_axi_awuser(1'd0), .cl_sh_ddr_axi_awready(d_awready),
        .cl_sh_ddr_axi_wdata(d_wdata), .cl_sh_ddr_axi_wstrb(d_wstrb),
        .cl_sh_ddr_axi_wlast(d_wlast), .cl_sh_ddr_axi_wvalid(d_wvalid),
        .cl_sh_ddr_axi_wready(d_wready),
        .cl_sh_ddr_axi_bid(d_bid), .cl_sh_ddr_axi_bresp(d_bresp),
        .cl_sh_ddr_axi_bvalid(d_bvalid), .cl_sh_ddr_axi_bready(d_bready),
        .cl_sh_ddr_axi_arid(d_arid), .cl_sh_ddr_axi_araddr(d_araddr),
        .cl_sh_ddr_axi_arlen(d_arlen), .cl_sh_ddr_axi_arsize(d_arsize),
        .cl_sh_ddr_axi_arvalid(d_arvalid), .cl_sh_ddr_axi_arburst(d_arburst),
        .cl_sh_ddr_axi_aruser(1'd0), .cl_sh_ddr_axi_arready(d_arready),
        .cl_sh_ddr_axi_rid(d_rid), .cl_sh_ddr_axi_rdata(d_rdata),
        .cl_sh_ddr_axi_rresp(d_rresp), .cl_sh_ddr_axi_rlast(d_rlast),
        .cl_sh_ddr_axi_rvalid(d_rvalid), .cl_sh_ddr_axi_rready(d_rready),
        .sh_ddr_stat_bus_addr(sh_ddr_stat_addr_q), .sh_ddr_stat_bus_wdata(sh_ddr_stat_wdata_q),
        .sh_ddr_stat_bus_wr(sh_ddr_stat_wr_q), .sh_ddr_stat_bus_rd(sh_ddr_stat_rd_q),
        .sh_ddr_stat_bus_ack(ddr_sh_stat_ack_q), .sh_ddr_stat_bus_rdata(ddr_sh_stat_rdata_q),
        .ddr_sh_stat_int(ddr_sh_stat_int_q), .sh_cl_ddr_is_ready(ddr_ready)
    );

    // ================= unused shell interfaces (tie-offs) =================
    // Identification / status.
    assign cl_sh_id0          = `CL_SH_ID0;
    assign cl_sh_id1          = `CL_SH_ID1;
    assign cl_sh_status0      = 32'b0;
    assign cl_sh_status1      = 32'b0;
    assign cl_sh_status2      = 32'b0;
    assign cl_sh_status_vled  = {13'b0, ddr_ready, core_halted, ~core_in_reset};
    assign cl_sh_flr_done     = sh_cl_flr_assert;     // ack FLR immediately
    assign cl_sh_dma_rd_full  = 1'b0;
    assign cl_sh_dma_wr_full  = 1'b0;
    assign cl_sh_apppf_irq_req = 16'b0;               // host polls vUART; no MSI-X

    // PCIe master (host DMA from CL) -- unused.
    assign cl_sh_pcim_awvalid = 1'b0;  assign cl_sh_pcim_awid    = 16'b0;
    assign cl_sh_pcim_awaddr  = 64'b0; assign cl_sh_pcim_awlen   = 8'b0;
    assign cl_sh_pcim_awsize  = 3'b0;  assign cl_sh_pcim_awburst = 2'b01;
    assign cl_sh_pcim_awcache = 4'b0;  assign cl_sh_pcim_awlock  = 1'b0;
    assign cl_sh_pcim_awprot  = 3'b0;  assign cl_sh_pcim_awqos   = 4'b0;
    assign cl_sh_pcim_awuser  = 55'b0;
    assign cl_sh_pcim_wvalid  = 1'b0;  assign cl_sh_pcim_wid     = 16'b0;
    assign cl_sh_pcim_wdata   = 512'b0; assign cl_sh_pcim_wstrb  = 64'b0;
    assign cl_sh_pcim_wlast   = 1'b0;  assign cl_sh_pcim_wuser   = 64'b0;
    assign cl_sh_pcim_bready  = 1'b1;
    assign cl_sh_pcim_arvalid = 1'b0;  assign cl_sh_pcim_arid    = 16'b0;
    assign cl_sh_pcim_araddr  = 64'b0; assign cl_sh_pcim_arlen   = 8'b0;
    assign cl_sh_pcim_arsize  = 3'b0;  assign cl_sh_pcim_arburst = 2'b01;
    assign cl_sh_pcim_arcache = 4'b0;  assign cl_sh_pcim_arlock  = 1'b0;
    assign cl_sh_pcim_arprot  = 3'b0;  assign cl_sh_pcim_arqos   = 4'b0;
    assign cl_sh_pcim_aruser  = 55'b0;
    assign cl_sh_pcim_rready  = 1'b1;

    // SDA management AXI-Lite -- unused (idle slave).
    assign cl_sda_awready = 1'b0;  assign cl_sda_wready  = 1'b0;
    assign cl_sda_bresp   = 2'b0;  assign cl_sda_bvalid  = 1'b0;
    assign cl_sda_arready = 1'b0;  assign cl_sda_rdata   = 32'b0;
    assign cl_sda_rresp   = 2'b0;  assign cl_sda_rvalid  = 1'b0;

    // Virtual JTAG debug bridge -- unused.
    assign tdo = 1'b0;

    // HBM APB monitor + HBM not used (DDR build). Tie the APB returns idle.
    assign hbm_apb_paddr_0 = 22'b0; assign hbm_apb_pprot_0 = 3'b0;
    assign hbm_apb_psel_0 = 1'b0; assign hbm_apb_penable_0 = 1'b0;
    assign hbm_apb_pwrite_0 = 1'b0; assign hbm_apb_pwdata_0 = 32'b0;
    assign hbm_apb_pstrb_0 = 4'b0; assign hbm_apb_pready_0 = 1'b1;
    assign hbm_apb_prdata_0 = 32'b0; assign hbm_apb_pslverr_0 = 1'b0;
    assign hbm_apb_paddr_1 = 22'b0; assign hbm_apb_pprot_1 = 3'b0;
    assign hbm_apb_psel_1 = 1'b0; assign hbm_apb_penable_1 = 1'b0;
    assign hbm_apb_pwrite_1 = 1'b0; assign hbm_apb_pwdata_1 = 32'b0;
    assign hbm_apb_pstrb_1 = 4'b0; assign hbm_apb_pready_1 = 1'b1;
    assign hbm_apb_prdata_1 = 32'b0; assign hbm_apb_pslverr_1 = 1'b0;

    // PCIe endpoint/root-port transceivers -- not driven by this CL.
    assign PCIE_EP_TXP = 8'b0; assign PCIE_EP_TXN = 8'b0;
    assign PCIE_RP_PERSTN = 1'b1; assign PCIE_RP_TXP = 8'b0; assign PCIE_RP_TXN = 8'b0;

endmodule : cl_niigo

`default_nettype wire
