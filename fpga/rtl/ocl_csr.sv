/**
 * ocl_csr.sv  (FB1)
 *
 * The niigo OCL control plane: a 32-bit AXI4-Lite slave (AppPF BAR0) that the
 * host pokes to bring the core up and to observe it. Three jobs:
 *
 *   1. Control: release/hold the core reset (CTRL.go / CTRL.soft_reset) and a
 *      counter clear, plus a STATUS read (halted / in-reset / FIFO flags).
 *   2. vUART: the console. UART_TX read pops a byte the core printed; UART_RX
 *      write pushes a host keystroke. FIFO levels in the _ST registers.
 *   3. Debug observability (FB1 ask): a post-mortem window into the OoO core for
 *      Linux/xv6 bring-up, fed by the commit-stage debug_probe (zero functional
 *      change to the core). Free-running cycle + retired-instruction counters
 *      (hang detection), a 16-deep committed-PC ring ("where did it die?"), a
 *      shadow architectural register file (read a0/sp/... at the failure), a
 *      last-trap log {cause,epc,tval} + trap count, and the L1 HPM event counts.
 *      Host memory peek is free over the same DMA_PCIS path used for preload.
 *
 * Single outstanding AXI-Lite transaction; everything is in clk_main_a0, the
 * same domain as the core, so the probe taps need no CDC. Register map (byte
 * offsets, all 32-bit; XLEN-wide values split LO/HI, HI reads 0 on RV32):
 *
 *   0x00 CTRL    RW  [0]go(level)  [1]soft_reset(level)  [2]clear_counters(W1P)
 *   0x04 STATUS  RO  [0]halted [1]in_reset [2]tx_empty [3]rx_full [4]trap_seen
 *   0x08 BUILD_ID RO 0x4E49_4731 ("NIG1")
 *   0x0C XLEN_INFO RO [7:0]=XLEN  [8]=paging
 *   0x10/0x14 CYCLE_LO/HI       RO  run-cycle counter (gated on !in_reset)
 *   0x18/0x1C INSTRET_LO/HI     RO  retired-instruction counter
 *   0x20 UART_TX RO  read pops TX FIFO: [7:0]=byte [8]=valid
 *   0x24 UART_TX_ST RO  [15:0]=count [16]=empty [17]=full
 *   0x28 UART_RX WO  write [7:0]=byte -> push to RX FIFO
 *   0x2C UART_RX_ST RO  [15:0]=count [16]=empty [17]=full
 *   0x30 HPM_L1I_MISS RO   0x34 HPM_L1D_MISS RO   0x38 HPM_L1D_WB RO
 *   0x40 DBG_PCIDX RW [3:0]=ring read index ; [11:8]=ring head (RO field)
 *   0x44/0x48 DBG_PC_LO/HI  RO  committed-PC ring[idx]
 *   0x50 DBG_TRAP_COUNT RO
 *   0x54 DBG_TRAP_CAUSE RO  [4:0]=cause [8]=is_int
 *   0x58/0x5C DBG_TRAP_EPC_LO/HI   RO
 *   0x60/0x64 DBG_TRAP_TVAL_LO/HI  RO
 *   0x80 DBG_REGSEL RW [4:0]=arch reg index
 *   0x84/0x88 DBG_REG_LO/HI  RO  shadow arch reg[sel]
 */
`include "ooo_types.vh"

`default_nettype none

module ocl_csr
    import OOO_Types::XLEN, OOO_Types::OOO_WIDTH, OOO_Types::debug_probe_t;
(
    input  wire logic        clk,
    input  wire logic        rst_main_n,        // active-low (shell main reset)

    // ---- AXI4-Lite slave (OCL / AppPF BAR0) ----
    input  wire logic [31:0] s_awaddr,
    input  wire logic        s_awvalid,
    output logic             s_awready,
    input  wire logic [31:0] s_wdata,
    input  wire logic [3:0]  s_wstrb,
    input  wire logic        s_wvalid,
    output logic             s_wready,
    output logic [1:0]       s_bresp,
    output logic             s_bvalid,
    input  wire logic        s_bready,
    input  wire logic [31:0] s_araddr,
    input  wire logic        s_arvalid,
    output logic             s_arready,
    output logic [31:0]      s_rdata,
    output logic [1:0]       s_rresp,
    output logic             s_rvalid,
    input  wire logic        s_rready,

    // ---- control plane ----
    output logic             core_go,           // level: release core reset
    output logic             core_soft_reset,   // level: hold core in reset
    input  wire logic        core_in_reset,     // ~soc_rst_l (from cl_niigo)
    input  wire logic        core_halted,

    // ---- vUART FIFOs (instantiated in cl_niigo) ----
    input  wire logic        tx_fifo_empty,
    input  wire logic        tx_fifo_full,
    input  wire logic [7:0]  tx_fifo_dout,
    input  wire logic [15:0] tx_fifo_count,
    output logic             tx_fifo_pop,
    input  wire logic        rx_fifo_empty,
    input  wire logic        rx_fifo_full,
    input  wire logic [15:0] rx_fifo_count,
    output logic             rx_fifo_push,
    output logic [7:0]       rx_fifo_din,

    // ---- commit-stage debug probe (from niigo_soc) ----
    input  debug_probe_t     dbg_probe
);
    // ---- word offsets (addr[7:2]) ----
    localparam logic [5:0]
        W_CTRL=6'h00, W_STATUS=6'h01, W_BUILDID=6'h02, W_XLEN=6'h03,
        W_CYCLO=6'h04, W_CYCHI=6'h05, W_INSTLO=6'h06, W_INSTHI=6'h07,
        W_UTX=6'h08, W_UTXST=6'h09, W_URX=6'h0A, W_URXST=6'h0B,
        W_HPMI=6'h0C, W_HPMD=6'h0D, W_HPMWB=6'h0E,
        W_PCIDX=6'h10, W_PCLO=6'h11, W_PCHI=6'h12,
        W_TRAPCNT=6'h14, W_TRAPCAUSE=6'h15, W_EPCLO=6'h16, W_EPCHI=6'h17,
        W_TVALLO=6'h18, W_TVALHI=6'h19,
        W_REGSEL=6'h20, W_REGLO=6'h21, W_REGHI=6'h22;

    localparam int PC_RING_DEPTH = 16;
    localparam int PCR_AW = $clog2(PC_RING_DEPTH);   // 4

    // ================= control + debug state =================
    logic        go_q, soft_reset_q;
    logic [63:0] cycle_q, instret_q;
    logic [31:0] hpm_l1i_q, hpm_l1d_q, hpm_wb_q;

    logic [XLEN-1:0] pc_ring_q [PC_RING_DEPTH];
    logic [PCR_AW-1:0] pc_head_q;                    // next write slot
    logic [PCR_AW-1:0] pcidx_q;                      // host-selected read index

    logic [31:0]     trap_count_q;
    logic            trap_is_int_q, trap_seen_q;
    logic [4:0]      trap_cause_q;
    logic [XLEN-1:0] trap_epc_q, trap_tval_q;

    logic [XLEN-1:0] shadow_reg_q [32];
    logic [4:0]      regsel_q;

    assign core_go         = go_q;
    assign core_soft_reset = soft_reset_q;

    // ================= AXI-Lite write channel =================
    logic [31:0] awaddr_q;  logic awaddr_vld_q;
    logic [31:0] wdata_q;   logic [3:0] wstrb_q;  logic wdata_vld_q;
    logic        bvalid_q;

    logic aw_hs, w_hs, wr_commit;
    assign s_awready = !awaddr_vld_q && !bvalid_q;
    assign s_wready  = !wdata_vld_q  && !bvalid_q;
    assign aw_hs     = s_awvalid && s_awready;
    assign w_hs      = s_wvalid  && s_wready;
    assign wr_commit = awaddr_vld_q && wdata_vld_q && !bvalid_q;
    assign s_bvalid  = bvalid_q;
    assign s_bresp   = 2'b00;

    logic [5:0] wr_word;
    assign wr_word = awaddr_q[7:2];

    // RX push: a UART_RX write with a live low byte lane.
    assign rx_fifo_push = wr_commit && (wr_word == W_URX) && wstrb_q[0];
    assign rx_fifo_din  = wdata_q[7:0];

    // clear-counters pulse (CTRL[2], write-1-pulse)
    logic clear_counters;
    assign clear_counters = wr_commit && (wr_word == W_CTRL) && wstrb_q[0] && wdata_q[2];

    // ================= AXI-Lite read channel =================
    logic [31:0] rdata_q;  logic rvalid_q;
    logic        ar_hs;
    logic [5:0]  rd_word;
    assign s_arready = !rvalid_q;
    assign ar_hs     = s_arvalid && s_arready;
    assign rd_word   = s_araddr[7:2];
    assign s_rvalid  = rvalid_q;
    assign s_rdata   = rdata_q;
    assign s_rresp   = 2'b00;

    // 64-bit views (zero-extended on RV32 so the _HI fields read 0 cleanly).
    logic [63:0] sel_pc64, sel_reg64, epc64, tval64;
    assign sel_pc64  = 64'(pc_ring_q[pcidx_q]);
    assign sel_reg64 = 64'((regsel_q == 5'd0) ? '0 : shadow_reg_q[regsel_q]);
    assign epc64     = 64'(trap_epc_q);
    assign tval64    = 64'(trap_tval_q);

    // Combinational read mux (latched at ar_hs).
    function automatic logic [31:0] read_mux(input logic [5:0] w);
        unique case (w)
            W_CTRL:     read_mux = {30'b0, soft_reset_q, go_q};
            W_STATUS:   read_mux = {27'b0, trap_seen_q, rx_fifo_full, tx_fifo_empty,
                                    core_in_reset, core_halted};
            W_BUILDID:  read_mux = 32'h4E49_4731;                       // "NIG1"
            W_XLEN:     read_mux = {23'b0, 1'b1, 8'(XLEN)};
            W_CYCLO:    read_mux = cycle_q[31:0];
            W_CYCHI:    read_mux = cycle_q[63:32];
            W_INSTLO:   read_mux = instret_q[31:0];
            W_INSTHI:   read_mux = instret_q[63:32];
            W_UTX:      read_mux = {23'b0, !tx_fifo_empty, tx_fifo_dout};
            W_UTXST:    read_mux = {14'b0, tx_fifo_full, tx_fifo_empty, tx_fifo_count};
            W_URXST:    read_mux = {14'b0, rx_fifo_full, rx_fifo_empty, rx_fifo_count};
            W_HPMI:     read_mux = hpm_l1i_q;
            W_HPMD:     read_mux = hpm_l1d_q;
            W_HPMWB:    read_mux = hpm_wb_q;
            W_PCIDX:    read_mux = {20'b0, pc_head_q, 4'b0, pcidx_q};
            W_PCLO:     read_mux = sel_pc64[31:0];
            W_PCHI:     read_mux = sel_pc64[63:32];
            W_TRAPCNT:  read_mux = trap_count_q;
            W_TRAPCAUSE:read_mux = {23'b0, trap_is_int_q, 3'b0, trap_cause_q};
            W_EPCLO:    read_mux = epc64[31:0];
            W_EPCHI:    read_mux = epc64[63:32];
            W_TVALLO:   read_mux = tval64[31:0];
            W_TVALHI:   read_mux = tval64[63:32];
            W_REGSEL:   read_mux = {27'b0, regsel_q};
            W_REGLO:    read_mux = sel_reg64[31:0];
            W_REGHI:    read_mux = sel_reg64[63:32];
            default:    read_mux = 32'h0;
        endcase
    endfunction

    // TX pop on a UART_TX read with a byte available.
    logic tx_pop_set;
    assign tx_pop_set = ar_hs && (rd_word == W_UTX) && !tx_fifo_empty;

    // ================= sequential =================
    always_ff @(posedge clk or negedge rst_main_n) begin
        // per-cycle accumulator for the multi-write committed-PC ring head.
        automatic logic [PCR_AW-1:0] head_v = pc_head_q;
        if (!rst_main_n) begin
            go_q <= 1'b0; soft_reset_q <= 1'b0;
            awaddr_q <= '0; awaddr_vld_q <= 1'b0;
            wdata_q <= '0; wstrb_q <= '0; wdata_vld_q <= 1'b0;
            bvalid_q <= 1'b0; rdata_q <= '0; rvalid_q <= 1'b0;
            tx_fifo_pop <= 1'b0;
            cycle_q <= '0; instret_q <= '0;
            hpm_l1i_q <= '0; hpm_l1d_q <= '0; hpm_wb_q <= '0;
            pc_head_q <= '0; pcidx_q <= '0;
            trap_count_q <= '0; trap_is_int_q <= 1'b0; trap_seen_q <= 1'b0;
            trap_cause_q <= '0; trap_epc_q <= '0; trap_tval_q <= '0;
            regsel_q <= '0;
            for (int i = 0; i < PC_RING_DEPTH; i++) pc_ring_q[i] <= '0;
            for (int i = 0; i < 32; i++) shadow_reg_q[i] <= '0;
        end else begin
            // ---- write channel ----
            if (aw_hs) begin awaddr_q <= s_awaddr; awaddr_vld_q <= 1'b1; end
            if (w_hs)  begin wdata_q <= s_wdata; wstrb_q <= s_wstrb; wdata_vld_q <= 1'b1; end
            if (wr_commit) begin
                awaddr_vld_q <= 1'b0; wdata_vld_q <= 1'b0; bvalid_q <= 1'b1;
                if (wstrb_q[0]) begin
                    unique case (wr_word)
                        W_CTRL:   begin go_q <= wdata_q[0]; soft_reset_q <= wdata_q[1]; end
                        W_PCIDX:  pcidx_q  <= wdata_q[PCR_AW-1:0];
                        W_REGSEL: regsel_q <= wdata_q[4:0];
                        default: ; // UART_RX handled via rx_fifo_push; others RO
                    endcase
                end
            end
            if (bvalid_q && s_bready) bvalid_q <= 1'b0;

            // ---- read channel ----
            if (ar_hs) begin rdata_q <= read_mux(rd_word); rvalid_q <= 1'b1; end
            if (rvalid_q && s_rready) rvalid_q <= 1'b0;
            tx_fifo_pop <= tx_pop_set;   // 1-cycle pop pulse (byte latched at ar_hs)

            // ---- debug counters / observability (advance while running) ----
            if (clear_counters) begin
                cycle_q <= '0; instret_q <= '0;
                hpm_l1i_q <= '0; hpm_l1d_q <= '0; hpm_wb_q <= '0;
                trap_count_q <= '0; trap_seen_q <= 1'b0;
            end else if (!core_in_reset) begin
                cycle_q   <= cycle_q + 64'd1;
                instret_q <= instret_q + 64'($countones(dbg_probe.retire_valid));
                if (dbg_probe.hpm_l1i_miss) hpm_l1i_q <= hpm_l1i_q + 32'd1;
                if (dbg_probe.hpm_l1d_miss) hpm_l1d_q <= hpm_l1d_q + 32'd1;
                if (dbg_probe.hpm_l1d_wb)   hpm_wb_q  <= hpm_wb_q  + 32'd1;
            end

            // committed-PC ring: push each retired lane (program order, lane 0
            // oldest), advancing the head by the number written this cycle.
            for (int i = 0; i < OOO_WIDTH; i++) begin
                if (dbg_probe.retire_valid[i]) begin
                    pc_ring_q[head_v] <= dbg_probe.retire_pc[i];
                    head_v = head_v + PCR_AW'(1);
                end
            end
            pc_head_q <= head_v;

            // shadow architectural regfile: youngest writer (highest lane) wins.
            for (int i = 0; i < OOO_WIDTH; i++)
                if (dbg_probe.arch_we[i] && (dbg_probe.arch_rd[i] != 5'd0))
                    shadow_reg_q[dbg_probe.arch_rd[i]] <= dbg_probe.arch_data[i];

            // last-trap log.
            if (dbg_probe.trap_valid) begin
                trap_count_q  <= trap_count_q + 32'd1;
                trap_seen_q   <= 1'b1;
                trap_is_int_q <= dbg_probe.trap_is_int;
                trap_cause_q  <= dbg_probe.trap_cause;
                trap_epc_q    <= dbg_probe.trap_epc;
                trap_tval_q   <= dbg_probe.trap_tval;
            end
        end
    end

endmodule : ocl_csr

`default_nettype wire
