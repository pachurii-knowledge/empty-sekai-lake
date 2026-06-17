/**
 * tb_ocl_csr.sv  (FB1) -- standalone Verilator unit test for the OCL control
 * plane + vUART FIFOs + debug observability block.
 *
 * Wires ocl_csr to the two uart_host_fifo instances exactly as cl_niigo does,
 * drives the AXI4-Lite slave from a simple master, synthesizes commit-stage
 * debug_probe events, and self-checks: build/xlen id, CTRL go/soft-reset,
 * vUART TX (core->host) + RX (host->core) FIFO paths, cycle/instret counters
 * (+ clear), the committed-PC ring, the shadow architectural regfile, and the
 * trap log. Build: verilator --binary --top-module tb_ocl_csr -DRV64.
 *
 * Timing convention: every clock wait is `step` = `@(posedge clk); #1`, which
 * lands in the settled region past the NBA updates, so reads see committed
 * register values and blocking drives are stable before the next edge (race-free
 * against the DUT's own NBA sampling).
 */
`include "ooo_types.vh"
`default_nettype none

module tb_ocl_csr;
    import OOO_Types::XLEN, OOO_Types::OOO_WIDTH, OOO_Types::debug_probe_t;

    localparam logic [31:0]
        CTRL=32'h00, STATUS=32'h04, BUILDID=32'h08, XLENI=32'h0C,
        CYCLO=32'h10, CYCHI=32'h14, INSTLO=32'h18, INSTHI=32'h1C,
        UTX=32'h20, UTXST=32'h24, URX=32'h28, URXST=32'h2C,
        HPMI=32'h30, HPMD=32'h34, HPMWB=32'h38,
        PCIDX=32'h40, PCLO=32'h44, PCHI=32'h48,
        TRAPCNT=32'h50, TRAPCAUSE=32'h54, EPCLO=32'h58, EPCHI=32'h5C,
        TVALLO=32'h60, TVALHI=32'h64,
        REGSEL=32'h80, REGLO=32'h84, REGHI=32'h88;

    logic clk = 1'b0, rst_n = 1'b0;
    always #5 clk = ~clk;

    // AXI-Lite master wires
    logic [31:0] awaddr; logic awvalid, awready;
    logic [31:0] wdata;  logic [3:0] wstrb; logic wvalid, wready;
    logic [1:0]  bresp;  logic bvalid, bready;
    logic [31:0] araddr; logic arvalid, arready;
    logic [31:0] rdata;  logic [1:0] rresp; logic rvalid, rready;

    logic core_go, core_soft_reset, core_halted;
    logic core_in_reset;
    assign core_in_reset = !(rst_n && core_go && !core_soft_reset);

    // vUART FIFOs (wired like cl_niigo)
    logic       tx_empty, tx_full, tx_pop;  logic [7:0] tx_dout; logic [8:0] tx_count;
    logic       rx_empty, rx_full, rx_push;  logic [7:0] rx_dout, rx_din; logic [8:0] rx_count;
    logic       core_tx_valid; logic [7:0] core_tx_byte;   // core->TX FIFO
    logic       core_rx_pop;                                // core pops RX FIFO

    uart_host_fifo #(.WIDTH(8), .DEPTH(256)) TXF (
        .clk, .rst_l(rst_n), .wr_en(core_tx_valid), .wr_data(core_tx_byte),
        .full(tx_full), .rd_en(tx_pop), .rd_data(tx_dout), .empty(tx_empty), .count(tx_count));
    uart_host_fifo #(.WIDTH(8), .DEPTH(256)) RXF (
        .clk, .rst_l(rst_n), .wr_en(rx_push), .wr_data(rx_din),
        .full(rx_full), .rd_en(core_rx_pop), .rd_data(rx_dout), .empty(rx_empty), .count(rx_count));

    debug_probe_t probe;

    ocl_csr DUT (
        .clk, .rst_main_n(rst_n),
        .s_awaddr(awaddr), .s_awvalid(awvalid), .s_awready(awready),
        .s_wdata(wdata), .s_wstrb(wstrb), .s_wvalid(wvalid), .s_wready(wready),
        .s_bresp(bresp), .s_bvalid(bvalid), .s_bready(bready),
        .s_araddr(araddr), .s_arvalid(arvalid), .s_arready(arready),
        .s_rdata(rdata), .s_rresp(rresp), .s_rvalid(rvalid), .s_rready(rready),
        .core_go(core_go), .core_soft_reset(core_soft_reset),
        .core_in_reset(core_in_reset), .core_halted(core_halted),
        .tx_fifo_empty(tx_empty), .tx_fifo_full(tx_full), .tx_fifo_dout(tx_dout),
        .tx_fifo_count(16'(tx_count)), .tx_fifo_pop(tx_pop),
        .rx_fifo_empty(rx_empty), .rx_fifo_full(rx_full), .rx_fifo_count(16'(rx_count)),
        .rx_fifo_push(rx_push), .rx_fifo_din(rx_din),
        .dbg_probe(probe)
    );

    int errors = 0;

    task automatic step; @(posedge clk); #1; endtask
    task automatic stepn(input int n); for (int i=0;i<n;i++) step; endtask

    // rready/bready are held high for the whole test (set after reset), so the
    // response handshake always completes the cycle valid is high.
    task automatic axil_write(input logic [31:0] a, input logic [31:0] d);
        step; awaddr=a; awvalid=1'b1; wdata=d; wstrb=4'hF; wvalid=1'b1;
        step; awvalid=1'b0; wvalid=1'b0;          // accepted (idle slave)
        while (!bvalid) step;                       // wait B
    endtask

    task automatic axil_read(input logic [31:0] a, output logic [31:0] d);
        step; araddr=a; arvalid=1'b1;
        step; arvalid=1'b0;                         // accepted (idle slave)
        while (!rvalid) step;                       // wait R
        d = rdata;
    endtask

    task automatic check(input logic [31:0] got, input logic [31:0] exp, input string nm);
        if (got !== exp) begin
            $display("  FAIL %-16s got=%08h exp=%08h", nm, got, exp); errors++;
        end else $display("  ok   %-16s = %08h", nm, got);
    endtask

    task automatic rd_check(input logic [31:0] a, input logic [31:0] exp, input string nm);
        logic [31:0] g; axil_read(a, g); check(g, exp, nm);
    endtask

    // Drive a single-cycle commit probe event (seen at exactly one posedge).
    task automatic probe_cycle(input debug_probe_t p);
        step; probe = p;
        step; probe = '0;
    endtask

    logic [31:0] g, ghi;

    initial begin
        awvalid=0; wvalid=0; bready=0; arvalid=0; rready=0;
        awaddr=0; wdata=0; wstrb=0; araddr=0;
        core_halted=0; probe='0;
        core_tx_valid=0; core_tx_byte=0; core_rx_pop=0;
        stepn(4); rst_n = 1'b1; stepn(2);
        bready = 1'b1; rready = 1'b1;   // master always accepts responses

        $display("[tb_ocl_csr] XLEN=%0d", XLEN);

        // --- identity ---
        rd_check(BUILDID, 32'h4E49_4731, "BUILD_ID");
        rd_check(XLENI, {23'b0, 1'b1, 8'(XLEN)}, "XLEN_INFO");

        // --- CTRL / STATUS ---
        rd_check(STATUS, 32'h6, "STATUS in_reset");   // in_reset(b1) | tx_empty(b2)
        axil_write(CTRL, 32'h1);                       // go=1
        rd_check(CTRL, 32'h1, "CTRL go");
        rd_check(STATUS, 32'h4, "STATUS tx_empty");    // running, tx empty (bit2)
        axil_write(CTRL, 32'h2);                       // soft_reset=1, go=0
        rd_check(STATUS, 32'h6, "STATUS soft_reset");  // in_reset(b1) | tx_empty(b2)
        axil_write(CTRL, 32'h1);                       // back to running

        // --- vUART RX (host -> core) ---
        axil_write(URX, 32'h41);   // 'A'
        axil_write(URX, 32'h42);   // 'B'
        stepn(2);
        rd_check(URXST, {16'b0, 16'd2}, "URXST count=2");
        check({24'b0, rx_dout}, 32'h41, "rx head A");
        step; core_rx_pop=1'b1; step; core_rx_pop=1'b0;   // core consumes 'A'
        stepn(2);
        check({24'b0, rx_dout}, 32'h42, "rx head B");

        // --- vUART TX (core -> host) ---
        step; core_tx_valid=1'b1; core_tx_byte=8'h58;   // 'X'
        step; core_tx_byte=8'h59;                        // 'Y'
        step; core_tx_valid=1'b0;
        stepn(2);
        rd_check(UTX, 32'h158, "UART_TX X");        // valid(bit8)|0x58
        rd_check(UTX, 32'h159, "UART_TX Y");
        rd_check(UTX, 32'h000, "UART_TX empty");    // valid=0

        // --- counters: clear, then 3 cycles of 2 retires each ---
        axil_write(CTRL, 32'h5);    // go=1 | clear_counters(bit2)
        axil_write(CTRL, 32'h1);    // go=1
        begin
            debug_probe_t p; p = '0; p.retire_valid = 4'b0011;
            p.retire_pc[0] = 'h1000; p.retire_pc[1] = 'h1004;
            probe_cycle(p); probe_cycle(p); probe_cycle(p);
        end
        rd_check(INSTLO, 32'd6, "INSTRET=6");        // 2 * 3
        axil_read(CYCLO, g);
        if (g == 0) begin $display("  FAIL CYCLE not advancing"); errors++; end
        else $display("  ok   CYCLE advancing = %0d", g);

        // --- committed-PC ring ---
        begin
            debug_probe_t p; p='0; p.retire_valid=4'b0001; p.retire_pc[0]='hBEEF0;
            probe_cycle(p);
        end
        axil_read(PCIDX, g);                          // [3:0]=idx [11:8]=head
        axil_write(PCIDX, ((g >> 8) & 32'hF) == 0 ? 32'hF : (((g >> 8) & 32'hF) - 1));
        rd_check(PCLO, 32'hBEEF0, "PC_RING last");

        // --- shadow architectural regfile (a0 = x10) ---
        begin
            debug_probe_t p; p='0; p.arch_we=4'b0001; p.arch_rd[0]=5'd10;
            p.arch_data[0] = XLEN'(64'hDEAD_BEEF_CAFE_F00D);
            probe_cycle(p);
        end
        axil_write(REGSEL, 32'd10);
        axil_read(REGLO, g); axil_read(REGHI, ghi);
        check(g, 32'hCAFE_F00D, "REG a0 lo");
        if (XLEN > 32) check(ghi, 32'hDEAD_BEEF, "REG a0 hi");
        begin debug_probe_t p; p='0; p.arch_we=4'b0001; p.arch_rd[0]=5'd0;
              p.arch_data[0]=XLEN'(64'hFFFF_FFFF); probe_cycle(p); end
        axil_write(REGSEL, 32'd0);
        rd_check(REGLO, 32'h0, "REG x0 = 0");

        // --- trap log ---
        begin
            debug_probe_t p; p='0; p.trap_valid=1'b1; p.trap_is_int=1'b0;
            p.trap_cause=5'd2; p.trap_epc=XLEN'(64'h8000_0040); p.trap_tval=XLEN'('h13);
            probe_cycle(p);
        end
        rd_check(TRAPCNT, 32'd1, "TRAP count");
        rd_check(TRAPCAUSE, {23'b0, 1'b0, 3'b0, 5'd2}, "TRAP cause");
        rd_check(EPCLO, 32'h8000_0040, "TRAP epc");
        rd_check(TVALLO, 32'h13, "TRAP tval");

        stepn(4);
        if (errors == 0) $display("RVCP-SUMMARY: TEST PASSED (tb_ocl_csr)");
        else             $display("RVCP-SUMMARY: TEST FAILED (%0d errors)", errors);
        $finish;
    end

    initial begin
        #200000;
        $display("RVCP-SUMMARY: TEST FAILED (timeout)");
        $finish;
    end

endmodule
`default_nettype wire
