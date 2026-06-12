/**
 * axi_chk.sv  (SIMULATION ONLY, phase X1)
 *
 * Passive AXI4 protocol monitor instantiated alongside the bridge/shim in the
 * AXI=1 testbench arm. Verilator-5-friendly immediate checks (no SVA):
 *   - valid stable + payload stable until ready (all 5 channels);
 *   - AR/AW are 64 B-aligned, SIZE=6, LEN=0, INCR (line ops only);
 *   - every W beat asserts WLAST (single-beat bursts);
 *   - <=1 outstanding read and <=1 outstanding write (the bridge guarantee);
 *   - RID/BID echo the outstanding AR/AW id;
 *   - read/write transaction counts balance at end of sim.
 * Any violation prints "AXI-CHK VIOLATION" (grep-able) and counts toward the
 * end-of-sim summary; the suites treat a non-zero count as a failure.
 */

`include "niigo_mem.vh"

`default_nettype none

module axi_chk
    import NIIGO_Mem::*;
(
    input logic clk,
    input logic rst_l,
    input logic                  axi_awvalid, axi_awready,
    input logic [AXI_ADDR_W-1:0] axi_awaddr,
    input logic [AXI_ID_W-1:0]   axi_awid,
    input logic [7:0]            axi_awlen,
    input logic [2:0]            axi_awsize,
    input logic [1:0]            axi_awburst,
    input logic                  axi_wvalid, axi_wready,
    input logic [AXI_DATA_W-1:0] axi_wdata,
    input logic                  axi_wlast,
    input logic                  axi_bvalid, axi_bready,
    input logic [AXI_ID_W-1:0]   axi_bid,
    input logic                  axi_arvalid, axi_arready,
    input logic [AXI_ADDR_W-1:0] axi_araddr,
    input logic [AXI_ID_W-1:0]   axi_arid,
    input logic [7:0]            axi_arlen,
    input logic [2:0]            axi_arsize,
    input logic [1:0]            axi_arburst,
    input logic                  axi_rvalid, axi_rready,
    input logic [AXI_ID_W-1:0]   axi_rid,
    input logic                  axi_rlast
);

    integer viol = 0;
    task automatic fail(input string msg);
        viol = viol + 1;
        $display("AXI-CHK VIOLATION: %s", msg);
    endtask

    // ---- previous-cycle snapshots for valid/payload-stable checks ----
    logic        aw_v_q, w_v_q, b_v_q, ar_v_q, r_v_q;
    logic [AXI_ADDR_W-1:0] aw_a_q, ar_a_q;
    logic [AXI_ID_W-1:0]   aw_i_q, ar_i_q;
    logic [AXI_DATA_W-1:0] w_d_q;

    // outstanding tracking (<=1 each)
    logic                rd_busy_q, wr_busy_q;
    logic [AXI_ID_W-1:0] rd_id_q,   wr_id_q;
    integer ar_n=0, r_n=0, aw_n=0, w_n=0, b_n=0;

    always_ff @(posedge clk) begin
        if (rst_l) begin
            // ---- request-channel geometry ----
            if (axi_arvalid) begin
                if (axi_araddr[5:0] != 6'd0) fail("ARADDR not 64B-aligned");
                if (axi_arsize != AXI_SIZE_LINE) fail("ARSIZE != 6");
                if (axi_arlen != 8'd0) fail("ARLEN != 0");
                if (axi_arburst != AXI_BURST_INCR) fail("ARBURST != INCR");
            end
            if (axi_awvalid) begin
                if (axi_awaddr[5:0] != 6'd0) fail("AWADDR not 64B-aligned");
                if (axi_awsize != AXI_SIZE_LINE) fail("AWSIZE != 6");
                if (axi_awlen != 8'd0) fail("AWLEN != 0");
                if (axi_awburst != AXI_BURST_INCR) fail("AWBURST != INCR");
            end
            if (axi_wvalid && !axi_wlast) fail("W beat without WLAST");

            // ---- valid stable until ready + payload stable ----
            if (ar_v_q && !axi_arvalid) fail("ARVALID dropped before ARREADY");
            if (ar_v_q && axi_arvalid && (axi_araddr != ar_a_q)) fail("ARADDR changed pre-handshake");
            if (aw_v_q && !axi_awvalid) fail("AWVALID dropped before AWREADY");
            if (aw_v_q && axi_awvalid && (axi_awaddr != aw_a_q)) fail("AWADDR changed pre-handshake");
            if (w_v_q  && !axi_wvalid)  fail("WVALID dropped before WREADY");
            if (w_v_q  && axi_wvalid && (axi_wdata != w_d_q)) fail("WDATA changed pre-handshake");
            if (r_v_q  && !axi_rvalid)  fail("RVALID dropped before RREADY");
            if (b_v_q  && !axi_bvalid)  fail("BVALID dropped before BREADY");

            // ---- outstanding tracking + id echo ----
            if (axi_arvalid && axi_arready) begin
                if (rd_busy_q) fail(">1 outstanding read");
                rd_busy_q <= 1'b1; rd_id_q <= axi_arid; ar_n <= ar_n + 1;
            end
            if (axi_rvalid && axi_rready) begin
                if (!rd_busy_q) fail("R with no outstanding AR");
                if (axi_rid != rd_id_q) fail("RID != outstanding ARID");
                if (!axi_rlast) fail("R without RLAST");
                rd_busy_q <= 1'b0; r_n <= r_n + 1;
            end
            if (axi_awvalid && axi_awready) begin
                if (wr_busy_q) fail(">1 outstanding write");
                wr_busy_q <= 1'b1; wr_id_q <= axi_awid; aw_n <= aw_n + 1;
            end
            if (axi_wvalid && axi_wready)  w_n <= w_n + 1;
            if (axi_bvalid && axi_bready) begin
                if (!wr_busy_q) fail("B with no outstanding AW");
                if (axi_bid != wr_id_q) fail("BID != outstanding AWID");
                wr_busy_q <= 1'b0; b_n <= b_n + 1;
            end

            // snapshot
            ar_v_q <= axi_arvalid && !axi_arready;
            aw_v_q <= axi_awvalid && !axi_awready;
            w_v_q  <= axi_wvalid  && !axi_wready;
            r_v_q  <= axi_rvalid  && !axi_rready;
            b_v_q  <= axi_bvalid  && !axi_bready;
            ar_a_q <= axi_araddr; aw_a_q <= axi_awaddr;
            ar_i_q <= axi_arid;   aw_i_q <= axi_awid;
            w_d_q  <= axi_wdata;
        end else begin
            rd_busy_q <= 1'b0; wr_busy_q <= 1'b0;
            ar_v_q <= 1'b0; aw_v_q <= 1'b0; w_v_q <= 1'b0; r_v_q <= 1'b0; b_v_q <= 1'b0;
        end
    end

    final begin
        if (ar_n != r_n)  fail("AR/R count mismatch at end of sim");
        if (aw_n != w_n)  fail("AW/W count mismatch at end of sim");
        if (aw_n != b_n)  fail("AW/B count mismatch at end of sim");
        if (rd_busy_q || wr_busy_q) fail("outstanding transaction at end of sim");
        $display("AXI-CHK: reads=%0d writes=%0d violations=%0d", r_n, b_n, viol);
    end

    logic unused_chk;
    assign unused_chk = (|ar_i_q) | (|aw_i_q);

endmodule : axi_chk

`default_nettype wire
