/**
 * nmi_axi_bridge.sv  (phase X1)
 *
 * NMI master -> AXI4 master, 512-bit data. One 64 B line op maps to one
 * single-beat AXI burst (AxLEN=0, AxSIZE=6, INCR): RD_LINE -> AR/R,
 * WR_LINE -> AW+W/B. The caches keep <=1 op outstanding per id and the NMI
 * arbiter serialises, so the bridge processes one transaction at a time and
 * needs no reorder logic; same-id AXI ordering supplies NMI rule R1. Word ops
 * never reach here (AXI=1 implies L1=1, all NMI traffic is line ops) -- they
 * are flagged.
 *
 * The NMI carries word addresses; the AXI address is the byte PA
 * (waddr << log2(XLEN_BYTES)), 64 B-aligned for line ops by construction.
 */

`include "niigo_mem.vh"

`default_nettype none

module nmi_axi_bridge
    import RISCV_ISA::XLEN_BYTES;
    import NIIGO_Mem::*;
(
    input wire logic clk,
    input wire logic rst_l,

    // ---- NMI slave ----
    input wire nmi_req_t  nmi_req,
    output logic      nmi_req_ready,
    output nmi_resp_t nmi_resp,

    // ---- AXI4 master: write address ----
    output logic                  axi_awvalid,
    input wire logic                  axi_awready,
    output logic [AXI_ADDR_W-1:0] axi_awaddr,
    output logic [AXI_ID_W-1:0]   axi_awid,
    output logic [7:0]            axi_awlen,
    output logic [2:0]            axi_awsize,
    output logic [1:0]            axi_awburst,
    // ---- write data ----
    output logic                  axi_wvalid,
    input wire logic                  axi_wready,
    output logic [AXI_DATA_W-1:0] axi_wdata,
    output logic [AXI_STRB_W-1:0] axi_wstrb,
    output logic                  axi_wlast,
    // ---- write response ----
    input wire logic                  axi_bvalid,
    output logic                  axi_bready,
    input wire logic [AXI_ID_W-1:0]   axi_bid,
    input wire logic [1:0]            axi_bresp,
    // ---- read address ----
    output logic                  axi_arvalid,
    input wire logic                  axi_arready,
    output logic [AXI_ADDR_W-1:0] axi_araddr,
    output logic [AXI_ID_W-1:0]   axi_arid,
    output logic [7:0]            axi_arlen,
    output logic [2:0]            axi_arsize,
    output logic [1:0]            axi_arburst,
    // ---- read data ----
    input wire logic                  axi_rvalid,
    output logic                  axi_rready,
    input wire logic [AXI_ID_W-1:0]   axi_rid,
    input wire logic [AXI_DATA_W-1:0] axi_rdata,
    input wire logic [1:0]            axi_rresp,
    input wire logic                  axi_rlast
);

    localparam int SHIFT = $clog2(XLEN_BYTES);

    typedef enum logic [2:0] { S_IDLE, S_AR, S_R, S_AWW, S_B, S_RESP } state_e;
    state_e state_q, state_n;

    logic [AXI_ADDR_W-1:0] addr_q,  addr_n;
    logic [AXI_ID_W-1:0]   id_q,    id_n;
    logic [AXI_DATA_W-1:0] data_q,  data_n;   // write payload / captured read line
    logic                  err_q,   err_n;
    logic                  aw_done_q, aw_done_n;
    logic                  w_done_q,  w_done_n;

    // Byte PA, zero-extended to the AXI address width.
    logic [AXI_ADDR_W-1:0] req_byte_addr;
    assign req_byte_addr = AXI_ADDR_W'({nmi_req.waddr, {SHIFT{1'b0}}});

    assign nmi_req_ready = (state_q == S_IDLE);

    // AXI constant burst geometry.
    assign axi_awlen   = 8'd0;
    assign axi_awsize  = AXI_SIZE_LINE;
    assign axi_awburst = AXI_BURST_INCR;
    assign axi_arlen   = 8'd0;
    assign axi_arsize  = AXI_SIZE_LINE;
    assign axi_arburst = AXI_BURST_INCR;
    assign axi_wstrb   = '1;            // WR_LINE is always a full line
    assign axi_wlast   = 1'b1;          // single-beat

    assign axi_awaddr = addr_q;
    assign axi_awid   = id_q;
    assign axi_araddr = addr_q;
    assign axi_arid   = id_q;
    assign axi_wdata  = data_q;

    assign axi_arvalid = (state_q == S_AR);
    assign axi_rready  = (state_q == S_R);
    assign axi_awvalid = (state_q == S_AWW) && !aw_done_q;
    assign axi_wvalid  = (state_q == S_AWW) && !w_done_q;
    assign axi_bready  = (state_q == S_B);

    always_comb begin
        nmi_resp       = '0;
        nmi_resp.valid = (state_q == S_RESP);
        nmi_resp.id    = id_q;
        nmi_resp.rdata = data_q;
        nmi_resp.err   = err_q;
    end

    always_comb begin
        state_n   = state_q;
        addr_n    = addr_q;
        id_n      = id_q;
        data_n    = data_q;
        err_n     = err_q;
        aw_done_n = aw_done_q;
        w_done_n  = w_done_q;

        unique case (state_q)
            S_IDLE: begin
                if (nmi_req.valid) begin
                    addr_n = req_byte_addr;
                    id_n   = nmi_req.id;
                    data_n = nmi_req.wdata;
                    err_n  = 1'b0;
                    aw_done_n = 1'b0;
                    w_done_n  = 1'b0;
                    unique case (nmi_req.op)
                        NMI_RD_LINE: state_n = S_AR;
                        NMI_WR_LINE: state_n = S_AWW;
                        default: begin
                            // Word ops must never reach the AXI bridge.
                            state_n = S_RESP;
`ifndef SYNTHESIS
                            $fatal(1, "nmi_axi_bridge: non-line NMI op %0d", nmi_req.op);
`endif
                        end
                    endcase
                end
            end

            S_AR:  if (axi_arready) state_n = S_R;
            S_R: if (axi_rvalid) begin
                data_n  = axi_rdata;
                err_n   = (axi_rresp != 2'b00);
                state_n = S_RESP;
            end

            S_AWW: begin
                if (axi_awvalid && axi_awready) aw_done_n = 1'b1;
                if (axi_wvalid  && axi_wready)  w_done_n  = 1'b1;
                if ((aw_done_n) && (w_done_n))  state_n   = S_B;
            end
            S_B: if (axi_bvalid) begin
                err_n   = (axi_bresp != 2'b00);
                state_n = S_RESP;
            end

            S_RESP: state_n = S_IDLE;
            default: state_n = S_IDLE;
        endcase
    end

    always_ff @(posedge clk or negedge rst_l) begin
        if (!rst_l) begin
            state_q   <= S_IDLE;
            addr_q    <= '0;
            id_q      <= '0;
            data_q    <= '0;
            err_q     <= 1'b0;
            aw_done_q <= 1'b0;
            w_done_q  <= 1'b0;
        end else begin
            state_q   <= state_n;
            addr_q    <= addr_n;
            id_q      <= id_n;
            data_q    <= data_n;
            err_q     <= err_n;
            aw_done_q <= aw_done_n;
            w_done_q  <= w_done_n;
        end
    end

    logic unused_axi;
    assign unused_axi = (|axi_bid) | (|axi_rid) | axi_rlast;

endmodule : nmi_axi_bridge

`default_nettype wire
