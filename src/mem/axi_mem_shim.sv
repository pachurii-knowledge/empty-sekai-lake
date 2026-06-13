/**
 * axi_mem_shim.sv  (SIMULATION ONLY, phase X1)
 *
 * AXI4 slave that terminates the nmi_axi_bridge and drives main_memory's word
 * ports (DD-11: storage stays Memory.memory, so image loading + signature dump
 * + priv_diag keep working). One transaction at a time (the bridge is
 * single-outstanding): a read fans a single 512 b R beat out of LINE_WORDS
 * word reads; a write drains a 512 b W beat into LINE_WORDS word writes.
 * Optional per-channel latency fuzzing via +axi_min_lat/+axi_max_lat/+axi_seed
 * (no $random -- replayable).
 */

`include "niigo_mem.vh"

`default_nettype none

module axi_mem_shim
    import RISCV_ISA::XLEN, RISCV_ISA::XLEN_BYTES;
    import RISCV_UArch::MEMORY_READ_WIDTH, RISCV_UArch::MEMORY_ADDR_WIDTH;
    import NIIGO_Mem::*;
(
    input wire logic clk,
    input wire logic rst_l,

    // ---- AXI4 slave ----
    input wire logic                  axi_awvalid,
    output logic                  axi_awready,
    input wire logic [AXI_ADDR_W-1:0] axi_awaddr,
    input wire logic [AXI_ID_W-1:0]   axi_awid,
    input wire logic                  axi_wvalid,
    output logic                  axi_wready,
    input wire logic [AXI_DATA_W-1:0] axi_wdata,
    input wire logic [AXI_STRB_W-1:0] axi_wstrb,
    input wire logic                  axi_wlast,
    output logic                  axi_bvalid,
    input wire logic                  axi_bready,
    output logic [AXI_ID_W-1:0]   axi_bid,
    output logic [1:0]            axi_bresp,
    input wire logic                  axi_arvalid,
    output logic                  axi_arready,
    input wire logic [AXI_ADDR_W-1:0] axi_araddr,
    input wire logic [AXI_ID_W-1:0]   axi_arid,
    output logic                  axi_rvalid,
    input wire logic                  axi_rready,
    output logic [AXI_ID_W-1:0]   axi_rid,
    output logic [AXI_DATA_W-1:0] axi_rdata,
    output logic [1:0]            axi_rresp,
    output logic                  axi_rlast,

    // ---- main_memory word ports ----
    output logic [MEMORY_ADDR_WIDTH-1:0]            mem_rd_addr,
    input wire logic [MEMORY_READ_WIDTH-1:0][XLEN-1:0]  mem_rd_data,
    output logic                          mem_wr_en,
    output logic [MEMORY_ADDR_WIDTH-1:0]  mem_wr_addr,
    output logic [XLEN-1:0]               mem_wr_data,
    output logic [XLEN_BYTES-1:0]         mem_wr_mask
);

    localparam int SHIFT = $clog2(XLEN_BYTES);
    localparam int RW    = MEMORY_READ_WIDTH;

    // ---- fuzz config (read once) ----
    logic        fz_en;
    int unsigned fz_seed, fz_min, fz_max;
    initial begin
        fz_en = $test$plusargs("axi_fuzz") != 0;
        if (!$value$plusargs("axi_seed=%d", fz_seed)) fz_seed = 1;
        if (!$value$plusargs("axi_min_lat=%d", fz_min)) fz_min = 0;
        if (!$value$plusargs("axi_max_lat=%d", fz_max)) fz_max = 0;
        if (fz_max < fz_min) fz_max = fz_min;
    end
    logic [31:0] lcg;
    logic seeded_q;
    function automatic logic [31:0] lcg_next(input logic [31:0] s);
        lcg_next = s * 32'd1664525 + 32'd1013904223;
    endfunction
    function automatic int unsigned lcg_range(input logic [31:0] s,
            input int unsigned lo, input int unsigned hi);
        lcg_range = (hi <= lo) ? lo : lo + (32'(s >> 8) % (hi - lo + 1));
    endfunction

    typedef enum logic [3:0] {
        S_IDLE, S_AR_LAT, S_RD_BEAT, S_R, S_W, S_WR_LAT, S_WR_BEAT, S_B
    } state_e;
    state_e state_q, state_n;

    logic [MEMORY_ADDR_WIDTH-1:0] base_q, base_n;   // word base addr
    logic [AXI_ID_W-1:0]          id_q,   id_n;
    logic [AXI_DATA_W-1:0]        data_q, data_n;
    logic [$clog2(LINE_WORDS+1)-1:0] cnt_q, cnt_n;
    logic [15:0]                  lat_q,  lat_n;

    logic [$clog2(LINE_WORDS)-1:0] widx;
    assign widx = cnt_q[$clog2(LINE_WORDS)-1:0];

    // Word base address from an AXI byte address.
    logic [MEMORY_ADDR_WIDTH-1:0] ar_base, aw_base;
    assign ar_base = axi_araddr[SHIFT +: MEMORY_ADDR_WIDTH];
    assign aw_base = axi_awaddr[SHIFT +: MEMORY_ADDR_WIDTH];

    // ---- AXI channel outputs ----
    assign axi_arready = (state_q == S_IDLE);
    assign axi_awready = (state_q == S_IDLE) && !axi_arvalid;  // AR wins a tie
    assign axi_wready  = (state_q == S_W);
    assign axi_rvalid  = (state_q == S_R);
    assign axi_rid     = id_q;
    assign axi_rdata   = data_q;
    assign axi_rresp   = 2'b00;
    assign axi_rlast   = 1'b1;
    assign axi_bvalid  = (state_q == S_B);
    assign axi_bid     = id_q;
    assign axi_bresp   = 2'b00;

    // ---- main_memory drive ----
    always_comb begin
        mem_rd_addr = base_q + MEMORY_ADDR_WIDTH'({cnt_q, {$clog2(RW){1'b0}}});
        mem_wr_en   = (state_q == S_WR_BEAT);
        mem_wr_addr = base_q + MEMORY_ADDR_WIDTH'(widx);
        mem_wr_data = data_q[widx*XLEN +: XLEN];
        mem_wr_mask = '1;
    end

    always_ff @(posedge clk or negedge rst_l) begin
        if (!rst_l) seeded_q <= 1'b0;
        else if (!seeded_q) begin seeded_q <= 1'b1; lcg <= 32'(fz_seed) ^ 32'h41584D53; end
        else if (state_q == S_IDLE) lcg <= lcg_next(lcg);
    end

    always_comb begin
        state_n = state_q;
        base_n  = base_q;
        id_n    = id_q;
        data_n  = data_q;
        cnt_n   = cnt_q;
        lat_n   = lat_q;

        unique case (state_q)
            S_IDLE: begin
                cnt_n = '0;
                if (axi_arvalid) begin
                    base_n = ar_base;
                    id_n   = axi_arid;
                    lat_n  = fz_en ? 16'(lcg_range(lcg, fz_min, fz_max)) : 16'd0;
                    state_n = S_AR_LAT;
                end else if (axi_awvalid) begin
                    base_n = aw_base;
                    id_n   = axi_awid;
                    state_n = S_W;
                end
            end

            S_AR_LAT: if (lat_q == '0) state_n = S_RD_BEAT; else lat_n = lat_q - 1'b1;
            S_RD_BEAT: begin
                for (int k = 0; k < RW; k += 1)
                    data_n[(cnt_q*RW + k)*XLEN +: XLEN] = mem_rd_data[k];
                if (cnt_q == (LINE_RD_BEATS-1)) begin cnt_n = '0; state_n = S_R; end
                else cnt_n = cnt_q + 1'b1;
            end
            S_R: if (axi_rready) state_n = S_IDLE;

            S_W: if (axi_wvalid) begin
                data_n = axi_wdata;
                lat_n  = fz_en ? 16'(lcg_range(lcg ^ 32'h57, fz_min, fz_max)) : 16'd0;
                state_n = S_WR_LAT;
            end
            S_WR_LAT: if (lat_q == '0) state_n = S_WR_BEAT; else lat_n = lat_q - 1'b1;
            S_WR_BEAT: begin
                if (cnt_q == (LINE_WORDS-1)) state_n = S_B;
                else cnt_n = cnt_q + 1'b1;
            end
            S_B: if (axi_bready) state_n = S_IDLE;

            default: state_n = S_IDLE;
        endcase
    end

    always_ff @(posedge clk or negedge rst_l) begin
        if (!rst_l) begin
            state_q <= S_IDLE; base_q <= '0; id_q <= '0; data_q <= '0;
            cnt_q <= '0; lat_q <= '0;
        end else begin
            state_q <= state_n; base_q <= base_n; id_q <= id_n; data_q <= data_n;
            cnt_q <= cnt_n; lat_q <= lat_n;
        end
    end

    logic unused_shim;
    assign unused_shim = (&axi_wstrb) | axi_wlast;

endmodule : axi_mem_shim

`default_nettype wire
