/**
 * clint.sv
 *
 * Minimal core-local interruptor (CLINT). Implements the SiFive-style memory
 * mapped registers for a single hart:
 *   BASE + 0x0000 : msip      (software interrupt, 1 bit)
 *   BASE + 0x4000 : mtimecmp  (64-bit)
 *   BASE + 0xBFF8 : mtime     (64-bit, free running)
 *
 * It snoops the core's data store port to capture writes and provides a
 * combinational load-hit/read path for reads of its registers. mtime drives
 * the machine timer interrupt (mtip = mtime >= mtimecmp) and msip drives the
 * machine software interrupt.
 */

`default_nettype none

module clint #(
    parameter logic [31:0] BASE = 32'h0200_0000
) (
    input  logic        clk,
    input  logic        rst_l,

    // Data store snoop (word address space: byte addr >> 2)
    input  logic        store_en,
    input  logic [29:0] store_waddr,
    input  logic [31:0] store_wdata,
    input  logic [3:0]  store_mask,

    // Combinational load query
    input  logic [29:0] load_addr,
    output logic        load_hit,
    output logic [31:0] load_data,

    output logic        irq_m_timer,
    output logic        irq_m_software,
    output logic [63:0] mtime_out
);

    localparam logic [29:0] MSIP_W      = 30'((BASE + 32'h0000) >> 2);
    localparam logic [29:0] MTIMECMP_LO = 30'((BASE + 32'h4000) >> 2);
    localparam logic [29:0] MTIMECMP_HI = 30'((BASE + 32'h4004) >> 2);
    localparam logic [29:0] MTIME_LO    = 30'((BASE + 32'hBFF8) >> 2);
    localparam logic [29:0] MTIME_HI    = 30'((BASE + 32'hBFFC) >> 2);

    logic [63:0] mtime_q;
    logic [63:0] mtimecmp_q;
    logic        msip_q;

    assign mtime_out      = mtime_q;
    assign irq_m_timer    = (mtime_q >= mtimecmp_q);
    assign irq_m_software = msip_q;

    // Combinational read path
    always_comb begin
        load_hit  = 1'b1;
        load_data = 32'b0;
        unique case (load_addr)
            MSIP_W:      load_data = {31'b0, msip_q};
            MTIMECMP_LO: load_data = mtimecmp_q[31:0];
            MTIMECMP_HI: load_data = mtimecmp_q[63:32];
            MTIME_LO:    load_data = mtime_q[31:0];
            MTIME_HI:    load_data = mtime_q[63:32];
            default:     load_hit  = 1'b0;
        endcase
    end

    function automatic logic [31:0] merge(input logic [31:0] old_w,
            input logic [31:0] new_w, input logic [3:0] mask);
        merge = old_w;
        for (int b = 0; b < 4; b += 1) begin
            if (mask[b]) merge[b*8 +: 8] = new_w[b*8 +: 8];
        end
    endfunction

    always_ff @(posedge clk or negedge rst_l) begin
        if (!rst_l) begin
            mtime_q    <= 64'b0;
            mtimecmp_q <= {64{1'b1}};
            msip_q     <= 1'b0;
        end else begin
            mtime_q <= mtime_q + 64'd1;
            if (store_en && (store_mask != 4'b0)) begin
                unique case (store_waddr)
                    MSIP_W:      if (store_mask[0]) msip_q <= store_wdata[0];
                    MTIMECMP_LO: mtimecmp_q[31:0]  <= merge(mtimecmp_q[31:0],
                                                 store_wdata, store_mask);
                    MTIMECMP_HI: mtimecmp_q[63:32] <= merge(mtimecmp_q[63:32],
                                                 store_wdata, store_mask);
                    MTIME_LO:    mtime_q[31:0]  <= merge(mtime_q[31:0],
                                                 store_wdata, store_mask);
                    MTIME_HI:    mtime_q[63:32] <= merge(mtime_q[63:32],
                                                 store_wdata, store_mask);
                    default: ;
                endcase
            end
        end
    end

endmodule: clint
