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
 *
 * The bus is one memory word wide (4 bytes at RV32, 8 at RV64). The device
 * registers are 32-bit; each bus word is decoded as XLEN/32 subwords, so on
 * RV64 a single 8-byte access reads/writes an adjacent register pair (e.g. a
 * 64-bit ld/sd of mtime or mtimecmp works naturally).
 */

`include "riscv_isa.vh"
`include "riscv_uarch.vh"

`default_nettype none

module clint
    import RISCV_ISA::XLEN, RISCV_ISA::XLEN_BYTES;
    import RISCV_UArch::MEMORY_ADDR_WIDTH;
#(
    parameter logic [31:0] BASE = 32'h0200_0000
) (
    input  logic        clk,
    input  logic        rst_l,

    // Data store snoop (memory-word address space: byte addr >> log2(XLEN_BYTES))
    input  logic        store_en,
    input  logic [MEMORY_ADDR_WIDTH-1:0] store_waddr,
    input  logic [XLEN-1:0] store_wdata,
    input  logic [XLEN_BYTES-1:0] store_mask,

    // Combinational load query
    input  logic [MEMORY_ADDR_WIDTH-1:0] load_addr,
    output logic        load_hit,
    output logic [XLEN-1:0] load_data,

    output logic        irq_m_timer,
    output logic        irq_m_software,
    output logic [63:0] mtime_out
);

    localparam int ADDR_SHIFT = $clog2(XLEN_BYTES);
    localparam int NSUB = XLEN / 32;       // 32-bit registers per bus word

    localparam logic [31:0] MSIP_B      = BASE + 32'h0000;
    localparam logic [31:0] MTIMECMP_LO = BASE + 32'h4000;
    localparam logic [31:0] MTIMECMP_HI = BASE + 32'h4004;
    localparam logic [31:0] MTIME_LO    = BASE + 32'hBFF8;
    localparam logic [31:0] MTIME_HI    = BASE + 32'hBFFC;

    logic [63:0] mtime_q;
    logic [63:0] mtimecmp_q;
    logic        msip_q;

    assign mtime_out      = mtime_q;
    assign irq_m_timer    = (mtime_q >= mtimecmp_q);
    assign irq_m_software = msip_q;

    // 32-bit register read at a byte address; hit reports whether the address
    // decodes to a CLINT register.
    function automatic logic [31:0] reg_read(input logic [31:0] baddr,
            output logic hit);
        hit = 1'b1;
        unique case (baddr)
            MSIP_B:      reg_read = {31'b0, msip_q};
            MTIMECMP_LO: reg_read = mtimecmp_q[31:0];
            MTIMECMP_HI: reg_read = mtimecmp_q[63:32];
            MTIME_LO:    reg_read = mtime_q[31:0];
            MTIME_HI:    reg_read = mtime_q[63:32];
            default: begin
                reg_read = 32'b0;
                hit = 1'b0;
            end
        endcase
    endfunction

    // Combinational read path: each bus subword decodes independently.
    always_comb begin
        logic sub_hit;
        load_hit  = 1'b0;
        load_data = '0;
        for (int i = 0; i < NSUB; i += 1) begin
            load_data[i*32 +: 32] = reg_read(
                32'(load_addr << ADDR_SHIFT) + 32'(unsigned'(i) * 4), sub_hit);
            load_hit |= sub_hit;
        end
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
            if (store_en && (store_mask != '0)) begin
                for (int i = 0; i < NSUB; i += 1) begin
                    logic [31:0] baddr;
                    logic [31:0] wsub;
                    logic [3:0]  msub;
                    baddr = 32'(store_waddr << ADDR_SHIFT) + 32'(unsigned'(i) * 4);
                    wsub  = store_wdata[i*32 +: 32];
                    msub  = store_mask[i*4 +: 4];
                    if (msub != 4'b0) begin
                        unique case (baddr)
                            MSIP_B:      if (msub[0]) msip_q <= wsub[0];
                            MTIMECMP_LO: mtimecmp_q[31:0]  <= merge(mtimecmp_q[31:0],  wsub, msub);
                            MTIMECMP_HI: mtimecmp_q[63:32] <= merge(mtimecmp_q[63:32], wsub, msub);
                            MTIME_LO:    mtime_q[31:0]  <= merge(mtime_q[31:0],  wsub, msub);
                            MTIME_HI:    mtime_q[63:32] <= merge(mtime_q[63:32], wsub, msub);
                            default: ;
                        endcase
                    end
                end
            end
        end
    end

endmodule: clint
