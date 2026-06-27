/**
 * clint.sv
 *
 * Minimal core-local interruptor (CLINT). SiFive-style memory-mapped registers,
 * now parametrized for NUM_HARTS harts (M4 SMP):
 *   BASE + 0x0000 + 4*h : msip[h]      (software interrupt, 1 bit)   h=0..NUM_HARTS-1
 *   BASE + 0x4000 + 8*h : mtimecmp[h]  (64-bit)
 *   BASE + 0xBFF8       : mtime        (64-bit, free running, SHARED)
 *
 * It snoops the core's data store port to capture writes and provides a
 * combinational load-hit/read path for reads of its registers. mtime drives
 * the per-hart machine timer interrupt (mtip[h] = mtime >= mtimecmp[h]) and
 * msip[h] drives the per-hart machine software interrupt (the IPI mechanism:
 * one hart writes another hart's msip register).
 *
 * NUM_HARTS=1 (the default, single-core build) places msip at BASE+0 and
 * mtimecmp at BASE+0x4000 -- byte-identical to the pre-SMP single-hart layout,
 * so the single-core CLINT is bit-identical.
 *
 * The bus is one memory word wide (4 bytes at RV32, 8 at RV64). The device
 * registers are 32-bit; each bus word is decoded as XLEN/32 subwords, so on
 * RV64 a single 8-byte access reads/writes an adjacent register pair (e.g. a
 * 64-bit ld/sd of mtime or an aligned mtimecmp[h] works naturally).
 */

`include "riscv_isa.vh"
`include "riscv_uarch.vh"

`default_nettype none

module clint
    import RISCV_ISA::XLEN, RISCV_ISA::XLEN_BYTES;
    import RISCV_UArch::MEMORY_ADDR_WIDTH;
#(
    parameter logic [31:0] BASE = 32'h0200_0000,
    parameter int          NUM_HARTS = 1
) (
    input wire logic        clk,
    input wire logic        rst_l,

    // Data store snoop (memory-word address space: byte addr >> log2(XLEN_BYTES))
    input wire logic        store_en,
    input wire logic [MEMORY_ADDR_WIDTH-1:0] store_waddr,
    input wire logic [XLEN-1:0] store_wdata,
    input wire logic [XLEN_BYTES-1:0] store_mask,

    // Combinational load query
    input wire logic [MEMORY_ADDR_WIDTH-1:0] load_addr,
    output logic        load_hit,
    output logic [XLEN-1:0] load_data,

    output logic [NUM_HARTS-1:0] irq_m_timer,
    output logic [NUM_HARTS-1:0] irq_m_software,
    output logic [63:0] mtime_out
);

    localparam int ADDR_SHIFT = $clog2(XLEN_BYTES);
    localparam int NSUB = XLEN / 32;       // 32-bit registers per bus word

    localparam logic [31:0] MSIP_BASE     = BASE + 32'h0000;        // +4*h
    localparam logic [31:0] MTIMECMP_BASE = BASE + 32'h4000;        // +8*h (lo), +4 hi
    localparam logic [31:0] MTIME_LO      = BASE + 32'hBFF8;
    localparam logic [31:0] MTIME_HI      = BASE + 32'hBFFC;

    logic [63:0] mtime_q;                     // shared free-running counter
    logic [63:0] mtimecmp_q [NUM_HARTS];
    logic        msip_q     [NUM_HARTS];

    assign mtime_out = mtime_q;
    for (genvar h = 0; h < NUM_HARTS; h += 1) begin : g_irq
        assign irq_m_timer[h]    = (mtime_q >= mtimecmp_q[h]);
        assign irq_m_software[h] = msip_q[h];
    end

    // Decode a 32-bit register byte address to (kind, hart).
    //   kind: 0=none 1=msip 2=mtimecmp_lo 3=mtimecmp_hi 4=mtime_lo 5=mtime_hi
    function automatic void decode(input logic [31:0] baddr,
            output int kind, output int hart);
        kind = 0;
        hart = 0;
        // msip[h] at MSIP_BASE + 4*h
        if ((baddr >= MSIP_BASE) && (baddr < MSIP_BASE + 32'(NUM_HARTS) * 32'd4) &&
                (baddr[1:0] == 2'b00)) begin
            kind = 1;
            hart = int'((baddr - MSIP_BASE) >> 2);
        end
        // mtimecmp[h] (64b) at MTIMECMP_BASE + 8*h
        else if ((baddr >= MTIMECMP_BASE) &&
                 (baddr < MTIMECMP_BASE + 32'(NUM_HARTS) * 32'd8) &&
                 (baddr[1:0] == 2'b00)) begin
            hart = int'((baddr - MTIMECMP_BASE) >> 3);
            kind = baddr[2] ? 3 : 2;   // +4 within the pair = hi word
        end
        else if (baddr == MTIME_LO) kind = 4;
        else if (baddr == MTIME_HI) kind = 5;
    endfunction

    function automatic logic [31:0] reg_read(input logic [31:0] baddr,
            output logic hit);
        int kind, hart;
        decode(baddr, kind, hart);
        hit = (kind != 0);
        unique case (kind)
            1:       reg_read = {31'b0, msip_q[hart]};
            2:       reg_read = mtimecmp_q[hart][31:0];
            3:       reg_read = mtimecmp_q[hart][63:32];
            4:       reg_read = mtime_q[31:0];
            5:       reg_read = mtime_q[63:32];
            default: reg_read = 32'b0;
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
            mtime_q <= 64'b0;
            for (int h = 0; h < NUM_HARTS; h += 1) begin
                mtimecmp_q[h] <= {64{1'b1}};
                msip_q[h]     <= 1'b0;
            end
        end else begin
            mtime_q <= mtime_q + 64'd1;
            if (store_en && (store_mask != '0)) begin
                for (int i = 0; i < NSUB; i += 1) begin
                    logic [31:0] baddr;
                    logic [31:0] wsub;
                    logic [3:0]  msub;
                    int          kind, hart;
                    baddr = 32'(store_waddr << ADDR_SHIFT) + 32'(unsigned'(i) * 4);
                    wsub  = store_wdata[i*32 +: 32];
                    msub  = store_mask[i*4 +: 4];
                    decode(baddr, kind, hart);
                    if (msub != 4'b0) begin
                        unique case (kind)
                            1:       if (msub[0]) msip_q[hart] <= wsub[0];
                            2:       mtimecmp_q[hart][31:0]  <= merge(mtimecmp_q[hart][31:0],  wsub, msub);
                            3:       mtimecmp_q[hart][63:32] <= merge(mtimecmp_q[hart][63:32], wsub, msub);
                            4:       mtime_q[31:0]  <= merge(mtime_q[31:0],  wsub, msub);
                            5:       mtime_q[63:32] <= merge(mtime_q[63:32], wsub, msub);
                            default: ;
                        endcase
                    end
                end
            end
        end
    end

endmodule: clint
