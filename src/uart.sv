/**
 * uart.sv
 *
 * Minimal NS16550-subset UART for the simulation console. Snoops the core's
 * word-addressed data store port (like clint.sv / plic.sv) and serves a
 * combinational load path for register reads. A byte written to THR is emitted
 * to the simulation console; the transmitter is always ready (LSR.THRE/TEMT=1),
 * so software polling (the common SBI/early-console path) works without RX.
 *
 * BASE sits in the device hole between the text (0x0040_0000) and data
 * (0x1000_0000) segments, alongside the CLINT (0x0200_0000) and PLIC
 * (0x0C00_0000) -- 0x1000_0000 itself is arch-test RAM. (A DTB-driven Linux map
 * later relocates RAM to 0x8000_0000 and can move the UART to the usual
 * 0x1000_0000.)
 *
 * Register map (byte offsets from BASE, default 0x0D00_0000), packed into two
 * 32-bit words on the word-addressed bus:
 *   word 0 (BASE+0): [0]=THR/RBR(+DLL when DLAB)  [1]=IER(+DLM)  [2]=IIR/FCR  [3]=LCR
 *   word 1 (BASE+4): [0]=MCR  [1]=LSR  [2]=MSR  [3]=SCR
 *
 * LSR (BASE+5): bit0 DR (rx data ready, always 0 here), bit5 THRE (tx holding
 * empty), bit6 TEMT (tx empty) -> reads 0x60. The interrupt line is asserted
 * while the THR-empty interrupt is enabled (IER bit1); it is a PLIC source.
 */

`default_nettype none

module uart #(
    parameter logic [31:0] BASE = 32'h0D00_0000
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

    // Interrupt request (level) -> PLIC source
    output logic        irq
);

    localparam logic [29:0] WORD0 = 30'((BASE + 32'h0) >> 2);   // THR/IER/IIR/LCR
    localparam logic [29:0] WORD1 = 30'((BASE + 32'h4) >> 2);   // MCR/LSR/MSR/SCR

    // Registers
    logic [7:0] ier_q;     // interrupt enable
    logic [7:0] lcr_q;     // line control (bit7 = DLAB)
    logic [7:0] scr_q;     // scratch
    logic       dlab;
    assign dlab = lcr_q[7];

    // Transmitter is always ready: THRE (bit5) and TEMT (bit6) set, no rx data.
    localparam logic [7:0] LSR_VAL = 8'h60;

    always_ff @(posedge clk or negedge rst_l) begin
        if (!rst_l) begin
            ier_q <= 8'h0;
            lcr_q <= 8'h0;
            scr_q <= 8'h0;
        end else if (store_en) begin
            if (store_waddr == WORD0) begin
                // byte1 = IER (only when DLAB=0; otherwise DLM, ignored)
                if (store_mask[1] && !dlab) ier_q <= store_wdata[15:8];
                // byte3 = LCR
                if (store_mask[3]) lcr_q <= store_wdata[31:24];
                // byte0 = THR (only when DLAB=0): emit to the console.
                if (store_mask[0] && !dlab) begin
                    $write("%c", store_wdata[7:0]);
                    $fflush();
                end
            end else if (store_waddr == WORD1) begin
                if (store_mask[3]) scr_q <= store_wdata[31:24];  // byte3 = SCR
            end
        end
    end

    // Combinational reads. THR-empty interrupt is identified in IIR when pending.
    logic       tx_int;
    assign tx_int = ier_q[1];               // THRE always true -> int while enabled
    always_comb begin
        load_hit  = 1'b0;
        load_data = 32'b0;
        if (load_addr == WORD0) begin
            load_hit = 1'b1;
            // [0]=RBR(0) [1]=IER [2]=IIR [3]=LCR. IIR: 0x02 = THR-empty, else 0x01.
            load_data = {lcr_q, (tx_int ? 8'h02 : 8'h01), ier_q, 8'h00};
        end else if (load_addr == WORD1) begin
            load_hit = 1'b1;
            // [0]=MCR(0) [1]=LSR [2]=MSR(0) [3]=SCR
            load_data = {scr_q, 8'h00, LSR_VAL, 8'h00};
        end
    end

    assign irq = tx_int;

endmodule: uart

`default_nettype wire
