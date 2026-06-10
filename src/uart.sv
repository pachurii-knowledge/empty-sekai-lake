/**
 * uart.sv
 *
 * NS16550A UART for the simulation console, modelled closely enough that the
 * Linux `8250`/`of_serial` driver probes and drives it. It snoops the core's
 * data store port (like clint.sv / plic.sv) and serves a combinational load
 * path; `load_en` marks the cycle a load result is actually consumed, so
 * register reads with side effects (IIR-read clears the THRE interrupt;
 * RBR-read consumes an RX byte) are not taken speculatively.
 *
 * Register layout: `reg-shift = 2`, `reg-io-width = 4` (a 4-byte stride per
 * architectural register). Byte offsets from BASE (default 0x0D00_0000):
 *   +0x00  RBR (read) / THR (write)   [+DLL when LCR.DLAB]
 *   +0x04  IER                         [+DLM when LCR.DLAB]
 *   +0x08  IIR (read) / FCR (write)
 *   +0x0C  LCR  (bit7 = DLAB)
 *   +0x10  MCR  (bit4 = LOOP)
 *   +0x14  LSR  (bit0 DR, bit5 THRE, bit6 TEMT)
 *   +0x18  MSR  (loopback-mapped from MCR when LOOP=1, for autoconfig)
 *   +0x1C  SCR  (scratch)
 *
 * The bus is one memory word wide (4 bytes at RV32, 8 at RV64). On RV64 a
 * register pair shares an 8-byte word, so the byte offset within the word is
 * needed to pick the register a load actually read (head_load_off) and a store
 * actually wrote (the byte mask). Stores decode per 32-bit subword; the read
 * data path returns every subword's register and lets the LSQ extract the
 * accessed bytes, while read side effects use head_load_off.
 *
 * The transmitter is always ready (THRE|TEMT set), so a written THR byte is
 * emitted to the console ($write) immediately -- unless MCR.LOOP is set, in
 * which case it loops back to RBR. RX data for an interactive console is
 * injected from the +uart_in=<string> plusarg, one byte at a time as the
 * driver consumes RBR.
 */

`include "riscv_isa.vh"
`include "riscv_uarch.vh"

`default_nettype none

module uart
    import RISCV_ISA::XLEN, RISCV_ISA::XLEN_BYTES;
    import RISCV_UArch::MEMORY_ADDR_WIDTH;
#(
    parameter logic [31:0] BASE = 32'h0D00_0000
) (
    input  logic        clk,
    input  logic        rst_l,

    // Data store snoop (memory-word address space)
    input  logic        store_en,
    input  logic [MEMORY_ADDR_WIDTH-1:0] store_waddr,
    input  logic [XLEN-1:0] store_wdata,
    input  logic [XLEN_BYTES-1:0] store_mask,

    // Combinational load query; load_en marks the cycle the load is consumed
    // (so RBR/IIR read side effects are not taken speculatively). head_load_off
    // is the byte offset of the consuming load within the bus word.
    input  logic [MEMORY_ADDR_WIDTH-1:0] load_addr,
    input  logic        load_en,
    input  logic [$clog2(XLEN_BYTES)-1:0] load_off,
    output logic        load_hit,
    output logic [XLEN-1:0] load_data,

    // Interrupt request (level) -> PLIC source
    output logic        irq
);

    localparam int ADDR_SHIFT = $clog2(XLEN_BYTES);
    localparam int NSUB = XLEN / 32;

    // Architectural registers
    logic [7:0] ier_q;     // interrupt enable (bit0 ERBFI rx, bit1 ETBEI tx)
    logic [7:0] fcr_q;     // FIFO control (bit0 = FIFO enable)
    logic [7:0] lcr_q;     // line control (bit7 = DLAB)
    logic [7:0] mcr_q;     // modem control (bit4 = LOOP)
    logic [7:0] scr_q;     // scratch
    logic [7:0] dll_q, dlm_q;   // divisor latch (accepted, baud is irrelevant)
    logic [7:0] rx_data_q; // RBR
    logic       rx_valid_q;     // LSR.DR
    logic       thre_pending_q; // THR-empty interrupt latched

    logic       dlab;
    assign dlab = lcr_q[7];

    // Interrupt causes; RX (received-data) outranks TX (THR-empty) in IIR.
    logic rx_int, tx_int;
    assign rx_int = rx_valid_q     & ier_q[0];
    assign tx_int = thre_pending_q & ier_q[1];

    // RX injection from +uart_in=<string>
    string rx_str;
    int    rx_idx;

    // Map a byte address to a register index 0..7, or 4'hF if out of range.
    function automatic logic [3:0] reg_index(input logic [31:0] baddr);
        if ((baddr >= BASE) && (baddr < BASE + 32) && (baddr[1:0] == 2'b00))
            reg_index = 4'((baddr - BASE) >> 2);
        else
            reg_index = 4'hF;
    endfunction

    // ---------------- writes / state ----------------
    always_ff @(posedge clk or negedge rst_l) begin
        if (!rst_l) begin
            ier_q <= 8'h0; fcr_q <= 8'h0; lcr_q <= 8'h0; mcr_q <= 8'h0;
            scr_q <= 8'h0; dll_q <= 8'h0; dlm_q <= 8'h0;
            rx_data_q <= 8'h0; rx_valid_q <= 1'b0; thre_pending_q <= 1'b0;
            rx_idx <= 0;
            if (!$value$plusargs("uart_in=%s", rx_str)) rx_str = "";
        end else begin
            // Register writes (committed stores), decoded per 32-bit subword.
            // The written byte is wsub[7:0] (a register is at a 4-byte-aligned
            // address, so a byte sb at offset 0 and a 32-bit writel agree).
            if (store_en) begin
                for (int i = 0; i < NSUB; i += 1) begin
                    logic [3:0]  ridx;
                    logic [31:0] wsub;
                    logic [3:0]  msub;
                    ridx = reg_index(32'(store_waddr << ADDR_SHIFT) +
                                     32'(unsigned'(i) * 4));
                    wsub = store_wdata[i*32 +: 32];
                    msub = store_mask[i*4 +: 4];
                    if (msub[0] && (ridx != 4'hF)) begin
                        case (ridx)
                            4'd0: begin
                                if (dlab) dll_q <= wsub[7:0];
                                else begin
                                    // THR: console out, or loop back under LOOP.
                                    if (mcr_q[4]) begin
                                        rx_data_q  <= wsub[7:0];
                                        rx_valid_q <= 1'b1;
                                    end else begin
                                        $write("%c", wsub[7:0]);
                                        $fflush();
                                    end
                                    thre_pending_q <= 1'b1;  // TX empties at once
                                end
                            end
                            4'd1: begin
                                if (dlab) dlm_q <= wsub[7:0];
                                else begin
                                    if (wsub[1] && !ier_q[1]) thre_pending_q <= 1'b1;
                                    ier_q <= wsub[7:0];
                                end
                            end
                            4'd2: fcr_q <= wsub[7:0];   // FCR
                            4'd3: lcr_q <= wsub[7:0];   // LCR
                            4'd4: mcr_q <= wsub[7:0];   // MCR
                            4'd7: scr_q <= wsub[7:0];   // SCR
                            default: ;                   // LSR/MSR read-only
                        endcase
                    end
                end
            end

            // Read side effects (only on a non-speculatively consumed load), on
            // the register the load actually addressed.
            if (load_en) begin
                logic [3:0] ridx;
                ridx = reg_index(32'(load_addr << ADDR_SHIFT) + 32'(load_off));
                // IIR read acknowledges a THR-empty interrupt (TX is the cause).
                if (ridx == 4'd2 && tx_int && !rx_int)
                    thre_pending_q <= 1'b0;
                // RBR read consumes the RX byte and advances the injector.
                if (ridx == 4'd0 && !dlab && rx_valid_q)
                    rx_valid_q <= 1'b0;
            end

            // Deliver the next injected RX byte once the previous is consumed
            // (not while looping back, which drives RBR itself).
            if (!rx_valid_q && !mcr_q[4] && rx_idx < rx_str.len()) begin
                rx_data_q  <= 8'(rx_str[rx_idx]);
                rx_valid_q <= 1'b1;
                rx_idx     <= rx_idx + 1;
            end
        end
    end

    // ---------------- combinational reads ----------------
    logic [7:0] iir_val, lsr_val, msr_val;
    logic [3:0] iir_id;
    assign iir_id  = rx_int ? 4'h4 : (tx_int ? 4'h2 : 4'h1);  // bit0=1 => none
    assign iir_val = {(fcr_q[0] ? 2'b11 : 2'b00), 2'b00, iir_id};
    assign lsr_val = {1'b0, 1'b1, 1'b1, 4'b0, rx_valid_q};    // TEMT|THRE|DR
    assign msr_val = mcr_q[4]
                   ? {mcr_q[3], mcr_q[2], mcr_q[0], mcr_q[1], 4'b0}
                   : 8'h00;

    // Read a register by index 0..7 (combinational; no side effects here).
    function automatic logic [7:0] read_reg(input logic [3:0] ridx);
        unique case (ridx)
            4'd0:    read_reg = dlab ? dll_q : rx_data_q;   // DLL / RBR
            4'd1:    read_reg = dlab ? dlm_q : ier_q;       // DLM / IER
            4'd2:    read_reg = iir_val;                    // IIR
            4'd3:    read_reg = lcr_q;                      // LCR
            4'd4:    read_reg = mcr_q;                      // MCR
            4'd5:    read_reg = lsr_val;                    // LSR
            4'd6:    read_reg = msr_val;                    // MSR
            default: read_reg = scr_q;                      // SCR
        endcase
    endfunction

    // Return each accessed subword's register; the LSQ extracts the bytes the
    // load wants. load_hit if any subword decodes to a UART register.
    always_comb begin
        load_hit  = 1'b0;
        load_data = '0;
        for (int i = 0; i < NSUB; i += 1) begin
            logic [3:0] ridx;
            ridx = reg_index(32'(load_addr << ADDR_SHIFT) + 32'(unsigned'(i) * 4));
            if (ridx != 4'hF) begin
                load_data[i*32 +: 32] = {24'b0, read_reg(ridx)};
                load_hit = 1'b1;
            end
        end
    end

    assign irq = rx_int | tx_int;

endmodule: uart

`default_nettype wire
