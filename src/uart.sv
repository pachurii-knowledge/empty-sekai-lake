/**
 * uart.sv
 *
 * NS16550A UART for the simulation console, modelled closely enough that the
 * Linux `8250`/`of_serial` driver probes and drives it. It snoops the core's
 * word-addressed data store port (like clint.sv / plic.sv) and serves a
 * combinational load path; `load_en` marks the cycle a load result is actually
 * consumed, so register reads with side effects (IIR-read clears the THRE
 * interrupt; RBR-read consumes an RX byte) are not taken speculatively -- the
 * same contract plic.sv uses for claim reads.
 *
 * Register layout: one register per 32-bit word (`reg-shift = 2`,
 * `reg-io-width = 4` in the device tree), which both matches a well-supported
 * Linux NS16550 configuration and maps cleanly onto the word-addressed device
 * bus (each architectural register gets its own word address, so RBR vs IIR vs
 * LSR reads are distinguishable -- impossible with the 8 byte-spaced registers
 * packed into 2 words). Byte offsets from BASE (default 0x0D00_0000):
 *   +0x00 (word0)  RBR (read) / THR (write)   [+DLL when LCR.DLAB]
 *   +0x04 (word1)  IER                         [+DLM when LCR.DLAB]
 *   +0x08 (word2)  IIR (read) / FCR (write)
 *   +0x0C (word3)  LCR  (bit7 = DLAB)
 *   +0x10 (word4)  MCR  (bit4 = LOOP)
 *   +0x14 (word5)  LSR  (bit0 DR, bit5 THRE, bit6 TEMT)
 *   +0x18 (word6)  MSR  (loopback-mapped from MCR when LOOP=1, for autoconfig)
 *   +0x1C (word7)  SCR  (scratch)
 *
 * The transmitter is always ready (THRE|TEMT set), so a written THR byte is
 * emitted to the console ($write) immediately -- unless MCR.LOOP is set, in
 * which case it loops back to RBR (exercised by the driver's autoconfig). RX
 * data for an interactive console is injected from the +uart_in=<string>
 * plusarg, delivered one byte at a time as the driver consumes RBR.
 *
 * The interrupt line (a PLIC source) follows IER: received-data-available
 * (IER.0) and THR-empty (IER.1), with RX higher priority in IIR.
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

    // Combinational load query; load_en marks the cycle the load is consumed
    // (so RBR/IIR read side effects are not taken speculatively).
    input  logic [29:0] load_addr,
    input  logic        load_en,
    output logic        load_hit,
    output logic [31:0] load_data,

    // Interrupt request (level) -> PLIC source
    output logic        irq
);

    // Word base; each register r is at word address WBASE + r (reg-shift=2).
    localparam logic [29:0] WBASE = 30'(BASE >> 2);

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

    // Address decode (in range and which register)
    logic        in_range;
    logic [2:0]  reg_sel;
    logic [2:0]  st_sel;
    assign in_range = (load_addr  >= WBASE) && (load_addr  < WBASE + 8);
    assign reg_sel  = 3'(load_addr  - WBASE);
    assign st_sel   = 3'(store_waddr - WBASE);

    // Interrupt causes; RX (received-data) outranks TX (THR-empty) in IIR.
    logic rx_int, tx_int;
    assign rx_int = rx_valid_q     & ier_q[0];
    assign tx_int = thre_pending_q & ier_q[1];

    // RX injection from +uart_in=<string>
    string rx_str;
    int    rx_idx;

    // ---------------- writes / state ----------------
    logic store_here;
    assign store_here = store_en && (store_waddr >= WBASE) && (store_waddr < WBASE + 8)
                        && store_mask[0];

    always_ff @(posedge clk or negedge rst_l) begin
        if (!rst_l) begin
            ier_q <= 8'h0; fcr_q <= 8'h0; lcr_q <= 8'h0; mcr_q <= 8'h0;
            scr_q <= 8'h0; dll_q <= 8'h0; dlm_q <= 8'h0;
            rx_data_q <= 8'h0; rx_valid_q <= 1'b0; thre_pending_q <= 1'b0;
            rx_idx <= 0;
            if (!$value$plusargs("uart_in=%s", rx_str)) rx_str = "";
        end else begin
            // Register writes (committed stores). Value is always in wdata[7:0]
            // because each register is at a word-aligned address (works for both
            // a byte `sb` at offset 0 and a 32-bit `writel`).
            if (store_here) begin
                case (st_sel)
                    3'd0: begin
                        if (dlab) dll_q <= store_wdata[7:0];
                        else begin
                            // THR: console out, or loop back to RBR under MCR.LOOP.
                            if (mcr_q[4]) begin
                                rx_data_q  <= store_wdata[7:0];
                                rx_valid_q <= 1'b1;
                            end else begin
                                $write("%c", store_wdata[7:0]);
                                $fflush();
                            end
                            thre_pending_q <= 1'b1;   // TX empties immediately
                        end
                    end
                    3'd1: begin
                        if (dlab) dlm_q <= store_wdata[7:0];
                        else begin
                            // Enabling ETBEI arms a THR-empty interrupt at once.
                            if (store_wdata[1] && !ier_q[1]) thre_pending_q <= 1'b1;
                            ier_q <= store_wdata[7:0];
                        end
                    end
                    3'd2: fcr_q <= store_wdata[7:0];   // FCR
                    3'd3: lcr_q <= store_wdata[7:0];   // LCR
                    3'd4: mcr_q <= store_wdata[7:0];   // MCR
                    3'd7: scr_q <= store_wdata[7:0];   // SCR
                    default: ;                          // LSR/MSR read-only
                endcase
            end

            // Read side effects (only on a non-speculatively consumed load).
            if (load_en && in_range) begin
                // IIR read acknowledges a THR-empty interrupt (when TX is the
                // reported cause).
                if (reg_sel == 3'd2 && tx_int && !rx_int)
                    thre_pending_q <= 1'b0;
                // RBR read consumes the RX byte and advances the injector.
                if (reg_sel == 3'd0 && !dlab && rx_valid_q) begin
                    rx_valid_q <= 1'b0;
                end
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
    // Modem status: in loopback the MCR control bits appear in MSR (autoconfig
    // checks MSR&0xF0 == 0x90 after writing MCR = LOOP|OUT2|RTS).
    assign msr_val = mcr_q[4]
                   ? {mcr_q[3], mcr_q[2], mcr_q[0], mcr_q[1], 4'b0} // DCD,RI,DSR,CTS
                   : 8'h00;

    logic [7:0] rd_byte;
    always_comb begin
        unique case (reg_sel)
            3'd0:    rd_byte = dlab ? dll_q : rx_data_q;   // DLL / RBR
            3'd1:    rd_byte = dlab ? dlm_q : ier_q;       // DLM / IER
            3'd2:    rd_byte = iir_val;                    // IIR
            3'd3:    rd_byte = lcr_q;                      // LCR
            3'd4:    rd_byte = mcr_q;                      // MCR
            3'd5:    rd_byte = lsr_val;                    // LSR
            3'd6:    rd_byte = msr_val;                    // MSR
            default: rd_byte = scr_q;                      // SCR
        endcase
        load_hit  = in_range;
        load_data = in_range ? {24'b0, rd_byte} : 32'b0;
    end

    assign irq = rx_int | tx_int;

endmodule: uart

`default_nettype wire
