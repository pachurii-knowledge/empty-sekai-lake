/**
 * uart_host_fifo.sv  (FB1)
 *
 * Small synchronous first-word-fall-through (FWFT) FIFO used as the vUART byte
 * buffers between the core's NS16550 UART and the host OCL control plane:
 *   - TX FIFO  (core -> host): pushed by the UART on each THR byte, popped by
 *     the host reading OCL UART_TX.
 *   - RX FIFO  (host -> core): pushed by the host writing OCL UART_RX, popped
 *     by the UART when it accepts a byte into RBR.
 *
 * FWFT: `dout` always reflects the current head when `!empty`, so the UART RX
 * path can read `dout`/`!empty` combinationally and pulse `rd_en` to advance.
 * Single clock domain (clk_main_a0); host and core both run on it. DEPTH must
 * be a power of two. Distributed-RAM sized (<= a few Kbit), async-read head.
 */
`default_nettype none

module uart_host_fifo #(
    parameter int WIDTH = 8,
    parameter int DEPTH = 256          // power of two
) (
    input  wire logic             clk,
    input  wire logic             rst_l,        // active-low synchronous reset

    input  wire logic             wr_en,
    input  wire logic [WIDTH-1:0] wr_data,
    output logic                  full,

    input  wire logic             rd_en,
    output logic [WIDTH-1:0]      rd_data,      // head (valid when !empty)
    output logic                  empty,

    output logic [$clog2(DEPTH):0] count        // 0..DEPTH
);
    localparam int AW = $clog2(DEPTH);

    logic [WIDTH-1:0]  mem [DEPTH];
    logic [AW-1:0]     head_q, tail_q;
    logic [AW:0]       count_q;

    // Accept a write only if not full (or if a simultaneous read frees a slot);
    // accept a read only if not empty. Standard guarded FIFO semantics.
    logic do_wr, do_rd;
    assign do_wr = wr_en && (!full  || do_rd);
    assign do_rd = rd_en && !empty;

    assign full    = (count_q == (AW+1)'(DEPTH));
    assign empty   = (count_q == '0);
    assign rd_data = mem[head_q];
    assign count   = count_q;

    always_ff @(posedge clk or negedge rst_l) begin
        if (!rst_l) begin
            head_q  <= '0;
            tail_q  <= '0;
            count_q <= '0;
        end else begin
            if (do_wr) begin
                mem[tail_q] <= wr_data;
                tail_q      <= tail_q + AW'(1);
            end
            if (do_rd) begin
                head_q <= head_q + AW'(1);
            end
            unique case ({do_wr, do_rd})
                2'b10:   count_q <= count_q + (AW+1)'(1);
                2'b01:   count_q <= count_q - (AW+1)'(1);
                default: count_q <= count_q;          // 00 or 11: no net change
            endcase
        end
    end

endmodule : uart_host_fifo

`default_nettype wire
