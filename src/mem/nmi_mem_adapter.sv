/**
 * nmi_mem_adapter.sv  (SIMULATION ONLY)
 *
 * Terminates the NMI line bus onto main_memory's word ports for every
 * Verilator configuration that is not AXI=1 (DD-11: main_memory stays the sim
 * storage so image loading + the signature dumper + priv_diag keep working).
 * The AXI bridge (phase X1) replaces this module with nmi_axi_bridge +
 * axi_mem_shim.
 *
 * One NMI op at a time (<=1 outstanding). A line read is issued to the backing
 * memory as LINE_RD_BEATS reads of MEMORY_READ_WIDTH words each (4 at RV32, 2
 * at RV64); a line write drains LINE_WORDS single-word writes. The optional
 * latency fuzzer (shared +mem_fuzz/+mem_seed plusargs, its own LCG stream)
 * injects request-acceptance stalls and extra response latency so the caches
 * are exercised against variable refill timing; fuzz off is fixed-latency and
 * fully reproducible.
 */

`include "niigo_mem.vh"

`default_nettype none

module nmi_mem_adapter
    import RISCV_ISA::XLEN, RISCV_ISA::XLEN_BYTES;
    import RISCV_UArch::MEMORY_READ_WIDTH, RISCV_UArch::MEMORY_ADDR_WIDTH;
    import NIIGO_Mem::*;
(
    input wire logic clk,
    input wire logic rst_l,

    // ---- NMI slave ----
    input wire nmi_req_t  nmi_req,
    output logic      nmi_req_ready,
    output nmi_resp_t nmi_resp,

    // ---- Backend: main_memory read port (combinational, MEMORY_READ_WIDTH words) ----
    output logic [MEMORY_ADDR_WIDTH-1:0]            mem_rd_addr,
    input wire logic [MEMORY_READ_WIDTH-1:0][XLEN-1:0]  mem_rd_data,
    input wire logic                                    mem_rd_excpt,

    // ---- Backend: main_memory write port (one word per cycle) ----
    output logic                          mem_wr_en,
    output logic [MEMORY_ADDR_WIDTH-1:0]  mem_wr_addr,
    output logic [XLEN-1:0]               mem_wr_data,
    output logic [XLEN_BYTES-1:0]         mem_wr_mask,

    // ---- Fuzz config (driven by niigo_memsys; constant after init) ----
    input wire logic        fz_en,
    input int unsigned fz_seed,
    input int unsigned fz_min,
    input int unsigned fz_max
);

    // Words returned per backend read beat.
    localparam int RW = MEMORY_READ_WIDTH;

    typedef enum logic [2:0] {
        S_IDLE, S_RD_BEAT, S_RD_DELAY, S_WR_WORD, S_RESP
    } state_e;
    state_e state_q, state_n;

    nmi_op_e                      op_q,    op_n;
    logic [3:0]                   id_q,    id_n;
    logic [MEMORY_ADDR_WIDTH-1:0] base_q,  base_n;     // line-base word addr
    logic [LINE_BITS-1:0]         data_q,  data_n;     // assembled / write payload
    logic [XLEN_BYTES-1:0]        wmask_q, wmask_n;
    logic                         err_q,   err_n;
    logic [$clog2(LINE_WORDS+1)-1:0] cnt_q, cnt_n;     // beat / word counter
    logic [7:0]                   delay_q, delay_n;

    // LCG stream for the adapter's refill latency (seeded from +mem_seed).
    logic [31:0] lat_lcg;
    initial lat_lcg = 32'h4E4D4941;  // "NMIA"
    function automatic logic [31:0] lcg_next(input logic [31:0] s);
        lcg_next = s * 32'd1664525 + 32'd1013904223;
    endfunction
    function automatic int unsigned lcg_range(input logic [31:0] s,
            input int unsigned lo, input int unsigned hi);
        lcg_range = lo + (32'(s >> 8) % (hi - lo + 1));
    endfunction
    logic seeded_q;
    always_ff @(posedge clk or negedge rst_l) begin
        if (!rst_l) begin
            seeded_q <= 1'b0;
        end else if (!seeded_q) begin
            seeded_q <= 1'b1;
            lat_lcg  <= 32'(fz_seed) ^ 32'h4E4D4941;
        end else if (state_q == S_IDLE) begin
            lat_lcg  <= lcg_next(lat_lcg);
        end
    end

    // Accept a request only in IDLE.
    assign nmi_req_ready = (state_q == S_IDLE);
    logic accept;
    assign accept = nmi_req.valid && nmi_req_ready;

    // Beat/word read address and write payload selection.
    logic [$clog2(LINE_WORDS)-1:0] word_idx;
    assign word_idx = cnt_q[$clog2(LINE_WORDS)-1:0];

    always_comb begin
        // Read port: during a read beat present base + beat*RW.
        mem_rd_addr = base_q +
            MEMORY_ADDR_WIDTH'({cnt_q, {$clog2(RW){1'b0}}});
        // Write port (line/word writes).
        mem_wr_en   = 1'b0;
        mem_wr_addr = base_q + MEMORY_ADDR_WIDTH'(word_idx);
        mem_wr_data = data_q[word_idx*XLEN +: XLEN];
        mem_wr_mask = '1;
        if (state_q == S_WR_WORD) begin
            mem_wr_en = 1'b1;
            if (op_q == NMI_WR_WORD) begin
                mem_wr_addr = base_q;
                mem_wr_data = data_q[XLEN-1:0];
                mem_wr_mask = wmask_q;
            end
        end
    end

    // Next-state / datapath.
    always_comb begin
        state_n = state_q;
        op_n    = op_q;
        id_n    = id_q;
        base_n  = base_q;
        data_n  = data_q;
        wmask_n = wmask_q;
        err_n   = err_q;
        cnt_n   = cnt_q;
        delay_n = delay_q;

        unique case (state_q)
            S_IDLE: begin
                if (accept) begin
                    op_n    = nmi_req.op;
                    id_n    = nmi_req.id;
                    base_n  = (nmi_req.op == NMI_RD_LINE || nmi_req.op == NMI_WR_LINE)
                              ? l1_line_base(nmi_req.waddr) : nmi_req.waddr;
                    data_n  = nmi_req.wdata;
                    wmask_n = nmi_req.wmask;
                    err_n   = 1'b0;
                    cnt_n   = '0;
                    unique case (nmi_req.op)
                        NMI_RD_LINE, NMI_RD_WORDS: state_n = S_RD_BEAT;
                        NMI_WR_LINE, NMI_WR_WORD:  state_n = S_WR_WORD;
                        default:                   state_n = S_RESP;
                    endcase
                end
            end

            S_RD_BEAT: begin
                // Capture this beat's RW words into the line image.
                for (int k = 0; k < RW; k += 1) begin
                    data_n[(cnt_q*RW + k)*XLEN +: XLEN] = mem_rd_data[k];
                end
                err_n = err_q | mem_rd_excpt;
                if ((op_q == NMI_RD_WORDS) || (cnt_q == (LINE_RD_BEATS-1))) begin
                    cnt_n   = '0;
                    delay_n = fz_en ? 8'(lcg_range(lat_lcg, fz_min, fz_max)) : 8'd0;
                    state_n = S_RD_DELAY;
                end else begin
                    cnt_n = cnt_q + 1'b1;
                end
            end

            S_RD_DELAY: begin
                if (delay_q == '0) state_n = S_RESP;
                else               delay_n = delay_q - 1'b1;
            end

            S_WR_WORD: begin
                // One word per cycle; WR_WORD is a single beat.
                if ((op_q == NMI_WR_WORD) || (cnt_q == (LINE_WORDS-1))) begin
                    state_n = S_RESP;
                end else begin
                    cnt_n = cnt_q + 1'b1;
                end
            end

            S_RESP: begin
                state_n = S_IDLE;
            end

            default: state_n = S_IDLE;
        endcase
    end

    // Response (valid only).
    always_comb begin
        nmi_resp       = '0;
        nmi_resp.valid = (state_q == S_RESP);
        nmi_resp.id    = id_q;
        nmi_resp.rdata = data_q;
        nmi_resp.err   = err_q;
    end

    always_ff @(posedge clk or negedge rst_l) begin
        if (!rst_l) begin
            state_q <= S_IDLE;
            op_q    <= NMI_RD_LINE;
            id_q    <= '0;
            base_q  <= '0;
            data_q  <= '0;
            wmask_q <= '0;
            err_q   <= 1'b0;
            cnt_q   <= '0;
            delay_q <= '0;
        end else begin
            state_q <= state_n;
            op_q    <= op_n;
            id_q    <= id_n;
            base_q  <= base_n;
            data_q  <= data_n;
            wmask_q <= wmask_n;
            err_q   <= err_n;
            cnt_q   <= cnt_n;
            delay_q <= delay_n;
        end
    end

endmodule : nmi_mem_adapter

`default_nettype wire
