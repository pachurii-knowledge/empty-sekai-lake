/**
 * plic.sv
 *
 * Minimal SiFive/RISC-V Platform-Level Interrupt Controller for a single hart
 * with two contexts (context 0 = M-mode external, context 1 = S-mode external).
 * Implements the standard register map at BASE (default 0x0C00_0000):
 *
 *   BASE + 0x000000 + 4*s      : priority of source s        (s = 1..NSOURCES)
 *   BASE + 0x001000            : interrupt-pending bits       (bit s = source s)
 *   BASE + 0x002000 + 0x80*ctx : interrupt-enable bits for context ctx
 *   BASE + 0x200000 + 0x1000*c : priority threshold for context c (offset 0)
 *   BASE + 0x200004 + 0x1000*c : claim/complete for context c   (offset 4)
 *
 * Sources are level inputs (src_irq), e.g. from a UART. For software-driven
 * injection (arch tests, IPI-style use) a write to the pending word sets the
 * corresponding source's software-pending latch; it is cleared on claim. Reads
 * are combinational; the store port snoops committed writes (like clint.sv).
 *
 * Claiming: reading a context's claim register returns the highest-priority
 * enabled & pending source (lowest id breaks ties) and marks it in flight
 * (gateway closed) so it stops contributing until completed. The in-flight set
 * happens when the claim load actually returns (load_en), not speculatively.
 * Completing: writing the source id back to the claim/complete register reopens
 * the gateway.
 *
 * The bus is one memory word wide (4 bytes at RV32, 8 at RV64); each bus word
 * decodes as XLEN/32 32-bit register subwords. Because a claim READ has a side
 * effect, the claim is taken only when the load's actual byte range (load_off/
 * load_size from the LSQ head) overlaps the claim register's subword -- a
 * 32-bit read of the adjacent threshold register must not claim.
 */

`include "riscv_isa.vh"
`include "riscv_uarch.vh"

`default_nettype none

module plic
    import RISCV_ISA::XLEN, RISCV_ISA::XLEN_BYTES;
    import RISCV_UArch::MEMORY_ADDR_WIDTH;
#(
    parameter logic [31:0] BASE = 32'h0C00_0000,
    parameter int          NSOURCES = 31           // sources 1..NSOURCES (<= 31)
) (
    input wire logic        clk,
    input wire logic        rst_l,

    // Level interrupt lines from devices (index 0 unused; source 0 is reserved).
    input wire logic [NSOURCES:0] src_irq,

    // Committed data store snoop (memory-word address space)
    input wire logic        store_en,
    input wire logic [MEMORY_ADDR_WIDTH-1:0] store_waddr,
    input wire logic [XLEN-1:0] store_wdata,
    input wire logic [XLEN_BYTES-1:0] store_mask,

    // Combinational load query; load_en marks the cycle the load result is
    // actually consumed (so a claim's side effect is not taken speculatively).
    // load_off is the consuming access's byte offset within the memory word,
    // for read-side-effect gating (which 32-bit register was read).
    input wire logic [MEMORY_ADDR_WIDTH-1:0] load_addr,
    input wire logic        load_en,
    input wire logic [$clog2(XLEN_BYTES)-1:0] load_off,
    output logic        load_hit,
    output logic [XLEN-1:0] load_data,

    output logic        irq_m_external,   // context 0
    output logic        irq_s_external    // context 1
);

    localparam int ADDR_SHIFT = $clog2(XLEN_BYTES);
    localparam int NSUB = XLEN / 32;

    localparam int NCTX = 2;
    localparam logic [31:0] PRIO_BASE_B  = BASE + 32'h0000_0000;
    localparam logic [31:0] PENDING_B    = BASE + 32'h0000_1000;
    // Non-standard test/IPI helper: write-1-to-clear the software-pending word.
    localparam logic [31:0] PENDING_CLR_B = BASE + 32'h0000_1004;
    localparam logic [31:0] ENABLE0_B    = BASE + 32'h0000_2000;
    localparam logic [31:0] ENABLE1_B    = BASE + 32'h0000_2080;
    localparam logic [31:0] THRESH0_B    = BASE + 32'h0020_0000;
    localparam logic [31:0] CLAIM0_B     = BASE + 32'h0020_0004;
    localparam logic [31:0] THRESH1_B    = BASE + 32'h0020_1000;
    localparam logic [31:0] CLAIM1_B     = BASE + 32'h0020_1004;

    // State
    logic [2:0]  prio_q     [NSOURCES+1];   // priority per source (0 disables)
    logic        sw_pend_q  [NSOURCES+1];   // software-injected pending latch
    logic        inflight_q [NSOURCES+1];   // claimed, not yet completed
    logic [31:0] enable_q   [NCTX];         // bit s = source s enabled for ctx
    logic [2:0]  thresh_q   [NCTX];

    // Gateway: a source contributes when asserted (line or sw latch) and not
    // currently in flight.
    logic gw_pending [NSOURCES+1];
    always_comb begin
        gw_pending[0] = 1'b0;
        for (int s = 1; s <= NSOURCES; s += 1)
            gw_pending[s] = (src_irq[s] | sw_pend_q[s]) & ~inflight_q[s];
    end

    // Per-context best (highest priority, lowest id) eligible source.
    logic [5:0]  best_id  [NCTX];
    logic [2:0]  best_pri [NCTX];
    // FB2b: parallel argmax replacing the serial NSOURCES-deep running-max scan
    // (best_pri/best_id chained through all 31 sources -- the PLIC read worst-path
    // cone feeding the LSQ device-read data_load, the post-R2' #1..#25 bottleneck).
    // Value-identical: `elig` is the per-source eligibility; the leading-bits loop
    // narrows `alive` to the eligible sources AT the max priority (each priority bit
    // from MSB: if any alive source has that bit, drop the alive sources lacking it);
    // best_pri is that priority and best_id the LOWEST surviving source (reverse scan,
    // matching the old strict-> tie-break = lowest id wins). Same winner, ~log-depth.
    always_comb begin
        for (int c = 0; c < NCTX; c += 1) begin
            logic [NSOURCES:0] elig;
            logic [NSOURCES:0] alive;
            logic [2:0]        bp;
            logic              any_b;
            elig = '0;
            for (int s = 1; s <= NSOURCES; s += 1) begin
                elig[s] = gw_pending[s] && enable_q[c][s] &&
                          (prio_q[s] > thresh_q[c]);
            end
            alive = elig;
            for (int b = 2; b >= 0; b -= 1) begin
                any_b = 1'b0;
                for (int s = 1; s <= NSOURCES; s += 1) begin
                    if (alive[s] && prio_q[s][b]) any_b = 1'b1;
                end
                bp[b] = any_b;
                if (any_b) begin
                    for (int s = 1; s <= NSOURCES; s += 1) begin
                        if (!prio_q[s][b]) alive[s] = 1'b0;
                    end
                end
            end
            best_pri[c] = bp;   // 0 iff no eligible source (alive all clear)
            best_id[c]  = 6'd0;
            for (int s = NSOURCES; s >= 1; s -= 1) begin
                if (alive[s]) best_id[c] = 6'(s);
            end
        end
    end

    assign irq_m_external = (best_id[0] != 6'd0);
    assign irq_s_external = (best_id[1] != 6'd0);

    // Pending-bits read word (bit s = source s gateway pending).
    logic [31:0] pending_word;
    always_comb begin
        pending_word = 32'b0;
        for (int s = 1; s <= NSOURCES; s += 1) pending_word[s] = gw_pending[s];
    end

    // 32-bit register read at a byte address.
    function automatic logic [31:0] reg_read(input logic [31:0] baddr,
            output logic hit);
        hit = 1'b1;
        if ((baddr >= PRIO_BASE_B + 32'd4) &&
                (baddr <= PRIO_BASE_B + 32'(NSOURCES * 4))) begin
            reg_read = {29'b0, prio_q[baddr[6:2]]};
        end else begin
            unique case (baddr)
                PENDING_B: reg_read = pending_word;
                ENABLE0_B: reg_read = enable_q[0];
                ENABLE1_B: reg_read = enable_q[1];
                THRESH0_B: reg_read = {29'b0, thresh_q[0]};
                THRESH1_B: reg_read = {29'b0, thresh_q[1]};
                CLAIM0_B:  reg_read = {26'b0, best_id[0]};
                CLAIM1_B:  reg_read = {26'b0, best_id[1]};
                default: begin
                    reg_read = 32'b0;
                    hit = 1'b0;
                end
            endcase
        end
    endfunction

    // ---- Combinational read path (per bus subword) ----
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

    // A claim load that actually completes marks its source in flight. The
    // claim register the load addressed is selected by its byte offset (a claim
    // read is a 32-bit access at the register's aligned address).
    logic        claim_take [NCTX];
    logic [5:0]  claim_id   [NCTX];
    logic [31:0] load_baddr;
    assign load_baddr = 32'(load_addr << ADDR_SHIFT) + 32'(load_off);
    always_comb begin
        for (int c = 0; c < NCTX; c += 1) begin
            claim_take[c] = 1'b0;
            claim_id[c]   = best_id[c];
        end
        if (load_en && (load_baddr == CLAIM0_B) && (best_id[0] != 6'd0))
            claim_take[0] = 1'b1;
        if (load_en && (load_baddr == CLAIM1_B) && (best_id[1] != 6'd0))
            claim_take[1] = 1'b1;
    end

    // Completion: a write of a source id to a claim/complete register reopens
    // that source's gateway. A write to the pending word sets sw-pending bits.
    always_ff @(posedge clk or negedge rst_l) begin
        if (!rst_l) begin
            for (int s = 0; s <= NSOURCES; s += 1) begin
                prio_q[s]     <= 3'b0;
                sw_pend_q[s]  <= 1'b0;
                inflight_q[s] <= 1'b0;
            end
            for (int c = 0; c < NCTX; c += 1) begin
                enable_q[c] <= 32'b0;
                thresh_q[c] <= 3'b0;
            end
        end else begin
            // Claim side effects (load): close the gateway, drop the sw latch.
            for (int c = 0; c < NCTX; c += 1) begin
                if (claim_take[c]) begin
                    inflight_q[claim_id[c]] <= 1'b1;
                    sw_pend_q [claim_id[c]] <= 1'b0;
                end
            end
            // Register writes (committed stores), per bus subword.
            if (store_en && (store_mask != '0)) begin
                for (int i = 0; i < NSUB; i += 1) begin
                    logic [31:0] baddr;
                    logic [31:0] wsub;
                    logic [3:0]  msub;
                    baddr = 32'(store_waddr << ADDR_SHIFT) + 32'(unsigned'(i) * 4);
                    wsub  = store_wdata[i*32 +: 32];
                    msub  = store_mask[i*4 +: 4];
                    if (msub != 4'b0) begin
                        if ((baddr >= PRIO_BASE_B + 32'd4) &&
                                (baddr <= PRIO_BASE_B + 32'(NSOURCES * 4))) begin
                            if (msub[0]) prio_q[baddr[6:2]] <= wsub[2:0];
                        end else begin
                            unique case (baddr)
                                PENDING_B: begin
                                    for (int s = 1; s <= NSOURCES; s += 1)
                                        if (wsub[s]) sw_pend_q[s] <= 1'b1;
                                end
                                PENDING_CLR_B: begin
                                    for (int s = 1; s <= NSOURCES; s += 1)
                                        if (wsub[s]) sw_pend_q[s] <= 1'b0;
                                end
                                ENABLE0_B: enable_q[0] <= wsub;
                                ENABLE1_B: enable_q[1] <= wsub;
                                THRESH0_B: if (msub[0]) thresh_q[0] <= wsub[2:0];
                                THRESH1_B: if (msub[0]) thresh_q[1] <= wsub[2:0];
                                CLAIM0_B:  if (wsub[5:0] <= 6'(NSOURCES))
                                               inflight_q[wsub[5:0]] <= 1'b0;
                                CLAIM1_B:  if (wsub[5:0] <= 6'(NSOURCES))
                                               inflight_q[wsub[5:0]] <= 1'b0;
                                default: ;
                            endcase
                        end
                    end
                end
            end
        end
    end

endmodule: plic
