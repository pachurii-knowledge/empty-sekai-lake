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
 */

`default_nettype none

module plic #(
    parameter logic [31:0] BASE = 32'h0C00_0000,
    parameter int          NSOURCES = 31           // sources 1..NSOURCES (<= 31)
) (
    input  logic        clk,
    input  logic        rst_l,

    // Level interrupt lines from devices (index 0 unused; source 0 is reserved).
    input  logic [NSOURCES:0] src_irq,

    // Committed data store snoop (word address space: byte addr >> 2)
    input  logic        store_en,
    input  logic [29:0] store_waddr,
    input  logic [31:0] store_wdata,
    input  logic [3:0]  store_mask,

    // Combinational load query; load_en marks the cycle the load result is
    // actually consumed (so a claim's side effect is not taken speculatively).
    input  logic [29:0] load_addr,
    input  logic        load_en,
    output logic        load_hit,
    output logic [31:0] load_data,

    output logic        irq_m_external,   // context 0
    output logic        irq_s_external    // context 1
);

    localparam int NCTX = 2;
    localparam logic [29:0] PRIO_BASE  = 30'((BASE + 32'h0000_0000) >> 2);
    localparam logic [29:0] PENDING_W  = 30'((BASE + 32'h0000_1000) >> 2);
    localparam logic [29:0] ENABLE0_W  = 30'((BASE + 32'h0000_2000) >> 2);
    localparam logic [29:0] ENABLE1_W  = 30'((BASE + 32'h0000_2080) >> 2);
    localparam logic [29:0] THRESH0_W  = 30'((BASE + 32'h0020_0000) >> 2);
    localparam logic [29:0] CLAIM0_W   = 30'((BASE + 32'h0020_0004) >> 2);
    localparam logic [29:0] THRESH1_W  = 30'((BASE + 32'h0020_1000) >> 2);
    localparam logic [29:0] CLAIM1_W   = 30'((BASE + 32'h0020_1004) >> 2);

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
    always_comb begin
        for (int c = 0; c < NCTX; c += 1) begin
            best_id[c]  = 6'd0;
            best_pri[c] = 3'd0;
            for (int s = 1; s <= NSOURCES; s += 1) begin
                if (gw_pending[s] && enable_q[c][s] &&
                        (prio_q[s] > thresh_q[c]) && (prio_q[s] > best_pri[c])) begin
                    best_pri[c] = prio_q[s];
                    best_id[c]  = 6'(s);
                end
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

    // ---- Combinational read path ----
    always_comb begin
        load_hit  = 1'b1;
        load_data = 32'b0;
        if ((load_addr >= PRIO_BASE + 30'd1) &&
                (load_addr <= PRIO_BASE + 30'(NSOURCES))) begin
            load_data = {29'b0, prio_q[load_addr - PRIO_BASE]};
        end else begin
            unique case (load_addr)
                PENDING_W: load_data = pending_word;
                ENABLE0_W: load_data = enable_q[0];
                ENABLE1_W: load_data = enable_q[1];
                THRESH0_W: load_data = {29'b0, thresh_q[0]};
                THRESH1_W: load_data = {29'b0, thresh_q[1]};
                CLAIM0_W:  load_data = {26'b0, best_id[0]};
                CLAIM1_W:  load_data = {26'b0, best_id[1]};
                default:   load_hit  = 1'b0;
            endcase
        end
    end

    // A claim load that actually completes marks its source in flight.
    logic        claim_take [NCTX];
    logic [5:0]  claim_id   [NCTX];
    always_comb begin
        for (int c = 0; c < NCTX; c += 1) begin
            claim_take[c] = 1'b0;
            claim_id[c]   = best_id[c];
        end
        if (load_en && (best_id[0] != 6'd0) && (load_addr == CLAIM0_W))
            claim_take[0] = 1'b1;
        if (load_en && (best_id[1] != 6'd0) && (load_addr == CLAIM1_W))
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
            // Register writes (committed stores).
            if (store_en && (store_mask != 4'b0)) begin
                if ((store_waddr >= PRIO_BASE + 30'd1) &&
                        (store_waddr <= PRIO_BASE + 30'(NSOURCES))) begin
                    if (store_mask[0])
                        prio_q[store_waddr - PRIO_BASE] <= store_wdata[2:0];
                end else begin
                    unique case (store_waddr)
                        PENDING_W: begin
                            for (int s = 1; s <= NSOURCES; s += 1)
                                if (store_wdata[s]) sw_pend_q[s] <= 1'b1;
                        end
                        ENABLE0_W: enable_q[0] <= store_wdata;
                        ENABLE1_W: enable_q[1] <= store_wdata;
                        THRESH0_W: if (store_mask[0]) thresh_q[0] <= store_wdata[2:0];
                        THRESH1_W: if (store_mask[0]) thresh_q[1] <= store_wdata[2:0];
                        CLAIM0_W:  if (store_wdata[5:0] <= 6'(NSOURCES))
                                       inflight_q[store_wdata[5:0]] <= 1'b0;
                        CLAIM1_W:  if (store_wdata[5:0] <= 6'(NSOURCES))
                                       inflight_q[store_wdata[5:0]] <= 1'b0;
                        default: ;
                    endcase
                end
            end
        end
    end

endmodule: plic
