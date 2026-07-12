/**
 * nmi_arbiter.sv
 *
 * Fixed-priority arbiter that funnels N_MASTERS NMI requesters onto one
 * downstream NMI channel (the sim adapter or, at AXI=1, the AXI bridge).
 * Lower master index = higher priority. Per plan §5.6 fixed priority is fine:
 * each requester keeps <=1 op outstanding and the downstream serves one op at
 * a time, so the arbiter forwards a request, records which master owns it, and
 * routes the single response back to that master (no cross-id ordering is ever
 * assumed -- rule R1).
 *
 * C1 instantiates this with the L1I as the sole active master (the reserved
 * input is tied off); C2 connects the L1D path to the reserved input.
 */

`include "niigo_mem.vh"

`default_nettype none

module nmi_arbiter
    import NIIGO_Mem::*;
#(
    parameter int N_MASTERS = 2
) (
    input wire logic clk,
    input wire logic rst_l,

    // ---- Masters (index 0 highest priority) ----
    input wire nmi_req_t  m_req   [N_MASTERS],
    output logic      m_ready [N_MASTERS],
    output nmi_resp_t m_resp  [N_MASTERS],

    // ---- Downstream NMI ----
    output nmi_req_t  d_req,
    input wire logic      d_ready,
    input wire nmi_resp_t d_resp
);

    localparam int SEL_BITS = (N_MASTERS > 1) ? $clog2(N_MASTERS) : 1;

`ifdef LSQ_MLP2
    // Combinational fixed-priority selection among requesting masters.
    logic                grant_valid;
    logic [SEL_BITS-1:0] grant_sel;
    always_comb begin
        grant_valid = 1'b0;
        grant_sel   = '0;
        for (int i = N_MASTERS-1; i >= 0; i -= 1) begin
            if (m_req[i].valid) begin
                grant_valid = 1'b1;
                grant_sel   = SEL_BITS'(i);
            end
        end
    end

    // ---- P3d-1: N-outstanding context table, id-routed ----
    // Up to NCTX ops can be outstanding downstream at once; each is tagged with its issued NMI
    // id + owning master index at forward time, and each response is routed to the master whose
    // id it carries (not a single sel_q). CYCLE-IDENTICAL to the scalar arbiter below while the
    // downstream (sim adapter / AXI bridge) is single-outstanding -- d_ready then gates to one
    // op at a time so only one slot ever fills, and with <=1 outstanding the id-match owner ==
    // the last-granted master == sel_q. Load-bearing once P3d-3 makes the adapter multi-
    // outstanding (two L1D fills, or L1I||L1D, in flight -> responses reorder and must id-route).
    localparam int NCTX     = 2;
    localparam int NCTX_W   = (NCTX > 1) ? $clog2(NCTX) : 1;
    logic                ctx_valid_q [NCTX], ctx_valid_n [NCTX];
    logic [3:0]          ctx_id_q    [NCTX], ctx_id_n    [NCTX];
    logic [SEL_BITS-1:0] ctx_owner_q [NCTX], ctx_owner_n [NCTX];

    // lowest free slot
    logic              ctx_free;
    logic [NCTX_W-1:0] ctx_alloc;
    always_comb begin
        ctx_free = 1'b0; ctx_alloc = '0;
        for (int k = NCTX-1; k >= 0; k -= 1)
            if (!ctx_valid_q[k]) begin ctx_free = 1'b1; ctx_alloc = NCTX_W'(k); end
    end

    // Forward when a context slot is free (the downstream d_ready is the physical 1-at-a-time
    // gate until P3d-3). fwd_fire = the op is actually accepted downstream this cycle.
    logic fwd_fire;
    always_comb begin
        d_req = '0;
        for (int i = 0; i < N_MASTERS; i += 1) m_ready[i] = 1'b0;
        fwd_fire = 1'b0;
        if (ctx_free && grant_valid) begin
            d_req              = m_req[grant_sel];
            m_ready[grant_sel] = d_ready;
            fwd_fire           = d_ready;
        end
    end

    // Route the response to the master whose id it carries.
    always_comb begin
        for (int i = 0; i < N_MASTERS; i += 1) m_resp[i] = '0;
        if (d_resp.valid) begin
            for (int k = 0; k < NCTX; k += 1)
                if (ctx_valid_q[k] && (ctx_id_q[k] == d_resp.id))
                    m_resp[ctx_owner_q[k]] = d_resp;
        end
    end

    // Next-state: free the id-matched slot on a response, allocate the lowest-free slot on a
    // forward. Free reads the registered valid, so a same-cycle free+alloc name distinct slots.
    always_comb begin
        for (int k = 0; k < NCTX; k += 1) begin
            ctx_valid_n[k] = ctx_valid_q[k];
            ctx_id_n[k]    = ctx_id_q[k];
            ctx_owner_n[k] = ctx_owner_q[k];
        end
        if (d_resp.valid)
            for (int k = 0; k < NCTX; k += 1)
                if (ctx_valid_q[k] && (ctx_id_q[k] == d_resp.id)) ctx_valid_n[k] = 1'b0;
        if (fwd_fire) begin
            ctx_valid_n[ctx_alloc] = 1'b1;
            ctx_id_n[ctx_alloc]    = m_req[grant_sel].id;
            ctx_owner_n[ctx_alloc] = grant_sel;
        end
    end

    always_ff @(posedge clk or negedge rst_l) begin
        if (!rst_l)
            for (int k = 0; k < NCTX; k += 1) begin
                ctx_valid_q[k] <= 1'b0; ctx_id_q[k] <= '0; ctx_owner_q[k] <= '0;
            end
        else
            for (int k = 0; k < NCTX; k += 1) begin
                ctx_valid_q[k] <= ctx_valid_n[k];
                ctx_id_q[k]    <= ctx_id_n[k];
                ctx_owner_q[k] <= ctx_owner_n[k];
            end
    end

    // A response must always name exactly one outstanding context (no orphan / no aliased id).
    always_ff @(posedge clk) begin
        if (rst_l && d_resp.valid) begin
            automatic int unsigned m = 0;
            for (int k = 0; k < NCTX; k += 1) if (ctx_valid_q[k] && (ctx_id_q[k] == d_resp.id)) m += 1;
            assert (m == 1) else $fatal(1, "nmi_arbiter: response id=%h matched %0d contexts", d_resp.id, m);
        end
    end
`else
    logic                busy_q, busy_n;
    logic [SEL_BITS-1:0] sel_q,  sel_n;

    // Combinational fixed-priority selection among requesting masters.
    logic                grant_valid;
    logic [SEL_BITS-1:0] grant_sel;
    always_comb begin
        grant_valid = 1'b0;
        grant_sel   = '0;
        for (int i = N_MASTERS-1; i >= 0; i -= 1) begin
            if (m_req[i].valid) begin
                grant_valid = 1'b1;
                grant_sel   = SEL_BITS'(i);
            end
        end
    end

    // Forward only when idle.
    always_comb begin
        d_req = '0;
        for (int i = 0; i < N_MASTERS; i += 1) m_ready[i] = 1'b0;
        if (!busy_q && grant_valid) begin
            d_req            = m_req[grant_sel];
            m_ready[grant_sel] = d_ready;
        end
    end

    // Route the response to the owning master.
    always_comb begin
        for (int i = 0; i < N_MASTERS; i += 1) m_resp[i] = '0;
        if (busy_q) m_resp[sel_q] = d_resp;
    end

    always_comb begin
        busy_n = busy_q;
        sel_n  = sel_q;
        if (!busy_q) begin
            if (grant_valid && d_ready) begin
                busy_n = 1'b1;
                sel_n  = grant_sel;
            end
        end else if (d_resp.valid) begin
            busy_n = 1'b0;
        end
    end

    always_ff @(posedge clk or negedge rst_l) begin
        if (!rst_l) begin
            busy_q <= 1'b0;
            sel_q  <= '0;
        end else begin
            busy_q <= busy_n;
            sel_q  <= sel_n;
        end
    end
`endif

endmodule : nmi_arbiter

`default_nettype wire
