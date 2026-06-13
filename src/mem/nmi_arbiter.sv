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

endmodule : nmi_arbiter

`default_nettype wire
