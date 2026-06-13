/**
 * trap_controller.sv
 *
 * Combinational trap-decision logic. Given the current privilege mode, the
 * relevant CSR state (mstatus/mie/mip/medeleg/mideleg/mtvec/stvec) and a
 * synchronous exception request from the committing instruction, it decides
 * whether a trap is taken this cycle, the cause, the target privilege (after
 * delegation), and the redirect (trap vector) PC. Interrupts are evaluated
 * with priority over a synchronous exception (the interrupted instruction is
 * squashed and re-fetched after the handler returns).
 *
 * The companion priv_csr_file applies the architectural state transition.
 */

`include "riscv_priv.vh"

`default_nettype none

module trap_controller
    import RISCV_Priv::*;
(
    input wire priv_mode_t  priv,
    input wire logic [MXLEN-1:0] mstatus,
    input wire logic [MXLEN-1:0] mie_csr,
    input wire logic [MXLEN-1:0] mip_csr,
    input wire logic [MXLEN-1:0] medeleg,
    input wire logic [MXLEN-1:0] mideleg,
    input wire logic [MXLEN-1:0] mtvec,
    input wire logic [MXLEN-1:0] stvec,

    // Synchronous exception request from the committing instruction
    input wire logic        exc_valid,
    input wire logic [4:0]  exc_cause,

    output logic        trap_valid,
    output logic        trap_is_interrupt,
    output logic [4:0]  trap_cause,
    output priv_mode_t  trap_target_priv,
    output logic [MXLEN-1:0] trap_vector
);

    localparam int SSI_BIT = 1;
    localparam int MSI_BIT = 3;
    localparam int STI_BIT = 5;
    localparam int MTI_BIT = 7;
    localparam int SEI_BIT = 9;
    localparam int MEI_BIT = 11;

    logic mie_global, sie_global;
    logic m_enabled, s_enabled;
    logic [MXLEN-1:0] pending;
    logic [MXLEN-1:0] m_ints, s_ints;
    logic        take_m_int, take_s_int;
    logic [4:0]  m_cause, s_cause;

    assign mie_global = mstatus[MSTATUS_MIE_BIT];
    assign sie_global = mstatus[MSTATUS_SIE_BIT];

    // An interrupt destined for M is globally enabled when running below M, or
    // in M with MIE set. Likewise for S.
    assign m_enabled = (priv != PRIV_M) || mie_global;
    assign s_enabled = (priv == PRIV_U) || ((priv == PRIV_S) && sie_global);

    assign pending = mip_csr & mie_csr;
    assign m_ints  = pending & ~mideleg;   // handled in M
    assign s_ints  = pending & mideleg;    // delegated to S

    // Standard interrupt priority: MEI > MSI > MTI > SEI > SSI > STI.
    function automatic logic [4:0] pick_int(input logic [MXLEN-1:0] bits);
        if (bits[MEI_BIT])      pick_int = INT_M_EXTERNAL;
        else if (bits[MSI_BIT]) pick_int = INT_M_SOFTWARE;
        else if (bits[MTI_BIT]) pick_int = INT_M_TIMER;
        else if (bits[SEI_BIT]) pick_int = INT_S_EXTERNAL;
        else if (bits[SSI_BIT]) pick_int = INT_S_SOFTWARE;
        else if (bits[STI_BIT]) pick_int = INT_S_TIMER;
        else                    pick_int = 5'd0;
    endfunction

    always_comb begin
        take_m_int = m_enabled && (m_ints != '0);
        take_s_int = s_enabled && (s_ints != '0);
        m_cause = pick_int(m_ints);
        s_cause = pick_int(s_ints);

        trap_valid        = 1'b0;
        trap_is_interrupt = 1'b0;
        trap_cause        = 5'd0;
        trap_target_priv  = PRIV_M;

        if (take_m_int) begin
            trap_valid        = 1'b1;
            trap_is_interrupt = 1'b1;
            trap_cause        = m_cause;
            trap_target_priv  = PRIV_M;
        end else if (take_s_int) begin
            trap_valid        = 1'b1;
            trap_is_interrupt = 1'b1;
            trap_cause        = s_cause;
            trap_target_priv  = PRIV_S;
        end else if (exc_valid) begin
            trap_valid        = 1'b1;
            trap_is_interrupt = 1'b0;
            trap_cause        = exc_cause;
            // Synchronous exceptions delegate to S only when taken below M and
            // the corresponding medeleg bit is set.
            trap_target_priv  = ((priv != PRIV_M) && medeleg[exc_cause]) ?
                PRIV_S : PRIV_M;
        end
    end

    // Trap vector computation (Direct vs Vectored).
    logic [MXLEN-1:0] base;
    logic [1:0]  mode;
    always_comb begin
        if (trap_target_priv == PRIV_M) begin
            base = {mtvec[MXLEN-1:2], 2'b00};
            mode = mtvec[1:0];
        end else begin
            base = {stvec[MXLEN-1:2], 2'b00};
            mode = stvec[1:0];
        end
        if ((mode == 2'b01) && trap_is_interrupt) begin
            trap_vector = base + MXLEN'({trap_cause, 2'b00});
        end else begin
            trap_vector = base;
        end
    end

endmodule: trap_controller
