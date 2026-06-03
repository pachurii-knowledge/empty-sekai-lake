/**
 * pmp_checker.sv
 *
 * Physical Memory Protection check. Combinationally evaluates the 16 PMP
 * entries (pmpcfg0..3 / pmpaddr0..15) against a physical address and access
 * type, honouring TOR / NA4 / NAPOT matching and the R/W/X/L bits. The lowest
 * matching entry wins. When no entry matches: M-mode is allowed; S/U modes are
 * denied only if at least one entry is enabled (otherwise PMP is considered
 * unconfigured and all accesses are permitted).
 */

`include "riscv_priv.vh"

`default_nettype none

module pmp_checker
    import RISCV_Priv::*;
(
    input  logic [31:0]       paddr,            // byte physical address
    input  logic [1:0]        access,           // 0 = fetch, 1 = load, 2 = store
    input  priv_mode_t        priv,
    input  logic [31:0]       pmpcfg [4],
    input  logic [31:0]       pmpaddr [16],
    output logic              fault
);

    localparam logic [1:0] ACC_FETCH = 2'd0;
    localparam logic [1:0] ACC_LOAD  = 2'd1;
    localparam logic [1:0] ACC_STORE = 2'd2;

    localparam logic [1:0] A_OFF   = 2'd0;
    localparam logic [1:0] A_TOR   = 2'd1;
    localparam logic [1:0] A_NA4   = 2'd2;
    localparam logic [1:0] A_NAPOT = 2'd3;

    logic [29:0] addr_word;
    assign addr_word = paddr[31:2];

    function automatic logic [7:0] cfg_byte(input int idx);
        cfg_byte = pmpcfg[idx / 4][(idx % 4) * 8 +: 8];
    endfunction

    // NAPOT match: a has k trailing ones encoding a 2^(k+3) byte region.
    function automatic logic napot_match(input logic [29:0] a,
            input logic [29:0] addr);
        logic [30:0] xor1;
        logic [29:0] ignore;
        xor1   = {1'b0, a} ^ ({1'b0, a} + 31'd1);  // (k+1) trailing ones
        ignore = xor1[30:1];                        // low k bits to ignore
        napot_match = ((addr ^ a) & ~ignore) == 30'b0;
    endfunction

    logic        any_enabled;
    logic        matched;
    logic        entry_fault;

    always_comb begin
        any_enabled = 1'b0;
        matched     = 1'b0;
        entry_fault = 1'b0;
        fault       = 1'b0;

        for (int i = 0; i < 16; i += 1) begin
            logic [7:0] cfg;
            logic [1:0] a_mode;
            logic       hit_i;
            logic [29:0] lo, hi;
            cfg    = cfg_byte(i);
            a_mode = cfg[4:3];
            hit_i  = 1'b0;
            if (a_mode != A_OFF) any_enabled = 1'b1;

            unique case (a_mode)
                A_TOR: begin
                    lo = (i == 0) ? 30'b0 : pmpaddr[i-1][29:0];
                    hi = pmpaddr[i][29:0];
                    hit_i = (addr_word >= lo) && (addr_word < hi);
                end
                A_NA4:   hit_i = (addr_word == pmpaddr[i][29:0]);
                A_NAPOT: hit_i = napot_match(pmpaddr[i][29:0], addr_word);
                default: hit_i = 1'b0;
            endcase

            if (hit_i && !matched) begin
                logic ok;
                logic locked;
                matched = 1'b1;
                locked  = cfg[7];
                // Locked entries also apply to M-mode; unlocked entries do not
                // restrict M-mode.
                if ((priv == PRIV_M) && !locked) begin
                    ok = 1'b1;
                end else begin
                    unique case (access)
                        ACC_FETCH: ok = cfg[2];      // X
                        ACC_LOAD:  ok = cfg[0];      // R
                        ACC_STORE: ok = cfg[1];      // W
                        default:   ok = 1'b0;
                    endcase
                end
                entry_fault = !ok;
            end
        end

        if (matched) begin
            fault = entry_fault;
        end else begin
            // No entry matched.
            fault = (priv != PRIV_M) && any_enabled;
        end
    end

endmodule: pmp_checker
