/**
 * mmu_tlb.sv
 *
 * A small fully-associative TLB: Sv32 at RV32, Sv39 at RV64 (geometry from
 * RISCV_Priv::VM_*). Instantiated separately for the instruction side (ITLB)
 * and data side (DTLB). Each entry caches a virtual-page -> physical-page
 * translation together with its leaf PTE permission bits and the level at
 * which the leaf was found (level > 0 = superpage: 4 MiB at Sv32; 2 MiB at
 * level 1 / 1 GiB at level 2 for Sv39).
 *
 * Lookup is combinational. Fills and flushes are synchronous. A SFENCE.VMA is
 * modeled as a full flush (address/ASID-selective flush is treated as a full
 * flush, which is always a legal over-approximation).
 */

`include "riscv_priv.vh"

`default_nettype none

module mmu_tlb
    import RISCV_Priv::*;
#(
    parameter int ENTRIES = 16
) (
    input wire logic        clk,
    input wire logic        rst_l,

    // Lookup (combinational)
    input wire logic        lookup_en,
    input wire logic [VM_VPN_W-1:0] lookup_vpn,
    input wire logic [VM_ASID_W-1:0] lookup_asid,
    output logic        hit,
    output logic [VM_PPN_W-1:0] hit_ppn,
    output logic [7:0]  hit_perm,      // {D,A,G,U,X,W,R,V}
    output logic [1:0]  hit_level,     // leaf level (0 = base page)

    // Fill (synchronous)
    input wire logic        fill_en,
    input wire logic [VM_VPN_W-1:0] fill_vpn,
    input wire logic [VM_ASID_W-1:0] fill_asid,
    input wire logic [VM_PPN_W-1:0] fill_ppn,
    input wire logic [7:0]  fill_perm,
    input wire logic [1:0]  fill_level,

    // Flush (synchronous) - full flush
    input wire logic        flush_en
);

    logic              valid_q [ENTRIES];
    logic [VM_VPN_W-1:0] vpn_q [ENTRIES];
    logic [VM_ASID_W-1:0] asid_q [ENTRIES];
    logic [VM_PPN_W-1:0] ppn_q [ENTRIES];
    logic [7:0]        perm_q  [ENTRIES];
    logic [1:0]        level_q [ENTRIES];
    logic [$clog2(ENTRIES)-1:0] repl_q;     // round-robin replacement

    // A leaf at level L matches on the VPN bits above L slices.
    function automatic logic vpn_match(input logic [VM_VPN_W-1:0] a,
            input logic [VM_VPN_W-1:0] b, input logic [1:0] lvl);
        logic ok;
        ok = 1'b1;
        for (int i = 0; i < VM_VPN_W; i += 1) begin
            if ((i >= 32'(lvl) * VM_VPN_SLICE) && (a[i] != b[i])) ok = 1'b0;
        end
        vpn_match = ok;
    endfunction

    // Combinational lookup. A superpage entry matches on the high VPN bits only.
    logic [ENTRIES-1:0] match;
    always_comb begin
        for (int i = 0; i < ENTRIES; i += 1) begin
            logic global_e, asid_ok, vpn_ok;
            global_e = perm_q[i][RISCV_Priv::PTE_G];
            asid_ok  = global_e || (asid_q[i] == lookup_asid);
            vpn_ok   = vpn_match(vpn_q[i], lookup_vpn, level_q[i]);
            match[i] = valid_q[i] && asid_ok && vpn_ok;
        end
    end

    always_comb begin
        hit       = 1'b0;
        hit_ppn   = '0;
        hit_perm  = 8'b0;
        hit_level = 2'b0;
        if (lookup_en) begin
            for (int i = 0; i < ENTRIES; i += 1) begin
                if (match[i]) begin
                    hit       = 1'b1;
                    hit_ppn   = ppn_q[i];
                    hit_perm  = perm_q[i];
                    hit_level = level_q[i];
                end
            end
        end
    end

    always_ff @(posedge clk or negedge rst_l) begin
        if (!rst_l) begin
            for (int i = 0; i < ENTRIES; i += 1) valid_q[i] <= 1'b0;
            repl_q <= '0;
        end else if (flush_en) begin
            for (int i = 0; i < ENTRIES; i += 1) valid_q[i] <= 1'b0;
        end else if (fill_en) begin
            valid_q[repl_q] <= 1'b1;
            vpn_q  [repl_q] <= fill_vpn;
            asid_q [repl_q] <= fill_asid;
            ppn_q  [repl_q] <= fill_ppn;
            perm_q [repl_q] <= fill_perm;
            level_q[repl_q] <= fill_level;
            repl_q <= repl_q + 1'b1;
        end
    end

endmodule: mmu_tlb
