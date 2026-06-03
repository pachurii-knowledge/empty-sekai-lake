/**
 * mmu_tlb.sv
 *
 * A small fully-associative Sv32 translation-lookaside buffer. Instantiated
 * separately for the instruction side (ITLB) and data side (DTLB). Each entry
 * caches a virtual-page -> physical-page translation together with its leaf PTE
 * permission bits and the level at which the leaf was found (level 1 = 4 MiB
 * superpage, level 0 = 4 KiB page).
 *
 * Lookup is combinational. Fills and flushes are synchronous. A SFENCE.VMA is
 * modeled as a full flush (address/ASID-selective flush is treated as a full
 * flush, which is always a legal over-approximation).
 */

`include "riscv_priv.vh"

`default_nettype none

module mmu_tlb #(
    parameter int ENTRIES = 16
) (
    input  logic        clk,
    input  logic        rst_l,

    // Lookup (combinational)
    input  logic        lookup_en,
    input  logic [19:0] lookup_vpn,    // full 20-bit VPN (vaddr[31:12])
    input  logic [8:0]  lookup_asid,
    output logic        hit,
    output logic [21:0] hit_ppn,       // full 22-bit PPN (Sv32)
    output logic [7:0]  hit_perm,      // {D,A,G,U,X,W,R,V}
    output logic        hit_superpage, // leaf found at level 1

    // Fill (synchronous)
    input  logic        fill_en,
    input  logic [19:0] fill_vpn,
    input  logic [8:0]  fill_asid,
    input  logic [21:0] fill_ppn,
    input  logic [7:0]  fill_perm,
    input  logic        fill_superpage,

    // Flush (synchronous) - full flush
    input  logic        flush_en
);

    logic              valid_q [ENTRIES];
    logic [19:0]       vpn_q   [ENTRIES];
    logic [8:0]        asid_q  [ENTRIES];
    logic [21:0]       ppn_q   [ENTRIES];
    logic [7:0]        perm_q  [ENTRIES];
    logic              super_q [ENTRIES];
    logic [$clog2(ENTRIES)-1:0] repl_q;     // round-robin replacement

    // Combinational lookup. A superpage entry matches on the high VPN bits only.
    logic [ENTRIES-1:0] match;
    always_comb begin
        for (int i = 0; i < ENTRIES; i += 1) begin
            logic global_e, asid_ok, vpn_ok;
            global_e = perm_q[i][RISCV_Priv::PTE_G];
            asid_ok  = global_e || (asid_q[i] == lookup_asid);
            if (super_q[i])
                vpn_ok = (vpn_q[i][19:10] == lookup_vpn[19:10]);
            else
                vpn_ok = (vpn_q[i] == lookup_vpn);
            match[i] = valid_q[i] && asid_ok && vpn_ok;
        end
    end

    always_comb begin
        hit           = 1'b0;
        hit_ppn       = 22'b0;
        hit_perm      = 8'b0;
        hit_superpage = 1'b0;
        if (lookup_en) begin
            for (int i = 0; i < ENTRIES; i += 1) begin
                if (match[i]) begin
                    hit           = 1'b1;
                    hit_ppn       = ppn_q[i];
                    hit_perm      = perm_q[i];
                    hit_superpage = super_q[i];
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
            super_q[repl_q] <= fill_superpage;
            repl_q <= repl_q + 1'b1;
        end
    end

endmodule: mmu_tlb
