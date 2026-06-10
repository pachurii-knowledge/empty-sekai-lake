/**
 * ptw.sv
 *
 * Hardware page-table walker: Sv32 (2-level, 4-byte PTEs) at RV32, Sv39
 * (3-level, 8-byte PTEs) at RV64 — selected by RISCV_Priv::VM_* geometry.
 * Performs the multi-level walk for a virtual page, checks the leaf PTE
 * permissions against the access type and privilege (honouring mstatus.SUM /
 * mstatus.MXR), and performs hardware A/D bit updates (writing the PTE back)
 * when required. Reports the leaf PPN, permission bits, the level the leaf was
 * found at (superpages), and a page-fault signal.
 *
 * The walker drives a simple single-word memory port (byte-addressed; one
 * memory word holds exactly one PTE at either XLEN) that the integrating core
 * arbitrates against its normal data accesses; mem_ack must be asserted the
 * cycle read/write data is valid.
 */

`include "riscv_priv.vh"

`default_nettype none

module ptw
    import RISCV_Priv::*;
(
    input  logic        clk,
    input  logic        rst_l,

    // Translation request
    input  logic        req_valid,
    input  logic [VM_VPN_W-1:0] req_vpn,    // vaddr[38:12] / vaddr[31:12]
    input  logic [VM_PPN_W-1:0] satp_ppn,   // satp.PPN
    input  logic [1:0]  req_access,     // 0 = fetch, 1 = load, 2 = store
    input  priv_mode_t  req_priv,       // effective privilege of the access
    input  logic        mstatus_sum,
    input  logic        mstatus_mxr,
    input  logic        adue,           // menvcfg.ADUE: 1=HW A/D update (Svadu),
                                        // 0=fault when an A/D update is needed (Svade)
    // Sv39: the full VA's bits [63:39] do not all equal bit 38 -- the access
    // page-faults without walking. Tied 0 at RV32 (every VA is canonical).
    input  logic        req_noncanonical,

    // Memory port (byte addressed)
    output logic        mem_req,
    output logic        mem_we,
    // The in-flight PTE access is a write (A/D update) -- unconditional intent,
    // unlike mem_we which is gated off when the PMP check denies the write. The
    // core uses this (not mem_we) to pick the PMP access type, so gating mem_we
    // cannot feed back and flip the access type / oscillate the PMP result.
    output logic        mem_is_write,
    output logic [MXLEN-1:0] mem_addr,
    output logic [MXLEN-1:0] mem_wdata,
    input  logic        mem_ack,
    input  logic [MXLEN-1:0] mem_rdata,
    // PMP violation on the PTE access at mem_addr (driven combinationally by the
    // integrating core against the address this walk is currently requesting).
    // Per the priv spec a PMP fault on a PTE access aborts the walk and raises an
    // access fault of the *original* access type, not a page fault.
    input  logic        pte_pmp_fault,

    // Result (asserted for one cycle with done)
    output logic        busy,
    output logic        done,
    output logic        fault,
    output logic        fault_access,   // fault is a PTE-access (PMP) fault, not a page fault
    output logic [VM_PPN_W-1:0] ppn,
    output logic [7:0]  perm,
    output logic [1:0]  leaf_level,     // 0 = base page, >0 = superpage level
    // The VPN and access class this walk was launched for, latched at request
    // time. A TLB must be filled against these (not the integrating core's live
    // head address), since in an out-of-order core the requesting access may
    // have advanced by the time the walk completes -- using the live address
    // would tag the fill with the wrong VPN.
    output logic [VM_VPN_W-1:0] walk_vpn,
    output logic        walk_is_data,
    // The privilege and satp.PPN this walk was launched under, also latched at
    // request time. The integrating core must gate *consuming* a completed walk
    // (resolved PA or fault) on these matching the live access context -- a walk
    // launched in another mode/address space (e.g. a speculative S-mode fetch of
    // a user VA during a trap window, walking the kernel page table) faults, and
    // that fault must not be mistaken for the current access's translation.
    output priv_mode_t  walk_priv,
    output logic [VM_PPN_W-1:0] walk_satp
);

    localparam logic [1:0] ACC_FETCH = 2'd0;
    localparam logic [1:0] ACC_LOAD  = 2'd1;
    localparam logic [1:0] ACC_STORE = 2'd2;

    typedef enum logic [2:0] {
        S_IDLE,
        S_REQ, S_WAIT,
        S_AD_REQ, S_AD_WAIT,
        S_DONE
    } state_t;

    state_t      state_q, state_n;
    logic [VM_VPN_W-1:0] vpn_q;
    logic [1:0]  acc_q;
    priv_mode_t  priv_q;
    logic        sum_q, mxr_q;
    logic [MXLEN-1:0] pte_q;     // current PTE
    logic [MXLEN-1:0] pte_addr_q;// address of current PTE (for A/D writeback)
    logic [1:0]  level_q, level_n;       // current walk level (LEVELS-1 .. 0)
    logic [VM_PPN_W-1:0] base_q, base_n; // current table base PPN
    logic [VM_PPN_W-1:0] walk_satp_q;    // satp.PPN this walk was launched under
    logic        walk_bad_q, walk_bad_n; // invalid / pointer-depth / misaligned /
                                         // non-canonical: page fault at done
    logic        noncanon_q;
    logic [1:0]  fault_level_q;  // level the walk completed at (leaf level)
    logic        pte_af_q;       // sticky: a PTE access (PMP) fault aborted the walk

    // A PTE memory access (read or A/D write) is in flight this cycle.
    logic        pte_access_now;
    assign pte_access_now = (state_q == S_REQ) || (state_q == S_AD_REQ);

    // VPN slice select for the current level
    function automatic logic [VM_VPN_SLICE-1:0] vpn_slice(input logic [1:0] lvl);
        vpn_slice = vpn_q[lvl * VM_VPN_SLICE +: VM_VPN_SLICE];
    endfunction

    // PTE address for the current level: table base + slice * PTESIZE. The
    // full PA is VM_PPN_W+12 bits (34 at Sv32, 56 at Sv39); it is truncated
    // to the MXLEN-wide memory port (the Sv32 port has always been 32-bit).
    localparam int VM_PA_W = VM_PPN_W + 12;
    logic [VM_PA_W-1:0] cur_pte_pa;
    logic [MXLEN-1:0] cur_pte_addr;
    assign cur_pte_pa = {base_q, 12'b0} +
        (VM_PA_W'(vpn_slice(level_q)) << VM_PTESHIFT);
    assign cur_pte_addr = MXLEN'(cur_pte_pa);

    // Latched request context
    always_ff @(posedge clk or negedge rst_l) begin
        if (!rst_l) begin
            state_q <= S_IDLE;
            vpn_q <= '0; acc_q <= 2'b0; priv_q <= PRIV_M;
            sum_q <= 1'b0; mxr_q <= 1'b0;
            pte_q <= '0; pte_addr_q <= '0;
            level_q <= '0; base_q <= '0; walk_satp_q <= '0;
            walk_bad_q <= 1'b0; noncanon_q <= 1'b0;
            fault_level_q <= '0;
            pte_af_q <= 1'b0;
        end else begin
            state_q <= state_n;
            level_q <= level_n;
            base_q  <= base_n;
            walk_bad_q <= walk_bad_n;
            if (state_q == S_IDLE && req_valid) begin
                vpn_q  <= req_vpn;
                acc_q  <= req_access;
                priv_q <= req_priv;
                sum_q  <= mstatus_sum;
                mxr_q  <= mstatus_mxr;
                noncanon_q <= req_noncanonical;
                walk_satp_q <= satp_ppn;
            end
            // Capture PTE reads when the memory acknowledges the request
            if ((state_q == S_REQ) && mem_ack) begin
                pte_q <= mem_rdata;
                pte_addr_q <= cur_pte_addr;
            end
            if (state_q == S_WAIT) begin
                fault_level_q <= level_q;
            end
            // Latch a PTE-access PMP fault; clear it as a fresh walk is launched.
            if (state_q == S_IDLE && req_valid)
                pte_af_q <= 1'b0;
            else if (pte_access_now && pte_pmp_fault)
                pte_af_q <= 1'b1;
        end
    end

    // PTE field helpers on the just-read word (combinational view)
    logic        v_bit, r_bit, w_bit, x_bit, u_bit, a_bit, d_bit;
    logic [VM_PPN_W-1:0] pte_ppn;
    assign v_bit   = pte_q[RISCV_Priv::PTE_V];
    assign r_bit   = pte_q[RISCV_Priv::PTE_R];
    assign w_bit   = pte_q[RISCV_Priv::PTE_W];
    assign x_bit   = pte_q[RISCV_Priv::PTE_X];
    assign u_bit   = pte_q[RISCV_Priv::PTE_U];
    assign a_bit   = pte_q[RISCV_Priv::PTE_A];
    assign d_bit   = pte_q[RISCV_Priv::PTE_D];
    assign pte_ppn = pte_q[10 +: VM_PPN_W];

    // Sv39 PTEs reserve bits 63:54 (N / PBMT / reserved): any set bit page-
    // faults on an implementation without Svnapot/Svpbmt. Never true at RV32.
    logic pte_reserved_bad;
    assign pte_reserved_bad = (MXLEN == 64) && (pte_q[MXLEN-1:MXLEN-10] != '0);

    // A leaf above level 0 must have its low PPN slices clear (aligned
    // superpage); otherwise the walk page-faults.
    function automatic logic super_misaligned(input logic [1:0] lvl);
        logic bad;
        bad = 1'b0;
        for (int b = 0; b < 2 * VM_VPN_SLICE; b += 1) begin
            if ((b < 32'(lvl) * VM_VPN_SLICE) && pte_ppn[b]) bad = 1'b1;
        end
        super_misaligned = bad;
    endfunction

    // Permission check for the leaf PTE
    function automatic logic perm_fault();
        logic fail;
        fail = 1'b0;
        // Access-type permission
        unique case (acc_q)
            ACC_FETCH: if (!x_bit) fail = 1'b1;
            ACC_LOAD:  if (!(r_bit || (x_bit && mxr_q))) fail = 1'b1;
            ACC_STORE: if (!w_bit) fail = 1'b1;
            default: ;
        endcase
        // U/S checks
        if (priv_q == PRIV_U) begin
            if (!u_bit) fail = 1'b1;
        end else if (priv_q == PRIV_S) begin
            if (u_bit) begin
                // S may read/write U pages only if SUM; never execute them.
                if (acc_q == ACC_FETCH) fail = 1'b1;
                else if (!sum_q)        fail = 1'b1;
            end
        end
        perm_fault = fail;
    endfunction

    logic leaf, need_ad;
    assign leaf    = r_bit || x_bit;
    // A/D update required (Svadu hardware update)
    assign need_ad = (!a_bit) || ((acc_q == ACC_STORE) && (!d_bit));

    // Next-state / datapath
    always_comb begin
        state_n   = state_q;
        level_n   = level_q;
        base_n    = base_q;
        walk_bad_n = walk_bad_q;
        mem_req   = 1'b0;
        mem_we    = 1'b0;
        mem_addr  = '0;
        mem_wdata = '0;

        unique case (state_q)
            S_IDLE: begin
                if (req_valid) begin
                    level_n = 2'(VM_LEVELS - 1);
                    base_n  = satp_ppn;
                    walk_bad_n = 1'b0;
                    // A non-canonical Sv39 VA page-faults without walking.
                    if (req_noncanonical) begin
                        walk_bad_n = 1'b1;
                        state_n = S_DONE;
                    end else begin
                        state_n = S_REQ;
                    end
                end
            end
            S_REQ: begin
                mem_req  = 1'b1;
                mem_addr = cur_pte_addr;
                if (pte_pmp_fault) state_n = S_DONE;   // PMP-denied PTE: abort walk
                else if (mem_ack)  state_n = S_WAIT;
            end
            S_WAIT: begin
                // pte_q now holds the PTE read at level_q
                if (!v_bit || (!r_bit && w_bit) || pte_reserved_bad) begin
                    walk_bad_n = 1'b1;                 // invalid encoding
                    state_n = S_DONE;
                end else if (leaf) begin
                    if (super_misaligned(level_q)) begin
                        walk_bad_n = 1'b1;             // misaligned superpage
                        state_n = S_DONE;
                    end else begin
                        state_n = (need_ad && adue && !perm_fault()) ?
                            S_AD_REQ : S_DONE;
                    end
                end else if (level_q == 2'd0) begin
                    walk_bad_n = 1'b1;                 // pointer at the last level
                    state_n = S_DONE;
                end else begin
                    level_n = level_q - 2'd1;          // descend
                    base_n  = pte_ppn;
                    state_n = S_REQ;
                end
            end
            S_AD_REQ: begin
                // Suppress the write itself when PMP denies it -- the A/D bits
                // must NOT be updated on a faulting A/D write (the access faults
                // instead). Asserting mem_we here regardless would wrongly commit
                // the update before the abort.
                mem_req   = !pte_pmp_fault;
                mem_we    = !pte_pmp_fault;
                mem_addr  = pte_addr_q;
                mem_wdata = pte_q | (MXLEN'(1) << RISCV_Priv::PTE_A) |
                    ((acc_q == ACC_STORE) ?
                        (MXLEN'(1) << RISCV_Priv::PTE_D) : '0);
                if (pte_pmp_fault) state_n = S_DONE;   // PMP-denied A/D write: abort
                else if (mem_ack)  state_n = S_AD_WAIT;
            end
            S_AD_WAIT: state_n = S_DONE;
            S_DONE:    state_n = S_IDLE;
            default:   state_n = S_IDLE;
        endcase
    end

    // Svade A/D fault: a permitted leaf that needs an A/D update but ADUE is
    // off raises a page fault instead of a hardware update.
    logic ad_fault;
    assign ad_fault = need_ad && !adue;

    assign busy         = (state_q != S_IDLE);
    assign mem_is_write = (state_q == S_AD_REQ);
    assign done         = (state_q == S_DONE);
    assign leaf_level   = fault_level_q;
    assign ppn          = pte_ppn;
    assign walk_vpn     = vpn_q;
    assign walk_is_data = (acc_q != ACC_FETCH);
    assign walk_priv    = priv_q;
    assign walk_satp    = walk_satp_q;
    // Reflect the A/D bits the hardware update set (Svadu) in the reported perm,
    // so the TLB is filled with the post-update PTE. Otherwise a store would see
    // a stale D=0 in the DTLB after its A/D walk, fail dtlb_usable on the commit
    // cycle, and miss its one-shot store-commit window (orphaning the access).
    assign perm      = pte_q[7:0] |
        ((need_ad && adue) ? ((8'b1 << RISCV_Priv::PTE_A) |
            ((acc_q == ACC_STORE) ? (8'b1 << RISCV_Priv::PTE_D) : 8'b0)) : 8'b0);

    // fault asserted with done: a PTE-access PMP fault (pte_af_q) aborts the walk
    // and is reported as an access fault; a structurally bad walk (invalid PTE,
    // pointer at the last level, misaligned superpage, non-canonical VA) or a
    // failing leaf permission / Svade A/D check is a page fault.
    assign fault        = done && (pte_af_q || walk_bad_q ||
                                   perm_fault() || ad_fault);
    assign fault_access = done && pte_af_q;

endmodule: ptw
