/**
 * ptw.sv
 *
 * Sv32 hardware page-table walker. Performs the two-level walk for a virtual
 * page, checks the leaf PTE permissions against the access type and privilege
 * (honouring mstatus.SUM / mstatus.MXR), and performs hardware A/D bit updates
 * (writing the PTE back) when required. Reports the leaf PPN, permission bits,
 * superpage flag, and a page-fault signal.
 *
 * The walker drives a simple single-word memory port (byte-addressed) that the
 * integrating core arbitrates against its normal data accesses; mem_ack must be
 * asserted the cycle read/write data is valid.
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
    input  logic [19:0] req_vpn,        // vaddr[31:12]
    input  logic [21:0] satp_ppn,       // satp.PPN
    input  logic [1:0]  req_access,     // 0 = fetch, 1 = load, 2 = store
    input  priv_mode_t  req_priv,       // effective privilege of the access
    input  logic        mstatus_sum,
    input  logic        mstatus_mxr,
    input  logic        adue,           // menvcfg.ADUE: 1=HW A/D update (Svadu),
                                        // 0=fault when an A/D update is needed (Svade)

    // Memory port (byte addressed)
    output logic        mem_req,
    output logic        mem_we,
    output logic [31:0] mem_addr,
    output logic [31:0] mem_wdata,
    input  logic        mem_ack,
    input  logic [31:0] mem_rdata,

    // Result (asserted for one cycle with done)
    output logic        busy,
    output logic        done,
    output logic        fault,
    output logic [21:0] ppn,
    output logic [7:0]  perm,
    output logic        superpage,
    // The VPN and access class this walk was launched for, latched at request
    // time. A TLB must be filled against these (not the integrating core's live
    // head address), since in an out-of-order core the requesting access may
    // have advanced by the time the walk completes -- using the live address
    // would tag the fill with the wrong VPN.
    output logic [19:0] walk_vpn,
    output logic        walk_is_data
);

    localparam logic [1:0] ACC_FETCH = 2'd0;
    localparam logic [1:0] ACC_LOAD  = 2'd1;
    localparam logic [1:0] ACC_STORE = 2'd2;

    typedef enum logic [2:0] {
        S_IDLE,
        S_L1_REQ, S_L1_WAIT,
        S_L0_REQ, S_L0_WAIT,
        S_AD_REQ, S_AD_WAIT,
        S_DONE
    } state_t;

    state_t      state_q, state_n;
    logic [19:0] vpn_q;
    logic [1:0]  acc_q;
    priv_mode_t  priv_q;
    logic        sum_q, mxr_q;
    logic [31:0] pte_q;          // current PTE
    logic [31:0] pte_addr_q;     // address of current PTE (for A/D writeback)
    logic        super_q;
    logic        fault_q;

    // Latched request context
    always_ff @(posedge clk or negedge rst_l) begin
        if (!rst_l) begin
            state_q <= S_IDLE;
            vpn_q <= 20'b0; acc_q <= 2'b0; priv_q <= PRIV_M;
            sum_q <= 1'b0; mxr_q <= 1'b0;
            pte_q <= 32'b0; pte_addr_q <= 32'b0; super_q <= 1'b0;
            fault_q <= 1'b0;
        end else begin
            state_q <= state_n;
            if (state_q == S_IDLE && req_valid) begin
                vpn_q  <= req_vpn;
                acc_q  <= req_access;
                priv_q <= req_priv;
                sum_q  <= mstatus_sum;
                mxr_q  <= mstatus_mxr;
            end
            // Capture PTE reads when the memory acknowledges the request
            if ((state_q == S_L1_REQ || state_q == S_L0_REQ) && mem_ack) begin
                pte_q <= mem_rdata;
            end
        end
    end

    // PTE field helpers on the just-read word (combinational view)
    logic        v_bit, r_bit, w_bit, x_bit, u_bit, a_bit, d_bit;
    logic [21:0] pte_ppn;
    assign v_bit   = pte_q[RISCV_Priv::PTE_V];
    assign r_bit   = pte_q[RISCV_Priv::PTE_R];
    assign w_bit   = pte_q[RISCV_Priv::PTE_W];
    assign x_bit   = pte_q[RISCV_Priv::PTE_X];
    assign u_bit   = pte_q[RISCV_Priv::PTE_U];
    assign a_bit   = pte_q[RISCV_Priv::PTE_A];
    assign d_bit   = pte_q[RISCV_Priv::PTE_D];
    assign pte_ppn = pte_q[31:10];

    // Permission check for a leaf PTE
    function automatic logic perm_fault(input logic is_super);
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
        // Misaligned superpage
        if (is_super && (pte_q[19:10] != 10'b0)) fail = 1'b1;
        perm_fault = fail;
    endfunction

    logic leaf, need_ad;
    assign leaf    = r_bit || x_bit;
    // A/D update required (Svade hardware update)
    assign need_ad = (!a_bit) || ((acc_q == ACC_STORE) && (!d_bit));

    // Next-state / datapath
    always_comb begin
        state_n   = state_q;
        mem_req   = 1'b0;
        mem_we    = 1'b0;
        mem_addr  = 32'b0;
        mem_wdata = 32'b0;

        unique case (state_q)
            S_IDLE: begin
                if (req_valid) state_n = S_L1_REQ;
            end
            S_L1_REQ: begin
                mem_req  = 1'b1;
                mem_addr = {satp_ppn[19:0], 12'b0} + {20'b0, vpn_q[19:10], 2'b00};
                if (mem_ack) state_n = S_L1_WAIT;
            end
            S_L1_WAIT: begin
                // pte_q now holds the level-1 PTE
                if (!v_bit || (!r_bit && w_bit)) state_n = S_DONE;       // invalid
                // Take the A/D-update path only for a permitted leaf when ADUE
                // enables hardware update; otherwise complete (and fault below
                // either on the permission violation or the Svade A/D fault).
                else if (leaf)                   state_n =
                    (need_ad && adue && !perm_fault(1'b1)) ? S_AD_REQ : S_DONE;
                else                             state_n = S_L0_REQ;     // pointer
            end
            S_L0_REQ: begin
                mem_req  = 1'b1;
                mem_addr = {pte_ppn[19:0], 12'b0} + {20'b0, vpn_q[9:0], 2'b00};
                if (mem_ack) state_n = S_L0_WAIT;
            end
            S_L0_WAIT: begin
                if (!v_bit || (!r_bit && w_bit) || !leaf) state_n = S_DONE;
                else                                      state_n =
                    (need_ad && adue && !perm_fault(1'b0)) ? S_AD_REQ : S_DONE;
            end
            S_AD_REQ: begin
                mem_req   = 1'b1;
                mem_we    = 1'b1;
                mem_addr  = pte_addr_q;
                mem_wdata = pte_q | (32'b1 << RISCV_Priv::PTE_A) |
                    ((acc_q == ACC_STORE) ? (32'b1 << RISCV_Priv::PTE_D) : 32'b0);
                if (mem_ack) state_n = S_AD_WAIT;
            end
            S_AD_WAIT: state_n = S_DONE;
            S_DONE:    state_n = S_IDLE;
            default:   state_n = S_IDLE;
        endcase
    end

    // Record the PTE address for a possible A/D writeback.
    always_ff @(posedge clk or negedge rst_l) begin
        if (!rst_l) begin
            super_q <= 1'b0;
        end else begin
            if (state_q == S_L1_REQ && mem_ack)
                pte_addr_q <= {satp_ppn[19:0], 12'b0} + {20'b0, vpn_q[19:10], 2'b00};
            if (state_q == S_L0_REQ && mem_ack)
                pte_addr_q <= {pte_ppn[19:0], 12'b0} + {20'b0, vpn_q[9:0], 2'b00};
            if (state_q == S_L1_WAIT)
                super_q <= leaf;        // level-1 leaf => superpage
            else if (state_q == S_L0_WAIT)
                super_q <= 1'b0;
        end
    end

    // Svade A/D fault: a permitted leaf that needs an A/D update but ADUE is
    // off raises a page fault instead of a hardware update.
    logic ad_fault;
    assign ad_fault = need_ad && !adue;

    // Fault computation at the completing level
    logic l1_fault, l0_fault;
    assign l1_fault = (!v_bit || (!r_bit && w_bit)) ||
                      (leaf && perm_fault(1'b1)) ||
                      (leaf && ad_fault);
    assign l0_fault = (!v_bit || (!r_bit && w_bit) || !leaf) ||
                      perm_fault(1'b0) ||
                      ad_fault;

    assign busy      = (state_q != S_IDLE);
    assign done      = (state_q == S_DONE);
    assign superpage = super_q;
    assign ppn       = pte_ppn;
    assign walk_vpn     = vpn_q;
    assign walk_is_data = (acc_q != ACC_FETCH);
    // Reflect the A/D bits the hardware update set (Svadu) in the reported perm,
    // so the TLB is filled with the post-update PTE. Otherwise a store would see
    // a stale D=0 in the DTLB after its A/D walk, fail dtlb_usable on the commit
    // cycle, and miss its one-shot store-commit window (orphaning the access).
    assign perm      = pte_q[7:0] |
        ((need_ad && adue) ? ((8'b1 << RISCV_Priv::PTE_A) |
            ((acc_q == ACC_STORE) ? (8'b1 << RISCV_Priv::PTE_D) : 8'b0)) : 8'b0);

    // fault asserted with done: recompute based on which level produced it.
    // (When done is reached from S_L1_WAIT path super_q is set.)
    assign fault     = done && (super_q ? l1_fault : l0_fault);

endmodule: ptw
