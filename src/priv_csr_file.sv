/**
 * priv_csr_file.sv
 *
 * Machine + Supervisor + User control/status register file for the niigo-lake
 * privileged ISA. Holds the architectural privilege-mode register and all M/S
 * CSR state, performs CSR read (with privilege/legality checks) and write, and
 * applies the architectural state transitions for trap entry (driven by
 * trap_controller) and trap return (mret/sret).
 *
 * The module is shared by the scalar prototype core and the out-of-order core.
 * CSR reads are combinational; CSR writes and trap/return updates are applied
 * synchronously. Trap/return updates take priority over a same-cycle CSR write.
 */

`include "riscv_priv.vh"

`default_nettype none

module priv_csr_file
    import RISCV_Priv::*;
(
    input  logic        clk,
    input  logic        rst_l,

    // Retirement / counters
    input  logic        retire,            // an instruction retired this cycle
    input  logic [63:0] mtime,             // CLINT time for the TIME CSR

    // CSR read port (combinational). Port 0 is used by the scalar core; the
    // out-of-order core additionally uses port 1 for its second ALU issue slot.
    input  logic [11:0] read_addr,
    output logic [31:0] read_data,
    output logic        read_illegal,
    input  logic [11:0] read_addr1,
    output logic [31:0] read_data1,
    output logic        read_illegal1,

    // CSR write port (applied at retire of a CSR instruction)
    input  logic        write_valid,
    input  logic [11:0] write_addr,
    input  logic [31:0] write_data,

    // Accrued FP flags (from FP units)
    input  logic        fp_fflags_valid,
    input  logic [4:0]  fp_fflags,
    output logic [2:0]  frm_value,

    // Hardware interrupt sources
    input  logic        irq_m_timer,
    input  logic        irq_m_software,
    input  logic        irq_m_external,
    input  logic        irq_s_external,

    // Trap entry (from trap_controller)
    input  logic        trap_valid,
    input  logic        trap_is_interrupt,
    input  logic [4:0]  trap_cause,
    input  logic [31:0] trap_epc,
    input  logic [31:0] trap_tval,
    input  priv_mode_t  trap_target_priv,

    // Trap return
    input  logic        ret_valid,
    input  logic        ret_from_s,        // 1 = SRET, 0 = MRET

    // Architectural state exposed to the rest of the core / trap_controller
    output priv_mode_t  priv,
    output logic [31:0] mstatus,
    output logic [31:0] medeleg,
    output logic [31:0] mideleg,
    output logic [31:0] mie_csr,
    output logic [31:0] mip_csr,
    output logic [31:0] mtvec,
    output logic [31:0] stvec,
    output logic [31:0] mepc,
    output logic [31:0] sepc,
    output logic [31:0] satp,

    // PMP configuration exposed to the PMP checker
    output logic [31:0] pmpcfg_o  [4],
    output logic [31:0] pmpaddr_o [16],

    // menvcfg.ADUE: when set, the PTW performs hardware A/D updates (Svadu);
    // when clear, a page needing an A/D update faults instead (Svade).
    output logic        menvcfg_adue
);

    /*------------------------------------------------------------------------
     * Interrupt bit positions within mie/mip
     *----------------------------------------------------------------------*/
    localparam int SSI_BIT = 1;
    localparam int MSI_BIT = 3;
    localparam int STI_BIT = 5;
    localparam int MTI_BIT = 7;
    localparam int SEI_BIT = 9;
    localparam int MEI_BIT = 11;
    // Software-writable / S-mode visible interrupt mask
    localparam logic [31:0] S_INT_MASK = 32'h0000_0222;  // SSI|STI|SEI
    localparam logic [31:0] M_INT_MASK = 32'h0000_0AAA;  // all M+S enables
    /* m/scounteren WARL mask: only CY (bit0), TM (bit1) and IR (bit2) are
     * implemented; the programmable HPM counters (bits 3-31) are read-zero, so
     * their enable bits are hardwired to zero. */
    localparam logic [31:0] COUNTEREN_MASK = 32'h0000_0007;

    /*------------------------------------------------------------------------
     * Architectural state
     *----------------------------------------------------------------------*/
    priv_mode_t priv_q;

    // mstatus fields
    logic        st_mie_q, st_sie_q, st_mpie_q, st_spie_q, st_spp_q;
    logic [1:0]  st_mpp_q;
    logic [1:0]  st_fs_q;
    logic        st_mprv_q, st_sum_q, st_mxr_q, st_tvm_q, st_tw_q, st_tsr_q;

    logic [31:0] mtvec_q, stvec_q;
    logic [31:0] mepc_q, sepc_q;
    logic [31:0] mcause_q, scause_q;
    logic [31:0] mtval_q, stval_q;
    logic [31:0] mscratch_q, sscratch_q;
    logic [31:0] medeleg_q, mideleg_q;
    logic [31:0] mie_q;                 // interrupt-enable register
    logic [31:0] mideleg_seip_q;        // unused placeholder (kept 0)
    logic [31:0] satp_q;
    logic [31:0] mcounteren_q, scounteren_q;
    logic [31:0] mcountinhibit_q;       // CY (bit0) / IR (bit2) implemented
    localparam logic [31:0] MCNTINHIBIT_MASK = 32'h0000_0005;
    logic [31:0] menvcfg_q;
    logic        menvcfg_adue_q;        // menvcfg.ADUE (bit 61 -> menvcfgh[29])
    logic        senvcfg_fiom_q;        // senvcfg.FIOM (bit 0); other fields 0
    // Software-writable interrupt-pending bits (S-mode bits)
    logic        ssip_q, stip_q, seip_sw_q;

    // Counters
    logic [63:0] mcycle_q, minstret_q;

    // FP CSRs
    logic [4:0]  fflags_q;
    logic [2:0]  frm_q;

    // PMP
    logic [31:0] pmpcfg_q  [4];
    logic [31:0] pmpaddr_q [16];

    assign frm_value = frm_q;
    assign priv      = priv_q;

    always_comb begin
        for (int i = 0; i < 4;  i += 1) pmpcfg_o[i]  = pmpcfg_q[i];
        for (int i = 0; i < 16; i += 1) pmpaddr_o[i] = pmpaddr_q[i];
    end

    /*------------------------------------------------------------------------
     * Assembled read-only views
     *----------------------------------------------------------------------*/
    logic [31:0] mstatus_v;
    always_comb begin
        mstatus_v = 32'b0;
        mstatus_v[MSTATUS_SIE_BIT]  = st_sie_q;
        mstatus_v[MSTATUS_MIE_BIT]  = st_mie_q;
        mstatus_v[MSTATUS_SPIE_BIT] = st_spie_q;
        mstatus_v[MSTATUS_MPIE_BIT] = st_mpie_q;
        mstatus_v[MSTATUS_SPP_BIT]  = st_spp_q;
        mstatus_v[MSTATUS_MPP_LO+:2] = st_mpp_q;
        mstatus_v[MSTATUS_FS_LO+:2]  = st_fs_q;
        mstatus_v[MSTATUS_MPRV_BIT] = st_mprv_q;
        mstatus_v[MSTATUS_SUM_BIT]  = st_sum_q;
        mstatus_v[MSTATUS_MXR_BIT]  = st_mxr_q;
        mstatus_v[MSTATUS_TVM_BIT]  = st_tvm_q;
        mstatus_v[MSTATUS_TW_BIT]   = st_tw_q;
        mstatus_v[MSTATUS_TSR_BIT]  = st_tsr_q;
        mstatus_v[MSTATUS_SD_BIT]   = (st_fs_q == 2'b11);
    end

    // mip: hardware bits OR software-writable S bits
    logic [31:0] mip_v;
    always_comb begin
        mip_v = 32'b0;
        mip_v[MTI_BIT] = irq_m_timer;
        mip_v[MSI_BIT] = irq_m_software;
        mip_v[MEI_BIT] = irq_m_external;
        mip_v[SEI_BIT] = irq_s_external | seip_sw_q;
        mip_v[STI_BIT] = stip_q;
        mip_v[SSI_BIT] = ssip_q;
    end

    localparam logic [31:0] MISA_VALUE =
        (32'b1 << 30) |   // MXL = 1 (RV32)
        (32'b1 << 0)  |   // A
        (32'b1 << 3)  |   // D
        (32'b1 << 5)  |   // F
        (32'b1 << 8)  |   // I
        (32'b1 << 12) |   // M
        (32'b1 << 18) |   // S
        (32'b1 << 20);    // U

    assign mstatus = mstatus_v;
    assign medeleg = medeleg_q;
    assign mideleg = mideleg_q;
    assign mie_csr = mie_q;
    assign mip_csr = mip_v;
    assign mtvec   = mtvec_q;
    assign stvec   = stvec_q;
    assign mepc    = mepc_q;
    assign sepc    = sepc_q;
    assign satp    = satp_q;
    assign menvcfg_adue = menvcfg_adue_q;

    /*------------------------------------------------------------------------
     * CSR read (combinational) with privilege + existence checks
     *----------------------------------------------------------------------*/
    always_comb begin
        read_csr(read_addr, read_data, read_illegal);
        read_csr(read_addr1, read_data1, read_illegal1);
    end

    task automatic read_csr(input logic [11:0] addr,
            output logic [31:0] data, output logic illegal);
        logic priv_ok;
        data = 32'b0;
        illegal = 1'b0;
        // Minimum privilege required is encoded in addr[9:8].
        priv_ok = (priv_q >= priv_mode_t'(addr[9:8]));
        unique case (addr)
            CSR_FFLAGS:    data = {27'b0, fflags_q};
            CSR_FRM:       data = {29'b0, frm_q};
            CSR_FCSR:      data = {24'b0, frm_q, fflags_q};

            CSR_CYCLE,
            CSR_MCYCLE:    data = mcycle_q[31:0];
            CSR_CYCLEH,
            CSR_MCYCLEH:   data = mcycle_q[63:32];
            CSR_TIME:      data = mtime[31:0];
            CSR_TIMEH:     data = mtime[63:32];
            CSR_INSTRET,
            CSR_MINSTRET:  data = minstret_q[31:0];
            CSR_INSTRETH,
            CSR_MINSTRETH: data = minstret_q[63:32];

            // Supervisor
            CSR_SSTATUS:   data = mstatus_v & SSTATUS_MASK;
            CSR_SIE:       data = mie_q & mideleg_q;
            CSR_STVEC:     data = stvec_q;
            CSR_SCOUNTEREN:data = scounteren_q;
            CSR_SSCRATCH:  data = sscratch_q;
            CSR_SEPC:      data = sepc_q;
            CSR_SCAUSE:    data = scause_q;
            CSR_STVAL:     data = stval_q;
            CSR_SIP:       data = mip_v & mideleg_q;
            CSR_SATP: begin
                data = satp_q;
                // satp inaccessible from S-mode when mstatus.TVM=1
                if ((priv_q == PRIV_S) && st_tvm_q) illegal = 1'b1;
            end
            // senvcfg: only FIOM (bit 0) is implemented (no Zicbom/Zicboz), the
            // rest read as zero (WARL).
            CSR_SENVCFG:   data = {31'b0, senvcfg_fiom_q};

            // Machine information
            CSR_MVENDORID: data = 32'b0;
            CSR_MARCHID:   data = 32'b0;
            CSR_MIMPID:    data = 32'b0;
            CSR_MHARTID:   data = 32'b0;

            // Machine trap setup
            CSR_MSTATUS:   data = mstatus_v;
            CSR_MSTATUSH:  data = 32'b0;
            CSR_MISA:      data = MISA_VALUE;
            CSR_MEDELEG:   data = medeleg_q;
            CSR_MIDELEG:   data = mideleg_q;
            CSR_MIE:       data = mie_q;
            CSR_MTVEC:     data = mtvec_q;
            CSR_MCOUNTEREN:data = mcounteren_q;
            CSR_MENVCFG:   data = menvcfg_q;
            // menvcfgh holds the upper 32 bits of menvcfg on RV32. Only ADUE
            // (bit 61 -> menvcfgh[29]) is implemented (Svadu HW A/D update);
            // STCE/PBMTE and the rest read as zero and ignore writes (WARL).
            CSR_MENVCFGH:  data = {2'b0, menvcfg_adue_q, 29'b0};

            // Machine trap handling
            CSR_MSCRATCH:  data = mscratch_q;
            CSR_MEPC:      data = mepc_q;
            CSR_MCAUSE:    data = mcause_q;
            CSR_MTVAL:     data = mtval_q;
            CSR_MIP:       data = mip_v;

            CSR_PMPCFG0:   data = pmpcfg_q[0];
            CSR_PMPCFG1:   data = pmpcfg_q[1];
            CSR_PMPCFG2:   data = pmpcfg_q[2];
            CSR_PMPCFG3:   data = pmpcfg_q[3];

            default: begin
                if ((addr >= CSR_PMPADDR0) && (addr <= CSR_PMPADDR0 + 12'd15)) begin
                    data = pmpaddr_q[addr - CSR_PMPADDR0];
                // Hardware performance-monitor CSRs. niigo implements none of the
                // programmable event counters, but the privileged spec requires
                // mcountinhibit, mhpmevent3-31 and mhpmcounter3-31 (plus the high
                // halves and the U-mode-readable mirrors) to exist as WARL
                // registers once any counter is implemented. Model them as
                // read-zero / ignore-write so the arch-test boot code that zeroes
                // them does not take a spurious illegal-instruction trap.
                end else if (addr == CSR_MCOUNTINHIBIT) begin
                    data = mcountinhibit_q;
                end else if ((addr >= CSR_MHPMEVENT3)   && (addr <= CSR_MHPMEVENT31))  begin
                    data = 32'b0;                       // mhpmevent3-31   (0x323-0x33F)
                end else if ((addr >= CSR_MHPMEVENT3H)  && (addr <= CSR_MHPMEVENT31H)) begin
                    data = 32'b0;                       // mhpmevent3h-31h (0x723-0x73F)
                end else if ((addr >= CSR_MHPMCOUNTER3)  && (addr <= CSR_MHPMCOUNTER31))  begin
                    data = 32'b0;                       // mhpmcounter3-31  (0xB03-0xB1F)
                end else if ((addr >= CSR_MHPMCOUNTER3H) && (addr <= CSR_MHPMCOUNTER31H)) begin
                    data = 32'b0;                       // mhpmcounter3h-31h(0xB83-0xB9F)
                end else if ((addr >= CSR_HPMCOUNTER3)   && (addr <= CSR_HPMCOUNTER31))  begin
                    data = 32'b0;                       // hpmcounter3-31   (0xC03-0xC1F)
                end else if ((addr >= CSR_HPMCOUNTER3H)  && (addr <= CSR_HPMCOUNTER31H)) begin
                    data = 32'b0;                       // hpmcounter3h-31h (0xC83-0xC9F)
                end else begin
                    data = 32'b0;
                    illegal = 1'b1;
                end
            end
        endcase
        // Counter access control: the unprivileged cycle/time/instret aliases
        // (0xC00-0xC02 and the *h 0xC80-0xC82) require the matching mcounteren
        // bit to be read from S/U, and additionally the scounteren bit from U.
        // The M-mode mcycle/minstret (0xB0x) aliases are never gated.
        if ((addr[11:8] == 4'hC) && (addr[6:2] == 5'b0) && (addr[1:0] != 2'b11)
                && (priv_q != PRIV_M)) begin
            if (!mcounteren_q[addr[1:0]]) illegal = 1'b1;
            else if ((priv_q == PRIV_U) && !scounteren_q[addr[1:0]])
                illegal = 1'b1;
        end
        if (!priv_ok) illegal = 1'b1;
    endtask

    /*------------------------------------------------------------------------
     * State update
     *----------------------------------------------------------------------*/
    always_ff @(posedge clk or negedge rst_l) begin
        if (!rst_l) begin
            priv_q      <= PRIV_M;
            st_mie_q    <= 1'b0;
            st_sie_q    <= 1'b0;
            st_mpie_q   <= 1'b0;
            st_spie_q   <= 1'b0;
            st_spp_q    <= 1'b0;
            st_mpp_q    <= 2'b00;
            st_fs_q     <= 2'b01;     // Initial: FP enabled (RV32G core)
            st_mprv_q   <= 1'b0;
            st_sum_q    <= 1'b0;
            st_mxr_q    <= 1'b0;
            st_tvm_q    <= 1'b0;
            st_tw_q     <= 1'b0;
            st_tsr_q    <= 1'b0;
            mtvec_q     <= 32'b0;
            stvec_q     <= 32'b0;
            mepc_q      <= 32'b0;
            sepc_q      <= 32'b0;
            mcause_q    <= 32'b0;
            scause_q    <= 32'b0;
            mtval_q     <= 32'b0;
            stval_q     <= 32'b0;
            mscratch_q  <= 32'b0;
            sscratch_q  <= 32'b0;
            medeleg_q   <= 32'b0;
            mideleg_q   <= 32'b0;
            mie_q       <= 32'b0;
            satp_q      <= 32'b0;
            mcounteren_q<= 32'b0;
            scounteren_q<= 32'b0;
            mcountinhibit_q <= 32'b0;
            menvcfg_q   <= 32'b0;
            menvcfg_adue_q <= 1'b0;
            senvcfg_fiom_q <= 1'b0;
            ssip_q      <= 1'b0;
            stip_q      <= 1'b0;
            seip_sw_q   <= 1'b0;
            mcycle_q    <= 64'b0;
            minstret_q  <= 64'b0;
            fflags_q    <= 5'b0;
            frm_q       <= 3'b0;
            for (int i = 0; i < 4; i += 1)  pmpcfg_q[i]  <= 32'b0;
            for (int i = 0; i < 16; i += 1) pmpaddr_q[i] <= 32'b0;
        end else begin
            // Free-running counters, gated by mcountinhibit (CY bit0 / IR bit2).
            if (!mcountinhibit_q[0]) mcycle_q <= mcycle_q + 64'd1;
            if (retire && !mcountinhibit_q[2])
                minstret_q <= minstret_q + 64'd1;

            // Accumulate FP flags every cycle.
            if (fp_fflags_valid) fflags_q <= fflags_q | fp_fflags;

            if (trap_valid) begin
                // Trap entry. Save state into the target privilege's CSRs.
                if (trap_target_priv == PRIV_M) begin
                    mepc_q   <= {trap_epc[31:2], 2'b0};
                    mcause_q <= {trap_is_interrupt, 26'b0, trap_cause};
                    mtval_q  <= trap_tval;
                    st_mpie_q <= st_mie_q;
                    st_mie_q  <= 1'b0;
                    st_mpp_q  <= priv_q;
                end else begin
                    sepc_q   <= {trap_epc[31:2], 2'b0};
                    scause_q <= {trap_is_interrupt, 26'b0, trap_cause};
                    stval_q  <= trap_tval;
                    st_spie_q <= st_sie_q;
                    st_sie_q  <= 1'b0;
                    st_spp_q  <= (priv_q == PRIV_U) ? 1'b0 : 1'b1;
                end
                priv_q <= trap_target_priv;
            end else if (ret_valid) begin
                if (ret_from_s) begin
                    // SRET
                    priv_q   <= st_spp_q ? PRIV_S : PRIV_U;
                    st_sie_q <= st_spie_q;
                    st_spie_q<= 1'b1;
                    st_spp_q <= 1'b0;
                    st_mprv_q<= 1'b0;     // returning below M clears MPRV
                end else begin
                    // MRET
                    priv_q   <= priv_mode_t'(st_mpp_q);
                    st_mie_q <= st_mpie_q;
                    st_mpie_q<= 1'b1;
                    if (priv_mode_t'(st_mpp_q) != PRIV_M) st_mprv_q <= 1'b0;
                    st_mpp_q <= 2'b00;
                end
            end else if (write_valid) begin
                apply_csr_write(write_addr, write_data);
            end

            /* Floating-point state tracking. Per the privileged spec, mstatus.FS
             * becomes Dirty (11) whenever the FP unit modifies FP state: an FP
             * instruction that accrues exception flags, or a write to one of the
             * FP CSRs (fflags/frm/fcsr). The arch-test boot relies on this (its
             * `csrw fcsr, zero` is what enables FP / sets SD), so without it the
             * DUT reads mstatus with FS=0 where the reference reads FS=Dirty.
             * An explicit mstatus/sstatus write in the same cycle takes priority
             * (it is the architectural source of truth for the FS field). */
            if ((fp_fflags_valid ||
                 (write_valid && (write_addr == CSR_FCSR ||
                                  write_addr == CSR_FFLAGS ||
                                  write_addr == CSR_FRM))) &&
                !(write_valid && (write_addr == CSR_MSTATUS ||
                                  write_addr == CSR_SSTATUS))) begin
                st_fs_q <= 2'b11;
            end
        end
    end

    /*------------------------------------------------------------------------
     * Apply a CSR write (WARL masking handled per register)
     *----------------------------------------------------------------------*/
    task automatic apply_csr_write(input logic [11:0] addr,
            input logic [31:0] wdata);
        unique case (addr)
            CSR_FFLAGS: fflags_q <= wdata[4:0];
            CSR_FRM:    frm_q    <= wdata[2:0];
            CSR_FCSR: begin
                fflags_q <= wdata[4:0];
                frm_q    <= wdata[7:5];
            end

            CSR_SSTATUS: begin
                // Only the S-visible writable fields
                st_sie_q  <= wdata[MSTATUS_SIE_BIT];
                st_spie_q <= wdata[MSTATUS_SPIE_BIT];
                st_spp_q  <= wdata[MSTATUS_SPP_BIT];
                st_fs_q   <= wdata[MSTATUS_FS_LO+:2];
                st_sum_q  <= wdata[MSTATUS_SUM_BIT];
                st_mxr_q  <= wdata[MSTATUS_MXR_BIT];
            end
            CSR_SIE:    mie_q <= (mie_q & ~mideleg_q) | (wdata & mideleg_q & S_INT_MASK);
            CSR_STVEC:  stvec_q <= {wdata[31:2], 1'b0, wdata[0]};
            CSR_SCOUNTEREN: scounteren_q <= wdata & COUNTEREN_MASK;
            CSR_SSCRATCH: sscratch_q <= wdata;
            CSR_SEPC:   sepc_q <= {wdata[31:2], 2'b0};
            CSR_SCAUSE: scause_q <= wdata;
            CSR_STVAL:  stval_q <= wdata;
            CSR_SIP: begin
                if (mideleg_q[SSI_BIT]) ssip_q <= wdata[SSI_BIT];
            end
            CSR_SATP:   satp_q <= wdata;
            CSR_SENVCFG: senvcfg_fiom_q <= wdata[0];

            CSR_MSTATUS: begin
                st_sie_q  <= wdata[MSTATUS_SIE_BIT];
                st_mie_q  <= wdata[MSTATUS_MIE_BIT];
                st_spie_q <= wdata[MSTATUS_SPIE_BIT];
                st_mpie_q <= wdata[MSTATUS_MPIE_BIT];
                st_spp_q  <= wdata[MSTATUS_SPP_BIT];
                st_mpp_q  <= legal_mpp(wdata[MSTATUS_MPP_LO+:2]);
                st_fs_q   <= wdata[MSTATUS_FS_LO+:2];
                st_mprv_q <= wdata[MSTATUS_MPRV_BIT];
                st_sum_q  <= wdata[MSTATUS_SUM_BIT];
                st_mxr_q  <= wdata[MSTATUS_MXR_BIT];
                st_tvm_q  <= wdata[MSTATUS_TVM_BIT];
                st_tw_q   <= wdata[MSTATUS_TW_BIT];
                st_tsr_q  <= wdata[MSTATUS_TSR_BIT];
            end
            CSR_MEDELEG: medeleg_q <= wdata & 32'h0000_F7FF;
            CSR_MIDELEG: mideleg_q <= wdata & S_INT_MASK;
            CSR_MIE:     mie_q <= wdata & M_INT_MASK;
            CSR_MTVEC:   mtvec_q <= {wdata[31:2], 1'b0, wdata[0]};
            CSR_MCOUNTEREN: mcounteren_q <= wdata & COUNTEREN_MASK;
            CSR_MCOUNTINHIBIT: mcountinhibit_q <= wdata & MCNTINHIBIT_MASK;
            CSR_MENVCFG: menvcfg_q <= wdata;
            CSR_MENVCFGH: menvcfg_adue_q <= wdata[29];   // WARL: only ADUE
            CSR_MSCRATCH: mscratch_q <= wdata;
            CSR_MEPC:    mepc_q <= {wdata[31:2], 2'b0};
            CSR_MCAUSE:  mcause_q <= wdata;
            CSR_MTVAL:   mtval_q <= wdata;
            CSR_MIP: begin
                // M-mode may write the software-settable S-mode pending bits.
                // mip.SEIP is a software-writable alias that ORs with the real
                // supervisor external interrupt input on read.
                ssip_q    <= wdata[SSI_BIT];
                stip_q    <= wdata[STI_BIT];
                seip_sw_q <= wdata[SEI_BIT];
            end
            // PMP config writes honour per-byte lock (L) and WARL masking.
            CSR_PMPCFG0: pmpcfg_q[0] <= pmp_cfg_word_wr(pmpcfg_q[0], wdata);
            CSR_PMPCFG1: pmpcfg_q[1] <= pmp_cfg_word_wr(pmpcfg_q[1], wdata);
            CSR_PMPCFG2: pmpcfg_q[2] <= pmp_cfg_word_wr(pmpcfg_q[2], wdata);
            CSR_PMPCFG3: pmpcfg_q[3] <= pmp_cfg_word_wr(pmpcfg_q[3], wdata);
            default: begin
                if ((addr >= CSR_PMPADDR0) && (addr <= CSR_PMPADDR0 + 12'd15)) begin
                    // A locked entry (or the base of a locked TOR entry above)
                    // ignores pmpaddr writes.
                    if (!pmp_addr_locked(addr - CSR_PMPADDR0))
                        pmpaddr_q[addr - CSR_PMPADDR0] <= wdata;
                end
            end
        endcase
    endtask

    // One PMP cfg byte: locked bytes are immutable; otherwise mask the WARL
    // reserved bits [6:5] and the reserved R=0,W=1 combination (force W=0).
    function automatic logic [7:0] pmp_cfg_byte_wr(input logic [7:0] cur,
            input logic [7:0] nw);
        logic [7:0] r;
        if (cur[7]) begin
            r = cur;                 // locked
        end else begin
            r = nw;
            r[6:5] = 2'b00;          // reserved
            if (!r[0]) r[1] = 1'b0;  // R=0 => W=0
        end
        pmp_cfg_byte_wr = r;
    endfunction

    function automatic logic [31:0] pmp_cfg_word_wr(input logic [31:0] cur,
            input logic [31:0] nw);
        logic [31:0] r;
        for (int b = 0; b < 4; b += 1)
            r[b*8 +: 8] = pmp_cfg_byte_wr(cur[b*8 +: 8], nw[b*8 +: 8]);
        pmp_cfg_word_wr = r;
    endfunction

    // pmpaddr[i] is locked if its own cfg byte is locked, or if the next entry
    // is a locked TOR (which uses pmpaddr[i] as its lower bound).
    function automatic logic pmp_addr_locked(input logic [3:0] i);
        logic [7:0] c_i, c_n;
        c_i = pmpcfg_q[i[3:2]][i[1:0]*8 +: 8];
        pmp_addr_locked = c_i[7];
        if (i != 4'd15) begin
            c_n = pmpcfg_q[(i+4'd1) >> 2][((i+4'd1) & 4'd3)*8 +: 8];
            if (c_n[7] && (c_n[4:3] == 2'd1)) pmp_addr_locked = 1'b1;
        end
    endfunction

    function automatic logic [1:0] legal_mpp(input logic [1:0] v);
        // Only U(00), S(01), M(11) are legal MPP encodings.
        unique case (v)
            2'b00, 2'b01, 2'b11: legal_mpp = v;
            default:             legal_mpp = 2'b00;
        endcase
    endfunction

`ifdef AGENT_DEBUG
    always_ff @(posedge clk) begin
        if (rst_l && write_valid &&
                (write_addr == CSR_MSTATUS || write_addr == CSR_SSTATUS))
            $display("[CSR] write %h <= %h (FS<=%b) priv=%0d",
                write_addr, write_data, write_data[MSTATUS_FS_LO+:2], priv_q);
        if (rst_l && (read_addr == CSR_MSTATUS) && !read_illegal)
            $display("[CSR] read mstatus => %h (FS=%b SD=%b) priv=%0d",
                read_data, read_data[MSTATUS_FS_LO+:2], read_data[MSTATUS_SD_BIT],
                priv_q);
    end
`endif

endmodule: priv_csr_file
