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
#(
    // M4: per-core hart id read back by the mhartid CSR. Default 0 == the single-core build,
    // so mhartid stays 0 and the baseline is bit-identical; an SMP top sets 0,1,2,3 per core.
    parameter logic [MXLEN-1:0] HART_ID = '0
)
(
    input wire logic        clk,
    input wire logic        rst_l,

    // Retirement / counters
    input wire logic [2:0]  retire_cnt,        // # instructions retired this cycle (0..4)
    input wire logic [63:0] mtime,             // CLINT time for the TIME CSR

    // CSR read port (combinational). Port 0 is used by the scalar core; the
    // out-of-order core additionally uses port 1 for its second ALU issue slot.
    input wire logic [11:0] read_addr,
    output logic [MXLEN-1:0] read_data,
    output logic        read_illegal,
    input wire logic [11:0] read_addr1,
    output logic [MXLEN-1:0] read_data1,
    output logic        read_illegal1,

    // CSR write port (applied at retire of a CSR instruction)
    input wire logic        write_valid,
    input wire logic [11:0] write_addr,
    input wire logic [MXLEN-1:0] write_data,

    // Accrued FP flags (from FP units)
    input wire logic        fp_fflags_valid,
    input wire logic [4:0]  fp_fflags,
    output logic [2:0]  frm_value,

    // Cache event pulses driving mhpmcounter3-5 (phase C3): L1I miss / L1D miss
    // / L1D writeback. Left unconnected (-> 0) on scalar / L1=0 builds, so those
    // counters read zero exactly as before.
    input wire logic        cache_ev_l1i_miss,
    input wire logic        cache_ev_l1d_miss,
    input wire logic        cache_ev_l1d_wb,

    // Hardware interrupt sources
    input wire logic        irq_m_timer,
    input wire logic        irq_m_software,
    input wire logic        irq_m_external,
    input wire logic        irq_s_external,

    // Trap entry (from trap_controller)
    input wire logic        trap_valid,
    input wire logic        trap_is_interrupt,
    input wire logic [4:0]  trap_cause,
    input wire logic [MXLEN-1:0] trap_epc,
    input wire logic [MXLEN-1:0] trap_tval,
    input wire priv_mode_t  trap_target_priv,

    // Trap return
    input wire logic        ret_valid,
    input wire logic        ret_from_s,        // 1 = SRET, 0 = MRET

    // Architectural state exposed to the rest of the core / trap_controller
    output priv_mode_t  priv,
    output logic [MXLEN-1:0] mstatus,
    output logic [MXLEN-1:0] medeleg,
    output logic [MXLEN-1:0] mideleg,
    output logic [MXLEN-1:0] mie_csr,
    output logic [MXLEN-1:0] mip_csr,
    output logic [MXLEN-1:0] mtvec,
    output logic [MXLEN-1:0] stvec,
    output logic [MXLEN-1:0] mepc,
    output logic [MXLEN-1:0] sepc,
    output logic [MXLEN-1:0] satp,

    // PMP configuration exposed to the PMP checker
    output logic [31:0] pmpcfg_o  [4],
    output logic [MXLEN-1:0] pmpaddr_o [16],

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
    localparam logic [MXLEN-1:0] S_INT_MASK = MXLEN'('h0000_0222);  // SSI|STI|SEI
    localparam logic [MXLEN-1:0] M_INT_MASK = MXLEN'('h0000_0AAA);  // all M+S enables
    /* m/scounteren WARL mask: only CY (bit0), TM (bit1) and IR (bit2) are
     * implemented; the programmable HPM counters (bits 3-31) are read-zero, so
     * their enable bits are hardwired to zero. */
    localparam logic [MXLEN-1:0] COUNTEREN_MASK = MXLEN'('h0000_0007);

    /*------------------------------------------------------------------------
     * Architectural state
     *----------------------------------------------------------------------*/
    priv_mode_t priv_q;

    // mstatus fields
    logic        st_mie_q, st_sie_q, st_mpie_q, st_spie_q, st_spp_q;
    logic [1:0]  st_mpp_q;
    logic [1:0]  st_fs_q;
    logic        st_mprv_q, st_sum_q, st_mxr_q, st_tvm_q, st_tw_q, st_tsr_q;

    logic [MXLEN-1:0] mtvec_q, stvec_q;
    logic [MXLEN-1:0] mepc_q, sepc_q;
    logic [MXLEN-1:0] mcause_q, scause_q;
    logic [MXLEN-1:0] mtval_q, stval_q;
    logic [MXLEN-1:0] mscratch_q, sscratch_q;
    logic [MXLEN-1:0] medeleg_q, mideleg_q;
    logic [MXLEN-1:0] mie_q;            // interrupt-enable register
    logic [MXLEN-1:0] mideleg_seip_q;   // unused placeholder (kept 0)
    logic [MXLEN-1:0] satp_q;
    logic [MXLEN-1:0] mcounteren_q, scounteren_q;
    logic [MXLEN-1:0] mcountinhibit_q;  // CY (bit0) / IR (bit2) implemented
    // CY (bit0), IR (bit2), and the three implemented HPM counters HPM3-5
    // (bits 3-5) are inhibitable; everything else reads/writes zero.
    localparam logic [MXLEN-1:0] MCNTINHIBIT_MASK = MXLEN'('h0000_003D);

    // Phase-C3 cache event counters (mhpmcounter3-5 / hpmcounter3-5).
    logic [63:0] hpm3_q, hpm4_q, hpm5_q;  // L1I miss / L1D miss / L1D writeback
    function automatic logic [MXLEN-1:0] hpm_read(input logic [11:0] idx,
            input logic hi);
        logic [63:0] c;
        unique case (idx)
            12'd0:   c = hpm3_q;
            12'd1:   c = hpm4_q;
            12'd2:   c = hpm5_q;
            default: c = 64'd0;
        endcase
`ifdef RV64
        hpm_read = c[MXLEN-1:0];
`else
        hpm_read = hi ? c[63:32] : c[31:0];
`endif
    endfunction
    logic [MXLEN-1:0] menvcfg_q;
    logic        menvcfg_adue_q;        // menvcfg.ADUE (bit 61 -> menvcfgh[29])
    logic        senvcfg_fiom_q;        // senvcfg.FIOM (bit 0); other fields 0
    // Software-writable interrupt-pending bits (S-mode bits)
    logic        ssip_q, stip_q, seip_sw_q;

    // Counters
    logic [63:0] mcycle_q, minstret_q;

    // FP CSRs
    logic [4:0]  fflags_q;
    logic [2:0]  frm_q;

    // PMP. The cfg state is kept as four 32-bit words (eight bytes pack into
    // the even pmpcfg CSRs on RV64); pmpaddr implements PA[33:2] (32 bits) at
    // RV32 and PA[55:2] (54 bits, WARL-zero above) at RV64.
    localparam logic [MXLEN-1:0] PMPADDR_MASK =
        (MXLEN == 64) ? MXLEN'(64'h003F_FFFF_FFFF_FFFF) : MXLEN'(32'hFFFF_FFFF);
    logic [31:0] pmpcfg_q  [4];
    logic [MXLEN-1:0] pmpaddr_q [16];

    assign frm_value = frm_q;
    assign priv      = priv_q;

    always_comb begin
        for (int i = 0; i < 4;  i += 1) pmpcfg_o[i]  = pmpcfg_q[i];
        for (int i = 0; i < 16; i += 1) pmpaddr_o[i] = pmpaddr_q[i];
    end

    /*------------------------------------------------------------------------
     * Assembled read-only views
     *----------------------------------------------------------------------*/
    logic [MXLEN-1:0] mstatus_v;
    always_comb begin
        mstatus_v = '0;
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
`ifdef RV64
        // SXL/UXL are WARL read-only 2: S and U execute at 64-bit.
        mstatus_v[MSTATUS_UXL_LO+:2] = 2'd2;
        mstatus_v[MSTATUS_SXL_LO+:2] = 2'd2;
`endif
    end

    // mip: hardware bits OR software-writable S bits
    logic [MXLEN-1:0] mip_v;
    always_comb begin
        mip_v = '0;
        mip_v[MTI_BIT] = irq_m_timer;
        mip_v[MSI_BIT] = irq_m_software;
        mip_v[MEI_BIT] = irq_m_external;
        mip_v[SEI_BIT] = irq_s_external | seip_sw_q;
        mip_v[STI_BIT] = stip_q;
        mip_v[SSI_BIT] = ssip_q;
    end

    localparam logic [MXLEN-1:0] MISA_VALUE =
        (MXLEN'(MXLEN == 64 ? 2 : 1) << (MXLEN - 2)) |   // MXL
        (MXLEN'(1) << 0)  |   // A
        (MXLEN'(1) << 3)  |   // D
        (MXLEN'(1) << 5)  |   // F
        (MXLEN'(1) << 8)  |   // I
        (MXLEN'(1) << 12) |   // M
        (MXLEN'(1) << 18) |   // S
        (MXLEN'(1) << 20);    // U

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
            output logic [MXLEN-1:0] data, output logic illegal);
        logic priv_ok;
        data = '0;
        illegal = 1'b0;
        // Minimum privilege required is encoded in addr[9:8].
        priv_ok = (priv_q >= priv_mode_t'(addr[9:8]));
        unique case (addr)
            CSR_FFLAGS:    data = MXLEN'(fflags_q);
            CSR_FRM:       data = MXLEN'(frm_q);
            CSR_FCSR:      data = MXLEN'({frm_q, fflags_q});

            // The counter/timer CSRs are 64-bit; on RV32 they are read through
            // the low half plus the *H high-half aliases, which do not exist
            // on RV64 (they fall to the illegal default there).
            CSR_CYCLE,
            CSR_MCYCLE:    data = MXLEN'(mcycle_q[MXLEN-1:0]);
            CSR_TIME:      data = MXLEN'(mtime[MXLEN-1:0]);
            CSR_INSTRET,
            CSR_MINSTRET:  data = MXLEN'(minstret_q[MXLEN-1:0]);
`ifndef RV64
            CSR_CYCLEH,
            CSR_MCYCLEH:   data = mcycle_q[63:32];
            CSR_TIMEH:     data = mtime[63:32];
            CSR_INSTRETH,
            CSR_MINSTRETH: data = minstret_q[63:32];
`endif

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
            CSR_SENVCFG:   data = MXLEN'(senvcfg_fiom_q);

            // Machine information
            CSR_MVENDORID: data = '0;
            CSR_MARCHID:   data = '0;
            CSR_MIMPID:    data = '0;
            CSR_MHARTID:   data = HART_ID;

            // Machine trap setup
            CSR_MSTATUS:   data = mstatus_v;
            CSR_MISA:      data = MISA_VALUE;
            CSR_MEDELEG:   data = medeleg_q;
            CSR_MIDELEG:   data = mideleg_q;
            CSR_MIE:       data = mie_q;
            CSR_MTVEC:     data = mtvec_q;
            CSR_MCOUNTEREN:data = mcounteren_q;
`ifdef RV64
            // menvcfg is a single 64-bit CSR on RV64; ADUE lives at its native
            // bit 61 (Svadu HW A/D update). STCE/PBMTE and the rest are WARL
            // read-zero. mstatush/menvcfgh do not exist (illegal default).
            CSR_MENVCFG:   data = menvcfg_q | (MXLEN'(menvcfg_adue_q) << 61);
`else
            CSR_MSTATUSH:  data = 32'b0;
            CSR_MENVCFG:   data = menvcfg_q;
            // menvcfgh holds the upper 32 bits of menvcfg on RV32. Only ADUE
            // (bit 61 -> menvcfgh[29]) is implemented (Svadu HW A/D update);
            // STCE/PBMTE and the rest read as zero and ignore writes (WARL).
            CSR_MENVCFGH:  data = {2'b0, menvcfg_adue_q, 29'b0};
`endif

            // Machine trap handling
            CSR_MSCRATCH:  data = mscratch_q;
            CSR_MEPC:      data = mepc_q;
            CSR_MCAUSE:    data = mcause_q;
            CSR_MTVAL:     data = mtval_q;
            CSR_MIP:       data = mip_v;

`ifdef RV64
            // RV64 packs eight PMP cfg bytes per even-numbered pmpcfg CSR; the
            // odd-numbered pmpcfg1/3 do not exist (illegal default).
            CSR_PMPCFG0:   data = {pmpcfg_q[1], pmpcfg_q[0]};
            CSR_PMPCFG2:   data = {pmpcfg_q[3], pmpcfg_q[2]};
`else
            CSR_PMPCFG0:   data = pmpcfg_q[0];
            CSR_PMPCFG1:   data = pmpcfg_q[1];
            CSR_PMPCFG2:   data = pmpcfg_q[2];
            CSR_PMPCFG3:   data = pmpcfg_q[3];
`endif

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
                    data = '0;                          // mhpmevent3-31   (0x323-0x33F)
                end else if ((addr >= CSR_MHPMCOUNTER3)  && (addr <= CSR_MHPMCOUNTER31))  begin
                    data = hpm_read(addr - CSR_MHPMCOUNTER3, 1'b0);  // mhpmcounter3-31
                end else if ((addr >= CSR_HPMCOUNTER3)   && (addr <= CSR_HPMCOUNTER31))  begin
                    data = hpm_read(addr - CSR_HPMCOUNTER3, 1'b0);   // hpmcounter3-31
`ifndef RV64
                // The high-half counter aliases exist only on RV32.
                end else if ((addr >= CSR_MHPMEVENT3H)  && (addr <= CSR_MHPMEVENT31H)) begin
                    data = '0;                          // mhpmevent3h-31h (0x723-0x73F)
                end else if ((addr >= CSR_MHPMCOUNTER3H) && (addr <= CSR_MHPMCOUNTER31H)) begin
                    data = hpm_read(addr - CSR_MHPMCOUNTER3H, 1'b1); // mhpmcounter3h-31h
                end else if ((addr >= CSR_HPMCOUNTER3H)  && (addr <= CSR_HPMCOUNTER31H)) begin
                    data = hpm_read(addr - CSR_HPMCOUNTER3H, 1'b1);  // hpmcounter3h-31h
`endif
                end else begin
                    data = '0;
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
            mtvec_q     <= '0;
            stvec_q     <= '0;
            mepc_q      <= '0;
            sepc_q      <= '0;
            mcause_q    <= '0;
            scause_q    <= '0;
            mtval_q     <= '0;
            stval_q     <= '0;
            mscratch_q  <= '0;
            sscratch_q  <= '0;
            medeleg_q   <= '0;
            mideleg_q   <= '0;
            mie_q       <= '0;
            satp_q      <= '0;
            mcounteren_q<= '0;
            scounteren_q<= '0;
            mcountinhibit_q <= '0;
            menvcfg_q   <= '0;
            menvcfg_adue_q <= 1'b0;
            senvcfg_fiom_q <= 1'b0;
            ssip_q      <= 1'b0;
            stip_q      <= 1'b0;
            seip_sw_q   <= 1'b0;
            mcycle_q    <= 64'b0;
            minstret_q  <= 64'b0;
            hpm3_q      <= 64'b0;
            hpm4_q      <= 64'b0;
            hpm5_q      <= 64'b0;
            fflags_q    <= 5'b0;
            frm_q       <= 3'b0;
            for (int i = 0; i < 4; i += 1)  pmpcfg_q[i]  <= 32'b0;
            for (int i = 0; i < 16; i += 1) pmpaddr_q[i] <= '0;
        end else begin
            // Free-running counters, gated by mcountinhibit (CY bit0 / IR bit2).
            if (!mcountinhibit_q[0]) mcycle_q <= mcycle_q + 64'd1;
            // minstret counts EVERY retired instruction, so on a multi-retire
            // (up to 4-wide) commit it must advance by the retire count -- a flat
            // +1 per cycle undercounts and diverges from the ISS golden models.
            if (!mcountinhibit_q[2])
                minstret_q <= minstret_q + 64'(retire_cnt);
            // Cache event counters HPM3-5 (phase C3), gated by mcountinhibit[3-5].
            if (cache_ev_l1i_miss && !mcountinhibit_q[3]) hpm3_q <= hpm3_q + 64'd1;
            if (cache_ev_l1d_miss && !mcountinhibit_q[4]) hpm4_q <= hpm4_q + 64'd1;
            if (cache_ev_l1d_wb   && !mcountinhibit_q[5]) hpm5_q <= hpm5_q + 64'd1;

            // Accumulate FP flags every cycle.
            if (fp_fflags_valid) fflags_q <= fflags_q | fp_fflags;

            if (trap_valid) begin
                // Trap entry. Save state into the target privilege's CSRs.
                // The cause interrupt flag lives at bit MXLEN-1.
                if (trap_target_priv == PRIV_M) begin
                    mepc_q   <= {trap_epc[MXLEN-1:2], 2'b0};
                    mcause_q <= {trap_is_interrupt, {(MXLEN-6){1'b0}}, trap_cause};
                    mtval_q  <= trap_tval;
                    st_mpie_q <= st_mie_q;
                    st_mie_q  <= 1'b0;
                    st_mpp_q  <= priv_q;
                end else begin
                    sepc_q   <= {trap_epc[MXLEN-1:2], 2'b0};
                    scause_q <= {trap_is_interrupt, {(MXLEN-6){1'b0}}, trap_cause};
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
            input logic [MXLEN-1:0] wdata);
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
            CSR_STVEC:  stvec_q <= {wdata[MXLEN-1:2], 1'b0, wdata[0]};
            CSR_SCOUNTEREN: scounteren_q <= wdata & COUNTEREN_MASK;
            CSR_SSCRATCH: sscratch_q <= wdata;
            CSR_SEPC:   sepc_q <= {wdata[MXLEN-1:2], 2'b0};
            CSR_SCAUSE: scause_q <= wdata;
            CSR_STVAL:  stval_q <= wdata;
            CSR_SIP: begin
                if (mideleg_q[SSI_BIT]) ssip_q <= wdata[SSI_BIT];
            end
`ifdef RV64
            // satp MODE[63:60] is WARL: only Bare (0) and Sv39 (8) are
            // supported; a write with any other MODE has no effect at all.
            CSR_SATP: begin
                if ((wdata[63:60] == 4'd0) || (wdata[63:60] == 4'd8))
                    satp_q <= wdata;
            end
`else
            CSR_SATP:   satp_q <= wdata;
`endif
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
            CSR_MEDELEG: medeleg_q <= wdata & MXLEN'('h0000_F7FF);
            CSR_MIDELEG: mideleg_q <= wdata & S_INT_MASK;
            CSR_MIE:     mie_q <= wdata & M_INT_MASK;
            CSR_MTVEC:   mtvec_q <= {wdata[MXLEN-1:2], 1'b0, wdata[0]};
            CSR_MCOUNTEREN: mcounteren_q <= wdata & COUNTEREN_MASK;
            CSR_MCOUNTINHIBIT: mcountinhibit_q <= wdata & MCNTINHIBIT_MASK;
`ifdef RV64
            // 64-bit menvcfg: ADUE at its native bit 61; everything else WARL
            // read-zero (kept out of menvcfg_q so the read can OR ADUE in).
            CSR_MENVCFG: begin
                menvcfg_q <= wdata & ~(MXLEN'(1) << 61);
                menvcfg_adue_q <= wdata[61];
            end
`else
            CSR_MENVCFG: menvcfg_q <= wdata;
            CSR_MENVCFGH: menvcfg_adue_q <= wdata[29];   // WARL: only ADUE
`endif
            CSR_MSCRATCH: mscratch_q <= wdata;
            CSR_MEPC:    mepc_q <= {wdata[MXLEN-1:2], 2'b0};
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
`ifdef RV64
            // RV64: each even pmpcfg CSR carries eight cfg bytes.
            CSR_PMPCFG0: begin
                pmpcfg_q[0] <= pmp_cfg_word_wr(pmpcfg_q[0], wdata[31:0]);
                pmpcfg_q[1] <= pmp_cfg_word_wr(pmpcfg_q[1], wdata[63:32]);
            end
            CSR_PMPCFG2: begin
                pmpcfg_q[2] <= pmp_cfg_word_wr(pmpcfg_q[2], wdata[31:0]);
                pmpcfg_q[3] <= pmp_cfg_word_wr(pmpcfg_q[3], wdata[63:32]);
            end
`else
            CSR_PMPCFG0: pmpcfg_q[0] <= pmp_cfg_word_wr(pmpcfg_q[0], wdata);
            CSR_PMPCFG1: pmpcfg_q[1] <= pmp_cfg_word_wr(pmpcfg_q[1], wdata);
            CSR_PMPCFG2: pmpcfg_q[2] <= pmp_cfg_word_wr(pmpcfg_q[2], wdata);
            CSR_PMPCFG3: pmpcfg_q[3] <= pmp_cfg_word_wr(pmpcfg_q[3], wdata);
`endif
            default: begin
                if ((addr >= CSR_PMPADDR0) && (addr <= CSR_PMPADDR0 + 12'd15)) begin
                    // A locked entry (or the base of a locked TOR entry above)
                    // ignores pmpaddr writes.
                    if (!pmp_addr_locked(addr - CSR_PMPADDR0))
                        pmpaddr_q[addr - CSR_PMPADDR0] <= wdata & PMPADDR_MASK;
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
