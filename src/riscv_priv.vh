/**
 * riscv_priv.vh
 *
 * RISC-V 32-bit Processor - Privileged architecture definitions.
 *
 * This header defines the privilege modes, CSR addresses, trap causes, and the
 * Sv32 virtual-memory field layouts used by the privileged ISA and MMU. It is
 * shared by both the scalar prototype core and the out-of-order core.
 */

`ifndef RISCV_PRIV_VH_
`define RISCV_PRIV_VH_

`include "riscv_isa.vh"     // XLEN (MXLEN == XLEN on this implementation)

package RISCV_Priv;

    localparam int MXLEN = RISCV_ISA::XLEN;

    /*------------------------------------------------------------------------
     * Privilege modes
     *----------------------------------------------------------------------*/
    typedef enum logic [1:0] {
        PRIV_U = 2'b00,
        PRIV_S = 2'b01,
        PRIV_M = 2'b11
    } priv_mode_t;

    /*------------------------------------------------------------------------
     * CSR addresses
     *----------------------------------------------------------------------*/
    // User floating-point CSRs (already implemented historically)
    localparam logic [11:0] CSR_FFLAGS    = 12'h001;
    localparam logic [11:0] CSR_FRM       = 12'h002;
    localparam logic [11:0] CSR_FCSR      = 12'h003;

    // Unprivileged counters
    localparam logic [11:0] CSR_CYCLE     = 12'hC00;
    localparam logic [11:0] CSR_TIME      = 12'hC01;
    localparam logic [11:0] CSR_INSTRET   = 12'hC02;
    localparam logic [11:0] CSR_CYCLEH    = 12'hC80;
    localparam logic [11:0] CSR_TIMEH     = 12'hC81;
    localparam logic [11:0] CSR_INSTRETH  = 12'hC82;

    // Supervisor CSRs
    localparam logic [11:0] CSR_SSTATUS    = 12'h100;
    localparam logic [11:0] CSR_SIE        = 12'h104;
    localparam logic [11:0] CSR_STVEC      = 12'h105;
    localparam logic [11:0] CSR_SCOUNTEREN = 12'h106;
    localparam logic [11:0] CSR_SENVCFG    = 12'h10A;
    localparam logic [11:0] CSR_SSCRATCH   = 12'h140;
    localparam logic [11:0] CSR_SEPC       = 12'h141;
    localparam logic [11:0] CSR_SCAUSE     = 12'h142;
    localparam logic [11:0] CSR_STVAL      = 12'h143;
    localparam logic [11:0] CSR_SIP        = 12'h144;
    localparam logic [11:0] CSR_SATP       = 12'h180;

    // Machine information registers
    localparam logic [11:0] CSR_MVENDORID  = 12'hF11;
    localparam logic [11:0] CSR_MARCHID    = 12'hF12;
    localparam logic [11:0] CSR_MIMPID     = 12'hF13;
    localparam logic [11:0] CSR_MHARTID    = 12'hF14;

    // Machine trap setup
    localparam logic [11:0] CSR_MSTATUS    = 12'h300;
    localparam logic [11:0] CSR_MISA       = 12'h301;
    localparam logic [11:0] CSR_MEDELEG    = 12'h302;
    localparam logic [11:0] CSR_MIDELEG    = 12'h303;
    localparam logic [11:0] CSR_MIE        = 12'h304;
    localparam logic [11:0] CSR_MTVEC      = 12'h305;
    localparam logic [11:0] CSR_MCOUNTEREN = 12'h306;
    localparam logic [11:0] CSR_MSTATUSH   = 12'h310;
    localparam logic [11:0] CSR_MENVCFG    = 12'h30A;
    localparam logic [11:0] CSR_MENVCFGH   = 12'h31A;

    // Machine trap handling
    localparam logic [11:0] CSR_MSCRATCH   = 12'h340;
    localparam logic [11:0] CSR_MEPC       = 12'h341;
    localparam logic [11:0] CSR_MCAUSE     = 12'h342;
    localparam logic [11:0] CSR_MTVAL      = 12'h343;
    localparam logic [11:0] CSR_MIP        = 12'h344;

    // Physical memory protection
    localparam logic [11:0] CSR_PMPCFG0    = 12'h3A0;
    localparam logic [11:0] CSR_PMPCFG1    = 12'h3A1;
    localparam logic [11:0] CSR_PMPCFG2    = 12'h3A2;
    localparam logic [11:0] CSR_PMPCFG3    = 12'h3A3;
    localparam logic [11:0] CSR_PMPADDR0   = 12'h3B0;  // .. 0x3BF (16 entries)

    // Machine counters
    localparam logic [11:0] CSR_MCYCLE     = 12'hB00;
    localparam logic [11:0] CSR_MINSTRET   = 12'hB02;
    localparam logic [11:0] CSR_MCYCLEH    = 12'hB80;
    localparam logic [11:0] CSR_MINSTRETH  = 12'hB82;

    // Hardware performance-monitor CSRs (modelled as WARL read-zero). Bounds of
    // the programmable counter / event-selector ranges used for decode.
    localparam logic [11:0] CSR_MCOUNTINHIBIT  = 12'h320;
    localparam logic [11:0] CSR_MHPMEVENT3     = 12'h323;  // .. 0x33F
    localparam logic [11:0] CSR_MHPMEVENT31    = 12'h33F;
    localparam logic [11:0] CSR_MHPMEVENT3H    = 12'h723;  // .. 0x73F
    localparam logic [11:0] CSR_MHPMEVENT31H   = 12'h73F;
    localparam logic [11:0] CSR_MHPMCOUNTER3   = 12'hB03;  // .. 0xB1F
    localparam logic [11:0] CSR_MHPMCOUNTER31  = 12'hB1F;
    localparam logic [11:0] CSR_MHPMCOUNTER3H  = 12'hB83;  // .. 0xB9F
    localparam logic [11:0] CSR_MHPMCOUNTER31H = 12'hB9F;
    localparam logic [11:0] CSR_HPMCOUNTER3    = 12'hC03;  // .. 0xC1F
    localparam logic [11:0] CSR_HPMCOUNTER31   = 12'hC1F;
    localparam logic [11:0] CSR_HPMCOUNTER3H   = 12'hC83;  // .. 0xC9F
    localparam logic [11:0] CSR_HPMCOUNTER31H  = 12'hC9F;

    /*------------------------------------------------------------------------
     * Trap causes (mcause / scause). Interrupt bit is the MSB of the XLEN word.
     *----------------------------------------------------------------------*/
    localparam logic [4:0] EXC_INSTR_MISALIGNED = 5'd0;
    localparam logic [4:0] EXC_INSTR_ACCESS     = 5'd1;
    localparam logic [4:0] EXC_ILLEGAL_INSTR    = 5'd2;
    localparam logic [4:0] EXC_BREAKPOINT       = 5'd3;
    localparam logic [4:0] EXC_LOAD_MISALIGNED  = 5'd4;
    localparam logic [4:0] EXC_LOAD_ACCESS      = 5'd5;
    localparam logic [4:0] EXC_STORE_MISALIGNED = 5'd6;
    localparam logic [4:0] EXC_STORE_ACCESS     = 5'd7;
    localparam logic [4:0] EXC_ECALL_U          = 5'd8;
    localparam logic [4:0] EXC_ECALL_S          = 5'd9;
    localparam logic [4:0] EXC_ECALL_M          = 5'd11;
    localparam logic [4:0] EXC_INSTR_PAGE_FAULT = 5'd12;
    localparam logic [4:0] EXC_LOAD_PAGE_FAULT  = 5'd13;
    localparam logic [4:0] EXC_STORE_PAGE_FAULT = 5'd15;

    // Interrupt cause codes (low bits; mcause MSB set)
    localparam logic [4:0] INT_S_SOFTWARE = 5'd1;
    localparam logic [4:0] INT_M_SOFTWARE = 5'd3;
    localparam logic [4:0] INT_S_TIMER    = 5'd5;
    localparam logic [4:0] INT_M_TIMER    = 5'd7;
    localparam logic [4:0] INT_S_EXTERNAL = 5'd9;
    localparam logic [4:0] INT_M_EXTERNAL = 5'd11;

    /*------------------------------------------------------------------------
     * mstatus / sstatus bit positions (RV32)
     *----------------------------------------------------------------------*/
    localparam int MSTATUS_SIE_BIT  = 1;
    localparam int MSTATUS_MIE_BIT  = 3;
    localparam int MSTATUS_SPIE_BIT = 5;
    localparam int MSTATUS_MPIE_BIT = 7;
    localparam int MSTATUS_SPP_BIT  = 8;
    localparam int MSTATUS_MPP_LO   = 11;  // [12:11]
    localparam int MSTATUS_FS_LO    = 13;  // [14:13]
    localparam int MSTATUS_MPRV_BIT = 17;
    localparam int MSTATUS_SUM_BIT  = 18;
    localparam int MSTATUS_MXR_BIT  = 19;
    localparam int MSTATUS_TVM_BIT  = 20;
    localparam int MSTATUS_TW_BIT   = 21;
    localparam int MSTATUS_TSR_BIT  = 22;
    // RV64-only two-bit fields (read-only 2 = 64-bit on this implementation)
    localparam int MSTATUS_UXL_LO   = 32;  // [33:32]
    localparam int MSTATUS_SXL_LO   = 34;  // [35:34]
    localparam int MSTATUS_SD_BIT   = MXLEN - 1;

    // sstatus is a restricted view of mstatus; this mask exposes the S-visible
    // fields (SIE, SPIE, SPP, FS, XS, SUM, MXR, SD — plus UXL on RV64).
    localparam logic [MXLEN-1:0] SSTATUS_MASK = (MXLEN == 64) ?
        MXLEN'(64'h8000_0003_000D_E122) : MXLEN'(32'h800D_E122);

    /*------------------------------------------------------------------------
     * Sv32 virtual memory
     *----------------------------------------------------------------------*/
    localparam logic [0:0] SATP_MODE_BARE = 1'b0;
    localparam logic [0:0] SATP_MODE_SV32 = 1'b1;
    localparam int SV32_LEVELS  = 2;
    localparam int SV32_PGSHIFT = 12;
    localparam int SV32_PTESIZE = 4;

    // PTE bit positions
    localparam int PTE_V = 0;
    localparam int PTE_R = 1;
    localparam int PTE_W = 2;
    localparam int PTE_X = 3;
    localparam int PTE_U = 4;
    localparam int PTE_G = 5;
    localparam int PTE_A = 6;
    localparam int PTE_D = 7;

endpackage: RISCV_Priv

`endif /* RISCV_PRIV_VH_ */
