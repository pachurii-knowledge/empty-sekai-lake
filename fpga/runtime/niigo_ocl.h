/*
 * niigo_ocl.h  (FB1) -- OCL control-plane register map for the niigo SoC on
 * AWS F2. Mirrors fpga/rtl/ocl_csr.sv exactly; keep the two in sync.
 *
 * All registers are 32-bit, accessed over the AppPF BAR0 AXI4-Lite (the shell
 * OCL interface). XLEN-wide values (PC/EPC/TVAL/regs) are split LO/HI; HI reads
 * 0 on an RV32 build.
 */
#ifndef NIIGO_OCL_H
#define NIIGO_OCL_H

/* --- control / status --- */
#define OCL_CTRL          0x00  /* RW  [0]go(level) [1]soft_reset(level) [2]clear_counters(W1P) */
#define   CTRL_GO            (1u << 0)
#define   CTRL_SOFT_RESET    (1u << 1)
#define   CTRL_CLEAR_CNT     (1u << 2)
#define OCL_STATUS        0x04  /* RO  [0]halted [1]in_reset [2]tx_empty [3]rx_full [4]trap_seen */
#define   STATUS_HALTED      (1u << 0)
#define   STATUS_IN_RESET    (1u << 1)
#define   STATUS_TX_EMPTY    (1u << 2)
#define   STATUS_RX_FULL     (1u << 3)
#define   STATUS_TRAP_SEEN   (1u << 4)
#define OCL_BUILD_ID      0x08  /* RO  0x4E494731 = "NIG1" */
#define   NIIGO_BUILD_ID     0x4E494731u
#define OCL_XLEN_INFO     0x0C  /* RO  [7:0]=XLEN [8]=paging */

/* --- free-running counters --- */
#define OCL_CYCLE_LO      0x10
#define OCL_CYCLE_HI      0x14
#define OCL_INSTRET_LO    0x18
#define OCL_INSTRET_HI    0x1C

/* --- vUART console --- */
#define OCL_UART_TX       0x20  /* RO  read pops TX FIFO: [7:0]=byte [8]=valid */
#define   UART_TX_VALID      (1u << 8)
#define OCL_UART_TX_ST    0x24  /* RO  [15:0]=count [16]=empty [17]=full */
#define OCL_UART_RX       0x28  /* WO  write [7:0]=byte -> push RX FIFO */
#define OCL_UART_RX_ST    0x2C  /* RO  [15:0]=count [16]=empty [17]=full */
#define   FIFO_ST_EMPTY      (1u << 16)
#define   FIFO_ST_FULL       (1u << 17)

/* --- L1 HPM event counters --- */
#define OCL_HPM_L1I_MISS  0x30
#define OCL_HPM_L1D_MISS  0x34
#define OCL_HPM_L1D_WB    0x38

/* --- debug: committed-PC ring --- */
#define OCL_DBG_PCIDX     0x40  /* RW  [3:0]=read index ; RO [11:8]=ring head */
#define OCL_DBG_PC_LO     0x44
#define OCL_DBG_PC_HI     0x48
#define   PC_RING_DEPTH      16

/* --- debug: trap log --- */
#define OCL_DBG_TRAP_CNT  0x50
#define OCL_DBG_TRAP_CAUSE 0x54 /* RO  [4:0]=cause [8]=is_int */
#define   TRAP_IS_INT        (1u << 8)
#define OCL_DBG_EPC_LO    0x58
#define OCL_DBG_EPC_HI    0x5C
#define OCL_DBG_TVAL_LO   0x60
#define OCL_DBG_TVAL_HI   0x64

/* --- debug: shadow architectural regfile --- */
#define OCL_DBG_REGSEL    0x80  /* RW  [4:0]=arch reg index 0..31 */
#define OCL_DBG_REG_LO    0x84
#define OCL_DBG_REG_HI    0x88

/* Default card-DRAM layout (PA identity-mapped; matches the SoC + xv6 image). */
#define NIIGO_RAM_BASE    0x80000000ULL   /* kernel load address */
#define NIIGO_DISK_BASE   0x90000000ULL   /* fs.img blob (xv6 virtio-less disk) */

#endif /* NIIGO_OCL_H */
