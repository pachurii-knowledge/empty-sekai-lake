/*
 * niigo_host.c  (FB1) -- host runtime for the niigo SoC on an AWS F2 instance.
 *
 * Brings the core up and runs the console over the vUART:
 *   1. attach the OCL AXI-Lite (AppPF BAR0) and sanity-check BUILD_ID / XLEN;
 *   2. hold the core in reset (CTRL.go=0), preload the kernel (+ optional fs
 *      image) into card DRAM over DMA_PCIS -- safe because the core is in reset
 *      (the plan's "host loads memory only while the core is in reset" rule);
 *   3. release reset (CTRL.go=1) and run an interactive console loop: drain the
 *      vUART TX FIFO to stdout, forward stdin keystrokes into the RX FIFO;
 *   4. on demand (--debug, or Ctrl-\ in the console) dump the debug block:
 *      cycle/instret counters, committed-PC ring, last trap, shadow registers.
 *
 * Build: see fpga/runtime/Makefile (needs the aws-fpga SDK: fpga_pci, fpga_mgmt,
 * fpga_dma). This is the FB1 scaffolding; it runs on the F2 instance after the
 * AFI is loaded (FB2-HDK build + fpga-load-local-image), not in this repo's sim.
 */
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <signal.h>

#include <fpga_pci.h>
#include <fpga_mgmt.h>
#include <fpga_dma.h>
#include <utils/lcd.h>

#include "niigo_ocl.h"

static pci_bar_handle_t ocl = PCI_BAR_HANDLE_INIT;
static int dma_fd = -1;
static int slot = 0;
static volatile int stop = 0;

static void die(const char *msg) { fprintf(stderr, "FATAL: %s (errno=%d)\n", msg, errno); exit(1); }

/* --- OCL 32-bit register access --- */
static uint32_t ocl_rd(uint32_t off) {
    uint32_t v;
    if (fpga_pci_peek(ocl, off, &v)) die("fpga_pci_peek");
    return v;
}
static void ocl_wr(uint32_t off, uint32_t v) {
    if (fpga_pci_poke(ocl, off, v)) die("fpga_pci_poke");
}
static uint64_t ocl_rd64(uint32_t lo, uint32_t hi) {
    uint32_t l = ocl_rd(lo), h = ocl_rd(hi);
    return ((uint64_t)h << 32) | l;
}

/* --- bring-up --- */
static void niigo_attach(void) {
    if (fpga_mgmt_init()) die("fpga_mgmt_init");
    if (fpga_pci_attach(slot, FPGA_APP_PF, APP_PF_BAR0, 0, &ocl)) die("fpga_pci_attach OCL");
    uint32_t id = ocl_rd(OCL_BUILD_ID);
    if (id != NIIGO_BUILD_ID) {
        fprintf(stderr, "FATAL: bad BUILD_ID 0x%08x (expected 0x%08x) -- wrong AFI?\n",
                id, NIIGO_BUILD_ID);
        exit(1);
    }
    uint32_t xi = ocl_rd(OCL_XLEN_INFO);
    printf("niigo: attached slot %d, XLEN=%u%s\n", slot, xi & 0xFF,
           (xi & 0x100) ? " (paging)" : "");
}

/* DMA a file into card DRAM at `addr` while the core is held in reset. */
static void niigo_preload(const char *path, uint64_t addr) {
    int f = open(path, O_RDONLY);
    if (f < 0) die("open image");
    off_t sz = lseek(f, 0, SEEK_END); lseek(f, 0, SEEK_SET);
    uint8_t *buf = malloc(sz);
    if (!buf || read(f, buf, sz) != sz) die("read image");
    close(f);

    if (dma_fd < 0) {
        dma_fd = fpga_dma_open_queue(FPGA_DMA_XDMA, slot, /*channel*/0, /*is_read*/false);
        if (dma_fd < 0) die("fpga_dma_open_queue");
    }
    /* Burst into DRAM. The XDMA path handles arbitrary sizes; chunk for safety. */
    const size_t CHUNK = 4u << 20;
    for (off_t off = 0; off < sz; off += CHUNK) {
        size_t n = (sz - off < (off_t)CHUNK) ? (size_t)(sz - off) : CHUNK;
        if (fpga_dma_burst_write(dma_fd, buf + off, n, addr + off)) die("fpga_dma_burst_write");
    }
    free(buf);
    printf("niigo: preloaded %s (%ld bytes) -> DRAM 0x%llx\n", path, (long)sz,
           (unsigned long long)addr);
}

/* --- debug observability dump --- */
static void niigo_dump_debug(void) {
    uint32_t st = ocl_rd(OCL_STATUS);
    printf("\n==== niigo debug ====\n");
    printf("STATUS    : 0x%08x  halted=%d in_reset=%d trap_seen=%d\n", st,
           !!(st & STATUS_HALTED), !!(st & STATUS_IN_RESET), !!(st & STATUS_TRAP_SEEN));
    printf("CYCLE     : %llu\n", (unsigned long long)ocl_rd64(OCL_CYCLE_LO, OCL_CYCLE_HI));
    printf("INSTRET   : %llu\n", (unsigned long long)ocl_rd64(OCL_INSTRET_LO, OCL_INSTRET_HI));
    printf("L1I miss  : %u\nL1D miss  : %u\nL1D wb    : %u\n",
           ocl_rd(OCL_HPM_L1I_MISS), ocl_rd(OCL_HPM_L1D_MISS), ocl_rd(OCL_HPM_L1D_WB));

    uint32_t tc = ocl_rd(OCL_DBG_TRAP_CNT);
    uint32_t tcause = ocl_rd(OCL_DBG_TRAP_CAUSE);
    printf("traps     : %u  last: cause=%u %s epc=0x%llx tval=0x%llx\n", tc,
           tcause & 0x1F, (tcause & TRAP_IS_INT) ? "(int)" : "(exc)",
           (unsigned long long)ocl_rd64(OCL_DBG_EPC_LO, OCL_DBG_EPC_HI),
           (unsigned long long)ocl_rd64(OCL_DBG_TVAL_LO, OCL_DBG_TVAL_HI));

    /* committed-PC ring: print head-1 (most recent) backwards. */
    uint32_t head = (ocl_rd(OCL_DBG_PCIDX) >> 8) & 0xF;
    printf("committed PCs (most recent first):\n");
    for (int i = 0; i < PC_RING_DEPTH; i++) {
        uint32_t idx = (head - 1 - i) & 0xF;
        ocl_wr(OCL_DBG_PCIDX, idx);
        printf("  [%2d] 0x%llx\n", idx,
               (unsigned long long)ocl_rd64(OCL_DBG_PC_LO, OCL_DBG_PC_HI));
    }

    /* shadow architectural regfile. */
    static const char *abi[32] = {
        "zero","ra","sp","gp","tp","t0","t1","t2","s0","s1","a0","a1","a2","a3",
        "a4","a5","a6","a7","s2","s3","s4","s5","s6","s7","s8","s9","s10","s11",
        "t3","t4","t5","t6"};
    printf("arch regs:\n");
    for (int r = 0; r < 32; r++) {
        ocl_wr(OCL_DBG_REGSEL, r);
        printf("  x%-2d %-4s = 0x%llx\n", r, abi[r],
               (unsigned long long)ocl_rd64(OCL_DBG_REG_LO, OCL_DBG_REG_HI));
    }
    printf("=====================\n\n");
}

/* --- interactive vUART console --- */
static struct termios saved_tio;
static void restore_tty(void) { tcsetattr(STDIN_FILENO, TCSANOW, &saved_tio); }
static void on_sigint(int s) { (void)s; stop = 1; }

static void niigo_console(void) {
    struct termios raw;
    tcgetattr(STDIN_FILENO, &saved_tio);
    atexit(restore_tty);
    raw = saved_tio;
    cfmakeraw(&raw);
    tcsetattr(STDIN_FILENO, TCSANOW, &raw);
    fcntl(STDIN_FILENO, F_SETFL, O_NONBLOCK);
    signal(SIGINT, on_sigint);

    fprintf(stderr, "[niigo console -- Ctrl-] quits, Ctrl-\\ dumps debug]\r\n");
    while (!stop) {
        int did = 0;
        /* drain TX FIFO (core -> host) */
        for (int i = 0; i < 256; i++) {
            uint32_t tx = ocl_rd(OCL_UART_TX);
            if (!(tx & UART_TX_VALID)) break;
            putchar(tx & 0xFF); did = 1;
        }
        if (did) fflush(stdout);
        /* forward stdin (host -> core), unless the RX FIFO is full */
        unsigned char c;
        while (read(STDIN_FILENO, &c, 1) == 1) {
            if (c == 0x1D) { stop = 1; break; }          /* Ctrl-] */
            if (c == 0x1C) { niigo_dump_debug(); break; } /* Ctrl-\ */
            if (!(ocl_rd(OCL_UART_RX_ST) & FIFO_ST_FULL))
                ocl_wr(OCL_UART_RX, c);
        }
        if (!did) usleep(200);   /* idle backoff */
    }
    restore_tty();
    fprintf(stderr, "\r\n[niigo console closed]\r\n");
}

static void usage(const char *p) {
    fprintf(stderr,
        "usage: %s --kernel <img> [--fs <img>] [--slot N] [--debug] [--no-console]\n"
        "  --kernel <img>  raw memory image loaded at 0x%llx\n"
        "  --fs <img>      optional disk/fs blob loaded at 0x%llx\n"
        "  --debug         dump the debug block after bring-up and on exit\n",
        p, (unsigned long long)NIIGO_RAM_BASE, (unsigned long long)NIIGO_DISK_BASE);
    exit(2);
}

int main(int argc, char **argv) {
    const char *kernel = NULL, *fs = NULL;
    int do_debug = 0, do_console = 1;
    for (int i = 1; i < argc; i++) {
        if      (!strcmp(argv[i], "--kernel") && i+1 < argc) kernel = argv[++i];
        else if (!strcmp(argv[i], "--fs") && i+1 < argc)     fs = argv[++i];
        else if (!strcmp(argv[i], "--slot") && i+1 < argc)   slot = atoi(argv[++i]);
        else if (!strcmp(argv[i], "--debug"))                do_debug = 1;
        else if (!strcmp(argv[i], "--no-console"))           do_console = 0;
        else usage(argv[0]);
    }
    if (!kernel) usage(argv[0]);

    niigo_attach();

    /* hold core in reset, clear counters, preload, then release. */
    ocl_wr(OCL_CTRL, CTRL_SOFT_RESET);            /* go=0, soft_reset=1 */
    niigo_preload(kernel, NIIGO_RAM_BASE);
    if (fs) niigo_preload(fs, NIIGO_DISK_BASE);
    ocl_wr(OCL_CTRL, CTRL_CLEAR_CNT);             /* zero counters (still in reset via go=0) */
    printf("niigo: releasing reset (go=1)\n");
    ocl_wr(OCL_CTRL, CTRL_GO);                     /* go=1, soft_reset=0 -> run */

    if (do_console) niigo_console();
    if (do_debug)   niigo_dump_debug();

    if (dma_fd >= 0) close(dma_fd);
    fpga_pci_detach(ocl);
    return 0;
}
