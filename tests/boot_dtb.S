    # DTB-driven boot flow (Phase K): reset -> M-firmware -> S-kernel.
    #
    # The M-mode firmware (the Phase-J SBI shim) hands control to an S-mode
    # "kernel" per the RISC-V boot protocol: a0 = hartid, a1 = DTB pointer.
    # The kernel then:
    #   1. validates the FDT magic at a1 (DTB handoff),
    #   2. parses the DTB struct block for the NS16550 UART node ("ns16550a"),
    #   3. builds an Sv32 identity page table (1024 4 MiB RWX megapages) and
    #      enables paging (satp),
    #   4. under paging, re-reads the DTB magic and prints via SBI console,
    #   5. issues SBI shutdown -> halt.
    #
    # The DTB blob is embedded as the first .data object, so it loads at
    # 0x1000_0000 and a1 = DTB_ADDR is a stable absolute pointer. Build the blob
    # first:  python3 scripts/make_dtb.py output/niigo_platform.dtb
    # (also writes the .include'd output/niigo_platform.inc), then:
    #   make assemble TEST=tests/boot_dtb.S OUTPUT=output/boot
    #   (cd output/boot && ../simulation/verilator_obj/Vtop)
    #
    # a0=10 pass. a1 fail codes: 81 bad DTB magic, 82 UART node not found,
    #   83 DTB unreadable under paging, 71/72 unexpected M/S trap.
    .option arch, +zicsr
    .equ SBI_SET_TIMER, 0
    .equ SBI_PUTCHAR,   1
    .equ SBI_SHUTDOWN,  8
    .equ UART_THR,      0x0D000000
    .equ DTB_ADDR,      0x10000000
    .equ PT_ROOT,       0x10010000     # Sv32 root table (4 KiB aligned, in RAM)
    .equ FDT_MAGIC_LE,  0xedfe0dd0      # 0xd00dfeed read little-endian

    # DTB blob: first object in .data -> loaded at DTB_ADDR.
    .section .data
    .include "output/niigo_platform.inc"

    .section .text
    .globl main
main:
    # ---------------- M-mode firmware ----------------
    la      t0, m_trap_vector
    csrw    mtvec, t0
    li      t0, 0x0000B1FF
    csrw    medeleg, t0
    li      t0, (1<<9)|(1<<5)|(1<<1)
    csrw    mideleg, t0
    li      t0, 0x7
    csrw    mcounteren, t0
    li      t0, (3<<11)
    csrc    mstatus, t0
    li      t0, (1<<11)             # MPP = S
    csrs    mstatus, t0
    li      a0, 0                   # boot protocol: a0 = hartid
    li      a1, DTB_ADDR            #               a1 = dtb pointer
    la      t0, s_kernel
    csrw    mepc, t0
    mret

    # ---------------- S-mode kernel ----------------
s_kernel:
    la      t0, s_trap_vector
    csrw    stvec, t0

    # (1) validate the DTB handoff
    lw      t0, 0(a1)
    li      t1, FDT_MAGIC_LE
    bne     t0, t1, fail_magic

    # (2) parse: scan the struct block for the UART node compatible "ns16550a"
    #     (a 4-byte-aligned property value, so a word-stride scan finds it)
    li      t1, 0x3631736e         # "ns16"
    li      t2, 0x61303535         # "550a"
    mv      t3, a1
    li      t4, DTB_ADDR + 1400
1:  lw      t5, 0(t3)
    bne     t5, t1, 2f
    lw      t5, 4(t3)
    beq     t5, t2, 3f
2:  addi    t3, t3, 4
    blt     t3, t4, 1b
    li      a1, 82
    j       fail
3:                                  # UART node found in the DTB

    # (3) build the Sv32 identity map: 1024 4 MiB megapages, V|R|W|X|A|D, S-mode
    li      t0, PT_ROOT
    li      t1, 0
    li      t3, 1024
4:  slli    t2, t1, 20             # PTE.PPN[1] = VPN1 (identity megapage)
    ori     t2, t2, 0xCF           # V R W X A D
    slli    t4, t1, 2
    add     t4, t0, t4
    sw      t2, 0(t4)
    addi    t1, t1, 1
    blt     t1, t3, 4b

    # enable paging: satp = Sv32(1) | PPN(PT_ROOT>>12)
    li      t0, 0x80000000 | (PT_ROOT >> 12)
    csrw    satp, t0
    sfence.vma
    # confirm Sv32 is actually engaged (a no-op satp would leave us in bare mode,
    # where the identity map would pass spuriously)
    csrr    t1, satp
    bne     t0, t1, fail_paging

    # (4) under paging: the DTB is identity-mapped, re-read its magic
    li      t6, DTB_ADDR
    lw      t0, 0(t6)
    li      t1, FDT_MAGIC_LE
    bne     t0, t1, fail_paging

    # console: "niigo boot ok" via SBI (no 0x0a; ECALL_ARG_HALT=10)
    .macro PUTC ch
    li      a7, SBI_PUTCHAR
    li      a0, \ch
    ecall
    .endm
    PUTC 'n'; PUTC 'i'; PUTC 'i'; PUTC 'g'; PUTC 'o'; PUTC ' '
    PUTC 'b'; PUTC 'o'; PUTC 'o'; PUTC 't'; PUTC ' '
    PUTC 'o'; PUTC 'k'

    # (5) SBI shutdown -> halt
    li      a7, SBI_SHUTDOWN
    ecall

fail_magic:
    li      a1, 81
    j       fail
fail_paging:
    li      a1, 83
fail:
    li      a0, 11
    ecall
9:  j       9b

    # ---------------- S-mode trap handler ----------------
    .align 2
s_trap_vector:
    li      a1, 72
    j       fail

    # ---------------- M-mode SBI trap handler ----------------
    .align 2
m_trap_vector:
    csrr    t0, mcause
    bltz    t0, m_interrupt
    li      t1, 9                  # ecall-from-S
    bne     t0, t1, m_bad
    li      t1, SBI_PUTCHAR
    beq     a7, t1, sbi_putchar
    li      t1, SBI_SHUTDOWN
    beq     a7, t1, sbi_shutdown
    li      a0, -2
    j       sbi_ret
sbi_putchar:
    li      t1, UART_THR
    sb      a0, 0(t1)
    li      a0, 0
    j       sbi_ret
sbi_shutdown:
    li      a0, 10
    ecall
sbi_ret:
    csrr    t0, mepc
    addi    t0, t0, 4
    csrw    mepc, t0
    mret
m_interrupt:
    mret                            # no M interrupts expected in this test
m_bad:
    li      a1, 71
    li      a0, 11
    ecall
