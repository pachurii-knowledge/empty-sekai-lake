    # Sv32 A/D update across a store THEN a load to different pages (Svadu repro).
    # Both leaf PTEs start A=0, D=1 (0x..87), ADUE=1. After a translated store to
    # page W and a translated load from page R, BOTH PTEs must have A set.
    #
    #   VA_w 0x12345000 -> PA 0x10002000 (l0[0x345] @ 0x10001D14)
    #   VA_r 0x12346000 -> PA 0x10003000 (l0[0x346] @ 0x10001D18)
    #
    # a0=10 pass; a0=11 fail, a1 = 91 store PTE A unset / 92 load PTE A unset /
    #                              93 store/load faulted unexpectedly.
    .section .text
    .globl main
main:
    la      t0, mtrap
    csrw    mtvec, t0
    li      s5, (1 << 17)           # MPRV

    li      t0, 0x10000000
    li      t1, 0x04000401          # root[0x48] -> l0 (PPN 0x10001)
    sw      t1, (0x48 * 4)(t0)

    li      s6, 0x10001D14          # &l0[0x345]  (store page)
    li      t1, 0x04000887          # PPN 0x10002 | V R W | D, A=0
    sw      t1, 0(s6)
    li      s7, 0x10001D18          # &l0[0x346]  (load page)
    li      t1, 0x04000C87          # PPN 0x10003 | V R W | D, A=0
    sw      t1, 0(s7)

    li      t0, 0x10003000          # preload the read page with a sentinel
    li      t1, 0xABCDEF01
    sw      t1, 0(t0)

    li      t0, 0x80010000
    csrw    satp, t0
    li      t0, (1 << 29)
    csrs    menvcfgh, t0            # ADUE = 1
    sfence.vma

    li      t0, (3 << 11)
    csrc    mstatus, t0
    li      t0, (1 << 11)
    csrs    mstatus, t0             # MPP = S
    csrs    mstatus, s5             # MPRV

    li      s3, 0
    li      t4, 0x12345000
    li      t5, 0x5A5A5A5A
    sw      t5, 0(t4)               # translated STORE -> A/D update of PTE_w
    li      t4, 0x12346000
    lw      s0, 0(t4)               # translated LOAD  -> A update of PTE_r

    csrc    mstatus, s5             # MPRV off
    li      a1, 93
    bnez    s3, fail
    # both PTEs must now have A (0x40) set
    lw      t2, 0(s6)
    andi    t2, t2, 0x40
    li      a1, 91
    beqz    t2, fail
    lw      t2, 0(s7)
    andi    t2, t2, 0x40
    li      a1, 92
    beqz    t2, fail
    li      a0, 10
    ecall
fail:
    li      a0, 11
    ecall

    .align 2
mtrap:
    csrr    s4, mcause
    li      s3, 1
    csrr    t6, mepc
    addi    t6, t6, 4
    csrw    mepc, t6
    mret
