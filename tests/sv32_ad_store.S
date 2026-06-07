    # Sv32 Svadu STORE A/D update: a store to a page with A=0 AND D=0, ADUE=1,
    # must succeed (no fault) and hardware must set BOTH A and D.
    #
    # a0=10 pass; a0=11 fail, a1 = 91 store faulted / 92 readback wrong /
    #                              93 A not set / 94 D not set.
    .section .text
    .globl main
main:
    la      t0, mtrap
    csrw    mtvec, t0
    li      s5, (1 << 17)           # MPRV

    # root[72] -> l0 table.
    li      t0, 0x10000000
    li      t1, 0x04000401
    sw      t1, (72 * 4)(t0)
    # l0[0x345] -> leaf PPN 0x10002 | V R W, A=0 D=0, U=0.
    li      s2, 0x10001000 + (0x345 * 4)
    li      t1, 0x04000807          # V R W (A=0,D=0)
    sw      t1, 0(s2)
    li      t0, 0x10002000          # data page (preset sentinel)
    li      t1, 0x11111111
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
    li      t5, 0xDEADBEEF
    sw      t5, 0(t4)               # translated STORE (needs A+D update)

    csrc    mstatus, s5             # MPRV off
    li      a1, 91
    bnez    s3, fail
    # readback the physical word
    li      t0, 0x10002000
    lw      t1, 0(t0)
    li      a1, 92
    li      t2, 0xDEADBEEF
    bne     t1, t2, fail
    # PTE must now have A and D set.
    lw      t2, 0(s2)
    li      a1, 93
    andi    t3, t2, 0x40
    beqz    t3, fail
    li      a1, 94
    andi    t3, t2, 0x80
    beqz    t3, fail
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
