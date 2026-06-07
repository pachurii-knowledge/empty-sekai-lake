    # Sv32 Svadu A/D-update on a 4 MiB SUPERPAGE (level-1 leaf).
    #
    # Mirrors the official Svadu case: a level-1 leaf PTE with A=0, ADUE=1, then
    # a translated load must succeed (no fault) and hardware must set A.
    #
    #   root[72] = leaf PPN 0x10000 | V R W  (A=0, D=0)  -> 4MB superpage
    #   VA 0x12002000 -> PA 0x10002000 (within the data window)
    #
    # a0=10 pass; a0=11 fail, a1 = 81 load faulted / 82 wrong value / 83 A not set.
    .section .text
    .globl main
main:
    la      t0, mtrap
    csrw    mtvec, t0
    li      s5, (1 << 17)           # MPRV mask

    # root[72] -> level-1 leaf (superpage), PPN 0x10000 | V R W, A=0 D=0, U=0.
    li      s2, 0x10000000 + (72 * 4)
    li      t1, 0x04000007          # PPN(0x10000)<<10 | V(1) R(2) W(4)
    sw      t1, 0(s2)

    li      t0, 0x10002000          # data within the superpage
    li      t1, 0xCAFEBABE
    sw      t1, 0(t0)

    li      t0, 0x80010000          # satp MODE=Sv32, root PPN 0x10000
    csrw    satp, t0

    li      t0, (1 << 29)
    csrs    menvcfgh, t0            # ADUE = 1 (Svadu)
    sfence.vma

    # MPP=S, MPRV on
    li      t0, (3 << 11)
    csrc    mstatus, t0
    li      t0, (1 << 11)
    csrs    mstatus, t0
    csrs    mstatus, s5

    li      s3, 0
    li      t4, 0x12002000
    li      t6, 0x5A5A5A5A
    sw      t6, 0(t4)               # translated STORE through the superpage

    csrc    mstatus, s5             # MPRV off
    li      a1, 81
    bnez    s3, fail                # must not have faulted
    li      a1, 82
    li      t0, 0x10002000
    lw      s0, 0(t0)               # physical readback of the stored value
    li      s1, 0x5A5A5A5A
    bne     s0, s1, fail
    # Both A and D must now be set in the level-1 superpage PTE.
    lw      t2, 0(s2)
    li      a1, 83
    andi    t3, t2, 0x40            # A
    beqz    t3, fail
    li      a1, 84
    andi    t3, t2, 0x80            # D
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
