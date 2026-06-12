    # hpm_cache: sanity-check the phase-C3 cache event counters. mhpmcounter4 is
    # L1D misses. Read it, run 64 loads to 64 distinct cache lines (64 cold
    # misses), read it again -- the delta must be >= 50 (loose, allowing slop).
    # Then set mcountinhibit[4] and confirm the counter freezes.
    .option arch, +zicsr
    .equ BASE, 0x10000000
    .section .text
    .globl main
main:
    csrw    0x320, x0           # mcountinhibit = 0 (all counters running)
    csrr    s0, 0xb04           # mhpmcounter4 (L1D miss) before
    li      t0, BASE
    li      t1, 64
1:  lw      t2, 0(t0)
    addi    t0, t0, 64          # next 64 B line
    addi    t1, t1, -1
    bnez    t1, 1b
    csrr    s1, 0xb04           # after
    sub     s2, s1, s0
    li      t3, 50
    blt     s2, t3, fail        # expected ~64 misses

    # ---- inhibit HPM4 and confirm it freezes ----
    li      t0, (1 << 4)
    csrw    0x320, t0           # mcountinhibit.HPM4 = 1
    csrr    s3, 0xb04
    li      t0, BASE
    li      t1, 64
2:  lw      t2, 0(t0)
    addi    t0, t0, 64
    addi    t1, t1, -1
    bnez    t1, 2b
    csrr    s4, 0xb04
    bne     s3, s4, fail        # inhibited: must be unchanged

    li      a0, 10
    ecall
fail:
    li      a0, 11
    ecall
9:  j       9b
