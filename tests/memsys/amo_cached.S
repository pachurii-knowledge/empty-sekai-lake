    # amo_cached: AMO read-modify-write through the L1D under eviction pressure.
    # Four accumulators sit in distinct cache sets; between AMOs a strided sweep
    # over a >16 KiB region evicts lines, so each amoadd reloads its line, adds,
    # and writes it back. Final accumulator values check the load+compute+commit
    # path survives eviction.
    .option arch, +zicsr, +a
    .equ BASE, 0x10000000        # accumulators
    .equ SWEEP, 0x10010000       # eviction-pressure region (separate)
    .section .text
    .globl main
main:
    li      s0, BASE
    sw      x0, 0(s0)
    sw      x0, 256(s0)          # +4 lines
    sw      x0, 512(s0)
    sw      x0, 768(s0)
    li      s1, 0                # iteration
    li      s2, 64               # iterations
loop:
    li      t3, 1
    amoadd.w x0, t3, (s0)        # acc0 += 1
    addi    t4, s0, 256
    amoadd.w x0, t3, (t4)        # acc1 += 1
    addi    t4, s0, 512
    li      t5, 2
    amoadd.w x0, t5, (t4)        # acc2 += 2
    addi    t4, s0, 768
    amoadd.w x0, t5, (t4)        # acc3 += 2
    # eviction sweep: touch a fresh line each iteration
    li      t0, SWEEP
    slli    t1, s1, 6            # 64 B stride
    add     t0, t0, t1
    lw      t2, 0(t0)
    addi    s1, s1, 1
    blt     s1, s2, loop
    # check: acc0=acc1=64, acc2=acc3=128
    lw      t0, 0(s0)
    li      t1, 64
    bne     t0, t1, fail
    lw      t0, 256(s0)
    bne     t0, t1, fail
    lw      t0, 512(s0)
    li      t1, 128
    bne     t0, t1, fail
    lw      t0, 768(s0)
    bne     t0, t1, fail
    li      a0, 10
    ecall
fail:
    li      a0, 11
    ecall
9:  j       9b
