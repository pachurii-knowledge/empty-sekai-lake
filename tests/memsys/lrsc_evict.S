    # lrsc_evict: LR a line, evict it via set-conflicting loads (no store to the
    # reserved address), then SC. The reservation lives in the LSQ, not the L1D,
    # so a plain eviction must not break it -- the SC should succeed. A second
    # round stores to the reserved address before the SC, which must then fail.
    # Checks architectural LR/SC behaviour and that nothing hangs under eviction.
    .option arch, +zicsr, +a
    .equ ADDR,  0x10000000
    .equ CONF,  0x10004000       # same set, different tag (16 KiB / 4 ways apart)
    .section .text
    .globl main
main:
    li      s0, ADDR
    li      t0, 100
    sw      t0, 0(s0)
    # ---- round 1: LR, evict (loads only), SC -> must succeed ----
    lr.w    t1, (s0)
    li      s1, CONF
    li      s2, 8
    li      s5, 0x4000           # 16 KiB / 4 ways: walks distinct tags in one set
1:  lw      t2, 0(s1)            # set-conflicting loads evict the reserved line
    add     s1, s1, s5
    addi    s2, s2, -1
    bnez    s2, 1b
    li      t3, 200
    sc.w    t4, t3, (s0)
    bnez    t4, fail             # SC must succeed (rc==0): reservation survived eviction
    lw      t5, 0(s0)
    li      t0, 200
    bne     t5, t0, fail
    # ---- round 2: LR, intervening store to ADDR, SC -> must fail ----
    lr.w    t1, (s0)
    li      t0, 300
    sw      t0, 0(s0)            # breaks the reservation
    li      t3, 400
    sc.w    t4, t3, (s0)
    beqz    t4, fail             # SC must fail (rc!=0)
    lw      t5, 0(s0)
    li      t0, 300              # memory unchanged by the failed SC
    bne     t5, t0, fail
    li      a0, 10
    ecall
fail:
    li      a0, 11
    ecall
9:  j       9b
