    # R3: RV64A — AMO*.D, AMO*.W at both halves of an 8-byte word (offset
    # positioning + sign-extension), and LR/SC at .W and .D.
    # a0=10 pass; a0=11 fail, a1 = which check.
    .section .text
    .globl main
main:
    li      s0, 0x10000000          # scratch RAM base (bare mode: VA=PA)

    # amoswap.d: full 64-bit swap
    li      t0, 0x1122334455667788
    sd      t0, 0(s0)
    li      t1, 0xcafebabedeadbeef
    amoswap.d t2, t1, (s0)
    li      a1, 1
    bne     t2, t0, fail            # rd = old value
    ld      t3, 0(s0)
    li      a1, 2
    bne     t3, t1, fail            # memory = new value

    # amoadd.d
    li      t0, 0x100000000
    sd      t0, 8(s0)
    li      t1, 0x200000001
    addi    t4, s0, 8
    amoadd.d t2, t1, (t4)
    li      a1, 3
    bne     t2, t0, fail
    ld      t3, 8(s0)
    li      t5, 0x300000001
    li      a1, 4
    bne     t3, t5, fail

    # amoadd.w at offset 4 within an 8-byte word: only the upper half changes,
    # rd is the sign-extended old upper half
    li      t0, 0x8000000122222222
    sd      t0, 16(s0)
    li      t1, 1
    addi    t4, s0, 20              # offset 4
    amoadd.w t2, t1, (t4)
    li      t3, 0xffffffff80000001  # sext32 of old upper word
    li      a1, 5
    bne     t2, t3, fail
    ld      t3, 16(s0)
    li      t5, 0x8000000222222222  # upper half += 1, lower untouched
    li      a1, 6
    bne     t3, t5, fail

    # amomax.w signed: old = 0x80000000 (INT32_MIN) vs 1 -> 1 stored
    li      t0, 0x80000000
    sw      t0, 24(s0)
    li      t1, 1
    addi    t4, s0, 24
    amomax.w t2, t1, (t4)
    li      t3, 0xffffffff80000000
    li      a1, 7
    bne     t2, t3, fail            # rd = sext32(old)
    lw      t3, 24(s0)
    li      t5, 1
    li      a1, 8
    bne     t3, t5, fail

    # amomaxu.w unsigned: old = 0x80000000 vs 1 -> old stays
    li      t0, 0x80000000
    sw      t0, 32(s0)
    li      t1, 1
    addi    t4, s0, 32
    amomaxu.w t2, t1, (t4)
    lwu     t3, 32(s0)
    li      t5, 0x80000000
    li      a1, 9
    bne     t3, t5, fail

    # lr.w / sc.w: reserved -> success (0); repeat sc -> failure (nonzero)
    li      t0, 0x55
    sw      t0, 40(s0)
    addi    t4, s0, 40
    lr.w    t2, (t4)
    li      t3, 0x55
    li      a1, 10
    bne     t2, t3, fail
    li      t1, 0x66
    sc.w    t2, t1, (t4)
    li      a1, 11
    bne     t2, x0, fail            # sc success = 0
    lw      t3, 40(s0)
    li      t5, 0x66
    li      a1, 12
    bne     t3, t5, fail
    li      t1, 0x77
    sc.w    t2, t1, (t4)            # no reservation -> must fail
    li      a1, 13
    beq     t2, x0, fail
    lw      t3, 40(s0)
    li      t5, 0x66                # memory unchanged
    li      a1, 14
    bne     t3, t5, fail

    # lr.d / sc.d round trip
    li      t0, 0xa5a5a5a5a5a5a5a5
    sd      t0, 48(s0)
    addi    t4, s0, 48
    lr.d    t2, (t4)
    li      a1, 15
    bne     t2, t0, fail
    li      t1, 0x5a5a5a5a5a5a5a5a
    sc.d    t2, t1, (t4)
    li      a1, 16
    bne     t2, x0, fail
    ld      t3, 48(s0)
    li      a1, 17
    bne     t3, t1, fail

    li      a0, 10
    ecall
fail:
    li      a0, 11
    ecall
9:  j       9b
