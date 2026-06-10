    # R6: RV64D FP. FMV.X.D/FMV.D.X (64-bit moves), FCVT.L/LU round trips,
    # FCVT.W sign-extension, FLD/FSD, and FSW/FLW at offset 4 of an 8-byte
    # memory word (NaN-box + byte positioning).
    # a0=10 pass; a0=11 fail, a1 = which check.
    .section .text
    .globl main
main:
    li      s0, 0x10000000          # scratch RAM base

    # 1: FMV.D.X -> FMV.X.D round trip (9.8)
    li      t0, 0x402399999999999a
    fmv.d.x f0, t0
    fmv.x.d t1, f0
    li      a1, 1
    bne     t1, t0, fail

    # 2: FCVT.L.D rtz of 9.8 = 9
    fcvt.l.d t2, f0, rtz
    li      t3, 9
    li      a1, 2
    bne     t2, t3, fail

    # 3: FCVT.D.L of -5, back via FCVT.L.D = -5
    li      t0, -5
    fcvt.d.l f1, t0
    fcvt.l.d t2, f1, rtz
    li      t3, -5
    li      a1, 3
    bne     t2, t3, fail

    # 4: FCVT.LU.D of 3.0e9 (> 2^31) = 3000000000
    li      t0, 0x41e65a0bc0000000
    fmv.d.x f2, t0
    fcvt.lu.d t2, f2, rtz
    li      t3, 3000000000
    li      a1, 4
    bne     t2, t3, fail

    # 5: FCVT.W.D of -9.8 = -9, sign-extended to 64 bits
    li      t0, 0xc02399999999999a
    fmv.d.x f3, t0
    fcvt.w.d t2, f3, rtz
    li      t3, -9
    li      a1, 5
    bne     t2, t3, fail

    # 6: FSD / FLD round trip through memory
    fsd     f0, 8(s0)
    fld     f4, 8(s0)
    fmv.x.d t1, f4
    li      t0, 0x402399999999999a
    li      a1, 6
    bne     t1, t0, fail
    ld      t1, 8(s0)                # raw memory holds the double bits
    li      a1, 7
    bne     t1, t0, fail

    # 8: FSW/FLW at offset 4 within an 8-byte word
    li      t0, 0x1111111122222222
    sd      t0, 16(s0)
    li      t0, 0x3fc00000           # 1.5f
    fmv.w.x f5, t0
    fsw     f5, 20(s0)               # upper half of the word at 16(s0)
    flw     f6, 20(s0)
    fmv.x.w t1, f6
    li      t3, 0x3fc00000
    li      a1, 8
    bne     t1, t3, fail
    ld      t1, 16(s0)               # word = {1.5f bits, untouched low half}
    li      t3, 0x3fc0000022222222
    li      a1, 9
    bne     t1, t3, fail

    # 10: FCVT.S.LU of 2^40 -> 0x53800000; FMV.X.W sign-extends (positive here)
    li      t0, 1
    slli    t0, t0, 40
    fcvt.s.lu f7, t0
    fmv.x.w t1, f7
    li      t3, 0x53800000
    li      a1, 10
    bne     t1, t3, fail

    li      a0, 10
    ecall
fail:
    li      a0, 11
    ecall
9:  j       9b
