    # R3: RV64M — 64-bit MUL/MULH*/DIV/REM + the W-forms MULW/DIVW/DIVUW/
    # REMW/REMUW, including div-by-zero and overflow specials.
    # a0=10 pass; a0=11 fail, a1 = which check.
    .section .text
    .globl main
main:
    # MUL 64-bit: (2^32+1)^2 mod 2^64 = 2^33 + 1
    li      t0, 0x100000001
    mul     t2, t0, t0
    li      t3, 0x200000001
    li      a1, 1
    bne     t2, t3, fail

    # MULH: -1 * -1 -> product 1, high = 0
    li      t0, -1
    mulh    t2, t0, t0
    li      a1, 2
    bne     t2, x0, fail

    # MULHU: (2^64-1)^2 -> high = 2^64 - 2
    li      t0, -1
    mulhu   t2, t0, t0
    li      t3, -2
    li      a1, 3
    bne     t2, t3, fail

    # MULHSU: -1 (signed) * (2^64-1) (unsigned) -> high = all-ones
    li      t0, -1
    mulhsu  t2, t0, t0
    li      t3, -1
    li      a1, 4
    bne     t2, t3, fail

    # MULW: ignores upper operand bits; sign-extends bit 31
    li      t0, 0x100000007         # low 32 = 7
    li      t1, 3
    mulw    t2, t0, t1              # 21
    li      t3, 21
    li      a1, 5
    bne     t2, t3, fail
    li      t0, 0x7fffffff
    li      t1, 2
    mulw    t2, t0, t1              # 0xfffffffe -> sext
    li      t3, 0xfffffffffffffffe
    li      a1, 6
    bne     t2, t3, fail

    # DIV/REM signed 64-bit: -20 / 3 = -6 rem -2
    li      t0, -20
    li      t1, 3
    div     t2, t0, t1
    li      t3, -6
    li      a1, 7
    bne     t2, t3, fail
    rem     t2, t0, t1
    li      t3, -2
    li      a1, 8
    bne     t2, t3, fail

    # DIVU 64-bit: (2^64-1) / 2 = 2^63 - 1
    li      t0, -1
    li      t1, 2
    divu    t2, t0, t1
    li      t3, 0x7fffffffffffffff
    li      a1, 9
    bne     t2, t3, fail

    # INT64_MIN / -1 overflow: quotient INT64_MIN, remainder 0
    li      t0, 0x8000000000000000
    li      t1, -1
    div     t2, t0, t1
    li      a1, 10
    bne     t2, t0, fail
    rem     t2, t0, t1
    li      a1, 11
    bne     t2, x0, fail

    # div by zero: quotient all-ones, remainder = dividend
    li      t0, 42
    div     t2, t0, x0
    li      t3, -1
    li      a1, 12
    bne     t2, t3, fail
    rem     t2, t0, x0
    li      t3, 42
    li      a1, 13
    bne     t2, t3, fail

    # DIVW: -20 / 3 = -6 (32-bit, ignores upper operand bits)
    li      t0, 0xdeadbeefffffffec  # low 32 = -20
    li      t1, 3
    divw    t2, t0, t1
    li      t3, -6
    li      a1, 14
    bne     t2, t3, fail

    # DIVW INT32_MIN / -1 = INT32_MIN sign-extended; REMW -> 0
    li      t0, 0x80000000
    li      t1, -1
    divw    t2, t0, t1
    li      t3, 0xffffffff80000000
    li      a1, 15
    bne     t2, t3, fail
    remw    t2, t0, t1
    li      a1, 16
    bne     t2, x0, fail

    # DIVUW: zero-extended 32-bit operands; result sign-extended from bit 31
    li      t0, 0xffffffff          # 2^32 - 1 unsigned
    li      t1, 1
    divuw   t2, t0, t1              # 0xffffffff -> sext = all-ones
    li      t3, -1
    li      a1, 17
    bne     t2, t3, fail

    # REMUW: zext32(0x80000005) mod 4 = 1
    li      t0, 0x80000005
    li      t1, 4
    remuw   t2, t0, t1
    li      t3, 1
    li      a1, 18
    bne     t2, t3, fail

    # REMW by zero = sext32 of the dividend's low 32 bits
    li      t0, 0x1ffffffff         # low 32 = 0xffffffff
    remw    t2, t0, x0
    li      t3, -1
    li      a1, 19
    bne     t2, t3, fail

    li      a0, 10
    ecall
fail:
    li      a0, 11
    ecall
9:  j       9b
