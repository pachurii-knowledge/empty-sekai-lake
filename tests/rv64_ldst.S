    # R2b: RV64 load/store path (LD/SD/LW/LWU/LH/LHU/LB/LBU/SH/SB), 64-bit words.
    # Uses an absolute scratch base (la to .data is unreliable in this harness).
    # a0=10 pass; a0=11 fail, a1 = which check (1..8).
    .section .text
    .globl main
main:
    li      s0, 0x10000000          # scratch RAM base (bare mode: VA=PA)

    # SD / LD (full doubleword)
    li      t0, 0x1122334455667788
    sd      t0, 0(s0)
    ld      t1, 0(s0)
    li      a1, 1
    bne     t0, t1, fail

    # SW then LW (sign-extend) and LWU (zero-extend)
    li      t0, 0x80000000
    sw      t0, 8(s0)
    lw      t1, 8(s0)               # -> 0xffffffff80000000
    li      t2, 0xffffffff80000000
    li      a1, 2
    bne     t1, t2, fail
    lwu     t1, 8(s0)               # -> 0x0000000080000000
    li      t2, 0x80000000
    li      a1, 3
    bne     t1, t2, fail

    # SH / LH (sign-extend) / LHU
    li      t0, 0x8001
    sh      t0, 16(s0)
    lh      t1, 16(s0)              # -> 0xffffffffffff8001
    li      t2, 0xffffffffffff8001
    li      a1, 4
    bne     t1, t2, fail
    lhu     t1, 16(s0)
    li      t2, 0x8001
    li      a1, 5
    bne     t1, t2, fail

    # SB / LB (sign-extend) / LBU
    li      t0, 0x81
    sb      t0, 24(s0)
    lb      t1, 24(s0)              # -> 0xffffffffffffff81
    li      t2, 0xffffffffffffff81
    li      a1, 6
    bne     t1, t2, fail
    lbu     t1, 24(s0)
    li      t2, 0x81
    li      a1, 7
    bne     t1, t2, fail

    # byte offset within a doubleword (offset 1 of the dword at +32)
    li      t0, 0xAA
    sb      t0, 33(s0)
    lbu     t1, 33(s0)
    li      t2, 0xAA
    li      a1, 8
    bne     t1, t2, fail

    li      a0, 10
    ecall
fail:
    li      a0, 11
    ecall
9:  j       9b
