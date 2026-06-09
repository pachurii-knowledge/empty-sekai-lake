    # R1 bring-up smoke test for the RV64 datapath. Pure base-integer + a 64-bit
    # shift that only behaves correctly with XLEN=64 (1<<40 is 0 on RV32).
    # a0=10 pass, a0=11 fail.   Build: make assemble TEST=tests/rv64_smoke.S RV64=1
    .section .text
    .globl main
main:
    li      t0, 5
    li      t1, 7
    add     t2, t0, t1              # 12
    li      t3, 12
    bne     t2, t3, fail

    sub     t2, t1, t0              # 2
    li      t3, 2
    bne     t2, t3, fail

    # 64-bit shift: 1 << 40, then back. On RV32 the value would be lost.
    li      t0, 1
    slli    t0, t0, 40
    srli    t0, t0, 40
    li      t3, 1
    bne     t0, t3, fail

    # sign extension of a negative immediate across 64 bits
    li      t0, -1                  # 0xffff_ffff_ffff_ffff
    srli    t1, t0, 60              # 0xf
    li      t3, 0xf
    bne     t1, t3, fail

    # branch + jump
    li      a0, 10
    ecall
fail:
    li      a0, 11
    ecall
9:  j       9b
