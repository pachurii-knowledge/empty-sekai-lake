    # RV32 load/store regression for the rewritten (XLEN-generic) load/store path.
    .section .text
    .globl main
main:
    li      s0, 0x10000000
    li      t0, 0x11223344
    sw      t0, 0(s0)
    lw      t1, 0(s0)
    li      a1, 1
    bne     t0, t1, fail
    li      t0, 0x8001
    sh      t0, 8(s0)
    lh      t1, 8(s0)              # -> 0xffff8001
    li      t2, 0xffff8001
    li      a1, 2
    bne     t1, t2, fail
    lhu     t1, 8(s0)
    li      t2, 0x8001
    li      a1, 3
    bne     t1, t2, fail
    li      t0, 0x81
    sb      t0, 16(s0)
    lb      t1, 16(s0)             # -> 0xffffff81
    li      t2, 0xffffff81
    li      a1, 4
    bne     t1, t2, fail
    lbu     t1, 16(s0)
    li      t2, 0x81
    li      a1, 5
    bne     t1, t2, fail
    li      a0, 10
    ecall
fail:
    li      a0, 11
    ecall
9:  j       9b
