    # dcache_alias_set: 5 fully-populated 16-word lines mapping to ONE set (4
    # ways). Writing the 5th evicts a full dirty 16-word victim (PLRU); readback
    # must refill every line intact -- exercises set-conflict thrash + victim
    # selection + multi-word writeback.
    .option arch, +zicsr
    .equ BASE, 0x10000000
    .equ STRIDE, 0x1000          # same set, different tag
    .section .text
    .globl main
main:
    li      s0, BASE
    # ---- write 5 lines, line k word j = 0xD000 + (k<<8) + j ----
    li      s2, 0                # line k
    add     s4, s0, x0           # line pointer
kw:
    slli    t5, s2, 8            # k<<8
    li      s3, 0                # word j
jw:
    slli    t0, s3, 2
    add     t1, s4, t0
    li      t2, 0xD000
    add     t2, t2, t5
    add     t2, t2, s3
    sw      t2, 0(t1)
    addi    s3, s3, 1
    li      t3, 16
    blt     s3, t3, jw
    li      t0, STRIDE
    add     s4, s4, t0
    addi    s2, s2, 1
    li      t3, 5
    blt     s2, t3, kw
    # ---- read 5 lines back, verify ----
    li      s2, 0
    add     s4, s0, x0
kr:
    slli    t5, s2, 8
    li      s3, 0
jr:
    slli    t0, s3, 2
    add     t1, s4, t0
    lw      t4, 0(t1)
    li      t2, 0xD000
    add     t2, t2, t5
    add     t2, t2, s3
    bne     t4, t2, fail
    addi    s3, s3, 1
    li      t3, 16
    blt     s3, t3, jr
    li      t0, STRIDE
    add     s4, s4, t0
    addi    s2, s2, 1
    li      t3, 5
    blt     s2, t3, kr
    li      a0, 10
    ecall
fail:
    li      a0, 11
    ecall
9:  j       9b
