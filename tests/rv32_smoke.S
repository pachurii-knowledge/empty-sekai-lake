    # Minimal pure-integer test for the bare scalar core (no peripherals).
    .section .text
    .globl main
main:
    li      t0, 5
    li      t1, 7
    add     t2, t0, t1
    li      t3, 12
    bne     t2, t3, fail
    sub     t2, t1, t0
    li      t3, 2
    bne     t2, t3, fail
    slli    t0, t0, 4
    li      t3, 80
    bne     t0, t3, fail
    li      a0, 10
    ecall
fail:
    li      a0, 11
    ecall
9:  j       9b
