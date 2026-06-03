    # Privileged-ISA smoke test: mtvec setup, ECALL trap, mcause/mepc, MRET.
    # Halts with a0 = 10 on success, a0 = 11 on failure.
    .section .text
    .globl main
main:
    la      t0, mtrap_handler
    csrw    mtvec, t0
    li      s0, 0               # trap counter

    li      a0, 0               # not the halt argument
    ecall                       # synchronous trap from M-mode -> handler

    li      t1, 1
    bne     s0, t1, fail        # handler must have run exactly once

    li      a0, 10              # success: halt
    ecall

fail:
    li      a0, 11              # failure: halt
    ecall

    .align 2
mtrap_handler:
    csrr    t2, mcause          # expect 11 (ECALL from M-mode)
    li      t3, 11
    bne     t2, t3, fail
    addi    s0, s0, 1
    csrr    t4, mepc
    addi    t4, t4, 4           # skip the ecall
    csrw    mepc, t4
    mret
