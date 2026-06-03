    # Delegation / S-mode / SRET test:
    #  - M-mode delegates ECALL-from-S (cause 9) to S via medeleg
    #  - MRET drops to S-mode (mstatus.MPP = 01)
    #  - S-mode ECALL traps to stvec with scause = 9, handler SRET-returns
    # Records scause in s1, trap count in s0; halts with a0 = 10.
    .section .text
    .globl main
main:
    la      t0, strap
    csrw    stvec, t0
    li      t1, (1 << 9)        # delegate ECALL-from-S
    csrw    medeleg, t1

    la      t2, smode_entry
    csrw    mepc, t2
    li      t3, 0x1800          # mstatus.MPP field mask
    csrc    mstatus, t3
    li      t4, 0x800           # MPP = 01 (Supervisor)
    csrs    mstatus, t4
    li      s0, 0
    mret                        # -> smode_entry, now in S-mode

smode_entry:
    li      a0, 0
    ecall                       # ECALL from S -> delegated to S handler
    li      t5, 1
    bne     s0, t5, fail
    li      a0, 10
    ecall                       # halt (success)

fail:
    li      a0, 11
    ecall

    .align 2
strap:
    csrr    s1, scause          # expect 9 (ECALL from S-mode)
    li      t6, 9
    bne     s1, t6, fail
    addi    s0, s0, 1
    csrr    t0, sepc
    addi    t0, t0, 4
    csrw    sepc, t0
    sret                        # return to S-mode
