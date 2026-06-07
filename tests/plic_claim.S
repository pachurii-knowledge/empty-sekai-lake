    # PLIC external-interrupt claim/complete test (M-mode).
    #
    # Configure PLIC source 3 (priority 5, enabled for context 0, threshold 0),
    # inject it via the pending word, and confirm the machine-external interrupt
    # is taken (mcause 0x8000000B). The handler claims (must read 3), completes,
    # and the interrupt does not re-fire.
    #
    # PLIC base 0x0C000000:
    #   priority[3] @ 0x0C00000C, pending @ 0x0C001000,
    #   enable[ctx0] @ 0x0C002000, threshold[ctx0] @ 0x0C200000,
    #   claim/complete[ctx0] @ 0x0C200004
    #
    # a0=10 pass; a0=11 fail, a1 = 61 no interrupt / 62 wrong cause /
    #                              63 wrong claim id / 64 re-fired after complete.
    .section .text
    .globl main
main:
    la      t0, handler
    csrw    mtvec, t0
    li      s0, 0x0C000000          # PLIC base
    li      s1, 0                   # interrupt-count

    li      t1, 5
    sw      t1, 0x0C(s0)            # priority[3] = 5
    li      t1, (1 << 3)
    li      t2, 0x0C002000
    sw      t1, 0(t2)               # enable source 3 for context 0
    li      t2, 0x0C200000
    sw      x0, 0(t2)               # threshold[ctx0] = 0

    li      t1, (1 << 11)           # MEIE
    csrw    mie, t1
    csrsi   mstatus, 0x8            # MIE

    # inject source 3
    li      t1, (1 << 3)
    li      t2, 0x0C001000
    sw      t1, 0(t2)

    # spin briefly; the interrupt should preempt us and the handler halts.
    li      t3, 200
1:  addi    t3, t3, -1
    bnez    t3, 1b

    # reached only if the interrupt never fired
    li      a0, 11
    li      a1, 61
    ecall

    .align 2
handler:
    addi    s1, s1, 1
    li      t4, 2
    bge     s1, t4, refired         # taken a second time -> bug
    csrr    t0, mcause
    li      t1, 0x8000000B          # machine external interrupt
    bne     t0, t1, wrong_cause
    li      t2, 0x0C200004          # claim/complete ctx0
    lw      t3, 0(t2)               # claim
    li      t4, 3
    bne     t3, t4, wrong_claim
    sw      t3, 0(t2)               # complete source 3
    # let a few instructions retire so a (buggy) re-fire would be observed
    li      t5, 20
2:  addi    t5, t5, -1
    bnez    t5, 2b
    li      a0, 10
    ecall

wrong_cause:  li a0, 11
    li a1, 62
    ecall
wrong_claim:  li a0, 11
    li a1, 63
    ecall
refired:      li a0, 11
    li a1, 64
    ecall
