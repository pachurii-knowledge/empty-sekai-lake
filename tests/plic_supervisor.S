    # PLIC supervisor-external interrupt test (context 1 -> mip.SEIP).
    #
    # Delegate the S-external interrupt (mideleg bit 9), configure PLIC source 7
    # for context 1, drop to S-mode with SIE + sie.SEIE, inject the source, and
    # confirm the interrupt is taken in S-mode (scause 0x80000009). The S handler
    # claims context 1 (must read 7) and completes.
    #
    # PLIC ctx1: enable @ 0x0C002080, threshold @ 0x0C201000,
    #            claim/complete @ 0x0C201004.
    #
    # a0=10 pass; a0=11 fail, a1 = 71 no S interrupt / 72 wrong scause /
    #                              73 wrong claim id.
    .section .text
    .globl main
main:
    la      t0, mtrap_fail
    csrw    mtvec, t0
    la      t0, shandler
    csrw    stvec, t0
    li      s0, 0x0C000000          # PLIC base
    li      s1, 0

    # delegate S-external interrupt to S-mode
    li      t1, (1 << 9)            # mideleg.SEI
    csrw    mideleg, t1

    # PLIC: priority[7]=4, enable for ctx1, threshold[ctx1]=0
    li      t1, 4
    sw      t1, 0x1C(s0)            # priority[7] @ 0x0C00001C
    li      t1, (1 << 7)
    li      t2, 0x0C002080
    sw      t1, 0(t2)               # enable source 7 for context 1
    li      t2, 0x0C201000
    sw      x0, 0(t2)               # threshold[ctx1] = 0

    # enable S external interrupt in sie; will run with SIE set in S-mode
    li      t1, (1 << 9)            # SEIE
    csrw    sie, t1

    # enter S-mode (MPP=S) with SPIE so SIE is set after sret... use mret with
    # MPP=S and pre-set mstatus.SIE; simpler: set sstatus.SIE before sret.
    li      t1, (1 << 1)            # sstatus.SIE
    csrs    sstatus, t1
    li      t0, (3 << 11)
    csrc    mstatus, t0
    li      t0, (1 << 11)
    csrs    mstatus, t0             # MPP = S
    la      t0, smode
    csrw    mepc, t0
    mret

    .align 2
smode:
    # inject PLIC source 7 (write to pending word)
    li      t1, (1 << 7)
    li      t2, 0x0C001000
    sw      t1, 0(t2)
    li      t3, 200
1:  addi    t3, t3, -1
    bnez    t3, 1b
    # reached only if the S interrupt never fired
    li      a0, 11
    li      a1, 71
    ecall

    .align 2
shandler:
    csrr    t0, scause
    li      t1, 0x80000009          # supervisor external interrupt
    bne     t0, t1, wrong_cause
    li      t2, 0x0C201004          # claim/complete ctx1
    lw      t3, 0(t2)
    li      t4, 7
    bne     t3, t4, wrong_claim
    sw      t3, 0(t2)               # complete source 7
    li      a0, 10
    ecall

wrong_cause:  li a0, 11
    li a1, 72
    ecall
wrong_claim:  li a0, 11
    li a1, 73
    ecall
    .align 2
mtrap_fail:
    li      a0, 11
    li      a1, 70                  # interrupt wrongly taken in M-mode
    ecall
