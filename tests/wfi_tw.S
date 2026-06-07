    # WFI mstatus.TW semantics test (medeleg=0, so traps reach M-mode):
    #   Phase 1: S-mode, TW=1 -> WFI is illegal (mcause=2).
    #   Phase 2: U-mode, TW=0 -> WFI is LEGAL and waits; an armed machine timer
    #            wakes it and the interrupt is taken (mcause=0x80000007), NOT an
    #            illegal-instruction trap. (Catches the bug where U-mode WFI was
    #            unconditionally illegal regardless of TW.)
    #
    # a0=10 pass; a0=11 fail, a1 = 51 wrong S/TW cause / 52 S WFI didn't trap /
    #                              53 U WFI trapped illegal / 54 wrong U wake cause.
    .section .text
    .globl main
main:
    la      t0, mtrap
    csrw    mtvec, t0

    #################### Phase 1: S-mode, TW=1 -> illegal ####################
    li      t0, (1 << 21)           # TW
    csrs    mstatus, t0
    li      t0, (3 << 11)
    csrc    mstatus, t0
    li      t0, (1 << 11)
    csrs    mstatus, t0             # MPP = S
    la      t0, smode
    csrw    mepc, t0
    mret

    .align 2
smode:
    wfi                             # must trap illegal (TW=1, S-mode)
    li      a0, 11
    li      a1, 52
    ecall

    #################### Phase 2: U-mode, TW=0 -> waits, timer wakes #########
phase2:
    li      t0, (1 << 21)
    csrc    mstatus, t0             # TW = 0
    # arm machine timer ~100 ticks out
    li      t1, 0x0200BFF8
    lw      t2, 0(t1)
    addi    t2, t2, 100
    li      t3, 0x02004000
    sw      x0, 4(t3)
    sw      t2, 0(t3)
    li      t0, (1 << 7)
    csrw    mie, t0                 # MTIE (wakes WFI even with MIE=0)
    li      t0, (3 << 11)
    csrc    mstatus, t0             # MPP = U
    la      t0, umode
    csrw    mepc, t0
    mret

    .align 2
umode:
    wfi                             # legal in U with TW=0: waits for the timer
    li      a0, 11
    li      a1, 53                  # reached only if WFI trapped/NOP'd wrongly
    ecall

    .align 2
mtrap:
    csrr    t5, mcause
    # Phase 1: illegal from S-mode.
    li      t1, 2
    beq     t5, t1, p1_ok
    # Phase 2: the timer interrupt that woke the U-mode WFI.
    li      t1, 0x80000007
    beq     t5, t1, pass
    # An illegal-instruction trap from U-mode WFI is the bug we guard against.
    li      a0, 11
    li      a1, 54
    ecall
p1_ok:
    j       phase2                  # M-mode resumes here (mepc was the smode wfi)
pass:
    li      a0, 10
    ecall
