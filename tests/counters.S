    # Counter access-control + mcountinhibit test (M/S-mode).
    #
    #  Phase 1: mcountinhibit gates mcycle. Set inhibit.CY, read mcycle twice ->
    #           unchanged; clear, read twice -> increased.
    #  Phase 2: mcounteren read-gating. Drop to S-mode (MPP=S) and read `cycle`
    #           with mcounteren.CY=0 -> illegal trap (mcause=2). Then with
    #           mcounteren.CY=1 -> succeeds (no trap).
    #
    # a0=10 pass; a0=11 fail, a1 = 61 inhibit-on changed / 62 inhibit-off frozen /
    #                              63 S-read not trapped / 64 wrong cause /
    #                              65 enabled S-read trapped.
    .section .text
    .globl main
main:
    la      t0, mtrap
    csrw    mtvec, t0
    li      s3, 0                   # trap-seen flag

    # ---- Phase 1: mcountinhibit.CY ----
    li      t0, 0x1
    csrw    0x320, t0               # mcountinhibit.CY = 1
    csrr    t1, mcycle
    csrr    t2, mcycle
    li      a1, 61
    bne     t1, t2, fail            # inhibited: must be unchanged

    csrw    0x320, x0               # mcountinhibit = 0
    csrr    t1, mcycle
    nop
    csrr    t2, mcycle
    li      a1, 62
    bgeu    t1, t2, fail            # running: t2 must be > t1

    # ---- Phase 2a: S-mode `cycle` read with mcounteren.CY=0 -> trap ----
    csrw    mcounteren, x0          # CY=0
    li      t0, (3 << 11)
    csrc    mstatus, t0
    li      t0, (1 << 11)
    csrs    mstatus, t0             # MPP = S
    la      t0, s_read_denied
    csrw    mepc, t0
    li      s3, 0
    mret

    .align 2
s_read_denied:
    csrr    t1, cycle               # should trap (illegal)
    # only reached if NOT trapped
    li      a0, 11
    li      a1, 63
    ecall

    # Resumes here (mtrap sets mepc to this label on the expected trap).
    .align 2
after_denied:
    li      a1, 64
    li      t1, 2
    bne     s4, t1, fail            # cause must be illegal instruction

    # ---- Phase 2b: enable mcounteren.CY, S-mode read must succeed ----
    li      t0, 0x1
    csrw    mcounteren, t0          # CY=1 (we are back in M-mode after the trap)
    li      t0, (3 << 11)
    csrc    mstatus, t0
    li      t0, (1 << 11)
    csrs    mstatus, t0             # MPP = S
    la      t0, s_read_ok
    csrw    mepc, t0
    li      s3, 0
    mret

    .align 2
s_read_ok:
    csrr    t1, cycle               # must NOT trap now
    ecall                           # ecall from S -> M handler; pass path below

    .align 2
mtrap:
    csrr    s4, mcause
    li      s3, 1
    # Distinguish the expected illegal-read trap from the final S-mode ecall.
    li      t5, 2                   # illegal instruction (denied cycle read)
    beq     s4, t5, recover_denied
    li      t5, 9                   # ECALL from S (the s_read_ok success path)
    beq     s4, t5, pass
    # ECALL from M (a0 already set) or anything else: fall through to mret.
    li      a1, 65
    j       fail
recover_denied:
    li      t6, (3 << 11)
    csrs    mstatus, t6             # MPP = M so mret resumes in M-mode
    la      t6, after_denied
    csrw    mepc, t6
    mret
pass:
    li      a0, 10
    ecall
fail:
    li      a0, 11
    ecall
