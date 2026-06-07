    # PMP enforcement test (M-mode, paging off).
    #
    # A locked TOR entry denies all access to [0x10002000, 0x10003000):
    #   pmpaddr0 = 0x10002000>>2   (lower bound of entry 1's TOR range)
    #   pmpaddr1 = 0x10003000>>2   (upper bound)
    #   pmpcfg0  = 0x00008800      (entry0 OFF; entry1 = L|TOR|---, no R/W/X)
    # Because it is locked (L=1), it applies even in M-mode.
    #
    # Checks:
    #   store to 0x10002000 -> store access fault (mcause 7)
    #   load  from 0x10002000 -> load access fault (mcause 5)
    #   store to 0x10001000 (outside the range) -> succeeds
    #   jump  to 0x10002000 -> instruction access fault (mcause 1)
    #
    # Exit a0=10 pass; a0=11 fail with a1 = diagnostic:
    #   31 store didn't fault   32 store wrong cause
    #   33 load didn't fault    34 load wrong cause
    #   35 allowed store faulted
    #   36 fetch didn't fault   37 fetch wrong cause
    .section .text
    .globl main
main:
    la      t0, mtrap
    csrw    mtvec, t0

    li      t0, 0x10002000 >> 2
    csrw    pmpaddr0, t0
    li      t0, 0x10003000 >> 2
    csrw    pmpaddr1, t0
    li      t0, 0x00008800          # entry1 L|TOR, no perms
    csrw    pmpcfg0, t0

    # ---- store to denied region: expect store access fault (7) ----
    li      s3, 0
    li      t1, 0x10002000
    sw      x0, 0(t1)
    li      a1, 31
    beqz    s3, fail
    li      a1, 32
    li      t2, 7
    bne     s4, t2, fail

    # ---- load from denied region: expect load access fault (5) ----
    li      s3, 0
    li      t1, 0x10002000
    lw      s0, 0(t1)
    li      a1, 33
    beqz    s3, fail
    li      a1, 34
    li      t2, 5
    bne     s4, t2, fail

    # ---- allowed store outside the range: must NOT fault ----
    li      s3, 0
    li      t1, 0x10001000
    li      t3, 0x1234
    sw      t3, 0(t1)
    li      a1, 35
    bnez    s3, fail

    # ---- fetch from denied region: expect instruction access fault (1) ----
    li      s3, 0
    li      t0, 0x10002000
    jalr    ra, 0(t0)               # fetch faults; handler recovers to after_fetch
after_fetch:
    li      a1, 36
    beqz    s3, fail
    li      a1, 37
    li      t2, 1
    bne     s4, t2, fail

    li      a0, 10
    ecall
fail:
    li      a0, 11
    ecall

    # Handler: record cause + seen-flag. For an instruction access fault (1) the
    # faulting PC is still in the denied region, so recover to a safe PC; other
    # faults simply skip the offending load/store (mepc+4).
    .align 2
mtrap:
    csrr    s4, mcause
    li      s3, 1
    li      t5, 1
    beq     s4, t5, fetch_recover
    csrr    t6, mepc
    addi    t6, t6, 4
    csrw    mepc, t6
    mret
fetch_recover:
    la      t6, after_fetch
    csrw    mepc, t6
    mret
