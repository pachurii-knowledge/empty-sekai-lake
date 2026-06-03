    # Timer-interrupt smoke test: program CLINT mtimecmp, enable MTIE+MIE, spin
    # until the machine timer interrupt fires, verify mcause in the handler.
    # Records the observed mcause in s1; halts with a0 = 10.
    .section .text
    .globl main
main:
    la      t0, handler
    csrw    mtvec, t0

    li      t1, 0x02004000      # CLINT mtimecmp low word
    li      t2, 80
    sw      t2, 0(t1)           # mtimecmp = 80
    sw      zero, 4(t1)         # mtimecmp high = 0

    li      t3, 0x80            # mie.MTIE (bit 7)
    csrw    mie, t3
    li      t4, 0x8             # mstatus.MIE (bit 3)
    csrs    mstatus, t4

    li      s0, 0
    li      s1, 0
spin:
    addi    s0, s0, 1
    j       spin

    .align 2
handler:
    csrr    s1, mcause          # expect 0x80000007 (machine timer interrupt)

    li      t1, 0x02004000      # silence the timer: mtimecmp = max
    li      t6, -1
    sw      t6, 0(t1)
    sw      t6, 4(t1)

    li      a0, 10
    ecall                       # halt
