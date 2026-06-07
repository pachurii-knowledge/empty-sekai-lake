    # WFI wake test (M-mode): arm the machine timer, WFI, and confirm the timer
    # interrupt wakes the core and is taken with epc just after the WFI.
    #
    # If WFI never woke, the core would idle forever and the test would time out.
    # Pass a0=10 (timer interrupt handler reached with the right cause).
    .section .text
    .globl main
main:
    la      t0, handler
    csrw    mtvec, t0

    # mtimecmp = mtime + 100
    li      s1, 0x0200BFF8          # mtime lo
    lw      t1, 0(s1)
    addi    t1, t1, 100
    li      s2, 0x02004000          # mtimecmp
    sw      x0, 4(s2)               # mtimecmp.hi = 0 first (avoid spurious fire)
    sw      t1, 0(s2)               # mtimecmp.lo

    # Enable machine timer interrupt + global interrupts.
    li      t0, (1 << 7)            # MTIE
    csrw    mie, t0
    csrsi   mstatus, 0x8            # MIE

    wfi                             # idle until the timer fires
    # Reached only if the interrupt is taken at epc = here; handler runs instead.
    li      a0, 11
    ecall

    .align 2
handler:
    csrr    t0, mcause
    li      t1, 0x80000007          # machine timer interrupt
    bne     t0, t1, fail
    li      a0, 10
    ecall
fail:
    li      a0, 11
    ecall
