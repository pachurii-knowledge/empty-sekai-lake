    # Minimal in-tree SBI runtime (Phase J), exercised as a directed test.
    #
    # Boots in M-mode (reset vector), installs an M-mode SBI trap handler,
    # delegates exceptions + S-interrupts to S, and mret's into an S-mode
    # "kernel". The kernel makes legacy-SBI ecalls:
    #   - console putchar  -> M writes the byte to the UART (console).
    #   - set_timer        -> M programs CLINT mtimecmp + enables MTIE.
    # The M-timer fires, the handler relays it to S (mip.STIP) and disables
    # MTIE; the kernel takes a supervisor-timer interrupt, re-arms the timer
    # far away (clearing STIP), then issues SBI shutdown -> halt (a0=10).
    #
    # NOTE: the core halts on ANY ecall with a0==10 (ECALL_ARG_HALT), so the
    # console string deliberately avoids the byte 0x0a, and only `shutdown`
    # uses a0=10.
    #
    # a0=10 pass (via SBI shutdown). a1 carries a fail code on the fail paths:
    #   71 unexpected M trap, 72 unexpected S trap, 73 timer never fired.
    .option arch, +zicsr
    .equ SBI_SET_TIMER, 0
    .equ SBI_PUTCHAR,   1
    .equ SBI_SHUTDOWN,  8
    .equ UART_THR,      0x0D000000
    .equ MTIMECMP,      0x02004000

    .section .text
    .globl main
main:
    # ---------------- M-mode SBI init ----------------
    la      t0, m_trap_vector
    csrw    mtvec, t0
    li      t0, 0x0000B1FF          # medeleg: delegate the usual exceptions to S
    csrw    medeleg, t0             #   (NOT ecall-from-S bit9 -> SBI stays in M)
    li      t0, (1<<9)|(1<<5)|(1<<1)  # mideleg: SEI, STI, SSI -> S
    csrw    mideleg, t0
    li      t0, 0x7                 # mcounteren: let S read cycle/time/instret
    csrw    mcounteren, t0
    li      t0, (3<<11)             # clear mstatus.MPP
    csrc    mstatus, t0
    li      t0, (1<<11)             # mstatus.MPP = 01 (Supervisor)
    csrs    mstatus, t0
    la      t0, s_kernel
    csrw    mepc, t0
    mret                            # enter S-mode

    # ---------------- S-mode kernel ----------------
s_kernel:
    la      t0, s_trap_vector
    csrw    stvec, t0

    # SBI console putchar: "SBI:ok" (no 0x0a; see note above)
    li      a7, SBI_PUTCHAR
    li      a0, 'S'; ecall
    li      a7, SBI_PUTCHAR; li a0, 'B'; ecall
    li      a7, SBI_PUTCHAR; li a0, 'I'; ecall
    li      a7, SBI_PUTCHAR; li a0, ':'; ecall
    li      a7, SBI_PUTCHAR; li a0, 'o'; ecall
    li      a7, SBI_PUTCHAR; li a0, 'k'; ecall

    # SBI set_timer: deadline = time + 200
    csrr    a0, time
    csrr    a1, timeh
    addi    a0, a0, 200
    li      a7, SBI_SET_TIMER
    ecall

    li      t0, (1<<5)              # sie.STIE
    csrs    sie, t0
    csrsi   sstatus, 0x2            # sstatus.SIE

    li      s1, 0                   # timer-fired flag
    li      s2, 2000000             # bounded spin so a missed interrupt fails, not hangs
1:  bnez    s1, 2f
    addi    s2, s2, -1
    bnez    s2, 1b
    li      a1, 73                  # timer never fired
    j       fail
2:
    li      a7, SBI_SHUTDOWN        # SBI shutdown -> halt (a0=10)
    ecall
fail:
    li      a0, 11
    ecall
9:  j       9b

    # ---------------- S-mode trap handler ----------------
    .align 2
s_trap_vector:
    csrr    t0, scause
    li      t1, 0x80000005          # supervisor timer interrupt
    bne     t0, t1, s_bad
    li      a7, SBI_SET_TIMER       # re-arm far away -> clears STIP
    li      a0, -1
    li      a1, -1
    ecall
    li      s1, 1
    sret
s_bad:
    li      a1, 72
    j       fail

    # ---------------- M-mode SBI trap handler ----------------
    .align 2
m_trap_vector:
    csrr    t0, mcause
    bltz    t0, m_interrupt         # mcause[31]=1 -> interrupt
    li      t1, 9                   # ecall-from-S
    bne     t0, t1, m_bad
    li      t1, SBI_PUTCHAR
    beq     a7, t1, sbi_putchar
    li      t1, SBI_SET_TIMER
    beq     a7, t1, sbi_set_timer
    li      t1, SBI_SHUTDOWN
    beq     a7, t1, sbi_shutdown
    li      a0, -2                  # unsupported SBI -> error
    j       sbi_ret

sbi_putchar:
    li      t1, UART_THR
    sb      a0, 0(t1)
    li      a0, 0
    j       sbi_ret
sbi_set_timer:
    li      t1, MTIMECMP
    sw      a0, 0(t1)               # mtimecmp_lo
    sw      a1, 4(t1)               # mtimecmp_hi
    li      t1, (1<<5)
    csrc    mip, t1                 # clear any relayed STIP
    li      t1, (1<<7)
    csrs    mie, t1                 # enable MTIE
    li      a0, 0
    j       sbi_ret
sbi_shutdown:
    li      a0, 10
    ecall                          # halt
sbi_ret:
    csrr    t0, mepc
    addi    t0, t0, 4
    csrw    mepc, t0
    mret

m_interrupt:
    slli    t1, t0, 1               # drop mcause[31]
    srli    t1, t1, 1
    li      t2, 7                   # machine timer interrupt
    bne     t1, t2, m_bad
    li      t1, (1<<5)
    csrs    mip, t1                 # relay -> supervisor timer pending
    li      t1, (1<<7)
    csrc    mie, t1                 # disable MTIE (one-shot)
    mret
m_bad:
    li      a1, 71
    li      a0, 11
    ecall
