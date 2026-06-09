    # Phase H4: console / long-run tooling validation stub.
    #
    # Stands in for a booting kernel to exercise scripts/run_linux.sh end to end
    # WITHOUT a real RV64 Linux: an M-mode SBI firmware + an S-mode "kernel" that
    # prints a login-prompt line -- including newline bytes (0x0a == 10) -- via
    # the SBI DBCN console, then idles. The newline is the point: byte 10 in a0
    # at the console ecall would trip the ECALL_ARG_HALT halt and stop the sim;
    # run with +no_ecall_halt so it instead traps to the SBI firmware and prints.
    # The run is terminated by run_linux.sh's console watch matching the prompt
    # (there is no self-halt), modelling how a real boot-to-prompt is detected.
    #
    #   make assemble TEST=tests/linux_console_stub.S OUTPUT=output/lxstub
    #   scripts/run_linux.sh --dir output/lxstub --watch 'niigo login:'
    .option arch, +zicsr
    .equ EID_DBCN,   0x4442434E
    .equ EID_SRST,   0x53525354
    .equ UART_THR,   0x0D000000

    .section .text
    .globl main
main:
    # ---------------- M-mode firmware ----------------
    la      t0, m_trap_vector
    csrw    mtvec, t0
    li      t0, 0x0000B1FF
    csrw    medeleg, t0
    li      t0, (1<<9)|(1<<5)|(1<<1)
    csrw    mideleg, t0
    li      t0, (3<<11)
    csrc    mstatus, t0
    li      t0, (1<<11)             # MPP = S
    csrs    mstatus, t0
    la      t0, s_kernel
    csrw    mepc, t0
    mret

    # ---------------- S-mode "kernel" ----------------
s_kernel:
    la      t0, s_trap_vector
    csrw    stvec, t0

    li      s2, 3                   # print the banner a few times
banner:
    la      s1, prompt_addr
prchar:
    lbu     a0, 0(s1)
    beqz    a0, next
    li      a7, EID_DBCN
    li      a6, 2                   # console_write_byte (a0 may be 10 = '\n')
    ecall
    addi    s1, s1, 1
    j       prchar
next:
    addi    s2, s2, -1
    bnez    s2, banner

    # idle forever; run_linux.sh's console watch ends the run on the prompt match
1:  wfi
    j       1b

    # the prompt string, placed in .text and addressed PC-relatively (la to a
    # .data symbol is unreliable in this harness; see the test-harness notes)
    .align 2
prompt_addr:
    .ascii "niigo login: \n"
    .byte 0

    # ---------------- S trap handler ----------------
    .align 2
s_trap_vector:
    sret                            # ignore (no traps expected)

    # ---------------- M-mode SBI firmware ----------------
    .align 2
m_trap_vector:
    csrr    t0, mcause
    bltz    t0, m_int
    li      t1, 9                   # ecall-from-S
    bne     t0, t1, m_ret
    li      t1, EID_DBCN
    bne     a7, t1, m_srst_q
    # DBCN console_write_byte
    li      t1, UART_THR
    sb      a0, 0(t1)
    li      a0, 0
    j       m_ecall_ret
m_srst_q:
    li      t1, EID_SRST
    bne     a7, t1, m_unsup
    li      a0, 10                  # system_reset -> halt (only without the gate)
    ecall
m_unsup:
    li      a0, -2
m_ecall_ret:
    csrr    t0, mepc
    addi    t0, t0, 4
    csrw    mepc, t0
m_ret:
    mret
m_int:
    mret
