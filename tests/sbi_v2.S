    # Phase H3: SBI v0.2 firmware surface, exercised as a directed test.
    #
    # An M-mode firmware shim implements the v0.2 ecall ABI (a7=EID, a6=FID,
    # args a0..a5, return a0=error/a1=value) for the extensions a minimal kernel
    # uses, plus the legacy console/timer/shutdown calls. An S-mode "kernel"
    # makes the calls and checks the sbiret values:
    #   Base    (0x10): get_spec_version, probe_extension
    #   TIME    (0x54494D45): set_timer  (-> CLINT mtimecmp + M-timer->STIP relay)
    #   HSM     (0x48534D):   hart_get_status (hart 0 -> STARTED)
    #   DBCN    (0x4442434E): console_write_byte -> UART
    #   IPI/RFENCE:           single-hart no-op success
    #   SRST    (0x53525354): system_reset -> halt
    #   legacy  (<0x10):      console_putchar / set_timer / shutdown
    #
    # The design is XLEN-agnostic; on RV64 the same shim is rebuilt (set_timer
    # takes the full 64-bit deadline in a0 instead of a0:a1).
    #
    # NOTE: the core halts on an ecall with a0==10 or a0==11 (pass/fail), in ANY
    # mode -- so an SBI call must never present a0 in {10,11} (it would halt
    # instead of trapping to the firmware). The kernel sets a0 explicitly before
    # each call and the console bytes avoid 10/11.
    #
    # a0=10 pass (via SRST). a0=11 fail, a1 = fail code:
    #   71 unexpected M trap, 72 unexpected S trap, 73 timer never fired,
    #   81 probe(TIME)!=1, 82 probe(unknown)!=0, 83 spec_version==0,
    #   84 hart_status!=STARTED.
    .option arch, +zicsr
    .equ EID_BASE,   0x10
    .equ EID_TIME,   0x54494D45
    .equ EID_IPI,    0x735049
    .equ EID_RFNC,   0x52464E43
    .equ EID_HSM,    0x48534D
    .equ EID_SRST,   0x53525354
    .equ EID_DBCN,   0x4442434E
    .equ LEG_SETTIMER, 0
    .equ LEG_PUTCHAR,  1
    .equ LEG_SHUTDOWN, 8
    .equ UART_THR,   0x0D000000
    .equ MTIMECMP,   0x02004000

    .section .text
    .globl main
main:
    # ---------------- M-mode firmware init ----------------
    la      t0, m_trap_vector
    csrw    mtvec, t0
    li      t0, 0x0000B1FF          # medeleg: usual exceptions -> S (not ecall-S)
    csrw    medeleg, t0
    li      t0, (1<<9)|(1<<5)|(1<<1)
    csrw    mideleg, t0
    li      t0, 0x7
    csrw    mcounteren, t0
    li      t0, (3<<11)
    csrc    mstatus, t0
    li      t0, (1<<11)             # MPP = S
    csrs    mstatus, t0
    la      t0, s_kernel
    csrw    mepc, t0
    mret

    # ---------------- S-mode kernel ----------------
s_kernel:
    la      t0, s_trap_vector
    csrw    stvec, t0

    # (1) Base: probe_extension(TIME) -> value 1
    li      a7, EID_BASE
    li      a6, 3                   # probe_extension
    li      a0, EID_TIME
    ecall
    li      t0, 1
    bne     a1, t0, f_probe_time

    # (2) Base: probe_extension(unknown) -> value 0
    li      a7, EID_BASE
    li      a6, 3
    li      a0, 0x09000000          # not implemented
    ecall
    bnez    a1, f_probe_unknown

    # (3) Base: get_spec_version -> nonzero
    li      a7, EID_BASE
    li      a6, 0
    li      a0, 0
    ecall
    beqz    a1, f_specver

    # (4) DBCN console_write_byte: "sbiv2" (no 10/11 bytes)
    .macro DBCN_PUTC ch
    li      a7, EID_DBCN
    li      a6, 2                   # console_write_byte
    li      a0, \ch
    ecall
    .endm
    DBCN_PUTC 's'; DBCN_PUTC 'b'; DBCN_PUTC 'i'; DBCN_PUTC 'v'; DBCN_PUTC '2'

    # (5) HSM hart_get_status(0) -> STARTED (0)
    li      a7, EID_HSM
    li      a6, 2                   # hart_get_status
    li      a0, 0
    ecall
    bnez    a1, f_hsm

    # (6) TIME set_timer -> supervisor timer interrupt fires
    li      s1, 0                   # timer-fired flag
    csrr    a0, time
    csrr    a1, timeh
    addi    a0, a0, 200
    li      a7, EID_TIME
    li      a6, 0                   # set_timer
    ecall
    li      t0, (1<<5)
    csrs    sie, t0                 # STIE
    csrsi   sstatus, 0x2            # SIE
    li      s2, 2000000
1:  bnez    s1, 2f
    addi    s2, s2, -1
    bnez    s2, 1b
    li      a1, 73
    j       fail
2:
    # (7) SRST system_reset -> halt (pass)
    li      a7, EID_SRST
    li      a6, 0                   # system_reset
    li      a0, 0                   # reset type = shutdown
    li      a1, 0                   # reason
    ecall
    # not reached
    li      a1, 99
    j       fail

f_probe_time:    li a1, 81; j fail
f_probe_unknown: li a1, 82; j fail
f_specver:       li a1, 83; j fail
f_hsm:           li a1, 84
fail:
    li      a0, 11
    ecall
9:  j       9b

    # ---------------- S-mode trap handler (timer) ----------------
    .align 2
s_trap_vector:
    csrr    t0, scause
    li      t1, 0x80000005          # supervisor timer interrupt
    bne     t0, t1, s_bad
    li      a7, EID_TIME            # re-arm far away -> clears STIP
    li      a6, 0
    li      a0, -1
    li      a1, -1
    ecall
    li      s1, 1
    sret
s_bad:
    li      a1, 72
    j       fail

    # ---------------- M-mode SBI firmware ----------------
    .align 2
m_trap_vector:
    csrr    t0, mcause
    bltz    t0, m_interrupt
    li      t1, 9                   # ecall-from-S
    bne     t0, t1, m_bad

    # dispatch on EID (a7)
    li      t1, EID_BASE
    blt     a7, t1, leg_dispatch    # legacy EIDs < 0x10
    beq     a7, t1, sbi_base
    li      t1, EID_TIME
    beq     a7, t1, sbi_time
    li      t1, EID_HSM
    beq     a7, t1, sbi_hsm
    li      t1, EID_DBCN
    beq     a7, t1, sbi_dbcn
    li      t1, EID_SRST
    beq     a7, t1, sbi_srst
    li      t1, EID_IPI
    beq     a7, t1, sbi_ok          # send_ipi: single-hart no-op
    li      t1, EID_RFNC
    beq     a7, t1, sbi_ok          # remote fences: single-hart no-op
    li      a0, -2                  # SBI_ERR_NOT_SUPPORTED
    li      a1, 0
    j       sbi_ret

sbi_base:
    li      t1, 0                   # FID 0: get_spec_version
    beq     a6, t1, base_specver
    li      t1, 3                   # FID 3: probe_extension
    beq     a6, t1, base_probe
    li      a0, 0                   # get_impl_id/version/m*id -> 0
    li      a1, 0
    j       sbi_ret
base_specver:
    li      a0, 0
    li      a1, 0x01000000          # SBI spec v1.0
    j       sbi_ret
base_probe:
    # a0 = queried EID; return a1=1 if we implement it, else 0
    li      a1, 1
    li      t1, EID_BASE; beq a0, t1, 1f
    li      t1, EID_TIME; beq a0, t1, 1f
    li      t1, EID_IPI;  beq a0, t1, 1f
    li      t1, EID_RFNC; beq a0, t1, 1f
    li      t1, EID_HSM;  beq a0, t1, 1f
    li      t1, EID_SRST; beq a0, t1, 1f
    li      t1, EID_DBCN; beq a0, t1, 1f
    li      a1, 0
1:  li      a0, 0
    j       sbi_ret

sbi_time:                           # FID 0: set_timer(a0=lo, a1=hi)
    li      t1, MTIMECMP
    sw      a0, 0(t1)
    sw      a1, 4(t1)
    li      t1, (1<<5)
    csrc    mip, t1                 # clear any relayed STIP
    li      t1, (1<<7)
    csrs    mie, t1                 # enable MTIE
    j       sbi_ok

sbi_hsm:
    li      t1, 2                   # FID 2: hart_get_status
    bne     a6, t1, hsm_other
    bnez    a0, hsm_badhart         # only hart 0 exists
    li      a0, 0
    li      a1, 0                   # 0 = STARTED
    j       sbi_ret
hsm_other:                          # hart_start/stop/suspend: single hart
    li      a0, -3                  # SBI_ERR_INVALID_PARAM
    li      a1, 0
    j       sbi_ret
hsm_badhart:
    li      a0, -3
    li      a1, 0
    j       sbi_ret

sbi_dbcn:
    li      t1, 2                   # FID 2: console_write_byte (a0=byte)
    beq     a6, t1, dbcn_byte
    li      t1, 0                   # FID 0: console_write(a0=len, a1=addr_lo)
    beq     a6, t1, dbcn_write
    li      a0, -2
    li      a1, 0
    j       sbi_ret
dbcn_byte:
    li      t1, UART_THR
    sb      a0, 0(t1)
    j       sbi_ok
dbcn_write:                         # write a0 bytes from [a1..] to the UART
    mv      t2, a1                  # src
    mv      t3, a0                  # count
    li      t4, UART_THR
2:  beqz    t3, sbi_ok
    lbu     t5, 0(t2)
    sb      t5, 0(t4)
    addi    t2, t2, 1
    addi    t3, t3, -1
    j       2b

sbi_srst:                           # system_reset -> halt
    li      a0, 10
    ecall

leg_dispatch:                       # legacy SBI (a7 < 0x10)
    li      t1, LEG_PUTCHAR
    beq     a7, t1, leg_putchar
    li      t1, LEG_SETTIMER
    beq     a7, t1, sbi_time
    li      t1, LEG_SHUTDOWN
    beq     a7, t1, sbi_srst
    li      a0, -2
    j       sbi_ret
leg_putchar:
    li      t1, UART_THR
    sb      a0, 0(t1)
    li      a0, 0
    j       sbi_ret

sbi_ok:
    li      a0, 0
    li      a1, 0
sbi_ret:
    csrr    t0, mepc
    addi    t0, t0, 4
    csrw    mepc, t0
    mret

m_interrupt:
    slli    t1, t0, 1
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
