# Directed lockstep torture test: sret_mret_nested
#
# Corner: nested S/M privilege transitions and the sstatus/mstatus shadow
# state (SPIE/SPP/MPP/MPIE) under a trap-while-in-the-S-handler, then SRET
# and MRET unwinding back through both privilege levels.
#
# Flow:
#   M boot: delegate ILLEGAL-INSTRUCTION (cause 2) to S; leave BREAKPOINT
#   (cause 3, ebreak) NOT delegated so it always goes to M. mret -> S main.
#   S main: read sstatus; execute a delegated illegal instr -> S handler.
#   S handler: read sstatus (SPP=1, SPIE=prior SIE). Execute ebreak ->
#     traps to M WHILE SPP/SPIE hold S-handler state.
#   M handler: read mstatus (MPP=S=1, plus live SPP/SPIE). mret -> back into
#     the S handler.
#   S handler (resumed): read sstatus again; advance sepc; sret -> S main.
#   S main (resumed): read sstatus; execute an ebreak (NOT delegated) -> M.
#   M handler: sees it is the finish path -> drop to M-mode and terminate
#     with the single ECALL-from-M (the trace stop point).
#
# IMPORTANT: there is exactly ONE `ecall` in the whole program (the M-mode
# terminator). Every mid-stream trap uses an illegal instruction or `ebreak`,
# because the lockstep trace converters stop at the first `ecall` (0x00000073)
# on ALL sims. Every sstatus/mstatus snapshot and every post-xRET sentinel is
# written into a dedicated integer GPR so it lands in the diffed trace.
#
# Avoid '/' followed by '*' anywhere (cpp).
#
# Dedicated GPRs (written once, never reused):
#   s2  = sstatus in S main, before the delegated illegal instr
#   s3  = sstatus at top of S handler   (expect SPP=1, SPIE=prior SIE)
#   s4  = mstatus at top of M handler   (expect MPP=S=1, SPP/SPIE live from S)
#   s5  = mcause in M handler (phase-1)  (expect 3 = breakpoint)
#   s6  = sentinel set in S handler after the nested-M round trip
#   s7  = sstatus in S handler after the M round trip
#   s8  = scause in S handler           (expect 2 = illegal instruction)
#   s9  = sentinel set in S main after the sret (back in S)
#   s10 = sstatus in S main after the sret
#   s11 = mcause in M handler on the finishing breakpoint (expect 3)
#   s1  = phase flag (0 boot, 1 nested corner, 2 finish)

.section .text
.globl _start
.option norvc
_start:
    la      t0, _mtrap
    csrw    mtvec, t0
    la      t0, _strap
    csrw    stvec, t0
    la      sp, _stack_top

    # Delegate ILLEGAL-INSTRUCTION (cause 2, bit 2 = 0x4) to S.
    # Breakpoint (cause 3) is intentionally left un-delegated.
    li      t0, 0x4
    csrw    medeleg, t0

    # Clear sstatus.SIE (bit 1) so SPIE on the S trap is a known 0.
    li      t0, 0x2
    csrc    sstatus, t0

    li      s1, 1                  # phase = 1 : nested-corner pass

    # mstatus.MPP = S (01) so mret enters S-mode.
    li      t0, 0x1800            # MPP field (bits 12:11)
    csrc    mstatus, t0
    li      t0, 0x0800            # MPP = 01 (S)
    csrs    mstatus, t0
    la      t0, _smain
    csrw    mepc, t0
    mret                          # --> _smain in S-mode

1:  j       1b

# ----------------------------------------------------------------------------
# S-mode main line.
# ----------------------------------------------------------------------------
_smain:
    csrr    s2, sstatus           # sstatus seen in S (fresh, SPP=0)
    .word   0xC0001073            # illegal CSR write (unimp csr) -> delegated S

_safter:
    # Resumed here after the S handler sret's back.
    li      s9, 0x5A5A            # post-sret sentinel (back in S main)
    csrr    s10, sstatus          # sstatus after the sret
    li      s1, 2                 # phase = 2 : finish
    ebreak                        # NOT delegated -> M handler -> terminate

1:  j       1b

# ----------------------------------------------------------------------------
# S-mode trap handler (stvec). Reached for the delegated illegal instruction.
# ----------------------------------------------------------------------------
.align 4
_strap:
    csrr    s8, scause            # expect 2 (illegal instruction)
    csrr    s3, sstatus           # SPP=1 (from S), SPIE = prior SIE (0)

    # From inside the S handler, take an M-only trap: ebreak (cause 3, not
    # delegated) re-enters M while SPP/SPIE hold S-handler state.
    ebreak                        # --> M handler (nested)

_sresume:
    # Resumed here after the M handler mret's back into the S handler.
    li      s6, 0x6B6B            # sentinel: returned from the nested M trap
    csrr    s7, sstatus           # sstatus after the M round trip

    # Advance sepc past the original illegal instr (4 bytes) and sret to S main.
    csrr    t0, sepc
    addi    t0, t0, 4
    csrw    sepc, t0
    sret                          # --> _safter in S-mode

1:  j       1b

# ----------------------------------------------------------------------------
# M-mode trap handler (mtvec).
# ----------------------------------------------------------------------------
.align 4
_mtrap:
    csrr    t1, mcause
    li      t0, 11
    beq     t1, t0, _mhalt        # ECALL-from-M -> HTIF exit + spin

    li      t0, 2
    beq     s1, t0, _mfinish      # phase 2 (finishing ebreak) -> terminate

    # ---- phase 1 nested corner: ebreak taken from inside the S handler ----
    csrr    s5, mcause            # expect 3 (breakpoint)
    csrr    s4, mstatus           # MPP=S=1, plus live SPP/SPIE from S
    # Advance mepc past the ebreak (4 bytes) and mret back to the S handler.
    csrr    t0, mepc
    addi    t0, t0, 4
    csrw    mepc, t0
    mret                          # --> _sresume

_mfinish:
    # Now in M-mode (the finishing ebreak trapped here). Record the last
    # snapshot, then terminate with the single ECALL-from-M (trace stop).
    csrr    s11, mcause           # the finishing breakpoint (expect 3)
    li      gp, 1
    ecall                         # THE single ecall: trace stop point

_mhalt:
    la      t1, tohost
    li      gp, 1
    sw      gp, 0(t1)             # HTIF exit marker (value 1)
1:  j       1b

.section .data
.align 8
_scratch: .dword 0
.align 12
_stack:   .rept 256
          .dword 0
          .endr
_stack_top:

.pushsection .tohost,"aw",@progbits
.align 6
.globl tohost
tohost:   .dword 0
.globl fromhost
fromhost: .dword 0
.popsection
