# Directed test: exc_precedence_fetchgroup
#
# Corner: exception precedence WITHIN a 16-byte fetch group (rank37).
# The OoO core fetches a 16-byte block (4 lanes). When several instructions
# in the SAME aligned group fault, the architecturally-correct behavior is
# that the OLDEST (lowest-PC) faulting instruction takes its trap first, in
# strict program order; younger lanes are squashed and re-fetched after the
# handler resumes. Each faulting instruction, when it becomes the trap, must
# report ITS OWN (mcause, mepc).
#
# RTL under test:
#   - src/ooo_fetch_decode.sv:50-52  prefix_fault (older-lane-wins squash mask)
#   - src/ooo_fetch_decode.sv:80-91  fault-carrier = block_fault & !prefix_fault
#   - src/riscv_core_ooo.sv:1945     in-order commit trap (active_commit_packet)
#
# Faults used: ILLEGAL INSTRUCTION (cause 2) and EBREAK (cause 3) -- both are
# fully deterministic in bare M-mode (no paging / no PMP), so Spike and Sail
# agree on (cause, epc). Mid-stream traps are NOT ecall (ecall stops the
# trace); the only ecall is the terminating one.
#
# Observability: the handler records (mcause, mepc-relative-to-_start) for each
# trap into a memory log indexed by a monotonic trap counter, then advances
# mepc by 4 and mret-resumes. After the deterministic body, every logged value
# is loaded back into an integer GPR (ld) so the FULL trap sequence -- order,
# cause, and which instruction -- appears in the diffed integer-GPR trace.
#
# Register usage (preserved across the whole body; the handler must not clobber
# anything the mainline needs):
#   tp (x4)  = base of the trap log in .data (set once, never reused)
#   s11(x27) = monotonic trap counter (number of traps taken so far)
# The handler only touches t0/t1/t2 (scratch) plus tp/s11; the mainline body
# does not use t0/t1/t2 across a faulting group, so this is safe.

.section .text
.globl _start
.option norvc
_start:
    la      t0, _trap
    csrw    mtvec, t0
    la      sp, _stack_top

    la      tp, _trap_log          # tp = trap-log base (cause,epc pairs)
    li      s11, 0                 # s11 = trap counter

    # ===================================================================
    # Group A: first faulting lane = 0.  All four lanes are 16-byte
    # aligned into one fetch block. Lane0 illegal, lanes 1..3 valid.
    # Older-lane-wins => lane0 (illegal) traps first; after resume the
    # remaining lanes (valid) execute normally, no further trap here.
    # ===================================================================
    .p2align 4
groupA:
    .word   0x00000000             # lane0: illegal (all-zero) -> cause 2
    addi    a0, x0, 0x11           # lane1: valid
    addi    a1, x0, 0x22           # lane2: valid
    addi    a2, x0, 0x33           # lane3: valid

    # ===================================================================
    # Group B: first faulting lane = 1.  Lane0 valid, lane1 illegal,
    # lanes 2..3 valid. prefix_fault must keep lane0 (older) live and
    # pick lane1 as the carrier.
    # ===================================================================
    .p2align 4
groupB:
    addi    a3, x0, 0x44           # lane0: valid
    .word   0x00000000             # lane1: illegal -> cause 2
    addi    a4, x0, 0x55           # lane2: valid
    addi    a5, x0, 0x66           # lane3: valid

    # ===================================================================
    # Group C: first faulting lane = 2.
    # ===================================================================
    .p2align 4
groupC:
    addi    a6, x0, 0x77           # lane0: valid
    addi    a7, x0, 0x18           # lane1: valid
    .word   0x00000000             # lane2: illegal -> cause 2
    addi    t3, x0, 0x19           # lane3: valid

    # ===================================================================
    # Group D: first faulting lane = 3.
    # ===================================================================
    .p2align 4
groupD:
    addi    t4, x0, 0x1a           # lane0: valid
    addi    t5, x0, 0x1b           # lane1: valid
    addi    t6, x0, 0x1c           # lane2: valid
    .word   0x00000000             # lane3: illegal -> cause 2

    # ===================================================================
    # Group E: TWO faults in one group (lanes 1 and 3).  Older-lane-wins
    # => lane1 traps first (cause 2). After the handler resumes at
    # lane1+4, lane2 executes, then lane3 (still illegal) traps SECOND
    # with ITS OWN mepc. This is the core precedence ordering check.
    # ===================================================================
    .p2align 4
groupE:
    addi    s1, x0, 0x1d           # lane0: valid
    .word   0x00000000             # lane1: illegal -> cause 2 (first)
    addi    s2, x0, 0x1e           # lane2: valid
    .word   0x00000000             # lane3: illegal -> cause 2 (second)

    # ===================================================================
    # Group F: mixed cause types in one group.  lane1 EBREAK (cause 3),
    # lane2 illegal (cause 2).  Confirms the carrier reports the cause of
    # the actual faulting lane (not a fixed cause), and that after EBREAK
    # resumes, the younger illegal traps with cause 2.
    # ===================================================================
    .p2align 4
groupF:
    addi    s3, x0, 0x1f           # lane0: valid
    ebreak                         # lane1: breakpoint -> cause 3 (first)
    .word   0x00000000             # lane2: illegal   -> cause 2 (second)
    addi    s4, x0, 0x20           # lane3: valid

    # ===================================================================
    # Make the recorded trap sequence observable in the integer trace.
    # Load each (cause, epc-offset) pair back from the log into GPRs.
    # 8 traps expected: A,B,C,D (1 each) + E (2) + F (2).
    # ===================================================================
    la      tp, _trap_log
    ld      a0,  0(tp)             # trap0 cause
    ld      a1,  8(tp)             # trap0 epc-offset
    ld      a2, 16(tp)             # trap1 cause
    ld      a3, 24(tp)             # trap1 epc-offset
    ld      a4, 32(tp)             # trap2 cause
    ld      a5, 40(tp)             # trap2 epc-offset
    ld      a6, 48(tp)             # trap3 cause
    ld      a7, 56(tp)             # trap3 epc-offset
    ld      s1, 64(tp)             # trap4 cause
    ld      s2, 72(tp)             # trap4 epc-offset
    ld      s3, 80(tp)             # trap5 cause
    ld      s4, 88(tp)             # trap5 epc-offset
    ld      s5, 96(tp)             # trap6 cause
    ld      s6,104(tp)             # trap6 epc-offset
    ld      s7,112(tp)             # trap7 cause
    ld      s8,120(tp)             # trap7 epc-offset
    ld      s9,128(tp)             # total trap count (written by handler)

    # ===================================================================
    # Terminate. mcause==11 (ECALL-from-M) in the handler -> HTIF exit.
    # ===================================================================
    li      gp, 1
    ecall
1:  j       1b

# -----------------------------------------------------------------------
# Trap handler. Deterministic and identical on all three sims.
#   - mcause==11 (ECALL-from-M): write gp to tohost (terminate).
#   - else (illegal / ebreak): log (cause, mepc-_start) at log[counter],
#     bump counter, advance mepc by 4, mret.
# Uses t0/t1/t2 + tp + s11 only.
# -----------------------------------------------------------------------
.p2align 4
_trap:
    csrr    t0, mcause
    li      t1, 11
    beq     t0, t1, _term         # ECALL-from-M -> terminate

    # Record (cause, epc-offset) into the log at index s11.
    slli    t2, s11, 4            # byte offset = counter * 16
    add     t2, tp, t2
    sd      t0, 0(t2)             # log[i].cause = mcause
    csrr    t1, mepc
    la      t0, _start
    sub     t1, t1, t0           # epc relative to _start (small, sim-stable)
    sd      t1, 8(t2)            # log[i].epc_off = mepc - _start
    addi    s11, s11, 1          # counter++

    # Stash the running count at a fixed slot (slot 16 = byte 128).
    li      t0, 128
    add     t0, tp, t0
    sd      s11, 0(t0)

    # Advance past the faulting (4-byte) instruction and resume.
    csrr    t0, mepc
    addi    t0, t0, 4
    csrw    mepc, t0
    mret

_term:
    la      t1, tohost
    sd      gp, 0(t1)             # HTIF exit (value 1)
1:  j       1b

.section .data
.p2align 4
_trap_log:
    .rept 18
    .dword 0
    .endr

.p2align 12
_stack:
    .rept 256
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
