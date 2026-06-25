# Directed lockstep torture test (rv64): fp_div_under_trap_flush
#
# Corner (spec rank22): fp-flush-during-cvfpu-vs-precise-trap.
# Start a long-latency FP op (fdiv.d / fsqrt.d) so it is still in-flight in the
# CVFPU pipe, then take a PRECISE TRAP from a YOUNGER instruction a few slots
# later. The trap recovery must flush the pipeline -- including the in-flight
# long-latency op -- and on resume the divide/sqrt destination physreg + its
# active_id must be freed/recycled correctly. If the OoO core mis-handles the
# in-flight op across the flush (stale branch_mask, active_id reuse, dangling
# physreg, or a lost writeback), the result shows up as a wrong integer GPR or a
# hang. We make the FP results OBSERVABLE by converting them to integer GPRs
# (fcvt.l.d / fcvt.w.d) and by reading fflags, AFTER the trap returns.
#
# Trap source: an ILLEGAL INSTRUCTION (.word 0) placed right after the long-
# latency op. It is a precise trap that is deterministic and identical on Spike,
# Sail, and niigo (mcause=2), so the only thing under test is the flush of the
# in-flight FP op -- no PMP/page-table/memory-map oracle ambiguity. The handler
# records cause/epc into dedicated s-regs, advances mepc past the faulting word,
# and mret-resumes. Terminates with `li gp,1 ; ecall` (mcause==11 -> HTIF exit).
#
# Observability: every effect we care about is landed in an integer GPR before
# the terminating ecall (the lockstep oracle only diffs integer-GPR writes):
#   - fdiv.d/fsqrt.d results       -> fcvt.l.d / fcvt.w.d into x-regs
#   - fflags after the FP ops      -> csrr into an x-reg
#   - trap cause/epc               -> csrr in the handler into s-regs

.section .text
.globl _start
.option norvc
_start:
    la      t0, _trap
    csrw    mtvec, t0
    la      sp, _stack_top

    # clear the dedicated trap-record regs so they trace deterministically even
    # when (hypothetically) the trap is skipped.
    li      s2, 0           # mcause record
    li      s3, 0           # mepc record
    li      s4, 0           # trap-taken counter

    # enable the FPU (mstatus.FS = Initial) so FP ops are legal in M-mode.
    li      t1, (1 << 13)   # MSTATUS.FS[14:13] = 01 (Initial)
    csrs    mstatus, t1

    # ---- load FP operands from .data (deterministic bit patterns) ----------
    la      t2, _fp_ops
    fld     fa1, 0(t2)      # dividend / sqrt operand  (= 49.0)
    fld     fa2, 8(t2)      # divisor                  (= 7.0)
    fld     fa3, 16(t2)     # second dividend          (= 1000.0)
    fld     fa4, 24(t2)     # second divisor           (= 8.0)

    # =====================================================================
    # Block A: fdiv.d in-flight, then an illegal instruction faults YOUNGER.
    # The precise trap from the illegal word must flush the pipe while the
    # fdiv (long latency) is still executing in CVFPU. fa0 must hold the
    # correct quotient AFTER the trap returns and the pipe drains.
    # =====================================================================
    fdiv.d  fa0, fa1, fa2       # 49.0 / 7.0 = 7.0  (long latency, in-flight)
    fdiv.d  fa5, fa3, fa4       # 1000.0 / 8.0 = 125.0 (second in-flight op)
    .word   0x00000000          # ILLEGAL INSTRUCTION -> precise trap (mcause=2)
    # ----- resume point after mret (mepc advanced past the .word) -----------
    fcvt.l.d s5, fa0            # quotient #1 -> int : expect 7
    fcvt.l.d s6, fa5           # quotient #2 -> int : expect 125

    # =====================================================================
    # Block B: fsqrt.d (the LONGEST CVFPU latency) in-flight, then trap.
    # =====================================================================
    fsqrt.d fa0, fa1            # sqrt(49.0) = 7.0  (longest latency, in-flight)
    fsqrt.d fa5, fa4           # sqrt(8.0) ~= 2.828 (second in-flight op)
    .word   0x00000000          # ILLEGAL INSTRUCTION -> precise trap (mcause=2)
    # ----- resume point -----------------------------------------------------
    fcvt.l.d s7, fa0           # sqrt #1 -> int (rtz default? rounding via rm) : 7
    fcvt.w.d s8, fa5           # sqrt(8.0) ~= 2.828 -> int (rne) : 3

    # =====================================================================
    # Block C: divw (integer long-latency div) in-flight, then trap. Same
    # corner on the integer divider path.
    # =====================================================================
    li      t3, 1000
    li      t4, 8
    div     s9, t3, t4         # 1000/8 = 125 (in-flight on the div unit)
    rem     s10, t3, t4        # 1000%8 = 0   (second in-flight div op)
    .word   0x00000000          # ILLEGAL INSTRUCTION -> precise trap (mcause=2)
    # ----- resume point -----------------------------------------------------
    # s9 = 125, s10 = 0 already hold the integer-div results (traced directly).

    # ---- read fflags so the FP exception state is observable in a GPR ------
    csrr    s11, fflags

    # ---- terminate ---------------------------------------------------------
    li      gp, 1
    ecall
1:  j       1b

.align 4
_trap:
    # mcause==11 (ECALL-from-M) -> HTIF exit. Else: a mid-stream precise trap;
    # record cause/epc, advance mepc past the 4-byte faulting word, mret-resume.
    csrr    t0, mcause
    li      t1, 11
    beq     t0, t1, _htif_exit

    # record the trap (overwrite each time; deterministic since identical traps)
    mv      s2, t0              # last mcause  (expect 2 = illegal instruction)
    csrr    s3, mepc            # faulting pc
    addi    s4, s4, 1           # count traps taken
    csrr    t2, mepc
    addi    t2, t2, 4           # skip the 4-byte illegal word
    csrw    mepc, t2
    mret

_htif_exit:
    la      t1, tohost
    sw      gp, 0(t1)           # HTIF exit, value 1
1:  j       1b

.section .data
.align 8
_fp_ops:
    .dword  0x4048800000000000   # 49.0   (double)
    .dword  0x401C000000000000   # 7.0    (double)
    .dword  0x408F400000000000   # 1000.0 (double)
    .dword  0x4020000000000000   # 8.0    (double)

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
