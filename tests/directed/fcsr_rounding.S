# Directed lockstep torture test: fcsr_rounding
#
# Corner: FRM rounding-mode selection + fflags OR-accumulate + the reserved
# rm-field illegal-instruction trap.
#  - For each frm in {RNE,RTZ,RDN,RUP,RMM} round a half-ULP tie (2.5 -> 2 or 3
#    etc.) several ways and observe the rounded result + fflags in integer GPRs.
#  - Exercise BOTH the static rm field (rm in the fcvt encoding) and the
#    dynamic rm=111 path (uses csr frm).
#  - Accumulate NX (inexact), DZ (divide by zero) and check the OR-accumulate
#    of fflags across several FP ops.
#  - Finally issue an fcvt.w.s whose rm field is the RESERVED value 101, which
#    per the RISC-V spec is an ILLEGAL INSTRUCTION (reserved dynamic rounding
#    mode in a static field) and must trap with mcause=2. The trap handler
#    records mcause and resumes.
#
# Observability: every FP result is funneled into an integer GPR via fcvt /
# csrr fflags / csrr fcsr; the illegal-instr cause is csrr'd in the handler.
# The lockstep oracle diffs integer-GPR writes against Spike and Sail.

.section .text
.globl _start
.option norvc
_start:
    la      t0, _trap
    csrw    mtvec, t0
    la      sp, _stack_top

    # ---- enable the FP unit: mstatus.FS = Dirty (11) so FP insns are legal.
    # Without this, FS=Off makes every FP instruction illegal on the goldens.
    li      t0, (3 << 13)          # MSTATUS.FS = 0b11 (Dirty)
    csrs    mstatus, t0

    # ---- load the test value 2.5f (0x40200000) into f1 -------------------
    la      t1, _f_two_half
    flw     f1, 0(t1)               # f1 = 2.5
    la      t1, _f_three_half
    flw     f2, 0(t1)               # f2 = 3.5
    la      t1, _f_big
    flw     f3, 0(t1)               # f3 = 1.0e30
    la      t1, _f_one
    flw     f4, 0(t1)               # f4 = 1.0
    la      t1, _f_zero
    flw     f5, 0(t1)               # f5 = 0.0

    # clear fflags so the accumulate starts from a known state
    csrwi   fflags, 0

    # ===================================================================
    # Part A: static rm-field fcvt.w.s of 2.5 under each rounding mode.
    # fcvt.w.s rounds to integer; 2.5 is an exact half-way tie -> the rm
    # matters: RNE->2, RTZ->2, RDN->2, RUP->3, RMM->3. Each is INEXACT (NX).
    # We use the STATIC rm field of fcvt so frm is irrelevant here.
    # ===================================================================
    csrwi   fflags, 0
    fcvt.w.s a0, f1, rne            # round to nearest, ties-even: 2.5 -> 2
    csrr     a1, fflags             # expect NX (0x01)

    csrwi   fflags, 0
    fcvt.w.s a2, f1, rtz            # toward zero: 2.5 -> 2
    csrr     a3, fflags

    csrwi   fflags, 0
    fcvt.w.s a4, f1, rdn            # down: 2.5 -> 2
    csrr     a5, fflags

    csrwi   fflags, 0
    fcvt.w.s a6, f1, rup           # up: 2.5 -> 3
    csrr     a7, fflags

    csrwi   fflags, 0
    fcvt.w.s s1, f1, rmm           # ties to max magnitude: 2.5 -> 3
    csrr     s2, fflags

    # ===================================================================
    # Part B: DYNAMIC rm (rm=111) fcvt.w.s of 3.5 -- result depends on the
    # csr frm. Set frm to each value, do a dynamic-rm fcvt, observe.
    # 3.5: RNE->4, RTZ->3, RDN->3, RUP->4, RMM->4.
    # ===================================================================
    csrwi   frm, 0                  # RNE
    csrwi   fflags, 0
    fcvt.w.s s3, f2, dyn           # dynamic rm -> frm=RNE: 3.5 -> 4
    csrr     s4, fcsr              # whole fcsr (frm<<5 | fflags)

    csrwi   frm, 1                  # RTZ
    fcvt.w.s s5, f2, dyn           # 3.5 -> 3

    csrwi   frm, 2                  # RDN
    fcvt.w.s s6, f2, dyn           # 3.5 -> 3

    csrwi   frm, 3                  # RUP
    fcvt.w.s s7, f2, dyn           # 3.5 -> 4

    csrwi   frm, 4                  # RMM
    fcvt.w.s s8, f2, dyn           # 3.5 -> 4

    # ===================================================================
    # Part C: fflags OR-accumulate across heterogeneous ops.
    # Start clean; do a div-by-zero (DZ), then an inexact add (NX), and
    # confirm fflags has accumulated both bits.
    # ===================================================================
    csrwi   fflags, 0
    fdiv.s   f6, f4, f5            # 1.0 / 0.0 -> +inf, sets DZ (0x08)
    csrr     s9, fflags            # expect DZ

    fadd.s   f7, f1, f4           # 2.5 + 1.0 = 3.5 (exact, no new flag)
    fmul.s   f8, f3, f3           # 1e30 * 1e30 -> +inf, OF+NX (0x14)
    csrr     s10, fflags          # expect DZ|OF|NX accumulated

    # land an integer view of f6/f8 too (convert inf -> INT_MAX, observable)
    fcvt.w.s s11, f6, rtz         # +inf -> 0x7fffffff, sets NV (0x10)
    csrr     t2, fflags           # now DZ|OF|NX|NV

    # ===================================================================
    # Part D: RESERVED rm field. fcvt.w.s with rm=101 is illegal; must trap
    # mcause=2 (illegal instruction). Encoded by hand so the assembler does
    # not reject it. Encoding of fcvt.w.s rd=x28(t3), rs1=f1, rm=101:
    #   funct7=1100000 rs2=00000 rs1=00001 rm=101 rd=11100 opcode=1010011
    #   = 1100000 00000 00001 101 11100 1010011
    # We mark t3 with a sentinel first; on a correct trap it stays sentinel.
    # ===================================================================
    li       t3, 0xdead            # sentinel; an illegal instr must NOT write t3
    .word    0xc000de53            # fcvt.w.s t3, f1, rm=101  (RESERVED -> illegal)
    # reaching here without a trap means t3 may have been wrongly written.
    csrr     t4, fflags

    # ---- terminate -------------------------------------------------------
    li      gp, 1
    ecall
1:  j       1b

.align 4
_trap:
    # ECALL-from-M (mcause=11) terminates; any other cause is the intentional
    # mid-stream illegal-instr trap: record mcause, advance mepc, mret.
    csrr    t5, mcause
    li      t6, 11
    beq     t5, t6, _htif_exit
    # record the illegal-instr cause into a dedicated reg (t5 already holds it)
    csrr    t5, mcause             # t5 = mcause (==2 for illegal instr)
    csrr    a0, mepc
    addi    a0, a0, 4              # skip the 4-byte faulting .word
    csrw    mepc, a0
    mret

_htif_exit:
    la      t1, tohost
    sw      gp, 0(t1)              # HTIF exit, value 1
1:  j       1b

.section .data
.align 8
_f_two_half:   .word 0x40200000    # 2.5f
_f_three_half: .word 0x40600000    # 3.5f
_f_big:        .word 0x7149f2ca    # ~1.0e30f
_f_one:        .word 0x3f800000    # 1.0f
_f_zero:       .word 0x00000000    # 0.0f
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
