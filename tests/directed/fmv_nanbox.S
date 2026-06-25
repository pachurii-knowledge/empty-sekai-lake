# Directed lockstep test: FMV.X.W sign-extension, FMV.W.X NaN-boxing, and
# improperly-NaN-boxed FP32 source-operand canonicalization (rank20 corner).
#
# Corner (per spec fmv_nanbox.json, risk fmv-x-w-d-and-nanbox-corner):
#  (a) A non-NaN-boxed FP32 source (high half != 0xFFFFFFFF) read by FADD.S
#      and FCLASS.S must be treated as the canonical qNaN 0x7fc00000.
#  (b) FMV.X.W sign-extends bit 31 of the low word into XLEN.
#  (c) FMV.X.D moves all 64 bits (round-trip through FMV.D.X).
#
# Observability: every FP/box effect is landed in an integer GPR (x5..x8 plus
# x6 via fclass) BEFORE the terminating ecall, so the integer-GPR-write trace
# captures it on all three sims. No mid-stream traps; the handler just exits.
.section .text
.globl _start
.option norvc
_start:
    la      t0, _trap
    csrw    mtvec, t0
    la      sp, _stack_top
    li      x1, 0x6000            # mstatus.FS = Dirty (enable FP); no GPR-traced read
    csrs    mstatus, x1

    # --- (a) improperly-NaN-boxed FP32 source -> canonical qNaN -------------
    # f1 high half = 0x00000000 (NOT all-ones) => not a valid boxed FP32.
    li      x2, 0x0000000040490fdb
    fmv.d.x f1, x2                 # f1 = 0x0000000040490fdb (mis-boxed as FP32)
    fadd.s  f3, f1, f1            # FP32 op: source must be read as qNaN 0x7fc00000
    fmv.x.w x5, f3                # low 32b sign-ext; expect 0x000000007fc00000
    fclass.s x6, f1               # f1 as FP32 => qNaN class bit 9 => 0x200

    # --- (b) FMV.X.W sign-extension of a boxed negative single --------------
    li      x3, 0x00000000bf800000 # -1.0f bit pattern (low 32b)
    fmv.w.x f4, x3               # NaN-boxes: f4 = 0xFFFFFFFFBF800000
    fmv.x.w x7, f4               # bit31=1 => sign-extend => 0xFFFFFFFFBF800000

    # --- (c) FMV.X.D round-trip (all 64 bits) -------------------------------
    li      x9, 0x400921fb54442d18 # pi as IEEE-754 double
    fmv.d.x f2, x9
    fmv.x.d x8, f2               # expect 0x400921FB54442D18

    # --- terminate ---------------------------------------------------------
    li      gp, 1
    ecall
1:  j       1b

.align 4
_trap:
    la      t1, tohost
    sw      gp, 0(t1)              # HTIF exit, value 1
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
