# Directed lockstep torture test: fcvt_corners (rank19)
#
# Corner: FCVT.W/WU/L/LU.{S,D} out-of-range / NaN / +-Inf saturation, plus the
# RV64 L-form 64-bit-result extension and the (subtle) sign-extension of the
# unsigned 32-bit WU result on RV64. RISC-V mandates EXACT saturation values:
#   NaN  -> signed max (2^31-1 / 2^63-1), unsigned max (2^32-1 / 2^64-1)
#   +Inf -> signed/unsigned max
#   -Inf -> signed min (-2^31 / -2^63), unsigned 0
#   in-range -> rounded value
# RTL risk: niigo_fp_unit.sv:430-431 packet_for_fpnew extension split
#   is_l_cvt ? XLEN'(result) : XLEN'($signed(result[31:0]))
# and the int_fmt / op_mod / is_l_cvt selection upstream (lines 236-244,321-332).
#
# Every FP->GPR conversion lands its result directly in an integer GPR, which is
# the traced quantity. The body runs in M-mode, no traps expected; terminates
# with li gp,1 ; ecall into the HTIF-exit trap handler.
#
# NOTE: no '/' followed by '*' anywhere (cpp would treat it as a C comment).

.section .text
.globl _start
.option norvc
_start:
    la      t0, _trap
    csrw    mtvec, t0
    la      sp, _stack_top

    # enable FP (mstatus.FS = Initial=01) so FP ops do not trap as illegal.
    li      t0, (1 << 13)
    csrs    mstatus, t0

    la      s0, _ops                # base of 64-bit operand bit-patterns
    la      s1, _results            # base of result store area
    li      s2, 0                   # store cursor (byte offset)

    # ---------------------------------------------------------------------
    # SINGLE-PRECISION (.S) operands. Each is a 32-bit float bit pattern.
    # We load the 32-bit pattern, fmv.w.x it into f1, then run all four
    # FP->GPR conversions and record each integer result.
    # ---------------------------------------------------------------------

    # qNaN  (0x7FC00000)
    li      t1, 0x7FC00000
    fmv.w.x f1, t1
    call    _emit_s

    # +Inf  (0x7F800000)
    li      t1, 0x7F800000
    fmv.w.x f1, t1
    call    _emit_s

    # -Inf  (0xFF800000)
    li      t1, 0xFFFFFFFFFF800000   # sign-extended; only low 32 used by fmv.w.x
    fmv.w.x f1, t1
    call    _emit_s

    # +max finite single (0x7F7FFFFF ~ 3.4028e38)
    li      t1, 0x7F7FFFFF
    fmv.w.x f1, t1
    call    _emit_s

    # +2^31 - 128 = 2147483520.0 (0x4EFFFFFF), largest single < 2^31
    li      t1, 0x4EFFFFFF
    fmv.w.x f1, t1
    call    _emit_s

    # +2^31 = 2147483648.0 (0x4F000000) -> out of range for W (saturates)
    li      t1, 0x4F000000
    fmv.w.x f1, t1
    call    _emit_s

    # +2^32 = 4294967296.0 (0x4F800000) -> OOR for W and WU
    li      t1, 0x4F800000
    fmv.w.x f1, t1
    call    _emit_s

    # +2^63 = 9.223372e18 (0x5F000000) -> OOR for L
    li      t1, 0x5F000000
    fmv.w.x f1, t1
    call    _emit_s

    # +2^64 = 1.8446744e19 (0x5F800000) -> OOR for L and LU
    li      t1, 0x5F800000
    fmv.w.x f1, t1
    call    _emit_s

    # small negative -1.5 (0xBFC00000) -> WU/LU must saturate to 0
    li      t1, 0xFFFFFFFFBFC00000
    fmv.w.x f1, t1
    call    _emit_s

    # large negative -2^40 (0xD3800000) -> W saturates to min, WU to 0,
    # L in-range negative, LU saturates to 0
    li      t1, 0xFFFFFFFFD3800000
    fmv.w.x f1, t1
    call    _emit_s

    # ---------------------------------------------------------------------
    # DOUBLE-PRECISION (.D) operands. Each is a 64-bit double bit pattern,
    # fmv.d.x into f1, then all four FP->GPR conversions.
    # ---------------------------------------------------------------------

    # qNaN double (0x7FF8000000000000)
    li      t1, 0x7FF8000000000000
    fmv.d.x f1, t1
    call    _emit_d

    # +Inf double (0x7FF0000000000000)
    li      t1, 0x7FF0000000000000
    fmv.d.x f1, t1
    call    _emit_d

    # -Inf double (0xFFF0000000000000)
    li      t1, 0xFFF0000000000000
    fmv.d.x f1, t1
    call    _emit_d

    # +max finite double (0x7FEFFFFFFFFFFFFF)
    li      t1, 0x7FEFFFFFFFFFFFFF
    fmv.d.x f1, t1
    call    _emit_d

    # 0xFFFFFFFF.0 = 4294967295.0 (in WU range, OOR for W).
    # double bit pattern of 4294967295.0 = 0x41EFFFFFFFE00000.
    # KEY corner: FCVT.WU.D result 0xFFFFFFFF must be SIGN-EXTENDED on RV64
    # -> 0xFFFFFFFFFFFFFFFF, per niigo_fp_unit.sv:431.
    li      t1, 0x41EFFFFFFFE00000
    fmv.d.x f1, t1
    call    _emit_d

    # +2^31 = 2147483648.0 (0x41E0000000000000) -> exactly OOR for W
    li      t1, 0x41E0000000000000
    fmv.d.x f1, t1
    call    _emit_d

    # +2^31 - 1 = 2147483647.0 (0x41DFFFFFFFC00000) -> max in-range for W;
    # FCVT.W.D = 0x7FFFFFFF sign-extended.
    li      t1, 0x41DFFFFFFFC00000
    fmv.d.x f1, t1
    call    _emit_d

    # +2^63 = 9.223372036854776e18 (0x43E0000000000000) -> OOR for L
    li      t1, 0x43E0000000000000
    fmv.d.x f1, t1
    call    _emit_d

    # +2^63 - 1024 = largest double < 2^63 (0x43DFFFFFFFFFFFFF) -> in range L
    li      t1, 0x43DFFFFFFFFFFFFF
    fmv.d.x f1, t1
    call    _emit_d

    # +2^64 = 1.8446744073709552e19 (0x43F0000000000000) -> OOR for L and LU
    li      t1, 0x43F0000000000000
    fmv.d.x f1, t1
    call    _emit_d

    # small negative -1.5 double (0xBFF8000000000000) -> WU/LU saturate to 0,
    # W/L produce -1 (RTZ) or rounded value
    li      t1, 0xBFF8000000000000
    fmv.d.x f1, t1
    call    _emit_d

    # large negative -2^40 double (0xC270000000000000) -> W min, WU 0,
    # L in-range negative, LU 0
    li      t1, 0xC270000000000000
    fmv.d.x f1, t1
    call    _emit_d

    # ---------------------------------------------------------------------
    # Read back all stored results into GPRs (load-back makes the stored
    # integer effects re-appear in the trace, belt-and-suspenders; the
    # conversions themselves already wrote traced integer GPRs above).
    # Also capture fflags to observe the NV flag accumulation.
    # ---------------------------------------------------------------------
    csrr    s4, fflags

    la      t2, _results
    ld      s5, 0(t2)
    ld      s6, 8(t2)
    ld      s7, 16(t2)
    ld      s8, 24(t2)
    ld      s9, 32(t2)
    ld      s10, 40(t2)

    # --- terminate ---------------------------------------------------------
    li      gp, 1
    ecall
1:  j       1b

# -------------------------------------------------------------------------
# _emit_s: f1 holds a single-precision operand. Run the four FP->GPR
# conversions of the .S group, leaving each in a distinct integer GPR
# (a0..a3) and storing them to the result area. Returns via ra.
# -------------------------------------------------------------------------
_emit_s:
    fcvt.w.s  a0, f1, rtz
    fcvt.wu.s a1, f1, rtz
    fcvt.l.s  a2, f1, rtz
    fcvt.lu.s a3, f1, rtz
    add       t3, s1, s2
    sd        a0, 0(t3)
    sd        a1, 8(t3)
    sd        a2, 16(t3)
    sd        a3, 24(t3)
    addi      s2, s2, 32
    ret

# -------------------------------------------------------------------------
# _emit_d: f1 holds a double-precision operand. Run the four FP->GPR
# conversions of the .D group.
# -------------------------------------------------------------------------
_emit_d:
    fcvt.w.d  a0, f1, rtz
    fcvt.wu.d a1, f1, rtz
    fcvt.l.d  a2, f1, rtz
    fcvt.lu.d a3, f1, rtz
    add       t3, s1, s2
    sd        a0, 0(t3)
    sd        a1, 8(t3)
    sd        a2, 16(t3)
    sd        a3, 24(t3)
    addi      s2, s2, 32
    ret

.align 4
_trap:
    # In this test no traps are expected mid-stream. The ECALL terminator
    # lands here; just HTIF-exit with gp.
    la      t1, tohost
    sw      gp, 0(t1)
1:  j       1b

.section .data
.align 8
_ops:     .rept 64
          .dword 0
          .endr
.align 8
_results: .rept 256
          .dword 0
          .endr
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
