# Directed lockstep test: ras_deep_clobber
#
# Corner: stress the RAS push/pop sequencing and the branch-mispredict
# recovery path (branch_stack restore + abort_mask squash + ras_count/ghr
# checkpoint restore) through control flow the RAS predictor mishandles:
#   - deep nested JAL-calls (RAS push) unwound by 'ret' (RAS pop),
#   - JALR-indirect calls via x1 (rd=x1) that are NEVER RAS-pushed
#     (lane_is_call requires PC_uncond == JAL only),
#   - calls/returns via the alternate link register x5 (never recognized
#     as call or return by the RAS logic),
#   - unpredictable LFSR-driven conditional branches that mispredict, so
#     RAS/GHR checkpoints get restored mid-stream,
#   - mul/div long-latency ops in the shadow of each call/return so the
#     control transfer resolves late.
# Architecturally every return target is fixed, so the integer-GPR trace
# is fully deterministic. A wrong return target, clobbered RAS, or hang
# under recovery would diverge from the golden models.
#
# Observability: every function accumulates a tag into a0 (running sum)
# and writes a per-depth marker into memory; we load the markers back and
# fold the LFSR final state + accumulator into GPRs before ecall.

.section .text
.globl _start
.option norvc
_start:
    la      t0, _trap
    csrw    mtvec, t0
    la      sp, _stack_top

    li      a0, 0                    # running accumulator (return-order witness)
    li      s11, 0xACE1              # LFSR state (16-bit), drives unpredictable beq
    la      s10, _markers            # base of per-depth markers in memory

    # ===================================================================
    # Section 1: deep nested JAL-call chain (RAS push), 12 deep, then
    # 'ret' unwind (RAS pop). Each frame multiplies/divides under the
    # shadow of the call so the return resolves late.
    # ===================================================================
    jal     ra, f1
    # after the whole chain unwinds, a0 holds the folded return-order sum

    # ===================================================================
    # Section 2: JALR-indirect call via x1 (rd=x1). decode -> PC_indirect,
    # so the RAS never pushes it; the matching 'ret' is RAS-mispredicted
    # against a stale/empty stack -> exercises recovery.
    # ===================================================================
    la      t0, g_indirect
    li      t6, 7
    mul     t5, t6, t6               # long-latency op in the call shadow
    jalr    ra, 0(t0)                # indirect call, ra-link, NOT RAS-pushed
    add     a0, a0, t5               # +49, fold result of shadow op

    # ===================================================================
    # Section 3: call/return through the ALTERNATE link register x5.
    # 'jalr x5, 0(t1)' is a call but rd=x5 (not x1) -> never RAS-call;
    # 'jr x5' == 'jalr x0,0(x5)' is a return but rs1=x5 (not x1) -> never
    # RAS-return. Both go through ITTAGE / pc+4 fallback + full recovery.
    # ===================================================================
    la      t1, h_altlink
    jalr    x5, 0(t1)                # call via x5 link, not RAS-pushed
    addi    a0, a0, 100              # witness we came back to the right spot

    # ===================================================================
    # Section 4: unpredictable LFSR branches interleaved with a short
    # nested call chain, so RAS/GHR checkpoints restore while RAS state is
    # mid-flight. Loop a fixed number of iterations (deterministic).
    # ===================================================================
    li      t2, 16                   # iteration count
sec4_loop:
    # advance 16-bit Fibonacci LFSR: bit = (s ^ (s>>2) ^ (s>>3) ^ (s>>5))&1
    srli    t3, s11, 0
    srli    t4, s11, 2
    xor     t3, t3, t4
    srli    t4, s11, 3
    xor     t3, t3, t4
    srli    t4, s11, 5
    xor     t3, t3, t4
    andi    t3, t3, 1                # new bit
    srli    s11, s11, 1
    slli    t4, t3, 15
    or      s11, s11, t4             # shift in new bit at top
    li      t4, 0xFFFF
    and     s11, s11, t4             # mask to 16 bits

    # unpredictable branch on the new bit; both sides do a nested call so
    # the RAS is exercised on either path
    andi    t3, s11, 1
    beq     t3, x0, sec4_even
    jal     ra, k_odd                # taken path: nested call
    j       sec4_after
sec4_even:
    jal     ra, k_even               # not-taken path: different nested call
sec4_after:
    addi    t2, t2, -1
    bne     t2, x0, sec4_loop

    # ===================================================================
    # Fold the persisted per-depth markers back into integer GPRs so the
    # RAS-unwound store effects are observable in the trace.
    # ===================================================================
    ld      a1, 0(s10)               # marker[0]
    ld      a2, 8(s10)               # marker[1]
    ld      a3, 16(s10)              # marker[2]
    ld      a4, 24(s10)              # marker[3]
    ld      a5, 32(s10)              # marker[4]
    add     a0, a0, a1
    add     a0, a0, a2
    add     a0, a0, a3
    add     a0, a0, a4
    add     a0, a0, a5
    add     a0, a0, s11              # fold final LFSR state

    # --- terminate ---------------------------------------------------------
    li      gp, 1
    ecall
1:  j       1b

# =======================================================================
# Section 1 frames: a 12-deep JAL-call chain. Each fN calls f(N+1), and
# on the way back accumulates a tag. f12 is the leaf. A mul/div sits in
# each frame so the return resolves under a long-latency shadow.
# =======================================================================
f1:
    addi    sp, sp, -8
    sd      ra, 0(sp)
    li      t6, 3
    mul     t5, t6, t6               # shadow op
    jal     ra, f2
    addi    a0, a0, 1
    sd      a0, 0(s10)               # marker[0]
    ld      ra, 0(sp)
    addi    sp, sp, 8
    ret

f2:
    addi    sp, sp, -8
    sd      ra, 0(sp)
    jal     ra, f3
    addi    a0, a0, 2
    sd      a0, 8(s10)               # marker[1]
    ld      ra, 0(sp)
    addi    sp, sp, 8
    ret

f3:
    addi    sp, sp, -8
    sd      ra, 0(sp)
    li      t6, 9
    li      t4, 4
    div     t5, t6, t4               # shadow op
    jal     ra, f4
    addi    a0, a0, 4
    sd      a0, 16(s10)              # marker[2]
    ld      ra, 0(sp)
    addi    sp, sp, 8
    ret

f4:
    addi    sp, sp, -8
    sd      ra, 0(sp)
    jal     ra, f5
    addi    a0, a0, 8
    sd      a0, 24(s10)              # marker[3]
    ld      ra, 0(sp)
    addi    sp, sp, 8
    ret

f5:
    addi    sp, sp, -8
    sd      ra, 0(sp)
    jal     ra, f6
    addi    a0, a0, 16
    sd      a0, 32(s10)              # marker[4]
    ld      ra, 0(sp)
    addi    sp, sp, 8
    ret

f6:
    addi    sp, sp, -8
    sd      ra, 0(sp)
    li      t6, 5
    mul     t5, t6, t6               # shadow op
    jal     ra, f7
    addi    a0, a0, 32
    ld      ra, 0(sp)
    addi    sp, sp, 8
    ret

f7:
    addi    sp, sp, -8
    sd      ra, 0(sp)
    jal     ra, f8
    addi    a0, a0, 64
    ld      ra, 0(sp)
    addi    sp, sp, 8
    ret

f8:
    addi    sp, sp, -8
    sd      ra, 0(sp)
    jal     ra, f9
    addi    a0, a0, 128
    ld      ra, 0(sp)
    addi    sp, sp, 8
    ret

f9:
    addi    sp, sp, -8
    sd      ra, 0(sp)
    li      t6, 13
    li      t4, 3
    div     t5, t6, t4               # shadow op
    jal     ra, f10
    addi    a0, a0, 256
    ld      ra, 0(sp)
    addi    sp, sp, 8
    ret

f10:
    addi    sp, sp, -8
    sd      ra, 0(sp)
    jal     ra, f11
    addi    a0, a0, 512
    ld      ra, 0(sp)
    addi    sp, sp, 8
    ret

f11:
    addi    sp, sp, -8
    sd      ra, 0(sp)
    jal     ra, f12
    addi    a0, a0, 1024
    ld      ra, 0(sp)
    addi    sp, sp, 8
    ret

f12:
    # leaf: no further call
    li      t6, 2048
    add     a0, a0, t6
    ret

# =======================================================================
# Section 2 target: reached via JALR-indirect call (ra-link, not RAS
# pushed). It itself does a nested JAL-call then returns; the inner ret
# IS a proper RAS pop, but the outer return from g_indirect is the one
# the RAS never pushed -> recovery on the outer ret.
# =======================================================================
g_indirect:
    addi    sp, sp, -8
    sd      ra, 0(sp)
    li      t6, 11
    li      t4, 2
    div     t5, t6, t4               # shadow op
    jal     ra, g_inner
    ld      ra, 0(sp)
    addi    sp, sp, 8
    ret

g_inner:
    addi    a0, a0, 1
    ret

# =======================================================================
# Section 3 target: entered via 'jalr x5' (x5 link). Returns via 'jr x5'
# (== jalr x0,0(x5)), which the RAS never recognizes as a return.
# Must preserve x5 across the body (no clobber).
# =======================================================================
h_altlink:
    addi    a0, a0, 1
    li      t6, 6
    mul     t5, t6, t6               # shadow op (does not touch x5)
    jr      x5                       # return via alternate link reg

# =======================================================================
# Section 4 nested-call targets (proper JAL-call/ret pairs), invoked from
# inside the unpredictable-branch loop.
# =======================================================================
k_odd:
    addi    sp, sp, -8
    sd      ra, 0(sp)
    li      t6, 7
    mul     t5, t6, s11              # shadow op uses live LFSR state
    jal     ra, k_leaf
    addi    a0, a0, 3
    ld      ra, 0(sp)
    addi    sp, sp, 8
    ret

k_even:
    addi    sp, sp, -8
    sd      ra, 0(sp)
    jal     ra, k_leaf
    addi    a0, a0, 5
    ld      ra, 0(sp)
    addi    sp, sp, 8
    ret

k_leaf:
    addi    a0, a0, 1
    ret

.align 4
_trap:
    # No intentional mid-stream traps in this test: handler just HTIF-exits.
    la      t1, tohost
    sw      gp, 0(t1)                # HTIF exit, value 1
1:  j       1b

.section .data
.align 8
_markers: .rept 8
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
