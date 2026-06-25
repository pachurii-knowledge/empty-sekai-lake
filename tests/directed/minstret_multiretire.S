# Directed lockstep torture test (rv64): minstret multi-retire undercount.
#
# Corner (rank12, counter-csr-minstret-undercount): the 4-wide OoO core can
# retire up to 4 instructions in one cycle, but the priv-CSR minstret counter is
# incremented by a flat +1 whenever the single OR-reduced `retire` strobe is set
# (priv_csr_file.sv:447-448  `if (retire && !mcountinhibit_q[2]) minstret_q <=
# minstret_q + 1`, driven by `csr_retire = 1'b1` for ANY retiring lane in
# riscv_core_ooo.sv:2208-2210). When 2..4 lanes commit in the same cycle the
# counter undercounts by (lanes-1). Architecturally minstret must count EVERY
# retired instruction, so Spike and Sail both increment per-instruction.
#
# Observability: minstret is a CSR (invisible) and its absolute value also rides
# on niigo silently dropping `csrw minstret` (no write case in priv_csr_file.sv),
# so we measure a DELTA that cancels the base value and isolates the multi-retire
# undercount: read minstret before/after a deterministic block of K independent
# ALU ops and land (after - before) in an integer GPR (a0). Architectural delta =
# K + 1 (the K block insns + the first csrr's own retire) on both goldens; niigo
# returns a smaller delta iff it bundles retirements. We also land the raw
# post-block minstret (a1), the mcountinhibit.IR freeze delta (a2), and a second
# multi-retire delta over a different op mix (a3) as corroborating evidence.
.section .text
.globl _start
.option norvc
_start:
    la      t0, _trap
    csrw    mtvec, t0
    la      sp, _stack_top

    # ----------------------------------------------------------------------
    # Primary: delta of minstret across a block of K=64 INDEPENDENT addi ops.
    # Independent (distinct destinations, no RAW chains, no loads/branches) so
    # the ROB fills and the commit stage retires 4-wide -> niigo undercounts.
    # ----------------------------------------------------------------------
    csrr    s0, minstret            # base (before-block snapshot)
    # 64 independent adds: each writes a distinct arch reg from a fixed source.
    li      t1, 0
    addi    a4, t1, 1
    addi    a5, t1, 2
    addi    a6, t1, 3
    addi    a7, t1, 4
    addi    t2, t1, 5
    addi    t3, t1, 6
    addi    t4, t1, 7
    addi    t5, t1, 8
    addi    s2, t1, 9
    addi    s3, t1, 10
    addi    s4, t1, 11
    addi    s5, t1, 12
    addi    s6, t1, 13
    addi    s7, t1, 14
    addi    s8, t1, 15
    addi    s9, t1, 16
    addi    a4, t1, 17
    addi    a5, t1, 18
    addi    a6, t1, 19
    addi    a7, t1, 20
    addi    t2, t1, 21
    addi    t3, t1, 22
    addi    t4, t1, 23
    addi    t5, t1, 24
    addi    s2, t1, 25
    addi    s3, t1, 26
    addi    s4, t1, 27
    addi    s5, t1, 28
    addi    s6, t1, 29
    addi    s7, t1, 30
    addi    s8, t1, 31
    addi    s9, t1, 32
    addi    a4, t1, 33
    addi    a5, t1, 34
    addi    a6, t1, 35
    addi    a7, t1, 36
    addi    t2, t1, 37
    addi    t3, t1, 38
    addi    t4, t1, 39
    addi    t5, t1, 40
    addi    s2, t1, 41
    addi    s3, t1, 42
    addi    s4, t1, 43
    addi    s5, t1, 44
    addi    s6, t1, 45
    addi    s7, t1, 46
    addi    s8, t1, 47
    addi    s9, t1, 48
    addi    a4, t1, 49
    addi    a5, t1, 50
    addi    a6, t1, 51
    addi    a7, t1, 52
    addi    t2, t1, 53
    addi    t3, t1, 54
    addi    t4, t1, 55
    addi    t5, t1, 56
    addi    s2, t1, 57
    addi    s3, t1, 58
    addi    s4, t1, 59
    addi    s5, t1, 60
    addi    s6, t1, 61
    addi    s7, t1, 62
    addi    s8, t1, 63
    addi    s9, t1, 64
    csrr    s1, minstret            # after-block snapshot
    sub     a0, s1, s0              # PRIMARY: architectural delta = 65 (=64+1)

    # ----------------------------------------------------------------------
    # Secondary: raw post-block minstret value (absolute). Diverges from goldens
    # both from the multi-retire undercount AND niigo dropping csrw minstret.
    # ----------------------------------------------------------------------
    csrr    a1, minstret

    # ----------------------------------------------------------------------
    # mcountinhibit.IR freeze: set IR (bit2), run a block, read delta. Frozen ->
    # delta must be 0 on all sims (this validates the inhibit path is not the
    # source of any undercount divergence).
    # ----------------------------------------------------------------------
    li      t0, 0x4                 # mcountinhibit.IR
    csrw    mcountinhibit, t0
    csrr    s0, minstret
    addi    a4, t1, 100
    addi    a5, t1, 101
    addi    a6, t1, 102
    addi    a7, t1, 103
    addi    t2, t1, 104
    addi    t3, t1, 105
    addi    t4, t1, 106
    addi    t5, t1, 107
    csrr    s1, minstret
    sub     a2, s1, s0              # frozen delta -> 0 on all sims
    csrw    mcountinhibit, zero     # un-inhibit

    # ----------------------------------------------------------------------
    # Second multi-retire delta over a different independent op mix (add/sub/or/
    # and/sll/srl/xor/slt) to widen coverage of the commit-bundle undercount.
    # ----------------------------------------------------------------------
    li      t6, 0x55
    csrr    s0, minstret
    add     a4, t6, t1
    sub     a5, t6, t1
    or      a6, t6, t1
    and     a7, t6, t1
    sll     t2, t6, t1
    srl     t3, t6, t1
    xor     t4, t6, t1
    slt     t5, t6, t1
    add     s2, t6, t1
    sub     s3, t6, t1
    or      s4, t6, t1
    and     s5, t6, t1
    sll     s6, t6, t1
    srl     s7, t6, t1
    xor     s8, t6, t1
    slt     s9, t6, t1
    csrr    s1, minstret
    sub     a3, s1, s0              # architectural delta = 17 (=16+1)

    # --- terminate ---------------------------------------------------------
    li      gp, 1
    ecall
1:  j       1b

.align 4
_trap:
    # No mid-stream trap is intended; the only ecall is the terminator
    # (mcause==11, ECALL-from-M). Write gp to tohost for the HTIF exit.
    la      t0, tohost
    sw      gp, 0(t0)
1:  j       1b

.section .data
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
