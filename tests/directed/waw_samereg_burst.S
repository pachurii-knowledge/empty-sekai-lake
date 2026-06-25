# Directed lockstep torture test (rv64): waw_samereg_burst
#
# Corner: intragroup WAW (write-after-write) to the SAME arch reg (x5) packed
# into a single 16-byte fetch group, so all four writers rename in ONE cycle.
# Lane N's old_prd must be lane N-1's freshly allocated physreg (the in-group
# WAW bypass in src/rename_map_table.sv:56-67), and at commit each intermediate
# physreg must be freed exactly once. A stale/leaked/double-freed physreg shows
# up as a wrong committed x5 downstream.
#
# To expose any leak, we run the burst ~200x in a loop, with a data-dependent
# bne x5,x6 that mispredicts ~50% (x6 toggles from memory), and a JALR rd=x5
# right after a burst (branch-own-rd snapshot patch). We FOLD the committed x5
# of every iteration into a running accumulator (a4) so a single wrong x5 in any
# iteration perturbs the final integer GPR trace.
#
# Lockstep-compared against Spike AND Sail (every integer GPR write diffed).
.section .text
.globl _start
.option norvc
_start:
    la      t0, _trap
    csrw    mtvec, t0
    la      sp, _stack_top

    la      t1, _toggle            # x6 source: a byte that flips 0/1 in memory
    li      a4, 0                  # running accumulator of committed x5
    li      a5, 0                  # secondary accumulator (post-jalr x5)
    li      s0, 200                # iteration count
    li      s1, 0                  # loop counter

loop_top:
    # load x6 from memory (data-dependent branch target, toggles each iter)
    lbu     a6, 0(t1)              # a6 = current toggle (0 or 1)

    # --------- the WAW burst: four writers to x5 in one fetch group ----------
    # p2align 4 forces these four 32-bit instrs to start at a 16-byte boundary
    # so they land in a single fetch group (4 * 4 bytes = 16 bytes).
    .p2align 4
    addi    x5, x0, 1              # x5 = 1            (lane 0)
    addi    x5, x0, 2              # x5 = 2            (lane 1, WAW)
    addi    x5, x5, 3              # x5 = 2 + 3 = 5    (lane 2, WAW + RAW on x5)
    addi    x5, x5, 4              # x5 = 5 + 4 = 9    (lane 3, WAW + RAW on x5)
    # committed x5 MUST be 9 every iteration.

    # data-dependent branch: mispredicts ~50% as a6 toggles. x6 := a6.
    add     x6, a6, x0
    bne     x5, x6, taken          # 9 != (0 or 1) always taken; tests resolve
    # (fallthrough never reached; keeps the branch live + data-dependent)
    addi    a4, a4, 1              # poison path (should never execute)
taken:
    # fold committed x5 into the accumulator
    add     a4, a4, x5             # a4 += 9 each iteration

    # --------- second WAW burst followed immediately by JALR rd=x5 ----------
    # branch-own-rd snapshot patch: the JALR writes x5 right after a burst.
    .p2align 4
    addi    x5, x0, 10             # x5 = 10           (lane 0)
    addi    x5, x5, 20             # x5 = 30           (lane 1, WAW + RAW)
    addi    x5, x5, 30             # x5 = 60           (lane 2, WAW + RAW)
    addi    x5, x5, 40             # x5 = 100          (lane 3, WAW + RAW)
    # x5 == 100 here; now a JALR with rd=x5 overwrites it with return addr.
    la      t2, jalr_ret
    jalr    x5, 0(t2)              # x5 := PC+4 of this jalr; jump to jalr_ret
jalr_ret:
    # x5 now holds the return address (jalr_ret-ish PC). Mask to low bits and
    # fold a STABLE function of it so the value is identical on all sims:
    # x5 == &jalr_ret (a link-fixed absolute address), deterministic.
    andi    t3, x5, 0xff           # low byte of the link address (deterministic)
    add     a5, a5, t3             # fold into secondary accumulator

    # flip the toggle byte in memory for the next iteration
    xori    a6, a6, 1
    sb      a6, 0(t1)

    addi    s1, s1, 1
    bne     s1, s0, loop_top

    # --------- terminate: make all accumulators visible, then ecall ---------
    # a4 = 9 * 200 = 1800 ; a5 = (low byte of &jalr_ret) * 200
    add     a0, a4, a5             # final folded result lands in a0 (traced)
    li      gp, 1
    ecall
1:  j       1b

.align 4
_trap:
    # No mid-stream traps are intended; mcause should be 11 (ECALL-from-M).
    la      t1, tohost
    sw      gp, 0(t1)              # HTIF exit, value 1
1:  j       1b

.section .data
.align 8
_toggle:  .byte 0
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
