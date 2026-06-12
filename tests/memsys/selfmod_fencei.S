    # selfmod_fencei (I-side): cache a code line, overwrite one instruction in
    # it, fence.i, and re-execute. Without a correct L1I invalidate the second
    # call would hit the stale cached instruction.
    #
    #   1. call target  -> a0 = 1   (caches target's line with the old encoding)
    #   2. store "li a0,42" (0x02a00513) over target's first instruction
    #   3. fence.i       (D is uncached in C1, so the store is already in memory;
    #                     fence.i must flash-invalidate the L1I)
    #   4. call target  -> a0 = 42  (refetch sees the modified instruction)
    #
    # a0=10 pass, a0=11 fail.
    .option arch, +zicsr, +zifencei
    .section .text
    .globl main
main:
    la      t0, target
    jalr    ra, t0, 0           # first call: returns a0 = 1, caches the line
    li      t3, 1
    bne     a0, t3, fail        # sanity: old code path

    li      t1, 0x02a00513      # encoding of: li a0, 42 (addi a0,x0,42)
    sw      t1, 0(t0)           # overwrite target's first instruction
    fence.i                     # invalidate L1I so the refetch sees the new word

    jalr    ra, t0, 0           # second call: must return a0 = 42
    li      t2, 42
    bne     a0, t2, fail

    li      a0, 10
    ecall
fail:
    li      a0, 11
    ecall

    .balign 4
target:
    li      a0, 1               # 0x00100513, overwritten to li a0,42 at runtime
    ret
9:  j       9b
