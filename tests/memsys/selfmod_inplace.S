    # selfmod_inplace (C4b consumer-staleness + C4a): execute a line so it is
    # resident in the L1I, store-patch the SAME physical line, then re-execute.
    # This needs BOTH coherence pieces:
    #   - C4b: the store must invalidate the L1I's now-stale copy of the line.
    #   - C4a: the refetch then misses and must pull the patched bytes, which
    #          are dirty in the L1D, not yet in memory.
    #
    # NOTE on ordering: C4b's snoop fires when the store *commits*. On an OoO
    # core a re-fetch issued in the same breath as the store can be fetched
    # speculatively BEFORE the store commits (tight in-place SMC without fence.i
    # is architecturally undefined for exactly this reason). C4b's guarantee is
    # for a line re-fetched AFTER the modifying store retires -- the realistic
    # JIT / patch-then-call-later pattern. We make that ordering explicit here by
    # data-dependencing the second call's target register on a load of the
    # patched word (which the LSQ serialises after the store), so the jalr cannot
    # resolve -- and target cannot be fetched -- until after the store committed
    # and snoop-invalidated the L1I line.
    #
    #   1. call target          -> a0 = 1   (caches target's line in the L1I)
    #   2. store "li a0,42" over target[0]   (dirty in L1D; snoops L1I at commit)
    #   3. load target[0] -> data-dep the jalr target on it (orders fetch>store)
    #   4. call target          -> a0 = 42  (L1I miss -> C4a probe -> patched)
    #
    # a0=10 pass, a0=11 fail.
    .option arch, +zicsr, +zifencei
    .section .text
    .globl main
main:
    la      t0, target
    jalr    ra, t0, 0           # first call: a0 = 1, caches the line in L1I
    li      t3, 1
    bne     a0, t3, fail

    li      t1, 0x02a00513      # li a0, 42
    sw      t1, 0(t0)           # patch target[0]; C4b snoop-invalidates at commit
    # deliberately no fence.i

    lw      t5, 0(t0)           # read patched word back (LSQ-serialised after store)
    sub     t5, t5, t1          # t5 = 0, but data-dependent on the load
    add     t0, t0, t5          # t0 = target, now dependent on the load (orders fetch)

    jalr    ra, t0, 0           # second call: resolves only post-store -> a0 = 42
    li      t2, 42
    bne     a0, t2, fail

    li      a0, 10
    ecall
fail:
    li      a0, 11
    ecall

    .balign 4
target:
    li      a0, 1               # overwritten to li a0,42 at runtime
    ret
