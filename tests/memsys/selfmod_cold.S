    # selfmod_cold (C4a producer-visibility): write code into a COLD line (one
    # never fetched into the L1I) and execute it WITHOUT fence.i. The stores
    # land dirty in the write-back L1D; the demand fetch misses the L1I and must
    # see the new bytes. Without C4a (clean-before-refill) the I-refill reads
    # stale memory (the dirty bytes are still in the L1D) and executes garbage.
    #
    #   1. store "li a0,42" ; "ret" into coldtgt   (dirty in L1D, NOT in memory)
    #   2. NO fence.i
    #   3. jalr coldtgt -> a0 must be 42
    #
    # coldtgt sits a page away so wrong-path sequential prefetch can never
    # pre-cache it: it is genuinely cold at the demand fetch (mirrors xv6 exec
    # writing fresh code pages). a0=10 pass, a0=11 fail.
    .option arch, +zicsr, +zifencei
    .section .text
    .globl main
main:
    la      t0, coldtgt
    li      t1, 0x02a00513      # li a0, 42  (addi a0,x0,42)
    sw      t1, 0(t0)
    li      t2, 0x00008067      # ret        (jalr x0,ra,0)
    sw      t2, 4(t0)
    # deliberately no fence.i
    jalr    ra, t0, 0           # cold fetch of the just-written line
    li      t3, 42
    bne     a0, t3, fail

    li      a0, 10
    ecall
fail:
    li      a0, 11
    ecall

    # A full page away: out of reach of any wrong-path prefetch after the jalr.
    .balign 4096
coldtgt:
    .space  64                  # fresh 64 B line, zero-filled, never fetched
