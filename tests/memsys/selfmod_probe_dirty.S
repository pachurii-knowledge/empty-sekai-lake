    # selfmod_probe_dirty (directed C4a probe-hit-dirty + D/D self-consistency):
    # make a line dirty-resident in the L1D via stores, cross-check that a data
    # load sees the dirty bytes (plain L1D hit), then cold-fetch the line so the
    # C4a probe hits a DIRTY line and writes it back before the I-refill.
    #
    #   1. store "li a0,42" ; "ret" into ptgt   (write-allocate -> dirty in L1D)
    #   2. load  ptgt[0]                         (must read 0x02a00513 from L1D)
    #   3. NO fence.i
    #   4. jalr  ptgt                            (cold I-fetch -> probe dirty hit)
    #
    # a0=10 pass, a0=11 fail.
    .option arch, +zicsr, +zifencei
    .section .text
    .globl main
main:
    la      t0, ptgt
    li      t1, 0x02a00513      # li a0, 42
    sw      t1, 0(t0)
    li      t2, 0x00008067      # ret
    sw      t2, 4(t0)

    lw      t4, 0(t0)           # data load must see the dirty store (L1D hit)
    bne     t4, t1, fail

    jalr    ra, t0, 0           # cold I-fetch: C4a probe finds the line dirty
    li      t3, 42
    bne     a0, t3, fail

    li      a0, 10
    ecall
fail:
    li      a0, 11
    ecall

    .balign 4096
ptgt:
    .space  64
