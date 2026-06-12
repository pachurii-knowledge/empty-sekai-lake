    # icache_thrash: straight-line code footprint larger than the 16 KiB L1I,
    # executed twice. The second pass re-misses lines the first pass evicted, so
    # this exercises L1I refill + PLRU eviction correctness end to end.
    #
    # 5000 `addi s0,s0,1` == 20 KiB of code > 16 KiB cache. Two passes accumulate
    # 10000 into s0. Mismatch == a refill or eviction dropped/duplicated an
    # instruction. a0=10 pass, a0=11 fail.
    .option arch, +zicsr
    .section .text
    .globl main
main:
    li      s0, 0           # accumulator
    li      s1, 2           # pass count
loop:
    .rept 5000
    addi    s0, s0, 1
    .endr
    addi    s1, s1, -1
    beqz    s1, done
    j       loop            # j reaches the ~20 KiB-back target (bnez cannot)
done:
    li      t0, 10000
    bne     s0, t0, fail
    li      a0, 10
    ecall
fail:
    li      a0, 11
    ecall
9:  j       9b
