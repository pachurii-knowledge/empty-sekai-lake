    # dcache_evict: a 20 KiB write/readback footprint exceeds the 16 KiB L1D, so
    # the write pass dirties and evicts lines (write-back), and the read pass
    # refills them -- a readback mismatch means a writeback dropped/corrupted a
    # cache line. Data lives at an absolute RAM address (la-to-.data is broken
    # name-remapping loader).
    .option arch, +zicsr
    .equ BASE, 0x10000000
    .equ NW,   5120              # 20 KiB / 4
    .section .text
    .globl main
main:
    li      s0, BASE
    li      s2, NW
    # write pass: mem[base + 4i] = i
    li      s1, 0
wloop:
    slli    t0, s1, 2
    add     t1, s0, t0
    sw      s1, 0(t1)
    addi    s1, s1, 1
    blt     s1, s2, wloop
    # read pass: verify + checksum
    li      s1, 0
    li      s3, 0
rloop:
    slli    t0, s1, 2
    add     t1, s0, t0
    lw      t2, 0(t1)
    bne     t2, s1, fail
    add     s3, s3, t2
    addi    s1, s1, 1
    blt     s1, s2, rloop
    li      t0, 13104640         # sum(0..5119)
    bne     s3, t0, fail
    li      a0, 10
    ecall
fail:
    li      a0, 11
    ecall
9:  j       9b
