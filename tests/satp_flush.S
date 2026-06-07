    # satp-write TLB-flush test.
    #
    # Two address spaces map the same VA 0x12345000 to different physical pages:
    #   satp_a (root PPN 0x10000): VA -> PA 0x10002000 (holds 0xAAAAAAAA)
    #   satp_b (root PPN 0x10004): VA -> PA 0x10003000 (holds 0xBBBBBBBB)
    #
    # Access the VA under satp_a (fills the DTLB), then switch to satp_b WITHOUT
    # an SFENCE.VMA. If a satp write flushes the TLB, the second read sees the
    # new mapping (0xBBBBBBBB); a stale DTLB would still return 0xAAAAAAAA.
    #
    # a0=10 pass; a0=11 fail, a1 = 71 first read wrong / 72 stale after switch.
    .section .text
    .globl main
main:
    la      t0, trap_fail
    csrw    mtvec, t0

    # ---- space A tables (root@0x10000000, l0@0x10001000) ----
    li      t0, 0x10000000
    li      t1, 0x04000401          # root_a[72] -> l0_a (PPN 0x10001), V
    sw      t1, (72 * 4)(t0)
    li      t0, 0x10001000 + (0x345 * 4)
    li      t1, 0x040008CF          # l0_a[0x345] -> PA 0x10002, V R W X A D
    sw      t1, 0(t0)

    # ---- space B tables (root@0x10004000, l0@0x10005000) ----
    li      t0, 0x10004000
    li      t1, 0x04001401          # root_b[72] -> l0_b (PPN 0x10005), V
    sw      t1, (72 * 4)(t0)
    li      t0, 0x10005000 + (0x345 * 4)
    li      t1, 0x04000CCF          # l0_b[0x345] -> PA 0x10003, V R W X A D
    sw      t1, 0(t0)

    # ---- data pages ----
    li      t0, 0x10002000
    li      t1, 0xAAAAAAAA
    sw      t1, 0(t0)
    li      t0, 0x10003000
    li      t1, 0xBBBBBBBB
    sw      t1, 0(t0)

    # ---- enable space A + MPRV(S) ----
    li      t0, 0x80010000          # satp_a (PPN 0x10000)
    csrw    satp, t0
    sfence.vma
    li      t0, (3 << 11)
    csrc    mstatus, t0
    li      t0, (1 << 11)
    csrs    mstatus, t0             # MPP = S
    li      t0, (1 << 17)
    csrs    mstatus, t0             # MPRV

    li      t4, 0x12345000
    lw      s0, 0(t4)               # fills DTLB for the VA under space A
    li      s1, 0xAAAAAAAA
    li      a1, 71
    bne     s0, s1, fail

    # ---- switch to space B WITHOUT sfence; relies on satp-write auto-flush ----
    li      t0, 0x80010004          # satp_b (PPN 0x10004)
    csrw    satp, t0

    lw      s0, 0(t4)               # must see the new mapping (0xBBBBBBBB)
    li      s1, 0xBBBBBBBB
    li      a1, 72
    bne     s0, s1, fail

    li      t0, (1 << 17)
    csrc    mstatus, t0             # MPRV off
    li      a0, 10
    ecall
fail:
    li      t0, (1 << 17)
    csrc    mstatus, t0
    li      a0, 11
    ecall

    .align 2
trap_fail:
    li      a0, 11
    li      a1, 99
    ecall
