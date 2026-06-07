    # Sv32 A/D update for a load issued by translated (S-mode) code, so the code
    # fetch and the data load both drive the shared PTW. Mirrors how the Svadu
    # arch test runs (its code executes at a VA alias). The data leaf starts
    # A=0,D=1; after the S-mode load it must have A set in memory.
    #
    #   code: 4MB superpage root[0x100] over .text (A=1, no code A-update needed)
    #   data: VA 0x12345000 -> PA 0x10002000, l0[0x345] @ 0x10001D14, A=0 D=1
    #
    # a0=10 pass / 11 fail a1=71 load PTE A unset / 12 unexpected non-ecall trap.
    .section .text
    .globl main
main:
    la      t0, mtrap
    csrw    mtvec, t0
    la      t0, strap_fail
    csrw    stvec, t0

    la      a1, svc_code
    srli    a2, a1, 22
    slli    a2, a2, 10              # superpage PPN of .text
    slli    a3, a1, 10
    srli    a3, a3, 10              # offset within the 4MB region

    # root[0x100] = code superpage leaf, V R X A (no update needed on fetch).
    slli    t1, a2, 10
    ori     t1, t1, 0x4B            # V R X A
    li      t0, 0x10000000
    sw      t1, (0x100 * 4)(t0)

    # root[0x48] -> l0 table (PPN 0x10001).
    li      t1, 0x04000401
    sw      t1, (0x48 * 4)(t0)
    # l0[0x345] = data leaf PA 0x10002 | V R W | D, A=0.
    li      s6, 0x10001D14
    li      t1, 0x04000887
    sw      t1, 0(s6)
    li      t0, 0x10002000
    li      t1, 0xCAFEBABE
    sw      t1, 0(t0)

    li      t0, 0x80010000
    csrw    satp, t0
    li      t0, (1 << 29)
    csrs    menvcfgh, t0            # ADUE = 1
    sfence.vma

    li      t0, 0x40000000
    add     t0, t0, a3
    csrw    mepc, t0
    li      t3, 0x1800
    csrc    mstatus, t3
    li      t4, 0x800               # MPP = S
    csrs    mstatus, t4
    li      s3, 0
    mret

    .align 2
svc_code:
    li      t4, 0x12345000
    lw      s0, 0(t4)               # translated load by translated code
    li      a0, 0
    li      s3, 1
    ecall                           # -> M (mcause 9), not a halt

    .align 2
mtrap:
    bnez    s3, check
    li      a0, 12
    ecall
check:
    lw      t2, 0(s6)               # physical read of the data leaf PTE
    andi    t2, t2, 0x40            # A bit
    bnez    t2, pass
    li      a0, 11
    li      a1, 71
    ecall
pass:
    li      a0, 10
    ecall

    .align 2
strap_fail:
    li      a0, 13
    ecall
