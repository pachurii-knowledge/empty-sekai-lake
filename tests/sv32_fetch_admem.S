    # Does an instruction-fetch A-bit update actually land in the PTE in MEMORY?
    #
    # Map a 4 MiB superpage over .text with X set but A=0 (ADUE=1), MRET into
    # S-mode at a virtual alias so the fetch walk hardware-sets A, then trap back
    # to M-mode and read the leaf PTE physically to confirm A is set in memory.
    #
    # Leaf PTE lives at root[0x100] = 0x10000000 + 0x100*4 = 0x10000400.
    #
    # a0 = 10 pass (A set in memory) / 11 fail with a1 = 71 A not set /
    #      12 unexpected M trap before the check / 13 unexpected S trap.
    .section .text
    .globl main
main:
    la      t0, mtrap
    csrw    mtvec, t0
    la      t0, strap_fail
    csrw    stvec, t0

    la      a1, svc_code
    srli    a2, a1, 22
    slli    a2, a2, 10              # a2 = superpage PPN (low 10 bits 0)
    slli    a3, a1, 10
    srli    a3, a3, 10              # a3 = offset within the 4MB region

    # root[0x100] = superpage leaf: (PPN<<10) | V R X, A=0 D=0.
    slli    t1, a2, 10
    ori     t1, t1, 0x0B            # V R X (no A/D)
    li      s2, 0x10000400          # &root[0x100]
    sw      t1, 0(s2)

    li      t0, 0x80010000          # satp Sv32, root PPN 0x10000
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
    li      s3, 0                   # 0 = not yet returned from svc_code
    mret

    .align 2
svc_code:
    li      a0, 0                   # non-halt value: ecall traps to M (mcause 9)
    li      s3, 1
    ecall

    .align 2
mtrap:
    bnez    s3, check_pte           # came back from the translated fetch
    li      a0, 12                  # trapped before reaching svc_code
    ecall
check_pte:
    lw      t2, 0(s2)               # physical read of the leaf PTE
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
