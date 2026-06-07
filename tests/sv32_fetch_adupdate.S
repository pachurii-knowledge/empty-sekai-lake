    # Sv32 instruction fetch that requires a hardware A-bit update (Svadu).
    #
    # Map a 4 MiB superpage (level-1 leaf) covering this program's .text, with the
    # X bit set but A=0, and ADUE=1. MRET into S-mode at a virtual alias of
    # svc_code: the fetch walk must hardware-set A and execute the code (no
    # fault). This exercises the fetch-side A/D-update path.
    #
    #   svc_code physical page rounds to a 4MB base PB; superpage PPN = PB>>12.
    #   Map VA 0x40000000-region (VPN1=0x100) -> PB, so VA = 0x40000000 + (svc
    #   offset within its 4MB region) aliases svc_code.
    #
    # Halts a0=10 (translated fetch ran), 12 unexpected M-trap, 13 S-trap.
    .section .text
    .globl main
main:
    la      t0, mtrap_fail
    csrw    mtvec, t0
    la      t0, strap_fail
    csrw    stvec, t0

    la      a1, svc_code
    srli    a2, a1, 22              # a2 = 4MB region index (superpage PPN>>10)
    slli    a2, a2, 10              # a2 = superpage PPN (low 10 bits zero)
    slli    a3, a1, 10              # isolate low 22 bits (offset within 4MB)
    srli    a3, a3, 10              # a3 = offset within the 4MB region

    # root[0x100] = superpage leaf: (PPN<<10) | V R X, A=0 D=0, U=0.
    slli    t1, a2, 10
    ori     t1, t1, 0x0B            # V(1) R(2) X(8)  -- no A(0x40)/D(0x80)
    li      t0, 0x10000000
    sw      t1, (0x100 * 4)(t0)     # root_table[0x100]

    li      t0, 0x80010000          # satp MODE=Sv32, root PPN 0x10000
    csrw    satp, t0
    li      t0, (1 << 29)
    csrs    menvcfgh, t0            # ADUE = 1
    sfence.vma

    # Enter S-mode at the virtual alias of svc_code.
    li      t0, 0x40000000
    add     t0, t0, a3
    csrw    mepc, t0
    li      t3, 0x1800
    csrc    mstatus, t3
    li      t4, 0x800               # MPP = S
    csrs    mstatus, t4
    mret

    .align 2
svc_code:
    li      a0, 10                  # reached via A-updating translated fetch
    ecall

    .align 2
mtrap_fail:
    li      a0, 12
    ecall
    .align 2
strap_fail:
    li      a0, 13
    ecall
