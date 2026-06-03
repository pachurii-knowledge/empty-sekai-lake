    # Sv32 instruction-fetch translation test.
    #
    # In M-mode (paging off) we build a 2-level page table that maps the virtual
    # code page 0x40000000 to the physical page that actually holds this program's
    # .text (loaded at the user-text window). We then MRET into S-mode with the PC
    # set to a virtual address inside that page, so the fetch is translated by the
    # ITLB + hardware page-table walker. The translated code halts with a0 = 10.
    #
    # Tables live in the user-data window (0x10000000):
    #   root_table @ 0x10000000 (PPN 0x10000), l0_table @ 0x10001000 (PPN 0x10001)
    #   satp = 0x80010000 (MODE=Sv32, root PPN 0x10000)
    #
    # VA 0x40000000 -> VPN[1]=0x100, VPN[0]=0, offset=0
    #   root[0x100] = 0x04000401            (-> l0 table PPN 0x10001, V)
    #   l0[0]       = (code_ppn<<10)|0xCB   (-> leaf, V R X A D; U=0 supervisor)
    #
    # Halts a0 = 10 (pass) / 11 (fail) / 12 (unexpected M trap) / 13 (unexpected S trap).
    .section .text
    .globl main
main:
    la      t0, mtrap_fail
    csrw    mtvec, t0
    la      t0, strap_fail
    csrw    stvec, t0

    # Physical page number / offset of the routine we will run virtually.
    la      a1, svc_code            # PC-relative -> physical address of svc_code
    srli    a2, a1, 12              # a2 = code physical page number
    slli    a3, a1, 20              # isolate low 12 bits (0xFFF too big for andi)
    srli    a3, a3, 20              # a3 = offset within the page

    # root[0x100] -> l0 table.
    li      t0, 0x10000000
    li      t1, 0x04000401
    sw      t1, (0x100 * 4)(t0)     # root_table[0x100], offset 0x400

    # l0[0] -> leaf for the code page: (code_ppn<<10) | V R X A D (U=0).
    slli    t1, a2, 10
    ori     t1, t1, 0xCB            # V(1) R(2) X(8) A(0x40) D(0x80)
    li      t0, 0x10001000
    sw      t1, 0(t0)               # l0_table[0]

    li      t0, 0x80010000          # satp: MODE=Sv32, root PPN 0x10000
    csrw    satp, t0
    sfence.vma

    # Enter S-mode at the virtual alias of svc_code.
    li      t0, 0x40000000
    add     t0, t0, a3              # VA = 0x40000000 + page offset
    csrw    mepc, t0
    li      t3, 0x1800              # clear MPP
    csrc    mstatus, t3
    li      t4, 0x800               # MPP = 01 (Supervisor)
    csrs    mstatus, t4
    mret                            # -> S-mode, fetch translated through Sv32

    .align 2
svc_code:
    li      a0, 10                  # reached via translated fetch -> success
    ecall

    .align 2
mtrap_fail:
    li      a0, 12
    ecall

    .align 2
strap_fail:
    li      a0, 13
    ecall
