    # Sv32 instruction-fetch PAGE-FAULT test (Phase A).
    #
    # Same setup as sv32_fetch.S, but the leaf PTE for the code page is mapped
    # WITHOUT the execute bit (V R A D, no X). MRET-ing into S-mode with the PC
    # in that page must raise an instruction page fault (scause = 12) with stval
    # set to the faulting virtual address, NOT execute the page.
    #
    # medeleg = 0, so the S-mode instruction page fault is taken in M-mode,
    # whose handler runs with fetch translation off (reachable physically).
    # The M handler checks mcause == 12 and mtval == the VA we jumped to, then
    # halts a0 = 10 (pass). Any other outcome halts non-10.
    #   a0 = 10 pass
    #   a0 = 11 fetch actually executed (no fault)   -> bug
    #   a0 = 14 wrong mcause
    #   a0 = 15 wrong mtval
    .section .text
    .globl main
main:
    la      t0, mtrap_handler
    csrw    mtvec, t0

    # Physical page number / offset of the routine we will (try to) run.
    la      a1, svc_code
    srli    a2, a1, 12              # a2 = code physical page number
    slli    a3, a1, 20
    srli    a3, a3, 20              # a3 = offset within the page

    # root[0x100] -> l0 table.
    li      t0, 0x10000000
    li      t1, 0x04000401
    sw      t1, (0x100 * 4)(t0)     # root_table[0x100]

    # l0[0] -> leaf for the code page WITHOUT X: (code_ppn<<10) | V R A D, U=0.
    slli    t1, a2, 10
    ori     t1, t1, 0xC3            # V(1) R(2) A(0x40) D(0x80)  -- no X(8)
    li      t0, 0x10001000
    sw      t1, 0(t0)               # l0_table[0]

    li      t0, 0x80010000          # satp: MODE=Sv32, root PPN 0x10000
    csrw    satp, t0
    sfence.vma

    # Compute and stash the virtual address we will jump to (for stval compare).
    li      t0, 0x40000000
    add     s0, t0, a3              # s0 = expected faulting VA
    csrw    mepc, s0
    li      t3, 0x1800              # clear MPP
    csrc    mstatus, t3
    li      t4, 0x800               # MPP = 01 (Supervisor)
    csrs    mstatus, t4
    mret                            # -> S-mode; the fetch should page-fault

    .align 2
svc_code:
    li      a0, 11                  # must NOT execute: no-execute page
    ecall

    .align 2
mtrap_handler:
    csrr    t0, mcause
    li      t1, 12                  # instruction page fault
    bne     t0, t1, mcause_bad
    csrr    t0, mtval
    bne     t0, s0, mtval_bad
    li      a0, 10                  # pass
    ecall

mcause_bad:
    li      a0, 14
    ecall
mtval_bad:
    li      a0, 15
    ecall
