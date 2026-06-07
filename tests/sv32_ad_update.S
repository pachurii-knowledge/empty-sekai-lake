    # Sv32 A/D-update mode test (Svade vs Svadu, menvcfg.ADUE).
    #
    # Built on sv32_data.S: page tables in the data window, translated loads
    # performed from M-mode via MPRV (MPP=S). The leaf PTE is mapped with A=0
    # (V R W, no A/D), so the access requires an A-bit update.
    #
    #   Phase 1 (Svade, menvcfg.ADUE=0): the load must take a LOAD PAGE FAULT
    #            (mcause=13), and the PTE in memory must be UNCHANGED (A still 0).
    #   Phase 2 (Svadu, menvcfg.ADUE=1): the same load must SUCCEED, return the
    #            sentinel, and the PTE in memory must have A set by hardware.
    #
    # Exit: a0 = 10 pass; a0 = 11 fail with a1 = diagnostic code:
    #   21 Svade: no fault   22 wrong cause   23 PTE A modified despite fault
    #   24 Svadu: faulted    25 wrong value   26 PTE A not set by hardware
    #
    # Regs: s2=&leaf PTE, s3=trap-seen, s4=trap cause, s5=MPRV mask, s6=MPP=S setup.
    .section .text
    .globl main
main:
    la      t0, mtrap
    csrw    mtvec, t0
    li      s5, (1 << 17)           # MPRV mask
    li      s6, (1 << 11)           # MPP = Supervisor

    # root[72] -> l0 table.
    li      t0, 0x10000000
    li      t1, 0x04000401
    sw      t1, (72 * 4)(t0)

    # l0[0x345] -> leaf PPN 0x10002 | V R W, A=0 D=0, U=0.
    li      s2, 0x10001000 + (0x345 * 4)
    li      t1, 0x04000807          # PPN(0x10002)<<10 | V(1) R(2) W(4)
    sw      t1, 0(s2)

    li      t0, 0x10002000          # data_page
    li      t1, 0xCAFEBABE
    sw      t1, 0(t0)

    li      t0, 0x80010000          # satp MODE=Sv32, root PPN 0x10000
    csrw    satp, t0
    sfence.vma

    #################### Phase 1: Svade (ADUE = 0) ####################
    li      t0, (1 << 29)
    csrc    menvcfgh, t0            # ADUE = 0
    sfence.vma

    jal     mprv_s_on              # MPP=S, MPRV on
    li      s3, 0
    li      t4, 0x12345000
    lw      s0, 0(t4)              # expect LOAD PAGE FAULT (handler skips it)
    jal     mprv_off

    li      a1, 21
    beqz    s3, fail               # no fault -> bug
    li      a1, 22
    li      t1, 13                 # load page fault
    bne     s4, t1, fail
    jal     read_pte_a             # t2 = PTE & A
    li      a1, 23
    bnez    t2, fail               # A must be unchanged

    #################### Phase 2: Svadu (ADUE = 1) ####################
    li      t0, (1 << 29)
    csrs    menvcfgh, t0           # ADUE = 1
    sfence.vma

    jal     mprv_s_on
    li      s3, 0
    li      t4, 0x12345000
    lw      s0, 0(t4)              # expect SUCCESS; hardware sets A
    jal     mprv_off

    li      a1, 24
    bnez    s3, fail               # unexpected fault
    li      a1, 25
    li      s1, 0xCAFEBABE
    bne     s0, s1, fail
    jal     read_pte_a
    li      a1, 26
    beqz    t2, fail               # A must now be set

    li      a0, 10                 # pass
    ecall
fail:
    li      a0, 11
    ecall

    # mprv_s_on: set MPP=S and MPRV on (re-established each time since a prior
    # mret resets MPP to U).
    .align 2
mprv_s_on:
    li      t0, (3 << 11)
    csrc    mstatus, t0            # clear MPP
    csrs    mstatus, s6            # MPP = S
    csrs    mstatus, s5            # MPRV on
    ret

mprv_off:
    csrc    mstatus, s5            # MPRV off
    ret

    # read_pte_a: t2 = (PTE at s2) & A-bit, read untranslated (MPRV off).
    .align 2
read_pte_a:
    csrc    mstatus, s5
    lw      t2, 0(s2)
    andi    t2, t2, 0x40          # A bit
    ret

    # M-mode trap handler (paging off -> always reachable): record cause, set
    # seen-flag, advance past the faulting load, return to M-mode.
    .align 2
mtrap:
    csrr    s4, mcause
    li      s3, 1
    csrr    t6, mepc
    addi    t6, t6, 4
    csrw    mepc, t6
    mret
