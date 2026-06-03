    # Sv32 data-translation test.
    #
    # The test infrastructure links test code PC-relative and loads it at the
    # user-text window (0x00400000), while absolute static .data does not land in
    # a simulator-backed window. So instead of static page tables we BUILD them at
    # runtime with stores into the user-data window (0x10000000) while still in
    # M-mode with paging disabled (identity-mapped). We then enable Sv32 and
    # perform a translated load.
    #
    # Layout (all inside the 512 KB data window 0x10000000..0x1007ffff):
    #   root_table @ 0x10000000  (PPN 0x10000)
    #   l0_table   @ 0x10001000  (PPN 0x10001)
    #   data_page  @ 0x10002000  (PPN 0x10002), holds sentinel 0xCAFEBABE
    #
    # Mapping VA 0x12345000 -> PA 0x10002000:
    #   VPN[1] = 0x12345000[31:22] = 72 (0x48)
    #   VPN[0] = 0x12345000[21:12] = 0x345
    #   offset = 0
    #   root[72]  = 0x04000401  (-> l0 table PPN 0x10001, V)
    #   l0[0x345] = 0x040008CF  (-> leaf PPN 0x10002, V R W X A D; U=0 supervisor)
    #   satp      = 0x80010000  (MODE=Sv32, PPN = 0x10000)
    #
    # The leaf is a supervisor page (U=0) so an S-mode access needs no SUM.
    # Halts with a0 = 10 (pass) / 11 (fail) / 12 (unexpected trap).
    .section .text
    .globl main
main:
    # Catch any unexpected trap so a fault halts cleanly instead of running away.
    la      t0, trap_fail
    csrw    mtvec, t0

    # --- Build the page tables in physical memory (M-mode, paging off) ---
    li      t0, 0x10000000          # root_table base
    li      t1, 0x04000401          # root[72] -> l0 table, V
    sw      t1, (72 * 4)(t0)        # root_table[72]

    li      t0, 0x10001000 + (0x345 * 4)   # &l0_table[0x345] (offset 0xD14 > imm12)
    li      t1, 0x040008CF          # l0[0x345] -> leaf, V R W X A D (supervisor)
    sw      t1, 0(t0)               # l0_table[0x345]

    li      t0, 0x10002000          # data_page base
    li      t1, 0xCAFEBABE          # sentinel value
    sw      t1, 0(t0)               # data_page[0]

    # --- Enable Sv32 translation ---
    li      t0, 0x80010000          # MODE=Sv32, root PPN = 0x10000
    csrw    satp, t0
    sfence.vma

    # --- Force data accesses through S-mode + paging via MPRV ---
    li      t3, (3 << 11)           # clear MPP field
    csrc    mstatus, t3
    li      t2, (1 << 11)           # MPP = 01 (Supervisor)
    csrs    mstatus, t2
    li      t1, (1 << 17)           # MPRV: data accesses use MPP's privilege+paging
    csrs    mstatus, t1

    li      t4, 0x12345000
    lw      s0, 0(t4)               # translated load -> PA 0x10002000

    csrc    mstatus, t1             # clear MPRV before returning to normal fetch

    li      s1, 0xCAFEBABE
    bne     s0, s1, fail
    li      a0, 10
    ecall
fail:
    li      a0, 11
    ecall

    .align 2
trap_fail:
    li      a0, 12                  # unexpected trap (e.g. page/access fault)
    ecall
