    # R5: Sv39 translation test. Builds a 3-level page table at runtime in
    # M-mode (paging off), then forces data accesses through S-mode paging via
    # MPRV and checks: a 4 KiB-page translated load, a 1 GiB superpage, a 2 MiB
    # superpage, a permission page fault, and a non-canonical-VA page fault.
    #
    # Layout (8-byte PTEs):
    #   root @ 0x10000000 (PPN 0x10000)
    #   l1   @ 0x10001000 (PPN 0x10001)
    #   l0   @ 0x10002000 (PPN 0x10002)
    #   data @ 0x10003000 (PPN 0x10003), sentinel at +0
    #
    # Mappings:
    #   root[0] = 1 GiB identity leaf (PPN 0, VRWXAD)      VA 0x10003008 -> PA
    #   root[1] -> l1; l1[1] -> l0; l1[2] = 2 MiB leaf PPN 0x10400 (VRWXAD)
    #   l0[1]   = 4 KiB leaf PPN 0x10003 (VRWXAD)          VA 0x40201000
    #   l0[2]   = 4 KiB leaf PPN 0x10003, X-only (V|X|A|D) -> load page fault
    #
    # a0=10 pass; a0=11 fail (a1 = check); a0=12 unexpected M trap.
    .section .text
    .globl main
main:
    la      t0, trap_handler
    csrw    mtvec, t0

    # --- Build the tables (M-mode, paging off) ---
    li      s0, 0x10000000

    # root[0]: 1 GiB identity leaf (PPN=0 | V R W X A D = 0xCF)
    li      t1, 0xCF
    sd      t1, 0(s0)

    # root[1] -> l1 (PPN 0x10001 | V)
    li      t1, (0x10001 << 10) | 0x1
    sd      t1, 8(s0)

    # l1[1] -> l0 (PPN 0x10002 | V)
    li      t0, 0x10001000
    li      t1, (0x10002 << 10) | 0x1
    sd      t1, 8(t0)

    # l1[2]: 2 MiB leaf PPN 0x10400 (V R W X A D)
    li      t1, (0x10400 << 10) | 0xCF
    sd      t1, 16(t0)

    # l0[1]: 4 KiB leaf PPN 0x10003 (V R W X A D)
    li      t0, 0x10002000
    li      t1, (0x10003 << 10) | 0xCF
    sd      t1, 8(t0)

    # l0[2]: 4 KiB X-only leaf PPN 0x10003 (V X A D = 0xC9): loads fault
    li      t1, (0x10003 << 10) | 0xC9
    sd      t1, 16(t0)

    # sentinels
    li      t0, 0x10003000
    li      t1, 0x1122334455667788
    sd      t1, 0(t0)                # 4 KiB-page sentinel
    li      t1, 0xa5a55a5acafef00d
    sd      t1, 8(t0)                # 1 GiB identity sentinel (VA==PA)
    li      t0, 0x10400000
    li      t1, 0x2222222233333333
    sd      t1, 16(t0)               # 2 MiB superpage sentinel

    # --- Enable Sv39 ---
    li      t0, (8 << 60) | 0x10000  # MODE=Sv39, root PPN 0x10000
    csrw    satp, t0
    sfence.vma

    # --- Data accesses as S-mode+paging via MPRV (MPP=S) ---
    li      t3, (3 << 11)
    csrc    mstatus, t3
    li      t2, (1 << 11)            # MPP = S
    csrs    mstatus, t2
    li      t1, (1 << 17)            # MPRV
    csrs    mstatus, t1

    # 1: 4 KiB page: VA 0x40201000 -> PA 0x10003000
    li      t4, 0x40201000
    ld      s2, 0(t4)
    li      s3, 0x1122334455667788
    li      a1, 1
    bne     s2, s3, fail

    # 2: 1 GiB identity superpage: VA 0x10003008 -> PA 0x10003008
    li      t4, 0x10003008
    ld      s2, 0(t4)
    li      s3, 0xa5a55a5acafef00d
    li      a1, 2
    bne     s2, s3, fail

    # 3: 2 MiB superpage: VA 0x40400010 -> PA 0x10400010
    li      t4, 0x40400010
    ld      s2, 0(t4)
    li      s3, 0x2222222233333333
    li      a1, 3
    bne     s2, s3, fail

    # 4: translated store + readback through the 4 KiB page
    li      t4, 0x40201000
    li      s3, 0xfeedfacefeedface
    sd      s3, 16(t4)
    ld      s2, 16(t4)
    li      a1, 4
    bne     s2, s3, fail

    # 5: load from the X-only page: load page fault (cause 13)
    li      s4, 0                    # trap flag
    li      t4, 0x40202000           # l0[2] mapping
    ld      s2, 0(t4)                # must fault; handler skips it
    li      a1, 5
    li      t2, 13                   # EXC_LOAD_PAGE_FAULT
    bne     s4, t2, fail

    # 6: non-canonical VA: bit 63 set -> load page fault without walking
    li      s4, 0
    li      t4, 0x8000000040201000
    ld      s2, 0(t4)                # must fault
    li      a1, 6
    li      t2, 13
    bne     s4, t2, fail

    # clear MPRV; done
    li      t1, (1 << 17)
    csrc    mstatus, t1
    li      a0, 10
    ecall
fail:
    li      t1, (1 << 17)
    csrc    mstatus, t1
    li      a0, 11
    ecall
9:  j       9b

    # M-mode trap handler. A load page fault (cause 13) while testing is
    # expected: record the cause in s4, skip the faulting load, and return.
    # Anything else halts with a0=12.
    .align 2
trap_handler:
    csrr    t5, mcause
    li      t6, 13
    bne     t5, t6, unexpected
    mv      s4, t5
    csrr    t5, mepc
    addi    t5, t5, 4
    csrw    mepc, t5
    mret
unexpected:
    li      t1, (1 << 17)            # clear MPRV so the halt path is sane
    csrc    mstatus, t1
    li      a0, 12
    ecall
9:  j       9b
