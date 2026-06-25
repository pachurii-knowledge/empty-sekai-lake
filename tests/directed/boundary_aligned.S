# Directed Sv39 lockstep test: boundary_aligned  (golden: Spike, all ALIGNED)
#
# Isolates a suspected niigo corner found via crosspage_split_ldsd: does an
# 8-byte ALIGNED access at 0x...FF8 (the LAST doubleword of a page, bytes
# 0xFF8..0xFFF -- entirely within the page, NOT crossing) get mis-detected as a
# cross-page access and wrongly fault when the NEXT page is unmapped?
#
# All accesses are naturally aligned, so Spike needs no --misaligned and is a
# clean oracle. VA 0x40201000 -> page A (RWXAD); VA 0x40202000 -> page B (RWXAD).
.section .text
.globl _start
.option norvc
_start:
    la      t0, _mtrap
    csrw    mtvec, t0
    la      sp, _stack_top
    la      t0, root_tbl
    la      t1, l1_tbl
    srli    t1, t1, 12
    slli    t1, t1, 10
    ori     t1, t1, 0x01
    sd      t1, 8(t0)
    la      t0, l1_tbl
    la      t1, l0_tbl
    srli    t1, t1, 12
    slli    t1, t1, 10
    ori     t1, t1, 0x01
    sd      t1, 8(t0)
    la      s0, l0_tbl
    la      t1, page_a
    srli    t1, t1, 12
    slli    t1, t1, 10
    ori     t1, t1, 0xCF
    sd      t1, 8(s0)                  # l0[1] = page A
    la      t1, page_b
    srli    t1, t1, 12
    slli    t1, t1, 10
    ori     t1, t1, 0xCF
    sd      t1, 16(s0)                 # l0[2] = page B
    la      t0, root_tbl
    srli    t0, t0, 12
    li      t1, (8 << 60)
    or      t0, t0, t1
    csrw    satp, t0
    sfence.vma
    li      t3, (3 << 11)
    csrc    mstatus, t3
    li      t2, (1 << 11)
    csrs    mstatus, t2
    li      t1, (1 << 17)
    csrs    mstatus, t1                # MPRV on (S-mode data translation)

    # both pages valid: aligned page-end + next-page-start accesses
    li      t4, 0x40201FF8             # last dword of page A (aligned)
    li      t5, 0x1111222233334444
    sd      t5, 0(t4)
    ld      a0, 0(t4)                  # within page A -> expect t5
    li      t4, 0x40202000            # first dword of page B (aligned)
    li      t5, 0x5555666677778888
    sd      t5, 0(t4)
    ld      a1, 0(t4)                  # within page B -> expect t5

    # invalidate page B, leave page A valid
    csrc    mstatus, t1                # MPRV off to touch the PTE
    sd      x0, 16(s0)                 # l0[2] = invalid
    sfence.vma
    csrs    mstatus, t1                # MPRV on

    # page-A last dword (0x...FF8): STILL valid -> must NOT fault
    li      s4, 0
    li      t4, 0x40201FF8
    ld      a2, 0(t4)                  # expect t5(0x1111...) -- NO fault
    mv      a3, s4                     # expect 0 (no fault); 13 if buggy fault

    # page-B first dword (0x40202000): now invalid -> MUST fault (13)
    li      s4, 0
    li      t4, 0x40202000
    ld      s2, 0(t4)
    mv      a4, s4                     # expect 13 (load page fault)

    csrc    mstatus, t1
    li      gp, 1
    ecall
1:  j       1b

.align 2
_mtrap:
    csrr    t6, mcause
    li      t5, 11
    beq     t6, t5, _term
    li      t5, 13
    beq     t6, t5, _rec
    li      t5, 15
    beq     t6, t5, _rec
    j       _term
_rec:
    mv      s4, t6
    csrr    t5, mepc
    addi    t5, t5, 4
    csrw    mepc, t5
    mret
_term:
    la      t1, tohost
    li      t2, 1
    sw      t2, 0(t1)
1:  j       1b

.section .data
.align 12
_stack:   .zero 4096
_stack_top:

.section .page_table,"aw",@progbits
.align 12
root_tbl:  .zero 4096
.align 12
l1_tbl:    .zero 4096
.align 12
l0_tbl:    .zero 4096
.align 12
page_a:    .zero 4096
.align 12
page_b:    .zero 4096

.pushsection .tohost,"aw",@progbits
.align 6
.globl tohost
tohost:   .dword 0
.globl fromhost
fromhost: .dword 0
.popsection
