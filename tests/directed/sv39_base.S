# Sv39 lockstep TEMPLATE (rv64). Validates a portable, golden-comparable Sv39
# page-table harness for the directed lockstep flow. Unlike tests/rv64_sv39.S
# (tables at PA 0x1000_0000, which Spike/Sail do not back as RAM), this places
# the 3-level tables + test pages in the .page_table section (link.ld puts it in
# 0x8000_xxxx RAM that ALL THREE sims load), so PTEs reference PPNs the golden
# models can walk. Effects are landed in integer GPRs (a0..a3) so the lockstep
# oracle (Spike AND Sail) checks them; ecall terminates, _mtrap writes tohost.
#
# PTE flags: V=0x1 R=0x2 W=0x4 X=0x8 U=0x10 G=0x20 A=0x40 D=0x80
#   leaf RWXAD = 0xCF ; pointer (V only) = 0x01 ; X-only+AD leaf = 0xC9
# VA 0x40201000 -> VPN2=1,VPN1=1,VPN0=1 : root[1]->l1, l1[1]->l0, l0[1]=leaf
# VA 0x40202000 -> VPN0=2 : l0[2] = X-only leaf (loads page-fault, cause 13)
.section .text
.globl _start
.option norvc
_start:
    la      t0, _mtrap
    csrw    mtvec, t0
    la      sp, _stack_top

    # root[1] -> l1 (pointer PTE, V only)
    la      t0, root_tbl
    la      t1, l1_tbl
    srli    t1, t1, 12
    slli    t1, t1, 10
    ori     t1, t1, 0x01
    sd      t1, 8(t0)

    # l1[1] -> l0 (pointer PTE)
    la      t0, l1_tbl
    la      t1, l0_tbl
    srli    t1, t1, 12
    slli    t1, t1, 10
    ori     t1, t1, 0x01
    sd      t1, 8(t0)

    # l0[1] -> test_page (leaf RWXAD)
    la      t0, l0_tbl
    la      t1, test_page
    srli    t1, t1, 12
    slli    t1, t1, 10
    ori     t1, t1, 0xCF
    sd      t1, 8(t0)

    # l0[2] -> test_page2 (X-only leaf: V X A D = 0xC9 -> loads fault)
    la      t1, test_page2
    srli    t1, t1, 12
    slli    t1, t1, 10
    ori     t1, t1, 0xC9
    sd      t1, 16(t0)

    # seed test_page[0]
    la      t0, test_page
    li      t1, 0x1122334455667788
    sd      t1, 0(t0)

    # enable Sv39: satp = (8<<60) | rootPPN
    la      t0, root_tbl
    srli    t0, t0, 12
    li      t1, (8 << 60)
    or      t0, t0, t1
    csrw    satp, t0
    sfence.vma

    # MPRV with MPP=S so M-mode data accesses translate as S-mode
    li      t3, (3 << 11)
    csrc    mstatus, t3
    li      t2, (1 << 11)
    csrs    mstatus, t2
    li      t1, (1 << 17)
    csrs    mstatus, t1

    # 1: translated load VA 0x40201000 -> a0 (expect 0x1122334455667788)
    li      t4, 0x40201000
    ld      a0, 0(t4)

    # 2: translated store + readback -> a1 (expect 0xfeedfacefeedface)
    li      s3, 0xfeedfacefeedface
    sd      s3, 8(t4)
    ld      a1, 8(t4)

    # 3: load from X-only page -> load page fault (cause 13) -> a2
    li      s4, 0
    li      t4, 0x40202000
    ld      s2, 0(t4)
    mv      a2, s4

    # 4: non-canonical VA (bit 63 set) -> load page fault (cause 13) -> a3
    li      s4, 0
    li      t4, 1
    slli    t4, t4, 63
    li      t5, 0x40201000
    or      t4, t4, t5
    ld      s2, 0(t4)
    mv      a3, s4

    # clear MPRV, terminate
    li      t1, (1 << 17)
    csrc    mstatus, t1
    li      gp, 1
    ecall
1:  j       1b

.align 2
_mtrap:
    csrr    t6, mcause
    li      t5, 11                  # ECALL-from-M -> terminate
    beq     t6, t5, _term
    li      t5, 13                  # load page fault -> record cause, skip insn
    bne     t6, t5, _term
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
root_tbl:   .zero 4096
.align 12
l1_tbl:     .zero 4096
.align 12
l0_tbl:     .zero 4096
.align 12
test_page:  .zero 4096
.align 12
test_page2: .zero 4096

.pushsection .tohost,"aw",@progbits
.align 6
.globl tohost
tohost:   .dword 0
.globl fromhost
fromhost: .dword 0
.popsection
