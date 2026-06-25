# Directed Sv39 lockstep test: svadu_ad_transition  (primary golden: Spike)
#
# Corner (rank6, ad-bit-hw-update-svadu-svade): with a leaf PTE that has A=0,D=0,
#   - menvcfg.ADUE=1 (Svadu): a load HW-sets A; a store HW-sets D. The updated
#     PTE word is read back from physical memory into an integer GPR (observable).
#   - menvcfg.ADUE=0 (Svade): the same access must instead PAGE-FAULT (cause 13/15)
#     and leave the PTE untouched.
# Every effect lands in an integer GPR: loaded values, the read-back PTE word, and
# the fault cause recorded by the trap handler. M-mode builds the tables; data
# accesses translate as S-mode via MPRV (toggled off to touch the PTE physically).
#
# Tables in .page_table (0x8000_xxxx RAM all sims load). VA 0x40201000 ->
# VPN2=1,VPN1=1,VPN0=1 : root[1]->l1, l1[1]->l0, l0[1]=leaf. PTE flags:
#   V=1 R=2 W=4 X=8 U=16 A=64 D=128 ; leaf-no-AD = 0x0F ; +A = 0x4F ; +A+D = 0xCF
.section .text
.globl _start
.option norvc
_start:
    la      t0, _mtrap
    csrw    mtvec, t0
    la      sp, _stack_top

    # root[1] -> l1
    la      t0, root_tbl
    la      t1, l1_tbl
    srli    t1, t1, 12
    slli    t1, t1, 10
    ori     t1, t1, 0x01
    sd      t1, 8(t0)
    # l1[1] -> l0
    la      t0, l1_tbl
    la      t1, l0_tbl
    srli    t1, t1, 12
    slli    t1, t1, 10
    ori     t1, t1, 0x01
    sd      t1, 8(t0)
    # l0[1] -> test_page, leaf with A=0 D=0 (0x0F)
    la      s0, l0_tbl
    addi    s0, s0, 8                  # s0 = &PTE (l0[1]); reused throughout
    la      t1, test_page
    srli    t1, t1, 12
    slli    t1, t1, 10
    ori     t1, t1, 0x0F               # V|R|W|X, A=0 D=0
    sd      t1, 0(s0)
    # seed test_page[0]
    la      t0, test_page
    li      t1, 0x1122334455667788
    sd      t1, 0(t0)

    # enable Sv39
    la      t0, root_tbl
    srli    t0, t0, 12
    li      t1, (8 << 60)
    or      t0, t0, t1
    csrw    satp, t0
    sfence.vma

    # menvcfg.ADUE = 1 (bit 61): hardware A/D update on
    li      t1, 1
    slli    t1, t1, 61
    csrs    menvcfg, t1

    # MPP = S, ready to toggle MPRV per access
    li      t3, (3 << 11)
    csrc    mstatus, t3
    li      t2, (1 << 11)
    csrs    mstatus, t2

    # ---- Phase 1: Svadu ON ------------------------------------------------
    # translated load -> HW sets A
    li      t1, (1 << 17)
    csrs    mstatus, t1                # MPRV on
    li      t4, 0x40201000
    ld      a0, 0(t4)                  # = 0x1122334455667788
    csrc    mstatus, t1                # MPRV off
    ld      a1, 0(s0)                  # PTE read-back: expect A set -> 0x...4F

    # translated store -> HW sets D
    csrs    mstatus, t1                # MPRV on
    li      t5, 0xA5A5A5A5A5A5A5A5
    sd      t5, 0(t4)
    csrc    mstatus, t1                # MPRV off
    ld      a2, 0(s0)                  # PTE read-back: expect A+D -> 0x...CF
    csrs    mstatus, t1
    ld      a3, 0(t4)                  # readback stored value (= t5) via translation
    csrc    mstatus, t1

    # ---- Phase 2: Svade (ADUE=0): same access must FAULT, PTE untouched ----
    # reset PTE to A=0 D=0
    la      t1, test_page
    srli    t1, t1, 12
    slli    t1, t1, 10
    ori     t1, t1, 0x0F
    sd      t1, 0(s0)
    sfence.vma
    li      t1, 1
    slli    t1, t1, 61
    csrc    menvcfg, t1                # ADUE = 0 (Svade)

    li      s4, 0                      # fault-cause recorder
    li      t1, (1 << 17)
    csrs    mstatus, t1                # MPRV on
    li      t4, 0x40201000
    ld      s2, 0(t4)                  # A=0 + Svade -> load page fault (13)
    csrc    mstatus, t1                # MPRV off (handler returns here+...)
    mv      a4, s4                     # a4 = fault cause (expect 13)
    ld      a5, 0(s0)                  # PTE still 0x0F (A not set under Svade)

    li      gp, 1
    ecall
1:  j       1b

.align 2
_mtrap:
    csrr    t6, mcause
    li      t5, 11
    beq     t6, t5, _term
    li      t5, 13                     # load page fault -> record + skip
    beq     t6, t5, _rec
    li      t5, 15                     # store page fault -> record + skip
    beq     t6, t5, _rec
    j       _term                      # unexpected: terminate (will diverge)
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
root_tbl:   .zero 4096
.align 12
l1_tbl:     .zero 4096
.align 12
l0_tbl:     .zero 4096
.align 12
test_page:  .zero 4096

.pushsection .tohost,"aw",@progbits
.align 6
.globl tohost
tohost:   .dword 0
.globl fromhost
fromhost: .dword 0
.popsection
