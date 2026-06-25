# Directed Sv39 lockstep test: crosspage_split_ldsd  (primary golden: Spike)
#
# Corner (rank5/14, cross-page-split): a misaligned 8-byte access whose window
# straddles a 4 KiB page boundary. The LSQ splits it into two word-granular beats
# that translate SEPARATELY (low word in page A, high word in page B). Checks:
#   (1) cross-page store+load roundtrip when both pages map (the split datapath);
#   (2) the fault path when the HIGH page is invalid -- cause + stval (the audit
#       flagged stval=base-VA vs high-beat-VA as the suspected divergence).
# Effects land in integer GPRs (loaded values, fault cause, mtval). M-mode builds
# the tables; data accesses translate as S-mode via MPRV.
#
# VA 0x40201000 -> l0[1] = page A (RWXAD); VA 0x40202000 -> l0[2] = page B (RWXAD).
# A misaligned ld/sd at 0x40201FFC spans 0x40201FFC..0x40202003 (crosses 0x402020000).
.section .text
.globl _start
.option norvc
_start:
    la      t0, _mtrap
    csrw    mtvec, t0
    la      sp, _stack_top

    # root[1]->l1, l1[1]->l0
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
    # l0[1] = page A leaf RWXAD; l0[2] = page B leaf RWXAD
    la      s0, l0_tbl
    la      t1, page_a
    srli    t1, t1, 12
    slli    t1, t1, 10
    ori     t1, t1, 0xCF
    sd      t1, 8(s0)
    la      t1, page_b
    srli    t1, t1, 12
    slli    t1, t1, 10
    ori     t1, t1, 0xCF
    sd      t1, 16(s0)

    # enable Sv39
    la      t0, root_tbl
    srli    t0, t0, 12
    li      t1, (8 << 60)
    or      t0, t0, t1
    csrw    satp, t0
    sfence.vma
    # MPP = S
    li      t3, (3 << 11)
    csrc    mstatus, t3
    li      t2, (1 << 11)
    csrs    mstatus, t2

    # ---- Variant 1: cross-page misaligned store + load roundtrip ----------
    li      t1, (1 << 17)
    csrs    mstatus, t1                # MPRV on
    li      t4, 0x40201FFC             # straddles the page boundary
    li      t5, 0x0123456789ABCDEF
    sd      t5, 0(t4)                  # cross-page store split
    ld      a0, 0(t4)                  # cross-page load split -> expect t5
    # aligned reads to see each half landed
    li      t6, 0x40201FF8
    ld      a1, 0(t6)                  # page A around boundary (low 4 bytes of t5 at +4)
    li      t6, 0x40202000
    ld      a2, 0(t6)                  # page B start (high 4 bytes of t5)
    csrc    mstatus, t1                # MPRV off

    # ---- Variant 2: high page invalid -> fault on the high beat -----------
    sd      x0, 16(s0)                 # l0[2] = invalid
    sfence.vma
    li      s4, 0
    li      s5, 0
    li      t1, (1 << 17)
    csrs    mstatus, t1                # MPRV on
    li      t4, 0x40201FFC
    ld      s2, 0(t4)                  # high beat (0x40202000) faults
    csrc    mstatus, t1                # MPRV off
    mv      a3, s4                     # fault cause (expect 13 = load page fault)
    mv      a4, s5                     # mtval (base 0x40201FFC vs high 0x40202000?)

    # ---- Variant 3: same but a cross-page STORE fault ---------------------
    li      s4, 0
    li      s5, 0
    li      t1, (1 << 17)
    csrs    mstatus, t1
    li      t4, 0x40201FFC
    li      t5, 0xDEADBEEFCAFEF00D
    sd      t5, 0(t4)                  # high beat store faults (cause 15)
    csrc    mstatus, t1
    mv      a5, s4                     # store fault cause (expect 15)
    mv      a6, s5                     # mtval
    # confirm the LOW page was NOT written (whole-store atomicity on fault)
    li      t1, (1 << 17)
    csrs    mstatus, t1
    li      t6, 0x40201FF8
    ld      a7, 0(t6)
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
    li      t5, 4                      # load misaligned (if a sim traps misaligned)
    beq     t6, t5, _rec
    li      t5, 6                      # store misaligned
    beq     t6, t5, _rec
    j       _term
_rec:
    mv      s4, t6
    csrr    s5, mtval
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
