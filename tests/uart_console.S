    # NS16550-subset UART console test (M-mode).
    #
    # 1. Write a string to THR (byte stores) -> simulation console ($write).
    # 2. Read LSR and confirm the transmitter-ready bits (THRE|TEMT = 0x60).
    # 3. Enable the THR-empty interrupt (IER bit1); route UART -> PLIC source 10
    #    -> machine-external interrupt. The handler claims (must read 10), clears
    #    the source by disabling IER, completes, and the interrupt does not refire.
    #
    # UART base 0x0D000000 (reg-shift=2, one register per word):
    #   THR @ +0x0, IER @ +0x4, IIR @ +0x8, LCR @ +0xC, LSR @ +0x14
    # PLIC base 0x0C000000:  priority[10] @ +0x28, pending @ +0x1000,
    #   enable[ctx0] @ +0x2000, threshold[ctx0] @ +0x200000, claim @ +0x200004
    #
    # a0=10 pass; a0=11 fail, a1 = 71 bad LSR / 72 no interrupt /
    #                              73 wrong claim id / 74 refired after complete.
    .option arch, +zicsr
    .section .text
    .globl main
main:
    la      t0, handler
    csrw    mtvec, t0
    li      s0, 0x0D000000          # UART base
    li      s2, 0x0C000000          # PLIC base
    li      s1, 0                   # interrupt count

    # ---- 1. console string ("niigo UART online\n") via THR byte stores ----
    # Chars are written as immediates to avoid the .data/la VMA-vs-load mismatch
    # in the standalone test harness (load_elf_mem remaps sections by name).
    .macro PUTC ch
    li      t1, \ch
    sb      t1, 0(s0)
    .endm
    PUTC 'n'
    PUTC 'i'
    PUTC 'i'
    PUTC 'g'
    PUTC 'o'
    PUTC ' '
    PUTC 'U'
    PUTC 'A'
    PUTC 'R'
    PUTC 'T'
    PUTC ' '
    PUTC 'o'
    PUTC 'n'
    PUTC 'l'
    PUTC 'i'
    PUTC 'n'
    PUTC 'e'
    PUTC '\n'

    # ---- 2. LSR readiness ----
    lbu     t1, 0x14(s0)            # LSR (reg-shift=2)
    li      t2, 0x60
    bne     t1, t2, fail_lsr

    # ---- 3. UART -> PLIC source 10 -> M-external interrupt ----
    li      t1, 5
    sw      t1, 0x28(s2)            # priority[10] = 5
    li      t1, (1 << 10)
    li      t2, 0x0C002000
    sw      t1, 0(t2)               # enable source 10 for context 0
    li      t2, 0x0C200000
    sw      x0, 0(t2)               # threshold[ctx0] = 0

    li      t1, (1 << 11)           # MEIE
    csrw    mie, t1
    csrsi   mstatus, 0x8            # MIE

    li      t1, 0x02
    sb      t1, 4(s0)               # IER bit1 = THR-empty int -> uart_irq high

    li      t3, 1000
3:  addi    t3, t3, -1              # spin; the interrupt should preempt us
    bnez    t3, 3b

    bnez    s1, pass                # interrupt fired and was handled -> pass
    li      a1, 72                  # reached here with no interrupt
    j       fail

pass:
    li      a0, 10
    ecall
0:  j       0b

fail_lsr:
    li      a1, 71
fail:
    li      a0, 11
    ecall
9:  j       9b

    .align 2
handler:
    csrr    t0, mcause
    # Machine external interrupt. The interrupt flag is mcause's MSB (bit
    # XLEN-1), so test the sign and the low cause bits XLEN-portably -- the
    # RV32 magic 0x8000000B puts the flag at bit 31, wrong on RV64 (bit 63).
    bge     t0, x0, fail            # not an interrupt (MSB clear) -> unexpected
    andi    t0, t0, 0xff            # cause field
    li      t1, 11                  # external interrupt
    bne     t0, t1, fail

    li      t2, 0x0C200004          # claim/complete[ctx0]
    lw      t3, 0(t2)               # claim
    li      t4, 10
    bne     t3, t4, claim_bad

    sb      x0, 4(s0)               # IER = 0 -> clear uart_irq (source deasserts)
    sw      t3, 0(t2)               # complete source 10
    addi    s1, s1, 1
    li      t5, 2
    bge     s1, t5, refired         # second entry => it refired
    mret

claim_bad:
    li      a1, 73
    j       fail
refired:
    li      a1, 74
    j       fail
