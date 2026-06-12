    # misaligned_line_cross: misaligned loads/stores that cross a word boundary
    # and, at byte 62/126, a 64 B cache-line boundary -- so the LSQ's two-beat
    # split lands the halves in two different L1D lines. Each value is stored and
    # read back; the stores are non-overlapping so the final re-read confirms the
    # cross-line halves were not clobbered.
    .option arch, +zicsr
    .equ BASE, 0x10000000
    .section .text
    .globl main
main:
    li      s0, BASE
    # word at byte 62: spans bytes 62..65 -> crosses the line-0/line-1 boundary
    li      t0, 0xCAFEBABE
    addi    t2, s0, 62
    sw      t0, 0(t2)
    lw      t3, 0(t2)
    bne     t3, t0, fail
    # halfword at byte 126: spans bytes 126..127 -> crosses the line-1/line-2
    # boundary (non-overlapping with byte 62)
    li      t0, 0x1234
    addi    t2, s0, 126
    sh      t0, 0(t2)
    lh      t3, 0(t2)
    bne     t3, t0, fail
    # word at byte 30: crosses a word boundary within line 0 (no line cross)
    li      t0, 0x0BADF00D
    addi    t2, s0, 30
    sw      t0, 0(t2)
    lw      t3, 0(t2)
    bne     t3, t0, fail
    # re-read the first value: byte 62..65 were written only by the first store
    addi    t2, s0, 62
    lw      t3, 0(t2)
    li      t0, 0xCAFEBABE
    bne     t3, t0, fail
    li      a0, 10
    ecall
fail:
    li      a0, 11
    ecall
9:  j       9b
