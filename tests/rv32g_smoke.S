    .section .text
    .globl main
main:
    li      x5, 7
    li      x6, -3
    mul     x7, x5, x6
    mulh    x8, x5, x6
    li      x28, -22
    div     x23, x28, x6
    rem     x24, x28, x6

    csrrw   x9, fflags, x0
    csrwi   frm, 1
    csrr    x25, frm
    fence
    fence.i

    addi    x20, x3, 0
    fence
    lr.w    x11, (x20)
    li      x12, 11
    sc.w    x13, x12, (x20)
    lw      x14, 0(x20)

    li      x15, 0x3fc00000
    fmv.w.x f1, x15
    fadd.s  f2, f1, f1
    fmv.x.w x16, f2
    li      x17, 0x40000000
    fmv.w.x f3, x17
    fmul.s  f4, f1, f3
    fmv.x.w x18, f4

    la      x19, double_data
    fld     f5, 0(x19)
    fld     f6, 8(x19)
    fadd.d  f7, f5, f6
    fsd     f7, 16(x19)
    lw      x21, 16(x19)
    lw      x22, 20(x19)

    li      x10, 10
    ecall

    .section .data
    .align 3
word_data:
    .word   5
double_data:
    .word   0x00000000
    .word   0x3ff80000
    .word   0x00000000
    .word   0x40000000
    .word   0x00000000
    .word   0x00000000
