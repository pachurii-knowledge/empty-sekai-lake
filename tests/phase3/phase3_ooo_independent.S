.text
.global main
main:
    addi x5, x0, 11
    addi x6, x0, 13
    addi x7, x0, 17
    addi x8, x0, 19
    add x9, x5, x6
    add x10, x7, x8
    add x11, x10, x9
    addi x10, x0, 10
    ecall
