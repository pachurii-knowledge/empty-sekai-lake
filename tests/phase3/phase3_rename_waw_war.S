.text
.global main
main:
    addi x5, x0, 1
    addi x6, x5, 2
    addi x5, x0, 7
    addi x7, x6, 3
    addi x11, x7, 0
    addi x10, x0, 10
    ecall
