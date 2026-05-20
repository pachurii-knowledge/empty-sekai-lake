.text
.global main
main:
    addi x5, x0, 21
    sw x5, -4(x2)
    lw x6, -4(x2)
    addi x11, x6, 21
    addi x10, x0, 10
    ecall
