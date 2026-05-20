.text
.global main
main:
    addi x5, x0, 42
    sw x5, -4(x2)
    lw x11, -4(x2)
    addi x10, x0, 10
    ecall
