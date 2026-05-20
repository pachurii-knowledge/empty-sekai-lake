.text
.global main
main:
    addi x5, x0, 1
    bne x5, x0, level1
    addi x11, x0, 99
level1:
    beq x5, x0, bad
    bne x5, x0, level2
bad:
    addi x11, x0, 98
level2:
    addi x11, x0, 10
    addi x10, x0, 10
    ecall
