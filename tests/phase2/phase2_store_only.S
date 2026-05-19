.text
.global main
main:
    addi t0, zero, 42
    sw   t0, -4(sp)
    addi a0, zero, 10
    ecall
