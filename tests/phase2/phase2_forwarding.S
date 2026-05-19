.text
.global main
main:
    addi t0, zero, 1
    addi t1, t0, 2
    add  t2, t1, t0
    addi t3, t2, 4
    addi a0, zero, 10
    ecall
