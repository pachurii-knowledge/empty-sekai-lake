.text
.global main
main:
    addi t0, zero, 1
    beq  t0, t0, target
    addi t1, zero, 99
    addi t2, zero, 99
target:
    addi a0, zero, 10
    ecall
