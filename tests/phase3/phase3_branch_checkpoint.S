.text
.global main
main:
    addi x5, x0, 1
    beq x5, x0, wrong
    addi x5, x0, 9
    jal x0, done
wrong:
    addi x5, x0, 3
done:
    addi x11, x5, 0
    addi x10, x0, 10
    ecall
