    # Phase H2: interactive RX validation. Bytes injected via +uart_in=<string>
    # are delivered through RBR (LSR.DR), consumed on read, and the next byte
    # advances. This test polls DR, reads RBR, echoes it to THR, for 2 bytes,
    # then halts -- run with +uart_in=Hi and grep the console for "Hi".
    #
    #   make assemble TEST=tests/uart_rx_echo.S OUTPUT=output/uartrx
    #   (cd output/uartrx && ../simulation/verilator_obj/Vtop +uart_in=Hi)
    #
    # The DR poll is bounded so the test halts (a0=11, a1=66) rather than hang
    # when no input was injected.
    .option arch, +zicsr
    .section .text
    .globl main
main:
    li      s0, 0x0D000000
    li      s1, 2                   # bytes to echo

loop:
    beqz    s1, done
    li      t4, 200000              # bounded wait for DR
1:  lbu     t2, 0x14(s0)            # LSR
    andi    t2, t2, 0x1             # DR
    bnez    t2, 2f
    addi    t4, t4, -1
    bnez    t4, 1b
    li      a1, 66                  # no RX byte arrived
    j       fail
2:  lbu     t2, 0x0(s0)             # RBR (consumes the byte)
    sb      t2, 0x0(s0)             # echo to THR (console)
    addi    s1, s1, -1
    j       loop

done:
    li      a0, 10
    ecall
0:  j       0b
fail:
    li      a0, 11
    ecall
9:  j       9b
