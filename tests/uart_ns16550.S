    # Phase H2: NS16550A register-model test (the bits the Linux 8250 driver
    # probes during autoconfig + the RX path), reg-shift=2 layout.
    #
    #   1. Scratch register round-trip (presence probe): SCR @ +0x1C.
    #   2. Divisor latch under LCR.DLAB: write/read DLL @ +0x0.
    #   3. FCR FIFO-enable -> IIR top bits report 16550A (IIR[7:6]=11).
    #   4. MCR loopback -> MSR autoconfig pattern: MCR=LOOP|OUT2|RTS (0x1A) makes
    #      (MSR & 0xF0) == 0x90 (this is exactly the 8250 autoconfig check).
    #   5. Loopback data path: a THR byte loops to RBR, sets LSR.DR, RBR returns it.
    #
    # Registers (reg-shift=2): THR/RBR +0, IER +4, IIR/FCR +8, LCR +0xC,
    #   MCR +0x10, LSR +0x14, MSR +0x18, SCR +0x1C.
    # a0=10 pass; a0=11 fail, a1 = 61 scratch / 62 DLL / 63 FIFO / 64 loopback MSR
    #   / 65 loopback RX.
    .option arch, +zicsr
    .section .text
    .globl main
main:
    li      s0, 0x0D000000

    # 1. scratch round-trip
    li      t1, 0xA5
    sb      t1, 0x1C(s0)
    lbu     t2, 0x1C(s0)
    li      t3, 0xA5
    bne     t2, t3, f_scr
    li      t1, 0x5A
    sb      t1, 0x1C(s0)
    lbu     t2, 0x1C(s0)
    li      t3, 0x5A
    bne     t2, t3, f_scr

    # 2. divisor latch under DLAB
    li      t1, 0x80
    sb      t1, 0xC(s0)             # LCR.DLAB = 1
    li      t1, 0x03
    sb      t1, 0x0(s0)             # DLL = 3
    lbu     t2, 0x0(s0)
    li      t3, 0x03
    bne     t2, t3, f_dll
    sb      x0, 0xC(s0)             # LCR = 0 (clear DLAB)

    # 3. FCR FIFO-enable -> IIR[7:6] = 11
    li      t1, 0x01
    sb      t1, 0x8(s0)             # FCR bit0 = FIFO enable
    lbu     t2, 0x8(s0)             # IIR
    andi    t2, t2, 0xC0
    li      t3, 0xC0
    bne     t2, t3, f_fifo

    # 4. MCR loopback -> MSR autoconfig pattern (MSR & 0xF0 == 0x90)
    li      t1, 0x1A                # LOOP | OUT2 | RTS
    sb      t1, 0x10(s0)
    lbu     t2, 0x18(s0)            # MSR
    andi    t2, t2, 0xF0
    li      t3, 0x90
    bne     t2, t3, f_loop

    # 5. loopback data path: THR byte -> RBR, LSR.DR set, RBR returns it
    li      t1, 0x6B               # 'k'
    sb      t1, 0x0(s0)            # THR -> loops back to RBR
    nop; nop; nop                  # let rx_valid register
    lbu     t2, 0x14(s0)           # LSR
    andi    t2, t2, 0x1            # DR
    beqz    t2, f_rx
    lbu     t2, 0x0(s0)            # RBR
    li      t3, 0x6B
    bne     t2, t3, f_rx

    sb      x0, 0x10(s0)           # MCR = 0
    li      a0, 10
    ecall
0:  j       0b

f_scr:  li a1, 61; j fail
f_dll:  li a1, 62; j fail
f_fifo: li a1, 63; j fail
f_loop: li a1, 64; j fail
f_rx:   li a1, 65
fail:
    li      a0, 11
    ecall
9:  j       9b
