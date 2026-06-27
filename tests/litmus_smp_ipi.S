# litmus_smp_ipi.S -- M4 S6b cross-hart IPI through the shared CLINT.
# Both harts run this (branch on mhartid). Linked at USER_TEXT_START=0x00400000
# (the harness behavioural ifetch base). hart 0 raises hart 1's msip (an IPI);
# hart 1 spins with M-software interrupts enabled, takes the trap, and the
# handler writes SENTINEL=1 to RAM (proving the trap was delivered).
#
#   CLINT base 0x0200_0000 ; msip[h] = base + 4*h
#   SENTINEL  = 0x200 (RAM, cacheable -> through the CCD directory)
#
# Assemble: riscv64-unknown-elf-gcc -march=rv32ima_zicsr -mabi=ilp32 -nostdlib \
#             -Wl,-Ttext=0x00400000 -o litmus_smp_ipi.elf litmus_smp_ipi.S
    .text
    .globl _start
_start:
    csrr  s0, mhartid          # s0 = hart id
    la    t0, handler          # mtvec = handler (PC-relative -> abs 0x00400000+off)
    csrw  mtvec, t0
    li    t0, 8                # bit 3 = {mie.MSIE, mstatus.MIE}
    csrs  mie, t0              # enable M software interrupt
    csrs  mstatus, t0          # global M interrupt enable
    bnez  s0, receiver

sender:                        # hart 0 -> IPI to hart 1
    li    t0, 0x02000000       # CLINT base
    li    t1, 1
    sw    t1, 4(t0)            # msip[1] = 1
spin0:
    j     spin0

receiver:                      # hart != 0: wait with interrupts on
spin1:
    j     spin1                # the IPI traps to handler

    .align 2
handler:                       # per-hart sentinel: SENTINEL[h] = 0x200 + 4*h
    csrr  s0, mhartid
    slli  s1, s0, 2            # s1 = 4*hart (reused for sentinel + msip)
    li    t2, 0x200
    add   t2, t2, s1
    li    t3, 1
    sw    t3, 0(t2)            # SENTINEL[hart] = 1  (this hart took the trap)
    li    t0, 0x02000000       # clear own msip[hart]
    add   t0, t0, s1
    sw    zero, 0(t0)          # msip[hart] = 0
hspin:
    j     hspin
