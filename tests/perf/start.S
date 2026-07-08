    # Minimal freestanding startup for perf microbenchmarks.
    # Placed in .text.init so the linker script puts _start at the reset vector
    # (0x00400000). Sets up sp in the sim's stack segment, calls main, then ends
    # simulation via the ECALL-halt convention (a0 == 0xa).
    .section .text.init,"ax",@progbits
    .global _start
_start:
    li    sp, 0x7ff7fff0          # top of the sim stack segment (STACK_END-16)
.option push
.option norelax
    la    gp, __global_pointer$
.option pop
    call  main
    li    a0, 0xa                 # ECALL-halt: a0 == 10 terminates simulation
    ecall
1:  j     1b
