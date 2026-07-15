    # Startup for the Dhrystone port (tests/perf/dhry).
    #
    # Same contract as tests/perf/start.S -- .text.init lands at the reset vector
    # (0x00400000), set sp/gp, run, then end the sim via the ECALL-halt (a0 == 10).
    # The one difference: we call dhry_entry() (env.c) rather than main() directly,
    # so the env can run its post-hoc result validation once main() has returned.
    # Dhrystone's own sources are used verbatim; nothing here is benchmark code.
    .section .text.init,"ax",@progbits
    .global _start
_start:
    li    sp, 0x7ff7fff0          # top of the sim stack segment (STACK_END-16)
.option push
.option norelax
    la    gp, __global_pointer$
.option pop
    call  dhry_entry
    li    a0, 0xa                 # ECALL-halt: a0 == 10 terminates simulation
    ecall
1:  j     1b
