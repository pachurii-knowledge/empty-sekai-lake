# Directed lockstep smoke test (rv64) -- validates the ASM= harness in
# run_riscvdv.sh end to end. Trace-compared against Spike AND Sail; the golden
# models ARE the check (every integer GPR write is diffed), so no self-check.
#
# Convention (shared by every directed test): link with riscv-dv
# scripts link.ld (ENTRY=_start at 0x8000_0000, a .tohost section); run the
# deterministic body in M-mode; terminate with `ecall` (the trace converters'
# stop point on all three sims) into a trap handler that writes gp to `tohost`
# (HTIF exit) so Spike/Sail/niigo all halt cleanly.
.section .text
.globl _start
.option norvc
_start:
    la      t0, _trap
    csrw    mtvec, t0
    la      sp, _stack_top

    # --- body: rv64 W-ops + 64-bit shifts + a store->load forward ----------
    li      a0, 0x123456789         # 64-bit immediate
    addiw   a1, a0, 0x10            # W-form: low 32b add, sign-extend to 64
    slli    a2, a0, 33             # shamt > 31 (rv64 6-bit shamt)
    srai    a3, a2, 40
    addw    a4, a0, a1             # W-form add of two 64b values
    la      t1, _scratch
    sd      a0, 0(t1)
    ld      a5, 0(t1)              # aligned store->load forward
    lwu     a6, 0(t1)             # zero-extended word load
    lw      a7, 4(t1)             # sign-extended high word load

    # --- terminate ---------------------------------------------------------
    li      gp, 1
    ecall
1:  j       1b

.align 4
_trap:
    la      t1, tohost
    sw      gp, 0(t1)              # HTIF exit, value 1
1:  j       1b

.section .data
.align 8
_scratch: .dword 0
.align 12
_stack:   .rept 256
          .dword 0
          .endr
_stack_top:

.pushsection .tohost,"aw",@progbits
.align 6
.globl tohost
tohost:   .dword 0
.globl fromhost
fromhost: .dword 0
.popsection
