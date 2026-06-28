# Cross-core self-modifying-code litmus (P4 remote-dirty I-fetch).
# Width-agnostic: only csrr/auipc/addi/lui/j/beq/lw/sw/fence (identical bits at
# RV32 and RV64; lw/sw used for the 32-bit flag/result even on RV64).
#
# hart 0 (producer): patches the FIRST word of the `patch` cache line from
#   "addi a0,x0,0x111" (WRONG) to "addi a0,x0,0x222" (RIGHT), fences, sets FLAG.
# hart 1 (consumer): spins on FLAG, then JUMPS to `patch` (a line it has never
#   fetched -> cold L1I miss -> must pull hart 0's DIRTY line via the directory).
#   The patched code writes a0 (0x222 if coherent, 0x111 if it read stale memory)
#   to RESULT and spins.
.option norvc
.section .text
.globl _start
_start:
    csrr  s0, mhartid
    bne   s0, x0, consumer
# ---- producer (hart 0) ----
producer:
1:  auipc t0, %pcrel_hi(patch)  # t0 = &patch (PC-relative, width-agnostic)
    addi  t0, t0, %pcrel_lo(1b)
    lui   t1, 0x22200           # build the word "addi a0,x0,0x222" = 0x22200513
    addi  t1, t1, 0x513
    sw    t1, 0(t0)             # PATCH the code word (dirty in hart0 L1D)
    fence rw, rw
    addi  t2, x0, 1
    addi  t3, x0, 0x100         # FLAG
    sw    t2, 0(t3)             # release: FLAG = 1
prod_spin:
    j     prod_spin
# ---- consumer (hart 1) ----
consumer:
    addi  t3, x0, 0x100         # FLAG
cwait:
    lw    t4, 0(t3)
    beq   t4, x0, cwait         # spin until FLAG != 0
    j     patch                 # control-dependent jump to the patched line
# ---- the patch slot: its own 64 B cache line; run only by the consumer ----
.balign 64
patch:
    addi  a0, x0, 0x111         # ORIGINAL (WRONG); producer overwrites -> 0x222
    addi  t5, x0, 0x140         # RESULT
    sw    a0, 0(t5)
    j     .
