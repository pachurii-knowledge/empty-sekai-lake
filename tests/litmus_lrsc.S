# litmus_lrsc.S -- M3d Stage 3 reservation-coherence-kill litmus (core 0 = the real core).
#
# core 0 reserves a shared line (lr.w), spins to open a wide window, then attempts sc.w.
# The behavioural peer (core 1) writes the shared line DURING the spin, which snoops core 0's
# agent (FwdGetM) -> snoop_kill -> the LSQ reservation coherence-kill -> the sc.w MUST FAIL.
# The result code is published to RESULT so the harness can check it:
#   RESULT = 1  -> sc.w succeeded   (negative control: peer wrote a DIFFERENT line)
#   RESULT = 2  -> sc.w failed      (the kill fired -- the expected contended outcome)
#
# Bare M-mode (satp=0, no paging -> PTW never fires). Data lives in the CCD's coherent memory
# (cacheable, below the device hole); code lives in main_memory served via the ifetch port.

    .section .text
    .globl main                  # crt0.S _start does `call main`
main:
    li      t3, 0x100            # shared line (cacheable data, in the CCD memory)
    li      t4, 0x140            # result word (a different line)
    li      t5, 0xDEAD           # "running" sentinel
    sw      t5, 0(t4)            # publish a not-done marker first

    lr.w    t0, (t3)             # establish the reservation on the shared line

    li      t1, 400              # spin: open a wide LR->SC window for the peer's remote write
1:  addi    t1, t1, -1
    bnez    t1, 1b

    li      t2, 0x55
    sc.w    a0, t2, (t3)         # a0 = 0 success / 1 fail
    addi    a0, a0, 1            # encode: 1 = success, 2 = fail
    sw      a0, 0(t4)            # publish the result (overwrites the sentinel)

2:  j       2b                   # done -- the harness polls RESULT and finishes
