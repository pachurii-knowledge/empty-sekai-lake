# litmus_smp_amo.S -- M4 #3 AMO atomicity. Both harts run this (branch on mhartid
# only for the done flag). Each hart does ITERS atomic `amoadd.w +1` to a shared
# COUNTER; the final value == NCORE*ITERS iff every RMW was atomic (no lost update
# from a remote write landing between this hart's AMO read and its commit-store).
# Linked at USER_TEXT_START=0x00400000 (harness behavioural ifetch base).
#   COUNTER = 0x140 (RAM, cacheable -> through the CCD directory)
#   DONE[h] = 0x180 + 4*h
    .text
    .globl _start
_start:
    csrr  s0, mhartid
    li    t0, 0x140            # COUNTER
    li    t1, 3                # ITERS  (must match the tb)
    li    t2, 1                # increment
loop:
    beqz  t1, done
    amoadd.w zero, t2, (t0)    # COUNTER += 1, atomically; discard the old value
    addi  t1, t1, -1
    j     loop
done:
    li    t6, 0x180           # DONE base
    slli  s1, s0, 2
    add   t6, t6, s1
    li    t5, 1
    sw    t5, 0(t6)           # DONE[hart] = 1
spin:
    j     spin
