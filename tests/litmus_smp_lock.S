# litmus_smp_lock.S -- M4 S4 SMP spinlock litmus. BOTH real cores run this; each self-differentiates
# on mhartid. They contend for an LR/SC test-and-set spinlock guarding a shared counter; each core does
# ITERS protected increments. If mutual exclusion holds (coherence + LR/SC reservation-kill correct), the
# final counter == NCORE*ITERS. A broken lock (both in the critical section) loses increments -> < that.
# Bare M-mode, position-independent (PC-relative branches + absolute data addrs in the CCD memory).
#
#   LOCK    = 0x100   (0 = free, 1 = held)
#   COUNTER = 0x140
#   DONE[h] = 0x180 + h*0x40   (per-core completion flag)

    .option arch, +zicsr
    .section .text
    .globl main
main:
    csrr    s0, mhartid          # 0 or 1
    li      t0, 0x100            # LOCK
    li      t1, 0x140            # COUNTER
    li      t2, 3                # ITERS per core
loop:
    beqz    t2, done
acq:
    lr.w    t3, (t0)             # load-reserved the lock
    bnez    t3, acq              # held -> spin
    li      t4, 1
    sc.w    t3, t4, (t0)         # try to take it
    bnez    t3, acq              # SC failed (lost reservation / contended) -> retry
    # --- critical section: counter++ ---
    lw      t5, (t1)
    addi    t5, t5, 1
    sw      t5, (t1)
    sw      zero, (t0)           # release the lock
    addi    t2, t2, -1
    j       loop
done:
    li      t6, 0x180            # DONE base
    slli    s1, s0, 6            # hartid * 64
    add     t6, t6, s1
    li      t5, 1
    sw      t5, (t6)             # publish this core's done flag
spin:
    j       spin
