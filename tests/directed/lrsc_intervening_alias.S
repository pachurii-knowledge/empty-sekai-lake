# Directed lockstep test: lrsc_intervening_alias
#
# Corner: LR/SC reservation invalidation by an intervening store that ALIASES
# the reserved location. Exercises the reservation-tracking risk
# "amo-lrsc-reservation-va-not-pa (rank15)": the OoO LSQ tracks the LR
# reservation by address and clears it on a committing store whose address
# matches the reservation address (src/load_store_queue.sv reservation_addr_q
# / the store-commit clear). The SC result rd (0=success, 1=fail) is the only
# integer-GPR-visible effect, so we capture each SC's rd into a dedicated GPR.
#
# Sub-cases (all in M-mode, no paging; identity VA==PA so the VA-vs-PA risk is
# probed through the reservation-granule comparison):
#   (a) lr.d t0,(a0); sd x0,0(a0)  [SAME 8-byte word]; sc.d -> SC MUST FAIL.
#       Architecturally well-defined: the intervening store hits the reserved
#       doubleword, so the reservation is invalidated and SC fails (rd=1).
#       Both golden models must agree; a DUT that leaves SC succeeding here
#       (rd=0) is the candidate bug.
#   (b) lr.w t0,0(a0); sw x0,4(a0) [OTHER 32-bit half of the 8-byte word];
#       sc.w 0(a0). Spec-permitted either way (reservation set size is
#       implementation-defined). We capture rd and let the goldens define the
#       reference; a DUT/golden split here is an oracle-split, not a bug.
#   (c) Two stores then a fresh LR/SC pair to confirm a clean SC succeeds when
#       NO intervening store aliased the reservation (control: rd=0 expected).
#
# Trace-compared against Spike AND Sail; the goldens ARE the check. We end with
# li gp,1 ; ecall into a handler that writes gp to tohost (HTIF exit).

.section .text
.globl _start
.option norvc
_start:
    la      t0, _trap
    csrw    mtvec, t0
    la      sp, _stack_top

    la      a0, _scratch            # 8-byte aligned reservation target

    # --- (a) intervening store to the SAME doubleword: SC must FAIL ----------
    li      t2, 0x1111              # SC store value (irrelevant; SC should fail)
    li      t3, 0xCAFE              # seed value
    sd      t3, 0(a0)
    lr.d    t0, (a0)                # reserve [a0 .. a0+7]
    sd      x0, 0(a0)               # intervening store, SAME word -> clears resv
    sc.d    s4, t2, (a0)            # s4 = SC result (expect 1 = FAIL)

    # --- (b) intervening store to the OTHER half of the 8-byte word ----------
    li      t2, 0x2222
    li      t3, 0x5A5A5A5A
    sd      t3, 0(a0)
    lr.w    t0, 0(a0)               # reserve naturally-aligned word at a0+0
    sw      x0, 4(a0)               # store to a0+4 (other 32-bit half)
    sc.w    s5, t2, 0(a0)           # s5 = SC result (spec-permitted either way)

    # --- (c) control: NO intervening alias store, SC should SUCCEED ----------
    li      t2, 0x3333
    li      t3, 0x9999
    sd      t3, 0(a0)
    lr.d    t0, (a0)                # reserve, no intervening store
    sc.d    s6, t2, (a0)            # s6 = SC result (expect 0 = SUCCESS)

    # Load back the post-(c) memory word to confirm the SC store landed when it
    # succeeded (makes the memory effect visible in an integer GPR).
    ld      s7, 0(a0)               # = t2 (0x3333) if (c) SC wrote, else 0x9999

    # --- (d) re-reserve and check SC-after-successful-SC fails (reservation
    #         is consumed by the prior SC) -------------------------------------
    li      t2, 0x4444
    sc.d    s8, t2, (a0)            # s8 = SC with no live reservation -> FAIL(1)

    # --- terminate ----------------------------------------------------------
    li      gp, 1
    ecall
1:  j       1b

.align 4
_trap:
    csrr    t0, mcause
    li      t1, 11                  # ECALL-from-M
    beq     t0, t1, _htif_exit
    # Unexpected trap: record cause and skip the faulting instruction so the
    # run stays deterministic across all sims.
    csrr    s0, mcause
    csrr    s1, mtval
    csrr    t1, mepc
    addi    t1, t1, 4
    csrw    mepc, t1
    mret
_htif_exit:
    la      t1, tohost
    sw      gp, 0(t1)               # HTIF exit, value 1
1:  j       1b

.section .data
.align 12
_scratch: .dword 0
          .dword 0
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
