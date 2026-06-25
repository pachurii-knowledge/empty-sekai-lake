# Directed lockstep torture test (rv64): two-adjacent-stores / one-commit-pulse.
#
# Corner (risk id: two-stores-one-commit-pulse):
#   When two stores retire in the SAME 4-wide commit group with both done and
#   the store port ready, ooo_commit_unit.sv:39-63 sets retire_valid for both
#   lanes but commit_store_id is a SINGLE signal -- lane 1 (younger) overwrites
#   lane 0 (older). In load_store_queue.sv the store-commit only fires for the
#   entry whose active_id == commit_store_id, so the OLDER store (the LSQ head)
#   never gets its write driven -> dropped store write or LSQ head stall.
#
# To hit it deterministically: keep two SDs to two DIFFERENT addresses (two
# different cachelines, 64 B apart) ADJACENT with no intervening op so they
# cluster into one commit group, repeat the pattern, then load BOTH back and
# branch-check. A dropped lane-0 store leaves the memory word stale -> the load
# reads the OLD value -> t0 != a1 -> bne taken to fail (writes a distinct GPR
# value) so the integer-GPR trace diverges from Spike/Sail (or the DUT hangs,
# stop_reason != ecall).
#
# Observability: store effects are invisible in the GPR trace, so we LOAD both
# words back into GPRs (t0/t1) every iteration and additionally accumulate a
# running checksum of the loaded values into s2 so any dropped/late store flips
# an integer GPR value that the lockstep oracle compares.

.section .text
.globl _start
.option norvc
_start:
    la      t0, _trap
    csrw    mtvec, t0
    la      sp, _stack_top

    # a4 -> word in cacheline A, a5 -> word in cacheline B (64 B apart).
    la      a4, _bufA
    la      a5, _bufB
    li      s2, 0                   # checksum accumulator (observable)
    li      s3, 0                   # iteration counter / store-value seed
    li      s4, 64                  # iteration count

.align 4
_loop:
    # Derive two distinct, varying store values from the seed s3.
    slli    a1, s3, 4
    addi    a1, a1, 0x55            # value for store A
    xori    a2, a1, -1             # value for store B (bitwise complement)

    # --- THE CORNER: two adjacent SDs, no intervening op, two cachelines -----
    sd      a1, 0(a4)
    sd      a2, 0(a5)
    # Load both back. A dropped older (lane-0) store -> t0 holds the stale word.
    ld      t0, 0(a4)
    ld      t1, 0(a5)
    bne     t0, a1, _fail          # store A must be visible
    bne     t1, a2, _fail          # store B must be visible
    # Fold both loaded values into the observable checksum.
    add     s2, s2, t0
    add     s2, s2, t1

    # --- tight variant: FOUR stores back-to-back into one commit group ------
    # Four SDs with no intervening ops -> up to 4 stores in one 4-wide group,
    # maximally stressing the single commit_store_id pulse.
    sd      a1, 8(a4)
    sd      a2, 8(a5)
    sd      a2, 16(a4)
    sd      a1, 16(a5)
    ld      t2, 8(a4)
    ld      t3, 8(a5)
    ld      t4, 16(a4)
    ld      t5, 16(a5)
    bne     t2, a1, _fail
    bne     t3, a2, _fail
    bne     t4, a2, _fail
    bne     t5, a1, _fail
    add     s2, s2, t2
    add     s2, s2, t3
    add     s2, s2, t4
    add     s2, s2, t5

    addi    s3, s3, 1
    bne     s3, s4, _loop

    # --- terminate: checksum is in s2; gp=1 ; ecall -------------------------
    li      gp, 1
    ecall
1:  j       1b

_fail:
    # Distinct, observable failure marker (also terminates so the trace ends).
    li      s5, 0xBAD
    li      gp, 1
    ecall
2:  j       2b

.align 4
_trap:
    la      t6, tohost
    sw      gp, 0(t6)              # HTIF exit, value 1
1:  j       1b

.section .data
.align 12
# Two buffers, each at the start of its own 64 B cacheline, far apart so the
# two SDs never alias and the loads can't store-to-load-forward from the wrong
# entry. Each buffer has room for the three offsets (0, 8, 16) used above.
_bufA:    .rept 8
          .dword 0
          .endr
.align 12
_bufB:    .rept 8
          .dword 0
          .endr
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
