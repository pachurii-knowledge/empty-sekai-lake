# Directed lockstep test: ITTAGE indirect-target mispredict + recovery.
#
# Corner (risk: ittage-indirect-target-mispredict-recovery): a computed jump
# table whose index varies UNPREDICTABLY (driven by an LFSR), so the ITTAGE
# indirect predictor frequently mispredicts the JALR target. Each case target
# has a DEPENDENT ALU op immediately after the jalr that, if the wrong target
# is speculatively executed, would corrupt a0 unless the OoO core squashes the
# younger ops (branch_mask carrying this JALR's id) and redirects to the
# resolved (src1+imm) target. The case targets are spread onto DIFFERENT 4KiB
# pages (.align 12) so a speculative fetch of a wrong predicted target couples
# to the fetch-translation path. We accumulate a0 by a per-case weight; the
# final a0 (committed integer GPR) is the observable. A mis-squash on the
# target-mispredict recovery shows up as a wrong a0 vs Spike+Sail.
#
# All deterministic, M-mode, terminate with `li gp,1 ; ecall`.
.section .text
.globl _start
.option norvc
_start:
    la      t0, _trap
    csrw    mtvec, t0
    la      sp, _stack_top

    # s0 = LFSR state (16-bit, taps 0xB400 -> x^16+x^14+x^13+x^11+1).
    li      s0, 0xACE1            # nonzero LFSR seed
    li      s1, 64               # loop iteration count
    li      a0, 0                # accumulator (the observable)
    la      t0, jtbl             # base of the dword jump table

loop:
    andi    t1, s0, 7            # index 0..7 (unpredictable from LFSR low bits)
    slli    t1, t1, 3            # *8 (dword stride)
    add     t1, t0, t1
    ld      t2, 0(t1)            # load case target address
    jalr    x0, 0(t2)            # indirect jump; ITTAGE predicts, often wrong

    # Each case: a DEPENDENT ALU op on a0 right after the jalr target. If the
    # core mis-speculates down a wrong predicted target, the wrong addi would
    # commit unless the mispredict recovery squashes it. Distinct weights make
    # any mis-squash change the final a0. Each case lands on its own 4KiB page.
.align 12
c0: addi    a0, a0, 1
    addi    a0, a0, 0            # extra dependent op in the speculative shadow
    j       join
.align 12
c1: addi    a0, a0, 2
    addi    a0, a0, 0
    j       join
.align 12
c2: addi    a0, a0, 4
    addi    a0, a0, 0
    j       join
.align 12
c3: addi    a0, a0, 8
    addi    a0, a0, 0
    j       join
.align 12
c4: addi    a0, a0, 16
    addi    a0, a0, 0
    j       join
.align 12
c5: addi    a0, a0, 32
    addi    a0, a0, 0
    j       join
.align 12
c6: addi    a0, a0, 64
    addi    a0, a0, 0
    j       join
.align 12
c7: addi    a0, a0, 128
    addi    a0, a0, 0
    j       join

.align 12
join:
    # advance the 16-bit LFSR (Galois): lsb = s0&1; s0 >>= 1; if lsb s0 ^= 0xB400
    andi    t3, s0, 1
    srli    s0, s0, 1
    beqz    t3, 1f
    li      t4, 0xB400
    xor     s0, s0, t4
1:
    addi    s1, s1, -1
    bnez    s1, loop

    # --- terminate ---------------------------------------------------------
    li      gp, 1
    ecall
1:  j       1b

.align 4
_trap:
    la      t1, tohost
    sw      gp, 0(t1)             # HTIF exit, value 1
1:  j       1b

.section .data
.align 12
jtbl:
    .dword  c0
    .dword  c1
    .dword  c2
    .dword  c3
    .dword  c4
    .dword  c5
    .dword  c6
    .dword  c7
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
