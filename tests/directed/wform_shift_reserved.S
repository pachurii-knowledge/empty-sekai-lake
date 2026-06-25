# Directed lockstep torture test: wform_shift_reserved
#
# Corner: RV64I W-form immediate shifts SLLIW/SRLIW/SRAIW (opcode 0x1B,
# OP-IMM-32) use a 5-bit shamt in instr[24:20] and REQUIRE instr[25]=0
# (and instr[31:26] fixed). Setting instr[25]=1 (i.e. shamt >= 32) is a
# RESERVED encoding that MUST raise an illegal-instruction trap (mcause=2).
# We also exercise OP-IMM (opcode 0x13) SLLI/SRLI/SRAI with instr[26]=1,
# which is reserved on RV64 (the 6-bit shamt uses instr[25:20]; instr[26]
# is a funct-field bit that must be 0).
#
# Suspected DUT bug (riscv_decode.sv:845-851): OP-IMM-32 selects the W-form
# shift purely on instr[14:12]/instr[30], never checking instr[31:26]/[25],
# and the ALU (ooo_alu_pipe.sv) masks the shift amount to b[4:0], so the DUT
# silently retires a 5-bit shift + GPR write at rd where the golden models
# trap. That divergence is directly observable: golden -> trap (no rd write,
# mcause=2 recorded by the handler); DUT -> a value written to rd.
#
# We make every reserved instruction's effect observable in an integer GPR:
#   - the trap handler records mcause into a dedicated GPR (s-reg) per site,
#     then advances mepc past the faulting word and mret-resumes.
#   - if a sim does NOT trap (the buggy DUT), the reserved instruction writes
#     its destination GPR directly, which is traced.
# Either way the integer-GPR trace diverges between a trapping golden and a
# silently-executing DUT.
#
# M-mode, deterministic, single ecall terminator.

.section .text
.globl _start
.option norvc
_start:
    la      t0, _trap
    csrw    mtvec, t0
    la      sp, _stack_top

    # Dedicated recorder GPRs (cleared so a non-trapping sim leaves them 0):
    #   s2..s9 hold mcause per reserved-encoding site (0 if no trap fired).
    #   s10 counts how many traps fired total.
    li      s2, 0
    li      s3, 0
    li      s4, 0
    li      s5, 0
    li      s6, 0
    li      s7, 0
    li      s8, 0
    li      s9, 0
    li      s10, 0
    li      s11, 0                 # site selector consumed by the handler

    # Seed value to be shifted; its low 32 bits are 0x12345678.
    li      a0, 0x12345678

    # --- Site 1: SLLIW x1, x1, shamt with instr[25]=1 (reserved) -----------
    # base 0x0000101b | rs1(x1=1)<<15 | rd(x1=1)<<7 | shamt(1)<<20 | bit25
    #   = 0x0210909b
    mv      x1, a0
    li      s11, 1
    .word   0x0210909b             # SLLIW x1, x1, (32|1)  -- RESERVED
    mv      a1, x1                 # capture DUT result (if it didn't trap)

    # --- Site 2: SRLIW x1, x1, shamt with instr[25]=1 (reserved) -----------
    # base 0x0000501b | rs1<<15 | rd<<7 | shamt(1)<<20 | bit25 = 0x0210d09b
    mv      x1, a0
    li      s11, 2
    .word   0x0210d09b             # SRLIW x1, x1, (32|1)  -- RESERVED
    mv      a2, x1

    # --- Site 3: SRAIW x1, x1, shamt with instr[25]=1 (reserved) -----------
    # base 0x4000501b | rs1<<15 | rd<<7 | shamt(1)<<20 | bit25 = 0x4210d09b
    mv      x1, a0
    li      s11, 3
    .word   0x4210d09b             # SRAIW x1, x1, (32|1)  -- RESERVED
    mv      a3, x1

    # --- Site 4: SLLIW x1, x1, shamt with bit25=1 and a high shamt bit -----
    # shamt field = 0x1f (=31), plus bit25 -> effective 63; base|...|0x02000000
    # base 0x0000101b | rs1<<15 | rd<<7 | shamt(0x1f)<<20 | bit25 = 0x03f0909b
    mv      x1, a0
    li      s11, 4
    .word   0x03f0909b             # SLLIW x1, x1, (32|31) -- RESERVED
    mv      a4, x1

    # --- Site 5: OP-IMM SLLI with instr[26]=1 (reserved on RV64) -----------
    # SLLI base 0x00001013 | rs1<<15 | rd<<7 | shamt<<20; set instr[26]=1 ->
    # OR 0x04000000. shamt(1): 0x00101093 | 0x04000000 = 0x04101093
    mv      x1, a0
    li      s11, 5
    .word   0x04101093             # SLLI x1, x1, 1 with funct bit26=1 -- RESERVED
    mv      a5, x1

    # --- Site 6: OP-IMM SRLI with instr[26]=1 (reserved on RV64) -----------
    # SRLI base 0x00005013 | rs1<<15 | rd<<7 | shamt<<20; set instr[26]=1.
    # shamt(1): 0x00105093 | 0x04000000 = 0x04105093
    mv      x1, a0
    li      s11, 6
    .word   0x04105093             # SRLI x1, x1, 1 with funct bit26=1 -- RESERVED
    mv      a6, x1

    # --- Site 7: OP-IMM SRAI with instr[26]=1 (reserved on RV64) -----------
    # SRAI base 0x40005013 | rs1<<15 | rd<<7 | shamt<<20; set instr[26]=1.
    # shamt(1): 0x40105093 | 0x04000000 = 0x44105093
    mv      x1, a0
    li      s11, 7
    .word   0x44105093             # SRAI x1, x1, 1 with funct bit26=1 -- RESERVED
    mv      a7, x1

    # --- terminate ---------------------------------------------------------
    li      gp, 1
    ecall
1:  j       1b

# Trap handler: ECALL-from-M (mcause==11) terminates via HTIF; any other
# cause is the reserved-encoding illegal trap -- record mcause into the
# per-site recorder GPR, advance mepc past the 4-byte faulting word, mret.
.align 4
_trap:
    csrr    t2, mcause
    li      t3, 11                 # ECALL from M-mode
    beq     t2, t3, _terminate

    # Record this cause into the slot selected by s11.
    addi    s10, s10, 1            # bump trap count
    li      t4, 1
    beq     s11, t4, _rec1
    li      t4, 2
    beq     s11, t4, _rec2
    li      t4, 3
    beq     s11, t4, _rec3
    li      t4, 4
    beq     s11, t4, _rec4
    li      t4, 5
    beq     s11, t4, _rec5
    li      t4, 6
    beq     s11, t4, _rec6
    j       _rec7                  # s11 == 7
_rec1:  mv  s2, t2
        j   _advance
_rec2:  mv  s3, t2
        j   _advance
_rec3:  mv  s4, t2
        j   _advance
_rec4:  mv  s5, t2
        j   _advance
_rec5:  mv  s6, t2
        j   _advance
_rec6:  mv  s7, t2
        j   _advance
_rec7:  mv  s8, t2
        j   _advance
_advance:
    csrr    t5, mepc
    addi    t5, t5, 4              # skip the 4-byte reserved word
    csrw    mepc, t5
    mret

_terminate:
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
