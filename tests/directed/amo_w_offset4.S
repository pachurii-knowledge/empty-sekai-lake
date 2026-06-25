# Directed lockstep test: amo_w_offset4
# Corner: RV64 AMO.W (32-bit atomics) targeting the HIGH 4 bytes of an
# 8-byte doubleword (address a0+4), and the LOW 4 bytes (address a0).
# Verifies:
#   - the AMO returns the sign-extended OLD 32-bit value in rd,
#   - the addressed 4-byte half is updated correctly,
#   - the OTHER 4-byte half is left UNTOUCHED.
# All effects are landed into integer GPRs (loaded back) before the ecall so the
# lockstep oracle (integer-GPR-write trace) can see them.
#
# NOTE: RISC-V AMO instructions take a bare register address (rs1) with NO
# immediate offset, so the "offset 4" address is precomputed (a0+4) into a5.
#
# Memory layout per 8-byte cell (little-endian):
#   bytes [0..3] = LOW  word, at address a0
#   bytes [4..7] = HIGH word, at address a0+4
# Seed value 0xDEADBEEF_CAFEBABE => high word = 0xDEADBEEF, low word = 0xCAFEBABE.
.section .text
.globl _start
.option norvc
_start:
    la      t0, _trap
    csrw    mtvec, t0
    la      sp, _stack_top

    # Seed constant 0xDEADBEEF_CAFEBABE into s11 for re-seeding each cell.
    li      s11, 0xDEADBEEF
    slli    s11, s11, 32
    li      t2, 0xCAFEBABE
    or      s11, s11, t2          # s11 = 0xDEADBEEF_CAFEBABE

    la      a0, _cell             # a0 = LOW-word address (offset 0)
    addi    a5, a0, 4             # a5 = HIGH-word address (offset 4)

    # ---------------------------------------------------------------------
    # 1) amoadd.w on HIGH word (a5). old=0xDEADBEEF, rs2=0x11.
    #    new high = 0xDEADBF00; low untouched 0xCAFEBABE.
    # ---------------------------------------------------------------------
    sd      s11, 0(a0)            # re-seed cell
    li      t1, 0x00000011        # addend
    amoadd.w t0, t1, (a5)         # t0 = sign-ext old high 0xDEADBEEF
    lwu     s0, 4(a0)             # new high word = 0xDEADBF00
    lwu     s1, 0(a0)             # low untouched 0xCAFEBABE
    mv      s2, t0                # old-value-1 (stable)

    # ---------------------------------------------------------------------
    # 2) amoadd.w on LOW word (a0). old=0xCAFEBABE, rs2=0x22.
    #    new low = 0xCAFEBAE0; high untouched 0xDEADBEEF.
    # ---------------------------------------------------------------------
    sd      s11, 0(a0)            # re-seed
    li      t1, 0x00000022
    amoadd.w t3, t1, (a0)         # t3 = sign-ext old low 0xCAFEBABE
    lwu     s3, 0(a0)             # new low word = 0xCAFEBAE0
    lwu     s4, 4(a0)             # high untouched 0xDEADBEEF
    mv      s5, t3                # old-value-2 (stable)

    # ---------------------------------------------------------------------
    # 3) amoswap.w on HIGH word (a5). old=0xDEADBEEF, rs2=0x12345678.
    #    new high = 0x12345678; low untouched.
    # ---------------------------------------------------------------------
    sd      s11, 0(a0)            # re-seed
    li      t1, 0x12345678
    amoswap.w t4, t1, (a5)        # t4 = sign-ext old high 0xDEADBEEF
    lwu     s6, 4(a0)             # new high = 0x12345678
    lwu     s7, 0(a0)             # low untouched 0xCAFEBABE
    mv      s8, t4                # old-value-3 (stable)

    # ---------------------------------------------------------------------
    # 4) amomax.w (signed) on HIGH word (a5). old=0xDEADBEEF (negative),
    #    rs2=0x00000001 (positive) -> max = 1 written to mem.
    # ---------------------------------------------------------------------
    sd      s11, 0(a0)            # re-seed
    li      t1, 0x00000001
    amomax.w t5, t1, (a5)         # t5 = sign-ext old 0xDEADBEEF (negative)
    lwu     s9, 4(a0)             # new high = 0x00000001
    lwu     s10, 0(a0)            # low untouched 0xCAFEBABE
    mv      a1, t5                # old-value-4 (stable)

    # ---------------------------------------------------------------------
    # 5) amominu.w (unsigned) on LOW word (a0). old=0xCAFEBABE (large unsigned),
    #    rs2=0x00000005 -> minu = 5 written to mem.
    # ---------------------------------------------------------------------
    sd      s11, 0(a0)            # re-seed
    li      t1, 0x00000005
    amominu.w a2, t1, (a0)        # a2 = sign-ext old low 0xCAFEBABE
    lwu     a3, 0(a0)             # new low = 0x00000005
    lwu     a4, 4(a0)             # high untouched 0xDEADBEEF

    # ---------------------------------------------------------------------
    # 6) amoxor.w on HIGH word (a5). old=0xDEADBEEF ^ 0xFFFFFFFF = 0x21524110.
    # ---------------------------------------------------------------------
    sd      s11, 0(a0)            # re-seed
    li      t1, 0xFFFFFFFF
    amoxor.w a6, t1, (a5)         # a6 = sign-ext old 0xDEADBEEF
    lwu     a7, 4(a0)             # new high = 0x21524110
    lwu     t0, 0(a0)             # low untouched 0xCAFEBABE
    mv      t3, a7                # keep new-high-6 (stable-ish)

    # ---------------------------------------------------------------------
    # 7) amoxor.w on LOW word (a0). old=0xCAFEBABE ^ 0x0F0F0F0F = 0xC5C1B5B1.
    # ---------------------------------------------------------------------
    sd      s11, 0(a0)            # re-seed
    li      t1, 0x0F0F0F0F
    amoxor.w t4, t1, (a0)         # t4 = sign-ext old low 0xCAFEBABE
    lwu     t5, 0(a0)             # new low = 0xC5C1B5B1
    lwu     t6, 4(a0)             # high untouched 0xDEADBEEF

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
.align 8
_cell:    .dword 0
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
