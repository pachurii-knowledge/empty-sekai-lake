# rv64c_smoke.S -- RV64C compressed-instruction smoke test.
#
# Self-checking: halts via `ecall` with a0 == 10 on success, a0 == 11 on the
# first failed check (the ACT/Vtop halt convention). Built -march=rv64gc so the
# assembler emits a dense mix of 16- and 32-bit instructions (creating parcel
# straddles), and uses explicit c.* mnemonics to guarantee each quadrant is
# exercised: arithmetic (c.li/c.mv/c.add/c.addi/c.addiw/c.sub/c.and/c.andi/
# c.slli/c.srli/c.srai/c.lui), control (c.beqz/c.bnez/c.j/c.jalr/c.jr), and
# sp-relative + reg loads/stores (c.sdsp/c.ldsp/c.sw/c.lw).
    .text
    .balign 4
    .globl _start
_start:
    # sp -> a known RAM scratch region (RAM base is 0x8000_0000).
    li   sp, 0x80008000

    # ---- compressed integer arithmetic ----
    c.li   a0, 5
    c.li   a1, 3
    c.add  a0, a1            # 8
    c.addi a0, 4             # 12
    li     t0, 12
    bne    a0, t0, fail

    c.li   a2, 20
    c.sub  a2, a1            # 17
    c.andi a2, 0x1f          # 17
    li     t0, 17
    bne    a2, t0, fail

    c.li   a3, 1
    c.slli a3, 8             # 256
    c.srli a3, 2             # 64
    li     t0, 64
    bne    a3, t0, fail

    c.li   a4, -8
    c.srai a4, 1             # -4
    li     t0, -4
    bne    a4, t0, fail

    # ---- c.addiw (RV64 W-form, sign-extends bit 31) ----
    li     a5, 0x7fffffff
    c.addiw a5, 1            # -> 0xffffffff_80000000 (sign-extended)
    li     t0, 0xffffffff80000000
    bne    a5, t0, fail

    # ---- c.lui / c.mv ----
    c.lui  t1, 0x10          # t1 = 0x10000
    c.mv   t2, t1
    li     t0, 0x10000
    bne    t2, t0, fail

    # ---- compressed branches ----
    c.li   s0, 0
    c.beqz s0, b1            # taken
    j      fail
b1:
    c.li   s0, 7
    c.bnez s0, b2            # taken
    j      fail
b2:
    c.j    b3                # unconditional compressed jump
    j      fail
b3:

    # ---- sp-relative + register loads/stores (c.sdsp/c.ldsp, c.sw/c.lw) ----
    li     t3, 0xdeadbeefcafe0000
    c.sdsp t3, 8(sp)
    c.ldsp t4, 8(sp)
    bne    t3, t4, fail
    li     t5, 0x12345678
    c.swsp t5, 0(sp)         # store low 32 bits
    c.lwsp t6, 0(sp)         # load, sign-extended
    li     t0, 0x12345678
    bne    t6, t0, fail

    # word load/store via a compressed base register (c.sw/c.lw use x8-x15)
    mv     s1, sp
    li     a0, 0x0000abcd
    c.sw   a0, 4(s1)
    c.lw   a1, 4(s1)
    bne    a0, a1, fail

    # ---- call / return (c.jalr / c.jr) for the RAS ----
    la     t0, addfn
    c.li   a0, 30
    c.li   a1, 12
    c.jalr t0                # ra <- pc+2; a0 <- addfn(a0,a1) = 42
    li     t0, 42
    bne    a0, t0, fail

    # success
    li     a0, 10
    ecall

fail:
    li     a0, 11
    ecall

    # int addfn(a0, a1) { return a0 + a1; }  returns via c.jr ra
    .balign 2
addfn:
    c.add  a0, a1
    c.jr   ra
