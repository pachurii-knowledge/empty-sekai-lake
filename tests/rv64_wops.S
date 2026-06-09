    # R2a: RV64 W-form ops. They operate on the low 32 bits of the operands and
    # sign-extend bit 31 of the 32-bit result to all 64 bits.
    # a0=10 pass; a0=11 fail, a1 = which check (61..68).
    # Build: make assemble TEST=tests/rv64_wops.S RV64=1
    .section .text
    .globl main
main:
    li      t0, 0x100000005         # addw ignores upper 32 bits
    li      t1, 0x200000007
    addw    t2, t0, t1              # low: 5+7 = 12
    li      t3, 12
    li      a1, 61
    bne     t2, t3, fail

    li      t0, 0x7fffffff          # addw overflow into bit31 -> sign-extend
    li      t1, 1
    addw    t2, t0, t1              # 0x8000_0000 -> 0xffffffff_80000000
    li      t3, 0xffffffff80000000
    li      a1, 62
    bne     t2, t3, fail

    li      t0, 0                   # subw: 0-1 = -1
    li      t1, 1
    subw    t2, t0, t1
    li      t3, -1
    li      a1, 63
    bne     t2, t3, fail

    li      t0, 1                   # slliw: 1<<31 -> sign-extend
    slliw   t2, t0, 31
    li      t3, 0xffffffff80000000
    li      a1, 64
    bne     t2, t3, fail

    li      t0, 0xffffffff80000000  # srliw: logical shift on low 32
    srliw   t2, t0, 4               # 0x0800_0000
    li      t3, 0x08000000
    li      a1, 65
    bne     t2, t3, fail

    li      t0, 0x80000000          # sraiw: arithmetic shift on low 32
    sraiw   t2, t0, 4               # 0xf800_0000 -> sign-extend
    li      t3, 0xfffffffff8000000
    li      a1, 66
    bne     t2, t3, fail

    li      t0, 0x7ffffffe          # addiw: low32 + imm, sign-extend
    addiw   t2, t0, 4               # 0x8000_0002 -> sign-extend
    li      t3, 0xffffffff80000002
    li      a1, 67
    bne     t2, t3, fail

    li      t0, 1                   # sllw with register shift amount
    li      t1, 31
    sllw    t2, t0, t1              # 0x8000_0000 -> sign-extend
    li      t3, 0xffffffff80000000
    li      a1, 68
    bne     t2, t3, fail

    li      a0, 10
    ecall
fail:
    li      a0, 11
    ecall
9:  j       9b
