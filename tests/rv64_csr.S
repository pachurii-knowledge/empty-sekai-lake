    # R4: MXLEN=64 CSR file. misa MXL, 64-bit CSR round trips, mtvec/mepc
    # masking, satp Sv39 WARL, mstatus SXL/UXL, mcause via a real M-mode trap,
    # and the removal of the RV32-only high-half CSRs (cycleh -> illegal).
    # a0=10 pass; a0=11 fail, a1 = which check.
    .section .text
    .globl main
main:
    # 1: misa = MXL=2 | ADFIMSU
    csrr    t0, misa
    li      t1, 0x8000000000141129
    li      a1, 1
    bne     t0, t1, fail

    # 2: mscratch full 64-bit round trip
    li      t0, 0xdeadbeefcafebabe
    csrw    mscratch, t0
    csrr    t1, mscratch
    li      a1, 2
    bne     t1, t0, fail

    # 3: mtvec write masks bit 1 only (direct mode, 64-bit base)
    li      t0, 0x123456789abcdef3
    csrw    mtvec, t0
    csrr    t1, mtvec
    li      t2, 0x123456789abcdef1
    li      a1, 3
    bne     t1, t2, fail

    # 4: mepc clears [1:0]
    li      t0, 0x8000000000000003
    csrw    mepc, t0
    csrr    t1, mepc
    li      t2, 0x8000000000000000
    li      a1, 4
    bne     t1, t2, fail

    # 5: satp accepts Bare(0) and Sv39(8); other MODEs leave it unchanged
    li      t0, 0x8000000000012345   # MODE=8 (Sv39), PPN=0x12345
    csrw    satp, t0
    csrr    t1, satp
    li      a1, 5
    bne     t1, t0, fail
    li      t2, 0x5000000000000999   # MODE=5: unsupported -> write ignored
    csrw    satp, t2
    csrr    t1, satp
    li      a1, 6
    bne     t1, t0, fail
    csrw    satp, x0                 # back to Bare
    csrr    t1, satp
    li      a1, 7
    bne     t1, x0, fail

    # 8: mstatus SXL/UXL read-only 2 (bits [35:34] / [33:32])
    csrr    t0, mstatus
    srli    t1, t0, 32
    andi    t1, t1, 0xf
    li      t2, 0xa                  # SXL=10, UXL=10
    li      a1, 8
    bne     t1, t2, fail

    # 9-11: real M-mode trap: ecall -> mcause=11 (bit 63 clear), mepc=ecall pc
    la      t0, handler
    csrw    mtvec, t0
    li      s2, 0                    # handler flag
    li      a0, 0                    # keep clear of the a0=10/11 sim halt
ecall_site:
    ecall
after_ecall:
    li      a1, 9
    li      t2, 11                   # EXC_ECALL_M
    bne     s2, t2, fail             # handler saw mcause=11, interrupt bit clear
    li      a1, 10
    la      t2, ecall_site
    bne     s3, t2, fail             # mepc == ecall pc

    # 12: cycleh (0xC80) does not exist on RV64 -> illegal instruction
    li      s2, 0
    csrr    t0, 0xc80                # cycleh
illegal_site:
    li      a1, 11
    li      t2, 2                    # EXC_ILLEGAL_INSTR
    bne     s2, t2, fail

    li      a0, 10
    ecall
fail:
    li      a0, 11
    ecall
9:  j       9b

    # M-mode trap handler: record mcause (s2) and mepc (s3), skip the
    # trapping instruction, return.
    .align 2
handler:
    csrr    s2, mcause
    csrr    s3, mepc
    addi    t4, s3, 4
    csrw    mepc, t4
    mret
