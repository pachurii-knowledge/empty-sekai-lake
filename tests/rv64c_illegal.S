# rv64c_illegal.S -- RV64C illegal-compressed-instruction mtval directed test.
#
# Executes a RESERVED compressed encoding (0x9C41: C1 reg-reg with {c12,c6:5}=110)
# and verifies the M-mode trap reports mcause = 2 (illegal instruction) and
# mtval = zext16(parcel) = 0x00009C41 -- NOT the expanded 32-bit word. This
# exercises the rvc_parcel plumbing (ctrl_signals_t.rvc_parcel -> ROB ->
# commit_exc_tval), since .instr carries the canonical expansion which cannot be
# used for the compressed mtval. Halts a0 == 10 on success, a0 == 11 on failure.
    .text
    .balign 4
    .globl _start
_start:
    la   t0, handler
    csrw mtvec, t0           # direct mode (base is 4-byte aligned)
    li   s0, 0               # success flag

    .2byte 0x9c41            # reserved compressed encoding -> illegal instruction
    # handler skips past this 2-byte parcel via mepc += 2, resuming here:

    li   t0, 1
    bne  s0, t0, fail
    li   a0, 10
    ecall
fail:
    li   a0, 11
    ecall

    .balign 4
handler:
    csrr t1, mcause
    li   t2, 2               # EXC_ILLEGAL_INSTR
    bne  t1, t2, hfail
    csrr t1, mtval
    li   t2, 0x9c41          # zero-extended 16-bit parcel
    bne  t1, t2, hfail
    li   s0, 1               # both checks passed
    csrr t1, mepc            # skip the 2-byte illegal parcel
    addi t1, t1, 2
    csrw mepc, t1
    mret
hfail:
    li   a0, 11
    ecall
