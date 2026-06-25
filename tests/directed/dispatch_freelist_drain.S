# Directed lockstep torture test: dispatch_freelist_drain
#
# Corner (spec freelist-exhaustion-partial-dispatch, rank3):
#   Park a long-latency op (DIV) writing rd at the ROB head, then issue a long
#   run of INDEPENDENT dest-writing ALU ops that ALL rename around it (every one
#   draws a fresh physreg) before any consumer of the DIV result. The active
#   list cannot commit while the DIV occupies the head, so no physregs are
#   freed; the free-list (32 free physregs at reset) is drained toward <4. When
#   a 4-wide dispatch group is presented while only 1..3 free physregs remain,
#   ooo_dispatch_control.sv:48-51 must split the group mid-way (dest_seen >=
#   free_list_available) and re-present the suppressed tail next cycle while the
#   DIV still holds the head (no frees arriving). An off-by-one (>= vs >, or
#   dest_seen counting the current lane) either over-allocates a garbage physreg
#   (free_list.sv:65 reads entries past the valid window; entries[32..63] hold
#   the 0..31 init region) or deadlocks dispatch.
#
# Observability: the DIV result and a spread of the renamed-around registers are
# read back into integer GPRs before the terminating ecall, so any wrong physreg
# mapping / over-allocation surfaces as a wrong GPR write in the trace, and a
# dispatch deadlock surfaces as a DUT hang.
#
# M-mode, deterministic, no mid-stream trap. Compared per-instruction GPR write
# against Spike AND Sail.
.section .text
.globl _start
.option norvc
_start:
    la      t0, _trap
    csrw    mtvec, t0
    la      sp, _stack_top

    # --- seed operands ------------------------------------------------------
    li      x6, 0x00000000abcdef01      # dividend
    li      x7, 0x0000000000000007      # divisor
    li      x28, 0x1111111100000000     # base pattern for the renamed-around ops
    li      x29, 0x0000000000000003     # small step

    # --- long-latency op parked at the ROB head -----------------------------
    # x5 is the DIV destination; NOTHING below consumes x5 until the final load
    # area, so all the following dest-writers rename around it while the DIV
    # holds the head and blocks commit -> the free-list drains.
    div     x5, x6, x7

    # --- ~30 independent dest-writing ALU ops cycling the full arch-reg set --
    # Each writes a distinct arch reg (x8..x31, then reuses to force renaming),
    # all independent of x5 and of each other's hazards beyond x28/x29 reads,
    # so each retires a fresh physreg and none can commit (DIV at head). This is
    # what pulls free_count down through {3,2,1} during a 4-wide dispatch group.
    addi    x8,  x28, 0x101
    addi    x9,  x28, 0x102
    addi    x10, x28, 0x103
    addi    x11, x28, 0x104
    addi    x12, x28, 0x105
    addi    x13, x28, 0x106
    addi    x14, x28, 0x107
    addi    x15, x28, 0x108
    addi    x16, x28, 0x109
    addi    x17, x28, 0x10a
    addi    x18, x28, 0x10b
    addi    x19, x28, 0x10c
    addi    x20, x28, 0x10d
    addi    x21, x28, 0x10e
    addi    x22, x28, 0x10f
    addi    x23, x28, 0x110
    addi    x24, x28, 0x111
    addi    x25, x28, 0x112
    addi    x26, x28, 0x113
    addi    x27, x28, 0x114
    addi    x30, x28, 0x115
    addi    x31, x28, 0x116
    # second wave: rewrite the same arch regs (forces more fresh physregs,
    # pushing free_count into the {1,2,3} split window) -- still no consumer of
    # x5, still no commit possible (DIV at head).
    xor     x8,  x29, x28
    xor     x9,  x8,  x28
    xor     x10, x9,  x28
    xor     x11, x10, x28
    xor     x12, x11, x28
    xor     x13, x12, x28
    xor     x14, x13, x28
    add     x15, x14, x29
    add     x16, x15, x29
    add     x17, x16, x29

    # --- now consume the DIV result; this is the first thing that lets the
    #     head retire, so it can only proceed once the drained free-list has
    #     correctly re-presented the suppressed dispatch tail.
    add     x6, x5, x29                  # x6 = quotient + 3 (depends on DIV)
    add     x7, x5, x8                   # x7 = quotient + x8 (second consumer)

    # --- spill the renamed-around chain to memory and reload, so the full set
    #     of physreg mappings is forced live and observable in the GPR trace.
    la      t1, _scratch
    sd      x5,  0(t1)
    sd      x17, 8(t1)
    sd      x14, 16(t1)
    sd      x6,  24(t1)
    ld      x5,  0(t1)                   # quotient back into x5
    ld      x17, 8(t1)
    ld      x14, 16(t1)
    ld      x6,  24(t1)

    # --- terminate ----------------------------------------------------------
    li      gp, 1
    ecall
1:  j       1b

.align 4
_trap:
    la      t1, tohost
    sw      gp, 0(t1)                    # HTIF exit, value 1
1:  j       1b

.section .data
.align 8
_scratch: .rept 8
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
