    # CLINT memory-mapped load test (M-mode, paging off).
    #
    # Exercises MMIO reads of the CLINT registers, which must return register
    # values (not DRAM). CLINT base = 0x02000000:
    #   msip      @ 0x02000000
    #   mtimecmp  @ 0x02004000 (lo) / 0x02004004 (hi)
    #   mtime     @ 0x0200BFF8 (lo) / 0x0200BFFC (hi)
    #
    # Checks:
    #   write mtimecmp.lo, read it back via MMIO -> matches
    #   read mtime.lo twice -> non-decreasing (free running)
    #   write msip=1, read it back via MMIO -> 1
    #
    # Exit a0=10 pass; a0=11 fail with a1 = 41 mtimecmp / 42 mtime / 43 msip.
    .section .text
    .globl main
main:
    li      s0, 0x02000000          # msip
    li      s1, 0x02004000          # mtimecmp lo
    li      s2, 0x0200BFF8          # mtime lo

    # ---- mtimecmp write/read ----
    li      t0, 0x0BADF00D
    sw      t0, 0(s1)               # snooped store into mtimecmp
    lw      t1, 0(s1)               # MMIO read back
    li      a1, 41
    bne     t0, t1, fail

    # ---- mtime non-decreasing ----
    lw      t2, 0(s2)               # mtime.lo first read
    nop
    nop
    lw      t3, 0(s2)               # mtime.lo second read
    li      a1, 42
    bltu    t3, t2, fail            # t3 < t2 -> time went backwards (bug)

    # ---- msip write/read ----
    li      t0, 1
    sw      t0, 0(s0)               # set msip
    lw      t1, 0(s0)               # MMIO read back
    li      a1, 43
    bne     t0, t1, fail

    li      a0, 10
    ecall
fail:
    li      a0, 11
    ecall
