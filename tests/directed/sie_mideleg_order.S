# Directed lockstep test: sie-write-masked-by-mideleg (rank36)
#
# Corner: writes/reads of the SIE CSR vs mideleg ordering.
#   DUT (priv_csr_file.sv:537 write, :283 read) masks BOTH the sie write AND
#   the sie read by mideleg, so an enable bit can only be set/read while its
#   interrupt is delegated. The architectural model (Spike/Sail) treats sie as
#   the S-visible subset of mie masked by S_INT_MASK (SSI|STI|SEI = 0x222),
#   INDEPENDENT of mideleg: sie.STIE is writable/readable whether or not STI is
#   delegated.
#
# We capture sie reads into a1..a4 (and a5/a6) around mideleg changes; any diff
# between DUT and golden pins the masking semantics. All effects land in integer
# GPRs before the terminating ecall (sie read via csrr -> GPR).
#
# Bit positions: SSI=1, STI=5, SEI=9. S_INT_MASK = 0x222.
.section .text
.globl _start
.option norvc
_start:
    la      t0, _trap
    csrw    mtvec, t0
    la      sp, _stack_top

    # all-ones write pattern (covers all S interrupt-enable bits and more)
    li      t1, -1                  # 0xFFFF...FFFF

    # --- (a) delegate STIE|SSIE|SEIE: mideleg = 0x222 -------------------------
    li      t2, 0x222
    csrw    mideleg, t2

    # --- (b) write all-ones to sie while fully delegated; read back ----------
    csrrw   a0, sie, t1             # a0 = old sie
    csrr    a1, sie                 # a1 = sie after write (delegated)

    # --- (c) un-delegate everything: mideleg = 0 -----------------------------
    csrw    mideleg, x0

    # --- (d) read sie with mideleg=0 -----------------------------------------
    # Architecturally the bits set in mie via (b) are still S-visible: sie
    # reads mie & S_INT_MASK = 0x222. DUT masks by mideleg=0 -> reads 0.
    csrr    a2, sie                 # a2 = sie after un-delegate (read-side test)

    # write all-ones to sie with mideleg=0, then read: are un-delegated bits
    # writable? Architecturally yes (sie = mie & 0x222). DUT: write masked by
    # mideleg=0 -> no change, read masked -> 0.
    csrrw   a3, sie, t1             # a3 = sie before this write
    csrr    a4, sie                 # a4 = sie after write (mideleg=0)

    # --- (e) restore mideleg = 0x222 and read sie again ----------------------
    # Architecturally the previously-set mie bits become visible again -> 0x222.
    # DUT: whatever mie bits survived the masked writes, re-masked by 0x222.
    li      t2, 0x222
    csrw    mideleg, t2
    csrr    a5, sie                 # a5 = sie after re-delegate

    # cross-check: read mie directly (M-mode) to expose the underlying mie that
    # sie is derived from. This pins WHERE the divergence lives (mie vs the
    # sie-read mask).
    csrr    a6, mie                 # a6 = full mie

    # --- terminate -----------------------------------------------------------
    li      gp, 1
    ecall
1:  j       1b

.align 4
_trap:
    la      t1, tohost
    sw      gp, 0(t1)               # HTIF exit, value 1
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
