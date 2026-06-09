    # Phase H1 self-test: the flat-image loader holds a Linux-sized (multi-MB)
    # blob in the sparse physical DRAM, beyond the fixed 512 KiB segment windows.
    #
    # scripts/load_linux_image.py synthetic writes a 16 MiB blob at 0x8000_0000
    # with sentinel words at known offsets, plus mem.image.manifest, into the run
    # directory. main_memory's initialize_flat_images() byte-loads it. This test
    # (running bare/M-mode from USER_TEXT) reads each sentinel back by absolute
    # physical address and checks the magic. The probes span 0..12 MiB and the
    # 512 KiB window edge, so they reach addresses the old fixed windows could not.
    #
    # Build:
    #   python3 scripts/load_linux_image.py synthetic -o <rundir>
    #   make assemble TEST=tests/big_image.S OUTPUT=<rundir>
    #   (cd <rundir> && ../simulation/verilator_obj/Vtop)
    # a0=10 pass, a0=11 fail (the magics must match the script's sentinel table).
    .section .text
    .globl main
main:
    .macro CHECK addr, magic
    li      t0, \addr
    lw      t1, 0(t0)
    li      t2, \magic
    bne     t1, t2, fail
    .endm

    CHECK   0x80000000, 0xC0DE0000    # 0 MiB   (blob base, inside old ktext window)
    CHECK   0x80080000, 0xC0DE0001    # 512 KiB (exactly past the window edge)
    CHECK   0x80100000, 0xC0DE0002    # 1 MiB
    CHECK   0x80400000, 0xC0DE0003    # 4 MiB
    CHECK   0x80800000, 0xC0DE0004    # 8 MiB
    CHECK   0x80C00000, 0xC0DE0005    # 12 MiB

    li      a0, 10                    # all sentinels matched -> pass
    ecall
fail:
    li      a0, 11
    ecall
9:  j       9b
