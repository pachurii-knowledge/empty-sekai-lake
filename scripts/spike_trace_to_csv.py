#!/usr/bin/env python3
"""Convert a Spike (riscv-isa-sim) commit log into a riscv-dv trace CSV.

The Spike golden model under ``references/riscv-isa-sim/`` is run as

    spike --isa=<rv32imafd_zicsr_zifencei|rv64imafd_zicsr_zifencei> \
          -l --log-commits --log=<log> <elf>

which, per committed instruction, prints two lines -- the fetch/disasm line and
the architectural-effect line carrying the GPR/CSR/mem writeback::

    core   0: 0x80000008 (0x002081b3) add     gp, ra, sp
    core   0: 3 0x80000008 (0x002081b3) x3  0x0000000c

The two are told apart by the privilege digit (``3``=M/``1``=S/``0``=U) that the
effect line inserts right after ``core N:`` (the fetch line goes straight to
``0x<addr>``).

riscv-dv ships its own ``spike_log_to_trace_csv.py``, but it uses a *different*
convention from this repo's DUT/Sail converters: it drops every instruction that
writes no GPR (stores, branches, fences) and keeps the trailing ``ecall``.  That
breaks the row-for-row total-count length gate in ``run_riscvdv.sh``.  This
parser instead matches ``niigo_log_to_trace_csv.py``/``sail_trace_to_csv.py``
field-for-field -- one row per committed instruction, ``gpr`` populated only for
*integer* writebacks (the niigo retire trace never carries f-register results --
``has_dest`` is the integer-regfile write), values lowercased and zero-padded to
XLEN/4 hex digits, and capture stopped at (but not including) the first ``ecall``
-- so the three CSVs diff cleanly under ``instr_trace_compare.py``.

Spike runs a reset trampoline at 0x1000..0x1010 before jumping to the ELF entry;
pass ``--start-pc <entry>`` to drop it and align with the DUT/Sail window.
"""

import argparse
import csv
import re
import sys

# x<n> -> ABI name, matching scripts/lib.py gpr_to_abi() / the sibling converters.
ABI = ["zero", "ra", "sp", "gp", "tp", "t0", "t1", "t2",
       "s0", "s1", "a0", "a1", "a2", "a3", "a4", "a5",
       "a6", "a7", "s2", "s3", "s4", "s5", "s6", "s7",
       "s8", "s9", "s10", "s11", "t3", "t4", "t5", "t6"]

CSV_FIELDS = ["pc", "instr", "gpr", "csr", "binary", "mode",
              "instr_str", "operand", "pad"]

# Effect line (has the privilege digit before the address):
#   core   0: 3 0x80000008 (0x002081b3) x3  0x0000000c
# Tried FIRST so an effect line is never misread as a fetch line.
EFFECT_RE = re.compile(
    r"^core\s+\d+:\s+(?P<pri>\d)\s+0x(?P<addr>[0-9A-Fa-f]+)\s+"
    r"\(0x(?P<bin>[0-9A-Fa-f]+)\)\s*(?P<rest>.*)$")
# Fetch/disasm line (goes straight to the address, no privilege digit):
#   core   0: 0x80000008 (0x002081b3) add     gp, ra, sp
INSTR_RE = re.compile(
    r"^core\s+\d+:\s+0x(?P<addr>[0-9A-Fa-f]+)\s+"
    r"\(0x(?P<bin>[0-9A-Fa-f]+)\)\s+(?P<disasm>.+?)\s*$")
# Integer GPR writeback inside an effect line's tail (ignores f-regs, mem, csr).
XREG_RE = re.compile(r"(?:^|\s)x(?P<reg>\d+)\s+0x(?P<val>[0-9A-Fa-f]+)")

ECALL = 0x00000073


def fmt_val(hex_str, xlen):
    """Lowercase, zero-padded XLEN-wide hex (no 0x), for stable string compare."""
    return format(int(hex_str, 16) & ((1 << xlen) - 1), "0{}x".format(xlen // 4))


def convert(log_path, csv_path, xlen, stop_at_ecall=True, start_pc=None):
    n = 0
    started = start_pc is None
    start_pc = None if start_pc is None else (int(start_pc, 16) & ((1 << xlen) - 1))
    stop_reason = "eof"
    last_pc = None
    with open(log_path) as fin, open(csv_path, "w", newline="") as fout:
        writer = csv.DictWriter(fout, fieldnames=CSV_FIELDS)
        writer.writeheader()
        pending = None  # row dict awaiting its (optional) effect line

        def flush(row):
            nonlocal n
            if row is not None:
                writer.writerow(row)
                n += 1

        for line in fin:
            # Effect line: fill the GPR of the instruction we just started.
            me = EFFECT_RE.match(line)
            if me:
                if pending is not None:
                    mx = XREG_RE.search(me.group("rest"))
                    if mx and int(mx.group("reg")) != 0:
                        pending["gpr"] = "{}:{}".format(
                            ABI[int(mx.group("reg"))],
                            fmt_val(mx.group("val"), xlen))
                    if pending["mode"] == "":
                        pending["mode"] = me.group("pri")
                continue
            # Fetch line: a new committed instruction.
            mi = INSTR_RE.match(line)
            if mi:
                flush(pending)
                pending = None
                addr = mi.group("addr").lower()
                binv = mi.group("bin").lower()
                if not started:
                    if (int(addr, 16) & ((1 << xlen) - 1)) != start_pc:
                        continue
                    started = True
                last_pc = addr
                if stop_at_ecall and int(binv, 16) == ECALL:
                    return n, "ecall", last_pc  # stop; do not emit the ecall
                pending = {
                    "pc": addr,
                    "instr": binv,
                    "gpr": "",
                    "csr": "",
                    "binary": binv,
                    "mode": "",
                    "instr_str": mi.group("disasm").strip(),
                    "operand": "",
                    "pad": "",
                }
        flush(pending)
    return n, stop_reason, last_pc


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--log", required=True, help="Spike -l --log-commits log")
    ap.add_argument("--csv", required=True, help="output riscv-dv trace CSV")
    ap.add_argument("--xlen", type=int, default=32, choices=[32, 64])
    ap.add_argument("--no-stop-at-ecall", dest="stop_at_ecall",
                    action="store_false", help="do not truncate at the first ecall")
    ap.add_argument("--start-pc", default=None,
                    help="begin capture at this hex PC (the program entry), "
                         "skipping Spike's reset trampoline at 0x1000..0x1010")
    args = ap.parse_args()
    n, stop_reason, last_pc = convert(args.log, args.csv, args.xlen,
                                      args.stop_at_ecall, args.start_pc)
    print("spike_trace_to_csv: wrote {} instructions to {} "
          "(stop_reason={} last_pc={})".format(n, args.csv, stop_reason, last_pc),
          file=sys.stderr)
    if n == 0:
        print("warning: no instruction lines found -- did you run spike with "
              "-l --log-commits?", file=sys.stderr)
        sys.exit(2)


if __name__ == "__main__":
    main()
