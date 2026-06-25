#!/usr/bin/env python3
"""Convert a (new, config-based) Sail RISC-V trace into a riscv-dv trace CSV.

The Sail model shipped under ``references/sail-riscv-Linux-x86_64/`` is the
modern config-based emulator (``--config``, ``--trace-output``), whose trace
format differs from the legacy one that riscv-dv's bundled
``scripts/sail_log_to_trace_csv.py`` expects (that parser keys off a fixed boot
marker that niigo's riscv-dv programs never emit).  Run it as:

    sail_riscv_sim --config <niigo sail.json> --trace-instr --trace-gpr \
                   --trace-output <trace> --inst-limit <N> <elf>

which produces:

    [<step>] [<priv>]: 0x<ADDR> (0x<BIN>) <disasm>            <sym>+<off>
    x<reg> <- 0x<val>

A ``x.. <- 0x..`` write line belongs to the most recent instruction line; any
write lines before the first instruction (HTIF boot setup of a0/a1) are
ignored.  Output matches ``niigo_log_to_trace_csv.py`` field-for-field so the
two CSVs diff cleanly under ``instr_trace_compare.py``.  Capture stops at the
first ``ecall`` so the window matches the DUT side.
"""

import argparse
import csv
import re
import sys

ABI = ["zero", "ra", "sp", "gp", "tp", "t0", "t1", "t2",
       "s0", "s1", "a0", "a1", "a2", "a3", "a4", "a5",
       "a6", "a7", "s2", "s3", "s4", "s5", "s6", "s7",
       "s8", "s9", "s10", "s11", "t3", "t4", "t5", "t6"]

CSV_FIELDS = ["pc", "instr", "gpr", "csr", "binary", "mode",
              "instr_str", "operand", "pad"]

# [12] [M]: 0x80000030 (0xE74D0D13) addi x26, x26, -0x18c    trap_vec_init+4
INSTR_RE = re.compile(
    r"^\[\d+\]\s+\[(?P<pri>.)\]:\s+0x(?P<addr>[0-9A-Fa-f]+)\s+"
    r"\(0x(?P<bin>[0-9A-Fa-f]+)\)\s+(?P<instr>.+?)\s*$")
# Register *write* only ('<-'); reads ('->'), if traced, are ignored.
RD_RE = re.compile(r"^\s*x(?P<reg>\d+)\s+<-\s+0x(?P<val>[0-9A-Fa-f]+)")

ECALL = 0x00000073


def fmt_val(hex_str, xlen):
    return format(int(hex_str, 16) & ((1 << xlen) - 1), "0{}x".format(xlen // 4))


def convert(log_path, csv_path, xlen, stop_at_ecall=True):
    n = 0
    stop_reason = "eof"
    with open(log_path) as fin, open(csv_path, "w", newline="") as fout:
        writer = csv.DictWriter(fout, fieldnames=CSV_FIELDS)
        writer.writeheader()
        pending = None  # row dict awaiting its (optional) gpr write line

        def flush(row):
            nonlocal n
            if row is not None:
                writer.writerow(row)
                n += 1

        for line in fin:
            mi = INSTR_RE.match(line)
            if mi:
                flush(pending)
                pending = None
                binv = mi.group("bin").lower()
                if stop_at_ecall and int(binv, 16) == ECALL:
                    return n, "ecall"  # stop; do not emit the ecall itself
                pending = {
                    "pc": mi.group("addr").lower(),
                    "instr": binv,
                    "gpr": "",
                    "csr": "",
                    "binary": binv,
                    "mode": mi.group("pri").lower(),
                    "instr_str": mi.group("instr").strip(),
                    "operand": "",
                    "pad": "",
                }
                continue
            mr = RD_RE.match(line)
            if mr and pending is not None:
                reg = int(mr.group("reg"))
                if reg != 0:
                    pending["gpr"] = "{}:{}".format(ABI[reg],
                                                    fmt_val(mr.group("val"), xlen))
        flush(pending)
    return n, stop_reason


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--log", required=True, help="Sail --trace-output file")
    ap.add_argument("--csv", required=True, help="output riscv-dv trace CSV")
    ap.add_argument("--xlen", type=int, default=32, choices=[32, 64])
    ap.add_argument("--no-stop-at-ecall", dest="stop_at_ecall",
                    action="store_false", help="do not truncate at the first ecall")
    args = ap.parse_args()
    n, stop_reason = convert(args.log, args.csv, args.xlen, args.stop_at_ecall)
    print("sail_trace_to_csv: wrote {} instructions to {} (stop_reason={})".format(
          n, args.csv, stop_reason), file=sys.stderr)
    if n == 0:
        print("warning: no instruction lines found -- did you pass "
              "--trace-instr --trace-gpr?", file=sys.stderr)
        sys.exit(2)


if __name__ == "__main__":
    main()
