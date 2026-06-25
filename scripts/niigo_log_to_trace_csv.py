#!/usr/bin/env python3
"""Convert a niigo OoO simulation log into a riscv-dv instruction-trace CSV.

The OoO core, built with ``AGENT_DEBUG=1`` (``-DAGENT_DEBUG``), prints one line
per *committed* (retired) instruction from ``src/riscv_core_ooo.sv``:

    [<cyc>] retire[<slot>] pc=<hex> instr=<hex> rd=<dec> wr=<bit> data=<hex> exc=<bit> cause=<dec>

The ROB retires in program order, so these lines already form a clean
architectural instruction stream -- exactly what riscv-dv's
``instr_trace_compare.py`` consumes in its default ``--in_order_mode``.

The emitted CSV uses the riscv-dv column schema
(``pc,instr,gpr,csr,binary,mode,instr_str,operand,pad``).  Only the ``gpr``
column is compared by the trace differ; values are normalised to lowercase,
zero-padded to XLEN/4 hex digits so the string compare in ``check_update_gpr``
lines up with the golden side (see ``sail_trace_to_csv.py``).

Capture stops at the first ``ecall`` (encoding ``0x00000073``); riscv-dv test
programs reach completion via ``test_done: ecall`` and the golden parser stops
there too, so both traces cover the same ``_start .. ecall`` window.
"""

import argparse
import csv
import re
import sys

# x<n> -> ABI name, matching riscv-dv scripts/lib.py gpr_to_abi().
ABI = ["zero", "ra", "sp", "gp", "tp", "t0", "t1", "t2",
       "s0", "s1", "a0", "a1", "a2", "a3", "a4", "a5",
       "a6", "a7", "s2", "s3", "s4", "s5", "s6", "s7",
       "s8", "s9", "s10", "s11", "t3", "t4", "t5", "t6"]

CSV_FIELDS = ["pc", "instr", "gpr", "csr", "binary", "mode",
              "instr_str", "operand", "pad"]

RETIRE_RE = re.compile(
    r"retire\[\d+\]\s+pc=([0-9a-fA-F]+)\s+instr=([0-9a-fA-F]+)\s+"
    r"rd=(\d+)\s+wr=([01])\s+data=([0-9a-fA-F]+)\s+"
    r"exc=([01])\s+cause=(\d+)")

ECALL = 0x00000073


def fmt_val(hex_str, xlen):
    """Lowercase, zero-padded XLEN-wide hex (no 0x), for stable string compare."""
    return format(int(hex_str, 16) & ((1 << xlen) - 1), "0{}x".format(xlen // 4))


def convert(log_path, csv_path, xlen, stop_at_ecall=True, start_pc=None):
    n = 0
    # niigo resets at 0x00400000 and runs a 2-instr trampoline
    # (auipc/jr, installed by load_elf_mem --bootstrap) into the program entry.
    # Sail starts at the entry directly, so skip retires until we reach start_pc
    # to keep the two streams aligned.
    started = start_pc is None
    start_pc = None if start_pc is None else int(start_pc, 16)
    stop_reason = "eof"   # "ecall" if we hit the program's terminating ecall
    last_pc = None
    with open(log_path) as fin, open(csv_path, "w", newline="") as fout:
        writer = csv.DictWriter(fout, fieldnames=CSV_FIELDS)
        writer.writeheader()
        for line in fin:
            m = RETIRE_RE.search(line)
            if not m:
                continue
            pc, instr, rd, wr, data, exc, _cause = m.groups()
            if not started:
                if int(pc, 16) != start_pc:
                    continue
                started = True
            last_pc = pc.lower()
            if stop_at_ecall and int(instr, 16) == ECALL:
                stop_reason = "ecall"
                break
            gpr = ""
            # A faulting instruction (exc=1) writes no destination; x0 is never
            # an architectural update.
            if wr == "1" and exc == "0" and int(rd) != 0:
                gpr = "{}:{}".format(ABI[int(rd)], fmt_val(data, xlen))
            writer.writerow({
                "pc": pc.lower(),
                "instr": instr.lower(),
                "gpr": gpr,
                "csr": "",
                "binary": instr.lower(),
                "mode": "",
                "instr_str": "",
                "operand": "",
                "pad": "",
            })
            n += 1
    return n, stop_reason, last_pc


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--log", required=True, help="niigo Vtop sim log (AGENT_DEBUG)")
    ap.add_argument("--csv", required=True, help="output riscv-dv trace CSV")
    ap.add_argument("--xlen", type=int, default=32, choices=[32, 64])
    ap.add_argument("--no-stop-at-ecall", dest="stop_at_ecall",
                    action="store_false", help="do not truncate at the first ecall")
    ap.add_argument("--start-pc", default=None,
                    help="begin capture at this hex PC (the program entry), "
                         "skipping the reset trampoline")
    args = ap.parse_args()
    n, stop_reason, last_pc = convert(args.log, args.csv, args.xlen,
                                      args.stop_at_ecall, args.start_pc)
    # stop_reason is machine-readable for the driver: "ecall" means the program
    # reached its terminating ecall; "eof" means the log ran out first (a hung or
    # killed run, or a divergence) -- the driver treats that as a failure.
    print("niigo_log_to_trace_csv: wrote {} instructions to {} "
          "(stop_reason={} last_pc={})".format(n, args.csv, stop_reason, last_pc),
          file=sys.stderr)
    if n == 0:
        print("warning: no 'retire[..]' lines found -- was Vtop built with "
              "AGENT_DEBUG=1?", file=sys.stderr)
        sys.exit(2)


if __name__ == "__main__":
    main()
