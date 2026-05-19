#!/usr/bin/env python3
"""Compare simulator register dumps.

Assembly tests compare every architectural register. C benchmark reference
dumps are compiler-version-sensitive in caller-saved temporaries, so benchmark
mode compares the ABI-defined program result preserved by crt0.S.
"""

from __future__ import annotations

import re
import sys
from pathlib import Path


REG_RE = re.compile(r"^(x\d+)\s+\([^)]+\)\s+=\s+(0x[0-9a-fA-F]+)")


def read_regs(path: Path) -> dict[str, str]:
    regs: dict[str, str] = {}
    for line in path.read_text().splitlines():
        match = REG_RE.match(line)
        if match:
            regs[match.group(1)] = match.group(2).lower()
    return regs


def compare(sim_path: Path, ref_path: Path, mode: str) -> int:
    sim = read_regs(sim_path)
    ref = read_regs(ref_path)
    regs = ["x2", "x3", "x10"] if mode == "c" else [f"x{i}" for i in range(32)]

    failed = False
    for reg in regs:
        if sim.get(reg) != ref.get(reg):
            print(f"{reg}: simulator={sim.get(reg)} reference={ref.get(reg)}")
            failed = True

    return 1 if failed else 0


def main(argv: list[str]) -> int:
    if len(argv) != 4 or argv[3] not in {"asm", "c"}:
        print("usage: compare_regdump.py SIM.reg REF.reg asm|c", file=sys.stderr)
        return 2

    return compare(Path(argv[1]), Path(argv[2]), argv[3])


if __name__ == "__main__":
    raise SystemExit(main(sys.argv))
