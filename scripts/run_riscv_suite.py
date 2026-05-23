#!/usr/bin/env python3
"""Run niigo-lake Verilator simulations against ACT ELF files."""

from __future__ import annotations

import argparse
import re
import subprocess
import sys
from concurrent.futures import ProcessPoolExecutor, as_completed
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
RUN_ONE = ROOT / "scripts" / "run_riscv_test.sh"
PASS_RE = re.compile(r"RVCP-SUMMARY: TEST PASSED")


def run_one(elf: Path, output_root: Path, timeout: int) -> tuple[Path, bool, str]:
    out_dir = output_root / elf.parent.name / elf.stem
    proc = subprocess.run(
        [str(RUN_ONE), str(elf), str(out_dir)],
        capture_output=True,
        text=True,
        timeout=timeout + 5,
    )
    summary = proc.stdout.strip() or proc.stderr.strip()
    passed = proc.returncode == 0 and bool(PASS_RE.search(summary))
    return elf, passed, summary.splitlines()[-1] if summary else "no summary"


def collect_elfs(elf_dir: Path, extensions: list[str]) -> list[Path]:
    elfs: list[Path] = []
    for ext in extensions:
        path = elf_dir / "rv32i" / ext
        if path.is_dir():
            elfs.extend(sorted(path.glob("*.elf")))
    return elfs


def main(argv: list[str]) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--elf-dir",
        type=Path,
        default=ROOT / "references/riscv-tests/work/niigo-rv32g/elfs",
    )
    parser.add_argument(
        "--extensions",
        default="I",
        help="Comma-separated ACT extension directory names under rv32i/",
    )
    parser.add_argument("--jobs", type=int, default=24)
    parser.add_argument("--timeout", type=int, default=120)
    parser.add_argument(
        "--output",
        type=Path,
        default=ROOT / "output/riscv-tests",
    )
    args = parser.parse_args(argv)

    ext_dirs = [part.strip() for part in args.extensions.split(",") if part.strip()]
    elfs = collect_elfs(args.elf_dir, ext_dirs)
    if not elfs:
        print(f"no ELFs found under {args.elf_dir}/rv32i/", file=sys.stderr)
        return 2

    passed = 0
    failed: list[tuple[Path, str]] = []

    with ProcessPoolExecutor(max_workers=args.jobs) as pool:
        futures = [
            pool.submit(run_one, elf, args.output, args.timeout) for elf in elfs
        ]
        for future in as_completed(futures):
            elf, ok, summary = future.result()
            if ok:
                passed += 1
                print(f"PASS  {elf.name}")
            else:
                failed.append((elf, summary))
                print(f"FAIL  {elf.name}: {summary}")

    print(f"\n{passed}/{len(elfs)} passed")
    if failed:
        print("\nFailures:")
        for elf, summary in failed:
            print(f"  {elf}: {summary}")
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
