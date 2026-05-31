#!/usr/bin/env python3
"""Run 447 reference benchmarks and aggregate OoO performance counters."""

from __future__ import annotations

import argparse
import math
import re
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]

SCALAR_PATTERNS = {
    "cycles": r"^Total cycles: (\d+)",
    "dispatched": r"^Instructions dispatched: (\d+)",
    "retired": r"^Instructions retired: (\d+)",
    "alu_instructions": r"^\s*ALU instructions: (\d+)",
    "load_instructions": r"^\s*Load instructions: (\d+)",
    "store_instructions": r"^\s*Store instructions: (\d+)",
    "frontend_stall_cycles": r"^Frontend stall cycles: (\d+)",
    "jalr_predicted_correct": r"^JALR predicted correct: (\d+)",
    "jalr_predicted_incorrect": r"^JALR predicted incorrect: (\d+)",
    "jalr_unpredicted": r"^JALR unpredicted: (\d+)",
    "return_predicted_correct": r"^Return predicted correct: (\d+)",
    "return_predicted_incorrect": r"^Return predicted incorrect: (\d+)",
    "return_unpredicted": r"^Return unpredicted: (\d+)",
    "data_reads": r"^Total data reads: (\d+)",
    "data_writes": r"^Total data writes: (\d+)",
    "control_flow_instructions": r"^Total control flow instructions: (\d+)",
    "mispredicted_control_flow_instructions":
        r"^Mispredicted control flow instructions: (\d+)",
}

COUNTER_REGEXES = {key: re.compile(pattern) for key, pattern in SCALAR_PATTERNS.items()}
STALL_RE = re.compile(r"^Dispatched instructions with (\d+) stalls: (\d+)")
BRANCH_RE = re.compile(r"^Branch inst \(idx (\d+)\):\s+(\d+)")
JAL_RE = re.compile(r"^JAL inst \(idx (\d+)\):\s+(\d+)")
JALR_RE = re.compile(r"^JALR inst \(idx (\d+)\):\s+(\d+)")

SUMMARY_COLUMNS = [
    "benchmark",
    "cycles",
    "dispatched",
    "retired",
    "ipc",
    "dispatch_per_cycle",
    "alu_instructions",
    "load_instructions",
    "store_instructions",
    "alu_pct_retired",
    "load_pct_retired",
    "store_pct_retired",
    "frontend_stall_cycles",
    "frontend_stall_pct",
    *[f"dispatch_stall_{i}" for i in range(8)],
    *[f"branch_idx_{i}" for i in range(16)],
    *[f"jal_idx_{i}" for i in range(8)],
    *[f"jalr_idx_{i}" for i in range(8)],
    "jalr_predicted_correct",
    "jalr_predicted_incorrect",
    "jalr_unpredicted",
    "return_predicted_correct",
    "return_predicted_incorrect",
    "return_unpredicted",
    "data_reads",
    "data_writes",
    "control_flow_instructions",
    "mispredicted_control_flow_instructions",
    "branch_mispredict_pct",
]


def fmt(value: object) -> str:
    if isinstance(value, float):
        return f"{value:.6f}"
    return str(value)


def parse_perf_counters(benchmark: str, text: str) -> tuple[dict[str, int | float | str], list[str]]:
    lines = text.splitlines()
    counters: dict[str, int | float | str] = {"benchmark": benchmark}
    for idx in range(8):
        counters[f"dispatch_stall_{idx}"] = 0
        counters[f"jal_idx_{idx}"] = 0
        counters[f"jalr_idx_{idx}"] = 0
    for idx in range(16):
        counters[f"branch_idx_{idx}"] = 0

    try:
        start = lines.index("FINAL OOO PERFORMANCE COUNTERS:")
    except ValueError as exc:
        raise ValueError(f"{benchmark}: missing performance counter block") from exc

    end = len(lines)
    for idx in range(start + 1, len(lines)):
        if lines[idx].startswith("18-447 Register File Dump"):
            end = idx
            break
    block = lines[start:end]

    for line in block:
        for key, regex in COUNTER_REGEXES.items():
            match = regex.match(line)
            if match:
                counters[key] = int(match.group(1))
        if match := STALL_RE.match(line):
            counters[f"dispatch_stall_{int(match.group(1))}"] = int(match.group(2))
        if match := BRANCH_RE.match(line):
            counters[f"branch_idx_{int(match.group(1))}"] = int(match.group(2))
        if match := JAL_RE.match(line):
            counters[f"jal_idx_{int(match.group(1))}"] = int(match.group(2))
        if match := JALR_RE.match(line):
            counters[f"jalr_idx_{int(match.group(1))}"] = int(match.group(2))

    missing = [key for key in COUNTER_REGEXES if key not in counters]
    if missing:
        raise ValueError(f"{benchmark}: missing counters {missing}")

    cycles = int(counters["cycles"])
    retired = int(counters["retired"])
    dispatched = int(counters["dispatched"])
    control_flow = int(counters["control_flow_instructions"])
    mispredicts = int(counters["mispredicted_control_flow_instructions"])

    counters["ipc"] = retired / cycles if cycles else 0.0
    counters["dispatch_per_cycle"] = dispatched / cycles if cycles else 0.0
    counters["frontend_stall_pct"] = (
        100.0 * int(counters["frontend_stall_cycles"]) / cycles if cycles else 0.0
    )
    counters["branch_mispredict_pct"] = (
        100.0 * mispredicts / control_flow if control_flow else 0.0
    )
    counters["load_pct_retired"] = (
        100.0 * int(counters["load_instructions"]) / retired if retired else 0.0
    )
    counters["store_pct_retired"] = (
        100.0 * int(counters["store_instructions"]) / retired if retired else 0.0
    )
    counters["alu_pct_retired"] = (
        100.0 * int(counters["alu_instructions"]) / retired if retired else 0.0
    )
    return counters, block


def run_benchmark(bench: Path, timeout: int) -> str:
    proc = subprocess.run(
        ["make", "verilator-verify", "OOO=1", f"TEST={bench}"],
        cwd=ROOT,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        timeout=timeout,
    )
    if proc.returncode != 0:
        tail = proc.stdout[-5000:]
        raise RuntimeError(f"{bench} failed with exit {proc.returncode}\n{tail}")
    return proc.stdout


def write_summary_tsv(path: Path, rows: list[dict[str, int | float | str]]) -> None:
    with path.open("w") as f:
        f.write("\t".join(SUMMARY_COLUMNS) + "\n")
        for row in rows:
            f.write("\t".join(fmt(row[column]) for column in SUMMARY_COLUMNS) + "\n")


def aggregate_rows(rows: list[dict[str, int | float | str]]) -> tuple[dict[str, int | float | str], float, float]:
    aggregate: dict[str, int | float | str] = {"benchmark": "AGGREGATE"}
    for column in SUMMARY_COLUMNS:
        if column == "benchmark" or column.endswith("_pct") or column in (
            "ipc",
            "dispatch_per_cycle",
        ):
            continue
        aggregate[column] = sum(int(row[column]) for row in rows)

    cycles = int(aggregate["cycles"])
    retired = int(aggregate["retired"])
    dispatched = int(aggregate["dispatched"])
    control_flow = int(aggregate["control_flow_instructions"])
    mispredicts = int(aggregate["mispredicted_control_flow_instructions"])

    aggregate["ipc"] = retired / cycles if cycles else 0.0
    aggregate["dispatch_per_cycle"] = dispatched / cycles if cycles else 0.0
    aggregate["frontend_stall_pct"] = (
        100.0 * int(aggregate["frontend_stall_cycles"]) / cycles if cycles else 0.0
    )
    aggregate["branch_mispredict_pct"] = (
        100.0 * mispredicts / control_flow if control_flow else 0.0
    )
    aggregate["load_pct_retired"] = (
        100.0 * int(aggregate["load_instructions"]) / retired if retired else 0.0
    )
    aggregate["store_pct_retired"] = (
        100.0 * int(aggregate["store_instructions"]) / retired if retired else 0.0
    )
    aggregate["alu_pct_retired"] = (
        100.0 * int(aggregate["alu_instructions"]) / retired if retired else 0.0
    )

    mean_ipc = sum(float(row["ipc"]) for row in rows) / len(rows)
    geomean_ipc = math.exp(sum(math.log(float(row["ipc"])) for row in rows) / len(rows))
    return aggregate, mean_ipc, geomean_ipc


def write_aggregate_report(
    path: Path,
    aggregate: dict[str, int | float | str],
    benchmark_count: int,
    mean_ipc: float,
    geomean_ipc: float,
) -> None:
    with path.open("w") as f:
        f.write("Aggregate performance metrics for 447ref/benchmarks run\n")
        f.write("=" * 72 + "\n")
        f.write(f"Benchmarks: {benchmark_count}\n")
        f.write(f"Total cycles: {aggregate['cycles']}\n")
        f.write(f"Total dispatched: {aggregate['dispatched']}\n")
        f.write(f"Total retired: {aggregate['retired']}\n")
        f.write(f"Aggregate IPC: {float(aggregate['ipc']):.6f}\n")
        f.write(f"Arithmetic mean IPC: {mean_ipc:.6f}\n")
        f.write(f"Geomean IPC: {geomean_ipc:.6f}\n")
        f.write(f"Aggregate dispatch/cycle: {float(aggregate['dispatch_per_cycle']):.6f}\n")
        f.write(
            f"Frontend stall cycles: {aggregate['frontend_stall_cycles']} "
            f"({float(aggregate['frontend_stall_pct']):.3f}%)\n"
        )
        f.write(
            f"ALU instructions: {aggregate['alu_instructions']} "
            f"({float(aggregate['alu_pct_retired']):.3f}% retired)\n"
        )
        f.write(
            f"Load instructions: {aggregate['load_instructions']} "
            f"({float(aggregate['load_pct_retired']):.3f}% retired)\n"
        )
        f.write(
            f"Store instructions: {aggregate['store_instructions']} "
            f"({float(aggregate['store_pct_retired']):.3f}% retired)\n"
        )
        f.write(f"Data reads: {aggregate['data_reads']}\n")
        f.write(f"Data writes: {aggregate['data_writes']}\n")
        f.write(f"Control-flow instructions: {aggregate['control_flow_instructions']}\n")
        f.write(
            "Mispredicted control-flow instructions: "
            f"{aggregate['mispredicted_control_flow_instructions']} "
            f"({float(aggregate['branch_mispredict_pct']):.3f}%)\n"
        )
        f.write(f"JALR predicted correct: {aggregate['jalr_predicted_correct']}\n")
        f.write(f"JALR predicted incorrect: {aggregate['jalr_predicted_incorrect']}\n")
        f.write(f"JALR unpredicted: {aggregate['jalr_unpredicted']}\n")
        f.write(f"Return predicted correct: {aggregate['return_predicted_correct']}\n")
        f.write(f"Return predicted incorrect: {aggregate['return_predicted_incorrect']}\n")
        f.write(f"Return unpredicted: {aggregate['return_unpredicted']}\n")

        f.write("\nDispatch stall histogram:\n")
        for idx in range(8):
            f.write(f"  {idx} stalls: {aggregate[f'dispatch_stall_{idx}']}\n")

        f.write("\nBranch idx histogram:\n")
        for idx in range(16):
            f.write(f"  idx {idx}: {aggregate[f'branch_idx_{idx}']}\n")

        f.write("\nJAL idx histogram:\n")
        for idx in range(8):
            f.write(f"  idx {idx}: {aggregate[f'jal_idx_{idx}']}\n")

        f.write("\nJALR idx histogram:\n")
        for idx in range(8):
            f.write(f"  idx {idx}: {aggregate[f'jalr_idx_{idx}']}\n")


def write_full_counter_report(path: Path, blocks: list[tuple[str, list[str]]]) -> None:
    with path.open("w") as f:
        f.write("Full OOO performance counter reports for 447ref/benchmarks run\n")
        f.write("=" * 72 + "\n\n")
        for name, block in blocks:
            f.write(f"## {name}\n")
            f.write("\n".join(block).rstrip())
            f.write("\n\n")


def read_previous_summary(path: Path) -> dict[str, dict[str, str]]:
    if not path.is_file():
        return {}
    lines = path.read_text().splitlines()
    if not lines:
        return {}
    header = lines[0].split("\t")
    rows = {}
    for line in lines[1:]:
        parts = line.split("\t")
        row = dict(zip(header, parts))
        rows[row["benchmark"]] = row
    return rows


def write_comparison(
    path: Path,
    previous_rows: dict[str, dict[str, str]],
    rows: list[dict[str, int | float | str]],
) -> None:
    if not previous_rows:
        return
    with path.open("w") as f:
        f.write(
            "benchmark\told_cycles\tnew_cycles\tcycle_delta_pct\t"
            "old_ipc\tnew_ipc\tipc_delta_pct\told_misp_pct\tnew_misp_pct\n"
        )
        for row in rows:
            previous = previous_rows.get(str(row["benchmark"]))
            if not previous:
                continue
            old_cycles = float(previous["cycles"])
            old_ipc = float(previous["ipc"])
            old_misp = float(previous["branch_mispredict_pct"])
            new_cycles = int(row["cycles"])
            new_ipc = float(row["ipc"])
            new_misp = float(row["branch_mispredict_pct"])
            f.write(
                f"{row['benchmark']}\t"
                f"{int(old_cycles)}\t{new_cycles}\t"
                f"{100 * (new_cycles - old_cycles) / old_cycles:.6f}\t"
                f"{old_ipc:.6f}\t{new_ipc:.6f}\t"
                f"{100 * (new_ipc - old_ipc) / old_ipc:.6f}\t"
                f"{old_misp:.6f}\t{new_misp:.6f}\n"
            )


def main(argv: list[str]) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--benchmark-dir",
        type=Path,
        default=ROOT / "447ref" / "benchmarks",
        help="Directory containing benchmark .c files.",
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=ROOT / "output" / "447ref-benchmarks-current",
        help="Directory for logs and aggregate reports.",
    )
    parser.add_argument(
        "--timeout",
        type=int,
        default=300,
        help="Timeout per benchmark, in seconds.",
    )
    parser.add_argument(
        "--compare",
        type=Path,
        default=ROOT / "output" / "447ref-benchmarks" / "summary.tsv",
        help="Optional prior summary TSV to compare against.",
    )
    args = parser.parse_args(argv)

    benchmark_dir = args.benchmark_dir.resolve()
    output_dir = args.output.resolve()
    benchmarks = sorted(benchmark_dir.glob("*.c"))
    if not benchmarks:
        print(f"no benchmark .c files found under {benchmark_dir}", file=sys.stderr)
        return 2

    output_dir.mkdir(parents=True, exist_ok=True)

    rows: list[dict[str, int | float | str]] = []
    full_blocks: list[tuple[str, list[str]]] = []
    for bench in benchmarks:
        rel_bench = bench.relative_to(ROOT) if bench.is_relative_to(ROOT) else bench
        print(f"RUN {rel_bench}", flush=True)
        try:
            stdout = run_benchmark(rel_bench, args.timeout)
            (output_dir / f"{bench.stem}.log").write_text(stdout)
            counters, block = parse_perf_counters(bench.stem, stdout)
        except Exception as exc:
            print(str(exc), file=sys.stderr)
            return 1
        rows.append(counters)
        full_blocks.append((bench.stem, block))
        print(
            f"PASS {bench.stem}: cycles={counters['cycles']} "
            f"retired={counters['retired']} ipc={float(counters['ipc']):.3f} "
            f"br_misp={float(counters['branch_mispredict_pct']):.1f}%",
            flush=True,
        )

    aggregate, mean_ipc, geomean_ipc = aggregate_rows(rows)

    summary_path = output_dir / "summary_all_metrics.tsv"
    aggregate_txt_path = output_dir / "aggregate_all_metrics.txt"
    aggregate_tsv_path = output_dir / "aggregate_all_metrics.tsv"
    full_report_path = output_dir / "full_perf_counters.txt"
    compare_path = output_dir / "compare_to_previous_summary.tsv"

    write_summary_tsv(summary_path, rows)
    write_summary_tsv(aggregate_tsv_path, [aggregate])
    write_aggregate_report(aggregate_txt_path, aggregate, len(rows), mean_ipc, geomean_ipc)
    write_full_counter_report(full_report_path, full_blocks)
    write_comparison(compare_path, read_previous_summary(args.compare), rows)

    print(f"SUMMARY_TSV {summary_path}")
    print(f"AGGREGATE_TXT {aggregate_txt_path}")
    print(f"AGGREGATE_TSV {aggregate_tsv_path}")
    print(f"FULL_REPORT {full_report_path}")
    if compare_path.is_file():
        print(f"COMPARE_TSV {compare_path}")
    print(f"BENCHMARKS {len(rows)}")
    print(f"TOTAL_CYCLES {aggregate['cycles']}")
    print(f"TOTAL_RETIRED {aggregate['retired']}")
    print(f"AGG_IPC {float(aggregate['ipc']):.6f}")
    print(f"MEAN_IPC {mean_ipc:.6f}")
    print(f"GEOMEAN_IPC {geomean_ipc:.6f}")
    print(f"AGG_DISPATCH_PER_CYCLE {float(aggregate['dispatch_per_cycle']):.6f}")
    print(f"AGG_FRONTEND_STALL_PCT {float(aggregate['frontend_stall_pct']):.3f}")
    print(f"AGG_BRANCH_MISPREDICT_PCT {float(aggregate['branch_mispredict_pct']):.3f}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
