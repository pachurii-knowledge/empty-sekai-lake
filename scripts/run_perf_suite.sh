#!/usr/bin/env bash
# run_perf_suite.sh — compile the tests/perf RV64GC microbenchmarks, run each on a
# given Vtop, and collect per-cycle performance counters (+perf_out).
#
# Usage:
#   scripts/run_perf_suite.sh <VTOP_BIN> <LABEL> [bench ...]
#     VTOP_BIN : path to a Vtop built with RV64=1 OOO=1 RVC=1 [L1D=1]
#     LABEL    : output subdir under output/perf/<LABEL>
#     bench    : benchmark basenames (default: all in tests/perf/*.c)
# Env: MAXCYC (optional +maxcyc cap for long runs).
#
# The benchmarks link .text@0x00400000 / .data@0x10000000 (bench.ld) to match the
# main_memory segment bases, and MUST be linked with `ld -T` directly — the gcc
# driver's default linker script silently overrides -T placement. main returning
# triggers the ECALL-halt, so the OoO core's `final` perf dump fires on $finish.
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
HERE="$ROOT/tests/perf"
GCC=riscv64-unknown-elf-gcc; LD=riscv64-unknown-elf-ld; OBJCOPY=riscv64-unknown-elf-objcopy
CF="-march=rv64gc -mabi=lp64d -O2 -msmall-data-limit=0 -ffreestanding -nostdlib"
LIBGCC="$($GCC -march=rv64gc -mabi=lp64d -print-libgcc-file-name)"

VTOP="${1:?usage: run_perf_suite.sh VTOP_BIN LABEL [bench ...]}"
LABEL="${2:?missing LABEL}"; shift 2
BENCHES=("$@"); [ ${#BENCHES[@]} -eq 0 ] && BENCHES=($(cd "$HERE" && ls *.c | sed 's/\.c$//'))

for NAME in "${BENCHES[@]}"; do
  RUN="$ROOT/output/perf/$LABEL/$NAME"; rm -rf "$RUN"; mkdir -p "$RUN"
  $GCC $CF -c "$HERE/start.S"     -o "$RUN/start.o"
  $GCC $CF -c "$HERE/$NAME.c"     -o "$RUN/bench.o"
  $LD -m elf64lriscv -T "$HERE/bench.ld" "$RUN/start.o" "$RUN/bench.o" "$LIBGCC" -o "$RUN/$NAME.elf"
  $OBJCOPY -O binary -j .text "$RUN/$NAME.elf" "$RUN/mem.text.bin"
  $OBJCOPY -O binary -j .data -j .bss --set-section-flags .bss=alloc,load,contents \
    "$RUN/$NAME.elf" "$RUN/mem.data.bin" 2>/dev/null || : > "$RUN/mem.data.bin"
  : > "$RUN/mem.ktext.bin"; : > "$RUN/mem.kdata.bin"
  ARGS=(+perf_out=perf.txt); [ -n "${MAXCYC:-}" ] && ARGS+=(+maxcyc="$MAXCYC")
  ( cd "$RUN" && "$VTOP" "${ARGS[@]}" ) > "$RUN/sim.log" 2>&1 || echo "  ($NAME Vtop exit $?)"
  CYC=$(grep -oP '^cycles=\K\d+' "$RUN/perf.txt" 2>/dev/null || echo 0)
  RET=$(grep -oP '^retired=\K\d+' "$RUN/perf.txt" 2>/dev/null || echo 0)
  IPC=$(awk "BEGIN{printf \"%.3f\", $CYC? $RET/$CYC : 0}")
  printf "%-10s IPC=%-6s cyc=%-10s ret=%s\n" "$NAME" "$IPC" "$CYC" "$RET"
done
