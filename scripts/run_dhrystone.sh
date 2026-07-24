#!/usr/bin/env bash
# run_dhrystone.sh -- build & run the upstream Dhrystone benchmark on a niigo Vtop
# and report DMIPS/MHz.
#
# Usage:
#   scripts/run_dhrystone.sh <VTOP_BIN> <LABEL> [RUNS ...]
#     VTOP_BIN : Vtop built with PERF=1 (or any RV64GC OoO build)
#     LABEL    : output subdir under output/dhrystone/<LABEL>
#     RUNS     : Dhrystone iteration counts to sweep (default: 500 2000 10000)
#
# Sources: the benchmark (dhrystone.c / dhrystone_main.c / dhrystone.h) is used
# VERBATIM from the riscv-software-src/riscv-tests clone under references/, built
# with upstream's exact RISCV_GCC_OPTS, as separate translation units (no LTO), so
# the score is comparable to other riscv-tests Dhrystone results. Only the
# environment is ours (tests/perf/dhry/{start.S,env.c} + tests/perf/bench.ld):
# upstream's crt.S/syscalls.c target the HTIF console, which this SoC lacks.
#
# Scoring. Under __riscv, dhrystone.h sets HZ = 1000000 and takes its timestamps
# from read_csr(mcycle) -- i.e. it measures in CYCLES and reports as if the clock
# were 1 MHz. Its printed "Dhrystones per Second" is therefore already
# per-megahertz, and needs no assumed clock frequency:
#
#     DMIPS/MHz = (Dhrystones/s per MHz) / 1757
#               = (1e6 / cycles_per_iteration) / 1757
#
# 1757 is the VAX 11/780 reference rate (1 VAX MIPS == 1757 Dhrystones/s).
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
UP="$ROOT/references/riscv-software-tests"          # riscv-software-src/riscv-tests
BM="$UP/benchmarks"
HERE="$ROOT/tests/perf/dhry"

if [ ! -d "$BM/dhrystone" ]; then
  echo "error: $BM/dhrystone not found." >&2
  echo "  git clone --recursive https://github.com/riscv-software-src/riscv-tests.git $UP" >&2
  exit 1
fi

GCC=riscv64-unknown-elf-gcc; LD=riscv64-unknown-elf-ld; OBJCOPY=riscv64-unknown-elf-objcopy

# Upstream benchmarks/Makefile RISCV_GCC_OPTS, verbatim (-O2, rv64gc, lp64d).
UPSTREAM_OPTS="-U_FORTIFY_SOURCE -DPREALLOCATE=1 -mcmodel=medany -static -std=gnu99 -O2 \
  -ffast-math -fno-common -fno-builtin-printf -fno-tree-loop-distribute-patterns \
  -Wno-implicit-int -Wno-implicit-function-declaration -march=rv64gc -mabi=lp64d"

VTOP="${1:?usage: run_dhrystone.sh VTOP_BIN LABEL [RUNS ...]}"
VTOP="$(cd "$(dirname "$VTOP")" && pwd)/$(basename "$VTOP")"   # abs: we cd into the run dir
LABEL="${2:?missing LABEL}"; shift 2
RUNS_LIST=("$@"); [ ${#RUNS_LIST[@]} -eq 0 ] && RUNS_LIST=(500 2000 10000)

printf "%-8s %-12s %-14s %-10s %-9s %s\n" RUNS CYCLES CYC/ITER DMIPS/MHz IPC CHECK

for RUNS in "${RUNS_LIST[@]}"; do
  RUN="$ROOT/output/dhrystone/$LABEL/runs$RUNS"; rm -rf "$RUN"; mkdir -p "$RUN/src"

  # Stage the benchmark. The .c files are copied BYTE-IDENTICAL (asserted below);
  # only dhrystone.h is touched, and only to wrap its hard `#define NUMBER_OF_RUNS
  # 500` in an include guard so -D on the command line can set the run count. The
  # benchmark body is never modified.
  #
  # The sources must be staged next to the patched header rather than -I'd: a
  # quoted #include "dhrystone.h" always searches the including file's own
  # directory first, so a patched copy reached via -I would be silently ignored
  # (it was, on the first attempt -- every run came out at 500 iterations).
  cp "$BM/dhrystone/dhrystone.c" "$BM/dhrystone/dhrystone_main.c" "$RUN/src/"
  cmp -s "$BM/dhrystone/dhrystone.c"      "$RUN/src/dhrystone.c"
  cmp -s "$BM/dhrystone/dhrystone_main.c" "$RUN/src/dhrystone_main.c" \
    || { echo "error: staged benchmark sources differ from upstream" >&2; exit 1; }

  sed 's|^#define NUMBER_OF_RUNS\t*500.*|#ifndef NUMBER_OF_RUNS\n#define NUMBER_OF_RUNS 500\n#endif|' \
    "$BM/dhrystone/dhrystone.h" > "$RUN/src/dhrystone.h"
  grep -q '#ifndef NUMBER_OF_RUNS' "$RUN/src/dhrystone.h" \
    || { echo "error: failed to patch NUMBER_OF_RUNS guard into dhrystone.h" >&2; exit 1; }

  INCS="-I$RUN/src -I$BM/common -I$UP/env -I$HERE"
  CF="$UPSTREAM_OPTS $INCS -DNUMBER_OF_RUNS=$RUNS"

  # Separate TUs, no LTO: dhrystone_main.c must not inline dhrystone.c's Proc_*/Func_*.
  $GCC $CF -c "$RUN/src/dhrystone.c"      -o "$RUN/dhrystone.o"
  $GCC $CF -c "$RUN/src/dhrystone_main.c" -o "$RUN/dhrystone_main.o"
  $GCC $CF -c "$HERE/env.c"               -o "$RUN/env.o"
  $GCC $CF -c "$HERE/start.S"             -o "$RUN/start.o"

  LIBGCC="$($GCC -march=rv64gc -mabi=lp64d -print-libgcc-file-name)"
  # Link with ld -T directly: the gcc driver's default script silently overrides
  # bench.ld's segment placement (see run_perf_suite.sh).
  $LD -m elf64lriscv -T "$ROOT/tests/perf/bench.ld" \
    "$RUN/start.o" "$RUN/dhrystone_main.o" "$RUN/dhrystone.o" "$RUN/env.o" \
    "$LIBGCC" -o "$RUN/dhrystone.elf"

  $OBJCOPY -O binary -j .text "$RUN/dhrystone.elf" "$RUN/mem.text.bin"
  $OBJCOPY -O binary -j .data -j .bss --set-section-flags .bss=alloc,load,contents \
    "$RUN/dhrystone.elf" "$RUN/mem.data.bin" 2>/dev/null || : > "$RUN/mem.data.bin"
  : > "$RUN/mem.ktext.bin"; : > "$RUN/mem.kdata.bin"

  # PLUSARGS: extra whitespace-separated Vtop plusargs (e.g. '+mem_fuzz +mem_min=16 +mem_max=16').
  # Must be IDENTICAL across both labels of an A/B.
  ( cd "$RUN" && "$VTOP" +perf_out=perf.txt ${PLUSARGS:-} ) > "$RUN/sim.log" 2>&1 \
    || echo "  (Vtop exit $?)"

  CHECK=$(grep -oP 'DHRY-CHECK: \K\w+'          "$RUN/sim.log" 2>/dev/null || echo "NONE")
  CYC=$(  grep -oP 'DHRY-USER-TIME-CYCLES: \K\d+' "$RUN/sim.log" 2>/dev/null || echo 0)
  LCYC=$( grep -oP 'DHRY-LOOP-CYCLES: \K\d+'    "$RUN/sim.log" 2>/dev/null || echo 0)
  LRET=$( grep -oP 'DHRY-LOOP-INSTRET: \K\d+'   "$RUN/sim.log" 2>/dev/null || echo 0)

  awk -v runs="$RUNS" -v cyc="$CYC" -v lcyc="$LCYC" -v lret="$LRET" -v chk="$CHECK" 'BEGIN{
    cpi  = cyc  ? cyc / runs : 0;
    dm   = cpi  ? (1e6 / cpi) / 1757.0 : 0;
    ipc  = lcyc ? lret / lcyc : 0;
    printf "%-8s %-12s %-14.2f %-10.3f %-9.3f %s\n", runs, cyc, cpi, dm, ipc, chk;
  }'
done
