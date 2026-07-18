#!/usr/bin/env bash
# run_embench.sh -- niigo's DEFAULT perf-benchmarking suite. Builds the Embench-IOT
# benchmarks bare-metal RV64GC, runs each on a niigo Vtop, and reports the timed-region
# (start_trigger->stop_trigger) cycles / instret / IPC per benchmark plus a geomean.
# Unlike Dhrystone (flat ~1.03 IPC, cs_latency-bound), Embench's IPC range (0.36-2.03)
# credits the backend width/window/ILP levers (ALU4/LSQ_MLP2/FP_OOO/DEEP_WINDOW/BTB/...).
# For lever A/B, run two labels then compare with scripts/embench_compare.sh.
#
# Usage:
#   scripts/run_embench.sh <VTOP_BIN> <LABEL> [bench ...]
#     VTOP_BIN : Vtop built with RV64=1 OOO=1 RVC=1 [L1D=1]  (PERF=1 to credit levers)
#     LABEL    : output subdir under output/embench/<LABEL>  (+ summary.tsv for compare)
#     bench    : benchmark names (default: all references/embench-iot/src/*)
# Env: LSF (LOCAL_SCALE_FACTOR, default 4 = the canonical scale for this ~128k-cyc/s
#      functional sim; the native HW scale targets billions of cycles and is impractical.
#      Same LSF across an A/B keeps the geomean-of-ratios valid. LSF=0 => native HW scale.
#      NOTE: this is a SCALED functional-sim score, not the official upstream Embench score),
#      GSF (GLOBAL_SCALE_FACTOR, default 1), WARMUP (WARMUP_HEAT, default 0),
#      MAXCYC (+maxcyc cap for long runs), HEAP (EMBENCH_HEAP_SIZE bytes).
#
# The Embench benchmarks are used VERBATIM from the gitignored upstream clone
# (references/embench-iot); only the environment is ours: tests/embench/boardsupport.c
# supplies the board hooks (mcycle/minstret triggers -> UART markers), a minimal
# freestanding libc, and main(); it is linked with tests/perf/{start.S,bench.ld}
# (same .text@0x00400000 / .data@0x10000000 placement as the perf suite). The score
# is the START->STOP delta so it excludes warmup/verify overhead. Skipped (not
# failed) if the upstream clone is absent.
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
EMB="$ROOT/references/embench-iot"; SUP="$EMB/support"; BSP="$ROOT/tests/embench"; PERF="$ROOT/tests/perf"
GCC=riscv64-unknown-elf-gcc; LD=riscv64-unknown-elf-ld; OBJCOPY=riscv64-unknown-elf-objcopy
# -std=gnu11: several benchmarks (wikisort) `typedef uint8_t bool`, which the gcc-14+
# gnu23 default rejects (bool is a keyword). gnu11 is also closer to Embench's intent.
CF="-march=rv64gc -mabi=lp64d -O2 -msmall-data-limit=0 -std=gnu11 -ffreestanding -nostdlib"
DEFS="-DGLOBAL_SCALE_FACTOR=${GSF:-1} -DWARMUP_HEAT=${WARMUP:-0} -DCPU_MHZ=1"
[ -n "${HEAP:-}" ] && DEFS="$DEFS -DEMBENCH_HEAP_SIZE=$HEAP"
LIBGCC="$($GCC -march=rv64gc -mabi=lp64d -print-libgcc-file-name)"
# newlib libc.a, pulled ON-DEMAND for self-contained libc symbols the benchmarks need
# but boardsupport.c does not provide (slre: tolower/_ctype_b). Already-defined symbols
# (our mem*/str*/printf) are NOT pulled, so no duplicate-symbol clash; syscall-dependent
# members would only be pulled if a benchmark referenced them (none do so far).
LIBC="$($GCC -march=rv64gc -mabi=lp64d -print-file-name=libc.a)"
LSF="${LSF:-4}"   # canonical default scale for the functional sim; LSF=0 => native HW scale

if [ ! -d "$EMB/src" ]; then
  echo "SKIP: clone embench/embench-iot -> references/embench-iot" >&2; exit 0
fi

VTOP="${1:?usage: run_embench.sh VTOP_BIN LABEL [bench ...]}"
VTOP="$(cd "$(dirname "$VTOP")" && pwd)/$(basename "$VTOP")"
LABEL="${2:?missing LABEL}"; shift 2
BENCHES=("$@"); [ ${#BENCHES[@]} -eq 0 ] && BENCHES=($(cd "$EMB/src" && ls -d */ | sed 's#/##'))

SUMDIR="$ROOT/output/embench/$LABEL"; mkdir -p "$SUMDIR"
SUM="$SUMDIR/summary.tsv"; printf "bench\tverify\tcycles\tinstret\tipc\n" > "$SUM"
echo "# Embench (LSF=$LSF) on $VTOP" > "$SUMDIR/meta.txt"
printf "%-16s %-8s %-12s %-11s %-7s\n" BENCH VERIFY CYCLES INSTRET IPC
sum_log=0; n_ok=0

for NAME in "${BENCHES[@]}"; do
  SRC="$EMB/src/$NAME"; [ -d "$SRC" ] || { printf "%-16s (no such benchmark)\n" "$NAME"; continue; }
  RUN="$ROOT/output/embench/$LABEL/$NAME"; rm -rf "$RUN"; mkdir -p "$RUN"
  OBJS=()
  # env: start.S + our board support + beebs allocator/rand
  $GCC $CF          -c "$PERF/start.S"     -o "$RUN/start.o"
  $GCC $CF $DEFS -I"$SUP" -I"$BSP" -c "$BSP/boardsupport.c" -o "$RUN/boardsupport.o"
  $GCC $CF $DEFS -I"$SUP"          -c "$SUP/beebsc.c"       -o "$RUN/beebsc.o"
  OBJS+=("$RUN/start.o" "$RUN/boardsupport.o" "$RUN/beebsc.o")
  # the benchmark's own translation unit(s). With LSF set, stage each .c and wrap
  # its `#define LOCAL_SCALE_FACTOR` in an include guard so -DLOCAL_SCALE_FACTOR wins
  # (mirrors run_dhrystone.sh's NUMBER_OF_RUNS patch); the benchmark body is otherwise
  # byte-identical. Without LSF, the native per-benchmark scale is used (HW-tuned).
  LSFDEF=""; [ "$LSF" != "0" ] && { LSFDEF="-DLOCAL_SCALE_FACTOR=$LSF"; mkdir -p "$RUN/src"; }
  ok=1
  for c in "$SRC"/*.c; do
    src="$c"
    if [ "$LSF" != "0" ]; then
      src="$RUN/src/$(basename "$c")"
      sed 's/^[[:space:]]*#define[[:space:]]\+LOCAL_SCALE_FACTOR\b.*/#ifndef LOCAL_SCALE_FACTOR\n#define LOCAL_SCALE_FACTOR 1\n#endif/' "$c" > "$src"
    fi
    o="$RUN/$(basename "${c%.c}").o"
    $GCC $CF $DEFS $LSFDEF -I"$SUP" -I"$SRC" -c "$src" -o "$o" 2>"$RUN/cc.log" || { printf "%-16s BUILD-FAIL (cc: %s)\n" "$NAME" "$(head -1 "$RUN/cc.log")"; printf "%s\tBUILD-FAIL\t0\t0\t0\n" "$NAME" >> "$SUM"; ok=0; break; }
    OBJS+=("$o")
  done
  [ $ok -eq 1 ] || continue
  $LD -m elf64lriscv -T "$PERF/bench.ld" "${OBJS[@]}" "$LIBC" "$LIBGCC" -o "$RUN/$NAME.elf" 2>"$RUN/ld.log" \
    || { printf "%-16s LINK-FAIL (%s)\n" "$NAME" "$(head -1 "$RUN/ld.log")"; printf "%s\tLINK-FAIL\t0\t0\t0\n" "$NAME" >> "$SUM"; continue; }
  $OBJCOPY -O binary -j .text "$RUN/$NAME.elf" "$RUN/mem.text.bin"
  $OBJCOPY -O binary -j .data -j .bss --set-section-flags .bss=alloc,load,contents \
    "$RUN/$NAME.elf" "$RUN/mem.data.bin" 2>/dev/null || : > "$RUN/mem.data.bin"
  : > "$RUN/mem.ktext.bin"; : > "$RUN/mem.kdata.bin"
  ARGS=(+perf_out=perf.txt); [ -n "${MAXCYC:-}" ] && ARGS+=(+maxcyc="$MAXCYC")
  ( cd "$RUN" && "$VTOP" "${ARGS[@]}" ) > "$RUN/sim.log" 2>&1 || echo "  ($NAME Vtop exit $?)"

  VER=$(grep -oP 'EMBENCH-VERIFY: \K\w+'  "$RUN/sim.log" 2>/dev/null || echo "NONE")
  CYC=$(grep -oP 'EMBENCH-CYCLES: \K\d+'  "$RUN/sim.log" 2>/dev/null || echo 0)
  INS=$(grep -oP 'EMBENCH-INSTRET: \K\d+' "$RUN/sim.log" 2>/dev/null || echo 0)
  IPC=$(awk "BEGIN{printf \"%.3f\", $CYC? $INS/$CYC : 0}")
  printf "%-16s %-8s %-12s %-11s %-7s\n" "$NAME" "$VER" "$CYC" "$INS" "$IPC"
  printf "%s\t%s\t%s\t%s\t%s\n" "$NAME" "$VER" "$CYC" "$INS" "$IPC" >> "$SUM"
  if [ "$VER" = "PASS" ] && [ "$CYC" -gt 0 ]; then
    sum_log=$(awk "BEGIN{print $sum_log + log($CYC)}"); n_ok=$((n_ok+1))
  fi
done

if [ "$n_ok" -gt 0 ]; then
  GEO=$(awk "BEGIN{printf \"%.0f\", exp($sum_log/$n_ok)}")
  printf -- "---\ngeomean cycles (%d PASS): %s\n" "$n_ok" "$GEO"
fi
echo "summary: $SUM  (A/B: scripts/embench_compare.sh <baseline-label> <this-label>)"
