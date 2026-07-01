#!/usr/bin/env bash
# char.sh — characterize per-module ASAP7 Fmax with controlled parallelism.
#   char.sh <parallelism> <period_ns> <module> [module ...]
# Each module is synthesized + placed + STA'd via timing_flow.tcl into its own BUILD_DIR;
# the Fmax line is appended to $CHAR/SUMMARY.txt. Logs land in $CHAR/<module>.log.
set -u
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "$HERE/../.." && pwd)"
OR=/home/mizuki/Desktop/workspace/OpenROAD/build/bin/openroad
FLIST="$ROOT/output/openroad/design.f"
CHAR="$ROOT/output/openroad/char"
PAR="$1"; PERIOD="$2"; shift 2
mkdir -p "$CHAR"

run_one() {
  local m="$1"
  local log="$CHAR/$m.log"
  local t0=$SECONDS
  TOP="$m" FILELIST="$FLIST" CLK_PERIOD_NS="$PERIOD" UTIL=40 STOP_AFTER=place \
    BUILD_DIR="$ROOT/output/openroad/char/$m" \
    timeout 2400 "$OR" -exit "$HERE/timing_flow.tcl" > "$log" 2>&1
  local rc=$?
  local dt=$((SECONDS - t0))
  local fmax cell area
  fmax=$(grep -oE 'PEAK FREQUENCY.*MHz' "$log" | grep -oE '[0-9]+\.[0-9]+ MHz' | head -1)
  area=$(grep -oE 'Design area [0-9]+ um\^2' "$log" | tail -1)
  if [ -z "$fmax" ]; then
    if [ $rc -eq 124 ]; then fmax="TIMEOUT(2400s)"; else fmax="FAIL(rc=$rc)"; fi
  fi
  printf '%-26s %-18s %-22s %5ds\n' "$m" "$fmax" "${area:-?}" "$dt" >> "$CHAR/SUMMARY.txt"
  echo "[done] $m -> $fmax (${dt}s)"
}
export -f run_one
export CHAR ROOT OR FLIST HERE PERIOD

printf '==== char run: %s modules, P=%s, period=%sns ====\n' "$#" "$PAR" "$PERIOD" >> "$CHAR/SUMMARY.txt"
printf '%s\n' "$@" | xargs -P "$PAR" -I{} bash -c 'run_one "$@"' _ {}
echo "==== ALL DONE ===="
sort -t' ' -k1 "$CHAR/SUMMARY.txt"
