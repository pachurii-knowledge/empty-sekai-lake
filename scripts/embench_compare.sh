#!/usr/bin/env bash
# embench_compare.sh -- A/B two Embench runs by their labels and report the
# per-benchmark + geomean speedup. Speedup = baseline_cyc / new_cyc  (>1 = the NEW
# build is faster). Reads the summary.tsv each run_embench.sh writes.
#
# Usage:
#   scripts/embench_compare.sh <BASELINE_LABEL> <NEW_LABEL>
# Example (credit the perf levers):
#   scripts/run_embench.sh <vtop_baseline> base       # RV64=1 OOO=1 RVC=1 L1D=1
#   scripts/run_embench.sh <vtop_perf>     perf       # PERF=1
#   scripts/embench_compare.sh base perf
#
# Only benchmarks that PASS in BOTH runs are compared. INSTRET should match between
# builds for the same benchmark (identical ELF) -- the report flags any mismatch,
# which would mean the two runs used different scales/binaries (not comparable).
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
A="${1:?usage: embench_compare.sh BASELINE_LABEL NEW_LABEL}"
B="${2:?missing NEW_LABEL}"
FA="$ROOT/output/embench/$A/summary.tsv"; FB="$ROOT/output/embench/$B/summary.tsv"
for f in "$FA" "$FB"; do [ -f "$f" ] || { echo "missing $f (run run_embench.sh with that label first)" >&2; exit 1; }; done

printf "%-16s %-12s %-12s %-8s %s\n" BENCH "${A}_cyc" "${B}_cyc" SPEEDUP NOTE
awk -v A="$A" -v B="$B" '
  FNR==NR { if ($2=="PASS" && $3+0>0) { ac[$1]=$3; ai[$1]=$4 } next }
  {
    if ($2=="PASS" && $3+0>0 && ($1 in ac)) {
      r = ac[$1] / $3; note = (ai[$1]!=$4) ? "INSTRET-MISMATCH!" : "";
      printf "%-16s %-12d %-12d %-8.3f %s\n", $1, ac[$1], $3, r, note;
      s += log(r); n++;
      if (r > bmax) { bmax=r; bn=$1 }
      if (mn==0 || r < mn) { mn=r; mnn=$1 }
    }
  }
  END {
    if (n>0)
      printf "---\ngeomean speedup %s/%s = %.3f over %d benches (best %s %.2fx, worst %s %.2fx)\n",
             A, B, exp(s/n), n, bn, bmax, mnn, mn;
    else
      print "no benchmarks PASS in both runs";
  }
' "$FA" "$FB"
