#!/usr/bin/env bash
# Run one or more priv ACT suite groups against the OOO build, categorising each
# result as PASS / SELFCHECK / TIMEOUT / ILLEGAL / OTHER. A per-test timeout
# bounds hangs so the sweep always terminates. Usage:
#   scripts/run_priv_suites.sh [TIMEOUT_SECONDS] GROUP [GROUP...]
# GROUP names are directories under work/niigo-rv32g/elfs/priv/.
set -u
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ELFDIR="$ROOT/references/riscv-tests/work/niigo-rv32g/elfs/priv"
TO="${1:?usage: run_priv_suites.sh TIMEOUT GROUP...}"; shift
pass=0; sc=0; to=0; ill=0; oth=0
for g in "$@"; do
  for elf in "$ELFDIR/$g"/*.elf; do
    [ -f "$elf" ] || continue
    name="$(basename "$elf")"
    out="$ROOT/output/act-$g/$(basename "$elf" .elf)"
    summary=$(NIIGO_BOOTSTRAP=1 NIIGO_TEST_TIMEOUT="$TO" \
      "$ROOT/scripts/run_riscv_test.sh" "$elf" "$out" 2>&1 | sed -n 's/RVCP-SUMMARY: //p')
    case "$summary" in
      *PASSED*)          printf '  PASS      %s\n' "$name"; pass=$((pass+1));;
      *"(timeout)"*)     printf '  TIMEOUT   %s\n' "$name"; to=$((to+1));;
      *"self-check"*)    printf '  SELFCHECK %s\n' "$name"; sc=$((sc+1));;
      *"illegal"*)       printf '  ILLEGAL   %s\n' "$name"; ill=$((ill+1));;
      *)                 printf '  OTHER     %s :: %s\n' "$name" "$summary"; oth=$((oth+1));;
    esac
  done
done
echo "----"
printf 'TOTAL: pass=%d selfcheck=%d timeout=%d illegal=%d other=%d\n' \
  "$pass" "$sc" "$to" "$ill" "$oth"
