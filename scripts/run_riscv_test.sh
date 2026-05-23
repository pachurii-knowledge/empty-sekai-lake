#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ELF="${1:?usage: run_riscv_test.sh ELF [OUTPUT_DIR]}"
OUTPUT="${2:-$ROOT/output/riscv-tests/$(basename "$ELF" .elf)}"
SIM="$ROOT/output/simulation"
TIMEOUT="${NIIGO_TEST_TIMEOUT:-120}"
BOOTSTRAP="${NIIGO_BOOTSTRAP:-0}"

mkdir -p "$OUTPUT"
python3 "$ROOT/scripts/load_elf_mem.py" "$ELF" -o "$OUTPUT" \
  $( [[ "$BOOTSTRAP" == "0" ]] && echo --no-bootstrap )

if [[ ! -x "$SIM/verilator_obj/Vtop" ]]; then
  echo "error: Verilator executable missing; run 'make OOO=1 verilator-build' first" >&2
  exit 2
fi

(
  cd "$OUTPUT"
  timeout "$TIMEOUT" "$SIM/verilator_obj/Vtop"
) > "$OUTPUT/sim.log" 2>&1 || {
  code=$?
  if [[ $code -eq 124 ]]; then
    echo "RVCP-SUMMARY: TEST FAILED - Test File \"$(basename "$ELF")\" (timeout)" >&2
    exit 1
  fi
}

if grep -q "ECALL invoked with halt argument" "$OUTPUT/sim.log"; then
  echo "RVCP-SUMMARY: TEST PASSED - Test File \"$(basename "$ELF")\""
  exit 0
fi

if [[ -f "$OUTPUT/simulation.reg" ]] &&
   grep -qE 'x10\s+\(a0\)\s+= 0x0000000a \(10\)' "$OUTPUT/simulation.reg"; then
  echo "RVCP-SUMMARY: TEST PASSED - Test File \"$(basename "$ELF")\""
  exit 0
fi

if [[ -f "$OUTPUT/simulation.reg" ]] &&
   grep -qE 'x10\s+\(a0\)\s+= 0x0000000b \(11\)' "$OUTPUT/simulation.reg"; then
  echo "RVCP-SUMMARY: TEST FAILED - Test File \"$(basename "$ELF")\" (self-check fail)" >&2
  exit 1
fi

if grep -q "Illegal instruction encountered" "$OUTPUT/sim.log"; then
  echo "RVCP-SUMMARY: TEST FAILED - Test File \"$(basename "$ELF")\" (illegal instruction)" >&2
elif grep -q "Memory exception" "$OUTPUT/sim.log"; then
  echo "RVCP-SUMMARY: TEST FAILED - Test File \"$(basename "$ELF")\" (memory exception)" >&2
else
  echo "RVCP-SUMMARY: TEST FAILED - Test File \"$(basename "$ELF")\" (did not halt cleanly)" >&2
fi
exit 1
