#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ELF="${1:?usage: run_riscv_test.sh ELF [OUTPUT_DIR]}"
OUTPUT="${2:-$ROOT/output/riscv-tests/$(basename "$ELF" .elf)}"
SIM="$ROOT/output/simulation"
# VTOP: override the simulator binary (default = the in-tree build). The parallel
# verification flows point every run at a private, immutable copy so a stray
# `make` in one worker can't clobber the shared binary mid-run.
VTOP_BIN="${VTOP:-$SIM/verilator_obj/Vtop}"
TIMEOUT="${NIIGO_TEST_TIMEOUT:-120}"
BOOTSTRAP="${NIIGO_BOOTSTRAP:-0}"

mkdir -p "$OUTPUT"
python3 "$ROOT/scripts/load_elf_mem.py" "$ELF" -o "$OUTPUT" \
  $( [[ "$BOOTSTRAP" == "0" ]] && echo --no-bootstrap )

if [[ ! -x "$VTOP_BIN" ]]; then
  echo "error: Verilator executable missing ($VTOP_BIN); run 'make OOO=1 verilator-build' first" >&2
  exit 2
fi

(
  cd "$OUTPUT"
  # NIIGO_VTOP_ARGS: extra plusargs forwarded to the simulator (e.g.
  # "+mem_fuzz +mem_seed=2026" for the niigo_memsys latency fuzzer).
  # shellcheck disable=SC2086
  timeout "$TIMEOUT" "$VTOP_BIN" ${NIIGO_VTOP_ARGS:-}
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

# The register dump prints a0 at the build's XLEN width (8 hex digits on
# RV32, 16 on RV64) -- match any number of leading zeros.
if [[ -f "$OUTPUT/simulation.reg" ]] &&
   grep -qE 'x10\s+\(a0\)\s+= 0x0+a \(10\)' "$OUTPUT/simulation.reg"; then
  echo "RVCP-SUMMARY: TEST PASSED - Test File \"$(basename "$ELF")\""
  exit 0
fi

if [[ -f "$OUTPUT/simulation.reg" ]] &&
   grep -qE 'x10\s+\(a0\)\s+= 0x0+b \(11\)' "$OUTPUT/simulation.reg"; then
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
