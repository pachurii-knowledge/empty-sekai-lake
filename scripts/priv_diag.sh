#!/usr/bin/env bash
# Debug helper for privileged arch-tests. Runs the niigo selfcheck ELF and, on a
# self-check failure, decodes the framework's failure_scratch region (failing
# instruction / address / bad value / expected value) by dumping that memory
# region via the testbench +sig_* signature dumper.
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ELF="${1:?usage: priv_diag.sh ELF [OUTDIR]}"
OUT="${2:-/tmp/priv_diag/$(basename "$ELF" .elf)}"
VTOP="$ROOT/output/simulation/verilator_obj/Vtop"
mkdir -p "$OUT"

sym() { readelf -s "$ELF" 2>/dev/null | awk -v n="$1" '$8==n{print $2; exit}'; }
base=$(sym failure_type)
end=$(printf "%08x" $((16#$base + 0x128)))

python3 "$ROOT/scripts/load_elf_mem.py" "$ELF" -o "$OUT" --no-bootstrap >/dev/null 2>&1
( cd "$OUT" && timeout "${NIIGO_TEST_TIMEOUT:-150}" "$VTOP" \
    +sig_begin="$base" +sig_end="$end" +sig_out="$OUT/diag.sig" > sim.log 2>&1 )

a0=$(grep -oE 'x10 .a0.\s+= 0x[0-9a-f]+' "$OUT/simulation.reg" 2>/dev/null | head -1)
w() { local off=$(( (16#$1 - 16#$base)/4 )); sed -n "$((off+1))p" "$OUT/diag.sig"; }
echo "=== $(basename "$ELF" .elf)  [$a0] ==="
echo "  failure_type   = $(w "$(sym failure_type)")"
echo "  failing_instr  = $(w "$(sym failing_instruction)")"
echo "  failing_reg    = $(w "$(sym failing_reg)")"
echo "  failing_addr   = $(w "$(sym failing_addr)")"
echo "  bad_value      = $(w "$(sym failing_value)")"
echo "  expected_value = $(w "$(sym expected_value)")"
