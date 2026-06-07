#!/usr/bin/env bash
# Sync the niigo ACT DUT config between the gitignored references/riscv-tests
# checkout (where build_riscv_tests.py reads it, and where you edit it) and the
# version-controlled act-config/ copy.
#
#   default        : checkout -> act-config   (run this before committing)
#   --to-checkout  : act-config -> checkout   (restore the tracked config into a
#                                              fresh/regenerated checkout)
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
TRACKED="$ROOT/act-config/niigo-lake/niigo-rv32g"
CHECKOUT="$ROOT/references/riscv-tests/config/cores/niigo-lake/niigo-rv32g"

FILES=(link.ld niigo-rv32g.yaml riscv_arch_test.h run_cmd.txt rvmodel_macros.h sail.json test_config.yaml)

if [[ "${1:-}" == "--to-checkout" ]]; then
    src="$TRACKED"; dst="$CHECKOUT"; dir="act-config -> checkout (restore)"
else
    src="$CHECKOUT"; dst="$TRACKED"; dir="checkout -> act-config"
fi

[[ -d "$src" ]] || { echo "source not found: $src" >&2; exit 1; }
mkdir -p "$dst"
for f in "${FILES[@]}"; do
    if [[ -f "$src/$f" ]]; then cp "$src/$f" "$dst/$f"; fi
done
echo "synced niigo ACT config ($dir)"
