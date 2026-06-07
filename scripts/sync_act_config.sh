#!/usr/bin/env bash
# Sync the version-controlled niigo ACT DUT config (act-config/) into the
# gitignored references/riscv-tests checkout, which is where build_riscv_tests.py
# reads it from. Run this after editing act-config/ (or pass --from-checkout to
# pull edits made directly in the checkout back into act-config/).
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
TRACKED="$ROOT/act-config/niigo-lake/niigo-rv32g"
CHECKOUT="$ROOT/references/riscv-tests/config/cores/niigo-lake/niigo-rv32g"

FILES=(link.ld niigo-rv32g.yaml riscv_arch_test.h run_cmd.txt rvmodel_macros.h sail.json test_config.yaml)

if [[ "${1:-}" == "--from-checkout" ]]; then
    src="$CHECKOUT"; dst="$TRACKED"; dir="checkout -> act-config"
else
    src="$TRACKED"; dst="$CHECKOUT"; dir="act-config -> checkout"
fi

[[ -d "$src" ]] || { echo "source not found: $src" >&2; exit 1; }
mkdir -p "$dst"
for f in "${FILES[@]}"; do
    if [[ -f "$src/$f" ]]; then cp "$src/$f" "$dst/$f"; fi
done
echo "synced niigo ACT config ($dir)"
