#!/usr/bin/env bash
# Sync the niigo ACT DUT configs between the gitignored references/riscv-tests
# checkout (where build_riscv_tests.py reads them, and where you edit them) and
# the version-controlled act-config/ copies.
#
#   default        : checkout -> act-config   (run this before committing)
#   --to-checkout  : act-config -> checkout   (restore the tracked configs into
#                                              a fresh/regenerated checkout)
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

CONFIGS=(niigo-rv32g niigo-rv64g niigo-rv64gc)

sync_one() {
    local name="$1" direction="$2"
    local tracked="$ROOT/act-config/niigo-lake/$name"
    local checkout="$ROOT/references/riscv-tests/config/cores/niigo-lake/$name"
    local files=(link.ld "$name.yaml" riscv_arch_test.h run_cmd.txt rvmodel_macros.h sail.json test_config.yaml)
    local src dst
    if [[ "$direction" == "--to-checkout" ]]; then
        src="$tracked"; dst="$checkout"
    else
        src="$checkout"; dst="$tracked"
    fi
    if [[ ! -d "$src" ]]; then
        echo "skipped $name (source not found: $src)" >&2
        return 0
    fi
    mkdir -p "$dst"
    for f in "${files[@]}"; do
        if [[ -f "$src/$f" ]]; then cp "$src/$f" "$dst/$f"; fi
    done
    echo "synced $name"
}

dir_label="checkout -> act-config"
[[ "${1:-}" == "--to-checkout" ]] && dir_label="act-config -> checkout (restore)"
for cfg in "${CONFIGS[@]}"; do
    sync_one "$cfg" "${1:-}"
done
echo "niigo ACT config sync done ($dir_label)"
