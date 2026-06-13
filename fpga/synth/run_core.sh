#!/usr/bin/env bash
# FB2 full-core: out-of-context Vivado synthesis of the OoO core (riscv_core_ooo).
#
# The L1 caches + niigo_memsys live in the testbench, so the OoO core's OOC
# boundary is its handshaked memory ports -- src/mem/ is NOT part of this synth
# (the caches are validated separately by run.sh / synth_cache.tcl). After the
# FB2 source hygiene (explicit `input wire` on ports; Internal_Defines packaged),
# the niigo RTL synthesizes directly; only the two vendored common_cells files
# (lzc.sv, rr_arb_tree.sv) still use `default_nettype none` + bare `input logic`
# and are preprocessed (none -> wire) into the build dir. CVFPU is read directly.
#
# Source Vivado settings64.sh first, then: bash fpga/synth/run_core.sh
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
BUILD="${BUILD_DIR:-$ROOT/output/synth/core}"
rm -rf "$BUILD"; mkdir -p "$BUILD"
cd "$ROOT"

# Sim-only / lab / alt-top files excluded from the OoO-core synth.
EXCLUDE='testbench.sv|main_memory.sv|sram_simulation.sv|cache.sv|cache_new.sv|register_file.sv|riscv_core_timing.sv'

# Vendored common_cells used by the core (rr_arb_tree, lzc): preprocess nettype.
for f in lzc.sv rr_arb_tree.sv; do
  sed 's/`default_nettype none/`default_nettype wire/' "src/common_cells/$f" > "$BUILD/$f"
done

# Build the Vivado read list: CVFPU (.v vendor then .sv, in dependency order),
# then the core .sv (src/*.sv minus excludes, NOT src/mem/), then common_cells.
READ="$BUILD/read.tcl"; : > "$READ"
while read -r f; do
  case "$f" in *.v) echo "read_verilog   $ROOT/$f" ;; *) echo "read_verilog -sv $ROOT/$f" ;; esac
done < /home/mizuki/.claude/jobs/6f5774ee/tmp/cvfpu_list.txt >> "$READ"
for f in src/*.sv; do
  [[ "$(basename "$f")" =~ ^($EXCLUDE)$ ]] && continue
  echo "read_verilog -sv $ROOT/$f" >> "$READ"
done
echo "read_verilog -sv $BUILD/lzc.sv"         >> "$READ"
echo "read_verilog -sv $BUILD/rr_arb_tree.sv" >> "$READ"
echo "read list: $(grep -c read_verilog "$READ") files"

command -v vivado >/dev/null || { echo "vivado not on PATH (source settings64.sh)"; exit 1; }
SYNTH_READ="$READ" vivado -mode batch -nojournal -nolog \
  -source "$ROOT/fpga/synth/synth_core.tcl" | tee "$BUILD/synth.log"
echo "report: $BUILD/synth.log"
