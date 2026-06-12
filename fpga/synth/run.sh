#!/usr/bin/env bash
# FB2: out-of-context Vivado synthesis of the niigo cache subsystem.
#
# Until the source is moved off `default_nettype none` + bare `input logic`
# (Vivado 8-6735, see OVERNIGHT_BUGLOG / plans/fpga-memsys.md), this wrapper
# preprocesses copies (`default_nettype none` -> `default_nettype wire`) into a
# build dir and runs synth_cache.tcl against them. Source Vivado settings64.sh
# first, then: bash fpga/synth/run.sh
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
BUILD="${BUILD_DIR:-$ROOT/output/synth/cache}"
rm -rf "$BUILD"; mkdir -p "$BUILD"

FILES=(
  src/riscv_isa.vh src/riscv_uarch.vh src/mem/niigo_mem.vh fpga/synth/defines.vh
  src/mem/l1_plru.sv src/mem/l1_tag_array.sv src/mem/l1_data_array.sv
  src/mem/l1_icache.sv src/mem/l1_dcache.sv src/mem/nmi_arbiter.sv src/mem/nmi_axi_bridge.sv
)
for f in "${FILES[@]}"; do
  sed 's/`default_nettype none/`default_nettype wire/' "$ROOT/$f" > "$BUILD/$(basename "$f")"
done

command -v vivado >/dev/null || { echo "vivado not on PATH (source settings64.sh)"; exit 1; }
SYNTH_SRC="$BUILD" vivado -mode batch -nojournal -nolog \
  -source "$ROOT/fpga/synth/synth_cache.tcl" | tee "$BUILD/synth.log"
echo "report: $BUILD/synth.log"
