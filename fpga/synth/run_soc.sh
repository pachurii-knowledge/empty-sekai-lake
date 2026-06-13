#!/usr/bin/env bash
# FB2 full-SoC: out-of-context Vivado synthesis of niigo_soc -- the OoO core PLUS
# the full memory subsystem (L1I/L1D caches + NMI bus + NMI->AXI4-512 bridge),
# exposing a single AXI4-512 master to external DRAM. Built with FPGA_BUILD (so
# niigo_memsys exposes the AXI master instead of the sim shim) + AXI_MEMSYS +
# L1_CACHES + L1D_CACHE + OOO_4WIDE + RV64.
#
# The vendored common_cells (lzc, rr_arb_tree) are preprocessed (default_nettype
# none -> wire); everything else synthesizes from source after the FB2 hygiene.
# Source Vivado settings64.sh first, then: bash fpga/synth/run_soc.sh
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
BUILD="${BUILD_DIR:-$ROOT/output/synth/soc}"
rm -rf "$BUILD"; mkdir -p "$BUILD"
cd "$ROOT"

# Excluded: sim-only/lab files, the alt cores + wrapper (niigo_soc instantiates
# riscv_core_ooo directly), and the sim-only NMI/AXI backends (the FPGA build
# uses the bridge + exposed AXI master, not the adapter/shim/monitor).
EXCLUDE='testbench.sv|main_memory.sv|sram_simulation.sv|cache.sv|cache_new.sv|register_file.sv|riscv_core_timing.sv|riscv_core.sv|riscv_core_scalar.sv|riscv_core_4wide.sv|nmi_mem_adapter.sv|axi_mem_shim.sv|axi_chk.sv'

for f in lzc.sv rr_arb_tree.sv; do
  sed 's/`default_nettype none/`default_nettype wire/' "src/common_cells/$f" > "$BUILD/$f"
done

READ="$BUILD/read.tcl"; : > "$READ"
while read -r f; do
  case "$f" in *.v) echo "read_verilog   $ROOT/$f" ;; *) echo "read_verilog -sv $ROOT/$f" ;; esac
done < /home/mizuki/.claude/jobs/6f5774ee/tmp/cvfpu_list.txt >> "$READ"
for f in src/*.sv src/mem/*.sv; do
  [[ "$(basename "$f")" =~ ^($EXCLUDE)$ ]] && continue
  echo "read_verilog -sv $ROOT/$f" >> "$READ"
done
echo "read_verilog -sv $BUILD/lzc.sv"         >> "$READ"
echo "read_verilog -sv $BUILD/rr_arb_tree.sv" >> "$READ"
echo "read list: $(grep -c read_verilog "$READ") files"

command -v vivado >/dev/null || { echo "vivado not on PATH (source settings64.sh)"; exit 1; }
SYNTH_READ="$READ" vivado -mode batch -nojournal -nolog \
  -source "$ROOT/fpga/synth/synth_soc.tcl" | tee "$BUILD/synth.log"
echo "report: $BUILD/synth.log"
