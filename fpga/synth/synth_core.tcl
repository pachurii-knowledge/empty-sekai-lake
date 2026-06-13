# Out-of-context synthesis of the OoO core (riscv_core_ooo) -- FB2 full-core.
# Driven by fpga/synth/run_core.sh (which writes $SYNTH_READ with the read_verilog
# commands and preprocesses the vendored common_cells). xcku11p is an installed
# UltraScale+ stand-in for the AWS F2 VU47P (same architecture family).
set part xcku11p-ffva1156-2-e
set period_ns 8.0
set root [file normalize [file dirname [info script]]/../..]

# SYNTHESIS gates the sim-only `ifndef SYNTHESIS` code; SIMULATION_18447 is left
# UNDEFINED so the lab's `ifdef SIMULATION_18447` perf/monitor blocks are excluded.
set_property -name "verilog_define" \
  -value {RV64 OOO_4WIDE L1_CACHES L1D_CACHE {LAB_18447="4b"} SYNTHESIS} \
  -objects [current_fileset]

source $::env(SYNTH_READ)

set_property include_dirs [list \
  $root/src \
  $root/src/mem \
  $root/src/common_cells \
  $root/src/cvfpu/src \
  $root/src/cvfpu/vendor \
  $root/src/cvfpu/vendor/openc910/C910_RTL_FACTORY/gen_rtl/vfdsu/rtl \
  $root/src/cvfpu/vendor/opene906/E906_RTL_FACTORY/gen_rtl/clk/rtl \
  $root/src/cvfpu/vendor/opene906/E906_RTL_FACTORY/gen_rtl/fdsu/rtl \
  $root/src/cvfpu/vendor/opene906/E906_RTL_FACTORY/gen_rtl/fpu/rtl ] [current_fileset]

puts "==================== SYNTH riscv_core_ooo ===================="
if {[catch {synth_design -top riscv_core_ooo -part $part -mode out_of_context \
      -flatten_hierarchy rebuilt} e]} {
  puts "CORE_SYNTH_FAIL: $e"
  exit 1
}
create_clock -name clk -period $period_ns [get_ports clk]
puts "---- UTIL ----"
report_utilization
puts "---- TIMING ----"
report_timing_summary -setup -max_paths 1
puts "CORE_SYNTH_DONE"
exit 0
