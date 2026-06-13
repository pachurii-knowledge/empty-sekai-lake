# Out-of-context synthesis of the full SoC (niigo_soc) -- FB2 full-SoC.
# Driven by fpga/synth/run_soc.sh. xcku11p is an installed UltraScale+ stand-in
# for the AWS F2 VU47P (same architecture family).
set part xcku11p-ffva1156-2-e
set period_ns 8.0
set root [file normalize [file dirname [info script]]/../..]

# FPGA_BUILD makes niigo_memsys expose its AXI master (no sim shim/monitor);
# AXI_MEMSYS instantiates the NMI->AXI4 bridge; SYNTHESIS drops sim-only code;
# SIMULATION_18447 is left undefined so the lab perf/monitor blocks are excluded.
set_property -name "verilog_define" \
  -value {RV64 OOO_4WIDE L1_CACHES L1D_CACHE AXI_MEMSYS FPGA_BUILD {LAB_18447="4b"} SYNTHESIS} \
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

puts "==================== SYNTH niigo_soc ===================="
if {[catch {synth_design -top niigo_soc -part $part -mode out_of_context \
      -flatten_hierarchy rebuilt} e]} {
  puts "SOC_SYNTH_FAIL: $e"
  exit 1
}
create_clock -name clk -period $period_ns [get_ports clk]
puts "---- UTIL ----"
report_utilization
puts "---- TIMING ----"
report_timing_summary -setup -max_paths 1
puts "SOC_SYNTH_DONE"
exit 0
