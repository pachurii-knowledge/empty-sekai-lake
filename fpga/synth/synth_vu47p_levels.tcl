# FB2b fast-iteration MEASUREMENT: synth + opt_design + report_timing on the VU47P,
# NO place_design. The metric that drives the flatten loop is worst-path LOGIC
# LEVELS + endpoint module -- these come from synth/opt and are independent of
# placement directive and of the STA noise from the core's pre-existing
# combinational loops, so they are the robust signal for "did this edit reduce
# depth". Route-estimated delay is reported too but is NOT the metric. ~3-4 min.
# Full place (synth_vu47p.tcl, Quick) is run only periodically for a WNS number.
# Run ALONE (no concurrent Verilator compile / second Vivado) -- OOM.
set part xcvu47p-fsvh2892-2-e
set root /home/mizuki/Desktop/workspace/niigo-lake
set out  $::env(LEVELS_OUT)
set_property -name "verilog_define" \
  -value {RV64 OOO_4WIDE L1_CACHES L1D_CACHE AXI_MEMSYS FPGA_BUILD {LAB_18447="4b"} SYNTHESIS} \
  -objects [current_fileset]
source $::env(SYNTH_READ)
set_property include_dirs [list \
  $root/src $root/src/mem $root/src/common_cells \
  $root/src/cvfpu/src $root/src/cvfpu/vendor \
  $root/src/cvfpu/vendor/openc910/C910_RTL_FACTORY/gen_rtl/vfdsu/rtl \
  $root/src/cvfpu/vendor/opene906/E906_RTL_FACTORY/gen_rtl/clk/rtl \
  $root/src/cvfpu/vendor/opene906/E906_RTL_FACTORY/gen_rtl/fdsu/rtl \
  $root/src/cvfpu/vendor/opene906/E906_RTL_FACTORY/gen_rtl/fpu/rtl ] [current_fileset]

if {[catch {synth_design -top niigo_soc -part $part -mode out_of_context \
      -flatten_hierarchy rebuilt} e]} { puts "VU47P_FAIL: $e"; exit 1 }
create_clock -name clk -period 8.0 [get_ports clk]
if {[catch {opt_design} e]} { puts "OPT_FAIL: $e" }

puts "==== POST-OPT WNS (route-estimated, NOT the metric) ===="
report_timing_summary -setup -max_paths 1
# The metric: worst 8 setup paths, full cell detail, to a file.
report_timing -setup -nworst 1 -max_paths 8 -input_pins -file $out/levels_paths.rpt
set wp [get_timing_paths -setup -nworst 1 -max_paths 1]
puts "WORST_LOGIC_LEVELS = [get_property LOGIC_LEVELS $wp]"
puts "VU47P_LEVELS_DONE"
exit 0
