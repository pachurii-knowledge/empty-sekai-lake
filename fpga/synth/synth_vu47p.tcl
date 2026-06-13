# FB2b timing: synthesize niigo_soc on the ACTUAL AWS F2 part (xcvu47p, the VU47P)
# and report timing after synth, opt_design, and place_design. OOC synth alone is
# unplaced -- its WNS is dominated by high-fanout route estimates (e.g. abort_mask,
# fanout ~400K). opt_design replicates high-fanout drivers and place_design gives
# real net delays, so the post-place WNS is the representative number. The design
# (407K LUT after the predictor RAM flatten) fits the VU47P comfortably (~31%).
# Driven by run_soc.sh-style read list. Run ALONE (no parallel Verilator) -- OOM.
set part xcvu47p-fsvh2892-2-e
set root /home/mizuki/Desktop/workspace/niigo-lake
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

puts "==== POST-SYNTH UTIL ===="
report_utilization
puts "==== POST-SYNTH WNS ===="
report_timing_summary -setup -max_paths 1

if {[catch {opt_design} e]} { puts "OPT_FAIL: $e" }
puts "==== POST-OPT WNS ===="
report_timing_summary -setup -max_paths 1

# -directive Quick: fast, timing-driven enough to rank the critical paths but
# skips the aggressive post-place physical synthesis (LUT-break/rewire) that the
# default directive spends ~30 min on trying to rescue paths that are physically
# unmeetable while we are still tens of ns from closure. Good for relative WNS
# deltas in the flatten loop; do one default-directive + route_design run for the
# true number once a change lands near 0.
if {[catch {place_design -directive Quick} e]} { puts "PLACE_FAIL: $e" }
puts "==== POST-PLACE WNS ===="
report_timing_summary -setup -max_paths 1

# Emit/refresh the placed reference checkpoint for the fast incremental loop
# (synth_vu47p_incr.tcl reads it back with read_checkpoint -incremental). This
# fresh run is the source-of-truth WNS; the incremental runs anchor to this DCP.
if {[info exists ::env(REF_OUT)]} {
  write_checkpoint -force $::env(REF_OUT)
  puts "REF_WRITTEN: $::env(REF_OUT)"
}
puts "VU47P_DONE"
exit 0
