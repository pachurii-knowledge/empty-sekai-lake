# FB2b wall #2 closure: the TRUE routed number (vs place_design Quick estimate).
# Every prior triage WNS is either Quick-place ESTIMATE or loop-corrupted. After
# the 2-stage ROB commit (ac8e6fc) the DAG is clean (UNOPTFLAT 0), so default
# place + phys_opt (high-fanout driver replication) + route_design gives the real
# net delays on the 75%-route wall-#2 cone. A moderate global fanout limit forces
# replication of the abort_mask/branch_mask/head_q/wakeup broadcast nets up front.
# Run ALONE (no parallel Verilator compile -- OOM).
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

# -fanout_limit 64: synth replicates drivers of nets exceeding 64 loads (the
# abort_mask/branch_mask/head_q/wakeup broadcast cones that dominate the 75% route).
if {[catch {synth_design -top niigo_soc -part $part -mode out_of_context \
      -flatten_hierarchy rebuilt -fanout_limit 64} e]} { puts "VU47P_FAIL: $e"; exit 1 }
create_clock -name clk -period 8.0 [get_ports clk]

puts "==== POST-SYNTH WNS ===="
report_timing_summary -setup -max_paths 1

set_param pwropt.maxFaninFanoutToNetRatio 1000000000
if {[catch {opt_design} e]} { puts "OPT_FAIL: $e" }

# Default directive = full timing-driven placement + post-place phys synthesis.
if {[catch {place_design} e]} { puts "PLACE_FAIL: $e" }
puts "==== POST-PLACE(default) WNS ===="
report_timing_summary -setup -max_paths 1

# phys_opt: replicate high-fanout drivers + reroute the critical cone.
if {[catch {phys_opt_design} e]} { puts "PHYSOPT_FAIL: $e" }
puts "==== POST-PHYSOPT WNS ===="
report_timing_summary -setup -max_paths 1

# route_design: the TRUE net delays (the number the triage has never had). May be
# slow at large negative slack; the post-route summary is the source of truth.
if {[catch {route_design} e]} { puts "ROUTE_FAIL: $e" }
puts "==== POST-ROUTE WNS (TRUE) ===="
report_timing_summary -setup -max_paths 2

if {[info exists ::env(REF_OUT)]} {
  write_checkpoint -force $::env(REF_OUT); puts "REF_WRITTEN: $::env(REF_OUT)"
}
puts "VU47P_ROUTED_DONE"
exit 0
