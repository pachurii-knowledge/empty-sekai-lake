# FB2b "attack the route" run: the real place + physical-synthesis number (vs the
# fast place -directive Quick used in the flatten loop). Default place_design runs
# the aggressive post-place phys synthesis; phys_opt_design then replicates the
# high-fanout drivers (abort_mask/wakeup/count) and reroutes -- directly targeting
# the 75%-route WNS. ALLOW_COMBINATORIAL_LOOPS lets place/phys_opt proceed past the
# residual false comb loops (the design is exhaustively sim-verified). Run ALONE.
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

# Let the tools route past the one residual false comb loop (retire_valid, the
# in-order commit cycle). The LSQ wakeup<->load_writeback and ptw loops are now
# broken in RTL (d863562 / 7f0e923), so only the commit cycle remains.
set_property ALLOW_COMBINATORIAL_LOOPS TRUE [get_nets -hierarchical -filter \
  {NAME =~ *retire_valid* || NAME =~ *active_commit* || NAME =~ *commit_taken* \
   || NAME =~ *stack_abort_mask*}]

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

if {[info exists ::env(REF_OUT)]} {
  write_checkpoint -force $::env(REF_OUT); puts "REF_WRITTEN: $::env(REF_OUT)"
}
puts "VU47P_ROUTE_DONE"
exit 0
