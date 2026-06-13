# FB2b fast-iteration: INCREMENTAL synth+place of niigo_soc on the VU47P. After
# a fresh synth of the (edited) RTL, it reuses a reference PLACED checkpoint
# (REF_DCP, produced by synth_vu47p.tcl with REF_OUT set) so place_design only
# re-places the cells that changed -- much faster than from-scratch when the edit
# touches only the LSQ/IQ and the rest (caches, CVFPU, rename/commit) is intact.
#
# Use this for quick per-edit WNS checks during the flatten loop. The reuse keys
# on cell-name matching, so flatten-rebuilt renaming on a big edit can lower
# reuse (report_incremental_reuse prints the %). Periodically re-baseline with a
# fresh run_vu47p.sh (source-of-truth WNS + refreshes the reference).
# Run ALONE (no concurrent Verilator compile / second Vivado) -- OOM.
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

if {[catch {opt_design} e]} { puts "OPT_FAIL: $e" }

# Anchor to the reference placement, then place incrementally. Fall back to a
# full place if no reference exists yet (first run of a phase).
if {[info exists ::env(REF_DCP)] && [file exists $::env(REF_DCP)]} {
  read_checkpoint -incremental $::env(REF_DCP)
  puts "INCR_REF: $::env(REF_DCP)"
  if {[catch {place_design} e]} { puts "PLACE_FAIL: $e" }
  puts "==== INCREMENTAL REUSE ===="
  report_incremental_reuse
} else {
  puts "INCR_REF: NONE -- full place"
  if {[catch {place_design} e]} { puts "PLACE_FAIL: $e" }
}
puts "==== POST-PLACE WNS (incremental) ===="
report_timing_summary -setup -max_paths 1
puts "VU47P_DONE"
exit 0
