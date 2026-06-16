# FB2b clean-ATTRIBUTION run: synth + opt_design + report_timing with
# -flatten_hierarchy NONE so the worst path keeps true module boundaries (rebuilt
# over-shares logic across modules and MISLABELS the owning module -- the memory's
# documented trap). Use this to confirm which module/cone actually owns the -12.6
# path before any RTL cut. NO place. ~4-6 min. Run ALONE (no concurrent Verilator).
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
      -flatten_hierarchy none} e]} { puts "VU47P_FAIL: $e"; exit 1 }
create_clock -name clk -period 8.0 [get_ports clk]
if {[catch {opt_design} e]} { puts "OPT_FAIL: $e" }

puts "==== POST-OPT WNS (none-flatten, route-estimated) ===="
report_timing_summary -setup -max_paths 1
report_timing -setup -nworst 1 -max_paths 6 -input_pins -file $out/attrib_paths.rpt
set wp [get_timing_paths -setup -nworst 1 -max_paths 1]
puts "WORST_LOGIC_LEVELS = [get_property LOGIC_LEVELS $wp]"
puts "VU47P_ATTRIB_DONE"
exit 0
