# Out-of-context synthesis of the niigo memory subsystem (phase FB2).
#
# Driven by fpga/synth/run.sh, which preprocesses the RTL into $SYNTH_SRC (a
# build dir of `default_nettype wire` copies -- see the 8-6735 finding in
# OVERNIGHT_BUGLOG). Validates that the L1I / L1D / NMI arbiter / AXI bridge
# RTL synthesizes for an UltraScale+ target (the AWS F2 VU47P is the same
# architecture family; xcku11p is an installed stand-in), that the sync-read
# tag/data arrays infer block RAM, and reports utilization + timing at 125 MHz.
set part xcku11p-ffva1156-2-e
set period_ns 8.0
if {[info exists ::env(SYNTH_SRC)]} { set sd $::env(SYNTH_SRC) } else { set sd [pwd] }

set_property -name "verilog_define" \
  -value {RV64 {LAB_18447="4b"} L1_CACHES L1D_CACHE SYNTHESIS} -objects [current_fileset]
read_verilog -sv [list \
  $sd/defines.vh $sd/riscv_isa.vh $sd/riscv_uarch.vh $sd/niigo_mem.vh \
  $sd/l1_plru.sv $sd/l1_tag_array.sv $sd/l1_data_array.sv \
  $sd/l1_icache.sv $sd/l1_dcache.sv $sd/nmi_arbiter.sv $sd/nmi_axi_bridge.sv ]
set_property include_dirs [list $sd] [current_fileset]

foreach top {l1_icache l1_dcache nmi_axi_bridge} {
  puts "==================== SYNTH $top ===================="
  if {[catch {synth_design -top $top -part $part -mode out_of_context \
        -flatten_hierarchy rebuilt} e]} {
    puts "SYNTH_FAIL $top: $e"
    continue
  }
  create_clock -name clk -period $period_ns [get_ports clk]
  puts "---- UTIL $top ----"
  report_utilization
  puts "---- TIMING $top ----"
  report_timing_summary -setup -max_paths 1
}
puts "SYNTH_CACHE_DONE"
exit
