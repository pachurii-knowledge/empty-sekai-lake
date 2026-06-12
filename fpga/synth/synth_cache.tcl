# Out-of-context synthesis of the niigo memory subsystem (phase FB2).
#
# Validates that the new L1I / L1D / NMI arbiter / AXI bridge RTL synthesizes
# for an UltraScale+ target (the AWS F2 VU47P is the same architecture family;
# xcku11p is used here as an installed stand-in), that the sync-read tag/data
# arrays infer block RAM, and reports utilization + timing at a conservative
# 125 MHz. Run:
#   vivado -mode batch -source fpga/synth/synth_cache.tcl
set part xcku11p-ffva1156-2-e
set period_ns 8.0
set root [pwd]
set incdirs [list $root/src $root/src/mem $root/fpga/synth]

read_verilog -sv [list \
  $root/fpga/synth/defines.vh \
  $root/src/riscv_isa.vh \
  $root/src/riscv_uarch.vh \
  $root/src/mem/niigo_mem.vh \
  $root/src/mem/l1_plru.sv \
  $root/src/mem/l1_tag_array.sv \
  $root/src/mem/l1_data_array.sv \
  $root/src/mem/l1_icache.sv \
  $root/src/mem/l1_dcache.sv \
  $root/src/mem/nmi_arbiter.sv \
  $root/src/mem/nmi_axi_bridge.sv ]
set_property include_dirs $incdirs [current_fileset]
set_property verilog_define {RV64 LAB_18447="4b" L1_CACHES L1D_CACHE SYNTHESIS} [current_fileset]

foreach top {l1_icache l1_dcache nmi_axi_bridge} {
  puts "==================== SYNTH $top ===================="
  synth_design -top $top -part $part -mode out_of_context -flatten_hierarchy rebuilt
  create_clock -name clk -period $period_ns [get_ports clk]
  puts "---- UTIL $top ----"
  report_utilization
  puts "---- TIMING $top ----"
  report_timing_summary -delay_type max -setup -max_paths 1 -no_header
}
puts "SYNTH_CACHE_DONE"
exit
