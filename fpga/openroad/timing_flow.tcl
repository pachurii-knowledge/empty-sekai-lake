# timing_flow.tcl — generic ASAP7 synth→place→STA for a parameterized top, to a Fmax report.
#
#   TOP=<module> FILELIST=<design.f> CLK_PERIOD_NS=<ns> UTIL=<pct> STOP_AFTER=synth|place \
#     openroad -exit timing_flow.tcl
#
# Reads the 5 ASAP7 RVT TT std-cell libs (flop-mapped, no SRAM macros), the matched 1x tech +
# R std-cell LEF, runs OpenROAD integrated synthesis (yosys-slang frontend + ABC map), applies
# the ORFS asap7 real per-layer RC, floorplans + globally places, then reports the worst
# register-to-register path and computes the achievable clock period / peak frequency.
#
# This is a *pre-CTS, placed-parasitic* number (signal RC estimated from placement): the
# standard altitude for a synthesis/early-floorplan Fmax estimate.

set HERE   [file normalize [file dirname [info script]]]
set PLAT   $HERE/orfs_platform/flow/platforms/asap7
proc env_or {k d} { return [expr {[info exists ::env($k)] ? $::env($k) : $d}] }
set TOP    [env_or TOP niigo_soc]
set FLIST  [env_or FILELIST $HERE/../../output/openroad/design.f]
set PERIOD [env_or CLK_PERIOD_NS 1.0]
set UTIL   [env_or UTIL 40]
set STOP   [env_or STOP_AFTER place]
set BUILD  [env_or BUILD_DIR $HERE/../../output/openroad]
file mkdir $BUILD
puts "==== timing_flow: TOP=$TOP PERIOD=$PERIOD ns UTIL=$UTIL% STOP=$STOP ===="

# ---------------------------------------------------------------- libraries (1x, RVT, TT)
foreach lib {
  asap7sc7p5t_SIMPLE_RVT_TT_nldm_211120.lib
  asap7sc7p5t_INVBUF_RVT_TT_nldm_220122.lib
  asap7sc7p5t_AO_RVT_TT_nldm_211120.lib
  asap7sc7p5t_OA_RVT_TT_nldm_211120.lib
  asap7sc7p5t_SEQ_RVT_TT_nldm_220123.lib
} { read_liberty $HERE/lib_tt/$lib }
# tech LEF FIRST (cells inherit its DBU=1x), then the matched RVT (R) std-cell LEF.
read_lef $PLAT/lef/asap7_tech_1x_201209.lef
read_lef $PLAT/lef/asap7sc7p5t_28_R_1x_220121a.lef
# Optional SRAM-macro collateral (colon-separated .lib / .lef paths) for the
# NIIGO_SRAM_MACRO memory-cell mapping. Liberty gives the macro timing/area so
# blackboxed arrays no longer flop-map; LEF gives its footprint for placement.
if {[info exists ::env(SRAM_LIB)]} {
  foreach l [split $::env(SRAM_LIB) ":"] { if {$l ne ""} { read_liberty $l; puts "  + SRAM lib $l" } }
}
if {[info exists ::env(SRAM_LEF)]} {
  foreach f [split $::env(SRAM_LEF) ":"] { if {$f ne ""} { read_lef $f; puts "  + SRAM lef $f" } }
}
puts "==== STAGE DONE: libs ===="

# ---------------------------------------------------------------- synthesis (integrated syn)
sv_elaborate -f $FLIST --top $TOP --single-unit --allow-use-before-declare
synthesize
# ASAP7 dont-use: low-drive *x1p*/*xp* cells, scan (SDF*), clock-gates (ICG*).
catch { set_dont_use {*x1p*_ASAP7* *xp*_ASAP7* SDF* ICG*} }
report_design_area
write_verilog $BUILD/$TOP.synth.v
write_db      $BUILD/$TOP.synth.odb
puts "==== STAGE DONE: synth ===="
if {$STOP eq "synth"} { exit 0 }

# ---------------------------------------------------------------- constraints + real RC
# ASAP7 Liberty time_unit is 1ps, so OpenSTA reports/consumes time in ps. The user gives the
# period in ns (readable); convert to ps for create_clock and keep all Fmax math in ps.
set PERIOD_PS [expr {$PERIOD * 1000.0}]
create_clock -name clk -period $PERIOD_PS [get_ports clk]
catch { set_false_path -from [get_ports rst_l] }
set_max_fanout 64 [current_design]
source $PLAT/setRC.tcl
puts "==== STAGE DONE: sdc+rc ===="

# ---------------------------------------------------------------- floorplan + global place
initialize_floorplan -site asap7sc7p5t -utilization $UTIL -aspect_ratio 1.0 -core_space 2.0
source $PLAT/openRoad/make_tracks.tcl
catch { place_pins -hor_layers M4 -ver_layers M5 }
global_placement -density 0.60
estimate_parasitics -placement
puts "==== STAGE DONE: place ===="

# ---------------------------------------------------------------- timing report + Fmax
puts "\n=================== TIMING REPORT ($TOP @ ${PERIOD}ns) ==================="
report_worst_slack -max
report_tns
report_checks -path_delay max -group_count 3 -fields {slew cap input_pins nets fanout} \
  -format full_clock_expanded
report_check_types -max_slew -max_capacitance -max_fanout -violators
report_design_area

# Fmax = 1 / (T_clk - WNS):  the worst reg-to-reg path delay (+setup) = T_clk - WNS. (all ps)
set wns_ps  [sta::worst_slack -max]
set crit_ps [expr {$PERIOD_PS - $wns_ps}]
set crit_ns [expr {$crit_ps / 1000.0}]
set fmax_mhz [expr {1.0e6 / $crit_ps}]
puts "\n================================================================"
puts [format "  Constraint period      : %.3f ns  (%.1f MHz target)" $PERIOD [expr {1000.0/$PERIOD}]]
puts [format "  Worst slack (WNS)      : %.3f ns  (%.1f ps)" [expr {$wns_ps/1000.0}] $wns_ps]
puts [format "  Critical path delay    : %.3f ns  (%.1f ps)" $crit_ns $crit_ps]
puts [format "  >>> PEAK FREQUENCY     : %.1f MHz  (%.3f GHz) <<<" $fmax_mhz [expr {$fmax_mhz/1000.0}]]
puts "================================================================"
exit 0
