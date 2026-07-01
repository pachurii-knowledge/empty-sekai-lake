# flow_asap7.tcl — Tier-1 (all-standard-cell, flop-mapped) OpenROAD bring-up of niigo_soc
# on ASAP7 7.5-track, RVT, TT corner, **1x scale**.
#
#   openroad flow_asap7.tcl
#   STOP_AFTER=synth|floorplan|place|cts|route  (default route)
#
# STATUS: staged, grounded in verified command names/paths, NOT yet run end-to-end (pending
# the OpenROAD build). The synthesis half (STOP_AFTER=synth) is self-contained. The P&R half
# needs the ASAP7 platform RC/tracks/pdn values — lift them from OpenROAD-flow-scripts
# flow/platforms/asap7/ (see SCOPING.md §6). Placeholders below are flagged ### PLATFORM ###.

set ROOT  [file normalize [file dirname [info script]]/../..]
set HERE  [file normalize [file dirname [info script]]]
set ASAP7 /home/mizuki/Desktop/workspace/asap7
set BUILD [expr {[info exists ::env(BUILD_DIR)] ? $::env(BUILD_DIR) : "$ROOT/output/openroad"}]
set STOP  [expr {[info exists ::env(STOP_AFTER)] ? $::env(STOP_AFTER) : "route"}]
file mkdir $BUILD
proc done {s} { puts "==== STAGE DONE: $s ====" }

# ---------------------------------------------------------------- libraries (1x, RVT, TT)
# tech LEF FIRST (cells inherit its DBU); then RVT std cells. Add the SRAM LEF for Tier-2.
read_liberty $HERE/lib_tt/asap7sc7p5t_SIMPLE_RVT_TT_nldm_211120.lib
read_liberty $HERE/lib_tt/asap7sc7p5t_INVBUF_RVT_TT_nldm_220122.lib
read_liberty $HERE/lib_tt/asap7sc7p5t_AO_RVT_TT_nldm_211120.lib
read_liberty $HERE/lib_tt/asap7sc7p5t_OA_RVT_TT_nldm_211120.lib
read_liberty $HERE/lib_tt/asap7sc7p5t_SEQ_RVT_TT_nldm_220123.lib
read_lef     $ASAP7/asap7sc7p5t_28/techlef_misc/asap7_tech_1x_201209.lef
read_lef     $ASAP7/asap7sc7p5t_28/LEF/asap7sc7p5t_28_R_1x_220121a.lef
done libs

# ---------------------------------------------------------------- synthesis (integrated syn)
exec bash $HERE/gen_filelist.sh $BUILD
# --single-unit + --allow-use-before-declare are REQUIRED for this tree (see SCOPING.md
# §10 "Validation findings"): packages live in `included headers, and there are benign
# forward references that Vivado/Verilator accept.
sv_elaborate -f $BUILD/design.f --top niigo_soc --single-unit --allow-use-before-declare
synthesize
source $HERE/dont_use.tcl
report_design_area
write_verilog $BUILD/niigo_soc.synth.v
write_db      $BUILD/niigo_soc.synth.odb
done synth
if {$STOP eq "synth"} { exit 0 }

# ---------------------------------------------------------------- constraints + RC
read_sdc $HERE/constraints.sdc
### PLATFORM ### — per-layer R/C is NOT in the tech LEF; without it timing/repair is garbage.
### Replace these two lines with the ORFS asap7 setRC.tcl (set_layer_rc per layer).
set_wire_rc -signal -layer M3
set_wire_rc -clock  -layer M5

# ---------------------------------------------------------------- floorplan
### PLATFORM ### — for an SRAM-macro run add `-additional_sites coreSite` AND first define
### coreSite (undefined in the PDK — see SCOPING.md §6).
initialize_floorplan -site asap7sc7p5t -utilization 40 -aspect_ratio 1.0 -core_space 2.0
make_tracks
place_pins -hor_layers M4 -ver_layers M5
done floorplan
if {$STOP eq "floorplan"} { write_def $BUILD/niigo_soc.fp.def; exit 0 }

# ---------------------------------------------------------------- power + taps
### PLATFORM ### — tapcell masters + pdn grid come from the ORFS asap7 platform.
catch { tapcell -distance 25 -tapcell_master TAPCELL_ASAP7_75t_R -endcap_master TAPCELL_ASAP7_75t_R }
catch { pdngen }
done power

# ---------------------------------------------------------------- global place + pre-CTS opt
global_placement -density 0.55
estimate_parasitics -placement
buffer_ports
repair_design
detailed_placement
done place
if {$STOP eq "place"} { write_def $BUILD/niigo_soc.place.def; exit 0 }

# ---------------------------------------------------------------- CTS + post-CTS opt
clock_tree_synthesis -root_buf BUFx4_ASAP7_75t_R -buf_list {BUFx2_ASAP7_75t_R BUFx4_ASAP7_75t_R}
set_propagated_clock [all_clocks]
estimate_parasitics -placement
repair_timing
detailed_placement
done cts
if {$STOP eq "cts"} { write_def $BUILD/niigo_soc.cts.def; exit 0 }

# ---------------------------------------------------------------- route
global_route
estimate_parasitics -global_routing
repair_design
detailed_route
filler_placement {FILLER_ASAP7_75t_R FILLERxp5_ASAP7_75t_R}
check_placement
done route

# ---------------------------------------------------------------- outputs
write_def     $BUILD/niigo_soc.routed.def
write_db      $BUILD/niigo_soc.routed.odb
write_verilog $BUILD/niigo_soc.routed.v
report_worst_slack
report_design_area
# GDS: no Tcl write_gds in this build — use odb Python (odb.write_gds) or KLayout def2stream.py.
puts "==== FLOW COMPLETE -> $BUILD ===="
