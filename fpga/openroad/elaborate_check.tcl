# elaborate_check.tcl — isolate the SV frontend: read ASAP7 1x RVT libs/LEF, then run
# yosys-slang `sv_elaborate` on the full niigo_soc tree. Confirms the frontend clears
# WITHOUT the (much heavier) ABC `synthesize` mapping. Prints ELAB_OK on success.
set ROOT  [file normalize [file dirname [info script]]/../..]
set HERE  [file normalize [file dirname [info script]]]
set ASAP7 /home/mizuki/Desktop/workspace/asap7
set BUILD [expr {[info exists ::env(BUILD_DIR)] ? $::env(BUILD_DIR) : "$ROOT/output/openroad"}]

read_liberty $HERE/lib_tt/asap7sc7p5t_SIMPLE_RVT_TT_nldm_211120.lib
read_liberty $HERE/lib_tt/asap7sc7p5t_INVBUF_RVT_TT_nldm_220122.lib
read_liberty $HERE/lib_tt/asap7sc7p5t_AO_RVT_TT_nldm_211120.lib
read_liberty $HERE/lib_tt/asap7sc7p5t_OA_RVT_TT_nldm_211120.lib
read_liberty $HERE/lib_tt/asap7sc7p5t_SEQ_RVT_TT_nldm_220123.lib
read_lef     $ASAP7/asap7sc7p5t_28/techlef_misc/asap7_tech_1x_201209.lef
read_lef     $ASAP7/asap7sc7p5t_28/LEF/asap7sc7p5t_28_R_1x_220121a.lef

# --single-unit: the package-defining .vh headers are `included into every .sv, so all
#   files must share one compilation unit (as Verilator/Vivado do) or packages duplicate.
# --allow-use-before-declare: niigo has forward refs (localparam used in a port width;
#   nets used in an `assign above their declaration) that Vivado/Verilator tolerate.
if {[catch {sv_elaborate -f $BUILD/design.f --top niigo_soc \
              --single-unit --allow-use-before-declare} err]} {
  puts "ELAB_FAIL: $err"
  exit 1
}
puts "ELAB_OK"
exit 0
