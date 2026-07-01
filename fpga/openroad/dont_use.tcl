# dont_use.tcl — ASAP7 7.5-track cells the mapper must not pick.
# Physical-only (fill/decap/tap/tie), all latches (integrated `syn` can't map latches anyway),
# clock-gates, and async-reset/scan flops (plain flop-mapped bring-up). Keep DFF*QN* flops.
#
# NOTE: this is a reasonable starting set grounded in the asap7sc7p5t_28 collateral. For a
# production run, prefer the canonical dont_use list from OpenROAD-flow-scripts
# flow/platforms/asap7/ (it is the validated reference). Glob names assume the *_ASAP7_75t_*
# cell naming in the R/L/SL LEF.

set_dont_use {
  *FILLER*  *FILLERxp5*
  *DECAPx*
  TAPCELL_*  *TAPCELL_WITH_FILLER*
  TIEHIx1_*  TIELOx1_*
  DHLx*  DLLx*
  ICGx*
  *DFFASRHQN*  *DFFASRNQ*
  SDF*
}
