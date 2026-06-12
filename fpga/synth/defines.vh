// Synthesis define bundle (phase FB2). Read first so the modules see the same
// configuration the FPGA target uses: RV64G + Sv39, 64 B / 4-word memory, the
// L1I+L1D caches and the AXI bridge. SYNTHESIS gates the sim-only $fatal in the
// AXI bridge.
`define RV64
`define LAB_18447 "4b"
`define L1_CACHES
`define L1D_CACHE
`define SYNTHESIS
