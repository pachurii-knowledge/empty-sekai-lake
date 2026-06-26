/**
 * cmi_wheel.sv  --  the wheel NoC fabric (4 core routers in a ring + 1 radix-5 hub)
 *
 * Assembles the M3 wheel (plans/multicore-ccd.md §W.1/§1): 4 cmi_router instances as the
 * radix-4 core routers wired into the bidirectional ring C0-C1-C3-C2-C0, plus one radix-5
 * cmi_router as the central hub crossbar (4 spoke ports + 1 internal L2/dir/MC port).
 *
 * Port map per core router: 0=W-ring, 1=E-ring, 2=Spoke, 3=Local.
 * Hub router ports:         0..3 = spoke to core 0..3 (spoke port index == core id),  4 = Internal.
 *
 * Exposed endpoints: per-core Local flit ports (the L1 agent attaches here via the M3b SerDes)
 * and the hub Internal flit port (the directory attaches here). Credits run reverse alongside
 * every link; the whole fabric is registered at every hop (§W.7) — the credit lanes are
 * NUM_VC_PHYS wide (the 5 logical VCs + the ring VC2b dateline sub-VC, §W.4.1).
 */
`include "niigo_cmi.vh"
`default_nettype none
module cmi_wheel
    import NIIGO_CMI::*;
(
    input  wire logic              clk,
    input  wire logic              rst_l,
    // ---- per-core Local endpoints (L1 agent side) ----
    input  wire cmi_link_t             loc_in    [NUM_CORES],   // endpoint -> core Local
    output logic [NUM_VC_PHYS-1:0]     loc_in_cr [NUM_CORES],   // core -> endpoint credit (for loc_in)
    output cmi_link_t                  loc_out   [NUM_CORES],   // core Local -> endpoint
    input  wire logic [NUM_VC_PHYS-1:0] loc_out_cr[NUM_CORES],  // endpoint -> core credit (for loc_out)
    // ---- hub Internal endpoint (directory / L2 / MC side) ----
    input  wire cmi_link_t             int_in,                  // dir -> hub Internal
    output logic [NUM_VC_PHYS-1:0]     int_in_cr,               // hub -> dir credit (for int_in)
    output cmi_link_t                  int_out,                 // hub Internal -> dir
    input  wire logic [NUM_VC_PHYS-1:0] int_out_cr              // dir -> hub credit (for int_out)
);
    localparam int NV = NUM_VC_PHYS;

    // per-core router links (index [core][port], port 0=W,1=E,2=Spoke,3=Local)
    cmi_link_t          ci  [NUM_CORES][4];   // into router
    logic [NV-1:0]      cicr[NUM_CORES][4];   // credit out of router (returned upstream)
    cmi_link_t          co  [NUM_CORES][4];   // out of router
    logic [NV-1:0]      cocr[NUM_CORES][4];   // credit into router (from downstream)

    // hub router links (port 0..3 = spoke to core 0..3, port 4 = internal)
    cmi_link_t          hi  [5];
    logic [NV-1:0]      hicr[5];
    cmi_link_t          ho  [5];
    logic [NV-1:0]      hocr[5];

    genvar gc;
    generate
        for (gc = 0; gc < NUM_CORES; gc++) begin : G_CORE
            cmi_router #(.NP(4), .NODE_ID(gc),
                         .P_RW(0), .P_RE(1), .P_SP(2), .P_LO(3), .P_IN(4))
            RT (.clk, .rst_l, .in(ci[gc]), .in_cr(cicr[gc]), .out(co[gc]), .out_cr(cocr[gc]));

            // ---- ring + spoke + local input links into this core router ----
            assign ci[gc][0] = co[cmi_ring_w(gc[CORE_ID_W-1:0])][1];  // W-in  <- west neighbour E-out
            assign ci[gc][1] = co[cmi_ring_e(gc[CORE_ID_W-1:0])][0];  // E-in  <- east neighbour W-out
            assign ci[gc][2] = ho[gc];                                // Spoke-in <- hub spoke-out
            assign ci[gc][3] = loc_in[gc];                            // Local-in <- endpoint
            // ---- credits INTO this core router (out_cr) returned by the downstream of its outputs ----
            assign cocr[gc][0] = cicr[cmi_ring_w(gc[CORE_ID_W-1:0])][1]; // W-out credit <- west nbr E-in
            assign cocr[gc][1] = cicr[cmi_ring_e(gc[CORE_ID_W-1:0])][0]; // E-out credit <- east nbr W-in
            assign cocr[gc][2] = hicr[gc];                              // Spoke-out credit <- hub
            assign cocr[gc][3] = loc_out_cr[gc];                        // Local-out credit <- endpoint
            // ---- endpoint-facing outputs ----
            assign loc_out[gc]   = co[gc][3];
            assign loc_in_cr[gc] = cicr[gc][3];
            // ---- spoke wiring to the hub ----
            assign hi[gc]   = co[gc][2];     // hub spoke-in  <- core Spoke-out
            assign hocr[gc] = cicr[gc][2];   // hub spoke-out credit <- core Spoke-in
        end
    endgenerate

    // ---- hub (radix-5: 4 spokes + internal) ----
    cmi_router #(.NP(5), .NODE_ID(int'(CMI_HUB_ID)),
                 .P_RW(5), .P_RE(5), .P_SP(5), .P_LO(5), .P_IN(4))
    HUB (.clk, .rst_l, .in(hi), .in_cr(hicr), .out(ho), .out_cr(hocr));

    // ---- hub internal endpoint (directory) ----
    assign hi[4]      = int_in;
    assign int_in_cr  = hicr[4];
    assign int_out    = ho[4];
    assign hocr[4]    = int_out_cr;
endmodule
`default_nettype wire
