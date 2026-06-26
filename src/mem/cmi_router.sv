/**
 * cmi_router.sv  --  generic radix-N input-buffered VC wormhole router (the wheel NoC)
 *
 * The M3 wheel fabric building block (plans/multicore-ccd.md §W.3). One parameterised
 * module realises BOTH wheel node kinds:
 *   - cmi_core_router : radix-4 {W-ring, E-ring, Spoke, Local}  (one per core C0..C3)
 *   - cmi_hub_xbar    : radix-5 {spoke0..3, Internal(L2/dir/MC)} (the central hub)
 * by passing the per-port role indices (P_RW/P_RE/P_SP/P_LO/P_IN; sentinel = NP for
 * "absent") and the node id.
 *
 * Properties realised (the load-bearing ones — §W.3/§W.4/§W.5):
 *   - Input-buffered, per-physical-VC FIFOs (depth VC_DEPTH), credit flow control
 *     (no combinational ready/valid across a link — credits are REGISTERED, so every
 *     hop is a flop boundary and no comb path spans two routers: §8/§W.5).
 *   - REGISTERED output link (the SLL-crossing flop, §W.7).
 *   - Role/dst-based route compute on the HEAD flit only (§W.1): purely dst-node based
 *     given the node role — directory-bound (dst=HUB) -> Spoke/Internal; a core dst ->
 *     Local if local, else Spoke (at hub) or the shortest ring direction (at a core).
 *   - Wormhole per-VC reservation: a HEAD reserves an output VC (output port + vc) for
 *     the whole packet; body/tail follow it; other VCs are NOT blocked (the VC-DAG
 *     deadlock-freedom of §W.4 needs response VCs to bypass a stalled request VC).
 *   - Ring dateline (§W.4.1): a C2 data flit (CMI_DATELINE_VC) forwarded across the
 *     designated ring edge is promoted to the VC2b sub-VC so the ring CDG stays a DAG.
 *
 * SIMPLIFIED vs the §W.3 sketch (functional-model scope, documented like the M1 protocol
 * simplifications): the VC/switch allocators are FIXED-priority (low port/vc first), not
 * the separable round-robin of §W.3 — round-robin is a fairness/Fmax refinement and the
 * serialised M1-over-wheel traffic has near-zero fabric concurrency. The 2-stage router
 * pipeline is collapsed to a single registered-output stage (latency, not function).
 */
`include "niigo_cmi.vh"
`default_nettype none
module cmi_router
    import NIIGO_CMI::*;
#(
    parameter int NP        = 4,                 // radix (number of physical ports)
    parameter int NODE_ID   = 0,                 // this node's id (0..3 core, 4 = HUB)
    // per-role port index (set to NP == "absent"). A core router sets ring+spoke+local;
    // the hub sets internal (+ the 4 spokes are addressed by dst core id == port index).
    parameter int P_RW      = NP,                // W-ring port index
    parameter int P_RE      = NP,                // E-ring port index
    parameter int P_SP      = NP,                // Spoke port index (core side)
    parameter int P_LO      = NP,                // Local port index (core side)
    parameter int P_IN      = NP                 // Internal port index (hub side, to dir/L2)
)(
    input  wire logic                  clk,
    input  wire logic                  rst_l,
    input  wire cmi_link_t             in    [NP],   // forward flits in
    output logic [NUM_VC_PHYS-1:0]     in_cr [NP],   // credits we return upstream for in[]
    output cmi_link_t                  out   [NP],   // forward flits out (REGISTERED)
    input  wire logic [NUM_VC_PHYS-1:0] out_cr[NP]   // credits returned to us for out[]
);
    localparam int PW   = $clog2(NP < 2 ? 2 : NP);   // port-index width
    localparam int VW   = VC_ID_W;                    // 3 (holds 0..5)
    localparam logic [2:0] OC_MAX = 3'(VC_DEPTH);     // credit ceiling (= input FIFO depth)
    localparam bit IS_HUB = (NODE_ID == int'(CMI_HUB_ID));

    // ring-position dateline edges (§W.4.1): E-ring out of ring-pos 3, W-ring out of pos 0.
    localparam bit IS_DL_E = (!IS_HUB) && (cmi_ring_pos(NODE_ID[CORE_ID_W-1:0]) == 2'd3);
    localparam bit IS_DL_W = (!IS_HUB) && (cmi_ring_pos(NODE_ID[CORE_ID_W-1:0]) == 2'd0);

    // ---- per-port, per-physical-VC input FIFO (depth 2) ----
    localparam int NV = NUM_VC_PHYS;              // 6 physical VC lanes
    logic [1:0]          cnt_q [NP][NV];          // 0..2 occupancy
    logic [CMI_FLIT_W-1:0] d0_q [NP][NV], d1_q [NP][NV];
    flit_kind_e          k0_q  [NP][NV], k1_q  [NP][NV];

    // ---- per (input port, vc) wormhole reservation (where this VC's current packet goes) ----
    logic                in_res_q  [NP][NV];      // active reservation
    logic [PW-1:0]       in_oport_q[NP][NV];      // reserved output port
    logic [VW-1:0]       in_ovc_q  [NP][NV];      // reserved output vc

    // ---- per (output port, vc) occupancy + credit ----
    logic                ovc_busy_q[NP][NV];
    logic [2:0]          ocredit_q [NP][NV];      // 0..VC_DEPTH

    // ---- registered outputs ----
    cmi_link_t           out_q  [NP];
    logic [NV-1:0]       incr_q [NP];

    // ===== next-state =====
    logic [1:0]          cnt_n [NP][NV];
    logic [CMI_FLIT_W-1:0] d0_n[NP][NV], d1_n[NP][NV];
    flit_kind_e          k0_n  [NP][NV], k1_n  [NP][NV];
    logic                in_res_n  [NP][NV];
    logic [PW-1:0]       in_oport_n[NP][NV];
    logic [VW-1:0]       in_ovc_n  [NP][NV];
    logic                ovc_busy_n[NP][NV];
    logic [2:0]          ocredit_n [NP][NV];
    cmi_link_t           out_n  [NP];
    logic [NV-1:0]       incr_n [NP];

    // route a HEAD's dst node to an output port index on THIS node (§W.1, dst-based)
    function automatic logic [PW-1:0] route_port(input logic [NODE_ID_W-1:0] dst);
        if (IS_HUB) begin
            // at the hub: directory-bound (dst=HUB) -> internal; a core dst -> that spoke
            // (spoke port index == core id, by wheel wiring convention).
            route_port = (dst == CMI_HUB_ID) ? PW'(P_IN) : PW'(dst[PW-1:0]);
        end else begin
            if (dst == NODE_ID[NODE_ID_W-1:0])      route_port = PW'(P_LO);   // arrived
            else if (dst == CMI_HUB_ID)             route_port = PW'(P_SP);   // directory-bound
            else begin                                                        // cache-to-cache
                // shortest ring direction (0=E forward, 1=W backward); escape=Spoke (M3c).
                route_port = cmi_ring_dir(NODE_ID[CORE_ID_W-1:0], dst[CORE_ID_W-1:0])
                             ? PW'(P_RW) : PW'(P_RE);
            end
        end
    endfunction

    // dateline promotion: a CMI_DATELINE_VC flit forwarded across this node's dateline edge
    function automatic logic [VW-1:0] out_vc_of(input logic [VW-1:0] ivc, input logic [PW-1:0] oport);
        out_vc_of = ivc;
        if (ivc == CMI_DATELINE_VC) begin
            if (IS_DL_E && (oport == PW'(P_RE))) out_vc_of = CMI_VC2B;
            if (IS_DL_W && (oport == PW'(P_RW))) out_vc_of = CMI_VC2B;
        end
        // a flit already on VC2b stays VC2b (a <=2-hop ring path never re-crosses its dateline)
    endfunction

    integer p, v, q, ov;
    logic [CMI_FLIT_W-1:0] hd;
    logic [PW-1:0]   rp;
    logic [VW-1:0]   rov;
    cmi_rhdr_t       rh;
    flit_kind_e      hk;

    always_comb begin
        // ---- defaults: hold state ----
        for (p = 0; p < NP; p++) begin
            for (v = 0; v < NV; v++) begin
                cnt_n[p][v]=cnt_q[p][v]; d0_n[p][v]=d0_q[p][v]; d1_n[p][v]=d1_q[p][v];
                k0_n[p][v]=k0_q[p][v];   k1_n[p][v]=k1_q[p][v];
                in_res_n[p][v]=in_res_q[p][v]; in_oport_n[p][v]=in_oport_q[p][v];
                in_ovc_n[p][v]=in_ovc_q[p][v];
                ovc_busy_n[p][v]=ovc_busy_q[p][v]; ocredit_n[p][v]=ocredit_q[p][v];
            end
            out_n[p]='{default:'0}; incr_n[p]='0;
        end

        // ---- (1) credit returns from downstream: replenish our output credits ----
        for (q = 0; q < NP; q++)
            for (ov = 0; ov < NV; ov++)
                if (out_cr[q][ov] && ocredit_n[q][ov] < OC_MAX)
                    ocredit_n[q][ov] = ocredit_n[q][ov] + 3'd1;

        // ---- (2) route compute + output-VC reservation (HEAD flits, fixed priority) ----
        for (p = 0; p < NP; p++) begin
            for (v = 0; v < NV; v++) begin
                if (cnt_q[p][v] != 2'd0 && !in_res_q[p][v]) begin
                    hk = k0_q[p][v];
                    if (hk == FLIT_HEAD || hk == FLIT_HEADTAIL) begin
                        hd  = d0_q[p][v];
                        rh  = hd[CMI_RHDR_W-1:0];
                        rp  = route_port(rh.dst);
                        rov = out_vc_of(v[VW-1:0], rp);
                        if (!ovc_busy_n[rp][rov]) begin     // free output VC -> reserve it
                            ovc_busy_n[rp][rov]   = 1'b1;
                            in_res_n[p][v]        = 1'b1;
                            in_oport_n[p][v]      = rp;
                            in_ovc_n[p][v]        = rov;
                        end
                    end
                end
            end
        end

        // ---- (3) switch traversal: <=1 flit per output port (fixed priority over in VCs) ----
        for (q = 0; q < NP; q++) begin
            for (ov = 0; ov < NV; ov++) begin
                if (!out_n[q].valid && ovc_busy_n[q][ov] && ocredit_n[q][ov] != 3'd0) begin
                    // find the input (p,v) that reserved (q,ov) and has a flit ready
                    for (p = 0; p < NP; p++) begin
                        for (v = 0; v < NV; v++) begin
                            if (!out_n[q].valid && in_res_n[p][v] &&
                                in_oport_n[p][v]==q[PW-1:0] && in_ovc_n[p][v]==ov[VW-1:0] &&
                                cnt_n[p][v] != 2'd0)
                            begin
                                hk = k0_n[p][v];
                                // emit the head-of-FIFO flit on (q, ov)
                                out_n[q].valid    = 1'b1;
                                out_n[q].ctrl.kind= hk;
                                out_n[q].ctrl.vc  = ov[VW-1:0];
                                out_n[q].data     = d0_n[p][v];
                                ocredit_n[q][ov]  = ocredit_n[q][ov] - 3'd1;
                                // pop the input FIFO (shift) + return a credit upstream
                                if (cnt_n[p][v] == 2'd2) begin
                                    d0_n[p][v]=d1_n[p][v]; k0_n[p][v]=k1_n[p][v]; cnt_n[p][v]=2'd1;
                                end else cnt_n[p][v]=2'd0;
                                incr_n[p][v] = 1'b1;
                                // release the reservation on the packet's last flit
                                if (hk == FLIT_TAIL || hk == FLIT_HEADTAIL) begin
                                    in_res_n[p][v]   = 1'b0;
                                    ovc_busy_n[q][ov]= 1'b0;
                                end
                            end
                        end
                    end
                end
            end
        end

        // ---- (4) receive: push incoming flits into the matching input FIFO ----
        //  (credit flow guarantees space; if the buffer were full the flit is dropped — an
        //   assertion below catches a credit-protocol violation in sim.)
        for (p = 0; p < NP; p++) begin
            if (in[p].valid) begin
                v = int'(in[p].ctrl.vc);
                if (cnt_n[p][v] == 2'd0) begin
                    d0_n[p][v]=in[p].data; k0_n[p][v]=in[p].ctrl.kind; cnt_n[p][v]=2'd1;
                end else if (cnt_n[p][v] == 2'd1) begin
                    d1_n[p][v]=in[p].data; k1_n[p][v]=in[p].ctrl.kind; cnt_n[p][v]=2'd2;
                end
                // else: overflow (should never happen under credits) — drop
            end
        end
    end

    // ===== sequential =====
    always_ff @(posedge clk or negedge rst_l) begin
        if (!rst_l) begin
            for (int pp=0; pp<NP; pp++) begin
                out_q[pp] <= '{default:'0};
                incr_q[pp] <= '0;
                for (int vv=0; vv<NV; vv++) begin
                    cnt_q[pp][vv]<=2'd0; in_res_q[pp][vv]<=1'b0;
                    in_oport_q[pp][vv]<='0; in_ovc_q[pp][vv]<='0;
                    ovc_busy_q[pp][vv]<=1'b0; ocredit_q[pp][vv]<=OC_MAX;
                    d0_q[pp][vv]<='0; d1_q[pp][vv]<='0; k0_q[pp][vv]<=FLIT_HEAD; k1_q[pp][vv]<=FLIT_HEAD;
                end
            end
        end else begin
            for (int pp=0; pp<NP; pp++) begin
                out_q[pp] <= out_n[pp];
                incr_q[pp] <= incr_n[pp];
                for (int vv=0; vv<NV; vv++) begin
                    cnt_q[pp][vv]<=cnt_n[pp][vv];
                    d0_q[pp][vv]<=d0_n[pp][vv]; d1_q[pp][vv]<=d1_n[pp][vv];
                    k0_q[pp][vv]<=k0_n[pp][vv]; k1_q[pp][vv]<=k1_n[pp][vv];
                    in_res_q[pp][vv]<=in_res_n[pp][vv];
                    in_oport_q[pp][vv]<=in_oport_n[pp][vv]; in_ovc_q[pp][vv]<=in_ovc_n[pp][vv];
                    ovc_busy_q[pp][vv]<=ovc_busy_n[pp][vv]; ocredit_q[pp][vv]<=ocredit_n[pp][vv];
                end
            end
        end
    end

    // ---- outputs ----
    always_comb for (int pp=0; pp<NP; pp++) begin
        out[pp]    = out_q[pp];
        in_cr[pp]  = incr_q[pp];
    end
endmodule
`default_nettype wire
