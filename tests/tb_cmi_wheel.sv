// tb_cmi_wheel.sv -- flit-level test of the M3 wheel NoC fabric (cmi_wheel = 4 core routers
// in the ring C0-C1-C3-C2 + radix-5 hub). Drives raw flits at the per-core Local endpoints
// and the hub Internal endpoint, an infinite sink that returns a credit per received flit,
// and checks: spoke up (core->hub), spoke down (hub->core), ring adjacent (1 hop) + opposite
// (2 hops, dateline VC2->VC2b promotion), hub 4->1 arbitration, and multi-flit (head+4 body)
// wormhole reassembly (contiguous, in order, VC preserved).
// Build/run: make ccd-wheel-test
`include "niigo_cmi.vh"
`default_nettype none
module tb_cmi_wheel
    import NIIGO_CMI::*;
;
    localparam int NV = NUM_VC_PHYS;
    localparam int VW = VC_ID_W;
    logic clk=0, rst_l=0;
    always #5 clk=~clk;

    cmi_link_t          loc_in    [NUM_CORES];
    logic [NV-1:0]      loc_in_cr [NUM_CORES];
    cmi_link_t          loc_out   [NUM_CORES];
    logic [NV-1:0]      loc_out_cr[NUM_CORES];
    cmi_link_t          int_in;
    logic [NV-1:0]      int_in_cr;
    cmi_link_t          int_out;
    logic [NV-1:0]      int_out_cr;

    cmi_wheel dut (.clk, .rst_l,
        .loc_in, .loc_in_cr, .loc_out, .loc_out_cr,
        .int_in, .int_in_cr, .int_out, .int_out_cr);

    // ---- infinite sink: return one credit for every flit observed at an endpoint ----
    always_comb begin
        for (int c=0;c<NUM_CORES;c++)
            loc_out_cr[c] = loc_out[c].valid ? (NV'(1) << loc_out[c].ctrl.vc) : '0;
        int_out_cr = int_out.valid ? (NV'(1) << int_out.ctrl.vc) : '0;
    end

    // ---- capture: endpoint 0..3 = core Local out, 4 = hub Internal out ----
    localparam int CAP = 64;
    logic [CMI_FLIT_W-1:0] rxd [5][CAP];
    logic [VW-1:0]         rxv [5][CAP];
    flit_kind_e            rxk [5][CAP];
    int                    rxn [5];
    always_ff @(posedge clk or negedge rst_l) begin
        if (!rst_l) begin for (int e=0;e<5;e++) rxn[e]<=0; end
        else begin
            for (int c=0;c<NUM_CORES;c++) if (loc_out[c].valid) begin
                rxd[c][rxn[c]]<=loc_out[c].data; rxv[c][rxn[c]]<=loc_out[c].ctrl.vc;
                rxk[c][rxn[c]]<=loc_out[c].ctrl.kind; rxn[c]<=rxn[c]+1;
            end
            if (int_out.valid) begin
                rxd[4][rxn[4]]<=int_out.data; rxv[4][rxn[4]]<=int_out.ctrl.vc;
                rxk[4][rxn[4]]<=int_out.ctrl.kind; rxn[4]<=rxn[4]+1;
            end
        end
    end

    // ---- flit builders ----
    function automatic logic [CMI_FLIT_W-1:0] mk_head(input logic [NODE_ID_W-1:0] dst,
            input logic [CORE_ID_W-1:0] src, input cmi_class_e mclass, input logic [31:0] tag);
        cmi_rhdr_t rh; logic [CMI_FLIT_W-1:0] d;
        rh.mclass=mclass; rh.src_core=src; rh.dst=dst;
        d = (CMI_FLIT_W'(tag) << CMI_RHDR_W);
        d[CMI_RHDR_W-1:0] = rh;
        return d;
    endfunction

    // ---- paced flit driver (1 flit then 2 idle cycles -> never overflows the depth-2 input
    //      buffer that drains ~1 flit / 2 cycles; the infinite sink prevents downstream backup) ----
    task automatic tx(input int port, input logic [VW-1:0] vc,
                      input flit_kind_e kind, input logic [CMI_FLIT_W-1:0] data);
        @(negedge clk);
        if (port < NUM_CORES) begin
            loc_in[port].valid<=1'b1; loc_in[port].ctrl.kind<=kind;
            loc_in[port].ctrl.vc<=vc; loc_in[port].data<=data;
        end else begin
            int_in.valid<=1'b1; int_in.ctrl.kind<=kind; int_in.ctrl.vc<=vc; int_in.data<=data;
        end
        @(negedge clk);
        if (port < NUM_CORES) loc_in[port].valid<=1'b0; else int_in.valid<=1'b0;
        @(negedge clk); @(negedge clk);
    endtask

    int errors=0;
    int t6base, t7b0;   // module-scope (a declaration-initializer inside `initial` is static @t0)
    task automatic chk(input bit ok, input string what);
        if (!ok) begin $display("  [FAIL] %s", what); errors++; end
        else        $display("  [ ok ] %s", what);
    endtask

    logic [VW-1:0] vc_c0 = VW'(C0_REQ);   // 0
    logic [VW-1:0] vc_c1 = VW'(C1_FWD);   // 1
    logic [VW-1:0] vc_c2 = VW'(C2_DATA);  // 2
    logic [VW-1:0] vc_c4 = VW'(cmi_vc(C4_ACK)); // 3

    initial begin
        for (int c=0;c<NUM_CORES;c++) loc_in[c]='{default:'0};
        int_in='{default:'0};
        rst_l=0; repeat(4) @(posedge clk); rst_l=1; repeat(2) @(posedge clk);

        // ---- T1: spoke up — core0 -> HUB (C0 request) ----
        $display("== T1: spoke up (core0 -> hub) ==");
        tx(0, vc_c0, FLIT_HEADTAIL, mk_head(CMI_HUB_ID, 2'd0, C0_REQ, 32'hA1));
        repeat(20) @(posedge clk);
        chk(rxn[4]==1 && rxv[4][0]==vc_c0 &&
            rxd[4][0][CMI_FLIT_W-1:CMI_RHDR_W]==(CMI_FLIT_W-CMI_RHDR_W)'(32'hA1),
            "T1: hub got core0's request on VC0, tag intact");

        // ---- T2: spoke down — HUB -> core2 (C1 fwd) ----
        $display("== T2: spoke down (hub -> core2) ==");
        tx(4, vc_c1, FLIT_HEADTAIL, mk_head(cmi_core_node(2'd2), 2'd0, C1_FWD, 32'hB2));
        repeat(20) @(posedge clk);
        chk(rxn[2]==1 && rxv[2][0]==vc_c1 &&
            rxd[2][0][CMI_FLIT_W-1:CMI_RHDR_W]==(CMI_FLIT_W-CMI_RHDR_W)'(32'hB2),
            "T2: core2 got hub's C1 fwd on VC1");

        // ---- T3: ring adjacent — core0 -> core1 (C2, 1 ring hop, no dateline) ----
        $display("== T3: ring adjacent (core0 -> core1, C2) ==");
        tx(0, vc_c2, FLIT_HEADTAIL, mk_head(cmi_core_node(2'd1), 2'd0, C2_DATA, 32'hC3));
        repeat(20) @(posedge clk);
        chk(rxn[1]==1 && rxv[1][0]==vc_c2 &&
            rxd[1][0][CMI_FLIT_W-1:CMI_RHDR_W]==(CMI_FLIT_W-CMI_RHDR_W)'(32'hC3),
            "T3: core1 got core0's C2 on VC2 (no promotion)");

        // ---- T4: ring opposite/dateline — core2 -> core0 (C2, E-ring wrap = dateline) ----
        $display("== T4: ring dateline (core2 -> core0, C2 across the dateline) ==");
        tx(2, vc_c2, FLIT_HEADTAIL, mk_head(cmi_core_node(2'd0), 2'd2, C2_DATA, 32'hD4));
        repeat(20) @(posedge clk);
        chk(rxn[0]==1 && rxv[0][0]==CMI_VC2B &&
            rxd[0][0][CMI_FLIT_W-1:CMI_RHDR_W]==(CMI_FLIT_W-CMI_RHDR_W)'(32'hD4),
            "T4: core0 got core2's C2 promoted to VC2b across the dateline");

        // ---- T5: hub 4->1 arbitration — core1 and core3 both -> HUB at once ----
        $display("== T5: hub arbitration (core1 & core3 -> hub) ==");
        fork
            tx(1, vc_c0, FLIT_HEADTAIL, mk_head(CMI_HUB_ID, 2'd1, C0_REQ, 32'h51));
            tx(3, vc_c0, FLIT_HEADTAIL, mk_head(CMI_HUB_ID, 2'd3, C0_REQ, 32'h53));
        join
        repeat(30) @(posedge clk);
        chk(rxn[4]==3, "T5: hub received both requests (total 3 incl T1)");

        // ---- T6: multi-flit wormhole — core0 sends HEAD+4 BODY to core1 (C2) ----
        $display("== T6: multi-flit reassembly (core0 -> core1, head + 4 body) ==");
        t6base = rxn[1];
        tx(0, vc_c2, FLIT_HEAD, mk_head(cmi_core_node(2'd1), 2'd0, C2_DATA, 32'h600));
        tx(0, vc_c2, FLIT_BODY, 128'h1111_1111_1111_1111_1111_1111_1111_1111);
        tx(0, vc_c2, FLIT_BODY, 128'h2222_2222_2222_2222_2222_2222_2222_2222);
        tx(0, vc_c2, FLIT_BODY, 128'h3333_3333_3333_3333_3333_3333_3333_3333);
        tx(0, vc_c2, FLIT_TAIL, 128'h4444_4444_4444_4444_4444_4444_4444_4444);
        repeat(30) @(posedge clk);
        chk(rxn[1]-t6base==5, "T6: all 5 flits arrived at core1");
        chk(rxk[1][t6base]==FLIT_HEAD && rxk[1][t6base+4]==FLIT_TAIL, "T6: head first, tail last");
        chk(rxd[1][t6base+1]==128'h1111_1111_1111_1111_1111_1111_1111_1111 &&
            rxd[1][t6base+2]==128'h2222_2222_2222_2222_2222_2222_2222_2222 &&
            rxd[1][t6base+3]==128'h3333_3333_3333_3333_3333_3333_3333_3333 &&
            rxd[1][t6base+4]==128'h4444_4444_4444_4444_4444_4444_4444_4444,
            "T6: body flits in order, contiguous");

        // ---- T7: non-C2 peer-dst (InvAck class) takes the SPOKE->hub->spoke path, not the ring ----
        $display("== T7: peer-dst C4 ack (core2 -> core0) routes via spoke/hub, not the ring ==");
        t7b0 = rxn[0];
        tx(2, vc_c4, FLIT_HEADTAIL, mk_head(cmi_core_node(2'd0), 2'd2, C4_ACK, 32'h7A));
        repeat(40) @(posedge clk);
        chk(rxn[0]-t7b0==1 && rxv[0][t7b0]==vc_c4 &&
            rxd[0][t7b0][CMI_FLIT_W-1:CMI_RHDR_W]==(CMI_FLIT_W-CMI_RHDR_W)'(32'h7A),
            "T7: core0 got core2's C4 ack via spoke->hub->spoke (VC3)");

        $display("");
        if (errors==0) $display("==== tb_cmi_wheel: ALL CHECKS PASSED ====");
        else           $display("==== tb_cmi_wheel: %0d CHECK(S) FAILED ====", errors);
        $finish;
    end

    initial begin repeat(4000) @(posedge clk); $display("WATCHDOG TIMEOUT"); $finish; end
endmodule
`default_nettype wire
