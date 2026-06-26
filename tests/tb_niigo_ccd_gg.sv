// tb_niigo_ccd_gg.sv -- M3b: the SAME two-core MOESI coherence program as tb_niigo_ccd_m1,
// but run through niigo_ccd_wheel (the M1 agents + directory talking over the real M3 wheel NoC
// + 128b multi-flit SerDes instead of the direct full-line star). If the S1-S6 checks stay green,
// the fabric is transparent to the coherence protocol (the load-bearing M3 integration check).
// Build/run: make ccd-wheel-coh-test
`include "niigo_mem.vh"
`include "niigo_cmi.vh"
`include "niigo_ccd_m1.vh"
`default_nettype none
module tb_niigo_ccd_gg
    import RISCV_ISA::XLEN;
    import RISCV_UArch::MEMORY_ADDR_WIDTH;
    import NIIGO_Mem::*;
    import NIIGO_CMI::*;
    import NIIGO_CCD_M1::*;
;
    logic clk=0, rst_l=0;
    always #5 clk=~clk;

    nmi_req_t mreq; logic mreq_ready; nmi_resp_t mresp;

    logic        creq_v   [2];  l1_core_op_e creq_op [2]; l1_amo_op_e creq_amo[2];
    logic [MEMORY_ADDR_WIDTH-1:0] creq_wa[2]; logic [XLEN-1:0] creq_wd[2];
    logic        creq_rdy [2];  logic [XLEN-1:0] cresp_rd[2]; logic cresp_sc[2];

    niigo_ccd_gg_direct #(.NACTIVE(2), .DIR_SETS(8), .L1_SETS(8)) dut (
        .clk, .rst_l,
        .c_req_valid(creq_v), .c_req_ready(creq_rdy), .c_req_op(creq_op), .c_req_amo(creq_amo),
        .c_req_waddr(creq_wa), .c_req_wdata(creq_wd),
        .c_resp_rdata(cresp_rd), .c_resp_sc_ok(cresp_sc),
        .mem_req_o(mreq), .mem_req_ready_i(mreq_ready), .mem_resp_i(mresp));

    // ---- behavioural NMI memory ----
    localparam int MEMLINES = 16;
    logic [LINE_BITS-1:0] MEM [MEMLINES];
    assign mreq_ready = 1'b1;
    function automatic int mline(input logic [MEMORY_ADDR_WIDTH-1:0] wa);
        mline = wa[LINE_WORD_BITS +: $clog2(MEMLINES)];
    endfunction
    always_ff @(posedge clk or negedge rst_l) begin
        if (!rst_l) mresp <= '0;
        else begin
            mresp <= '0;
            if (mreq.valid) begin
                if (mreq.op==NMI_WR_LINE) MEM[mline(mreq.waddr)] <= mreq.wdata;
                mresp.valid <= 1'b1; mresp.rdata <= MEM[mline(mreq.waddr)];
            end
        end
    end

    int errors=0;
    task automatic chk(input bit ok, input string what);
        if (!ok) begin $display("  [FAIL] %s", what); errors++; end
        else        $display("  [ ok ] %s", what);
    endtask
    task automatic chkv(input logic [XLEN-1:0] got, input logic [XLEN-1:0] exp, input string what);
        if (got!==exp) begin $display("  [FAIL] %s  (got %0d exp %0d)", what, got, exp); errors++; end
        else            $display("  [ ok ] %s = %0d", what, got);
    endtask

    task automatic cstore(input int ci, input logic [MEMORY_ADDR_WIDTH-1:0] wa, input logic [XLEN-1:0] wd);
        @(negedge clk); creq_v[ci]=1; creq_op[ci]=COP_STORE; creq_wa[ci]=wa; creq_wd[ci]=wd;
        do @(posedge clk); while(!creq_rdy[ci]); @(negedge clk); creq_v[ci]=0;
    endtask
    task automatic cload(input int ci, input logic [MEMORY_ADDR_WIDTH-1:0] wa, output logic [XLEN-1:0] rd);
        @(negedge clk); creq_v[ci]=1; creq_op[ci]=COP_LOAD; creq_wa[ci]=wa;
        do @(posedge clk); while(!creq_rdy[ci]); rd=cresp_rd[ci]; @(negedge clk); creq_v[ci]=0;
    endtask
    task automatic clr(input int ci, input logic [MEMORY_ADDR_WIDTH-1:0] wa, output logic [XLEN-1:0] rd);
        @(negedge clk); creq_v[ci]=1; creq_op[ci]=COP_LR; creq_wa[ci]=wa;
        do @(posedge clk); while(!creq_rdy[ci]); rd=cresp_rd[ci]; @(negedge clk); creq_v[ci]=0;
    endtask
    task automatic csc(input int ci, input logic [MEMORY_ADDR_WIDTH-1:0] wa, input logic [XLEN-1:0] wd,
                       output logic ok);
        @(negedge clk); creq_v[ci]=1; creq_op[ci]=COP_SC; creq_wa[ci]=wa; creq_wd[ci]=wd;
        do @(posedge clk); while(!creq_rdy[ci]); ok=cresp_sc[ci]; @(negedge clk); creq_v[ci]=0;
    endtask
    task automatic camo(input int ci, input l1_amo_op_e a, input logic [MEMORY_ADDR_WIDTH-1:0] wa,
                        input logic [XLEN-1:0] wd, output logic [XLEN-1:0] old);
        @(negedge clk); creq_v[ci]=1; creq_op[ci]=COP_AMO; creq_amo[ci]=a; creq_wa[ci]=wa; creq_wd[ci]=wd;
        do @(posedge clk); while(!creq_rdy[ci]); old=cresp_rd[ci]; @(negedge clk); creq_v[ci]=0;
    endtask

    localparam logic [MEMORY_ADDR_WIDTH-1:0] WV='h10, WW='h11, WZ='h40, WZ2='hC0;
    logic [XLEN-1:0] r; logic ok;

    initial begin
        for (int i=0;i<MEMLINES;i++) MEM[i]='0;
        for (int c=0;c<2;c++) begin creq_v[c]=0; creq_op[c]=COP_LOAD; creq_amo[c]=AMO_ADD; creq_wa[c]='0; creq_wd[c]='0; end
        rst_l=0; repeat(4) @(posedge clk); rst_l=1; repeat(2) @(posedge clk);

        $display("== S1: core0 store V=100, core1 load V (cross-core coherence over the grant-and-go direct interconnect) ==");
        cstore(0, WV, 100);
        cload (1, WV, r); chkv(r, 100, "S1: core1 sees core0's write");

        $display("== S2: core1 store V=200, core0 load V ==");
        cstore(1, WV, 200);
        cload (0, WV, r); chkv(r, 200, "S2: core0 sees core1's write");

        $display("== S3: shared counter via AMO (atomicity across cores) ==");
        camo(0, AMO_ADD, WV, 5, r); chkv(r, 200, "S3: core0 amoadd returns old 200");
        camo(1, AMO_ADD, WV, 3, r); chkv(r, 205, "S3: core1 amoadd returns old 205");
        cload(0, WV, r);            chkv(r, 208, "S3: final V = 200+5+3");

        $display("== S4: false sharing (V & W in one line, written by different cores) ==");
        cstore(0, WV, 1);
        cstore(1, WW, 2);
        cload (0, WW, r); chkv(r, 2, "S4: core0 reads W written by core1");
        cload (1, WV, r); chkv(r, 1, "S4: core1 reads V (line integrity preserved)");

        $display("== S5: LR/SC reservation + coherence-kill ==");
        clr  (0, WV, r);
        cstore(1, WV, 99);
        csc  (0, WV, 88, ok); chk(ok==1'b0, "S5: contended SC FAILS");
        cload(0, WV, r);      chkv(r, 99, "S5: V holds core1's 99, not 88");
        clr  (0, WV, r);
        csc  (0, WV, 77, ok); chk(ok==1'b1, "S5: uncontended SC succeeds");
        cload(1, WV, r);      chkv(r, 77, "S5: SC wrote 77, visible to core1");

        $display("== S6: conflict eviction + writeback + refill (Z and Z2 share a set) ==");
        cstore(0, WZ, 11);
        cstore(0, WZ2, 22);
        cload (0, WZ, r);  chkv(r, 11, "S6: core0 re-fetches Z after eviction");
        cload (1, WZ2, r); chkv(r, 22, "S6: core1 reads Z2 from core0");

        $display("");
        if (errors==0) $display("==== tb_niigo_ccd_gg: ALL CHECKS PASSED ====");
        else           $display("==== tb_niigo_ccd_gg: %0d CHECK(S) FAILED ====", errors);
        $finish;
    end

    initial begin repeat(40000) @(posedge clk); $display("WATCHDOG TIMEOUT"); $finish; end
endmodule
`default_nettype wire
