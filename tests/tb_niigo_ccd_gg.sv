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
    logic [MEMORY_ADDR_WIDTH-1:0] creq_wa[2]; logic [XLEN-1:0] creq_wd[2]; logic [XLEN/8-1:0] creq_wm[2];
    logic        creq_rdy [2];  logic [XLEN-1:0] cresp_rd[2]; logic cresp_sc[2];

    // DIR_SETS >> L1_SETS so an L1 conflict-evict does not also evict the directory entry (a NINE
    // directory-capacity / backup-dir concern, OPEN-3, out of M3c scope).
    niigo_ccd_gg_direct #(.NACTIVE(2), .DIR_SETS(64), .L1_SETS(8), .RESP_DLY(4)) dut (
        .clk, .rst_l,
        .c_req_valid(creq_v), .c_req_ready(creq_rdy), .c_req_op(creq_op), .c_req_amo(creq_amo),
        .c_req_waddr(creq_wa), .c_req_wdata(creq_wd), .c_req_wmask(creq_wm),
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

    // deferred-snoop firing monitor (a snoop recorded into an agent's MSHR defer slot)
    int defer_cnt=0, c1d=0, c7d=0, c3d=0;
    always_ff @(posedge clk) if (rst_l) begin
        if (dut.G_AGENT[0].L1D.d_val_n && !dut.G_AGENT[0].L1D.d_val_q) defer_cnt<=defer_cnt+1;
        if (dut.G_AGENT[1].L1D.d_val_n && !dut.G_AGENT[1].L1D.d_val_q) defer_cnt<=defer_cnt+1;
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
        @(negedge clk); creq_v[ci]=1; creq_op[ci]=COP_STORE; creq_wa[ci]=wa; creq_wd[ci]=wd; creq_wm[ci]='1;
        do @(posedge clk); while(!creq_rdy[ci]); @(negedge clk); creq_v[ci]=0;
    endtask
    // sub-word store: only the byte-enabled lanes of `wd` are written (M3d c_req_wmask)
    task automatic cstoreb(input int ci, input logic [MEMORY_ADDR_WIDTH-1:0] wa, input logic [XLEN-1:0] wd, input logic [XLEN/8-1:0] be);
        @(negedge clk); creq_v[ci]=1; creq_op[ci]=COP_STORE; creq_wa[ci]=wa; creq_wd[ci]=wd; creq_wm[ci]=be;
        do @(posedge clk); while(!creq_rdy[ci]); @(negedge clk); creq_v[ci]=0; creq_wm[ci]='1;
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
    logic [XLEN-1:0] r, r1; logic ok;

    initial begin
        for (int i=0;i<MEMLINES;i++) MEM[i]='0;
        for (int c=0;c<2;c++) begin creq_v[c]=0; creq_op[c]=COP_LOAD; creq_amo[c]=AMO_ADD; creq_wa[c]='0; creq_wd[c]='0; creq_wm[c]='1; end
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

        // ---- C1: deferred-snoop -- two cores re-read a line concurrently; one becomes the
        //      grant-and-go owner while still in IS_D and DEFERS the other's FwdGetS (the path
        //      no sequential test can reach). RESP_DLY holds the L2 grant to open the window. ----
        $display("== C1: deferred-snoop (concurrent read; FwdGetS lands mid-IS_D) ==");
        c1d = defer_cnt;
        fork
            cload(0, 'h20, r);          // fresh line WC1 = 0x20 (set 2)
            cload(1, 'h20, r1);         // core1 reads concurrently
        join
        chkv(r,  r1, "C1: both cores read a consistent value");
        // both must observe the same (fresh => 0) value; coherence preserved
        cload(0, 'h20, r);  chkv(r, 0, "C1: core0 reads WC1");
        cload(1, 'h20, r);  chkv(r, 0, "C1: core1 reads WC1");
        chk(defer_cnt - c1d >= 1, "C1: a snoop was DEFERRED mid-acquire (grant-and-go window hit)");
        // coherence still works after the deferral: core0 writes, core1 sees it
        cstore(0, 'h20, 321);
        cload (1, 'h20, r); chkv(r, 321, "C1: post-deferral coherence intact");

        // ---- C7: concurrent AMO to one line -- the loser's GetM forwards from the winner
        //      (FwdGetM deferred during the acquire); both RMWs apply atomically. ----
        $display("== C7: concurrent AMO_ADD (FwdGetM deferred mid-acquire; atomic sum) ==");
        c7d = defer_cnt;
        fork
            camo(0, AMO_ADD, 'h30, 5, r);   // fresh line WC7 = 0x30 (set 3)
            camo(1, AMO_ADD, 'h30, 3, r1);
        join
        cload(0, 'h30, r); chkv(r, 8, "C7: concurrent AMO_ADD final = 0+5+3");
        chk(defer_cnt - c7d >= 1, "C7: a FwdGetM was deferred during an AMO acquire");

        // ---- C2 (DIR-1 regression): a clean PutS from an S-sharer must NOT drop a dirty owner.
        //      core0 owns L dirty (O); core1 shares (S); core1 evicts L (clean PutS via a set
        //      conflict). The dir must stay DIR_O(owner=core0) so a re-read forwards the dirty
        //      value -- not demote to DIR_S and serve stale memory. ----
        $display("== C2: clean PutS preserves a dirty owner (DIR_O not demoted) ==");
        cstore(0, 'h50, 50);                 // core0 -> M(50)
        cload (1, 'h50, r); chkv(r, 50, "C2: core1 shares (core0 M->O, dir DIR_O)");
        cload (1, 'hD0, r);                  // core1 LOAD an L1 set-5 conflict -> clean PutS of 0x50
        cload (1, 'h50, r); chkv(r, 50, "C2: re-read forwards dirty 50 (owner preserved, not stale mem)");

        // ---- C3 (AGT-1 regression): an LR reservation must die when a deferred remote write
        //      (FwdGetM) takes the line mid-acquire; a later SC must FAIL. ----
        $display("== C3: LR reservation killed by a deferred remote write (SC must fail) ==");
        c3d = defer_cnt;
        fork
            clr   (0, 'h60, r);              // core0 LR (grant-and-go E), DATA held by RESP_DLY
            cstore(1, 'h60, 99);             // core1 STORE -> FwdGetM to core0 mid-IS_D -> deferred
        join
        cload(0, 'h60, r);                   // core0 re-fetches; the LR's reservation must be dead
        csc  (0, 'h60, 77, ok); chk(ok==1'b0, "C3: SC fails (reservation killed by remote write)");
        cload(1, 'h60, r);      chkv(r, 99, "C3: core1's store stands");
        chk(defer_cnt - c3d >= 1, "C3: a FwdGetM was deferred during the LR's acquire");

        // ---- C8 (M3d): sub-word store byte-merge via c_req_wmask (needed for the real core's
        //      byte/half stores). Hit-merge + miss-merge (store-MISS install path). ----
        $display("== C8: sub-word store byte-merge (c_req_wmask) ==");
        cstore (0, 'h70, 32'hAABBCCDD);              // full word
        cstoreb(0, 'h70, 32'h00000011, 4'b0001);     // byte0 <- 0x11, bytes 1..3 preserved
        cload  (0, 'h70, r); chkv(r, 32'hAABBCC11, "C8: hit byte-store merges (AABBCC11)");
        cstoreb(1, 'h80, 32'h0000EE00, 4'b0010);     // MISS + byte1 <- 0xEE onto a fresh (0) line
        cload  (1, 'h80, r); chkv(r, 32'h0000EE00, "C8: miss byte-store merges onto fresh line");

        $display("");
        if (errors==0) $display("==== tb_niigo_ccd_gg: ALL CHECKS PASSED ====");
        else           $display("==== tb_niigo_ccd_gg: %0d CHECK(S) FAILED ====", errors);
        $finish;
    end

    initial begin repeat(40000) @(posedge clk); $display("WATCHDOG TIMEOUT"); $finish; end
endmodule
`default_nettype wire
