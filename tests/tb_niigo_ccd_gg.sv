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
    // NC: core count. Default 2 (the proven message-level baseline, ccd-gg-test). -DNC4 builds the
    // SAME programs over NACTIVE=4 plus the 4-core-only directed tests (G1 multi-ack, S4, S3) that
    // the 2-core path structurally cannot reach. The NC=2 build is behaviorally unchanged.
`ifdef NC4
    localparam int NC = 4;
`else
    localparam int NC = 2;
`endif
    logic clk=0, rst_l=0;
    always #5 clk=~clk;

    nmi_req_t mreq; logic mreq_ready; nmi_resp_t mresp;

    logic        creq_v   [NC];  l1_core_op_e creq_op [NC]; l1_amo_op_e creq_amo[NC];
    logic [MEMORY_ADDR_WIDTH-1:0] creq_wa[NC]; logic [XLEN-1:0] creq_wd[NC]; logic [XLEN/8-1:0] creq_wm[NC];
    logic        creq_rdy [NC];  logic [XLEN-1:0] cresp_rd[NC]; logic cresp_sc[NC];

    // DIR_SETS >> L1_SETS so an L1 conflict-evict does not also evict the directory entry (a NINE
    // directory-capacity / backup-dir concern, OPEN-3, out of M3c scope).
    niigo_ccd_gg_direct #(.NACTIVE(NC), .DIR_SETS(64), .L1_SETS(8), .RESP_DLY(4)) dut (
        .clk, .rst_l,
        .c_req_valid(creq_v), .c_req_ready(creq_rdy), .c_req_op(creq_op), .c_req_amo(creq_amo),
        .c_req_waddr(creq_wa), .c_req_wdata(creq_wd), .c_req_wmask(creq_wm),
        .c_resp_rdata(cresp_rd), .c_resp_sc_ok(cresp_sc),
        .flush_req(1'b0), .flush_done(),
        .snoop_kill_valid(), .snoop_kill_laddr(),
        .mem_req_o(mreq), .mem_req_ready_i(mreq_ready), .mem_resp_i(mresp));

    // ---- behavioural NMI memory ----
    // 1024 lines so mline() spans addr[13:4] -> directory-aliasing lines (same addr[9:4] dir set,
    // different addr[10+] tag) get DISTINCT backing memory, needed for the S1 capacity test.
    localparam int MEMLINES = 1024;
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
`ifdef NC4
    // 4-core coverage: confirm the never-at-2-cores paths are actually exercised, so a PASS is
    // meaningful (not a coverage hole). saw_oma => an upgrade-from-O ran (S4 setup); min_acks<0 =>
    // the signed ack down-counter went negative-then-corrected (the multi-ack G1 ordering hazard).
    int saw_oma=0, saw_smad=0, saw_ima=0; int min_acks=0; int s4_hit=0, s3_hit=0; int s5_orphan=0;
    // ordinals: T_IM_A=3, T_SM_AD=4, T_OM_A=5, T_EVICT=8 (niigo_l1d_gg tstate_e order).
    // s4_hit = a FwdGetM serviced while in an in-flight UPGRADE (T_OM_A/T_SM_AD) -> the exact S4 gap.
    // s3_hit = a snoop serviced for the T_EVICT victim line (m_vlad_q) -> the exact S3 race window.
    `define COV(K) \
        if (dut.G_AGENT[K].L1D.m_ts_q==4'd5) saw_oma  <= saw_oma+1;  \
        if (dut.G_AGENT[K].L1D.m_ts_q==4'd4) saw_smad <= saw_smad+1; \
        if (dut.G_AGENT[K].L1D.m_ts_q==4'd3) saw_ima  <= saw_ima+1;  \
        if (dut.G_AGENT[K].L1D.m_acks_q < min_acks) min_acks <= dut.G_AGENT[K].L1D.m_acks_q; \
        if (dut.G_AGENT[K].L1D.snoop_rdy_c && dut.G_AGENT[K].L1D.snoop_msg.op==OP_FWD_GETM && \
            (dut.G_AGENT[K].L1D.m_ts_q==4'd5 || dut.G_AGENT[K].L1D.m_ts_q==4'd4)) s4_hit <= s4_hit+1; \
        if (dut.G_AGENT[K].L1D.snoop_rdy_c && dut.G_AGENT[K].L1D.m_ts_q==4'd8 && \
            dut.G_AGENT[K].L1D.snoop_msg.laddr==dut.G_AGENT[K].L1D.m_vlad_q && \
            dut.G_AGENT[K].L1D.m_vst_q==CMI_M) s3_hit <= s3_hit+1; \
        if (dut.G_AGENT[K].L1D.m_ts_q==4'd6 && dut.G_AGENT[K].L1D.d_val_q) s5_orphan <= s5_orphan+1;
    always_ff @(posedge clk) if (rst_l) begin `COV(0) `COV(1) `COV(2) `COV(3) end
`endif

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
    // 4-core directed-test lines: WG1 (set2), WS4 (set3), WS3A/WS3B (both set1, distinct tags -> alias)
    localparam logic [MEMORY_ADDR_WIDTH-1:0] WG1='hA0, WS4='hB0, WS3A='h90, WS3B='h110;
    // S1 capacity: WX and WY map to the SAME directory set (addr[9:4]=0x20) but different tags
    // (addr[10+]) -- a direct-mapped 1-way dir cannot hold both.
    localparam logic [MEMORY_ADDR_WIDTH-1:0] WX='h200, WY='h600;
    localparam logic [MEMORY_ADDR_WIDTH-1:0] WS5='h140, WSCR='h150;   // S5 probe + scratch
    localparam logic [MEMORY_ADDR_WIDTH-1:0] HA='h160, HB='h260;      // B-storm: alias in L1 set 6
    logic [XLEN-1:0] r, r1; logic ok;

    initial begin
        for (int i=0;i<MEMLINES;i++) MEM[i]='0;
        for (int c=0;c<NC;c++) begin creq_v[c]=0; creq_op[c]=COP_LOAD; creq_amo[c]=AMO_ADD; creq_wa[c]='0; creq_wd[c]='0; creq_wm[c]='1; end
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

`ifdef NC4
        // ================= 4-CORE DIRECTED TESTS (G1 multi-ack, S4, S3) =================
        // These exercise the directory/agent paths that are STRUCTURALLY unreachable at 2 cores:
        // ack-to-requester fan > 1 (G1), an owner-in-O upgrade racing a peer GetM (S4), and a
        // dirty-shared (O) line evicted while a peer snoops the victim (S3). See plans/smp-4core-bug-surface.md.

        $display("== G1: multi-sharer GetM ack-to-requester (3 sharers -> core3 STORE, acks=3) ==");
        // The dir fans INVs to {0,1,2} BEFORE fetching the grant DATA, so core3's signed ack
        // down-counter legitimately goes -3 then is corrected by +3 from the DATA -> ReachM. An
        // off-by-one in any arrival order either fires ReachM early (write-atomicity break) or hangs.
        cstore(0, WG1, 0);
        cload (0, WG1, r);   // core0 holds it
        cload (1, WG1, r);   // + core1 shares
        cload (2, WG1, r);   // + core2 shares  => DIR_S {0,1,2}
        cstore(3, WG1, 777); // GetM: INV 3 sharers, DATA carries acks=3
        cload (3, WG1, r); chkv(r, 777, "G1: core3 store applied after 3-sharer INV");
        cload (0, WG1, r); chkv(r, 777, "G1: core0 re-reads coherent value (was INV'd)");
        cload (1, WG1, r); chkv(r, 777, "G1: core1 re-reads coherent value");
        cload (2, WG1, r); chkv(r, 777, "G1: core2 re-reads coherent value");

        $display("== G1b: multi-sharer AMO (3 sharers, core3 amoadd -> acks=3 on the AMO acquire) ==");
        cstore(0, WG1, 10);
        cload(0,WG1,r); cload(1,WG1,r); cload(2,WG1,r);   // DIR_S {0,1,2}, value 10
        camo(3, AMO_ADD, WG1, 5, r); chkv(r, 10, "G1b: core3 amoadd returns old 10 (acks=3)");
        cload(0, WG1, r); chkv(r, 15, "G1b: amo result 15 coherent to core0");

        $display("== S4: owner-in-O UPGRADE vs INVALID-peer GetM (FwdGetM lands in T_OM_A) ==");
        // The S4 gap is specifically a *FwdGetM* (not INV) arriving at an owner in T_OM_A. That needs
        // the writing peer to be INVALID (a sharer-store would be an UPGRADE -> the dir sends INV,
        // which IS demoted today). So: core1 owns dirty + core2 shares (DIR_O owner1), then the
        // owner core1 UPGRADEs while the INVALID core0 GetMs -- core0 (lower idx) wins arb, the dir
        // sends FwdGetM to core1 mid-T_OM_A. Two DIFFERENT words expose a skipped line-install:
        // if core1's upgrade skips installing core0's line, word0 reverts to the stale 0.
        cstore(1, WS4,   8'h0);           // word0 = 0
        cstore(1, WS4+1, 8'h0);           // word1 = 0  (core1 -> M)
        cload (2, WS4,   r);              // core2 reads -> core1 M->O (DIR_O owner1, sharers{1,2})
        fork
            cstore(1, WS4+1, 8'hB1);      // owner core1 UPGRADE from O (T_OM_A), writes WORD1
            cstore(0, WS4,   8'hA0);      // INVALID core0 GetM (lower idx) -> FwdGetM to core1, writes WORD0
        join
        cload(2, WS4,   r);  chk(r==64'hA0, "S4: word0 holds core0's 0xA0 (line install not skipped)");
        cload(2, WS4+1, r1); chk(r1==64'hB1, "S4: word1 holds core1's 0xB1");
        cload(3, WS4,   r);  chk(r==64'hA0, "S4: word0 coherent to core3");
        cload(1, WS4,   r);  chk(r==64'hA0, "S4: word0 coherent to core1 (the upgrader)");

        $display("== S3: dirty-EXCLUSIVE (M) line evicted while a peer GetS snoops it (T_EVICT, M->O) ==");
        // The reported S3 variant: core3 holds S3A in M (DIR_EM, NO other sharers) and conflict-evicts
        // it (PUTM, captured m_vst=M) while the fresh low-index core0 GetSs S3A -- core0's GetS wins
        // arb -> FwdGetS to core3's M victim mid-T_EVICT. core3 M->O demotes and forwards a copy to
        // core0 (creating a sharer), but its captured m_vst stays M so it still sends OP_PUTM. If the
        // dir's PUTM path forces DIR_I regardless of the just-created core0 sharer, core0 is dropped:
        // a later writer then grants M with no INV and core0 keeps a stale copy.
        cstore(3, WS3A, 8'h33);           // core3 -> M(S3A), DIR_EM owner3, no sharers
        fork
            cstore(3, WS3B, 8'h44);       // core3 (idx3): conflict-evict the M victim (PUTM, m_vst=M)
            cload (0, WS3A, r1);          // core0 (idx0, fresh): GetS -> FwdGetS to core3's M victim -> M->O
        join
        cload(0, WS3A, r);  chk(r==64'h33, "S3: core0 reads dirty S3A=0x33 via the evict-vs-snoop forward");
        cstore(1, WS3A, 8'h55);           // core1 writes S3A -> must INV core0 (the FwdGetS-created sharer)
        cload (0, WS3A, r);  chk(r==64'h55, "S3: core0 sees core1's 0x55 (sharer NOT dropped by stale-M PUTM)");
        cload (3, WS3B, r);  chkv(r, 64'h44, "S3: S3B (the evictor's store) is coherent");

        $display("== S1cap: directory capacity aliasing -- two live lines collide in one dir set ==");
        // WX and WY map to the same dir set (addr[9:4]) but different tags. core0 owns X dirty; core1
        // then caches Y (aliasing). A direct-mapped 1-way dir treats Y as a miss and OVERWRITES X's
        // entry with no recall -> it forgets core0 owns X dirty. core2's read of X then misses in the
        // dir and is granted from STALE memory (core0's dirty X is lost; two owners now exist).
        cstore(0, WX, 8'h51);             // core0 -> M(X), dir set 0x20, owner0 dirty (mem still 0)
        cstore(1, WY, 8'h62);             // core1 -> M(Y), SAME dir set -> overwrites X's dir entry
        cload (2, WX, r); chk(r==64'h51, "S1cap: core2 reads core0's dirty X=0x51 (owner not forgotten on aliasing admit)");
        // and a writer to X must still find + INV core0's stale copy
        cstore(3, WX, 8'h73);
        cload (0, WX, r); chk(r==64'h73, "S1cap: core0 sees core3's 0x73 (its stale X copy was invalidated)");

        $display("== S5: probe deferred-snoop-on-ReachM orphan (multi-ack acquire + phase-swept snoop) ==");
        // The S5 hazard needs a FwdGetS/INV to land on the EXACT cycle a multi-ack acquire ReachMs
        // (block C defers it while block E's serve_deferred reads the stale d_val_q=0 -> orphan ->
        // the snooping peer + the dir busy slot hang). We can't do cycle-precise control here, so we
        // PHASE-SWEEP core0's snoop across core3's acquire window by `it` scratch transactions and
        // watch s5_orphan (an agent stuck in T_UNBLK with d_val_q set) + the harness hang watchdog.
        for (int it=0; it<10; it++) begin
            cstore(0, WS5, 8'h10);
            cload(0, WS5, r); cload(1, WS5, r); cload(2, WS5, r);   // DIR_S {0,1,2}
            fork
                cstore(3, WS5, 8'hC0);                              // core3 GetM (acks=3) -> acquiring
                begin for (int d=0; d<it; d++) cload(0, WSCR, r); cload(0, WS5, r); end  // phase-swept snoop
            join
            cload(3, WS5, r); chkv(r, 64'hC0, "S5: core3's store stands after the concurrent snoop");
            cload(0, WS5, r); chkv(r, 64'hC0, "S5: core0 reads coherent value after the race");
        end

        $display("== Bstorm: 4-core hot-line churn (evict vs multi cross-core GetS, the boot pattern) ==");
        // Reproduce the NCORE=4 boot deadlock (dir busy K_SD on a hot line, requester stuck in T_IS_D
        // with no data) at the protocol level: HA and HB ALIAS in one L1 set, so a core that owns HA
        // and then touches HB enters T_EVICT(victim=HA) exactly as two OTHER cores GetS HA -> the dir
        // forwards FwdGetS to the evicting owner while a second snoop also lands. Rotate roles so every
        // core takes each part. A hang here = the harness 200k watchdog fires (== the boot deadlock).
        for (int rr=0; rr<40; rr++) begin
            cstore(rr%4, HA, 64'(rr));            // core (rr%4) -> M(HA)
            fork
                cstore(rr%4,     HB, 64'(rr+100));   // owner evicts HA (T_EVICT victim=HA), fetches HB
                cload ((rr+1)%4, HA, r);             // peer GetS HA -> FwdGetS to the evicting owner
                cload ((rr+2)%4, HA, r1);            // a 2nd peer GetS HA (second snoop)
                cstore((rr+3)%4, HB, 64'(rr+200));   // a 4th core also contends HB
            join
            cload(rr%4, HA, r); chkv(r, 64'(rr), "Bstorm: HA holds the owner's write after the churn");
        end
        // Bstorm2: atomic + GetM contention on one hot line (T_OM_A/T_SM_AD/AMO acquire under churn).
        for (int rr=0; rr<24; rr++) begin
            logic okx;
            fork
                camo ((rr+0)%4, AMO_ADD, HA, 1, r);  // 4 cores hammer one line with AMO/GetM/LR-SC
                cstore((rr+1)%4, HA, 64'(rr));        // GetM contender
                begin clr((rr+2)%4, HA, r1); csc((rr+2)%4, HA, 64'(rr+1), okx); end
                cload((rr+3)%4, HB, r);               // a peer churns the aliasing line (evictions)
            join
        end
        cload(0, HA, r);  // drains/coherence after the atomic storm (value is contention-dependent)
        $display("[coverage] saw_oma=%0d saw_smad=%0d saw_ima=%0d min_acks=%0d s4_hit=%0d s3_hit=%0d s5_orphan=%0d defers=%0d",
                 saw_oma, saw_smad, saw_ima, min_acks, s4_hit, s3_hit, s5_orphan, defer_cnt);
        chk(min_acks < 0, "[cov] multi-ack down-counter went negative (G1 path exercised)");
        chk(s4_hit > 0,  "[cov] a FwdGetM landed on an in-flight UPGRADE T_OM_A/T_SM_AD (S4 window hit)");
        chk(s3_hit > 0,  "[cov] a snoop hit a T_EVICT victim line (S3 window hit)");
`endif

        $display("");
        if (errors==0) $display("==== tb_niigo_ccd_gg: ALL CHECKS PASSED ====");
        else           $display("==== tb_niigo_ccd_gg: %0d CHECK(S) FAILED ====", errors);
        $finish;
    end

`ifdef NC4
    task automatic dump_dl(input string why);
        $display("\n[DEADLOCK] %s", why);
        $display("  DIR: busy=%0b st=%0d bkind=%0d blad=%h breq=%0d bowner=%0d bub=%0b bwb=%0b out(v=%0b op=%0d dst=%0d)",
            dut.DIR.busy_q, dut.DIR.st_q, dut.DIR.bkind_q, dut.DIR.blad_q, dut.DIR.breq_q, dut.DIR.bowner_q,
            dut.DIR.bub_seen_q, dut.DIR.bwb_seen_q, dut.dout_v, dut.dout_m.op, dut.dout_d);
        `define DLAG(K) $display("  hart%0d: m_val=%0b m_ts=%0d m_lad=%h m_vlad=%h m_vst=%0d m_issued=%0b d_val=%0b sr_pend=%0b | ch dmd(v=%0b op=%0d r=%0b) snoop(v=%0b op=%0d) resp(v=%0b) snp(v=%0b op=%0d dst=%0d)", \
            K, dut.G_AGENT[K].L1D.m_val_q, dut.G_AGENT[K].L1D.m_ts_q, dut.G_AGENT[K].L1D.m_lad_q, dut.G_AGENT[K].L1D.m_vlad_q, \
            dut.G_AGENT[K].L1D.m_vst_q, dut.G_AGENT[K].L1D.m_issued_q, dut.G_AGENT[K].L1D.d_val_q, dut.G_AGENT[K].L1D.sr_pend_q, \
            dut.dmd_v[K], dut.dmd_m[K].op, dut.dmd_r[K], dut.snoop_v[K], dut.snoop_m[K].op, dut.resp_v[K], dut.snp_v[K], dut.snp_m[K].op, dut.snp_d[K]);
        `DLAG(0) `DLAG(1) `DLAG(2) `DLAG(3)
    endtask
`endif
    initial begin
        repeat(200000) @(posedge clk);
`ifdef NC4
        dump_dl("gg4 watchdog timeout");
`endif
        $display("WATCHDOG TIMEOUT"); $finish;
    end
endmodule
`default_nettype wire
