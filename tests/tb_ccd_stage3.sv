// tb_ccd_stage3.sv -- M3d Stage 3 validation harness: ONE real riscv_core (core 0) sharing the
// grant-and-go MOESI directory (niigo_ccd_gg_direct #(.NACTIVE(2))) with a BEHAVIOURAL peer
// (core 1). The peer issues a remote write to a line core 0 has reserved (lr.w), which snoops
// core 0's agent -> snoop_kill -> the LSQ reservation coherence-kill (M3d Stage 3a) -> core 0's
// sc.w must FAIL. This gives the (otherwise inert single-core) Stage-3 snoop path live teeth.
//
// I-side: a self-contained behavioural ifetch serving the assembled litmus ($readmemh of the
// standard mem.text.hex, word-per-line, indexed from USER_TEXT_START). D-side: the core's dmem
// is rerouted through a REPLICATED launch adapter (faithful copy of the niigo_memsys CCD arm,
// device-bypass + PTW dropped -- the litmus is bare M-mode + all-cacheable) into c_req[0]. PTW is
// tied off (satp=0). The CCD's coherent backing is the behavioural NMI line memory.
//
// Build/run: make ccd-stage3-test
`include "niigo_mem.vh"
`include "niigo_cmi.vh"
`include "niigo_ccd_m1.vh"
`default_nettype none
// Named `top` (not tb_ccd_stage3) so the lab register_file.sv's `$root.top.cycle_count`/`.pc`
// debug XMR resolves; testbench.sv (which also defines `top`) is excluded from this build.
module top
    import RISCV_ISA::XLEN, RISCV_ISA::XLEN_BYTES;
    import RISCV_UArch::MEMORY_ADDR_WIDTH, RISCV_UArch::MEMORY_READ_WIDTH;
    import NIIGO_Mem::*;
    import NIIGO_CMI::*;
    import NIIGO_CCD_M1::*;
    import MemorySegments::USER_TEXT_START;
;
    localparam int ADDR_SHIFT = $clog2(XLEN_BYTES);
    logic clk=0, rst_l=0;
    always #5 clk=~clk;
    // Referenced by register_file.sv's print_cpu_state XMR ($root.top.cycle_count/.pc); the
    // harness never calls it, so these only need to exist (cycle_count is kept live for traces).
    int              cycle_count = 0;
    logic [XLEN-1:0] pc = '0;
    always_ff @(posedge clk) cycle_count <= cycle_count + 1;

    // ===== real core 0 boundary signals =====
    logic                          ifetch_req_valid, ifetch_req_ready;
    logic [MEMORY_ADDR_WIDTH-1:0]  ifetch_req_addr;
    logic                          ifetch_resp_valid, ifetch_resp_excpt;
    logic [MEMORY_READ_WIDTH-1:0][XLEN-1:0] ifetch_resp_data;
    logic                          dmem_req_valid, dmem_req_ready, dmem_req_write;
    logic [MEMORY_ADDR_WIDTH-1:0]  dmem_req_addr;
    logic [XLEN-1:0]               dmem_req_wdata;
    logic [XLEN_BYTES-1:0]         dmem_req_wmask;
    logic [2:0]                    dmem_req_op;
    logic                          dmem_resp_valid;
    logic [MEMORY_ADDR_WIDTH-1:0]  dmem_resp_addr;
    logic [XLEN-1:0]               dmem_resp_data;
    logic                          dmem_snoop_kill_valid;
    logic [MEMORY_ADDR_WIDTH-1:0]  dmem_snoop_kill_laddr;
    logic                          ptw_mem_req, ptw_mem_we, ptw_mem_ack;
    logic [MEMORY_ADDR_WIDTH-1:0]  ptw_mem_addr_w;
    logic [XLEN-1:0]               ptw_mem_wdata, ptw_mem_rdata;
    logic                          ifetch_inval, dmem_req_device, dcache_flush_req, dcache_flush_done;
    logic                          hpm_l1i_miss, hpm_l1d_miss, hpm_l1d_wb, halted;

    riscv_core #(.COHERENT(1'b1)) Core0 (
        .clk, .rst_l,
        .ifetch_req_valid, .ifetch_req_ready, .ifetch_req_addr,
        .ifetch_resp_valid, .ifetch_resp_data, .ifetch_resp_excpt,
        .dmem_req_valid, .dmem_req_ready, .dmem_req_write, .dmem_req_addr,
        .dmem_req_wdata, .dmem_req_wmask, .dmem_req_op, .dmem_req_amo(),
        .dmem_resp_valid, .dmem_resp_addr, .dmem_resp_data,
        .dmem_snoop_kill_valid, .dmem_snoop_kill_laddr,
        .ptw_mem_req, .ptw_mem_we, .ptw_mem_addr_w, .ptw_mem_wdata,
        .ptw_mem_ack, .ptw_mem_rdata,
        .ifetch_inval, .dmem_req_device, .dcache_flush_req, .dcache_flush_done,
        .hpm_l1i_miss, .hpm_l1d_miss, .hpm_l1d_wb, .halted
    );

    // PTW tied off (bare M-mode litmus, satp=0 -> ptw_mem_req never asserts).
    assign ptw_mem_ack = 1'b0; assign ptw_mem_rdata = '0;
    // No fence.i / halt flush in the litmus; self-complete any spurious request (dormant).
    assign dcache_flush_done = dcache_flush_req;
    assign hpm_l1i_miss = 1'b0; assign hpm_l1d_miss = 1'b0; assign hpm_l1d_wb = 1'b0;

    // ===== behavioural ifetch: serve the litmus, hand-loaded at the reset vector =====
    // The litmus (tests/litmus_lrsc.S, kept as the documentation/source of these encodings) is
    // position-independent (PC-relative branches + absolute data addresses), so it runs unchanged
    // at USER_TEXT_START where the core resets -- no link-address / bootstrap-trampoline juggling.
    localparam int IMEM_WORDS = 4096;
    localparam logic [MEMORY_ADDR_WIDTH-1:0] TEXT_BASE_W = USER_TEXT_START[XLEN-1:ADDR_SHIFT];
    logic [XLEN-1:0] IMEM [0:IMEM_WORDS-1];
    logic                          if_v_q;
    logic [MEMORY_ADDR_WIDTH-1:0]  if_a_q;
    assign ifetch_req_ready = 1'b1;
    always_ff @(posedge clk or negedge rst_l) begin
        if (!rst_l) if_v_q <= 1'b0;
        else begin if_v_q <= ifetch_req_valid && ifetch_req_ready; if_a_q <= ifetch_req_addr; end
    end
    assign ifetch_resp_valid = if_v_q;
    assign ifetch_resp_excpt  = 1'b0;
    for (genvar w=0; w<MEMORY_READ_WIDTH; w++)
        assign ifetch_resp_data[w] = IMEM[(if_a_q - TEXT_BASE_W) + w[MEMORY_ADDR_WIDTH-1:0]];

    // ===== CCD subsystem: 2 agents (core 0 via the adapter; core 1 = behavioural peer) =====
    logic        creq_v   [2];  l1_core_op_e creq_op [2]; l1_amo_op_e creq_amo[2];
    logic [MEMORY_ADDR_WIDTH-1:0] creq_wa[2]; logic [XLEN-1:0] creq_wd[2]; logic [XLEN_BYTES-1:0] creq_wm[2];
    logic        creq_rdy [2];  logic [XLEN-1:0] cresp_rd[2]; logic cresp_sc[2];
    logic        ccd_snoop_kill_v [2]; logic [MEMORY_ADDR_WIDTH-1:0] ccd_snoop_kill_la [2];
    nmi_req_t    mreq; logic mreq_ready; nmi_resp_t mresp;

    niigo_ccd_gg_direct #(.NACTIVE(2), .L1_SETS(64), .DIR_SETS(256), .RESP_DLY(4)) CCD (
        .clk, .rst_l,
        .c_req_valid(creq_v), .c_req_ready(creq_rdy), .c_req_op(creq_op), .c_req_amo(creq_amo),
        .c_req_waddr(creq_wa), .c_req_wdata(creq_wd), .c_req_wmask(creq_wm),
        .c_resp_rdata(cresp_rd), .c_resp_sc_ok(cresp_sc),
        .flush_req(1'b0), .flush_done(),
        .snoop_kill_valid(ccd_snoop_kill_v), .snoop_kill_laddr(ccd_snoop_kill_la),
        .mem_req_o(mreq), .mem_req_ready_i(mreq_ready), .mem_resp_i(mresp));

    // snoop-kill from core 0's agent -> the real core's LSQ (the path under test). M4 S1: the
    // wrapper now exposes a per-agent array; the real core is agent [0].
    assign dmem_snoop_kill_valid = ccd_snoop_kill_v[0];
    assign dmem_snoop_kill_laddr = ccd_snoop_kill_la[0];

    // ===== replicated dmem launch adapter (core 0's dmem <-> c_req[0]) =====
    // Faithful copy of the niigo_memsys CCD arm (device-bypass + PTW dropped). Registered launch
    // breaks the agent's combinational c_req_ready -> dmem_req_valid loop; 1-deep response latch.
    function automatic l1_core_op_e map_dmem_op(input logic [2:0] c);
        unique case (c)
            3'd1:    map_dmem_op = COP_STORE;
            3'd2:    map_dmem_op = COP_LR;
            3'd3:    map_dmem_op = COP_AMO_RD;
            3'd4:    map_dmem_op = COP_SC;
            default: map_dmem_op = COP_LOAD;
        endcase
    endfunction
    logic                          ad_busy_q, ad_is_load_q, ad_is_sc_q;
    l1_core_op_e                   ad_op_q;
    logic [MEMORY_ADDR_WIDTH-1:0]  ad_addr_q;
    logic [XLEN-1:0]               ad_wdata_q;
    logic [XLEN_BYTES-1:0]         ad_wmask_q;
    logic                          ad_resp_pend_q;
    logic [XLEN-1:0]               ad_resp_data_q;
    logic [MEMORY_ADDR_WIDTH-1:0]  ad_resp_addr_q;
    wire present_dmem  = dmem_req_valid && !dmem_req_device;
    wire ad_can_accept = !ad_busy_q && !ad_resp_pend_q;
    wire ad_launch_fire = present_dmem && ad_can_accept;
    assign dmem_req_ready = ad_can_accept;
    assign creq_v[0]   = ad_busy_q;
    assign creq_op[0]  = ad_op_q;
    assign creq_amo[0] = AMO_ADD;
    assign creq_wa[0]  = ad_addr_q;
    assign creq_wd[0]  = ad_wdata_q;
    assign creq_wm[0]  = ad_wmask_q;
    wire ad_done = ad_busy_q && creq_rdy[0];
    always_ff @(posedge clk or negedge rst_l) begin
        if (!rst_l) begin ad_busy_q <= 1'b0; ad_resp_pend_q <= 1'b0; end
        else begin
            if (ad_launch_fire) begin
                ad_busy_q    <= 1'b1;
                ad_is_load_q <= !dmem_req_write;
                ad_is_sc_q   <= (dmem_req_op == 3'd4);   // M4-S5b
                ad_op_q      <= map_dmem_op(dmem_req_op);
                ad_addr_q    <= dmem_req_addr;
                ad_wdata_q   <= dmem_req_wdata;
                ad_wmask_q   <= dmem_req_wmask;
            end else if (ad_done) ad_busy_q <= 1'b0;
            if (ad_done && (ad_is_load_q || ad_is_sc_q)) begin
                ad_resp_pend_q <= 1'b1; ad_resp_addr_q <= ad_addr_q;
                ad_resp_data_q <= ad_is_sc_q ? (cresp_sc[0] ? '0 : XLEN'(1)) : cresp_rd[0];
            end else if (ad_resp_pend_q) ad_resp_pend_q <= 1'b0;
        end
    end
    assign dmem_resp_valid = ad_resp_pend_q;
    assign dmem_resp_addr  = ad_resp_addr_q;
    assign dmem_resp_data  = ad_resp_data_q;

    // ===== behavioural NMI line memory (the CCD's coherent data backing) =====
    // Sparse (associative, keyed by the line word-address) so no address aliases onto another --
    // the directory only writes back on eviction/Put, and reads miss to 0 for never-written lines.
    logic [LINE_BITS-1:0] MEM [logic [MEMORY_ADDR_WIDTH-1:0]];
    assign mreq_ready = 1'b1;
    always_ff @(posedge clk or negedge rst_l) begin
        if (!rst_l) mresp <= '0;
        else begin
            mresp <= '0;
            if (mreq.valid) begin
                mresp.valid <= 1'b1;
                mresp.rdata <= MEM.exists(mreq.waddr) ? MEM[mreq.waddr] : '0;
                if (mreq.op==NMI_WR_LINE) MEM[mreq.waddr] = mreq.wdata;  // assoc array: blocking
            end
        end
    end

    // ===== litmus data addresses (must match litmus_lrsc.S) =====
    localparam logic [MEMORY_ADDR_WIDTH-1:0] SHARED_W = MEMORY_ADDR_WIDTH'('h100 >> ADDR_SHIFT);
    localparam logic [MEMORY_ADDR_WIDTH-1:0] RESULT_W = MEMORY_ADDR_WIDTH'('h140 >> ADDR_SHIFT);
    // Negative control (+nokill): the peer writes a DIFFERENT, un-reserved line, so NO snoop
    // reaches core 0's reserved line -> the reservation survives -> sc.w must SUCCEED. This is the
    // teeth: if the harness passed vacuously, both cases would read the same RESULT.
    localparam logic [MEMORY_ADDR_WIDTH-1:0] OTHER_W  = MEMORY_ADDR_WIDTH'('h200 >> ADDR_SHIFT);
    // The litmus's stores stay dirty (M) in core 0's agent cache (no writeback unless evicted),
    // so read the published RESULT word WHITE-BOX from core 0's agent data array. RES_SET/RES_WOFF
    // mirror the agent's ixf()/off() for RESULT_W at L1_SETS=64.
    localparam int L1_IDX = $clog2(64);
    localparam logic [L1_IDX-1:0]            RES_SET  = RESULT_W[LINE_WORD_BITS +: L1_IDX];
    localparam logic [LINE_WORD_BITS-1:0]    RES_WOFF = RESULT_W[LINE_WORD_BITS-1:0];
    function automatic logic [XLEN-1:0] read_result;
        read_result = CCD.G_AGENT[0].L1D.data_q[RES_SET][RES_WOFF*XLEN +: XLEN];
    endfunction

    // ===== behavioural peer (core 1): write the shared line after core 0's LR completes =====
    int   errors = 0;
    logic saw_lr = 1'b0;
    // detect core 0's LR launching + completing through the adapter
    always_ff @(posedge clk) if (rst_l && ad_done && ad_op_q==COP_LR) saw_lr <= 1'b1;

    task automatic peer_store(input logic [MEMORY_ADDR_WIDTH-1:0] wa, input logic [XLEN-1:0] wd);
        @(negedge clk); creq_v[1]=1; creq_op[1]=COP_STORE; creq_wa[1]=wa; creq_wd[1]=wd; creq_wm[1]='1;
        do @(posedge clk); while(!creq_rdy[1]); @(negedge clk); creq_v[1]=0;
    endtask

    task automatic chk(input bit ok, input string what);
        if (!ok) begin $display("  [FAIL] %s", what); errors++; end
        else        $display("  [ ok ] %s", what);
    endtask

    logic [XLEN-1:0] res;
    int timeout;
    logic nokill;

    initial begin
        nokill = $test$plusargs("nokill");
        for (int i=0;i<IMEM_WORDS;i++) IMEM[i]='0;
        // litmus_lrsc.S, assembled (rv32ima). main: t3=0x100 shared, t4=0x140 result.
        IMEM[0]  = 32'h10000e13;  // li   t3,256          (shared)
        IMEM[1]  = 32'h14000e93;  // li   t4,320          (result)
        IMEM[2]  = 32'h0000ef37;  // lui  t5,0xe
        IMEM[3]  = 32'headf0f13;  // addi t5,t5,-339      -> t5=0xdead sentinel
        IMEM[4]  = 32'h01eea023;  // sw   t5,0(t4)        publish not-done marker
        IMEM[5]  = 32'h100e22af;  // lr.w t0,(t3)         establish reservation
        IMEM[6]  = 32'h19000313;  // li   t1,400
        IMEM[7]  = 32'hfff30313;  // addi t1,t1,-1        \ spin
        IMEM[8]  = 32'hfe031ee3;  // bne  t1,zero,-4      /
        IMEM[9]  = 32'h05500393;  // li   t2,85
        IMEM[10] = 32'h187e252f;  // sc.w a0,t2,(t3)      a0=0 ok / 1 fail
        IMEM[11] = 32'h00150513;  // addi a0,a0,1         1=success, 2=fail
        IMEM[12] = 32'h00aea023;  // sw   a0,0(t4)        publish result
        IMEM[13] = 32'h0000006f;  // 1: j 1b              done
        creq_v[0]=0; creq_v[1]=0; creq_op[1]=COP_LOAD; creq_amo[1]=AMO_ADD; creq_wa[1]='0; creq_wd[1]='0; creq_wm[1]='1;
        rst_l=0; repeat(8) @(posedge clk); rst_l=1;

        $display("== M3d Stage 3 reservation-coherence-kill litmus (%s) ==",
                 nokill ? "negative control: peer writes a DIFFERENT line" : "contended");
        // wait until core 0 has established its reservation (LR completed through the agent)
        timeout=0;
        while (!saw_lr && timeout<20000) begin @(posedge clk); timeout++; end
        chk(saw_lr, "core0 issued lr.w (reservation established)");
        // contended: remote write to the SAME reserved line -> snoop_kill -> LSQ rsv-kill -> SC fails.
        // negative control (+nokill): remote write to OTHER line -> reservation survives -> SC succeeds.
        peer_store(nokill ? OTHER_W : SHARED_W, 32'h99);

        // wait for core 0 to publish the SC result (overwrites the 0xDEAD sentinel)
        timeout=0;
        do begin @(posedge clk); res=read_result(); timeout++; end
        while ((res==32'h0 || res==32'hDEAD) && timeout<20000);
        if (nokill) begin
            chk(res==32'd1, "uncontended sc.w SUCCEEDS (no kill -- reservation survived)");
            if (res!=32'd1) $display("       (RESULT=%h, expected 1=success)", res);
        end else begin
            chk(res==32'd2, "contended sc.w FAILED (reservation coherence-kill fired)");
            if (res!=32'd2) $display("       (RESULT=%h, expected 2=fail)", res);
        end

        $display("");
        if (errors==0) $display("==== tb_ccd_stage3: ALL CHECKS PASSED ====");
        else           $display("==== tb_ccd_stage3: %0d CHECK(S) FAILED ====", errors);
        $finish;
    end

    initial begin repeat(80000) @(posedge clk); $display("WATCHDOG TIMEOUT"); $finish; end
endmodule
`default_nettype wire
