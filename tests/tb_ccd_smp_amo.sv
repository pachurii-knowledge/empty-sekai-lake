// tb_ccd_smp_amo.sv -- M4 #3: AMO atomicity under real two-core contention. Same
// SMP skeleton as tb_ccd_smp (NCORE real riscv_core_ooo over niigo_ccd_gg_direct,
// behavioural ifetch from a shared IMEM, replicated launch adapter), but the litmus
// is litmus_smp_amo.S: each hart does ITERS atomic `amoadd.w +1` to a shared
// COUNTER. The final counter == NCORE*ITERS iff every RMW is atomic. The Stage-2
// AMO path (COP_AMO_RD acquire-M + LSQ-computed commit-store) has an unprotected
// read->commit-store window (AT_AMO_RD does NOT snoop-replay) -> a remote AMO in
// that window is a lost update; the COHERENT agent-authoritative COP_AMO closes it.
// No devices needed (all-cacheable), so this builds WITHOUT NIIGO_EXT_DEVICES.
// Build/run: make ccd-smp-amo-test
`include "niigo_mem.vh"
`include "niigo_cmi.vh"
`include "niigo_ccd_m1.vh"
`default_nettype none
module top
    import RISCV_ISA::XLEN, RISCV_ISA::XLEN_BYTES;
    import RISCV_UArch::MEMORY_ADDR_WIDTH, RISCV_UArch::MEMORY_READ_WIDTH;
    import NIIGO_Mem::*;
    import NIIGO_CMI::*;
    import NIIGO_CCD_M1::*;
    import MemorySegments::USER_TEXT_START;
;
    localparam int NCORE      = 2;
    localparam int ITERS      = 3;        // per-core atomic increments (must match the litmus)
    localparam int ADDR_SHIFT = $clog2(XLEN_BYTES);
    logic clk=0, rst_l=0;
    always #5 clk=~clk;
    int              cycle_count = 0;
    logic [XLEN-1:0] pc = '0;
    always_ff @(posedge clk) cycle_count <= cycle_count + 1;

    localparam int IMEM_WORDS = 4096;
    localparam logic [MEMORY_ADDR_WIDTH-1:0] TEXT_BASE_W = USER_TEXT_START[XLEN-1:ADDR_SHIFT];
    logic [XLEN-1:0] IMEM [0:IMEM_WORDS-1];

    logic        creq_v   [NCORE];  l1_core_op_e creq_op [NCORE]; l1_amo_op_e creq_amo[NCORE];
    logic [MEMORY_ADDR_WIDTH-1:0] creq_wa[NCORE]; logic [XLEN-1:0] creq_wd[NCORE]; logic [XLEN_BYTES-1:0] creq_wm[NCORE];
    logic        creq_rdy [NCORE];  logic [XLEN-1:0] cresp_rd[NCORE]; logic cresp_sc[NCORE];
    logic        ccd_sk_v [NCORE];  logic [MEMORY_ADDR_WIDTH-1:0] ccd_sk_la[NCORE];
    nmi_req_t    mreq; logic mreq_ready; nmi_resp_t mresp;

    niigo_ccd_gg_direct #(.NACTIVE(NCORE), .L1_SETS(64), .DIR_SETS(256), .RESP_DLY(2)) CCD (
        .clk, .rst_l,
        .c_req_valid(creq_v), .c_req_ready(creq_rdy), .c_req_op(creq_op), .c_req_amo(creq_amo),
        .c_req_waddr(creq_wa), .c_req_wdata(creq_wd), .c_req_wmask(creq_wm),
        .c_resp_rdata(cresp_rd), .c_resp_sc_ok(cresp_sc),
        .flush_req(1'b0), .flush_done(),
        .snoop_kill_valid(ccd_sk_v), .snoop_kill_laddr(ccd_sk_la),
        .mem_req_o(mreq), .mem_req_ready_i(mreq_ready), .mem_resp_i(mresp));

    // typed dmem op -> agent op. M4 #3: 3'd5 = COP_AMO (agent-authoritative atomic RMW).
    function automatic l1_core_op_e map_dmem_op(input logic [2:0] c);
        unique case (c) 3'd1: map_dmem_op=COP_STORE; 3'd2: map_dmem_op=COP_LR;
                        3'd3: map_dmem_op=COP_AMO_RD; 3'd4: map_dmem_op=COP_SC;
                        3'd5: map_dmem_op=COP_AMO;
                        default: map_dmem_op=COP_LOAD; endcase
    endfunction
    // fine AMO sub-op (amo_op_t ordinal on dmem_req_amo) -> agent l1_amo_op_e.
    function automatic l1_amo_op_e map_amo(input logic [3:0] a);
        unique case (a) 4'd3: map_amo=AMO_SWAP; 4'd4: map_amo=AMO_ADD; 4'd5: map_amo=AMO_XOR;
                        4'd6: map_amo=AMO_AND;  4'd7: map_amo=AMO_OR;  4'd8: map_amo=AMO_MIN;
                        4'd9: map_amo=AMO_MAX;  4'd10:map_amo=AMO_MINU; 4'd11:map_amo=AMO_MAXU;
                        default: map_amo=AMO_ADD; endcase
    endfunction

    genvar g;
    generate for (g=0; g<NCORE; g++) begin : CORE
        logic                          if_req_v, if_req_r;
        logic [MEMORY_ADDR_WIDTH-1:0]  if_req_a;
        logic                          if_resp_v, if_resp_e;
        logic [MEMORY_READ_WIDTH-1:0][XLEN-1:0] if_resp_d;
        logic                          d_req_v, d_req_r, d_req_w;
        logic [MEMORY_ADDR_WIDTH-1:0]  d_req_a;
        logic [XLEN-1:0]               d_req_wd;
        logic [XLEN_BYTES-1:0]         d_req_wm;
        logic [2:0]                    d_req_op;
        logic [3:0]                    d_req_amo;
        logic                          d_resp_v;
        logic [MEMORY_ADDR_WIDTH-1:0]  d_resp_a;
        logic [XLEN-1:0]               d_resp_d;
        logic                          pt_req, pt_we; logic [MEMORY_ADDR_WIDTH-1:0] pt_aw; logic [XLEN-1:0] pt_wd;
        logic                          if_inval, d_dev, dcflush_req, halted_c;

        riscv_core #(.HART_ID(g[XLEN-1:0]), .COHERENT(1'b1)) Core (
            .clk, .rst_l,
            .ifetch_req_valid(if_req_v), .ifetch_req_ready(if_req_r), .ifetch_req_addr(if_req_a),
            .ifetch_resp_valid(if_resp_v), .ifetch_resp_data(if_resp_d), .ifetch_resp_excpt(if_resp_e),
            .dmem_req_valid(d_req_v), .dmem_req_ready(d_req_r), .dmem_req_write(d_req_w), .dmem_req_addr(d_req_a),
            .dmem_req_wdata(d_req_wd), .dmem_req_wmask(d_req_wm), .dmem_req_op(d_req_op), .dmem_req_amo(d_req_amo),
            .dmem_resp_valid(d_resp_v), .dmem_resp_addr(d_resp_a), .dmem_resp_data(d_resp_d),
            .dmem_snoop_kill_valid(ccd_sk_v[g]), .dmem_snoop_kill_laddr(ccd_sk_la[g]),
            .ptw_mem_req(pt_req), .ptw_mem_we(pt_we), .ptw_mem_addr_w(pt_aw), .ptw_mem_wdata(pt_wd),
            .ptw_mem_ack(1'b0), .ptw_mem_rdata('0),
            .ifetch_inval(if_inval), .dmem_req_device(d_dev),
            .dcache_flush_req(dcflush_req), .dcache_flush_done(dcflush_req),
            .hpm_l1i_miss(1'b0), .hpm_l1d_miss(1'b0), .hpm_l1d_wb(1'b0), .halted(halted_c));

        logic if_v_q; logic [MEMORY_ADDR_WIDTH-1:0] if_a_q;
        assign if_req_r = 1'b1;
        always_ff @(posedge clk or negedge rst_l) begin
            if (!rst_l) if_v_q <= 1'b0;
            else begin if_v_q <= if_req_v; if_a_q <= if_req_a; end
        end
        assign if_resp_v = if_v_q; assign if_resp_e = 1'b0;
        for (genvar w=0; w<MEMORY_READ_WIDTH; w++)
            assign if_resp_d[w] = IMEM[(if_a_q - TEXT_BASE_W) + w[MEMORY_ADDR_WIDTH-1:0]];

        // replicated launch adapter (dmem <-> c_req[g]); device path unused. The
        // COHERENT AMO issues a COP_AMO (agent-authoritative atomic RMW) which
        // returns the OLD word like a load; map the fine sub-op onto c_req_amo.
        logic ad_busy_q, ad_is_load_q, ad_is_sc_q, ad_is_amo_q; l1_core_op_e ad_op_q; l1_amo_op_e ad_amo_q;
        logic [MEMORY_ADDR_WIDTH-1:0] ad_addr_q; logic [XLEN-1:0] ad_wdata_q; logic [XLEN_BYTES-1:0] ad_wmask_q;
        logic ad_resp_pend_q; logic [XLEN-1:0] ad_resp_data_q; logic [MEMORY_ADDR_WIDTH-1:0] ad_resp_addr_q;
        wire present_dmem  = d_req_v && !d_dev;
        wire ad_can_accept = !ad_busy_q && !ad_resp_pend_q;
        wire ad_launch_fire = present_dmem && ad_can_accept;
        assign d_req_r = ad_can_accept;
        assign creq_v[g]=ad_busy_q; assign creq_op[g]=ad_op_q; assign creq_amo[g]=ad_amo_q;
        assign creq_wa[g]=ad_addr_q; assign creq_wd[g]=ad_wdata_q; assign creq_wm[g]=ad_wmask_q;
        wire ad_done = ad_busy_q && creq_rdy[g];
        always_ff @(posedge clk or negedge rst_l) begin
            if (!rst_l) begin ad_busy_q<=1'b0; ad_resp_pend_q<=1'b0; end
            else begin
                if (ad_launch_fire) begin
                    ad_busy_q<=1'b1; ad_is_load_q<=!d_req_w; ad_is_sc_q<=(d_req_op==3'd4);
                    ad_is_amo_q<=(d_req_op==3'd5);
                    ad_op_q<=map_dmem_op(d_req_op); ad_amo_q<=map_amo(d_req_amo);
                    ad_addr_q<=d_req_a; ad_wdata_q<=d_req_wd; ad_wmask_q<=d_req_wm;
                end else if (ad_done) ad_busy_q<=1'b0;
                if (ad_done && (ad_is_load_q || ad_is_sc_q || ad_is_amo_q)) begin
                    ad_resp_pend_q<=1'b1; ad_resp_addr_q<=ad_addr_q;
                    ad_resp_data_q<= ad_is_sc_q ? (cresp_sc[g] ? '0 : XLEN'(1)) : cresp_rd[g];
                end else if (ad_resp_pend_q) ad_resp_pend_q<=1'b0;
            end
        end
        assign d_resp_v=ad_resp_pend_q; assign d_resp_a=ad_resp_addr_q; assign d_resp_d=ad_resp_data_q;
        wire unused = pt_req|pt_we|(|pt_aw)|(|pt_wd)|if_inval|halted_c|d_dev|dcflush_req;
    end endgenerate

    logic [LINE_BITS-1:0] MEM [logic [MEMORY_ADDR_WIDTH-1:0]];
    assign mreq_ready = 1'b1;
    always_ff @(posedge clk or negedge rst_l) begin
        if (!rst_l) mresp <= '0;
        else begin
            mresp <= '0;
            if (mreq.valid) begin
                mresp.valid <= 1'b1;
                mresp.rdata <= MEM.exists(mreq.waddr) ? MEM[mreq.waddr] : '0;
                if (mreq.op==NMI_WR_LINE) MEM[mreq.waddr] = mreq.wdata;
            end
        end
    end

    localparam logic [MEMORY_ADDR_WIDTH-1:0] COUNTER_W = MEMORY_ADDR_WIDTH'('h140 >> ADDR_SHIFT);
    localparam int L1_IDX = $clog2(64);
    localparam logic [L1_IDX-1:0]         CTR_SET  = COUNTER_W[LINE_WORD_BITS +: L1_IDX];
    localparam logic [LINE_WORD_BITS-1:0] CTR_WOFF = COUNTER_W[LINE_WORD_BITS-1:0];
    function automatic logic [XLEN-1:0] read_counter;
        logic [XLEN-1:0] v; v = '0;
        if (CCD.G_AGENT[0].L1D.state_q[CTR_SET] != CMI_I)
            v = CCD.G_AGENT[0].L1D.data_q[CTR_SET][CTR_WOFF*XLEN +: XLEN];
        else if (CCD.G_AGENT[1].L1D.state_q[CTR_SET] != CMI_I)
            v = CCD.G_AGENT[1].L1D.data_q[CTR_SET][CTR_WOFF*XLEN +: XLEN];
        read_counter = v;
    endfunction

    int errors=0, timeout;
    logic [XLEN-1:0] ctr;
    task automatic chk(input bit ok, input string what);
        if (!ok) begin $display("  [FAIL] %s", what); errors++; end else $display("  [ ok ] %s", what);
    endtask

    initial begin
        for (int i=0;i<IMEM_WORDS;i++) IMEM[i]='0;
        // litmus_smp_amo.S, assembled (rv32ima_zicsr). COUNTER=0x140, DONE[h]=0x180+4h.
        IMEM[ 0] = 32'hf1402473;  // csrr s0,mhartid
        IMEM[ 1] = 32'h14000293;  // li   t0,320      (COUNTER)
        IMEM[ 2] = 32'h00300313;  // li   t1,3        (ITERS)
        IMEM[ 3] = 32'h00100393;  // li   t2,1
        IMEM[ 4] = 32'h00030863;  // loop: beqz t1,done
        IMEM[ 5] = 32'h0072a02f;  // amoadd.w zero,t2,(t0)
        IMEM[ 6] = 32'hfff30313;  // addi t1,t1,-1
        IMEM[ 7] = 32'hff5ff06f;  // j    loop
        IMEM[ 8] = 32'h18000f93;  // done: li t6,384
        IMEM[ 9] = 32'h00241493;  // slli s1,s0,2
        IMEM[10] = 32'h009f8fb3;  // add  t6,t6,s1
        IMEM[11] = 32'h00100f13;  // li   t5,1
        IMEM[12] = 32'h01efa023;  // sw   t5,0(t6)    (DONE[hart]=1)
        IMEM[13] = 32'h0000006f;  // spin: j spin
        for (int c=0;c<NCORE;c++) begin creq_v[c]=0; creq_op[c]=COP_LOAD; creq_amo[c]=AMO_ADD; creq_wm[c]='1; end
        rst_l=0; repeat(8) @(posedge clk); rst_l=1;

        $display("== M4 #3 AMO atomicity: %0d real cores x %0d atomic +1 -> counter == %0d ==",
                 NCORE, ITERS, NCORE*ITERS);
        timeout=0;
        do begin @(posedge clk); ctr=read_counter(); timeout++; end
        while (ctr != XLEN'(NCORE*ITERS) && timeout < 60000);
        ctr = read_counter();

        chk(ctr != 0, "two real cores booted + coherently shared the counter line");
        chk(ctr <= XLEN'(NCORE*ITERS), "counter never exceeds NCORE*ITERS (no spurious increments)");
        if (ctr == XLEN'(NCORE*ITERS))
            $display("  [ ok ] AMO ATOMICITY HELD: counter=%0d == %0d", ctr, NCORE*ITERS);
        else
            $display("  [DIAG] AMO atomicity gap: counter=%0d (expected %0d, %0d lost)",
                     ctr, NCORE*ITERS, NCORE*ITERS-ctr);
        chk(ctr == XLEN'(NCORE*ITERS), "AMO mutual exclusion: every atomic increment landed");

        $display("");
        if (errors==0)
            $display("==== tb_ccd_smp_amo: ALL CHECKS PASSED (AMO atomic under contention) ====");
        else
            $display("==== tb_ccd_smp_amo: %0d CHECK(S) FAILED ====", errors);
        $finish;
    end

    initial begin repeat(120000) @(posedge clk); $display("WATCHDOG TIMEOUT (counter=%0d)", read_counter()); $finish; end
endmodule
`default_nettype wire
