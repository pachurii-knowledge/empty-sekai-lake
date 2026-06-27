// tb_ccd_smp_ipi.sv -- M4 S6b: cross-hart IPI through the ONE shared CLINT, end
// to end through TWO real riscv_core_ooo cores. Same SMP skeleton as tb_ccd_smp
// (NCORE real cores over niigo_ccd_gg_direct, behavioural ifetch from a shared
// IMEM, replicated launch adapter), but the per-core CLINT/PLIC are lifted to
// ONE shared instance (NIIGO_EXT_DEVICES). Both cores run litmus_smp_ipi.S:
// hart 0 stores msip[1]=1 (an IPI); hart 1 spins with M-software interrupts
// enabled, takes the trap, and the handler writes SENTINEL[1]=1 to RAM. The
// device store (CLINT) bypasses the CCD; the SENTINEL store is cacheable and
// flows through the directory. Build/run: make ccd-smp-ipi-test
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
`ifdef NCORE4
    localparam int NCORE      = 4;        // M4 #4: hart 0 broadcasts an IPI to harts 1..3
`else
    localparam int NCORE      = 2;
`endif
    localparam int ADDR_SHIFT = $clog2(XLEN_BYTES);
    logic clk=0, rst_l=0;
    always #5 clk=~clk;
    // register_file.sv's print_cpu_state XMR ($root.top.cycle_count/.pc); unused here.
    int              cycle_count = 0;
    logic [XLEN-1:0] pc = '0;
    always_ff @(posedge clk) cycle_count <= cycle_count + 1;

    // ===== shared litmus instruction memory (both cores fetch it; branch on mhartid) =====
    localparam int IMEM_WORDS = 4096;
    localparam logic [MEMORY_ADDR_WIDTH-1:0] TEXT_BASE_W = USER_TEXT_START[XLEN-1:ADDR_SHIFT];
    logic [XLEN-1:0] IMEM [0:IMEM_WORDS-1];

    // ===== CCD: NCORE real-core agents sharing one directory =====
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

    function automatic l1_core_op_e map_dmem_op(input logic [2:0] c);
        unique case (c) 3'd1: map_dmem_op=COP_STORE; 3'd2: map_dmem_op=COP_LR;
                        3'd3: map_dmem_op=COP_AMO_RD; 3'd4: map_dmem_op=COP_SC;
                        default: map_dmem_op=COP_LOAD; endcase
    endfunction

    // ===== M4 S6b: ONE shared CLINT + PLIC (device hole bypasses the CCD) =====
    localparam int NCTX = 2*NCORE;
    logic                          ds_en [NCORE];
    logic [MEMORY_ADDR_WIDTH-1:0]  ds_wa [NCORE];
    logic [XLEN-1:0]               ds_wd [NCORE];
    logic [XLEN_BYTES-1:0]         ds_wm [NCORE];
    logic [MEMORY_ADDR_WIDTH-1:0]  dl_a  [NCORE];
    logic                          dl_en [NCORE];
    logic [ADDR_SHIFT-1:0]         dl_off[NCORE];
    logic                          ext_hit [NCORE]; logic [XLEN-1:0] ext_dat [NCORE];
    logic [NCORE*MEMORY_ADDR_WIDTH-1:0] dl_a_p;
    logic [NCORE-1:0]                   dl_en_p;
    logic [NCORE*ADDR_SHIFT-1:0]        dl_off_p;
    logic [NCORE-1:0]                   cl_hit_p, pl_hit_p, cl_mtip, cl_msip, pl_mext, pl_sext;
    logic [NCORE*XLEN-1:0]              cl_data_p, pl_data_p;
    logic [63:0]                        cl_mtime;
    // non-dropping device-store hub: a FIFO buffers up to NCORE committed device
    // stores per cycle and drains ONE to the single shared device store port per
    // cycle, so no cross-core device store is ever lost (the priority-mux would
    // drop simultaneous stores -- visible once 4 cores hammer msip).
    logic                          dev_st_en;
    logic [MEMORY_ADDR_WIDTH-1:0]  dev_st_wa;
    logic [XLEN-1:0]               dev_st_wd;
    logic [XLEN_BYTES-1:0]         dev_st_wm;
    localparam int DQ = 64;
    logic [MEMORY_ADDR_WIDTH-1:0]  dq_wa [DQ];
    logic [XLEN-1:0]               dq_wd [DQ];
    logic [XLEN_BYTES-1:0]         dq_wm [DQ];
    int                            dq_head = 0, dq_tail = 0;
    int                            dst_drop = 0;   // FIFO overflow (should stay 0)

    // ===== per-core: real core + behavioural ifetch + replicated launch adapter =====
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
            .dmem_req_wdata(d_req_wd), .dmem_req_wmask(d_req_wm), .dmem_req_op(d_req_op), .dmem_req_amo(),
            .dmem_resp_valid(d_resp_v), .dmem_resp_addr(d_resp_a), .dmem_resp_data(d_resp_d),
            .dmem_snoop_kill_valid(ccd_sk_v[g]), .dmem_snoop_kill_laddr(ccd_sk_la[g]),
            .ptw_mem_req(pt_req), .ptw_mem_we(pt_we), .ptw_mem_addr_w(pt_aw), .ptw_mem_wdata(pt_wd),
            .ptw_mem_ack(1'b0), .ptw_mem_rdata('0),
            .ifetch_inval(if_inval), .dmem_req_device(d_dev),
            .dcache_flush_req(dcflush_req), .dcache_flush_done(dcflush_req),
            .hpm_l1i_miss(1'b0), .hpm_l1d_miss(1'b0), .hpm_l1d_wb(1'b0),
            // M4 S6b shared-device interface into the hub.
            .dsnoop_store_en(ds_en[g]), .dsnoop_store_waddr(ds_wa[g]),
            .dsnoop_store_wdata(ds_wd[g]), .dsnoop_store_mask(ds_wm[g]),
            .dsnoop_load_addr(dl_a[g]), .dsnoop_load_en(dl_en[g]), .dsnoop_load_off(dl_off[g]),
            .ext_load_hit(ext_hit[g]), .ext_load_data(ext_dat[g]), .ext_mtime(cl_mtime),
            .ext_irq_m_timer(cl_mtip[g]), .ext_irq_m_software(cl_msip[g]),
            .ext_irq_m_external(pl_mext[g]), .ext_irq_s_external(pl_sext[g]),
            .halted(halted_c));

        // behavioural ifetch from the shared IMEM (1-cycle latency)
        logic if_v_q; logic [MEMORY_ADDR_WIDTH-1:0] if_a_q;
        assign if_req_r = 1'b1;
        always_ff @(posedge clk or negedge rst_l) begin
            if (!rst_l) if_v_q <= 1'b0;
            else begin if_v_q <= if_req_v; if_a_q <= if_req_a; end
        end
        assign if_resp_v = if_v_q; assign if_resp_e = 1'b0;
        for (genvar w=0; w<MEMORY_READ_WIDTH; w++)
            assign if_resp_d[w] = IMEM[(if_a_q - TEXT_BASE_W) + w[MEMORY_ADDR_WIDTH-1:0]];

        // replicated launch adapter (dmem <-> c_req[g]); device path bypasses CCD
        logic ad_busy_q, ad_is_load_q, ad_is_sc_q; l1_core_op_e ad_op_q;
        logic [MEMORY_ADDR_WIDTH-1:0] ad_addr_q; logic [XLEN-1:0] ad_wdata_q; logic [XLEN_BYTES-1:0] ad_wmask_q;
        logic ad_resp_pend_q; logic [XLEN-1:0] ad_resp_data_q; logic [MEMORY_ADDR_WIDTH-1:0] ad_resp_addr_q;
        wire present_dmem  = d_req_v && !d_dev;
        wire ad_can_accept = !ad_busy_q && !ad_resp_pend_q;
        wire ad_launch_fire = present_dmem && ad_can_accept;
        assign d_req_r = ad_can_accept;
        assign creq_v[g]=ad_busy_q; assign creq_op[g]=ad_op_q; assign creq_amo[g]=AMO_ADD;
        assign creq_wa[g]=ad_addr_q; assign creq_wd[g]=ad_wdata_q; assign creq_wm[g]=ad_wmask_q;
        wire ad_done = ad_busy_q && creq_rdy[g];
        always_ff @(posedge clk or negedge rst_l) begin
            if (!rst_l) begin ad_busy_q<=1'b0; ad_resp_pend_q<=1'b0; end
            else begin
                if (ad_launch_fire) begin
                    ad_busy_q<=1'b1; ad_is_load_q<=!d_req_w; ad_is_sc_q<=(d_req_op==3'd4);
                    ad_op_q<=map_dmem_op(d_req_op);
                    ad_addr_q<=d_req_a; ad_wdata_q<=d_req_wd; ad_wmask_q<=d_req_wm;
                end else if (ad_done) ad_busy_q<=1'b0;
                if (ad_done && (ad_is_load_q || ad_is_sc_q)) begin
                    ad_resp_pend_q<=1'b1; ad_resp_addr_q<=ad_addr_q;
                    ad_resp_data_q<= ad_is_sc_q ? (cresp_sc[g] ? '0 : XLEN'(1)) : cresp_rd[g];
                end else if (ad_resp_pend_q) ad_resp_pend_q<=1'b0;
            end
        end
        assign d_resp_v=ad_resp_pend_q; assign d_resp_a=ad_resp_addr_q; assign d_resp_d=ad_resp_data_q;
        wire unused = pt_req|pt_we|(|pt_aw)|(|pt_wd)|if_inval|halted_c|d_dev|dcflush_req;
    end endgenerate

    // ===== shared-device hub: non-dropping store FIFO, load-query packing, instances =====
    // drain the FIFO head to the device store port (one store/cycle).
    wire dq_empty = (dq_head == dq_tail);
    always_comb begin
        dev_st_en = !dq_empty;
        dev_st_wa = dq_wa[dq_head % DQ];
        dev_st_wd = dq_wd[dq_head % DQ];
        dev_st_wm = dq_wm[dq_head % DQ];
    end
    always_ff @(posedge clk or negedge rst_l) begin
        if (!rst_l) begin dq_head<=0; dq_tail<=0; dst_drop<=0; end
        else begin
            int t; t = dq_tail;
            for (int c=0;c<NCORE;c++) if (ds_en[c]) begin
                if ((t - dq_head) < DQ) begin
                    dq_wa[t % DQ] <= ds_wa[c]; dq_wd[t % DQ] <= ds_wd[c]; dq_wm[t % DQ] <= ds_wm[c];
                    t = t + 1;
                end else dst_drop <= dst_drop + 1;   // FIFO full (never expected)
            end
            dq_tail <= t;
            if (!dq_empty) dq_head <= dq_head + 1;     // drain one this cycle
        end
    end
    always_comb begin
        for (int c=0;c<NCORE;c++) begin
            dl_a_p[c*MEMORY_ADDR_WIDTH +: MEMORY_ADDR_WIDTH] = dl_a[c];
            dl_en_p[c]                                       = dl_en[c];
            dl_off_p[c*ADDR_SHIFT +: ADDR_SHIFT]             = dl_off[c];
            ext_hit[c] = cl_hit_p[c] | pl_hit_p[c];
            ext_dat[c] = cl_hit_p[c] ? cl_data_p[c*XLEN +: XLEN] : pl_data_p[c*XLEN +: XLEN];
        end
    end
    clint #(.NUM_HARTS(NCORE), .NPORT(NCORE)) CLINT (
        .clk, .rst_l,
        .store_en(dev_st_en), .store_waddr(dev_st_wa), .store_wdata(dev_st_wd), .store_mask(dev_st_wm),
        .load_addr(dl_a_p), .load_hit(cl_hit_p), .load_data(cl_data_p),
        .irq_m_timer(cl_mtip), .irq_m_software(cl_msip), .mtime_out(cl_mtime));
    plic #(.NCTX(NCTX), .NSOURCES(31), .NPORT(NCORE)) PLIC (
        .clk, .rst_l, .src_irq('0),
        .store_en(dev_st_en), .store_waddr(dev_st_wa), .store_wdata(dev_st_wd), .store_mask(dev_st_wm),
        .load_addr(dl_a_p), .load_en(dl_en_p), .load_off(dl_off_p),
        .load_hit(pl_hit_p), .load_data(pl_data_p),
        .irq_m_external(pl_mext), .irq_s_external(pl_sext));

    // ===== shared sparse NMI line memory =====
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

    // ===== white-box SENTINEL[h] readout (RAM line owned by whichever agent) =====
    localparam int L1_IDX = $clog2(64);
    function automatic logic [XLEN-1:0] read_sentinel(input int h);
        logic [MEMORY_ADDR_WIDTH-1:0] wa;
        logic [L1_IDX-1:0]         st;
        logic [LINE_WORD_BITS-1:0] woff;
        logic [XLEN-1:0] v; v='0;
        wa   = MEMORY_ADDR_WIDTH'((32'h200 + 32'(h)*4) >> ADDR_SHIFT);
        st   = wa[LINE_WORD_BITS +: L1_IDX];
        woff = wa[LINE_WORD_BITS-1:0];
        if (CCD.G_AGENT[0].L1D.state_q[st] != CMI_I)
            v = CCD.G_AGENT[0].L1D.data_q[st][woff*XLEN +: XLEN];
        else if (CCD.G_AGENT[1].L1D.state_q[st] != CMI_I)
            v = CCD.G_AGENT[1].L1D.data_q[st][woff*XLEN +: XLEN];
`ifdef NCORE4
        else if (CCD.G_AGENT[2].L1D.state_q[st] != CMI_I)
            v = CCD.G_AGENT[2].L1D.data_q[st][woff*XLEN +: XLEN];
        else if (CCD.G_AGENT[3].L1D.state_q[st] != CMI_I)
            v = CCD.G_AGENT[3].L1D.data_q[st][woff*XLEN +: XLEN];
`endif
        read_sentinel = v;
    endfunction

    int errors=0, timeout;
    logic all_trapped;
    logic saw_msip1 = 1'b0;          // latch: hart-1 msip was actually raised in the CLINT
    always_ff @(posedge clk) if (cl_msip[1]) saw_msip1 <= 1'b1;
    task automatic chk(input bit ok, input string what);
        if (!ok) begin $display("  [FAIL] %s", what); errors++; end else $display("  [ ok ] %s", what);
    endtask

    initial begin
        for (int i=0;i<IMEM_WORDS;i++) IMEM[i]='0;
        // litmus_smp_ipi.S, assembled (rv32ima_zicsr). CLINT 0x0200_0000; SENTINEL 0x200.
        // hart 0 broadcasts msip[1..3]=1 (2-core: 2,3 are no-ops); each non-0 hart traps.
        IMEM[ 0] = 32'hf1402473;  // csrr s0,mhartid
        IMEM[ 1] = 32'h00000297;  // auipc t0,0
        IMEM[ 2] = 32'h04c28293;  // addi t0,t0,76   -> handler
        IMEM[ 3] = 32'h30529073;  // csrw mtvec,t0
        IMEM[ 4] = 32'h00800293;  // li   t0,8
        IMEM[ 5] = 32'h3042a073;  // csrs mie,t0     (MSIE)
        IMEM[ 6] = 32'h3002a073;  // csrs mstatus,t0 (MIE)
        IMEM[ 7] = 32'h02041863;  // bnez s0,receiver
        IMEM[ 8] = 32'h020002b7;  // sender: lui t0,0x2000 (CLINT)
        IMEM[ 9] = 32'h00100e93;  // li   t4,1
        IMEM[10] = 32'h00100313;  // li   t1,1       (h)
        IMEM[11] = 32'h00400393;  // li   t2,4       (limit)
        IMEM[12] = 32'h00735c63;  // sloop: bge t1,t2,spin0
        IMEM[13] = 32'h00231e13;  // slli t3,t1,2
        IMEM[14] = 32'h01c28f33;  // add  t5,t0,t3
        IMEM[15] = 32'h01df2023;  // sw   t4,0(t5)   (msip[h]=1)
        IMEM[16] = 32'h00130313;  // addi t1,t1,1
        IMEM[17] = 32'hfedff06f;  // j    sloop
        IMEM[18] = 32'h0000006f;  // spin0: j spin0
        IMEM[19] = 32'h0000006f;  // receiver: j receiver
        IMEM[20] = 32'hf1402473;  // handler: csrr s0,mhartid
        IMEM[21] = 32'h00241493;  // slli s1,s0,2
        IMEM[22] = 32'h20000393;  // li   t2,512     (SENTINEL)
        IMEM[23] = 32'h009383b3;  // add  t2,t2,s1
        IMEM[24] = 32'h00100e13;  // li   t3,1
        IMEM[25] = 32'h01c3a023;  // sw   t3,0(t2)   (SENTINEL[hart]=1)
        IMEM[26] = 32'h020002b7;  // lui  t0,0x2000
        IMEM[27] = 32'h009282b3;  // add  t0,t0,s1
        IMEM[28] = 32'h0002a023;  // sw   zero,0(t0) (msip[hart]=0)
        IMEM[29] = 32'h0000006f;  // hspin: j hspin
        for (int c=0;c<NCORE;c++) begin creq_v[c]=0; creq_op[c]=COP_LOAD; creq_amo[c]=AMO_ADD; creq_wm[c]='1; end
        rst_l=0; repeat(8) @(posedge clk); rst_l=1;

        $display("== M4 #4 cross-hart IPI: hart0 -> msip[1..%0d] -> each takes M-software trap ==", NCORE-1);
        // wait until every receiver hart (1..NCORE-1) has taken its trap (SENTINEL[h]=1)
        timeout=0;
        do begin
            @(posedge clk); timeout++;
            all_trapped = 1'b1;
            for (int h=1; h<NCORE; h++) if (read_sentinel(h) != XLEN'(1)) all_trapped = 1'b0;
        end while (!all_trapped && timeout < 40000);
        // handlers clear their own msip a few instructions after SENTINEL; let those drain.
        timeout=0;
        while ((|cl_msip) && timeout < 4000) begin @(posedge clk); timeout++; end

        chk(saw_msip1, "CLINT raised hart-1 msip (the IPI landed in the shared device)");
        for (int h=1; h<NCORE; h++)
            chk(read_sentinel(h) == XLEN'(1),
                $sformatf("hart %0d took the M-software-interrupt trap (SENTINEL[%0d]=1)", h, h));
        chk(read_sentinel(0) == XLEN'(0), "hart 0 did NOT trap (SENTINEL[0]=0; it only sent the IPIs)");
        chk(cl_msip == '0, "all handlers cleared their own msip (cl_msip all deasserted)");
        chk(dst_drop == 0, "device-store FIFO never overflowed (no lost cross-core stores)");
        chk(dq_head == dq_tail, "device-store FIFO fully drained (every store delivered)");

        $display("");
        if (errors==0)
            $display("==== tb_ccd_smp_ipi: ALL CHECKS PASSED (cross-hart IPI through shared CLINT) ====");
        else
            $display("==== tb_ccd_smp_ipi: %0d CHECK(S) FAILED ====", errors);
        $finish;
    end

    initial begin repeat(60000) @(posedge clk);
        $display("WATCHDOG TIMEOUT (SENTINEL[1]=%0d saw_msip1=%0d)", read_sentinel(1), saw_msip1); $finish; end
endmodule
`default_nettype wire
