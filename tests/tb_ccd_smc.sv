// tb_ccd_smc.sv -- P4: cross-core self-modifying-code (SMC) litmus. Validates the
// REMOTE-DIRTY I-FETCH mechanism that xv6-SMP needs (a process's freshly-written
// text fetched by another hart). Each core has a REAL l1_icache behind the shared
// grant-and-go directory. On an L1I refill the harness probes the core's LOCAL L1D
// agent (P2); on a probe MISS it injects a COP_LOAD so the directory's GetS pulls
// the line cold-from-memory OR DIRTY-FROM-THE-OWNER, installs it locally, then
// probe-serves it to the L1I. The L1I never reads backing memory directly, so a
// line another core holds dirty is always fetched coherently -- no fence.i.
//
//   hart 0 (producer): patches word0 of the `patch` cache line from
//     "addi a0,x0,0x111" (0x11100513, WRONG) to "addi a0,x0,0x222"
//     (0x22200513, RIGHT) via a data store (-> DIRTY in hart 0's L1D), fences,
//     then releases FLAG=1.
//   hart 1 (consumer): spins on FLAG, then JUMPS to `patch` -- a line it has
//     NEVER fetched -> cold L1I miss -> COP_LOAD -> directory forwards hart 0's
//     dirty line. The patched code writes a0 to RESULT (0x140) and spins.
//
// RESULT == 0x222 iff the consumer fetched hart 0's DIRTY copy coherently;
// RESULT == 0x111 means it read the stale image (the mechanism is broken). The
// program (assembled, both widths identical) uses only width-agnostic ops, so
// one stream serves RV32 and RV64 (only the IMEM->line packing is width-gated).
// Build/run: make ccd-smc-test  (RV32)  |  make ccd-smc-rv64-test  (RV64)
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
    localparam int NCORE      = 4;
`else
    localparam int NCORE      = 2;
`endif
    localparam int ADDR_SHIFT = $clog2(XLEN_BYTES);
    localparam int LINE_WORDS = LINE_BITS / XLEN;   // bus words per 64 B line
    localparam int NPROG = 33;                        // assembled SMC litmus length (words)
`ifdef RV64
    localparam int NW = (NPROG+1)/2;                  // RV64: 2 insns per 64-bit bus word
`else
    localparam int NW = NPROG;                        // RV32: 1 insn per bus word
`endif
    logic clk=0, rst_l=0;
    always #5 clk=~clk;
    // register_file.sv's print_cpu_state XMR ($root.top.cycle_count/.pc); never called here.
    int              cycle_count = 0;
    logic [XLEN-1:0] pc = '0;
    always_ff @(posedge clk) cycle_count <= cycle_count + 1;

    localparam logic [MEMORY_ADDR_WIDTH-1:0] TEXT_BASE_W = USER_TEXT_START[XLEN-1:ADDR_SHIFT];
    localparam logic [XLEN-1:0] RESULT_RIGHT = XLEN'('h222);
    localparam logic [XLEN-1:0] RESULT_WRONG = XLEN'('h111);

    // ===== CCD: NCORE real-core agents sharing one directory (+ P2 probe arrays) =====
    logic        creq_v   [NCORE];  l1_core_op_e creq_op [NCORE]; l1_amo_op_e creq_amo[NCORE];
    logic [MEMORY_ADDR_WIDTH-1:0] creq_wa[NCORE]; logic [XLEN-1:0] creq_wd[NCORE]; logic [XLEN_BYTES-1:0] creq_wm[NCORE];
    logic        creq_rdy [NCORE];  logic [XLEN-1:0] cresp_rd[NCORE]; logic cresp_sc[NCORE];
    logic        ccd_sk_v [NCORE];  logic [MEMORY_ADDR_WIDTH-1:0] ccd_sk_la[NCORE];
    logic        ccd_probe_v [NCORE]; logic [MEMORY_ADDR_WIDTH-1:0] ccd_probe_wa[NCORE];
    logic        ccd_probe_hit[NCORE]; logic [LINE_BITS-1:0] ccd_probe_line[NCORE];
    nmi_req_t    mreq; logic mreq_ready; nmi_resp_t mresp;

    niigo_ccd_gg_direct #(.NACTIVE(NCORE), .L1_SETS(64), .DIR_SETS(256), .RESP_DLY(2)) CCD (
        .clk, .rst_l,
        .c_req_valid(creq_v), .c_req_ready(creq_rdy), .c_req_op(creq_op), .c_req_amo(creq_amo),
        .c_req_waddr(creq_wa), .c_req_wdata(creq_wd), .c_req_wmask(creq_wm),
        .c_resp_rdata(cresp_rd), .c_resp_sc_ok(cresp_sc),
        .flush_req(1'b0), .flush_done(),
        .snoop_kill_valid(ccd_sk_v), .snoop_kill_laddr(ccd_sk_la),
        .probe_valid(ccd_probe_v), .probe_waddr(ccd_probe_wa),
        .probe_hit(ccd_probe_hit), .probe_line(ccd_probe_line),
        .mem_req_o(mreq), .mem_req_ready_i(mreq_ready), .mem_resp_i(mresp));

    function automatic l1_core_op_e map_dmem_op(input logic [2:0] c);
        unique case (c) 3'd1: map_dmem_op=COP_STORE; 3'd2: map_dmem_op=COP_LR;
                        3'd3: map_dmem_op=COP_AMO_RD; 3'd4: map_dmem_op=COP_SC;
                        default: map_dmem_op=COP_LOAD; endcase
    endfunction

    // ===== per-core: real core + REAL l1_icache (agent-served refill) + launch adapter =====
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
            .dcache_flush_req(dcflush_req), .dcache_flush_done(dcflush_req),  // self-complete (no flush in litmus)
            .hpm_l1i_miss(1'b0), .hpm_l1d_miss(1'b0), .hpm_l1d_wb(1'b0),
            .halted(halted_c));

        // ---- REAL L1 instruction cache (refill served by the local agent) ----
        nmi_req_t  l1i_req;  logic l1i_rdy;  nmi_resp_t l1i_resp;
        logic                          l1i_snp_v;  logic [MEMORY_ADDR_WIDTH-1:0] l1i_snp_wa;
        logic                          l1i_acc, l1i_mis;
        l1_icache L1I (
            .clk, .rst_l,
            .ifetch_req_valid(if_req_v), .ifetch_req_ready(if_req_r), .ifetch_req_addr(if_req_a),
            .ifetch_resp_valid(if_resp_v), .ifetch_resp_data(if_resp_d), .ifetch_resp_excpt(if_resp_e),
            .inval_all(if_inval),
            .snoop_valid(l1i_snp_v), .snoop_waddr(l1i_snp_wa),
            .nmi_req(l1i_req), .nmi_req_ready(l1i_rdy), .nmi_resp(l1i_resp),
            .ev_access(l1i_acc), .ev_miss(l1i_mis));

        // ---- launch adapter (dmem -> c_req[g]) + iref (I-fetch COP_LOAD on probe miss) ----
        logic ad_busy_q, ad_is_load_q, ad_is_sc_q; l1_core_op_e ad_op_q;
        logic [MEMORY_ADDR_WIDTH-1:0] ad_addr_q; logic [XLEN-1:0] ad_wdata_q; logic [XLEN_BYTES-1:0] ad_wmask_q;
        logic ad_resp_pend_q; logic [XLEN-1:0] ad_resp_data_q; logic [MEMORY_ADDR_WIDTH-1:0] ad_resp_addr_q;
        logic iref_busy_q; logic [MEMORY_ADDR_WIDTH-1:0] iref_addr_q;

        // P2 probe: drive this agent's probe with the L1I's pending refill line addr.
        assign ccd_probe_v[g]  = l1i_req.valid && (l1i_req.op==NMI_RD_LINE);
        assign ccd_probe_wa[g] = l1i_req.waddr;

        wire present_dmem  = d_req_v && !d_dev;
        wire ad_can_accept = !ad_busy_q && !ad_resp_pend_q;
        wire ad_launch_fire = present_dmem && ad_can_accept && !iref_busy_q;       // dmem priority, yields to nothing
        wire iref_need     = ccd_probe_v[g] && !ccd_probe_hit[g];                  // local miss -> need a coherent pull
        wire iref_launch   = iref_need && !iref_busy_q && ad_can_accept && !ad_launch_fire;
        assign d_req_r = ad_can_accept && !iref_busy_q;                            // hold dmem off while an iref owns c_req

        // c_req mux: the launch adapter (registered) OR the iref COP_LOAD (mutually exclusive)
        assign creq_v[g]  = ad_busy_q || iref_busy_q;
        assign creq_op[g] = iref_busy_q ? COP_LOAD : ad_op_q;
        assign creq_amo[g]= AMO_ADD;
        assign creq_wa[g] = iref_busy_q ? iref_addr_q : ad_addr_q;
        assign creq_wd[g] = ad_wdata_q;
        assign creq_wm[g] = iref_busy_q ? '0 : ad_wmask_q;
        wire ad_done   = ad_busy_q   && creq_rdy[g];
        wire iref_done = iref_busy_q && creq_rdy[g];

        always_ff @(posedge clk or negedge rst_l) begin
            if (!rst_l) begin ad_busy_q<=1'b0; ad_resp_pend_q<=1'b0; iref_busy_q<=1'b0; end
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
                // iref: a single COP_LOAD that pulls the refill line into the local L1D
                if (iref_launch) begin iref_busy_q<=1'b1; iref_addr_q<=l1i_req.waddr; end
                else if (iref_done) iref_busy_q<=1'b0;
            end
        end
        assign d_resp_v=ad_resp_pend_q; assign d_resp_a=ad_resp_addr_q; assign d_resp_d=ad_resp_data_q;

        // ---- probe-serve: once the refill line is local (M/O/E/S), deliver it to the L1I ----
        logic pserve_q; logic [LINE_BITS-1:0] pserve_line_q;
        wire l1i_probe_serve = ccd_probe_v[g] && ccd_probe_hit[g];
        assign l1i_rdy = l1i_probe_serve && !pserve_q;     // accept (1 cy) then deliver (next cy)
        always_ff @(posedge clk or negedge rst_l) begin
            if (!rst_l)                                 pserve_q<=1'b0;
            else if (l1i_probe_serve && !pserve_q) begin pserve_q<=1'b1; pserve_line_q<=ccd_probe_line[g]; end
            else if (pserve_q)                          pserve_q<=1'b0;
        end
        always_comb begin
            l1i_resp = '0;
            if (pserve_q) begin l1i_resp.valid=1'b1; l1i_resp.rdata=pserve_line_q; l1i_resp.err=1'b0; end
        end

        // ---- L1I snoop-invalidate: a local committed store OR a remote write-snoop ----
        wire ad_commit_write = ad_done && (ad_op_q==COP_STORE);
        assign l1i_snp_v  = ad_commit_write || ccd_sk_v[g];
        assign l1i_snp_wa = ad_commit_write ? ad_addr_q : ccd_sk_la[g];

        // sink unused outputs
        wire unused = pt_req|pt_we|(|pt_aw)|(|pt_wd)|halted_c|d_dev|if_inval|l1i_acc|l1i_mis;
    end endgenerate

    // ===== shared sparse NMI line memory (the directory's backend; seeded with the program) =====
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

    // ===== RESULT readout: white-box from whichever agent owns the RESULT line =====
    // patch line base (word address) for the backing-memory discriminator check.
    localparam logic [MEMORY_ADDR_WIDTH-1:0] PATCH_W  = (USER_TEXT_START + 'h40) >> ADDR_SHIFT;
    localparam logic [MEMORY_ADDR_WIDTH-1:0] PATCH_LB = PATCH_W & ~MEMORY_ADDR_WIDTH'(LINE_WORDS-1);
    localparam logic [MEMORY_ADDR_WIDTH-1:0] RESULT_W = MEMORY_ADDR_WIDTH'('h140 >> ADDR_SHIFT);
    localparam int L1_IDX = $clog2(64);
    localparam logic [L1_IDX-1:0]         RES_SET  = RESULT_W[LINE_WORD_BITS +: L1_IDX];
    localparam logic [LINE_WORD_BITS-1:0] RES_WOFF = RESULT_W[LINE_WORD_BITS-1:0];
    // the RESULT line lives in whichever agent last wrote it; pick the owning (non-I) agent
    // (constant indices: Verilator can't index a generate hierarchy with a loop variable).
    function automatic logic [XLEN-1:0] read_result;
        logic [XLEN-1:0] v; v = '0;
        if (CCD.G_AGENT[0].L1D.state_q[RES_SET] != CMI_I)
            v = CCD.G_AGENT[0].L1D.data_q[RES_SET][RES_WOFF*XLEN +: XLEN];
        else if (CCD.G_AGENT[1].L1D.state_q[RES_SET] != CMI_I)
            v = CCD.G_AGENT[1].L1D.data_q[RES_SET][RES_WOFF*XLEN +: XLEN];
`ifdef NCORE4
        else if (CCD.G_AGENT[2].L1D.state_q[RES_SET] != CMI_I)
            v = CCD.G_AGENT[2].L1D.data_q[RES_SET][RES_WOFF*XLEN +: XLEN];
        else if (CCD.G_AGENT[3].L1D.state_q[RES_SET] != CMI_I)
            v = CCD.G_AGENT[3].L1D.data_q[RES_SET][RES_WOFF*XLEN +: XLEN];
`endif
        read_result = v;
    endfunction

    int errors=0, timeout;
    logic [XLEN-1:0] res;
    task automatic chk(input bit ok, input string what);
        if (!ok) begin $display("  [FAIL] %s", what); errors++; end else $display("  [ ok ] %s", what);
    endtask

    // ===== the assembled SMC litmus (linked @USER_TEXT_START; RV32==RV64 word stream) =====
    logic [31:0] prog [0:NPROG-1];
    logic [XLEN-1:0] busword [0:NPROG-1];   // packed bus words (RV64 = 2 insns/word)
    initial begin
        prog[ 0]=32'hf1402473; prog[ 1]=32'h02041663; prog[ 2]=32'h00000297; prog[ 3]=32'h03828293;
        prog[ 4]=32'h22200337; prog[ 5]=32'h51330313; prog[ 6]=32'h0062a023; prog[ 7]=32'h0330000f;
        prog[ 8]=32'h00100393; prog[ 9]=32'h10000e13; prog[10]=32'h007e2023; prog[11]=32'h0000006f;
        prog[12]=32'h10000e13; prog[13]=32'h000e2e83; prog[14]=32'hfe0e8ee3; prog[15]=32'h0040006f;
        prog[16]=32'h11100513; prog[17]=32'h14000f13; prog[18]=32'h00af2023; prog[19]=32'h0000006f;
        prog[20]=32'h00000013; prog[21]=32'h00000013; prog[22]=32'h00000013; prog[23]=32'h00000013;
        prog[24]=32'h00000013; prog[25]=32'h00000013; prog[26]=32'h00000013; prog[27]=32'h00000013;
        prog[28]=32'h00000013; prog[29]=32'h00000013; prog[30]=32'h00000013; prog[31]=32'h00000013;
        prog[32]=32'h00000013;
        // pack into XLEN-wide bus words: RV32 = 1 insn/word; RV64 = 2 insns/word (lo insn in lo half).
        for (int i=0;i<NPROG;i++) busword[i] = '0;
`ifdef RV64
        for (int k=0;k<(NPROG+1)/2;k++)
            busword[k] = { (2*k+1<NPROG ? prog[2*k+1] : 32'h0), prog[2*k] };
`else
        for (int k=0;k<NPROG;k++) busword[k] = prog[k];
`endif
        // seed MEM line-by-line at the text base (each bus word -> its line + sub-offset)
        begin
            for (int j=0;j<NW;j++) begin
                automatic logic [MEMORY_ADDR_WIDTH-1:0] wa  = TEXT_BASE_W + MEMORY_ADDR_WIDTH'(j);
                automatic logic [MEMORY_ADDR_WIDTH-1:0] lb  = wa & ~MEMORY_ADDR_WIDTH'(LINE_WORDS-1);
                automatic int                           so  = int'(wa[LINE_WORD_BITS-1:0]);
                automatic logic [LINE_BITS-1:0]         ln  = MEM.exists(lb) ? MEM[lb] : '0;
                ln[so*XLEN +: XLEN] = busword[j];
                MEM[lb] = ln;
            end
        end
        for (int c=0;c<NCORE;c++) begin creq_v[c]=0; creq_op[c]=COP_LOAD; creq_amo[c]=AMO_ADD; creq_wm[c]='1; end
        rst_l=0; repeat(8) @(posedge clk); rst_l=1;

        $display("== P4 SMC: hart 0 patches a code line, hart %0d (cold) fetches it via the directory ==", NCORE-1);
        $display("   patched insn 'addi a0,x0,0x222' -> RESULT(0x140) should read 0x222 (stale image = 0x111)");
        timeout=0;
        do begin @(posedge clk); res=read_result(); timeout++; end
        while (res != RESULT_RIGHT && res != RESULT_WRONG && timeout < 80000);
        res = read_result();

        chk(res != 0, "consumer reached the patched code and wrote RESULT");
        chk(res != RESULT_WRONG, "consumer did NOT fetch the stale image (0x111)");
        chk(res == RESULT_RIGHT, "REMOTE-DIRTY I-FETCH: consumer ran hart 0's PATCHED insn (RESULT=0x222)");
        // Airtight discriminator: backing memory STILL holds the original word (the patch
        // never reached it -- it lived dirty in hart 0's L1D), so the consumer's 0x222 can
        // ONLY have come from coherent forwarding, not a stale/lucky memory read.
        chk(!MEM.exists(PATCH_LB) || MEM[PATCH_LB][31:0] == 32'h11100513,
            "backing memory unchanged (0x222 came from coherent forwarding, not memory)");

        $display("");
        if (errors==0)
            $display("==== tb_ccd_smc: PASS -- cross-core SMC coherent (RESULT=0x%0h) ====", res);
        else
            $display("==== tb_ccd_smc: %0d CHECK(S) FAILED (RESULT=0x%0h) ====", errors, res);
        $finish;
    end

    initial begin repeat(160000) @(posedge clk); $display("WATCHDOG TIMEOUT (RESULT=0x%0h)", read_result()); $finish; end
endmodule
`default_nettype wire
