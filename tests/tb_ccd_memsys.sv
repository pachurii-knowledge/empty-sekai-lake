// tb_ccd_memsys.sv -- P4-incr2: validates the reusable niigo_ccd_memsys multi-core memory
// subsystem by running the SAME cross-core SMC litmus as tb_ccd_smc, but with the per-core
// arm (L1I + launch adapter + iref + probe) FACTORED INTO the module instead of inline. NCORE
// real riscv_core_ooo connect to niigo_ccd_memsys #(.NACTIVE); the module's single NMI master
// drives a behavioural line memory seeded with the program. RESULT(0x140)==0x222 iff hart 0's
// freshly-patched (dirty) code line is fetched coherently by the cold-fetching consumer.
// This is the module the xv6-SMP harness instantiates (PTW + device-bypass paths included).
// Build/run: make ccd-memsys-test (RV32) | make ccd-memsys-rv64-test (RV64)
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
    localparam int NCORE = 4;
`else
    localparam int NCORE = 2;
`endif
    localparam int ADDR_SHIFT = $clog2(XLEN_BYTES);
    localparam int LINE_WORDS = LINE_BITS / XLEN;
    localparam int NPROG = 33;
`ifdef RV64
    localparam int NW = (NPROG+1)/2;
`else
    localparam int NW = NPROG;
`endif
    logic clk=0, rst_l=0;
    always #5 clk=~clk;
    int              cycle_count = 0;   // register_file.sv XMR ($root.top.cycle_count/.pc)
    logic [XLEN-1:0] pc = '0;
    always_ff @(posedge clk) cycle_count <= cycle_count + 1;

    localparam logic [MEMORY_ADDR_WIDTH-1:0] TEXT_BASE_W = USER_TEXT_START[XLEN-1:ADDR_SHIFT];
    localparam logic [XLEN-1:0] RESULT_RIGHT = XLEN'('h222);
    localparam logic [XLEN-1:0] RESULT_WRONG = XLEN'('h111);

    // ===== per-core <-> memsys arrays =====
    logic                          if_req_v[NCORE], if_req_r[NCORE], if_resp_v[NCORE], if_resp_e[NCORE], if_inval[NCORE];
    logic [MEMORY_ADDR_WIDTH-1:0]  if_req_a[NCORE];
    logic [MEMORY_READ_WIDTH-1:0][XLEN-1:0] if_resp_d[NCORE];
    logic                          d_req_v[NCORE], d_req_r[NCORE], d_req_w[NCORE], d_dev[NCORE], d_resp_v[NCORE];
    logic [MEMORY_ADDR_WIDTH-1:0]  d_req_a[NCORE], d_resp_a[NCORE];
    logic [XLEN-1:0]               d_req_wd[NCORE], d_resp_d[NCORE];
    logic [XLEN_BYTES-1:0]         d_req_wm[NCORE];
    logic [2:0]                    d_req_op[NCORE];
    logic [3:0]                    d_req_amo[NCORE];
    logic                          sk_v[NCORE]; logic [MEMORY_ADDR_WIDTH-1:0] sk_la[NCORE];
    logic                          pt_req[NCORE], pt_we[NCORE], pt_ack[NCORE];
    logic [MEMORY_ADDR_WIDTH-1:0]  pt_aw[NCORE]; logic [XLEN-1:0] pt_wd[NCORE], pt_rd[NCORE];
    logic                          dcflush_req[NCORE], dcflush_done[NCORE];
    logic                          hpm_im[NCORE], hpm_dm[NCORE], hpm_dw[NCORE];
    nmi_req_t    mreq; logic mreq_ready; nmi_resp_t mresp;

    // ===== NCORE real cores =====
    genvar g;
    generate for (g=0; g<NCORE; g++) begin : CORE
        logic halted_c;
        riscv_core #(.HART_ID(g[XLEN-1:0]), .COHERENT(1'b1)) Core (
            .clk, .rst_l,
            .ifetch_req_valid(if_req_v[g]), .ifetch_req_ready(if_req_r[g]), .ifetch_req_addr(if_req_a[g]),
            .ifetch_resp_valid(if_resp_v[g]), .ifetch_resp_data(if_resp_d[g]), .ifetch_resp_excpt(if_resp_e[g]),
            .dmem_req_valid(d_req_v[g]), .dmem_req_ready(d_req_r[g]), .dmem_req_write(d_req_w[g]), .dmem_req_addr(d_req_a[g]),
            .dmem_req_wdata(d_req_wd[g]), .dmem_req_wmask(d_req_wm[g]), .dmem_req_op(d_req_op[g]), .dmem_req_amo(d_req_amo[g]),
            .dmem_resp_valid(d_resp_v[g]), .dmem_resp_addr(d_resp_a[g]), .dmem_resp_data(d_resp_d[g]),
            .dmem_snoop_kill_valid(sk_v[g]), .dmem_snoop_kill_laddr(sk_la[g]),
            .ptw_mem_req(pt_req[g]), .ptw_mem_we(pt_we[g]), .ptw_mem_addr_w(pt_aw[g]), .ptw_mem_wdata(pt_wd[g]),
            .ptw_mem_ack(pt_ack[g]), .ptw_mem_rdata(pt_rd[g]),
            .ifetch_inval(if_inval[g]), .dmem_req_device(d_dev[g]),
            .dcache_flush_req(dcflush_req[g]), .dcache_flush_done(dcflush_done[g]),
            .hpm_l1i_miss(hpm_im[g]), .hpm_l1d_miss(hpm_dm[g]), .hpm_l1d_wb(hpm_dw[g]),
            .halted(halted_c));
        wire unused = halted_c;
    end endgenerate

    // ===== the reusable multi-core coherent memsys under test =====
    niigo_ccd_memsys #(.NACTIVE(NCORE), .L1_SETS(64), .DIR_SETS(256), .RESP_DLY(2)) MEMSYS (
        .clk, .rst_l,
        .ifetch_req_valid(if_req_v), .ifetch_req_ready(if_req_r), .ifetch_req_addr(if_req_a),
        .ifetch_resp_valid(if_resp_v), .ifetch_resp_data(if_resp_d), .ifetch_resp_excpt(if_resp_e),
        .ifetch_inval(if_inval),
        .dmem_req_valid(d_req_v), .dmem_req_ready(d_req_r), .dmem_req_write(d_req_w), .dmem_req_addr(d_req_a),
        .dmem_req_wdata(d_req_wd), .dmem_req_wmask(d_req_wm), .dmem_req_op(d_req_op), .dmem_req_amo(d_req_amo),
        .dmem_req_device(d_dev),
        .dmem_resp_valid(d_resp_v), .dmem_resp_addr(d_resp_a), .dmem_resp_data(d_resp_d),
        .dmem_snoop_kill_valid(sk_v), .dmem_snoop_kill_laddr(sk_la),
        .ptw_req_valid(pt_req), .ptw_req_we(pt_we), .ptw_req_addr(pt_aw), .ptw_req_wdata(pt_wd),
        .ptw_req_ack(pt_ack), .ptw_resp_rdata(pt_rd),
        .dcache_flush_req(dcflush_req), .dcache_flush_done(dcflush_done),
        .hpm_l1i_miss(hpm_im), .hpm_l1d_miss(hpm_dm), .hpm_l1d_wb(hpm_dw),
        .mem_req_o(mreq), .mem_req_ready_i(mreq_ready), .mem_resp_i(mresp));

    // ===== shared sparse NMI line memory (backend; seeded with the program) =====
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

    // ===== RESULT readout: white-box from whichever agent owns the RESULT line =====
    localparam logic [MEMORY_ADDR_WIDTH-1:0] PATCH_W  = (USER_TEXT_START + 'h40) >> ADDR_SHIFT;
    localparam logic [MEMORY_ADDR_WIDTH-1:0] PATCH_LB = PATCH_W & ~MEMORY_ADDR_WIDTH'(LINE_WORDS-1);
    localparam logic [MEMORY_ADDR_WIDTH-1:0] RESULT_W = MEMORY_ADDR_WIDTH'('h140 >> ADDR_SHIFT);
    localparam int L1_IDX = $clog2(64);
    localparam logic [L1_IDX-1:0]         RES_SET  = RESULT_W[LINE_WORD_BITS +: L1_IDX];
    localparam logic [LINE_WORD_BITS-1:0] RES_WOFF = RESULT_W[LINE_WORD_BITS-1:0];
    function automatic logic [XLEN-1:0] read_result;
        logic [XLEN-1:0] v; v = '0;
        if (MEMSYS.CCD.G_AGENT[0].L1D.state_q[RES_SET] != CMI_I)
            v = MEMSYS.CCD.G_AGENT[0].L1D.data_q[RES_SET][RES_WOFF*XLEN +: XLEN];
        else if (MEMSYS.CCD.G_AGENT[1].L1D.state_q[RES_SET] != CMI_I)
            v = MEMSYS.CCD.G_AGENT[1].L1D.data_q[RES_SET][RES_WOFF*XLEN +: XLEN];
`ifdef NCORE4
        else if (MEMSYS.CCD.G_AGENT[2].L1D.state_q[RES_SET] != CMI_I)
            v = MEMSYS.CCD.G_AGENT[2].L1D.data_q[RES_SET][RES_WOFF*XLEN +: XLEN];
        else if (MEMSYS.CCD.G_AGENT[3].L1D.state_q[RES_SET] != CMI_I)
            v = MEMSYS.CCD.G_AGENT[3].L1D.data_q[RES_SET][RES_WOFF*XLEN +: XLEN];
`endif
        read_result = v;
    endfunction

    int errors=0, timeout;
    logic [XLEN-1:0] res;
    task automatic chk(input bit ok, input string what);
        if (!ok) begin $display("  [FAIL] %s", what); errors++; end else $display("  [ ok ] %s", what);
    endtask

    logic [31:0] prog [0:NPROG-1];
    logic [XLEN-1:0] busword [0:NPROG-1];
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
        for (int i=0;i<NPROG;i++) busword[i] = '0;
`ifdef RV64
        for (int k=0;k<NW;k++) busword[k] = { (2*k+1<NPROG ? prog[2*k+1] : 32'h0), prog[2*k] };
`else
        for (int k=0;k<NW;k++) busword[k] = prog[k];
`endif
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
        rst_l=0; repeat(8) @(posedge clk); rst_l=1;

        $display("== P4 niigo_ccd_memsys: %0d cores, SMC via the FACTORED module ==", NCORE);
        timeout=0;
        do begin @(posedge clk); res=read_result(); timeout++; end
        while (res != RESULT_RIGHT && res != RESULT_WRONG && timeout < 80000);
        res = read_result();

        chk(res != 0, "consumer reached the patched code and wrote RESULT");
        chk(res != RESULT_WRONG, "consumer did NOT fetch the stale image (0x111)");
        chk(res == RESULT_RIGHT, "REMOTE-DIRTY I-FETCH (module): consumer ran hart 0's PATCHED insn (0x222)");
        chk(!MEM.exists(PATCH_LB) || MEM[PATCH_LB][31:0] == 32'h11100513,
            "backing memory unchanged (0x222 came from coherent forwarding, not memory)");

        $display("");
        if (errors==0)
            $display("==== tb_ccd_memsys: PASS -- niigo_ccd_memsys reproduces coherent cross-core SMC (RESULT=0x%0h) ====", res);
        else
            $display("==== tb_ccd_memsys: %0d CHECK(S) FAILED (RESULT=0x%0h) ====", errors, res);
        $finish;
    end

    initial begin repeat(160000) @(posedge clk); $display("WATCHDOG TIMEOUT (RESULT=0x%0h)", read_result()); $finish; end
endmodule
`default_nettype wire
