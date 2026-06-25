// tb_niigo_dir.sv -- directed conformance testbench for the M1 MOESI directory.
// Drives 2 L1D agents (cores 0,1) over the M1 full-line CMI links + a behavioural NMI
// memory, and checks the directory's observable protocol on the canonical transactions:
//   cold GetS->E, second GetS (cache-to-cache, E->S), GetM+Inv (M handoff), store/M,
//   read of a dirty line (M->O), and PutM (writeback to memory).
// Build:  verilator --binary -DLAB_18447='"4b"' -Isrc -Isrc/mem tests/tb_niigo_dir.sv --top tb_niigo_dir
`include "niigo_mem.vh"
`include "niigo_cmi.vh"
`include "niigo_ccd_m1.vh"
`default_nettype none
module tb_niigo_dir
    import RISCV_UArch::MEMORY_ADDR_WIDTH;
    import NIIGO_Mem::*;
    import NIIGO_CMI::*;
    import NIIGO_CCD_M1::*;
;
    localparam int CORES = NIIGO_CMI::NUM_CORES;   // 4 (M1 drives 0,1)

    logic clk = 0, rst_l = 0;
    always #5 clk = ~clk;

    ccd_chan_t up   [CORES];
    logic      up_ready  [CORES];
    ccd_chan_t down [CORES];
    logic      down_ready[CORES];
    nmi_req_t  mreq; logic mreq_ready; nmi_resp_t mresp;

    // tie all up channels idle by default; the test drives 0/1 explicitly
    initial for (int c=0;c<CORES;c++) begin up[c]='0; down_ready[c]=1'b1; end

    niigo_dir #(.CORES(CORES), .DIR_SETS(8)) dut (
        .clk, .rst_l,
        .up_i(up), .up_ready_o(up_ready), .down_o(down), .down_ready_i(down_ready),
        .mem_req_o(mreq), .mem_req_ready_i(mreq_ready), .mem_resp_i(mresp)
    );

    // ---- behavioural NMI memory: 16 lines, 1-cycle latency, always ready ----
    localparam int MEMLINES = 16;
    logic [LINE_BITS-1:0] MEM [MEMLINES];
    assign mreq_ready = 1'b1;
    always_ff @(posedge clk or negedge rst_l) begin
        if (!rst_l) mresp <= '0;
        else begin
            mresp <= '0;
            if (mreq.valid) begin
                if (mreq.op == NMI_WR_LINE) MEM[mreq.waddr[$clog2(MEMLINES)-1:0]] <= mreq.wdata;
                mresp.valid <= 1'b1;
                mresp.rdata <= MEM[mreq.waddr[$clog2(MEMLINES)-1:0]];
            end
        end
    end

    int errors = 0;
    task automatic chk(input bit ok, input string what);
        if (!ok) begin $display("  [FAIL] %s", what); errors++; end
        else        $display("  [ ok ] %s", what);
    endtask

    // drive a request from `core` and wait until the dir accepts it (up_ready pulse)
    task automatic put_req(input int core, input cmi_op_e op,
                           input logic [MEMORY_ADDR_WIDTH-1:0] la,
                           input logic [LINE_BITS-1:0] line);
        @(negedge clk);
        up[core].valid     = 1'b1;
        up[core].msg       = '0;
        up[core].msg.op    = op;
        up[core].msg.src   = core[CORE_ID_W-1:0];
        up[core].msg.laddr = la;
        up[core].msg.line  = line;
        // hold until accepted
        do @(posedge clk); while (!up_ready[core]);
        @(negedge clk); up[core].valid = 1'b0;
    endtask

    // wait for a down message of `op` to `core`; return it (consumed via down_ready=1)
    task automatic get_down(input int core, output ccd_msg_t m);
        int guard; guard = 0;
        while (!(down[core].valid)) begin @(posedge clk); guard++; if (guard>100) begin
            $display("  [FAIL] timeout waiting for down to core %0d", core); errors++; m='0; return; end end
        m = down[core].msg;
        @(negedge clk); // let the dir see down_ready (tied 1) and advance
    endtask

    // respond to a forwarded snoop: owner sends DATA(line,onext) up to the dir
    task automatic fwd_data(input int core, input logic [LINE_BITS-1:0] line, input cmi_owner_next_e on);
        @(negedge clk);
        up[core].valid     = 1'b1;
        up[core].msg       = '0;
        up[core].msg.op    = OP_DATA;
        up[core].msg.src   = core[CORE_ID_W-1:0];
        up[core].msg.onext = on;
        up[core].msg.line  = line;
        do @(posedge clk); while (!up_ready[core]);
        @(negedge clk); up[core].valid = 1'b0;
    endtask

    // respond to an Inv: sharer sends InvAck up to the dir, addressed to `req`
    task automatic inv_ack(input int core, input int req);
        @(negedge clk);
        up[core].valid   = 1'b1;
        up[core].msg     = '0;
        up[core].msg.op  = OP_INV_ACK;
        up[core].msg.src = core[CORE_ID_W-1:0];
        up[core].msg.req = req[CORE_ID_W-1:0];
        do @(posedge clk); while (!up_ready[core]);
        @(negedge clk); up[core].valid = 1'b0;
    endtask

    ccd_msg_t m;
    localparam logic [MEMORY_ADDR_WIDTH-1:0] A = 'h20;   // a line addr
    localparam logic [LINE_BITS-1:0] V0 = {16{32'hA5A5_0000}};
    localparam logic [LINE_BITS-1:0] V1 = {16{32'h1234_5678}};

    initial begin
        for (int i=0;i<MEMLINES;i++) MEM[i] = V0;        // memory preloaded with V0 at A
        rst_l = 0; repeat (4) @(posedge clk); rst_l = 1; @(posedge clk);

        $display("== S1: core0 GetS to a cold line -> expect E grant, data from memory ==");
        fork put_req(0, OP_GETS, A, '0); join_none
        get_down(0, m);
        chk(m.op==OP_DATA && m.gst==CMI_E, "S1: core0 gets DATA(E)");
        chk(m.line==V0,                    "S1: data == memory V0");
        wait_idle();

        $display("== S2: core1 GetS -> cache-to-cache from core0 (E->S) ==");
        fork put_req(1, OP_GETS, A, '0); join_none
        get_down(0, m);                                  // dir forwards to owner core0
        chk(m.op==OP_FWD_GETS && m.req==2'd1, "S2: core0 sees FwdGetS(req=1)");
        fwd_data(0, V0, ON_S);                            // core0 was E -> supplies clean, ON_S
        get_down(1, m);
        chk(m.op==OP_DATA && m.gst==CMI_S, "S2: core1 gets DATA(S)");
        chk(m.line==V0,                    "S2: forwarded data == V0");
        wait_idle();

        $display("== S3: core0 GetM (it is a sharer) -> Inv core1, grant M ==");
        fork put_req(0, OP_UPGRADE, A, '0); join_none     // core0 holds S -> Upgrade
        get_down(1, m);
        chk(m.op==OP_INV && m.req==2'd0, "S3: core1 sees Inv(req=0)");
        inv_ack(1, 0);
        get_down(0, m);
        chk(m.op==OP_DATA && m.gst==CMI_M, "S3: core0 gets grant M");
        wait_idle();

        $display("== S4: core1 GetS -> cache-to-cache from core0 which is now M (M->O) ==");
        fork put_req(1, OP_GETS, A, '0); join_none
        get_down(0, m);
        chk(m.op==OP_FWD_GETS && m.req==2'd1, "S4: core0 sees FwdGetS");
        fwd_data(0, V1, ON_O);                            // core0 is M (dirty V1) -> keeps O, ON_O
        get_down(1, m);
        chk(m.op==OP_DATA && m.gst==CMI_S, "S4: core1 gets DATA(S)");
        chk(m.line==V1,                    "S4: gets the DIRTY value V1 (cache-to-cache)");
        wait_idle();

        $display("== S5: core0 PutO (evict the owned/dirty line) -> writeback V1 to memory ==");
        fork put_req(0, OP_PUTO, A, V1); join_none
        get_down(0, m);
        chk(m.op==OP_ACK, "S5: core0 gets WBAck");
        repeat (3) @(posedge clk);
        chk(MEM[A[$clog2(MEMLINES)-1:0]]==V1, "S5: memory now holds the written-back V1");
        wait_idle();

        $display("== S6: core0 GetS again -> line now S at core1 only; memory current -> S grant ==");
        fork put_req(0, OP_GETS, A, '0); join_none
        get_down(0, m);
        chk(m.op==OP_DATA && m.gst==CMI_S, "S6: core0 gets DATA(S) from memory");
        chk(m.line==V1,                    "S6: data == V1 (memory was refreshed)");
        wait_idle();

        $display("");
        if (errors==0) $display("==== tb_niigo_dir: ALL CHECKS PASSED ====");
        else           $display("==== tb_niigo_dir: %0d CHECK(S) FAILED ====", errors);
        $finish;
    end

    task automatic wait_idle(); repeat (3) @(posedge clk); endtask

    // global watchdog
    initial begin repeat (4000) @(posedge clk); $display("WATCHDOG TIMEOUT"); $finish; end
endmodule
`default_nettype wire
