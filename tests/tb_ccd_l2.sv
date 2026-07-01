// tb_ccd_l2.sv -- directed unit test for niigo_l2 (transparent write-back NINE L2).
//
// Drives the L2's NMI slave port with RD_LINE/WR_LINE sequences against a
// behavioural, per-request pseudo-random-latency NMI backing memory (line
// addressed). Checks (plans/l2-integration.md Inc 1):
//   1. cold RD miss -> fill -> 2nd RD of the same line HITS (no downstream op).
//   2. WR_LINE then RD_LINE of the same line returns the WRITTEN data.
//   3. dirty victim is written back to memory before being dropped, and a later
//      RD of the evicted line returns the written value (round-trip durability).
//   4. WR_LINE miss write-allocates with NO fill read.
//   5. held-valid (directory discipline) -> exactly ONE downstream op + ONE resp.
// The random backing latency proves the L2 is latency-agnostic.
//
// PASS = "L2-TB: ALL CHECKS PASSED".
`timescale 1ns/1ps
`include "niigo_mem.vh"
`default_nettype none

module tb_ccd_l2
    import RISCV_UArch::MEMORY_ADDR_WIDTH;
    import NIIGO_Mem::*;
;
    localparam int MEMW = MEMORY_ADDR_WIDTH;
    localparam int LB   = LINE_BITS;
    localparam int SETS = 16;                 // small L2 for easy eviction (WAYS fixed 8)
    localparam int WAYS = 8;
    localparam int IDX  = $clog2(SETS);       // 4
    localparam int LWB  = LINE_WORD_BITS;

    logic clk = 1'b0;
    always #5 clk = ~clk;
    logic rst_l;

    // ---- DUT wiring ----
    nmi_req_t  s_req;   logic s_req_ready;  nmi_resp_t s_resp;
    nmi_req_t  m_req;   logic m_req_ready;  nmi_resp_t m_resp;

    niigo_l2 #(.SETS(SETS), .WAYS(WAYS)) DUT (
        .clk, .rst_l,
        .s_req(s_req), .s_req_ready(s_req_ready), .s_resp(s_resp),
        .m_req(m_req), .m_req_ready(m_req_ready), .m_resp(m_resp)
    );

    // ================= behavioural NMI backing memory (line-addressed) =================
    logic [LB-1:0] backing [logic [MEMW-1:0]];

    // deterministic "memory" contents for a never-written line
    function automatic logic [LB-1:0] golden(input logic [MEMW-1:0] a);
        golden = '0;
        for (int k = 0; k < LB/32; k++)
            golden[k*32 +: 32] = (32'(a) + k*32'h1000_0000) ^ 32'hA5A5_0000;
    endfunction
    function automatic logic [LB-1:0] rd_backing(input logic [MEMW-1:0] a);
        rd_backing = backing.exists(a) ? backing[a] : golden(a);
    endfunction

    typedef enum logic [1:0] { B_IDLE, B_WAIT, B_RESP } bstate_e;
    bstate_e bst;
    logic [7:0]      bdelay;
    logic [3:0]      bid;
    logic [LB-1:0]   brdata;
    logic [31:0]     blcg;
    function automatic logic [31:0] lcg_next(input logic [31:0] s);
        lcg_next = s * 32'd1664525 + 32'd1013904223;
    endfunction

    assign m_req_ready = (bst == B_IDLE);

    // downstream op monitors + last-writeback capture
    int dn_rd = 0, dn_wr = 0;
    logic [MEMW-1:0] last_wb_addr;
    logic [LB-1:0]   last_wb_data;

    always_ff @(posedge clk or negedge rst_l) begin
        if (!rst_l) begin
            bst <= B_IDLE; bdelay <= '0; bid <= '0; brdata <= '0;
            blcg <= 32'hC0FFEE11;
        end else begin
            unique case (bst)
                B_IDLE: if (m_req.valid) begin
                    automatic logic [7:0] d = blcg[10:8];   // 0..7 cycles
                    bid    <= m_req.id;
                    brdata <= rd_backing(m_req.waddr);
                    blcg   <= lcg_next(blcg);
                    if (d == 0) bst <= B_RESP;
                    else begin bdelay <= d; bst <= B_WAIT; end
                end
                B_WAIT: if (bdelay <= 8'd1) bst <= B_RESP; else bdelay <= bdelay - 8'd1;
                B_RESP: bst <= B_IDLE;
                default: bst <= B_IDLE;
            endcase
        end
    end

    // Backing store write + downstream monitors (blocking: assoc arrays need
    // blocking writes; kept in a separate block from the nonblocking FSM state).
    always @(posedge clk) begin
        if (rst_l && (bst == B_IDLE) && m_req.valid) begin
            if (m_req.op == NMI_WR_LINE) begin
                backing[m_req.waddr] = m_req.wdata;
                dn_wr        = dn_wr + 1;
                last_wb_addr = m_req.waddr;
                last_wb_data = m_req.wdata;
            end
            if (m_req.op == NMI_RD_LINE) dn_rd = dn_rd + 1;
        end
    end
    always_comb begin
        m_resp       = '0;
        m_resp.valid = (bst == B_RESP);
        m_resp.id    = bid;
        m_resp.rdata = brdata;
        m_resp.err   = 1'b0;
    end

    // resp-pulse monitor
    int resp_cnt = 0;
    always_ff @(posedge clk) if (rst_l && s_resp.valid) resp_cnt <= resp_cnt + 1;

    // ================= line-address helper =================
    // LA(tag, set) = line-base word address selecting a given (tag,set).
    function automatic logic [MEMW-1:0] LA(input int tag, input int set);
        LA = (MEMW'(tag) << (IDX + LWB)) | (MEMW'(set) << LWB);
    endfunction

    // ================= driver =================
    int errors = 0;
    task automatic chk(input logic cond, input string msg);
        if (!cond) begin $display("FAIL: %s", msg); errors++; end
    endtask

    // Standard op: assert valid until accepted, deassert, wait for resp.
    task automatic do_op(input logic wr, input logic [MEMW-1:0] addr,
                         input logic [LB-1:0] wdata, output logic [LB-1:0] rdata);
        @(negedge clk);
        s_req.valid <= 1'b1;
        s_req.op    <= wr ? NMI_WR_LINE : NMI_RD_LINE;
        s_req.waddr <= addr;
        s_req.wdata <= wdata;
        s_req.id    <= 4'h5;
        s_req.wmask <= '0;
        forever begin @(posedge clk); if (s_req_ready) break; end   // accepted this edge
        @(negedge clk); s_req.valid <= 1'b0;
        forever begin @(posedge clk); if (s_resp.valid) begin rdata = s_resp.rdata; break; end end
    endtask

    task automatic do_wr(input logic [MEMW-1:0] addr, input logic [LB-1:0] data);
        logic [LB-1:0] junk; do_op(1'b1, addr, data, junk);
    endtask
    task automatic do_rd_chk(input logic [MEMW-1:0] addr, input logic [LB-1:0] exp, input string msg);
        logic [LB-1:0] got; do_op(1'b0, addr, '0, got);
        chk(got === exp, msg);
    endtask

    // Held-valid op (directory discipline): hold valid high until the resp cycle,
    // deassert only then. Must yield exactly one downstream op + one resp.
    task automatic do_rd_held(input logic [MEMW-1:0] addr, input logic [LB-1:0] exp, input string msg);
        int r0, d0; logic [LB-1:0] got;
        @(negedge clk);                    // settle after any prior op's resp posedge
        r0 = resp_cnt; d0 = dn_rd;
        s_req.valid <= 1'b1; s_req.op <= NMI_RD_LINE; s_req.waddr <= addr;
        s_req.wdata <= '0; s_req.id <= 4'h5; s_req.wmask <= '0;
        forever begin @(posedge clk); if (s_resp.valid) begin got = s_resp.rdata; break; end end
        @(negedge clk); s_req.valid <= 1'b0;   // deassert on the resp cycle, like the directory
        repeat (4) @(posedge clk);             // settle; catch any spurious re-accept
        chk(got === exp,                 {msg, ": data"});
        chk((resp_cnt - r0) == 1,        {msg, ": exactly one resp pulse"});
        chk((dn_rd  - d0)  == 1,         {msg, ": exactly one downstream RD"});
    endtask

    // ================= test body =================
    logic [LB-1:0] a0d, a1d, xd, zd;
    int rd0, wr0;
    initial begin
        s_req = '0; rst_l = 1'b0;
        repeat (4) @(negedge clk); rst_l = 1'b1;
        repeat (2) @(negedge clk);

        // ---- 1. cold RD miss -> fill; 2nd RD hits (no downstream) ----
        a0d = golden(LA(1,3));
        rd0 = dn_rd; wr0 = dn_wr;
        do_rd_chk(LA(1,3), a0d, "1a cold RD returns golden");
        chk((dn_rd-rd0)==1 && (dn_wr-wr0)==0, "1b cold RD = one fill, no WB");
        rd0 = dn_rd;
        do_rd_chk(LA(1,3), a0d, "1c re-RD returns same");
        chk((dn_rd-rd0)==0, "1d re-RD is a HIT (no downstream)");

        // ---- 2. WR then RD same line -> written data ----
        a1d = {16{32'hDEAD_0002}};
        rd0 = dn_rd; wr0 = dn_wr;
        do_wr(LA(2,5), a1d);
        chk((dn_rd-rd0)==0 && (dn_wr-wr0)==0, "2a WR cold = allocate, no fill/WB");
        do_rd_chk(LA(2,5), a1d, "2b RD after WR returns written data");

        // ---- 3. dirty victim writeback durability ----
        xd = {16{32'hBEEF_0009}};
        do_wr(LA(9,7), xd);                       // X dirty in the L2 (mem still golden)
        for (int t = 100; t < 116; t++)           // 16 distinct aliasing lines -> X evicted for sure
            do_rd_chk(LA(t,7), golden(LA(t,7)), "3a sweep fill");
        chk(backing.exists(LA(9,7)) && backing[LA(9,7)] === xd,
            "3b evicted dirty line written back to memory with correct data");
        do_rd_chk(LA(9,7), xd, "3c re-RD of evicted line returns written data (round-trip)");

        // ---- 4. WR miss write-allocate, NO fill read ----
        zd = {16{32'h1234_0003}};
        rd0 = dn_rd; wr0 = dn_wr;
        do_wr(LA(3,11), zd);
        chk((dn_rd-rd0)==0, "4a WR miss does NOT fill-read");
        chk((dn_wr-wr0)==0, "4b WR miss (clean victim) does NOT write back");
        do_rd_chk(LA(3,11), zd, "4c RD returns write-allocated data");

        // ---- 5. held-valid -> exactly one op + one resp ----
        do_rd_held(LA(4,13), golden(LA(4,13)), "5 held-valid single op");

        if (errors == 0) $display("L2-TB: ALL CHECKS PASSED");
        else             $display("L2-TB: %0d FAILURES", errors);
        $finish;
    end

    // watchdog
    initial begin
        repeat (200000) @(posedge clk);
        $display("L2-TB: TIMEOUT"); $finish;
    end
endmodule
`default_nettype wire
