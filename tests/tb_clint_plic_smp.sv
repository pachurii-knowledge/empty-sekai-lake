// tb_clint_plic_smp.sv -- M4-S6a: directed test of the multi-hart shared CLINT/PLIC.
// Instantiates ONE clint #(.NUM_HARTS(4)) and ONE plic #(.NCTX(8)) (= 4 harts, 2
// contexts each) and drives them through the same word-granular store/load snoop
// ports the cores use, checking per-hart / per-context isolation + the cross-hart
// IPI mechanism (one hart writes another hart's msip). RV32 (default build).
// Build/run: make clint-plic-smp-test
`include "riscv_isa.vh"
`include "riscv_uarch.vh"
`default_nettype none
module top
    import RISCV_ISA::XLEN, RISCV_ISA::XLEN_BYTES;
    import RISCV_UArch::MEMORY_ADDR_WIDTH;
;
    localparam int ADDR_SHIFT = $clog2(XLEN_BYTES);
    localparam int MAW = MEMORY_ADDR_WIDTH;
    localparam int NP  = 4;                 // one load port per hart
    localparam logic [31:0] CLINT_BASE = 32'h0200_0000;
    localparam logic [31:0] PLIC_BASE  = 32'h0C00_0000;

    logic clk = 0, rst_l = 0;
    always #5 clk = ~clk;

    // ---- shared store snoop + NP-wide load query bus (cores drive these) ----
    logic                          st_en;
    logic [MEMORY_ADDR_WIDTH-1:0]  st_wa;
    logic [XLEN-1:0]               st_wd;
    logic [XLEN_BYTES-1:0]         st_wm;
    logic [NP*MAW-1:0]             ld_a;        // NP packed load ports
    logic [NP-1:0]                 ld_en;
    logic [NP*ADDR_SHIFT-1:0]      ld_off;

    logic [NP-1:0]       cl_hit;  logic [NP*XLEN-1:0] cl_data;
    logic [3:0]          cl_mtip, cl_msip;   // NUM_HARTS=4
    logic [63:0]         cl_mtime;
    logic [NP-1:0]       pl_hit;  logic [NP*XLEN-1:0] pl_data;
    logic [3:0]          pl_mext, pl_sext;   // NCTX/2 = 4 harts

    clint #(.NUM_HARTS(4), .NPORT(NP)) CL (
        .clk, .rst_l,
        .store_en(st_en), .store_waddr(st_wa), .store_wdata(st_wd), .store_mask(st_wm),
        .load_addr(ld_a), .load_hit(cl_hit), .load_data(cl_data),
        .irq_m_timer(cl_mtip), .irq_m_software(cl_msip), .mtime_out(cl_mtime));

    plic #(.NCTX(8), .NSOURCES(31), .NPORT(NP)) PL (
        .clk, .rst_l, .src_irq('0),
        .store_en(st_en), .store_waddr(st_wa), .store_wdata(st_wd), .store_mask(st_wm),
        .load_addr(ld_a), .load_en(ld_en), .load_off(ld_off),
        .load_hit(pl_hit), .load_data(pl_data),
        .irq_m_external(pl_mext), .irq_s_external(pl_sext));

    int errors = 0;
    task automatic chk(input bit ok, input string what);
        if (!ok) begin $display("  [FAIL] %s", what); errors++; end
        else $display("  [ ok ] %s", what);
    endtask

    // 32-bit store at a BYTE address (RV32: one reg per bus word).
    task automatic st32(input logic [31:0] baddr, input logic [31:0] val);
        @(negedge clk);
        st_en = 1'b1; st_wa = baddr[MEMORY_ADDR_WIDTH-1+ADDR_SHIFT:ADDR_SHIFT];
        st_wd = val; st_wm = '1;
        @(negedge clk); st_en = 1'b0; st_wm = '0;
    endtask

    // Point load port p at a BYTE address with a given load_en (no edge wait).
    task automatic drive_port(input int p, input logic [31:0] baddr, input bit en);
        ld_a[p*MAW +: MAW]            = baddr[MAW-1+ADDR_SHIFT:ADDR_SHIFT];
        ld_off[p*ADDR_SHIFT +: ADDR_SHIFT] = baddr[ADDR_SHIFT-1:0];
        ld_en[p]                      = en;
    endtask

    // 32-bit load query at a BYTE address on port 0 (no side effect; settle comb).
    task automatic rd32(input logic [31:0] baddr, input bit is_plic,
            output logic [31:0] val);
        drive_port(0, baddr, 1'b0);
        #1;
        val = is_plic ? pl_data[0*XLEN +: 32] : cl_data[0*XLEN +: 32];
    endtask

    // A PLIC claim load on port 0 (asserts load_en for one cycle to close the gw).
    task automatic claim(input logic [31:0] baddr, output logic [31:0] val);
        @(negedge clk);
        drive_port(0, baddr, 1'b1);
        #1; val = pl_data[0*XLEN +: 32];
        @(negedge clk); ld_en[0] = 1'b0;
    endtask

    logic [31:0] rv, rva, rvb;
    initial begin
        st_en=0; st_wa='0; st_wd='0; st_wm='0; ld_a='0; ld_en=0; ld_off='0;
        rst_l=0; repeat(4) @(negedge clk); rst_l=1; @(negedge clk);

        $display("== M4-S6a shared CLINT/PLIC: 4 harts (clint NUM_HARTS=4 / plic NCTX=8) ==");

        // ---- CLINT: per-hart msip (the IPI mechanism) ----
        // hart 0 writes hart 2's msip (BASE + 4*2): an inter-processor interrupt.
        st32(CLINT_BASE + 32'd8, 32'd1);
        chk(cl_msip == 4'b0100, "IPI: write msip[2]=1 raises ONLY hart-2 software irq");
        st32(CLINT_BASE + 32'd0, 32'd1);                 // hart 0 msip
        chk(cl_msip == 4'b0101, "msip[0] independent of msip[2]");
        st32(CLINT_BASE + 32'd8, 32'd0);                 // clear hart 2
        chk(cl_msip == 4'b0001, "clear msip[2] leaves msip[0] set");
        rd32(CLINT_BASE + 32'd0, 1'b0, rv);
        chk(rv[0] == 1'b1, "readback msip[0]==1");

        // ---- CLINT: per-hart mtimecmp / mtip ----
        // hart 1 mtimecmp = 0 (lo @ 0x4000+8, hi @ 0x400C) -> mtip[1] fires; others stay armed high (reset).
        st32(CLINT_BASE + 32'h4008, 32'd0);              // mtimecmp[1] lo
        st32(CLINT_BASE + 32'h400C, 32'd0);              // mtimecmp[1] hi
        @(negedge clk);
        chk(cl_mtip[1] == 1'b1, "mtimecmp[1]=0 -> mtip[1] asserted");
        chk(cl_mtip[0] == 1'b0 && cl_mtip[2] == 1'b0 && cl_mtip[3] == 1'b0,
            "other harts' mtip stay low (mtimecmp reset = all-ones)");

        // ---- PLIC: per-context enable/pending/external + claim/complete ----
        // source 5, priority 7; enable for ctx 3 (= hart 1 S-mode) only.
        st32(PLIC_BASE + 32'h0000_0000 + 32'd4*5, 32'd7);          // priority[5]=7
        st32(PLIC_BASE + 32'h0000_2000 + 32'h80*3, 32'd1 << 5);    // enable[ctx3] src5
        st32(PLIC_BASE + 32'h0020_0000 + 32'h1000*3, 32'd0);       // threshold[ctx3]=0
        st32(PLIC_BASE + 32'h0000_1000, 32'd1 << 5);               // inject pending src5
        @(negedge clk);
        chk(pl_sext == 4'b0010, "src5 enabled@ctx3 -> ONLY hart-1 S-external");
        chk(pl_mext == 4'b0000, "no M-external (ctx2 not enabled)");
        // claim via ctx 3 (THRESH base + 0x1000*3 + 4) -> returns source 5, closes gateway.
        claim(PLIC_BASE + 32'h0020_0000 + 32'h1000*3 + 32'd4, rv);
        chk(rv == 32'd5, "ctx3 claim returns source 5");
        @(negedge clk);
        chk(pl_sext[1] == 1'b0, "after claim, hart-1 S-external de-asserts (in flight)");
        // complete: write the source id back to the claim register -> reopens the
        // gateway (the sw-injected pending was consumed by the claim, so re-inject to
        // show the source can fire again -- i.e. the gateway is not stuck closed).
        st32(PLIC_BASE + 32'h0020_0000 + 32'h1000*3 + 32'd4, 32'd5);  // complete ctx3
        st32(PLIC_BASE + 32'h0000_1000, 32'd1 << 5);                  // re-inject pending src5
        @(negedge clk);
        chk(pl_sext[1] == 1'b1, "after complete + re-pend, hart-1 S-external re-asserts (gateway reopened)");
        // enabling the SAME source for a different hart's M context routes independently.
        st32(PLIC_BASE + 32'h0000_2000 + 32'h80*4, 32'd1 << 5);    // enable[ctx4] = hart 2 M
        st32(PLIC_BASE + 32'h0020_0000 + 32'h1000*4, 32'd0);       // threshold[ctx4]=0
        @(negedge clk);
        chk(pl_mext == 4'b0100, "same source (pending) enabled@ctx4 -> hart-2 M-external (per-context routing)");
        chk(pl_sext == 4'b0010, "...and still routed to hart-1 S-external (shared source, two contexts)");

        // ===== multi-port: two harts claim DIFFERENT contexts the SAME cycle =====
        st32(PLIC_BASE + 32'd4*6, 32'd7);                          // priority[6]=7
        st32(PLIC_BASE + 32'd4*7, 32'd6);                          // priority[7]=6
        st32(PLIC_BASE + 32'h0000_2000 + 32'h80*5, 32'd1 << 6);    // enable[ctx5] src6
        st32(PLIC_BASE + 32'h0000_2000 + 32'h80*6, 32'd1 << 7);    // enable[ctx6] src7
        st32(PLIC_BASE + 32'h0020_0000 + 32'h1000*5, 32'd0);       // threshold[ctx5]=0
        st32(PLIC_BASE + 32'h0020_0000 + 32'h1000*6, 32'd0);       // threshold[ctx6]=0
        st32(PLIC_BASE + 32'h0000_1000, (32'd1<<6)|(32'd1<<7));    // pending src6, src7
        @(negedge clk);
        drive_port(2, PLIC_BASE + 32'h0020_0000 + 32'h1000*5 + 32'd4, 1'b1); // port2 claim ctx5
        drive_port(3, PLIC_BASE + 32'h0020_0000 + 32'h1000*6 + 32'd4, 1'b1); // port3 claim ctx6
        #1;
        rva = pl_data[2*XLEN +: 32];
        rvb = pl_data[3*XLEN +: 32];
        chk(rva == 32'd6, "two-port: port2 claims src6 (ctx5) simultaneously");
        chk(rvb == 32'd7, "two-port: port3 claims src7 (ctx6) simultaneously");
        @(negedge clk); ld_en[2] = 1'b0; ld_en[3] = 1'b0;

        // ===== double-claim: same source, two contexts, two ports, SAME cycle =====
        // src8 enabled for ctx0 (hart0 M) AND ctx2 (hart1 M); both elect it. Only
        // the lower port wins; the higher reads 0 (true no-double-claim).
        st32(PLIC_BASE + 32'd4*8, 32'd5);                          // priority[8]=5
        st32(PLIC_BASE + 32'h0000_2000 + 32'h80*0, 32'd1 << 8);    // enable[ctx0] src8
        st32(PLIC_BASE + 32'h0000_2000 + 32'h80*2, 32'd1 << 8);    // enable[ctx2] src8
        st32(PLIC_BASE + 32'h0020_0000 + 32'h1000*0, 32'd0);       // threshold[ctx0]=0
        st32(PLIC_BASE + 32'h0020_0000 + 32'h1000*2, 32'd0);       // threshold[ctx2]=0
        st32(PLIC_BASE + 32'h0000_1000, 32'd1 << 8);               // pending src8
        @(negedge clk);
        drive_port(0, PLIC_BASE + 32'h0020_0000 + 32'h1000*0 + 32'd4, 1'b1); // port0 claim ctx0
        drive_port(1, PLIC_BASE + 32'h0020_0000 + 32'h1000*2 + 32'd4, 1'b1); // port1 claim ctx2
        #1;
        rva = pl_data[0*XLEN +: 32];
        rvb = pl_data[1*XLEN +: 32];
        chk(rva == 32'd8, "double-claim: lower port (port0) wins src8");
        chk(rvb == 32'd0, "double-claim: higher port (port1) gets 0 (no double-claim)");
        @(negedge clk); ld_en[0] = 1'b0; ld_en[1] = 1'b0;

        $display("");
        if (errors == 0)
            $display("==== tb_clint_plic_smp: ALL CHECKS PASSED (multi-hart CLINT/PLIC) ====");
        else
            $display("==== tb_clint_plic_smp: %0d CHECK(S) FAILED ====", errors);
        $finish;
    end
endmodule
