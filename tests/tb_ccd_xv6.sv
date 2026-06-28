// tb_ccd_xv6.sv -- P6: the multi-core xv6-SMP boot harness. NCORE real riscv_core_ooo
// (RV64G+Sv39, per-hart HART_ID, COHERENT, NIIGO_EXT_DEVICES) share ONE grant-and-go MOESI
// directory through the validated niigo_ccd_memsys module (remote-dirty I-fetch + PTW + device
// bypass). The directory's single NMI master drives nmi_mem_adapter -> main_memory, loaded with
// the xv6 kernel image + the in-RAM fs.img ramdisk (the SAME mem.N / manifest the single-core
// boot uses; xv6 is SMP by design -- harts self-register up to NCPU, CPUS only affects QEMU).
// ONE shared CLINT(NUM_HARTS)/PLIC(NCTX=2*NCORE)/UART hub lives here (NIIGO_EXT_DEVICES); the
// UART auto-$write's the console and reads +uart_in for stdin.
//
//   Build:  make ccd-xv6-test        (NCORE=2)  |  ccd-xv6-1-test (NCORE=1, module/boot sanity)
//   Run:    cd output/xv6m2 && <Mdir>/Vtop +no_ecall_halt +uart_in=$'ls\n'
`include "niigo_mem.vh"
`include "niigo_cmi.vh"
`include "niigo_ccd_m1.vh"
`include "riscv_uarch.vh"
`default_nettype none
module top
    import RISCV_ISA::XLEN, RISCV_ISA::XLEN_BYTES;
    import RISCV_UArch::MEMORY_NUM_PORTS, RISCV_UArch::MEMORY_ADDR_WIDTH, RISCV_UArch::MEMORY_READ_WIDTH;
    import MemorySegments::SEGMENT_WORDS;
    import NIIGO_Mem::*;
    import NIIGO_CMI::*;
    import NIIGO_CCD_M1::*;
;
`ifdef NCORE1
    localparam int NCORE = 1;
`elsif NCORE4
    localparam int NCORE = 4;
`else
    localparam int NCORE = 2;
`endif
    localparam int NCTX       = 2*NCORE;
    localparam int ADDR_SHIFT = $clog2(XLEN_BYTES);
    logic clk=0, rst_l=0;
    always #5 clk=~clk;
    int              cycle_count = 0;   // register_file.sv XMR ($root.top.cycle_count/.pc)
    logic [XLEN-1:0] pc = '0;
    always_ff @(posedge clk) cycle_count <= cycle_count + 1;

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

    // ===== shared-device hub plumbing (NIIGO_EXT_DEVICES) =====
    logic                          ds_en[NCORE]; logic [MEMORY_ADDR_WIDTH-1:0] ds_wa[NCORE];
    logic [XLEN-1:0]               ds_wd[NCORE]; logic [XLEN_BYTES-1:0] ds_wm[NCORE];
    logic [MEMORY_ADDR_WIDTH-1:0]  dl_a[NCORE]; logic dl_en[NCORE]; logic [ADDR_SHIFT-1:0] dl_off[NCORE];
    logic                          ext_hit[NCORE]; logic [XLEN-1:0] ext_dat[NCORE];
    logic [NCORE*MEMORY_ADDR_WIDTH-1:0] dl_a_p;
    logic [NCORE-1:0]                   dl_en_p;
    logic [NCORE*ADDR_SHIFT-1:0]        dl_off_p;
    logic [NCORE-1:0]                   cl_hit_p, pl_hit_p, cl_mtip, cl_msip, pl_mext, pl_sext;
    logic [NCORE*XLEN-1:0]              cl_data_p, pl_data_p;
    logic [63:0]                        cl_mtime;
    logic                          dev_st_en; logic [MEMORY_ADDR_WIDTH-1:0] dev_st_wa;
    logic [XLEN-1:0]               dev_st_wd; logic [XLEN_BYTES-1:0] dev_st_wm;

    // ===== NCORE real cores (RV64G+Sv39, shared external devices) =====
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
            .dsnoop_store_en(ds_en[g]), .dsnoop_store_waddr(ds_wa[g]),
            .dsnoop_store_wdata(ds_wd[g]), .dsnoop_store_mask(ds_wm[g]),
            .dsnoop_load_addr(dl_a[g]), .dsnoop_load_en(dl_en[g]), .dsnoop_load_off(dl_off[g]),
            .ext_load_hit(ext_hit[g]), .ext_load_data(ext_dat[g]), .ext_mtime(cl_mtime),
            .ext_irq_m_timer(cl_mtip[g]), .ext_irq_m_software(cl_msip[g]),
            .ext_irq_m_external(pl_mext[g]), .ext_irq_s_external(pl_sext[g]),
            .halted(halted_c));
        wire unused = halted_c;
    end endgenerate

    // ===== the multi-core coherent memsys (validated module) =====
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

    // ===== data-side backend: NMI adapter -> main_memory (xv6 image via mem.N / manifest) =====
    logic [MEMORY_ADDR_WIDTH-1:0]            ms_i_addr;
    logic [MEMORY_READ_WIDTH-1:0][XLEN-1:0]  ms_i_load_data, ms_d_load_data;
    logic                                    ms_i_excpt, ms_d_excpt;
    logic                          adp_wr_en; logic [MEMORY_ADDR_WIDTH-1:0] adp_wr_addr;
    logic [XLEN-1:0]               adp_wr_data; logic [XLEN_BYTES-1:0] adp_wr_mask;
    logic [XLEN-1:0]               ms_ptw_rdata;

    nmi_mem_adapter Adapter (
        .clk, .rst_l,
        .nmi_req(mreq), .nmi_req_ready(mreq_ready), .nmi_resp(mresp),
        .mem_rd_addr(ms_i_addr), .mem_rd_data(ms_i_load_data), .mem_rd_excpt(ms_i_excpt),
        .mem_wr_en(adp_wr_en), .mem_wr_addr(adp_wr_addr), .mem_wr_data(adp_wr_data), .mem_wr_mask(adp_wr_mask),
        .fz_en(1'b0), .fz_seed(32'd1), .fz_min(32'd1), .fz_max(32'd1));

    main_memory #(
        .NUM_PORTS(MEMORY_NUM_PORTS), .LOAD_WORDS(MEMORY_READ_WIDTH),
        .WORD_BYTES(XLEN_BYTES), .ADDR_WIDTH(MEMORY_ADDR_WIDTH), .SEGMENT_WORDS(SEGMENT_WORDS)
    ) Memory (
        .clk, .rst_l,
        .load_ens   ({1'b0, 1'b1}),                                  // [1]=D no-load, [0]=I always
        .store_masks({adp_wr_en ? adp_wr_mask : {XLEN_BYTES{1'b0}}, {XLEN_BYTES{1'b0}}}),
        .addrs      ({adp_wr_addr, ms_i_addr}),
        .store_data ({adp_wr_data, {XLEN{1'bx}}}),
        .mem_excpts ({ms_d_excpt, ms_i_excpt}),
        .load_data  ({ms_d_load_data, ms_i_load_data}),
        .ptw_addr   ('0), .ptw_we(1'b0), .ptw_wdata('0), .ptw_rdata(ms_ptw_rdata));
    wire unused_mem = ms_d_excpt | (|ms_d_load_data) | (|ms_ptw_rdata);

    // ===== shared device hub: store arbiter, per-port CLINT/PLIC, single-port UART =====
    always_comb begin
        dev_st_en=1'b0; dev_st_wa='0; dev_st_wd='0; dev_st_wm='0;
        for (int c=0;c<NCORE;c++) if (ds_en[c] && !dev_st_en) begin
            dev_st_en=1'b1; dev_st_wa=ds_wa[c]; dev_st_wd=ds_wd[c]; dev_st_wm=ds_wm[c];
        end
    end
    // single-port UART: route the lowest-indexed core whose device-load addr is in the UART
    // window to the one UART port (xv6 serialises console access behind a lock).
    localparam logic [MEMORY_ADDR_WIDTH-1:0] UART_BASE_W = MEMORY_ADDR_WIDTH'(32'h0D00_0000 >> ADDR_SHIFT);
    logic                          ua_en; logic [MEMORY_ADDR_WIDTH-1:0] ua_la; logic [ADDR_SHIFT-1:0] ua_off;
    int                            ua_sel;
    logic                          ua_hit; logic [XLEN-1:0] ua_data;
    logic                          uart_irq;
    always_comb begin
        ua_en=1'b0; ua_la='0; ua_off='0; ua_sel=-1;
        for (int c=0;c<NCORE;c++)
            if (ua_sel<0 && dl_en[c] && dl_a[c] >= UART_BASE_W && dl_a[c] < UART_BASE_W + MEMORY_ADDR_WIDTH'(16)) begin
                ua_sel=c; ua_en=1'b1; ua_la=dl_a[c]; ua_off=dl_off[c];
            end
    end
    always_comb begin
        for (int c=0;c<NCORE;c++) begin
            dl_a_p[c*MEMORY_ADDR_WIDTH +: MEMORY_ADDR_WIDTH] = dl_a[c];
            dl_en_p[c]                                       = dl_en[c];
            dl_off_p[c*ADDR_SHIFT +: ADDR_SHIFT]             = dl_off[c];
            ext_hit[c] = cl_hit_p[c] | pl_hit_p[c] | ((c==ua_sel) ? ua_hit : 1'b0);
            ext_dat[c] = cl_hit_p[c] ? cl_data_p[c*XLEN +: XLEN]
                       : pl_hit_p[c] ? pl_data_p[c*XLEN +: XLEN]
                       : ua_data;
        end
    end
    clint #(.NUM_HARTS(NCORE), .NPORT(NCORE)) CLINT (
        .clk, .rst_l,
        .store_en(dev_st_en), .store_waddr(dev_st_wa), .store_wdata(dev_st_wd), .store_mask(dev_st_wm),
        .load_addr(dl_a_p), .load_hit(cl_hit_p), .load_data(cl_data_p),
        .irq_m_timer(cl_mtip), .irq_m_software(cl_msip), .mtime_out(cl_mtime));
    logic [31:0] plic_src;
    always_comb begin plic_src = 32'b0; plic_src[10] = uart_irq; end
    plic #(.NCTX(NCTX), .NSOURCES(31), .NPORT(NCORE)) PLIC (
        .clk, .rst_l, .src_irq(plic_src),
        .store_en(dev_st_en), .store_waddr(dev_st_wa), .store_wdata(dev_st_wd), .store_mask(dev_st_wm),
        .load_addr(dl_a_p), .load_en(dl_en_p), .load_off(dl_off_p),
        .load_hit(pl_hit_p), .load_data(pl_data_p),
        .irq_m_external(pl_mext), .irq_s_external(pl_sext));
    uart Uart (
        .clk, .rst_l,
        .store_en(dev_st_en), .store_waddr(dev_st_wa), .store_wdata(dev_st_wd), .store_mask(dev_st_wm),
        .load_addr(ua_la), .load_en(ua_en), .load_off(ua_off),
        .load_hit(ua_hit), .load_data(ua_data), .irq(uart_irq));

    // ===== run loop: xv6 never halts; stream the UART ($write) for a cycle budget =====
    int unsigned MAXCYC;
    initial begin
        if (!$value$plusargs("maxcyc=%d", MAXCYC)) MAXCYC = 32'd20_000_000;
        rst_l=0; repeat(20) @(posedge clk); rst_l=1;
        $display("== P6 xv6-SMP: %0d harts on the grant-and-go directory (niigo_ccd_memsys) ==", NCORE);
    end
    initial begin
        @(posedge rst_l);
        repeat (MAXCYC) @(posedge clk);
        $display("\n[tb_ccd_xv6] cycle budget reached (%0d) -- stopping", MAXCYC);
        $finish;
    end
endmodule
`default_nettype wire
