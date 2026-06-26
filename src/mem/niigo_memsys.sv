/**
 * niigo_memsys.sv
 *
 * Memory subsystem for the out-of-order core (phases N1+N2: passthrough with
 * an optional latency/reorder fuzzer).
 *
 * This module is the single seam between the OoO core and backing memory.
 * The core side speaks three handshaked ports (the core makes NO latency
 * assumptions about any of them):
 *
 *   - ifetch: one 16-byte instruction block per request (valid/ready), with
 *     an in-order response stream {valid, data, excpt}. No request ID: the
 *     core associates responses with requests by arrival order.
 *   - dmem: one word-granular load OR store per request (valid/ready).
 *     Loads produce an in-order response {valid, addr echo, data}; stores
 *     are fire-and-forget once accepted (the memsys guarantees that an
 *     accepted store is ordered before any later-accepted D-side access).
 *     The address echo exists for the memory-mapped devices, which decode
 *     the returning load's physical address at delivery time.
 *   - ptw: the page-table walker's level req/ack word port (reads and A/D
 *     writebacks). rdata is valid in the ack cycle; the A/D write applies
 *     exactly once, at ack.
 *
 * The memory side mirrors the (DO NOT MODIFY) main_memory port shapes; the
 * testbench wires it to the same dual-port memory + PTW port the legacy
 * cores use. Reads sample memory combinationally in the acceptance cycle
 * and the data rides a response queue, so the value observed is the memory
 * state at the point of acceptance (exactly the legacy delay_buffer
 * semantics); stores apply at the acceptance clock edge; PTW reads sample
 * at ack.
 *
 * Default configuration (no plusargs): requests are always accepted
 * (ready == 1) and responses arrive a fixed I_RESP_DELAY / D_RESP_DELAY
 * cycles later, matching the legacy testbench's IMEMORY_READ_DELAY /
 * DMEMORY_READ_DELAY, and the PTW ack is combinational -- cycle-identical
 * to the phase-N1 passthrough.
 *
 * Latency fuzzer (phase N2), enabled with +mem_fuzz:
 *   +mem_fuzz            enable randomized timing (presence flag)
 *   +mem_seed=<n>        LCG seed (default 1); runs are fully reproducible
 *   +mem_min=<n>         minimum response latency in cycles (default 1)
 *   +mem_max=<n>         maximum response latency in cycles (default 20)
 * Randomized per accepted request: I/D response latency in [min,max]
 * (responses stay in order per port; independent port streams reorder
 * I vs D vs PTW relative to each other). Randomized per cycle: I/D request
 * acceptance (ready deasserts ~25% of cycles, force-asserted after 7
 * consecutive stalls so nothing starves) and the PTW ack delay (1..8
 * cycles). No $random anywhere -- a fixed LCG keeps failures replayable
 * with the same seed.
 */

`include "riscv_isa.vh"
`include "riscv_uarch.vh"
`include "niigo_mem.vh"
`ifdef CCD_AGENT
`include "niigo_ccd_m1.vh"   // l1_core_op_e / l1_amo_op_e / COP_* / AMO_* (M3d agent core port)
`endif

// A write-back D-side (the C2 L1D cache OR the M3d grant-and-go MOESI agent) shares
// the device-bypass response gen, the adapter line-write onto main_memory's D port,
// and the exclusion of the L1=0 passthrough D/PTW block. Derive one guard for both.
`ifdef L1D_CACHE
  `define NIIGO_AGENT_DSIDE
`elsif CCD_AGENT
  `define NIIGO_AGENT_DSIDE
`endif

`default_nettype none

module niigo_memsys
    import RISCV_ISA::XLEN, RISCV_ISA::XLEN_BYTES;
    import RISCV_UArch::MEMORY_READ_WIDTH, RISCV_UArch::MEMORY_ADDR_WIDTH;
    import NIIGO_Mem::*;
`ifdef CCD_AGENT
    import NIIGO_CCD_M1::*;
`endif
(
    input wire logic clk,
    input wire logic rst_l,

    // ---- Core: instruction fetch port ----
    input wire logic                          ifetch_req_valid,
    output logic                          ifetch_req_ready,
    input wire logic [MEMORY_ADDR_WIDTH-1:0]  ifetch_req_addr,
    output logic                          ifetch_resp_valid,
    output logic [MEMORY_READ_WIDTH-1:0][XLEN-1:0] ifetch_resp_data,
    output logic                          ifetch_resp_excpt,
    // fence.i / flush: flash-invalidate the L1I (ignored in the L1=0 build).
    input wire logic                          ifetch_inval,
    // Cacheable/device split for the data port (used at L1D=1; the device hole
    // bypasses the L1D). Ignored by the L1=0/C1 passthrough.
    input wire logic                          dmem_req_device,
    // L1D writeback flush handshake (fence.i + halt). At L1D=1 the L1D walks and
    // writes back all dirty lines; otherwise it completes immediately.
    input wire logic                          dcache_flush_req,
    output logic                          dcache_flush_done,
    // Cache event pulses for mhpmcounter3-5 (phase C3). Zero in the L1=0 build.
    output logic                          hpm_l1i_miss,
    output logic                          hpm_l1d_miss,
    output logic                          hpm_l1d_wb,

    // ---- Core: data port ----
    input wire logic                          dmem_req_valid,
    output logic                          dmem_req_ready,
    input wire logic                          dmem_req_write,
    input wire logic [MEMORY_ADDR_WIDTH-1:0]  dmem_req_addr,
    input wire logic [XLEN-1:0]               dmem_req_wdata,
    input wire logic [XLEN_BYTES-1:0]         dmem_req_wmask,
    output logic                          dmem_resp_valid,
    output logic [MEMORY_ADDR_WIDTH-1:0]  dmem_resp_addr,
    output logic [XLEN-1:0]               dmem_resp_data,

    // ---- Core: page-table-walker port ----
    input wire logic                          ptw_req_valid,
    input wire logic                          ptw_req_we,
    input wire logic [MEMORY_ADDR_WIDTH-1:0]  ptw_req_addr,
    input wire logic [XLEN-1:0]               ptw_req_wdata,
    output logic                          ptw_req_ack,
    output logic [XLEN-1:0]               ptw_resp_rdata,

    // ---- Memory side (testbench wires these to main_memory) ----
    output logic                          mem_d_load_en,
    output logic [XLEN_BYTES-1:0]         mem_d_store_mask,
    output logic [MEMORY_ADDR_WIDTH-1:0]  mem_d_addr,
    output logic [XLEN-1:0]               mem_d_store_data,
    input wire logic [MEMORY_READ_WIDTH-1:0][XLEN-1:0] mem_d_load_data,
    input wire logic                          mem_d_excpt,
    output logic [MEMORY_ADDR_WIDTH-1:0]  mem_i_addr,
    input wire logic [MEMORY_READ_WIDTH-1:0][XLEN-1:0] mem_i_load_data,
    input wire logic                          mem_i_excpt,
    output logic [MEMORY_ADDR_WIDTH-1:0]  mem_ptw_addr,
    output logic                          mem_ptw_we,
    output logic [XLEN-1:0]               mem_ptw_wdata,
    input wire logic [XLEN-1:0]               mem_ptw_rdata
`ifdef FPGA_BUILD
    // ---- AXI4-512 master to external DRAM (FPGA build; requires AXI_MEMSYS).
    //      The sim AXI shim/monitor are replaced by these exposed ports so a SoC
    //      wrapper (niigo_soc) can drive the shell DRAM controller. ----
    ,
    output logic                      m_axi_awvalid,
    input  wire logic                 m_axi_awready,
    output logic [AXI_ADDR_W-1:0]     m_axi_awaddr,
    output logic [AXI_ID_W-1:0]       m_axi_awid,
    output logic [7:0]                m_axi_awlen,
    output logic [2:0]                m_axi_awsize,
    output logic [1:0]                m_axi_awburst,
    output logic                      m_axi_wvalid,
    input  wire logic                 m_axi_wready,
    output logic [AXI_DATA_W-1:0]     m_axi_wdata,
    output logic [AXI_STRB_W-1:0]     m_axi_wstrb,
    output logic                      m_axi_wlast,
    input  wire logic                 m_axi_bvalid,
    output logic                      m_axi_bready,
    input  wire logic [AXI_ID_W-1:0]  m_axi_bid,
    input  wire logic [1:0]           m_axi_bresp,
    output logic                      m_axi_arvalid,
    input  wire logic                 m_axi_arready,
    output logic [AXI_ADDR_W-1:0]     m_axi_araddr,
    output logic [AXI_ID_W-1:0]       m_axi_arid,
    output logic [7:0]                m_axi_arlen,
    output logic [2:0]                m_axi_arsize,
    output logic [1:0]                m_axi_arburst,
    input  wire logic                 m_axi_rvalid,
    output logic                      m_axi_rready,
    input  wire logic [AXI_ID_W-1:0]  m_axi_rid,
    input  wire logic [AXI_DATA_W-1:0] m_axi_rdata,
    input  wire logic [1:0]           m_axi_rresp,
    input  wire logic                 m_axi_rlast
`endif
);

    // Fixed response latencies for the default (non-fuzz) configuration
    // (>= 1 so a response never lands in the cycle its request was
    // accepted). Reuse the legacy lab constants so the default build keeps
    // the timing character the suites were tuned on.
    localparam int I_RESP_DELAY =
        (RISCV_UArch::IMEMORY_READ_DELAY < 1) ? 1 : RISCV_UArch::IMEMORY_READ_DELAY;
    localparam int D_RESP_DELAY =
        (RISCV_UArch::DMEMORY_READ_DELAY < 1) ? 1 : RISCV_UArch::DMEMORY_READ_DELAY;

    // Response queue depth. The core bounds itself to 2 outstanding fetches
    // and 1 outstanding load, so 4 never fills; pushes guard on it anyway.
    localparam int QD = 4;

    // ------------------------------------------------------------------
    // Fuzz configuration (plusargs, read once) and LCG streams.
    // ------------------------------------------------------------------
    logic        fz_en;
    int unsigned fz_seed, fz_min, fz_max;
    initial begin
        fz_en = $test$plusargs("mem_fuzz") != 0;
        if (!$value$plusargs("mem_seed=%d", fz_seed)) fz_seed = 1;
        if (!$value$plusargs("mem_min=%d",  fz_min))  fz_min  = 1;
        if (!$value$plusargs("mem_max=%d",  fz_max))  fz_max  = 20;
        if (fz_min < 1) fz_min = 1;
        if (fz_max < fz_min) fz_max = fz_min;
        if (fz_en)
            $display("niigo_memsys: +mem_fuzz on (seed=%0d latency=[%0d,%0d])",
                fz_seed, fz_min, fz_max);
    end

    function automatic logic [31:0] lcg_next(input logic [31:0] s);
        lcg_next = s * 32'd1664525 + 32'd1013904223;
    endfunction

    // Per-port streams: delay draws step per accepted request; ready/ack
    // streams step every cycle. Seeded once, never reset (reproducible from
    // +mem_seed; reset replay is not a supported flow in this testbench).
    logic [31:0] ilat_lcg, dlat_lcg, irdy_lcg, drdy_lcg, pack_lcg;
    initial begin
        ilat_lcg = 32'(fz_seed) ^ 32'h49464C54;   // "IFLT"
        dlat_lcg = 32'(fz_seed) ^ 32'h444C4154;   // "DLAT"
        irdy_lcg = 32'(fz_seed) ^ 32'h49524459;   // "IRDY"
        drdy_lcg = 32'(fz_seed) ^ 32'h44524459;   // "DRDY"
        pack_lcg = 32'(fz_seed) ^ 32'h5057414B;   // "PWAK"
    end

    function automatic int unsigned lcg_range(input logic [31:0] s,
            input int unsigned lo, input int unsigned hi);
        lcg_range = lo + (32'(s >> 8) % (hi - lo + 1));
    endfunction

    // Free-running cycle counter for response due-times.
    logic [63:0] cyc_q;
    always_ff @(posedge clk or negedge rst_l) begin
        if (!rst_l) cyc_q <= '0;
        else        cyc_q <= cyc_q + 64'd1;
    end

    // ------------------------------------------------------------------
    // Request acceptance (ready) streams: in fuzz mode, deassert ~25% of
    // cycles with a 7-cycle starvation bound; otherwise constant 1.
    // ------------------------------------------------------------------
    logic [2:0] irdy_stall_q, drdy_stall_q;
    logic       irdy_rand, drdy_rand;
    assign irdy_rand = (irdy_lcg[1:0] != 2'b00) || (irdy_stall_q >= 3'd7);
    assign drdy_rand = (drdy_lcg[1:0] != 2'b00) || (drdy_stall_q >= 3'd7);

    always_ff @(posedge clk) begin
        if (fz_en) begin
            irdy_lcg <= lcg_next(irdy_lcg);
            drdy_lcg <= lcg_next(drdy_lcg);
            irdy_stall_q <= irdy_rand ? 3'd0 : (irdy_stall_q + 3'd1);
            drdy_stall_q <= drdy_rand ? 3'd0 : (drdy_stall_q + 3'd1);
        end
    end
    initial begin
        irdy_stall_q = '0;
        drdy_stall_q = '0;
    end

`ifdef L1_CACHES
    // ------------------------------------------------------------------
    // Instruction port (L1=1): the L1I cache serves fetches; misses refill a
    // 64 B line over the NMI bus (arbiter -> sim adapter) from main_memory's
    // I-read port. The data (D) and PTW ports remain passthrough below; the
    // L1D and PTW-through-cache land in phase C2. Refill latency is fuzzed in
    // the adapter; fetch hits accept every cycle.
    // ------------------------------------------------------------------
    nmi_req_t  l1i_nmi_req;
    logic      l1i_nmi_ready;
    nmi_resp_t l1i_nmi_resp;
    logic      l1i_ev_access, l1i_ev_miss;

    // C4b: committed-store snoop into the L1I (source assigned from the store
    // path below: the L1D store at L1D=1, the passthrough store at L1=1-only).
    logic                          l1i_snoop_valid;
    logic [MEMORY_ADDR_WIDTH-1:0]  l1i_snoop_waddr;

    l1_icache L1I (
        .clk, .rst_l,
        .ifetch_req_valid, .ifetch_req_ready, .ifetch_req_addr,
        .ifetch_resp_valid, .ifetch_resp_data, .ifetch_resp_excpt,
        .inval_all(ifetch_inval),
        .snoop_valid(l1i_snoop_valid), .snoop_waddr(l1i_snoop_waddr),
        .nmi_req(l1i_nmi_req),
        .nmi_req_ready(l1i_nmi_ready),
        .nmi_resp(l1i_nmi_resp),
        .ev_access(l1i_ev_access),
        .ev_miss(l1i_ev_miss)
    );

    // Snoop source: a committed D-store's line invalidates any L1I copy. At
    // L1D=1 it is the store accepted into the L1D; at L1=1-only it is the
    // passthrough store (which writes memory directly, so the producer side is
    // automatic and only this consumer-side invalidate is needed).
`ifdef L1D_CACHE
    assign l1i_snoop_valid = present_dmem && dmem_req_write && l1d_req_fire;
    assign l1i_snoop_waddr = l1d_req_waddr;
`elsif CCD_AGENT
    // C4 I/D-coherence (committed-store -> L1I snoop) is deferred for M3d Stage 1;
    // fence.i drives the agent writeback flush + ifetch_inval, which covers SMC.
    assign l1i_snoop_valid = 1'b0;
    assign l1i_snoop_waddr = '0;
`else
    assign l1i_snoop_valid = dmem_store_fire;
    assign l1i_snoop_waddr = dmem_req_addr;
`endif
    // The NMI arbiter + sim adapter that the L1I (and, at L1D=1, the L1D) refill
    // through live in the shared backend block below.
`else
    // ------------------------------------------------------------------
    // Instruction port (L1=0): combinational read at the request address in
    // the acceptance cycle; {excpt, data} ride an in-order response queue
    // whose entries become deliverable after a fixed (default) or randomized
    // (fuzz) latency. At most one response delivers per cycle, preserving
    // arrival-order association in the core.
    // ------------------------------------------------------------------
    logic unused_ifetch_inval;
    assign unused_ifetch_inval = ifetch_inval;
    assign mem_i_addr = ifetch_req_addr;

    typedef struct packed {
        logic                                  excpt;
        logic [MEMORY_READ_WIDTH-1:0][XLEN-1:0] data;
        logic [63:0]                           due;
    } i_ent_t;

    i_ent_t      i_q [QD];
    logic [1:0]  i_rd_q, i_wr_q;
    logic [2:0]  i_cnt_q;
    logic        i_push, i_pop;
    int unsigned i_delay;

    assign ifetch_req_ready = (i_cnt_q < 3'(QD)) && (!fz_en || irdy_rand);
    assign i_push = ifetch_req_valid && ifetch_req_ready;
    assign i_delay = fz_en ? lcg_range(ilat_lcg, fz_min, fz_max)
                           : I_RESP_DELAY;

    assign i_pop = (i_cnt_q != '0) && (cyc_q >= i_q[i_rd_q].due);
    assign ifetch_resp_valid = i_pop;
    assign ifetch_resp_excpt = i_q[i_rd_q].excpt;
    assign ifetch_resp_data  = i_q[i_rd_q].data;

    always_ff @(posedge clk or negedge rst_l) begin
        if (!rst_l) begin
            i_rd_q <= '0;
            i_wr_q <= '0;
            i_cnt_q <= '0;
        end else begin
            if (i_push) begin
                i_q[i_wr_q].excpt <= mem_i_excpt;
                i_q[i_wr_q].data  <= mem_i_load_data;
                // An entry accepted during cycle T (counter value T) with
                // delay d delivers during cycle T+d: the pop compare sees
                // the counter value of the delivery cycle. d >= 1, so a
                // response never lands in its own acceptance cycle.
                i_q[i_wr_q].due   <= cyc_q + 64'(i_delay);
                i_wr_q <= i_wr_q + 2'd1;
                if (fz_en) ilat_lcg <= lcg_next(ilat_lcg);
            end
            if (i_pop) i_rd_q <= i_rd_q + 2'd1;
            i_cnt_q <= i_cnt_q + (i_push ? 3'd1 : 3'd0) - (i_pop ? 3'd1 : 3'd0);
        end
    end
`endif /* L1_CACHES */

`ifndef NIIGO_AGENT_DSIDE
    // ------------------------------------------------------------------
    // Data port (L1=0 / no write-back D-side): loads sample memory in the
    // acceptance cycle and ride the same kind of in-order response queue (with
    // the request word address echoed for the device decode at delivery). Stores
    // forward to the memory write port in the acceptance cycle and apply at the
    // next clock edge; they produce no response.
    // ------------------------------------------------------------------
    logic dmem_load_fire, dmem_store_fire;
    assign dmem_req_ready  = (d_cnt_q < 3'(QD)) && (!fz_en || drdy_rand);
    assign dmem_load_fire  = dmem_req_valid && dmem_req_ready && !dmem_req_write;
    assign dmem_store_fire = dmem_req_valid && dmem_req_ready &&  dmem_req_write;

    assign mem_d_addr       = dmem_req_addr;
    assign mem_d_load_en    = dmem_load_fire;
    assign mem_d_store_mask = dmem_store_fire ? dmem_req_wmask : '0;
    assign mem_d_store_data = dmem_req_wdata;

    typedef struct packed {
        logic [MEMORY_ADDR_WIDTH-1:0] addr;
        logic [XLEN-1:0]              data;
        logic [63:0]                  due;
    } d_ent_t;

    d_ent_t      d_q [QD];
    logic [1:0]  d_rd_q, d_wr_q;
    logic [2:0]  d_cnt_q;
    logic        d_push, d_pop;
    int unsigned d_delay;

    assign d_push = dmem_load_fire;
    assign d_delay = fz_en ? lcg_range(dlat_lcg, fz_min, fz_max)
                           : D_RESP_DELAY;

    assign d_pop = (d_cnt_q != '0) && (cyc_q >= d_q[d_rd_q].due);
    assign dmem_resp_valid = d_pop;
    assign dmem_resp_addr  = d_q[d_rd_q].addr;
    assign dmem_resp_data  = d_q[d_rd_q].data;

    always_ff @(posedge clk or negedge rst_l) begin
        if (!rst_l) begin
            d_rd_q <= '0;
            d_wr_q <= '0;
            d_cnt_q <= '0;
        end else begin
            if (d_push) begin
                if (d_cnt_q >= 3'(QD))
                    $fatal(1, "niigo_memsys: D response queue overflow");
                d_q[d_wr_q].addr <= dmem_req_addr;
                d_q[d_wr_q].data <= mem_d_load_data[0];
                d_q[d_wr_q].due  <= cyc_q + 64'(d_delay);
                d_wr_q <= d_wr_q + 2'd1;
                if (fz_en) dlat_lcg <= lcg_next(dlat_lcg);
            end
            if (d_pop) d_rd_q <= d_rd_q + 2'd1;
            d_cnt_q <= d_cnt_q + (d_push ? 3'd1 : 3'd0) - (d_pop ? 3'd1 : 3'd0);
        end
    end

    // The data-port memory exception is unused by the OoO core (faults are
    // raised by the PMP/translation checks before an access is issued).
    logic unused_d_excpt;
    assign unused_d_excpt = mem_d_excpt;

    // ------------------------------------------------------------------
    // PTW port. Default: combinational passthrough (ack in the request
    // cycle), identical to the legacy direct connection. Fuzz: ack delays
    // 1..8 cycles after the request asserts; the walker holds req/we as
    // levels until ack, and a request abandoned mid-wait (PMP abort drops
    // req without ack) resets the countdown. rdata is sampled (and the A/D
    // write applied) exactly once, in the ack cycle.
    // ------------------------------------------------------------------
    logic       ptw_armed_q;
    logic [2:0] ptw_wait_q;

    always_ff @(posedge clk or negedge rst_l) begin
        if (!rst_l) begin
            ptw_armed_q <= 1'b0;
            ptw_wait_q <= '0;
        end else if (fz_en) begin
            if (!ptw_req_valid || ptw_req_ack) begin
                ptw_armed_q <= 1'b0;
            end else if (!ptw_armed_q) begin
                ptw_armed_q <= 1'b1;
                ptw_wait_q <= pack_lcg[2:0];
                pack_lcg <= lcg_next(pack_lcg);
            end else if (ptw_wait_q != '0) begin
                ptw_wait_q <= ptw_wait_q - 3'd1;
            end
        end
    end

    assign ptw_req_ack = fz_en
        ? (ptw_req_valid && ptw_armed_q && (ptw_wait_q == '0))
        : ptw_req_valid;
    assign mem_ptw_addr   = ptw_req_addr;
    assign mem_ptw_we     = ptw_req_we && ptw_req_ack;
    assign mem_ptw_wdata  = ptw_req_wdata;
    assign ptw_resp_rdata = mem_ptw_rdata;

    // No L1D in this build: the writeback-flush handshake completes immediately
    // and the cacheable/device split is unused (all D/PTW traffic is passthrough
    // above).
    assign dcache_flush_done = dcache_flush_req;
    logic unused_dmem_dev;
    assign unused_dmem_dev = dmem_req_device;
`endif /* !NIIGO_AGENT_DSIDE */

`ifdef L1D_CACHE
    // ==================================================================
    // Phase C2 data side: write-back L1D + PTW-through-L1D + device bypass.
    //
    //   - Cacheable dmem ops (LSQ) and PTW PTE accesses are muxed (LSQ
    //     priority) onto the single L1D requester; the L1D refills/writes back
    //     64 B lines over the NMI bus.
    //   - Device dmem ops (CLINT/PLIC/UART) bypass the cache: a load gets a
    //     dummy response (the core overrides it with the device register value)
    //     and a store is accepted (the core's devices snoop the accepted store);
    //     neither touches main_memory.
    //   - The L1D writeback flush (fence.i + halt) drains all dirty lines.
    // ------------------------------------------------------------------

    // ---- device-bypass response generator (1 outstanding; the LSQ serialises) ----
    logic                          dev_pend_q;
    logic [MEMORY_ADDR_WIDTH-1:0]  dev_addr_q;
    logic [63:0]                   dev_due_q;
    logic                          dev_ready, dev_load_fire, dev_resp_valid;
    assign dev_ready     = !dev_pend_q && (!fz_en || drdy_rand);
    assign dev_load_fire = dmem_req_valid && dmem_req_device && dev_ready && !dmem_req_write;
    assign dev_resp_valid = dev_pend_q && (cyc_q >= dev_due_q);
    always_ff @(posedge clk or negedge rst_l) begin
        if (!rst_l) begin
            dev_pend_q <= 1'b0;
            dev_addr_q <= '0;
            dev_due_q  <= '0;
        end else begin
            if (dev_load_fire) begin
                dev_pend_q <= 1'b1;
                dev_addr_q <= dmem_req_addr;
                dev_due_q  <= cyc_q + 64'(fz_en ? lcg_range(dlat_lcg, fz_min, fz_max)
                                               : D_RESP_DELAY);
                if (fz_en) dlat_lcg <= lcg_next(dlat_lcg);
            end else if (dev_resp_valid) begin
                dev_pend_q <= 1'b0;
            end
        end
    end

    // ---- L1D front arbiter: cacheable dmem (priority) vs PTW ----
    logic                          l1d_req_valid, l1d_req_ready, l1d_req_write;
    logic [MEMORY_ADDR_WIDTH-1:0]  l1d_req_waddr;
    logic [XLEN-1:0]               l1d_req_wdata;
    logic [XLEN_BYTES-1:0]         l1d_req_wmask;
    logic                          l1d_resp_valid, l1d_wr_accept;
    logic [XLEN-1:0]               l1d_resp_data;
    logic [MEMORY_ADDR_WIDTH-1:0]  l1d_resp_addr;

    logic present_dmem, present_ptw;
    logic owner_ptw_q;        // owner of the in-flight L1D op (0 = dmem, 1 = PTW)
    assign present_dmem = dmem_req_valid && !dmem_req_device;
    assign present_ptw  = !present_dmem && ptw_req_valid;

    assign l1d_req_valid = present_dmem || present_ptw;
    assign l1d_req_write = present_dmem ? dmem_req_write : ptw_req_we;
    assign l1d_req_waddr = present_dmem ? dmem_req_addr  : ptw_req_addr;
    assign l1d_req_wdata = present_dmem ? dmem_req_wdata : ptw_req_wdata;
    assign l1d_req_wmask = present_dmem ? dmem_req_wmask : {XLEN_BYTES{1'b1}};

    logic l1d_req_fire;
    assign l1d_req_fire = l1d_req_valid && l1d_req_ready;
    always_ff @(posedge clk or negedge rst_l) begin
        if (!rst_l)            owner_ptw_q <= 1'b0;
        else if (l1d_req_fire) owner_ptw_q <= present_ptw;
    end

    nmi_req_t  l1d_nmi_req;
    logic      l1d_nmi_ready;
    nmi_resp_t l1d_nmi_resp;
    logic      l1d_ev_access, l1d_ev_miss, l1d_ev_wb;

    // C4a: probe the L1D for the line an L1I refill is about to fetch. The probe
    // is the L1I's pending RD_LINE address; probe_clean gates the refill below.
    logic                          l1d_probe_valid, l1d_probe_clean;
    logic [MEMORY_ADDR_WIDTH-1:0]  l1d_probe_waddr;
    assign l1d_probe_valid = l1i_nmi_req.valid && (l1i_nmi_req.op == NMI_RD_LINE);
    assign l1d_probe_waddr = l1i_nmi_req.waddr;

    l1_dcache L1D (
        .clk, .rst_l,
        .req_valid(l1d_req_valid), .req_ready(l1d_req_ready),
        .req_write(l1d_req_write), .req_waddr(l1d_req_waddr),
        .req_wdata(l1d_req_wdata), .req_wmask(l1d_req_wmask),
        .resp_valid(l1d_resp_valid), .resp_data(l1d_resp_data),
        .resp_addr(l1d_resp_addr), .wr_accept(l1d_wr_accept),
        .flush_req(dcache_flush_req), .flush_done(dcache_flush_done),
        .probe_valid(l1d_probe_valid), .probe_waddr(l1d_probe_waddr),
        .probe_clean(l1d_probe_clean),
        .nmi_req(l1d_nmi_req), .nmi_req_ready(l1d_nmi_ready), .nmi_resp(l1d_nmi_resp),
        .ev_access(l1d_ev_access), .ev_miss(l1d_ev_miss), .ev_wb(l1d_ev_wb)
    );

    // dmem ready: gate on BOTH the device path and the L1D being free, rather
    // than selecting on dmem_req_device. The LSQ gates its request address on
    // dmem_req_ready, so selecting the ready by dmem_req_device (which is derived
    // from that address) would form a combinational cycle. Requiring both free
    // is loss-free here: the LSQ keeps one D-access outstanding, so whichever
    // path the previous op used is already idle by the time the next op issues.
    assign dmem_req_ready = dev_ready && l1d_req_ready;
    // dmem response: device dummy (core overrides data) or L1D load.
    assign dmem_resp_valid = dev_resp_valid || (l1d_resp_valid && !owner_ptw_q);
    assign dmem_resp_addr  = dev_resp_valid ? dev_addr_q : l1d_resp_addr;
    assign dmem_resp_data  = dev_resp_valid ? '0 : l1d_resp_data;

    // PTW ack: acked when its L1D op completes -- a read on the load response,
    // a write (A/D update) on the store-accept pulse. Both come from the L1D's
    // registered state and the registered owner, so there is no combinational
    // path from ack back into the walker's level-held request (which would form
    // a cycle if the ack were derived from the live request presentation).
    assign ptw_req_ack    = owner_ptw_q && (l1d_resp_valid || l1d_wr_accept);
    assign ptw_resp_rdata = l1d_resp_data;

    // The PTW main_memory port is retired (PTW now flows through the L1D).
    assign mem_ptw_addr  = '0;
    assign mem_ptw_we    = 1'b0;
    assign mem_ptw_wdata = '0;
    logic unused_c2;
    assign unused_c2 = (|mem_ptw_rdata) | mem_d_excpt | l1d_ev_access;
`endif /* L1D_CACHE */

`ifdef CCD_AGENT
    // ==================================================================
    // M3d data side: the grant-and-go MOESI L1D agent (niigo_l1d_gg, instanced via
    // niigo_ccd_gg_direct #(.NACTIVE(1)) + niigo_dir_gg) replaces the C2 L1D. Single
    // core: coherence is inert (cores 1..3 never become sharers, so the directory
    // yields ack_count==0 and broadcasts no INV -- R12). Cacheable dmem + PTW are
    // muxed (LSQ priority) onto the agent's one c_req port through a REGISTERED launch
    // adapter that converts the agent's grant-and-go handshake (c_req_ready *is*
    // completion, same-cycle c_resp_rdata) into the core's decoupled dmem/ptw ports.
    // The register is mandatory: the agent's c_req_ready is combinational on
    // c_req_valid, so wiring it straight to the LSQ issue gate would form the same
    // comb loop the L1D path avoids. Devices bypass the agent exactly as at L1D=1.
    // C4 I/D-coherence is deferred; fence.i drives the agent flush + ifetch_inval.
    // ------------------------------------------------------------------
    localparam int CCD_L1_SETS  = 64;    // direct-mapped agent; sized up vs L1D to limit conflict thrash
    localparam int CCD_DIR_SETS = 256;   // directory >> L1 sets (avoid dir-capacity eviction, OPEN-3)

    // ---- device-bypass response generator (1 outstanding; the LSQ serialises) ----
    logic                          dev_pend_q;
    logic [MEMORY_ADDR_WIDTH-1:0]  dev_addr_q;
    logic [63:0]                   dev_due_q;
    logic                          dev_ready, dev_load_fire, dev_resp_valid;
    assign dev_ready      = !dev_pend_q && (!fz_en || drdy_rand);
    assign dev_load_fire  = dmem_req_valid && dmem_req_device && dev_ready && !dmem_req_write;
    assign dev_resp_valid = dev_pend_q && (cyc_q >= dev_due_q);
    always_ff @(posedge clk or negedge rst_l) begin
        if (!rst_l) begin dev_pend_q <= 1'b0; dev_addr_q <= '0; dev_due_q <= '0; end
        else begin
            if (dev_load_fire) begin
                dev_pend_q <= 1'b1; dev_addr_q <= dmem_req_addr;
                dev_due_q  <= cyc_q + 64'(fz_en ? lcg_range(dlat_lcg, fz_min, fz_max) : D_RESP_DELAY);
                if (fz_en) dlat_lcg <= lcg_next(dlat_lcg);
            end else if (dev_resp_valid) dev_pend_q <= 1'b0;
        end
    end

    // ---- request select (LSQ dmem priority over PTW), device-bypassed ----
    logic present_dmem, present_ptw;
    assign present_dmem = dmem_req_valid && !dmem_req_device;
    assign present_ptw  = !present_dmem && ptw_req_valid;

    // ---- registered launch adapter (breaks the c_req_ready comb loop) ----
    logic                          ad_busy_q, ad_is_ptw_q, ad_is_load_q;
    l1_core_op_e                   ad_op_q;
    logic [MEMORY_ADDR_WIDTH-1:0]  ad_addr_q;
    logic [XLEN-1:0]               ad_wdata_q;
    logic [XLEN_BYTES-1:0]         ad_wmask_q;
    logic                          ad_resp_pend_q;
    logic [XLEN-1:0]               ad_resp_data_q;
    logic [MEMORY_ADDR_WIDTH-1:0]  ad_resp_addr_q;

    wire ad_can_accept  = !ad_busy_q && !ad_resp_pend_q;       // single-outstanding gate
    wire ad_launch_fire = (present_dmem || present_ptw) && ad_can_accept;

    // loop-free, loss-free ready: mirror the L1D arm (AND of registered-derived terms;
    // NEITHER operand is in the dmem_req_valid fan-in). A load cannot be accepted until
    // the prior store's ad_busy clears (its cww_we cycle) -> store-visibility safe (R2).
    assign dmem_req_ready = dev_ready && ad_can_accept;

    // agent core-side request arrays (length-1: NACTIVE=1)
    logic                          c_req_valid [1];
    logic                          c_req_ready [1];
    l1_core_op_e                   c_req_op    [1];
    l1_amo_op_e                    c_req_amo   [1];
    logic [MEMORY_ADDR_WIDTH-1:0]  c_req_waddr [1];
    logic [XLEN-1:0]               c_req_wdata [1];
    logic [XLEN_BYTES-1:0]         c_req_wmask [1];
    logic [XLEN-1:0]               c_resp_rdata[1];
    logic                          c_resp_sc_ok[1];

    assign c_req_valid[0] = ad_busy_q;
    assign c_req_op[0]    = ad_op_q;
    assign c_req_amo[0]   = AMO_ADD;     // Stage-1 don't-care: the LSQ owns AMO RMW
    assign c_req_waddr[0] = ad_addr_q;
    assign c_req_wdata[0] = ad_wdata_q;
    assign c_req_wmask[0] = ad_wmask_q;

    wire ad_done = ad_busy_q && c_req_ready[0];   // grant-and-go completion pulse

    always_ff @(posedge clk or negedge rst_l) begin
        if (!rst_l) begin ad_busy_q <= 1'b0; ad_resp_pend_q <= 1'b0; end
        else begin
            if (ad_launch_fire) begin
                ad_busy_q    <= 1'b1;
                ad_is_ptw_q  <= present_ptw;
                ad_is_load_q <= present_dmem ? !dmem_req_write : !ptw_req_we;
                ad_op_q      <= (present_dmem ? dmem_req_write : ptw_req_we) ? COP_STORE : COP_LOAD;
                ad_addr_q    <= present_dmem ? dmem_req_addr  : ptw_req_addr;
                ad_wdata_q   <= present_dmem ? dmem_req_wdata : ptw_req_wdata;
                ad_wmask_q   <= present_dmem ? dmem_req_wmask : {XLEN_BYTES{1'b1}};
            end else if (ad_done) ad_busy_q <= 1'b0;
            // 1-deep response latch (dmem load only; PTW acks via ptw_req_ack below)
            if (ad_done && ad_is_load_q && !ad_is_ptw_q) begin
                ad_resp_pend_q <= 1'b1; ad_resp_data_q <= c_resp_rdata[0]; ad_resp_addr_q <= ad_addr_q;
            end else if (ad_resp_pend_q) ad_resp_pend_q <= 1'b0;
        end
    end

    // dmem response: device dummy (core overrides data) OR the registered agent load
    assign dmem_resp_valid = dev_resp_valid || ad_resp_pend_q;
    assign dmem_resp_addr  = dev_resp_valid ? dev_addr_q : ad_resp_addr_q;
    assign dmem_resp_data  = dev_resp_valid ? '0         : ad_resp_data_q;

    // PTW ack regenerated from the registered completion (read or A/D write both ack once)
    assign ptw_req_ack    = ad_done && ad_is_ptw_q;
    assign ptw_resp_rdata = c_resp_rdata[0];

    // ---- the grant-and-go CCD subsystem (agent + directory + behavioural interconnect) ----
    nmi_req_t  ccd_nmi_req;
    logic      ccd_nmi_ready;
    nmi_resp_t ccd_nmi_resp;

    // Hold the agent's flush off until the launch adapter has drained the in-flight op
    // (ad_can_accept) -- otherwise the agent gates block H on flush_req and the last
    // (often code-writing) store stuck in ad_busy never lands in data_q before the walk
    // Puts its line. The core holds dcache_flush_req and quiesces behind it, so the
    // adapter drains, then ccd_flush_req stays high for the whole walk (no new ops arrive).
    wire ccd_flush_req = dcache_flush_req && ad_can_accept;

    niigo_ccd_gg_direct #(.NACTIVE(1), .L1_SETS(CCD_L1_SETS), .DIR_SETS(CCD_DIR_SETS)) CCD (
        .clk, .rst_l,
        .c_req_valid(c_req_valid), .c_req_ready(c_req_ready),
        .c_req_op(c_req_op), .c_req_amo(c_req_amo),
        .c_req_waddr(c_req_waddr), .c_req_wdata(c_req_wdata), .c_req_wmask(c_req_wmask),
        .c_resp_rdata(c_resp_rdata), .c_resp_sc_ok(c_resp_sc_ok),
        .flush_req(ccd_flush_req), .flush_done(dcache_flush_done),
        .mem_req_o(ccd_nmi_req), .mem_req_ready_i(ccd_nmi_ready), .mem_resp_i(ccd_nmi_resp)
    );

    // PTW main_memory port retired (PTW now flows through the agent).
    assign mem_ptw_addr  = '0;
    assign mem_ptw_we    = 1'b0;
    assign mem_ptw_wdata = '0;
    logic unused_ccd;
    assign unused_ccd = (|mem_ptw_rdata) | mem_d_excpt | c_resp_sc_ok[0];
`endif /* CCD_AGENT */

`ifdef L1_CACHES
    // ==================================================================
    // Shared NMI backend: arbiter (L1D priority over L1I) + sim adapter onto
    // main_memory (reads on the I port, writes on the D port).
    // ------------------------------------------------------------------
    nmi_req_t  arb_m_req  [2];
    logic      arb_m_ready[2];
    nmi_resp_t arb_m_resp [2];
    nmi_req_t  arb_d_req;
    logic      arb_d_ready;
    nmi_resp_t arb_d_resp;

`ifdef L1D_CACHE
    assign arb_m_req[0]  = l1d_nmi_req;
    assign l1d_nmi_ready = arb_m_ready[0];
    assign l1d_nmi_resp  = arb_m_resp[0];
    // C4a: hold the L1I refill out of the arbiter until the L1D probe reports
    // the line clean (any dirty copy written back). Registered-derived on both
    // sides (L1I state, L1D probe_done) -- no combinational path.
    logic l1i_refill_gate;
    assign l1i_refill_gate = l1d_probe_valid && !l1d_probe_clean;
    assign arb_m_req[1]  = l1i_refill_gate ? '0 : l1i_nmi_req;
    assign l1i_nmi_ready = l1i_refill_gate ? 1'b0 : arb_m_ready[1];
`elsif CCD_AGENT
    // M3d: the grant-and-go directory's NMI master takes the L1D arbiter slot.
    assign arb_m_req[0]  = ccd_nmi_req;
    assign ccd_nmi_ready = arb_m_ready[0];
    assign ccd_nmi_resp  = arb_m_resp[0];
    // No clean-before-refill probe (C4 deferred): the L1I refills freely; fence.i
    // writeback-flush + ifetch_inval covers self-modifying code.
    assign arb_m_req[1]  = l1i_nmi_req;
    assign l1i_nmi_ready = arb_m_ready[1];
`else
    assign arb_m_req[0] = '0;          // L1D / CCD agent attaches here at L1D=1 / CCD=1
    assign arb_m_req[1]  = l1i_nmi_req;
    assign l1i_nmi_ready = arb_m_ready[1];
`endif
    assign l1i_nmi_resp  = arb_m_resp[1];

    nmi_arbiter #(.N_MASTERS(2)) Arb (
        .clk, .rst_l,
        .m_req(arb_m_req), .m_ready(arb_m_ready), .m_resp(arb_m_resp),
        .d_req(arb_d_req), .d_ready(arb_d_ready), .d_resp(arb_d_resp)
    );

    logic                          adp_wr_en;
    logic [MEMORY_ADDR_WIDTH-1:0]  adp_wr_addr;
    logic [XLEN-1:0]               adp_wr_data;
    logic [XLEN_BYTES-1:0]         adp_wr_mask;

`ifdef AXI_MEMSYS
    // AXI=1: NMI -> AXI4-512 bridge -> (protocol monitor) -> sim AXI slave ->
    // main_memory. The whole AXI link is internal to the memsys in sim; on the
    // FPGA the bridge's master is exposed to the shell instead of the shim.
    // Requires L1 (the bridge accepts line ops only).
    logic                  ax_awvalid, ax_awready;
    logic [AXI_ADDR_W-1:0] ax_awaddr;
    logic [AXI_ID_W-1:0]   ax_awid;
    logic [7:0]            ax_awlen;
    logic [2:0]            ax_awsize;
    logic [1:0]            ax_awburst;
    logic                  ax_wvalid, ax_wready, ax_wlast;
    logic [AXI_DATA_W-1:0] ax_wdata;
    logic [AXI_STRB_W-1:0] ax_wstrb;
    logic                  ax_bvalid, ax_bready;
    logic [AXI_ID_W-1:0]   ax_bid;
    logic [1:0]            ax_bresp;
    logic                  ax_arvalid, ax_arready;
    logic [AXI_ADDR_W-1:0] ax_araddr;
    logic [AXI_ID_W-1:0]   ax_arid;
    logic [7:0]            ax_arlen;
    logic [2:0]            ax_arsize;
    logic [1:0]            ax_arburst;
    logic                  ax_rvalid, ax_rready, ax_rlast;
    logic [AXI_ID_W-1:0]   ax_rid;
    logic [AXI_DATA_W-1:0] ax_rdata;
    logic [1:0]            ax_rresp;

    nmi_axi_bridge Bridge (
        .clk, .rst_l,
        .nmi_req(arb_d_req), .nmi_req_ready(arb_d_ready), .nmi_resp(arb_d_resp),
        .axi_awvalid(ax_awvalid), .axi_awready(ax_awready), .axi_awaddr(ax_awaddr),
        .axi_awid(ax_awid), .axi_awlen(ax_awlen), .axi_awsize(ax_awsize), .axi_awburst(ax_awburst),
        .axi_wvalid(ax_wvalid), .axi_wready(ax_wready), .axi_wdata(ax_wdata),
        .axi_wstrb(ax_wstrb), .axi_wlast(ax_wlast),
        .axi_bvalid(ax_bvalid), .axi_bready(ax_bready), .axi_bid(ax_bid), .axi_bresp(ax_bresp),
        .axi_arvalid(ax_arvalid), .axi_arready(ax_arready), .axi_araddr(ax_araddr),
        .axi_arid(ax_arid), .axi_arlen(ax_arlen), .axi_arsize(ax_arsize), .axi_arburst(ax_arburst),
        .axi_rvalid(ax_rvalid), .axi_rready(ax_rready), .axi_rid(ax_rid),
        .axi_rdata(ax_rdata), .axi_rresp(ax_rresp), .axi_rlast(ax_rlast)
    );

`ifdef FPGA_BUILD
    // FPGA: expose the bridge's AXI master to the SoC boundary (external DRAM).
    // No sim protocol monitor / shim; the sim main_memory ports are unused.
    assign m_axi_awvalid = ax_awvalid;   assign ax_awready    = m_axi_awready;
    assign m_axi_awaddr  = ax_awaddr;    assign m_axi_awid    = ax_awid;
    assign m_axi_awlen   = ax_awlen;     assign m_axi_awsize  = ax_awsize;
    assign m_axi_awburst = ax_awburst;
    assign m_axi_wvalid  = ax_wvalid;    assign ax_wready     = m_axi_wready;
    assign m_axi_wdata   = ax_wdata;     assign m_axi_wstrb   = ax_wstrb;
    assign m_axi_wlast   = ax_wlast;
    assign ax_bvalid     = m_axi_bvalid; assign m_axi_bready  = ax_bready;
    assign ax_bid        = m_axi_bid;    assign ax_bresp      = m_axi_bresp;
    assign m_axi_arvalid = ax_arvalid;   assign ax_arready    = m_axi_arready;
    assign m_axi_araddr  = ax_araddr;    assign m_axi_arid    = ax_arid;
    assign m_axi_arlen   = ax_arlen;     assign m_axi_arsize  = ax_arsize;
    assign m_axi_arburst = ax_arburst;
    assign ax_rvalid     = m_axi_rvalid; assign m_axi_rready  = ax_rready;
    assign ax_rid        = m_axi_rid;    assign ax_rdata      = m_axi_rdata;
    assign ax_rresp      = m_axi_rresp;  assign ax_rlast      = m_axi_rlast;
    assign mem_i_addr    = '0;
    assign adp_wr_en     = 1'b0;  assign adp_wr_addr = '0;
    assign adp_wr_data   = '0;    assign adp_wr_mask = '0;
    logic unused_fpga_in;
    assign unused_fpga_in = mem_i_excpt | (|mem_i_load_data);
`else
    axi_chk Chk (
        .clk, .rst_l,
        .axi_awvalid(ax_awvalid), .axi_awready(ax_awready), .axi_awaddr(ax_awaddr),
        .axi_awid(ax_awid), .axi_awlen(ax_awlen), .axi_awsize(ax_awsize), .axi_awburst(ax_awburst),
        .axi_wvalid(ax_wvalid), .axi_wready(ax_wready), .axi_wdata(ax_wdata), .axi_wlast(ax_wlast),
        .axi_bvalid(ax_bvalid), .axi_bready(ax_bready), .axi_bid(ax_bid),
        .axi_arvalid(ax_arvalid), .axi_arready(ax_arready), .axi_araddr(ax_araddr),
        .axi_arid(ax_arid), .axi_arlen(ax_arlen), .axi_arsize(ax_arsize), .axi_arburst(ax_arburst),
        .axi_rvalid(ax_rvalid), .axi_rready(ax_rready), .axi_rid(ax_rid), .axi_rlast(ax_rlast)
    );

    axi_mem_shim Shim (
        .clk, .rst_l,
        .axi_awvalid(ax_awvalid), .axi_awready(ax_awready), .axi_awaddr(ax_awaddr),
        .axi_awid(ax_awid),
        .axi_wvalid(ax_wvalid), .axi_wready(ax_wready), .axi_wdata(ax_wdata),
        .axi_wstrb(ax_wstrb), .axi_wlast(ax_wlast),
        .axi_bvalid(ax_bvalid), .axi_bready(ax_bready), .axi_bid(ax_bid), .axi_bresp(ax_bresp),
        .axi_arvalid(ax_arvalid), .axi_arready(ax_arready), .axi_araddr(ax_araddr),
        .axi_arid(ax_arid),
        .axi_rvalid(ax_rvalid), .axi_rready(ax_rready), .axi_rid(ax_rid),
        .axi_rdata(ax_rdata), .axi_rresp(ax_rresp), .axi_rlast(ax_rlast),
        .mem_rd_addr(mem_i_addr), .mem_rd_data(mem_i_load_data),
        .mem_wr_en(adp_wr_en), .mem_wr_addr(adp_wr_addr),
        .mem_wr_data(adp_wr_data), .mem_wr_mask(adp_wr_mask)
    );
    logic unused_axi_excpt; assign unused_axi_excpt = mem_i_excpt;
`endif
`else
    nmi_mem_adapter Adapter (
        .clk, .rst_l,
        .nmi_req(arb_d_req), .nmi_req_ready(arb_d_ready), .nmi_resp(arb_d_resp),
        .mem_rd_addr(mem_i_addr),
        .mem_rd_data(mem_i_load_data),
        .mem_rd_excpt(mem_i_excpt),
        .mem_wr_en(adp_wr_en), .mem_wr_addr(adp_wr_addr),
        .mem_wr_data(adp_wr_data), .mem_wr_mask(adp_wr_mask),
        .fz_en(fz_en), .fz_seed(fz_seed), .fz_min(fz_min), .fz_max(fz_max)
    );
`endif

`ifdef NIIGO_AGENT_DSIDE
    // Write-back D-side (L1D cache or M3d agent): line writebacks drive
    // main_memory's D write port (port 1) via the adapter/AXI shim.
    assign mem_d_addr       = adp_wr_addr;
    assign mem_d_store_mask = adp_wr_en ? adp_wr_mask : '0;
    assign mem_d_store_data = adp_wr_data;
    assign mem_d_load_en    = 1'b0;
`else
    // C1: the adapter writes nothing (the D passthrough owns the D port).
    logic unused_l1i_wr;
    assign unused_l1i_wr = adp_wr_en | (|adp_wr_addr) | (|adp_wr_data) |
        (|adp_wr_mask) | l1i_ev_access;
`endif
`endif /* L1_CACHES */

    // ---- Cache event pulses for mhpmcounter3-5 (phase C3) ----
`ifdef L1_CACHES
    assign hpm_l1i_miss = l1i_ev_miss;
  `ifdef L1D_CACHE
    assign hpm_l1d_miss = l1d_ev_miss;
    assign hpm_l1d_wb   = l1d_ev_wb;
  `else
    assign hpm_l1d_miss = 1'b0;
    assign hpm_l1d_wb   = 1'b0;
  `endif
`else
    assign hpm_l1i_miss = 1'b0;
    assign hpm_l1d_miss = 1'b0;
    assign hpm_l1d_wb   = 1'b0;
`endif

`ifdef AGENT_DEBUG
    // End-of-sim cache statistics (counters + a final summary line).
    logic [63:0] stat_i_acc, stat_i_miss, stat_d_acc, stat_d_miss, stat_d_wb;
    always_ff @(posedge clk or negedge rst_l) begin
        if (!rst_l) begin
            stat_i_acc <= '0; stat_i_miss <= '0;
            stat_d_acc <= '0; stat_d_miss <= '0; stat_d_wb <= '0;
        end else begin
`ifdef L1_CACHES
            if (l1i_ev_access) stat_i_acc  <= stat_i_acc  + 64'd1;
            if (l1i_ev_miss)   stat_i_miss <= stat_i_miss + 64'd1;
  `ifdef L1D_CACHE
            if (l1d_ev_access) stat_d_acc  <= stat_d_acc  + 64'd1;
            if (l1d_ev_miss)   stat_d_miss <= stat_d_miss + 64'd1;
            if (l1d_ev_wb)     stat_d_wb   <= stat_d_wb   + 64'd1;
  `endif
`endif
        end
    end
    final begin
        $display("L1 STATS: L1I acc=%0d miss=%0d | L1D acc=%0d miss=%0d wb=%0d",
            stat_i_acc, stat_i_miss, stat_d_acc, stat_d_miss, stat_d_wb);
    end
`endif

endmodule: niigo_memsys

`ifdef NIIGO_AGENT_DSIDE
`undef NIIGO_AGENT_DSIDE
`endif
