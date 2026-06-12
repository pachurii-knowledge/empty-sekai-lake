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

`default_nettype none

module niigo_memsys
    import RISCV_ISA::XLEN, RISCV_ISA::XLEN_BYTES;
    import RISCV_UArch::MEMORY_READ_WIDTH, RISCV_UArch::MEMORY_ADDR_WIDTH;
(
    input  logic clk,
    input  logic rst_l,

    // ---- Core: instruction fetch port ----
    input  logic                          ifetch_req_valid,
    output logic                          ifetch_req_ready,
    input  logic [MEMORY_ADDR_WIDTH-1:0]  ifetch_req_addr,
    output logic                          ifetch_resp_valid,
    output logic [MEMORY_READ_WIDTH-1:0][XLEN-1:0] ifetch_resp_data,
    output logic                          ifetch_resp_excpt,

    // ---- Core: data port ----
    input  logic                          dmem_req_valid,
    output logic                          dmem_req_ready,
    input  logic                          dmem_req_write,
    input  logic [MEMORY_ADDR_WIDTH-1:0]  dmem_req_addr,
    input  logic [XLEN-1:0]               dmem_req_wdata,
    input  logic [XLEN_BYTES-1:0]         dmem_req_wmask,
    output logic                          dmem_resp_valid,
    output logic [MEMORY_ADDR_WIDTH-1:0]  dmem_resp_addr,
    output logic [XLEN-1:0]               dmem_resp_data,

    // ---- Core: page-table-walker port ----
    input  logic                          ptw_req_valid,
    input  logic                          ptw_req_we,
    input  logic [MEMORY_ADDR_WIDTH-1:0]  ptw_req_addr,
    input  logic [XLEN-1:0]               ptw_req_wdata,
    output logic                          ptw_req_ack,
    output logic [XLEN-1:0]               ptw_resp_rdata,

    // ---- Memory side (testbench wires these to main_memory) ----
    output logic                          mem_d_load_en,
    output logic [XLEN_BYTES-1:0]         mem_d_store_mask,
    output logic [MEMORY_ADDR_WIDTH-1:0]  mem_d_addr,
    output logic [XLEN-1:0]               mem_d_store_data,
    input  logic [MEMORY_READ_WIDTH-1:0][XLEN-1:0] mem_d_load_data,
    input  logic                          mem_d_excpt,
    output logic [MEMORY_ADDR_WIDTH-1:0]  mem_i_addr,
    input  logic [MEMORY_READ_WIDTH-1:0][XLEN-1:0] mem_i_load_data,
    input  logic                          mem_i_excpt,
    output logic [MEMORY_ADDR_WIDTH-1:0]  mem_ptw_addr,
    output logic                          mem_ptw_we,
    output logic [XLEN-1:0]               mem_ptw_wdata,
    input  logic [XLEN-1:0]               mem_ptw_rdata
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

    // ------------------------------------------------------------------
    // Instruction port: combinational read at the request address in the
    // acceptance cycle; {excpt, data} ride an in-order response queue whose
    // entries become deliverable after a fixed (default) or randomized
    // (fuzz) latency. At most one response delivers per cycle, preserving
    // arrival-order association in the core.
    // ------------------------------------------------------------------
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

    // ------------------------------------------------------------------
    // Data port: loads sample memory in the acceptance cycle and ride the
    // same kind of in-order response queue (with the request word address
    // echoed for the device decode at delivery). Stores forward to the
    // memory write port in the acceptance cycle and apply at the next
    // clock edge; they produce no response.
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

endmodule: niigo_memsys
