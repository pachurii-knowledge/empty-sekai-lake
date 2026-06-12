/**
 * niigo_memsys.sv
 *
 * Memory subsystem for the out-of-order core (phase N1: passthrough).
 *
 * This module is the single seam between the OoO core and backing memory.
 * The core side speaks three handshaked ports (the core makes NO latency
 * assumptions about any of them):
 *
 *   - ifetch: one 16-byte instruction block per request (valid/ready), with
 *     an in-order response stream {valid, data, excpt}. No request ID: the
 *     core associates responses with requests by arrival order.
 *   - dmem: one word-granular load OR store per request (valid/ready).
 *     Loads produce an in-order response {valid, addr echo, data, (excpt)};
 *     stores are fire-and-forget once accepted (the memsys guarantees that
 *     an accepted store is ordered before any later-accepted D-side access).
 *     The address echo exists for the memory-mapped devices, which decode
 *     the returning load's physical address at delivery time.
 *   - ptw: the page-table walker's level req/ack word port (reads and A/D
 *     writebacks). rdata is valid in the ack cycle.
 *
 * The memory side mirrors the (DO NOT MODIFY) main_memory port shapes; the
 * testbench wires it to the same dual-port memory + PTW port the legacy
 * cores use. Reads sample memory combinationally in the acceptance cycle
 * and the data rides a response pipe, so the value observed is the memory
 * state at the point of acceptance (exactly the legacy delay_buffer
 * semantics); stores apply at the acceptance clock edge.
 *
 * Passthrough configuration: requests are always accepted (ready == 1) and
 * responses arrive a fixed I_RESP_DELAY / D_RESP_DELAY cycles later,
 * matching the legacy testbench's IMEMORY_READ_DELAY / DMEMORY_READ_DELAY
 * so the N1 refactor preserves the core's timing character. Phase N2 adds
 * plusarg-driven randomized acceptance/response delays here; phases C1/C2
 * replace the passthrough internals with the L1 caches + line bus.
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

    // Fixed response latencies (>= 1 so a response never lands in the cycle
    // its request was accepted). Reuse the legacy lab constants so the
    // passthrough build keeps the timing character the suites were tuned on.
    localparam int I_RESP_DELAY =
        (RISCV_UArch::IMEMORY_READ_DELAY < 1) ? 1 : RISCV_UArch::IMEMORY_READ_DELAY;
    localparam int D_RESP_DELAY =
        (RISCV_UArch::DMEMORY_READ_DELAY < 1) ? 1 : RISCV_UArch::DMEMORY_READ_DELAY;

    // ------------------------------------------------------------------
    // Instruction port: combinational read at the request address in the
    // acceptance cycle; {fire, excpt, data} ride an I_RESP_DELAY-deep pipe.
    // ------------------------------------------------------------------
    assign ifetch_req_ready = 1'b1;
    assign mem_i_addr = ifetch_req_addr;

    typedef struct packed {
        logic                                  valid;
        logic                                  excpt;
        logic [MEMORY_READ_WIDTH-1:0][XLEN-1:0] data;
    } i_resp_t;

    i_resp_t i_pipe_q [I_RESP_DELAY];

    always_ff @(posedge clk or negedge rst_l) begin
        if (!rst_l) begin
            for (int i = 0; i < I_RESP_DELAY; i += 1) begin
                i_pipe_q[i] <= '0;
            end
        end else begin
            i_pipe_q[0].valid <= ifetch_req_valid && ifetch_req_ready;
            i_pipe_q[0].excpt <= mem_i_excpt;
            i_pipe_q[0].data  <= mem_i_load_data;
            for (int i = 1; i < I_RESP_DELAY; i += 1) begin
                i_pipe_q[i] <= i_pipe_q[i-1];
            end
        end
    end

    assign ifetch_resp_valid = i_pipe_q[I_RESP_DELAY-1].valid;
    assign ifetch_resp_excpt = i_pipe_q[I_RESP_DELAY-1].excpt;
    assign ifetch_resp_data  = i_pipe_q[I_RESP_DELAY-1].data;

    // ------------------------------------------------------------------
    // Data port: loads sample memory in the acceptance cycle and ride a
    // D_RESP_DELAY-deep pipe (with the request word address echoed for the
    // device decode at delivery). Stores forward to the memory write port
    // in the acceptance cycle and apply at the next clock edge.
    // ------------------------------------------------------------------
    assign dmem_req_ready = 1'b1;

    logic dmem_load_fire, dmem_store_fire;
    assign dmem_load_fire  = dmem_req_valid && dmem_req_ready && !dmem_req_write;
    assign dmem_store_fire = dmem_req_valid && dmem_req_ready &&  dmem_req_write;

    assign mem_d_addr       = dmem_req_addr;
    assign mem_d_load_en    = dmem_load_fire;
    assign mem_d_store_mask = dmem_store_fire ? dmem_req_wmask : '0;
    assign mem_d_store_data = dmem_req_wdata;

    typedef struct packed {
        logic                         valid;
        logic [MEMORY_ADDR_WIDTH-1:0] addr;
        logic [XLEN-1:0]              data;
    } d_resp_t;

    d_resp_t d_pipe_q [D_RESP_DELAY];

    always_ff @(posedge clk or negedge rst_l) begin
        if (!rst_l) begin
            for (int i = 0; i < D_RESP_DELAY; i += 1) begin
                d_pipe_q[i] <= '0;
            end
        end else begin
            d_pipe_q[0].valid <= dmem_load_fire;
            d_pipe_q[0].addr  <= dmem_req_addr;
            d_pipe_q[0].data  <= mem_d_load_data[0];
            for (int i = 1; i < D_RESP_DELAY; i += 1) begin
                d_pipe_q[i] <= d_pipe_q[i-1];
            end
        end
    end

    assign dmem_resp_valid = d_pipe_q[D_RESP_DELAY-1].valid;
    assign dmem_resp_addr  = d_pipe_q[D_RESP_DELAY-1].addr;
    assign dmem_resp_data  = d_pipe_q[D_RESP_DELAY-1].data;

    // The data-port memory exception is unused by the OoO core (faults are
    // raised by the PMP/translation checks before an access is issued).
    logic unused_d_excpt;
    assign unused_d_excpt = mem_d_excpt;

    // ------------------------------------------------------------------
    // PTW port: combinational passthrough (ack in the request cycle, rdata
    // valid with ack; A/D writes apply at the next clock edge) -- identical
    // to the legacy direct connection. The walker holds req/we as levels
    // until ack, so a delayed ack (N2 fuzz) simply stretches the state.
    // ------------------------------------------------------------------
    assign mem_ptw_addr   = ptw_req_addr;
    assign mem_ptw_we     = ptw_req_valid && ptw_req_we;
    assign mem_ptw_wdata  = ptw_req_wdata;
    assign ptw_req_ack    = ptw_req_valid;
    assign ptw_resp_rdata = mem_ptw_rdata;

endmodule: niigo_memsys
