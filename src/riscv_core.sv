/**
 * riscv_core.sv
 *
 * Phase 2 wrapper. The known-good Phase 1 scalar implementation is preserved in
 * riscv_core_scalar. Defining SUPERSCALAR_4WIDE selects the conservative 4-wide
 * in-order implementation.
 *
 * Under OOO_4WIDE the wrapper exposes the handshaked memory boundary
 * (instruction-fetch / data / PTW request-response ports served by
 * niigo_memsys) instead of the legacy fixed-latency Lab 4 interface; the
 * in-order cores keep the legacy interface unchanged.
 */

`include "riscv_isa.vh"
`include "riscv_uarch.vh"

`default_nettype none

module riscv_core
    import RISCV_ISA::XLEN, RISCV_ISA::XLEN_BYTES;
    import RISCV_UArch::MEMORY_READ_WIDTH, RISCV_UArch::MEMORY_ADDR_WIDTH;
#(
    parameter logic [XLEN-1:0] HART_ID = '0   // M4: per-core mhartid (OoO build; default 0 = single core)
)
`ifdef OOO_4WIDE
(
    input wire logic             clk, rst_l,
    // Instruction-fetch port (handshaked; see riscv_core_ooo)
    output logic             ifetch_req_valid,
    input wire logic             ifetch_req_ready,
    output logic [MEMORY_ADDR_WIDTH-1:0] ifetch_req_addr,
    input wire logic             ifetch_resp_valid,
    input wire logic [MEMORY_READ_WIDTH-1:0][XLEN-1:0] ifetch_resp_data,
    input wire logic             ifetch_resp_excpt,
    // Data port (handshaked)
    output logic             dmem_req_valid,
    input wire logic             dmem_req_ready,
    output logic             dmem_req_write,
    output logic [MEMORY_ADDR_WIDTH-1:0] dmem_req_addr,
    output logic [XLEN-1:0]  dmem_req_wdata,
    output logic [XLEN_BYTES-1:0]        dmem_req_wmask,
    output logic [2:0]       dmem_req_op,        // M3d Stage 2: typed op (CCD agent)
    input wire logic             dmem_resp_valid,
    input wire logic [MEMORY_ADDR_WIDTH-1:0] dmem_resp_addr,
    input wire logic [XLEN-1:0]  dmem_resp_data,
    input wire logic             dmem_snoop_kill_valid,   // M3d Stage 3: CCD snoop-kill -> LSQ
    input wire logic [MEMORY_ADDR_WIDTH-1:0] dmem_snoop_kill_laddr,
    // Page-table-walk port (req/ack)
    output logic             ptw_mem_req,
    output logic             ptw_mem_we,
    output logic [MEMORY_ADDR_WIDTH-1:0] ptw_mem_addr_w,
    output logic [XLEN-1:0]  ptw_mem_wdata,
    input wire logic             ptw_mem_ack,
    input wire logic [XLEN-1:0]  ptw_mem_rdata,
    output logic             ifetch_inval,
    output logic             dmem_req_device,
    output logic             dcache_flush_req,
    input wire logic             dcache_flush_done,
    input wire logic             hpm_l1i_miss,
    input wire logic             hpm_l1d_miss,
    input wire logic             hpm_l1d_wb,
    output logic             halted
);

    riscv_core_ooo #(.HART_ID(HART_ID)) OoOCore (
        .clk,
        .rst_l,
        .ifetch_req_valid,
        .ifetch_req_ready,
        .ifetch_req_addr,
        .ifetch_resp_valid,
        .ifetch_resp_data,
        .ifetch_resp_excpt,
        .dmem_req_valid,
        .dmem_req_ready,
        .dmem_req_write,
        .dmem_req_addr,
        .dmem_req_wdata,
        .dmem_req_wmask,
        .dmem_req_op,
        .dmem_resp_valid,
        .dmem_resp_addr,
        .dmem_resp_data,
        .dmem_snoop_kill_valid,
        .dmem_snoop_kill_laddr,
        .ptw_mem_req,
        .ptw_mem_we,
        .ptw_mem_addr_w,
        .ptw_mem_wdata,
        .ptw_mem_ack,
        .ptw_mem_rdata,
        .ifetch_inval,
        .dmem_req_device,
        .dcache_flush_req,
        .dcache_flush_done,
        .hpm_l1i_miss,
        .hpm_l1d_miss,
        .hpm_l1d_wb,
        .halted
    );

`else /* !OOO_4WIDE: legacy fixed-latency Lab 4 interface */
(
    input wire logic             clk, rst_l, instr_mem_excpt, data_mem_excpt,
    input wire logic [MEMORY_READ_WIDTH-1:0][XLEN-1:0] instr, data_load,
    input wire logic [MEMORY_ADDR_WIDTH-1:0] data_load_addr,
    input wire logic             data_load_valid,
    output logic             data_load_en, halted,
    output logic [XLEN_BYTES-1:0]        data_store_mask,
    output logic [MEMORY_ADDR_WIDTH-1:0] instr_addr, data_addr,
    output logic             instr_stall, data_stall,
    output logic [XLEN-1:0]  data_store,
    // MMU page-table-walk port
    output logic [MEMORY_ADDR_WIDTH-1:0] ptw_addr,
    output logic             ptw_we,
    output logic [XLEN-1:0]  ptw_wdata,
    input wire logic [XLEN-1:0]  ptw_rdata
);

    generate
        if (RISCV_UArch::SUPERSCALAR_WAYS == 4) begin : gen_4wide
            riscv_core_4wide Core4Wide (
                .clk,
                .rst_l,
                .instr_mem_excpt,
                .data_mem_excpt,
                .instr,
                .data_load,
                .data_load_addr,
                .data_load_valid,
                .data_load_en,
                .halted,
                .data_store_mask,
                .instr_addr,
                .data_addr,
                .instr_stall,
                .data_stall,
                .data_store,
                .ptw_addr,
                .ptw_we,
                .ptw_wdata,
                .ptw_rdata
            );
        end else begin : gen_scalar
            riscv_core_scalar ScalarCore (
                .clk,
                .rst_l,
                .instr_mem_excpt,
                .data_mem_excpt,
                .instr,
                .data_load,
                .data_load_addr,
                .data_load_valid,
                .data_load_en,
                .halted,
                .data_store_mask,
                .instr_addr,
                .data_addr,
                .instr_stall,
                .data_stall,
                .data_store,
                .ptw_addr,
                .ptw_we,
                .ptw_wdata,
                .ptw_rdata
            );
        end
    endgenerate

`endif /* OOO_4WIDE */

endmodule: riscv_core
