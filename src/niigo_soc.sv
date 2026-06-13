/**
 * niigo_soc.sv  (phase FB1 boundary / FB2 full-SoC synth)
 *
 * Synthesizable top-level SoC: the 4-wide OoO core plus the full memory
 * subsystem (split L1I/L1D caches + NMI line bus + NMI->AXI4-512 bridge),
 * exposing a single AXI4 (512-bit) master to external DRAM. The memory-mapped
 * devices (CLINT/PLIC/UART) live inside the core, so only cacheable traffic
 * leaves through the AXI master; device space never appears on AXI.
 *
 * This is the unit an FPGA shell wrapper (FB1 cl_niigo) instantiates: connect
 * m_axi_* to the shell DRAM controller (sh_ddr) and add the OCL control plane +
 * vUART around it. Built with FPGA_BUILD (implies AXI_MEMSYS; the niigo_memsys
 * sim shim/monitor are replaced by the exposed AXI ports) + L1_CACHES +
 * L1D_CACHE + OOO_4WIDE (+ RV64 for the xv6/Linux datapath).
 *
 * Only compiled in the FPGA build; the simulation testbench wires the core and
 * niigo_memsys directly onto main_memory instead.
 */

`include "riscv_isa.vh"
`include "riscv_uarch.vh"
`include "niigo_mem.vh"

`default_nettype none

`ifdef FPGA_BUILD
module niigo_soc
    import RISCV_ISA::XLEN, RISCV_ISA::XLEN_BYTES;
    import RISCV_UArch::MEMORY_READ_WIDTH, RISCV_UArch::MEMORY_ADDR_WIDTH;
    import NIIGO_Mem::*;
(
    input  wire logic                  clk,
    input  wire logic                  rst_l,
    output logic                       halted,

    // ---- AXI4 master to external DRAM (512-bit data, single-beat line bursts) ----
    output logic                       m_axi_awvalid,
    input  wire logic                  m_axi_awready,
    output logic [AXI_ADDR_W-1:0]      m_axi_awaddr,
    output logic [AXI_ID_W-1:0]        m_axi_awid,
    output logic [7:0]                 m_axi_awlen,
    output logic [2:0]                 m_axi_awsize,
    output logic [1:0]                 m_axi_awburst,
    output logic                       m_axi_wvalid,
    input  wire logic                  m_axi_wready,
    output logic [AXI_DATA_W-1:0]      m_axi_wdata,
    output logic [AXI_STRB_W-1:0]      m_axi_wstrb,
    output logic                       m_axi_wlast,
    input  wire logic                  m_axi_bvalid,
    output logic                       m_axi_bready,
    input  wire logic [AXI_ID_W-1:0]   m_axi_bid,
    input  wire logic [1:0]            m_axi_bresp,
    output logic                       m_axi_arvalid,
    input  wire logic                  m_axi_arready,
    output logic [AXI_ADDR_W-1:0]      m_axi_araddr,
    output logic [AXI_ID_W-1:0]        m_axi_arid,
    output logic [7:0]                 m_axi_arlen,
    output logic [2:0]                 m_axi_arsize,
    output logic [1:0]                 m_axi_arburst,
    input  wire logic                  m_axi_rvalid,
    output logic                       m_axi_rready,
    input  wire logic [AXI_ID_W-1:0]   m_axi_rid,
    input  wire logic [AXI_DATA_W-1:0] m_axi_rdata,
    input  wire logic [1:0]            m_axi_rresp,
    input  wire logic                  m_axi_rlast
);

    // ---- core <-> memsys handshaked ports ----
    logic                                    ifetch_req_valid, ifetch_req_ready;
    logic [MEMORY_ADDR_WIDTH-1:0]            ifetch_req_addr;
    logic                                    ifetch_resp_valid, ifetch_resp_excpt;
    logic [MEMORY_READ_WIDTH-1:0][XLEN-1:0]  ifetch_resp_data;

    logic                                    dmem_req_valid, dmem_req_ready, dmem_req_write;
    logic [MEMORY_ADDR_WIDTH-1:0]            dmem_req_addr;
    logic [XLEN-1:0]                         dmem_req_wdata;
    logic [XLEN_BYTES-1:0]                   dmem_req_wmask;
    logic                                    dmem_resp_valid;
    logic [MEMORY_ADDR_WIDTH-1:0]            dmem_resp_addr;
    logic [XLEN-1:0]                         dmem_resp_data;

    logic                                    ptw_mem_req, ptw_mem_we, ptw_mem_ack;
    logic [MEMORY_ADDR_WIDTH-1:0]            ptw_mem_addr_w;
    logic [XLEN-1:0]                         ptw_mem_wdata, ptw_mem_rdata;

    logic ifetch_inval, dmem_req_device, dcache_flush_req, dcache_flush_done;
    logic hpm_l1i_miss, hpm_l1d_miss, hpm_l1d_wb;

    riscv_core_ooo Core (
        .clk, .rst_l,
        .ifetch_req_valid, .ifetch_req_ready, .ifetch_req_addr,
        .ifetch_resp_valid, .ifetch_resp_data, .ifetch_resp_excpt,
        .dmem_req_valid, .dmem_req_ready, .dmem_req_write, .dmem_req_addr,
        .dmem_req_wdata, .dmem_req_wmask,
        .dmem_resp_valid, .dmem_resp_addr, .dmem_resp_data,
        .ptw_mem_req, .ptw_mem_we, .ptw_mem_addr_w, .ptw_mem_wdata,
        .ptw_mem_ack, .ptw_mem_rdata,
        .ifetch_inval, .dmem_req_device, .dcache_flush_req, .dcache_flush_done,
        .hpm_l1i_miss, .hpm_l1d_miss, .hpm_l1d_wb,
        .halted
    );

    niigo_memsys MemSys (
        .clk, .rst_l,
        .ifetch_req_valid, .ifetch_req_ready, .ifetch_req_addr,
        .ifetch_resp_valid, .ifetch_resp_data, .ifetch_resp_excpt,
        .ifetch_inval, .dmem_req_device, .dcache_flush_req, .dcache_flush_done,
        .hpm_l1i_miss, .hpm_l1d_miss, .hpm_l1d_wb,
        .dmem_req_valid, .dmem_req_ready, .dmem_req_write, .dmem_req_addr,
        .dmem_req_wdata, .dmem_req_wmask,
        .dmem_resp_valid, .dmem_resp_addr, .dmem_resp_data,
        .ptw_req_valid(ptw_mem_req), .ptw_req_we(ptw_mem_we),
        .ptw_req_addr(ptw_mem_addr_w), .ptw_req_wdata(ptw_mem_wdata),
        .ptw_req_ack(ptw_mem_ack), .ptw_resp_rdata(ptw_mem_rdata),
        // sim main_memory side: unused on FPGA (outputs open, inputs tied off).
        .mem_d_load_en(), .mem_d_store_mask(), .mem_d_addr(), .mem_d_store_data(),
        .mem_d_load_data('0), .mem_d_excpt(1'b0),
        .mem_i_addr(), .mem_i_load_data('0), .mem_i_excpt(1'b0),
        .mem_ptw_addr(), .mem_ptw_we(), .mem_ptw_wdata(), .mem_ptw_rdata('0),
        // AXI4-512 master to DRAM.
        .m_axi_awvalid, .m_axi_awready, .m_axi_awaddr, .m_axi_awid, .m_axi_awlen,
        .m_axi_awsize, .m_axi_awburst,
        .m_axi_wvalid, .m_axi_wready, .m_axi_wdata, .m_axi_wstrb, .m_axi_wlast,
        .m_axi_bvalid, .m_axi_bready, .m_axi_bid, .m_axi_bresp,
        .m_axi_arvalid, .m_axi_arready, .m_axi_araddr, .m_axi_arid, .m_axi_arlen,
        .m_axi_arsize, .m_axi_arburst,
        .m_axi_rvalid, .m_axi_rready, .m_axi_rid, .m_axi_rdata, .m_axi_rresp, .m_axi_rlast
    );

endmodule : niigo_soc
`endif

`default_nettype wire
