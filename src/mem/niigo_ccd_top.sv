/**
 * niigo_ccd_top.sv  --  M1 two-core CCD coherence subsystem (wrapper)
 *
 * Packages the M1 MOESI coherence subsystem as one synthesizable block: NACTIVE private L1D
 * agents (niigo_l1d_moesi) + the directory (niigo_dir), wired in the star topology (each core
 * <-> directory over the M1 full-line CMI links), with a single NMI master out to the backing
 * memory. Cores attach via per-core word-granular LSQ ports; M3 replaces these stub LSQ ports
 * with the real riscv_core_ooo LSQ/fetch + the wheel fabric (grant-and-go dir, ack-to-requester,
 * flit serialisation). This wrapper is what tb_niigo_ccd_m1 exercises (14/14 coherence checks).
 *
 * The directory is sized to the package NUM_CORES (4) so core-id widths match the CMI message;
 * M1 drives NACTIVE (=2) of them and ties the rest idle.
 */
`include "niigo_mem.vh"
`include "niigo_cmi.vh"
`include "niigo_ccd_m1.vh"
`default_nettype none
module niigo_ccd_top
    import RISCV_ISA::XLEN;
    import RISCV_UArch::MEMORY_ADDR_WIDTH;
    import NIIGO_Mem::*;
    import NIIGO_CMI::NUM_CORES;
    import NIIGO_CCD_M1::*;
#(
    parameter int NACTIVE  = 2,                 // cores actually driven (M1 = 2)
    parameter int DIR_SETS = 8,
    parameter int L1_SETS  = 8                  // per-agent L1D sets (M1 bring-up size)
)(
    input  wire logic clk,
    input  wire logic rst_l,

    // ---- per-core LSQ ports (word-granular; blocking: hold req until c_req_ready) ----
    input  wire logic                          c_req_valid [NACTIVE],
    output logic                               c_req_ready [NACTIVE],
    input  wire l1_core_op_e                   c_req_op    [NACTIVE],
    input  wire l1_amo_op_e                    c_req_amo   [NACTIVE],
    input  wire logic [MEMORY_ADDR_WIDTH-1:0]  c_req_waddr [NACTIVE],
    input  wire logic [XLEN-1:0]               c_req_wdata [NACTIVE],
    output logic [XLEN-1:0]                     c_resp_rdata[NACTIVE],
    output logic                               c_resp_sc_ok[NACTIVE],

    // ---- backing memory (NMI master) ----
    output nmi_req_t   mem_req_o,
    input  wire logic  mem_req_ready_i,
    input  nmi_resp_t  mem_resp_i
);
    localparam int CORES = NUM_CORES;           // directory width (matches ccd_msg_t core ids)

    ccd_chan_t dir_up   [CORES]; logic dir_up_ready  [CORES];
    ccd_chan_t dir_down [CORES]; logic dir_down_ready[CORES];

    niigo_dir #(.CORES(CORES), .DIR_SETS(DIR_SETS)) DIR (
        .clk, .rst_l,
        .up_i(dir_up), .up_ready_o(dir_up_ready),
        .down_o(dir_down), .down_ready_i(dir_down_ready),
        .mem_req_o(mem_req_o), .mem_req_ready_i(mem_req_ready_i), .mem_resp_i(mem_resp_i)
    );

    genvar gi;
    generate
        for (gi = 0; gi < NACTIVE; gi++) begin : G_CORE
            ccd_chan_t a_up, a_down; logic a_up_ready, a_down_ready;
            niigo_l1d_moesi #(.CORE_ID(gi), .SETS(L1_SETS)) L1D (
                .clk, .rst_l,
                .c_req_valid (c_req_valid [gi]), .c_req_ready (c_req_ready [gi]),
                .c_req_op    (c_req_op    [gi]), .c_req_amo   (c_req_amo   [gi]),
                .c_req_waddr (c_req_waddr [gi]), .c_req_wdata (c_req_wdata [gi]),
                .c_resp_rdata(c_resp_rdata[gi]), .c_resp_sc_ok(c_resp_sc_ok[gi]),
                .up_o(a_up), .up_ready_i(a_up_ready),
                .down_i(a_down), .down_ready_o(a_down_ready)
            );
            assign dir_up[gi]         = a_up;
            assign a_up_ready         = dir_up_ready[gi];
            assign a_down             = dir_down[gi];
            assign dir_down_ready[gi] = a_down_ready;
        end
        // tie the directory's unused core ports idle
        for (gi = NACTIVE; gi < CORES; gi++) begin : G_IDLE
            assign dir_up[gi]         = '0;
            assign dir_down_ready[gi] = 1'b1;
        end
    endgenerate

endmodule
`default_nettype wire
