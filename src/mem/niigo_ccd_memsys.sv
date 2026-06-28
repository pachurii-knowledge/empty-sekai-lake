/**
 * niigo_ccd_memsys.sv  --  P4: multi-core coherent memory subsystem (the SMP "SoC memsys").
 *
 * Generalises the single-core niigo_memsys CCD arm to NACTIVE real cores sharing ONE
 * grant-and-go MOESI directory (niigo_ccd_gg_direct #(.NACTIVE)). Per core it instantiates
 * the SAME proven pieces as niigo_memsys's CCD arm -- a REAL l1_icache, the registered launch
 * adapter (cacheable dmem + PTW muxed onto the agent's one c_req, devices bypassed), the P2
 * coherent-L1I probe, and the FwdGetM/INV snoop-invalidate -- plus the P4 remote-dirty I-fetch:
 * on an L1I refill the local L1D agent is probed; on a probe MISS a COP_LOAD is injected onto
 * c_req so the directory's GetS pulls the line cold-from-memory OR dirty-from-the-owner, installs
 * it locally, and the probe-serve then hands it to the L1I. The L1I therefore never reads backing
 * memory directly, so a line another core holds dirty is always fetched coherently (no fence.i).
 *
 * Because every L1I refill is agent-served, the data-side backend is the directory's single NMI
 * master -- there is NO per-core L1I->memory path and no N+1 backend arbiter. The harness wires
 * mem_req_o/mem_resp_i to a backend (sim line memory, or nmi_mem_adapter->main_memory for xv6).
 *
 * Devices (CLINT/PLIC/UART) are SHARED at the harness via NIIGO_EXT_DEVICES: this memsys merely
 * bypasses device dmem off the coherent path and times the device-load completion (the core
 * overrides the data from the shared device). Single-outstanding per core (the LSQ serialises).
 */
`include "niigo_mem.vh"
`include "niigo_cmi.vh"
`include "niigo_ccd_m1.vh"
`default_nettype none

module niigo_ccd_memsys
    import RISCV_ISA::XLEN, RISCV_ISA::XLEN_BYTES;
    import RISCV_UArch::MEMORY_READ_WIDTH, RISCV_UArch::MEMORY_ADDR_WIDTH;
    import NIIGO_Mem::*;
    import NIIGO_CCD_M1::*;
#(
    parameter int NACTIVE  = 2,
    parameter int L1_SETS  = 64,    // directory agent (L1D) sets
    parameter int DIR_SETS = 256,   // directory >> L1 (avoid dir-capacity eviction)
    parameter int RESP_DLY = 2      // CCD interconnect grant-delivery delay
)(
    input  wire logic clk, rst_l,

    // ---- per-core instruction-fetch ports ----
    input  wire logic                          ifetch_req_valid [NACTIVE],
    output logic                               ifetch_req_ready [NACTIVE],
    input  wire logic [MEMORY_ADDR_WIDTH-1:0]  ifetch_req_addr  [NACTIVE],
    output logic                               ifetch_resp_valid[NACTIVE],
    output logic [MEMORY_READ_WIDTH-1:0][XLEN-1:0] ifetch_resp_data [NACTIVE],
    output logic                               ifetch_resp_excpt[NACTIVE],
    input  wire logic                          ifetch_inval     [NACTIVE],

    // ---- per-core data ports ----
    input  wire logic                          dmem_req_valid [NACTIVE],
    output logic                               dmem_req_ready [NACTIVE],
    input  wire logic                          dmem_req_write [NACTIVE],
    input  wire logic [MEMORY_ADDR_WIDTH-1:0]  dmem_req_addr  [NACTIVE],
    input  wire logic [XLEN-1:0]               dmem_req_wdata [NACTIVE],
    input  wire logic [XLEN_BYTES-1:0]         dmem_req_wmask [NACTIVE],
    input  wire logic [2:0]                    dmem_req_op    [NACTIVE],
    input  wire logic [3:0]                    dmem_req_amo   [NACTIVE],
    input  wire logic                          dmem_req_device[NACTIVE],
    output logic                               dmem_resp_valid[NACTIVE],
    output logic [MEMORY_ADDR_WIDTH-1:0]       dmem_resp_addr [NACTIVE],
    output logic [XLEN-1:0]                    dmem_resp_data [NACTIVE],
    output logic                               dmem_snoop_kill_valid[NACTIVE],
    output logic [MEMORY_ADDR_WIDTH-1:0]       dmem_snoop_kill_laddr[NACTIVE],

    // ---- per-core page-table-walker ports ----
    input  wire logic                          ptw_req_valid [NACTIVE],
    input  wire logic                          ptw_req_we    [NACTIVE],
    input  wire logic [MEMORY_ADDR_WIDTH-1:0]  ptw_req_addr  [NACTIVE],
    input  wire logic [XLEN-1:0]               ptw_req_wdata [NACTIVE],
    output logic                               ptw_req_ack   [NACTIVE],
    output logic [XLEN-1:0]                    ptw_resp_rdata[NACTIVE],

    // ---- per-core fence.i flush handshake (self-completes; coherence makes WB unnecessary) ----
    input  wire logic                          dcache_flush_req [NACTIVE],
    output logic                               dcache_flush_done[NACTIVE],

    // ---- per-core cache-event pulses (mhpmcounter3-5) ----
    output logic                               hpm_l1i_miss[NACTIVE],
    output logic                               hpm_l1d_miss[NACTIVE],
    output logic                               hpm_l1d_wb  [NACTIVE],

    // ---- data-side backend: the directory's single NMI master ----
    output nmi_req_t   mem_req_o,
    input  wire logic  mem_req_ready_i,
    input  nmi_resp_t  mem_resp_i
);
    localparam int D_RESP_DELAY =
        (RISCV_UArch::DMEMORY_READ_DELAY < 1) ? 1 : RISCV_UArch::DMEMORY_READ_DELAY;

    // free-running cycle counter for device-bypass response due-times
    logic [63:0] cyc_q;
    always_ff @(posedge clk or negedge rst_l) begin
        if (!rst_l) cyc_q <= '0; else cyc_q <= cyc_q + 64'd1;
    end

    // op decode (matches load_store_queue.sv's 3-bit contract + niigo_memsys)
    function automatic l1_core_op_e map_dmem_op(input logic [2:0] c);
        unique case (c)
            3'd1:    map_dmem_op = COP_STORE;
            3'd2:    map_dmem_op = COP_LR;
            3'd3:    map_dmem_op = COP_AMO_RD;
            3'd4:    map_dmem_op = COP_SC;
            3'd5:    map_dmem_op = COP_AMO;
            default: map_dmem_op = COP_LOAD;
        endcase
    endfunction
    function automatic l1_amo_op_e map_amo(input logic [3:0] a);
        unique case (a)
            4'd3:    map_amo = AMO_SWAP;  4'd4: map_amo = AMO_ADD;  4'd5: map_amo = AMO_XOR;
            4'd6:    map_amo = AMO_AND;   4'd7: map_amo = AMO_OR;   4'd8: map_amo = AMO_MIN;
            4'd9:    map_amo = AMO_MAX;   4'd10:map_amo = AMO_MINU; 4'd11:map_amo = AMO_MAXU;
            default: map_amo = AMO_ADD;
        endcase
    endfunction

    // ---- CCD core-side request/probe/snoop arrays (length NACTIVE) ----
    logic                          c_req_valid [NACTIVE];
    logic                          c_req_ready [NACTIVE];
    l1_core_op_e                   c_req_op    [NACTIVE];
    l1_amo_op_e                    c_req_amo   [NACTIVE];
    logic [MEMORY_ADDR_WIDTH-1:0]  c_req_waddr [NACTIVE];
    logic [XLEN-1:0]               c_req_wdata [NACTIVE];
    logic [XLEN_BYTES-1:0]         c_req_wmask [NACTIVE];
    logic [XLEN-1:0]               c_resp_rdata[NACTIVE];
    logic                          c_resp_sc_ok[NACTIVE];
    logic                          ccd_sk_v    [NACTIVE];
    logic [MEMORY_ADDR_WIDTH-1:0]  ccd_sk_la   [NACTIVE];
    logic                          ccd_probe_v [NACTIVE];
    logic [MEMORY_ADDR_WIDTH-1:0]  ccd_probe_wa[NACTIVE];
    logic                          ccd_probe_hit [NACTIVE];
    logic [LINE_BITS-1:0]          ccd_probe_line[NACTIVE];

    genvar gi;
    generate for (gi=0; gi<NACTIVE; gi++) begin : ARM
        // ===== REAL L1 instruction cache =====
        nmi_req_t  l1i_req;  logic l1i_rdy;  nmi_resp_t l1i_resp;
        logic                          l1i_snp_v;  logic [MEMORY_ADDR_WIDTH-1:0] l1i_snp_wa;
        logic                          l1i_acc, l1i_mis;
        l1_icache L1I (
            .clk, .rst_l,
            .ifetch_req_valid(ifetch_req_valid[gi]), .ifetch_req_ready(ifetch_req_ready[gi]),
            .ifetch_req_addr(ifetch_req_addr[gi]),
            .ifetch_resp_valid(ifetch_resp_valid[gi]), .ifetch_resp_data(ifetch_resp_data[gi]),
            .ifetch_resp_excpt(ifetch_resp_excpt[gi]),
            .inval_all(ifetch_inval[gi]),
            .snoop_valid(l1i_snp_v), .snoop_waddr(l1i_snp_wa),
            .nmi_req(l1i_req), .nmi_req_ready(l1i_rdy), .nmi_resp(l1i_resp),
            .ev_access(l1i_acc), .ev_miss(l1i_mis));
        assign hpm_l1i_miss[gi] = l1i_mis;
        assign hpm_l1d_miss[gi] = 1'b0;   // grant-and-go agent does not export C3 miss/wb pulses
        assign hpm_l1d_wb[gi]   = 1'b0;

        // ===== device-bypass response generator (1 outstanding; LSQ serialises) =====
        logic                          dev_pend_q;
        logic [MEMORY_ADDR_WIDTH-1:0]  dev_addr_q;
        logic [63:0]                   dev_due_q;
        wire  dev_ready     = !dev_pend_q;
        wire  dev_load_fire = dmem_req_valid[gi] && dmem_req_device[gi] && dev_ready && !dmem_req_write[gi];
        wire  dev_resp_v    = dev_pend_q && (cyc_q >= dev_due_q);
        always_ff @(posedge clk or negedge rst_l) begin
            if (!rst_l) begin dev_pend_q<=1'b0; dev_addr_q<='0; dev_due_q<='0; end
            else begin
                if (dev_load_fire) begin dev_pend_q<=1'b1; dev_addr_q<=dmem_req_addr[gi]; dev_due_q<=cyc_q + 64'(D_RESP_DELAY); end
                else if (dev_resp_v) dev_pend_q<=1'b0;
            end
        end

        // ===== registered launch adapter (dmem + PTW) + iref (I-fetch COP_LOAD) on one c_req =====
        logic                          ad_busy_q, ad_is_ptw_q, ad_is_load_q, ad_is_sc_q, ad_is_amo_q;
        l1_core_op_e                   ad_op_q;  l1_amo_op_e ad_amo_q;
        logic [MEMORY_ADDR_WIDTH-1:0]  ad_addr_q;  logic [XLEN-1:0] ad_wdata_q;  logic [XLEN_BYTES-1:0] ad_wmask_q;
        logic                          ad_resp_pend_q;  logic [XLEN-1:0] ad_resp_data_q;  logic [MEMORY_ADDR_WIDTH-1:0] ad_resp_addr_q;
        logic                          iref_busy_q;  logic [MEMORY_ADDR_WIDTH-1:0] iref_addr_q;

        // P2 probe: drive this agent's probe with the L1I's pending refill line addr
        assign ccd_probe_v[gi]  = l1i_req.valid && (l1i_req.op == NMI_RD_LINE);
        assign ccd_probe_wa[gi] = l1i_req.waddr;

        wire present_dmem  = dmem_req_valid[gi] && !dmem_req_device[gi];
        wire present_ptw   = !present_dmem && ptw_req_valid[gi];
        wire ad_can_accept = !ad_busy_q && !ad_resp_pend_q;
        wire ad_launch_fire = (present_dmem || present_ptw) && ad_can_accept && !iref_busy_q;
        wire iref_need      = ccd_probe_v[gi] && !ccd_probe_hit[gi];
        wire iref_launch    = iref_need && !iref_busy_q && ad_can_accept && !ad_launch_fire;
        // loop-free, loss-free ready (AND of registered-derived terms); device-gated; hold off while an iref owns c_req
        assign dmem_req_ready[gi] = dev_ready && ad_can_accept && !iref_busy_q;

        // c_req mux: launch adapter (registered) OR iref COP_LOAD (mutually exclusive)
        assign c_req_valid[gi] = ad_busy_q || iref_busy_q;
        assign c_req_op[gi]    = iref_busy_q ? COP_LOAD : ad_op_q;
        assign c_req_amo[gi]   = ad_amo_q;
        assign c_req_waddr[gi] = iref_busy_q ? iref_addr_q : ad_addr_q;
        assign c_req_wdata[gi] = ad_wdata_q;
        assign c_req_wmask[gi] = iref_busy_q ? '0 : ad_wmask_q;
        wire ad_done   = ad_busy_q   && c_req_ready[gi];
        wire iref_done = iref_busy_q && c_req_ready[gi];

        always_ff @(posedge clk or negedge rst_l) begin
            if (!rst_l) begin ad_busy_q<=1'b0; ad_resp_pend_q<=1'b0; iref_busy_q<=1'b0; end
            else begin
                if (ad_launch_fire) begin
                    ad_busy_q    <= 1'b1;
                    ad_is_ptw_q  <= present_ptw;
                    ad_is_load_q <= present_dmem ? !dmem_req_write[gi] : !ptw_req_we[gi];
                    ad_is_sc_q   <= present_dmem && (dmem_req_op[gi]==3'd4);
                    ad_is_amo_q  <= present_dmem && (dmem_req_op[gi]==3'd5);
                    ad_amo_q     <= present_dmem ? map_amo(dmem_req_amo[gi]) : AMO_ADD;
                    ad_op_q      <= present_dmem ? map_dmem_op(dmem_req_op[gi])
                                                 : (ptw_req_we[gi] ? COP_STORE : COP_LOAD);
                    ad_addr_q    <= present_dmem ? dmem_req_addr[gi]  : ptw_req_addr[gi];
                    ad_wdata_q   <= present_dmem ? dmem_req_wdata[gi] : ptw_req_wdata[gi];
                    ad_wmask_q   <= present_dmem ? dmem_req_wmask[gi] : {XLEN_BYTES{1'b1}};
                end else if (ad_done) ad_busy_q <= 1'b0;
                if (ad_done && ((ad_is_load_q && !ad_is_ptw_q) || ad_is_sc_q || ad_is_amo_q)) begin
                    ad_resp_pend_q <= 1'b1;
                    ad_resp_data_q <= ad_is_sc_q ? (c_resp_sc_ok[gi] ? '0 : XLEN'(1)) : c_resp_rdata[gi];
                    ad_resp_addr_q <= ad_addr_q;
                end else if (ad_resp_pend_q) ad_resp_pend_q <= 1'b0;
                if (iref_launch) begin iref_busy_q<=1'b1; iref_addr_q<=l1i_req.waddr; end
                else if (iref_done) iref_busy_q<=1'b0;
            end
        end
        // dmem response: device dummy (core overrides data) OR the registered agent load/SC/AMO
        assign dmem_resp_valid[gi] = dev_resp_v || ad_resp_pend_q;
        assign dmem_resp_addr [gi] = dev_resp_v ? dev_addr_q : ad_resp_addr_q;
        assign dmem_resp_data [gi] = dev_resp_v ? '0         : ad_resp_data_q;
        // PTW ack regenerated from the registered completion (read or A/D write both ack once)
        assign ptw_req_ack   [gi] = ad_done && ad_is_ptw_q;
        assign ptw_resp_rdata[gi] = c_resp_rdata[gi];

        // ===== probe-serve: once the refill line is local, deliver it to the L1I (2-cycle) =====
        logic pserve_q; logic [LINE_BITS-1:0] pserve_line_q;
        wire l1i_probe_serve = ccd_probe_v[gi] && ccd_probe_hit[gi];
        assign l1i_rdy = l1i_probe_serve && !pserve_q;
        always_ff @(posedge clk or negedge rst_l) begin
            if (!rst_l)                                 pserve_q<=1'b0;
            else if (l1i_probe_serve && !pserve_q) begin pserve_q<=1'b1; pserve_line_q<=ccd_probe_line[gi]; end
            else if (pserve_q)                          pserve_q<=1'b0;
        end
        always_comb begin
            l1i_resp = '0;
            if (pserve_q) begin l1i_resp.valid=1'b1; l1i_resp.rdata=pserve_line_q; l1i_resp.err=1'b0; end
        end

        // ===== L1I snoop-invalidate: a local committed write OR a remote write-snoop =====
        wire ad_commit_write = ad_done && (ad_op_q==COP_STORE || ad_is_amo_q || ad_is_sc_q);
        assign l1i_snp_v  = ad_commit_write || ccd_sk_v[gi];
        assign l1i_snp_wa = ad_commit_write ? ad_addr_q : ccd_sk_la[gi];

        // ===== fence.i flush self-completes (coherent forwarding makes a WB unnecessary;
        //       the L1I flash-invalidate via ifetch_inval still re-pulls coherently) =====
        assign dcache_flush_done[gi] = dcache_flush_req[gi];

        // snoop-kill to the core LSQ
        assign dmem_snoop_kill_valid[gi] = ccd_sk_v[gi];
        assign dmem_snoop_kill_laddr[gi] = ccd_sk_la[gi];

        logic unused_arm; assign unused_arm = l1i_acc | (|dev_addr_q);
    end endgenerate

    // ===== the shared grant-and-go CCD subsystem (agents + directory + interconnect) =====
    niigo_ccd_gg_direct #(.NACTIVE(NACTIVE), .L1_SETS(L1_SETS), .DIR_SETS(DIR_SETS), .RESP_DLY(RESP_DLY)) CCD (
        .clk, .rst_l,
        .c_req_valid(c_req_valid), .c_req_ready(c_req_ready),
        .c_req_op(c_req_op), .c_req_amo(c_req_amo),
        .c_req_waddr(c_req_waddr), .c_req_wdata(c_req_wdata), .c_req_wmask(c_req_wmask),
        .c_resp_rdata(c_resp_rdata), .c_resp_sc_ok(c_resp_sc_ok),
        .flush_req(1'b0), .flush_done(),                 // per-core fence.i self-completes above
        .snoop_kill_valid(ccd_sk_v), .snoop_kill_laddr(ccd_sk_la),
        .probe_valid(ccd_probe_v), .probe_waddr(ccd_probe_wa),
        .probe_hit(ccd_probe_hit), .probe_line(ccd_probe_line),
        .mem_req_o(mem_req_o), .mem_req_ready_i(mem_req_ready_i), .mem_resp_i(mem_resp_i)
    );
endmodule: niigo_ccd_memsys
`default_nettype wire
