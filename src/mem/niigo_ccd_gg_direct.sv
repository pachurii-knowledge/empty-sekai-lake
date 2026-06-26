/**
 * niigo_ccd_gg_direct.sv  --  M3c grant-and-go protocol on a behavioural DIRECT interconnect
 *
 * Validates the grant-and-go PROTOCOL (niigo_dir_gg + niigo_l1d_gg) decoupled from the wheel
 * fabric: a combinational message router wires the clean channels (the clocked dir/agent FSMs
 * supply all the real latency). Same external port list as niigo_ccd_top, so the S1-S6 coherence
 * program runs unchanged. When green, the SAME dir/agent drop onto the wheel hub-funnel (M3c-C).
 *
 * Routing:
 *   agent[c].dmd  (to HUB)  --by op-->  dir.req / dir.unblk / dir.wb     (arb across cores)
 *   dir.out (msg+dst)       --by op-->  agent[dst].snoop / .resp / .ack
 *   agent[k].snp (msg+dst)  --by op-->  agent[dst].resp (DATA fwd) / .ack (INV_ACK)
 * Per-destination input arbitration (dir has priority on resp/ack; peers round-robined by index).
 */
`include "niigo_mem.vh"
`include "niigo_cmi.vh"
`include "niigo_ccd_m1.vh"
`default_nettype none
module niigo_ccd_gg_direct
    import RISCV_ISA::XLEN;
    import RISCV_UArch::MEMORY_ADDR_WIDTH;
    import NIIGO_Mem::*;
    import NIIGO_CMI::*;
    import NIIGO_CCD_M1::*;
#(
    parameter int NACTIVE  = 2,
    parameter int DIR_SETS = 16,
    parameter int L1_SETS  = 16,
    // COH_FORCE: extra cycles to hold an L2-sourced (directory) DATA grant before delivering it,
    // so a peer's snoop can land on a requester still mid-acquire (IS_D/IM_*) -> exercises the
    // deferred-snoop matrix deterministically. 0 = a single registered delay (still correct).
    parameter int RESP_DLY = 0
)(
    input  wire logic clk,
    input  wire logic rst_l,
    input  wire logic                          c_req_valid [NACTIVE],
    output logic                               c_req_ready [NACTIVE],
    input  wire l1_core_op_e                   c_req_op    [NACTIVE],
    input  wire l1_amo_op_e                    c_req_amo   [NACTIVE],
    input  wire logic [MEMORY_ADDR_WIDTH-1:0]  c_req_waddr [NACTIVE],
    input  wire logic [XLEN-1:0]               c_req_wdata [NACTIVE],
    input  wire logic [XLEN/8-1:0]             c_req_wmask [NACTIVE],
    output logic [XLEN-1:0]                     c_resp_rdata[NACTIVE],
    output logic                               c_resp_sc_ok[NACTIVE],
    output nmi_req_t   mem_req_o,
    input  wire logic  mem_req_ready_i,
    input  nmi_resp_t  mem_resp_i
);
    localparam int CORES = NUM_CORES;
    /* verilator lint_off ENUMVALUE */   // '{default:'0} on ccd_msg_t (enum fields, 0 is valid)

    // ---- per-agent channels ----
    logic       dmd_v [NACTIVE]; ccd_msg_t dmd_m [NACTIVE]; logic dmd_r [NACTIVE];
    logic       snp_v [NACTIVE]; ccd_msg_t snp_m [NACTIVE]; logic [NODE_ID_W-1:0] snp_d [NACTIVE]; logic snp_r [NACTIVE];
    logic       snoop_v[NACTIVE]; ccd_msg_t snoop_m[NACTIVE]; logic snoop_r[NACTIVE];
    logic       resp_v [NACTIVE]; ccd_msg_t resp_m [NACTIVE]; logic resp_r [NACTIVE];
    logic       ack_v  [NACTIVE]; ccd_msg_t ack_m  [NACTIVE]; logic ack_r  [NACTIVE];

    // ---- dir channels ----
    logic       dreq_v;  ccd_msg_t dreq_m;  logic dreq_r;
    logic       dunb_v;  ccd_msg_t dunb_m;  logic dunb_r;
    logic       dwb_v;   ccd_msg_t dwb_m;   logic dwb_r;
    logic       dout_v;  ccd_msg_t dout_m;  logic [NODE_ID_W-1:0] dout_d; logic dout_r;

    niigo_dir_gg #(.CORES(CORES), .DIR_SETS(DIR_SETS)) DIR (
        .clk, .rst_l,
        .req_valid(dreq_v), .req_msg(dreq_m), .req_ready(dreq_r),
        .unblk_valid(dunb_v), .unblk_msg(dunb_m), .unblk_ready(dunb_r),
        .wb_valid(dwb_v), .wb_msg(dwb_m), .wb_ready(dwb_r),
        .out_valid(dout_v), .out_msg(dout_m), .out_dst(dout_d), .out_ready(dout_r),
        .mem_req_o(mem_req_o), .mem_req_ready_i(mem_req_ready_i), .mem_resp_i(mem_resp_i)
    );

    genvar gi;
    generate for (gi=0; gi<NACTIVE; gi++) begin : G_AGENT
        niigo_l1d_gg #(.CORE_ID(gi), .SETS(L1_SETS)) L1D (
            .clk, .rst_l,
            .c_req_valid(c_req_valid[gi]), .c_req_ready(c_req_ready[gi]),
            .c_req_op(c_req_op[gi]), .c_req_amo(c_req_amo[gi]),
            .c_req_waddr(c_req_waddr[gi]), .c_req_wdata(c_req_wdata[gi]), .c_req_wmask(c_req_wmask[gi]),
            .c_resp_rdata(c_resp_rdata[gi]), .c_resp_sc_ok(c_resp_sc_ok[gi]),
            .dmd_valid(dmd_v[gi]), .dmd_msg(dmd_m[gi]), .dmd_ready(dmd_r[gi]),
            .snp_valid(snp_v[gi]), .snp_msg(snp_m[gi]), .snp_dst(snp_d[gi]), .snp_ready(snp_r[gi]),
            .snoop_valid(snoop_v[gi]), .snoop_msg(snoop_m[gi]), .snoop_ready(snoop_r[gi]),
            .resp_valid(resp_v[gi]), .resp_msg(resp_m[gi]), .resp_ready(resp_r[gi]),
            .ack_valid(ack_v[gi]), .ack_msg(ack_m[gi]), .ack_ready(ack_r[gi])
        );
    end endgenerate

    // ---- COH_FORCE L2-DATA hold registers (per core) ----
    logic                 dd_busy [NACTIVE];
    logic [7:0]           dd_cnt  [NACTIVE];
    ccd_msg_t             dd_msg  [NACTIVE];
    always_ff @(posedge clk or negedge rst_l) begin
        if (!rst_l) for (int z=0;z<NACTIVE;z++) dd_busy[z]<=1'b0;
        else for (int z=0;z<NACTIVE;z++) begin
            if (dd_busy[z]) begin
                if (dd_cnt[z]!=0)      dd_cnt[z]<=dd_cnt[z]-8'd1;   // hold
                else if (resp_r[z])    dd_busy[z]<=1'b0;            // agent took the delayed DATA
            end else if (dout_v && (dout_m.op==OP_DATA) && (dout_d==z[NODE_ID_W-1:0])) begin
                dd_busy[z]<=1'b1; dd_cnt[z]<=8'(RESP_DLY); dd_msg[z]<=dout_m;
            end
        end
    end

    // helper: op classes
    function automatic logic is_req_op(input cmi_op_e op);
        is_req_op = (op==OP_GETS)||(op==OP_GETM)||(op==OP_UPGRADE)||
                    (op==OP_PUTM)||(op==OP_PUTO)||(op==OP_PUTS)||(op==OP_PUTE);
    endfunction
    function automatic logic is_snoop_op(input cmi_op_e op);
        is_snoop_op = (op==OP_FWD_GETS)||(op==OP_FWD_GETM)||(op==OP_INV)||(op==OP_DOWNGRADE);
    endfunction

    // ====================================================================
    // Combinational message routing
    // ====================================================================
    integer c, k;
    logic [NODE_ID_W-1:0] selreq, selunb, selwb;
    logic                 hasreq, hasunb, haswb;

    always_comb begin
        // ---- agent dmd -> dir.req/unblk/wb (lowest-core priority) ----
        hasreq=0; selreq='0; hasunb=0; selunb='0; haswb=0; selwb='0;
        for (c=0;c<NACTIVE;c++) begin
            if (dmd_v[c] && is_req_op(dmd_m[c].op)        && !hasreq) begin hasreq=1; selreq=c[NODE_ID_W-1:0]; end
            if (dmd_v[c] && dmd_m[c].op==OP_UNBLOCK       && !hasunb) begin hasunb=1; selunb=c[NODE_ID_W-1:0]; end
            if (dmd_v[c] && dmd_m[c].op==OP_WB_DATA       && !haswb)  begin haswb=1;  selwb=c[NODE_ID_W-1:0];  end
        end
        dreq_v = hasreq; dreq_m = dmd_m[selreq];
        dunb_v = hasunb; dunb_m = dmd_m[selunb];
        dwb_v  = haswb;  dwb_m  = dmd_m[selwb];

        // dmd_ready per agent
        for (c=0;c<NACTIVE;c++) begin
            dmd_r[c] = 1'b0;
            if (dmd_v[c]) begin
                if (is_req_op(dmd_m[c].op))      dmd_r[c] = (c[NODE_ID_W-1:0]==selreq) && hasreq && dreq_r;
                else if (dmd_m[c].op==OP_UNBLOCK)dmd_r[c] = (c[NODE_ID_W-1:0]==selunb) && hasunb && dunb_r;
                else if (dmd_m[c].op==OP_WB_DATA)dmd_r[c] = (c[NODE_ID_W-1:0]==selwb)  && haswb  && dwb_r;
            end
        end

        // ---- destination input channels ----
        for (c=0;c<NACTIVE;c++) begin
            // snoop: only the dir sends snoops
            snoop_v[c] = dout_v && is_snoop_op(dout_m.op) && (dout_d==c[NODE_ID_W-1:0]);
            snoop_m[c] = dout_m;

            // resp: a delayed dir DATA (priority) else a peer DATA-forward (combinational)
            resp_v[c] = 1'b0; resp_m[c] = '{default:'0};
            if (dd_busy[c] && dd_cnt[c]==0) begin
                resp_v[c]=1'b1; resp_m[c]=dd_msg[c];   // L2-sourced DATA, delivered after the hold
            end else begin
                for (k=0;k<NACTIVE;k++) if (!resp_v[c] && k!=c &&
                    snp_v[k] && (snp_m[k].op==OP_DATA) && (snp_d[k]==c[NODE_ID_W-1:0])) begin
                    resp_v[c]=1'b1; resp_m[c]=snp_m[k];
                end
            end

            // ack: dir ACK (priority) else a peer INV_ACK
            ack_v[c]=1'b0; ack_m[c]='{default:'0};
            if (dout_v && (dout_m.op==OP_ACK) && (dout_d==c[NODE_ID_W-1:0])) begin
                ack_v[c]=1'b1; ack_m[c]=dout_m;
            end else begin
                for (k=0;k<NACTIVE;k++) if (!ack_v[c] && k!=c &&
                    snp_v[k] && (snp_m[k].op==OP_INV_ACK) && (snp_d[k]==c[NODE_ID_W-1:0])) begin
                    ack_v[c]=1'b1; ack_m[c]=snp_m[k];
                end
            end
        end

        // ---- dir.out_ready (route to the addressed agent's channel) ----
        dout_r = 1'b0;
        if (dout_v) begin
            for (c=0;c<NACTIVE;c++) if (dout_d==c[NODE_ID_W-1:0]) begin
                if (is_snoop_op(dout_m.op)) dout_r = snoop_r[c];
                else if (dout_m.op==OP_DATA) dout_r = !dd_busy[c];   // accepted into the delay reg
                else if (dout_m.op==OP_ACK)  dout_r = ack_r[c];
            end
        end

        // ---- agent snp_ready (its DATA/INV_ACK consumed at the destination) ----
        for (k=0;k<NACTIVE;k++) begin
            snp_r[k] = 1'b0;
            if (snp_v[k]) begin
                for (c=0;c<NACTIVE;c++) if (c!=k && snp_d[k]==c[NODE_ID_W-1:0]) begin
                    if (snp_m[k].op==OP_DATA)
                        // selected only if no dir-DATA to c and this k is the chosen peer
                        snp_r[k] = resp_r[c] && resp_v[c] && (resp_m[c].op==OP_DATA) &&
                                   !(dout_v && dout_m.op==OP_DATA && dout_d==c[NODE_ID_W-1:0]) &&
                                   (resp_m[c].src==snp_m[k].src);
                    else if (snp_m[k].op==OP_INV_ACK)
                        snp_r[k] = ack_r[c] && ack_v[c] && (ack_m[c].op==OP_INV_ACK) &&
                                   !(dout_v && dout_m.op==OP_ACK && dout_d==c[NODE_ID_W-1:0]) &&
                                   (ack_m[c].src==snp_m[k].src);
                end
            end
        end
    end
    /* verilator lint_on ENUMVALUE */
endmodule
`default_nettype wire
