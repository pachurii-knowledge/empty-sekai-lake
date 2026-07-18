`include "ooo_types.vh"

`default_nettype none

module ooo_writeback_bus
    import OOO_Types::*;
(
    input wire writeback_packet_t alu0_writeback,
    input wire writeback_packet_t alu1_writeback,
`ifdef ALU4
    input wire writeback_packet_t alu2_writeback,
`endif
    input wire writeback_packet_t load_writeback,
    input wire writeback_packet_t mul_writeback,
    input wire writeback_packet_t div_writeback,
    input wire writeback_packet_t fp_writeback,
`ifdef FUSE_LDBR
    input wire writeback_packet_t fbr_writeback,
`endif
    input wire branch_mask_t      abort_mask_q,
    output logic              mul_writeback_ready,
    output logic              div_writeback_ready,
    output logic              fp_writeback_ready,
`ifdef FUSE_LDBR
    output logic              fbr_writeback_ready,
`endif
    output logic [OOO_WIDTH-1:0] writeback_valid,
    output active_id_t           writeback_active_id [OOO_WIDTH],
    output phys_reg_t            writeback_prd [OOO_WIDTH],
    output logic [OOO_WIDTH-1:0][XLEN-1:0] writeback_data,
    output logic [OOO_WIDTH-1:0] writeback_has_dest,
    output logic [OOO_WIDTH-1:0] writeback_fp_write,
    output arch_reg_t            writeback_fp_rd [OOO_WIDTH],
    output fp_reg_data_t         writeback_fp_data [OOO_WIDTH],
    output logic [OOO_WIDTH-1:0] writeback_csr_write,
    output logic [OOO_WIDTH-1:0][11:0] writeback_csr_addr,
    output logic [OOO_WIDTH-1:0][XLEN-1:0] writeback_csr_wdata,
    output logic [OOO_WIDTH-1:0] writeback_fp_fflags_valid,
    output logic [OOO_WIDTH-1:0][4:0] writeback_fp_fflags,
    output logic [OOO_WIDTH-1:0] writeback_exception,
    output logic [OOO_WIDTH-1:0][4:0] writeback_exc_cause,
    output logic [OOO_WIDTH-1:0] writeback_halted,
    output writeback_packet_t    branch_writeback
);

    writeback_packet_t packets [WB_SOURCES];
    logic [WB_SOURCES-1:0] source_valid;
    logic [WB_SOURCES-1:0] source_accepted;

    // Source indices = arbitration priority (ALU0 highest). Gated so OFF is
    // byte-identical (WB_LOAD=2/MUL=3/DIV=4/FP=5). Under ALU4 the 3rd ALU takes
    // index 2 and load/mul/div/fp shift up; load stays above the backpressurable
    // mul/div/fp with only 3 ALUs over it, so load is never dropped (never needs a
    // ready port) at 3 ports.
`ifdef ALU4
    localparam int WB_ALU2 = 2, WB_LOAD = 3, WB_MUL = 4, WB_DIV = 5, WB_FP = 6;
`else
    localparam int WB_LOAD = 2, WB_MUL = 3, WB_DIV = 4, WB_FP = 5;
`endif
`ifdef FUSE_LDBR
    // FUSE_LDBR: the pend_fbr fused-branch-resolve source — WB-only (no issue
    // port), LOWEST arbitration priority (it is backpressurable, unlike the
    // load; the hold is bounded — the unretired slave branch caps the ROB,
    // which drains every competing source). The pre-existing index lines above
    // stay untouched so OFF is byte-identical.
    localparam int WB_FBR = WB_SOURCES - 1;
    // branch_stack has ONE resolve port, so the fused resolve may seat only
    // when no ALU pipe resolves a branch this cycle (spec blocker 3: the WB
    // seating and the branch_writeback drive must be the same cycle). ALU
    // writebacks are flops, so this is loop-free.
    logic any_alu_branch_resolving;
`endif

    always_comb begin
        packets[0] = alu0_writeback;
        packets[1] = alu1_writeback;
`ifdef ALU4
        packets[WB_ALU2] = alu2_writeback;
`endif
        packets[WB_LOAD] = load_writeback;
        packets[WB_MUL]  = mul_writeback;
        packets[WB_DIV]  = div_writeback;
        packets[WB_FP]   = fp_writeback;
`ifdef FUSE_LDBR
        packets[WB_FBR]  = fbr_writeback;
`endif
        source_valid = '0;
        source_accepted = '0;

        for (int i = 0; i < OOO_WIDTH; i += 1) begin
            writeback_valid[i] = 1'b0;
            writeback_active_id[i] = '0;
            writeback_prd[i] = '0;
            writeback_data[i] = '0;
            writeback_has_dest[i] = 1'b0;
            writeback_fp_write[i] = 1'b0;
            writeback_fp_rd[i] = '0;
            writeback_fp_data[i] = '0;
            writeback_csr_write[i] = 1'b0;
            writeback_csr_addr[i] = '0;
            writeback_csr_wdata[i] = '0;
            writeback_fp_fflags_valid[i] = 1'b0;
            writeback_fp_fflags[i] = '0;
            writeback_exception[i] = 1'b0;
            writeback_exc_cause[i] = 5'd0;
            writeback_halted[i] = 1'b0;
        end

        for (int source = 0; source < WB_SOURCES; source += 1) begin
            source_valid[source] = packets[source].valid &&
                ((packets[source].branch_mask & abort_mask_q) == '0);
        end
`ifdef FUSE_LDBR
        // Blocker 3 (atomic completion+resolve+drain): the fused resolve seats
        // only in a cycle the branch-resolve port is free, so its WB seating
        // and its branch_writeback drive below are always the same cycle.
        source_valid[WB_FBR] = source_valid[WB_FBR] && !any_alu_branch_resolving;
`endif

        for (int lane = 0; lane < OOO_WIDTH; lane += 1) begin
            for (int source = 0; source < WB_SOURCES; source += 1) begin
                if (!writeback_valid[lane] && source_valid[source] &&
                        !source_accepted[source]) begin
                    source_accepted[source] = 1'b1;
                    writeback_valid[lane] = 1'b1;
                    writeback_active_id[lane] = packets[source].active_id;
                    writeback_prd[lane] = packets[source].prd;
                    writeback_data[lane] = packets[source].data;
                    writeback_has_dest[lane] = packets[source].has_dest;
                    writeback_fp_write[lane] = packets[source].fp_write;
                    writeback_fp_rd[lane] = packets[source].fp_rd;
                    writeback_fp_data[lane] = packets[source].fp_data;
                    writeback_csr_write[lane] = packets[source].csr_write;
                    writeback_csr_addr[lane] = packets[source].csr_addr;
                    writeback_csr_wdata[lane] = packets[source].csr_wdata;
                    writeback_fp_fflags_valid[lane] =
                        packets[source].fp_fflags_valid;
                    writeback_fp_fflags[lane] = packets[source].fp_fflags;
                    writeback_exception[lane] = packets[source].exception;
                    writeback_exc_cause[lane] = packets[source].exc_cause;
                    writeback_halted[lane] = packets[source].halted;
                end
            end
        end

        mul_writeback_ready = !mul_writeback.valid || source_accepted[WB_MUL] ||
            ((mul_writeback.branch_mask & abort_mask_q) != '0);
        div_writeback_ready = !div_writeback.valid || source_accepted[WB_DIV] ||
            ((div_writeback.branch_mask & abort_mask_q) != '0);
        fp_writeback_ready = !fp_writeback.valid || source_accepted[WB_FP] ||
            ((fp_writeback.branch_mask & abort_mask_q) != '0);
`ifdef FUSE_LDBR
        // Seated (implies the branch port was free) or aborted: the core may
        // drop pend_fbr. Otherwise it holds and retries — never dropped.
        fbr_writeback_ready = !fbr_writeback.valid || source_accepted[WB_FBR] ||
            ((fbr_writeback.branch_mask & abort_mask_q) != '0);
`endif
    end

    // ---- branch_writeback (split out to break the false load_writeback ->
    // branch_writeback loop edge). Only ALU ops carry branch_valid, and ALU sources
    // 0/1 are top-priority in the arbitration above (the 4 lanes always seat them),
    // so an ALU branch packet is always accepted onto the bus when valid and passing
    // the abort_mask_q gate -- the same condition checked here. At most one
    // control-flow op issues per cycle (IQ branch constraint), so alu0/alu1 are
    // never both branches; the alu1 override mirrors the former in-loop last-write.
    // Reads ONLY the ALU writebacks, so branch_writeback no longer whole-block-
    // aliases load_writeback. Value-identical.
    always_comb begin
        branch_writeback = '0;
`ifdef FUSE_LDBR
        any_alu_branch_resolving =
            (alu0_writeback.valid && alu0_writeback.branch_valid &&
                ((alu0_writeback.branch_mask & abort_mask_q) == '0)) ||
            (alu1_writeback.valid && alu1_writeback.branch_valid &&
                ((alu1_writeback.branch_mask & abort_mask_q) == '0))
`ifdef ALU4
            ||
            (alu2_writeback.valid && alu2_writeback.branch_valid &&
                ((alu2_writeback.branch_mask & abort_mask_q) == '0))
`endif
            ;
`endif
        if (alu0_writeback.valid && alu0_writeback.branch_valid &&
                ((alu0_writeback.branch_mask & abort_mask_q) == '0)) begin
            branch_writeback = alu0_writeback;
        end
        if (alu1_writeback.valid && alu1_writeback.branch_valid &&
                ((alu1_writeback.branch_mask & abort_mask_q) == '0)) begin
            branch_writeback = alu1_writeback;
        end
`ifdef ALU4
        if (alu2_writeback.valid && alu2_writeback.branch_valid &&
                ((alu2_writeback.branch_mask & abort_mask_q) == '0)) begin
            branch_writeback = alu2_writeback;
        end
`endif
`ifdef FUSE_LDBR
        // Fused load->branch resolve (pend_fbr): drives the port ONLY in a
        // cycle it actually seats on the bus (source_accepted implies the port
        // is free via the source_valid gate above — completion+resolve+drain
        // atomic), so it can never collide with an ALU branch resolve on
        // branch_stack's single resolve port, and never double-resolves.
        if (fbr_writeback.valid && fbr_writeback.branch_valid &&
                ((fbr_writeback.branch_mask & abort_mask_q) == '0) &&
                source_accepted[WB_FBR]) begin
            branch_writeback = fbr_writeback;
        end
`endif
    end

endmodule: ooo_writeback_bus
