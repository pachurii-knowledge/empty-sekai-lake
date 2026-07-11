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
    input wire branch_mask_t      abort_mask_q,
    output logic              mul_writeback_ready,
    output logic              div_writeback_ready,
    output logic              fp_writeback_ready,
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
    end

endmodule: ooo_writeback_bus
