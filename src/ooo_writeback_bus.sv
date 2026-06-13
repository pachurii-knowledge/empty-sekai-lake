`include "ooo_types.vh"

`default_nettype none

module ooo_writeback_bus
    import OOO_Types::*;
(
    input wire writeback_packet_t alu0_writeback,
    input wire writeback_packet_t alu1_writeback,
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

    always_comb begin
        packets[0] = alu0_writeback;
        packets[1] = alu1_writeback;
        packets[2] = load_writeback;
        packets[3] = mul_writeback;
        packets[4] = div_writeback;
        packets[5] = fp_writeback;
        branch_writeback = '0;
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
                    if (packets[source].branch_valid) begin
                        branch_writeback = packets[source];
                    end
                end
            end
        end

        mul_writeback_ready = !mul_writeback.valid || source_accepted[3] ||
            ((mul_writeback.branch_mask & abort_mask_q) != '0);
        div_writeback_ready = !div_writeback.valid || source_accepted[4] ||
            ((div_writeback.branch_mask & abort_mask_q) != '0);
        fp_writeback_ready = !fp_writeback.valid || source_accepted[5] ||
            ((fp_writeback.branch_mask & abort_mask_q) != '0);
    end

endmodule: ooo_writeback_bus
