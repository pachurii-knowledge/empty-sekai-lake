`include "ooo_types.vh"

`default_nettype none

module ooo_writeback_bus
    import OOO_Types::*;
(
    input  writeback_packet_t alu0_writeback,
    input  writeback_packet_t alu1_writeback,
    input  writeback_packet_t load_writeback,
    input  branch_mask_t      abort_mask_q,
    output logic [OOO_WIDTH-1:0] writeback_valid,
    output active_id_t           writeback_active_id [OOO_WIDTH],
    output phys_reg_t            writeback_prd [OOO_WIDTH],
    output logic [OOO_WIDTH-1:0][31:0] writeback_data,
    output logic [OOO_WIDTH-1:0] writeback_has_dest,
    output logic [OOO_WIDTH-1:0] writeback_exception,
    output logic [OOO_WIDTH-1:0] writeback_halted,
    output writeback_packet_t    branch_writeback
);

    writeback_packet_t packets [OOO_WIDTH];

    always_comb begin
        packets[0] = alu0_writeback;
        packets[1] = alu1_writeback;
        packets[2] = load_writeback;
        packets[3] = '0;
        branch_writeback = '0;

        for (int i = 0; i < OOO_WIDTH; i += 1) begin
            writeback_valid[i] = packets[i].valid &&
                ((packets[i].branch_mask & abort_mask_q) == '0);
            writeback_active_id[i] = packets[i].active_id;
            writeback_prd[i] = packets[i].prd;
            writeback_data[i] = packets[i].data;
            writeback_has_dest[i] = packets[i].has_dest;
            writeback_exception[i] = packets[i].exception;
            writeback_halted[i] = packets[i].halted;
            if (writeback_valid[i] && packets[i].branch_valid) begin
                branch_writeback = packets[i];
            end
        end
    end

endmodule: ooo_writeback_bus
