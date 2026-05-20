`include "ooo_types.vh"

`default_nettype none

module ooo_commit_unit
    import OOO_Types::*;
(
    input  logic [OOO_WIDTH-1:0] commit_valid,
    input  commit_packet_t       commit_packet [OOO_WIDTH],
    input  logic                 store_port_busy,
    output logic [OOO_WIDTH-1:0] retire_valid,
    output logic [OOO_WIDTH-1:0] free_valid,
    output phys_reg_t            free_prd [OOO_WIDTH],
    output logic                 commit_store,
    output active_id_t           commit_store_id,
    output logic                 precise_halt,
    output logic                 precise_exception
);

    logic stop_prefix;

    always_comb begin
        retire_valid = '0;
        free_valid = '0;
        for (int i = 0; i < OOO_WIDTH; i += 1) begin
            free_prd[i] = '0;
        end
        commit_store = 1'b0;
        commit_store_id = '0;
        precise_halt = 1'b0;
        precise_exception = 1'b0;
        stop_prefix = 1'b0;

        for (int i = 0; i < OOO_WIDTH; i += 1) begin
            if (commit_valid[i] && !stop_prefix) begin
                if (commit_packet[i].is_store && store_port_busy) begin
                    stop_prefix = 1'b1;
                end else begin
                    retire_valid[i] = 1'b1;
                    free_valid[i] = commit_packet[i].has_dest;
                    free_prd[i] = commit_packet[i].old_prd;

                    if (commit_packet[i].is_store) begin
                        commit_store = 1'b1;
                        commit_store_id = commit_packet[i].active_id;
                    end
                    if (commit_packet[i].halted) begin
                        precise_halt = 1'b1;
                        stop_prefix = 1'b1;
                    end
                    if (commit_packet[i].exception) begin
                        precise_exception = 1'b1;
                        stop_prefix = 1'b1;
                    end
                end
            end
        end
    end

endmodule: ooo_commit_unit
