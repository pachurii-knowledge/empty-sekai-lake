`include "ooo_types.vh"

`default_nettype none

module ooo_dispatch_control
    import OOO_Types::*;
(
    input  logic [OOO_WIDTH-1:0] lane_valid,
    input  logic [OOO_WIDTH-1:0] lane_has_dest,
    input  logic [OOO_WIDTH-1:0] lane_is_branch,
    input  logic [OOO_WIDTH-1:0] lane_is_memory,
    input  logic [OOO_WIDTH-1:0] lane_is_terminal,
    input  logic                 active_list_full,
    input  logic                 int_iq_full,
    input  logic                 mem_queue_full,
    input  logic                 branch_stack_full,
    input  logic                 free_list_can_allocate,
    input  logic [$clog2(PHYS_REGS+1)-1:0] free_list_available,
    input  logic                 suppress_dispatch,
    output logic [OOO_WIDTH-1:0] dispatch_valid,
    output logic                 dispatch_stall
);

    logic stop_prefix;
    logic branch_seen;
    logic memory_seen;
    logic prefix_dispatched;
    logic [$clog2(OOO_WIDTH+1)-1:0] dest_seen;

    always_comb begin
        dispatch_valid = '0;
        stop_prefix = 1'b0;
        branch_seen = 1'b0;
        memory_seen = 1'b0;
        prefix_dispatched = 1'b0;
        dest_seen = '0;
        dispatch_stall = suppress_dispatch || active_list_full || int_iq_full ||
            mem_queue_full;

        for (int i = 0; i < OOO_WIDTH; i += 1) begin
            if (lane_is_branch[i] && branch_seen) begin
                stop_prefix = 1'b1;
            end
            if (lane_is_memory[i] && memory_seen) begin
                stop_prefix = 1'b1;
            end
            if (lane_valid[i] && lane_has_dest[i] &&
                    (dest_seen >= free_list_available)) begin
                stop_prefix = 1'b1;
            end
            if (i != 0 && lane_is_terminal[i - 1]) begin
                stop_prefix = 1'b1;
            end
            if (lane_is_branch[i] && branch_stack_full) begin
                stop_prefix = 1'b1;
                if (!prefix_dispatched) begin
                    dispatch_stall = 1'b1;
                end
            end

            dispatch_valid[i] = lane_valid[i] && !dispatch_stall && !stop_prefix;
            prefix_dispatched |= dispatch_valid[i];

            if (dispatch_valid[i] && lane_is_branch[i]) begin
                branch_seen = 1'b1;
                stop_prefix = 1'b1;
            end
            if (dispatch_valid[i] && lane_is_memory[i]) begin
                memory_seen = 1'b1;
            end
            if (dispatch_valid[i] && lane_has_dest[i]) begin
                dest_seen += 1'b1;
            end
        end
    end

endmodule: ooo_dispatch_control
