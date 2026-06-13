`include "ooo_types.vh"

`default_nettype none

module busy_table
    import OOO_Types::*;
(
    input wire logic                 clk,
    input wire logic                 rst_l,
    input wire logic [OOO_WIDTH-1:0] allocate_valid,
    input wire phys_reg_t            allocate_prd [OOO_WIDTH],
    input wire logic [OOO_WIDTH-1:0] writeback_valid,
    input wire phys_reg_t            writeback_prd [OOO_WIDTH],
    input wire phys_reg_t            src1_prd [OOO_WIDTH],
    input wire phys_reg_t            src2_prd [OOO_WIDTH],
    output logic [OOO_WIDTH-1:0] src1_ready,
    output logic [OOO_WIDTH-1:0] src2_ready
);

    logic [PHYS_REGS-1:0] busy_q, busy_next;

    always_comb begin
        busy_next = busy_q;
        for (int i = 0; i < OOO_WIDTH; i += 1) begin
            if (writeback_valid[i]) begin
                busy_next[writeback_prd[i]] = 1'b0;
            end
        end
        for (int i = 0; i < OOO_WIDTH; i += 1) begin
            if (allocate_valid[i] && (allocate_prd[i] != '0)) begin
                busy_next[allocate_prd[i]] = 1'b1;
            end
        end

        for (int i = 0; i < OOO_WIDTH; i += 1) begin
            src1_ready[i] = (src1_prd[i] == '0) || !busy_next[src1_prd[i]];
            src2_ready[i] = (src2_prd[i] == '0) || !busy_next[src2_prd[i]];
        end
    end

    always_ff @(posedge clk or negedge rst_l) begin
        if (!rst_l) begin
            busy_q <= '0;
        end else begin
            busy_q <= busy_next;
        end
    end

endmodule: busy_table
