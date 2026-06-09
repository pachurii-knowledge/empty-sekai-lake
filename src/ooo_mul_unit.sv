`include "ooo_types.vh"

`default_nettype none

module ooo_mul_unit
    import OOO_Types::*;
(
    input  logic              clk,
    input  logic              rst_l,
    input  logic              issue_valid,
    output logic              issue_ready,
    input  issue_entry_t      issue_entry,
    input  logic [XLEN-1:0]   rs1_data,
    input  logic [XLEN-1:0]   rs2_data,
    input  branch_mask_t      abort_mask,
    // Precise-trap full flush: squash every in-flight stage so no stale
    // writeback lands after the pipeline is reset (the active-list id this op
    // targets may be reused by a younger instruction next cycle).
    input  logic              flush,
    input  logic              writeback_ready,
    output writeback_packet_t writeback
);

    localparam int MUL_LATENCY = 3;

    logic [MUL_LATENCY-1:0] valid_q;
    writeback_packet_t pipe_q [MUL_LATENCY];
    logic advance;
    logic output_aborted;

    assign output_aborted = valid_q[MUL_LATENCY-1] &&
        ((pipe_q[MUL_LATENCY-1].branch_mask & abort_mask) != '0);
    assign advance = writeback_ready || output_aborted ||
        !valid_q[MUL_LATENCY-1];
    assign issue_ready = advance;

    always_comb begin
        writeback = pipe_q[MUL_LATENCY-1];
        writeback.valid = valid_q[MUL_LATENCY-1] && !output_aborted && !flush;
    end

    always_ff @(posedge clk or negedge rst_l) begin
        if (!rst_l) begin
            valid_q <= '0;
            for (int i = 0; i < MUL_LATENCY; i += 1) begin
                pipe_q[i] <= '0;
            end
        end else if (flush) begin
            valid_q <= '0;
        end else if (advance) begin
            for (int i = MUL_LATENCY - 1; i > 0; i -= 1) begin
                valid_q[i] <= valid_q[i - 1] &&
                    ((pipe_q[i - 1].branch_mask & abort_mask) == '0);
                pipe_q[i] <= pipe_q[i - 1];
            end
            valid_q[0] <= issue_valid &&
                ((issue_entry.branch_mask & abort_mask) == '0);
            pipe_q[0] <= packet_for(issue_entry, rs1_data, rs2_data);
        end
    end

    function automatic writeback_packet_t packet_for(input issue_entry_t entry,
            input logic [XLEN-1:0] lhs, input logic [XLEN-1:0] rhs);
        writeback_packet_t packet;
        logic [2*XLEN-1:0] product;
        packet = '0;
        packet.valid = 1'b1;
        packet.active_id = entry.active_id;
        packet.pc = entry.pc;
        packet.instr = entry.instr;
        packet.prd = entry.prd;
        packet.has_dest = entry.has_dest;
        packet.branch_mask = entry.branch_mask;
        product = signed_product_for(entry.ctrl.alu_op, lhs, rhs);
        unique case (entry.ctrl.alu_op)
            ALU_MUL:  packet.data = product[XLEN-1:0];
            // MULW: the low 32 product bits depend only on the low 32 operand
            // bits, so the full-width product needs no operand truncation.
            ALU_MULW: packet.data = XLEN'($signed(product[31:0]));
            default:  packet.data = product[2*XLEN-1:XLEN];
        endcase
        return packet;
    endfunction

    function automatic logic [2*XLEN-1:0] signed_product_for(input alu_op_t op,
            input logic [XLEN-1:0] lhs, input logic [XLEN-1:0] rhs);
        logic lhs_signed;
        logic rhs_signed;
        logic product_negative;
        logic [XLEN-1:0] lhs_mag;
        logic [XLEN-1:0] rhs_mag;
        logic [2*XLEN-1:0] product_mag;
        lhs_signed = (op == ALU_MUL) || (op == ALU_MULH) ||
            (op == ALU_MULHSU) || (op == ALU_MULW);
        rhs_signed = (op == ALU_MUL) || (op == ALU_MULH) || (op == ALU_MULW);
        product_negative = (lhs_signed && lhs[XLEN-1]) ^
            (rhs_signed && rhs[XLEN-1]);
        lhs_mag = (lhs_signed && lhs[XLEN-1]) ? (~lhs + 1'b1) : lhs;
        rhs_mag = (rhs_signed && rhs[XLEN-1]) ? (~rhs + 1'b1) : rhs;
        product_mag = wallace_product(lhs_mag, rhs_mag);
        signed_product_for = product_negative ? (~product_mag + 1'b1) :
            product_mag;
    endfunction

    function automatic logic [2*XLEN-1:0] wallace_product(
            input logic [XLEN-1:0] lhs, input logic [XLEN-1:0] rhs);
        logic [2*XLEN-1:0] rows [XLEN];
        logic [2*XLEN-1:0] sum_row;
        logic [2*XLEN-1:0] carry_row;
        int row_count;
        int out_idx;
        for (int i = 0; i < XLEN; i += 1) begin
            rows[i] = rhs[i] ? ({{XLEN{1'b0}}, lhs} << i) : '0;
        end
        row_count = XLEN;
        while (row_count > 2) begin
            out_idx = 0;
            for (int i = 0; i < row_count; i += 3) begin
                if ((i + 2) < row_count) begin
                    sum_row = rows[i] ^ rows[i + 1] ^ rows[i + 2];
                    carry_row = ((rows[i] & rows[i + 1]) |
                        (rows[i] & rows[i + 2]) |
                        (rows[i + 1] & rows[i + 2])) << 1;
                    rows[out_idx] = sum_row;
                    rows[out_idx + 1] = carry_row;
                    out_idx += 2;
                end else begin
                    rows[out_idx] = rows[i];
                    out_idx += 1;
                    if ((i + 1) < row_count) begin
                        rows[out_idx] = rows[i + 1];
                        out_idx += 1;
                    end
                end
            end
            for (int i = out_idx; i < XLEN; i += 1) begin
                rows[i] = '0;
            end
            row_count = out_idx;
        end
        wallace_product = rows[0] + rows[1];
    endfunction

endmodule: ooo_mul_unit
