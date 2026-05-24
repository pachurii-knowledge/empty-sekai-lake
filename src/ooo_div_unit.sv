`include "ooo_types.vh"

`default_nettype none

module ooo_div_unit
    import OOO_Types::*;
(
    input  logic              clk,
    input  logic              rst_l,
    input  logic              issue_valid,
    output logic              issue_ready,
    input  issue_entry_t      issue_entry,
    input  logic [31:0]       rs1_data,
    input  logic [31:0]       rs2_data,
    input  branch_mask_t      abort_mask,
    input  logic              writeback_ready,
    output writeback_packet_t writeback
);

    typedef enum logic [1:0] {
        DIV_IDLE,
        DIV_RUN,
        DIV_DONE
    } div_state_t;

    div_state_t state_q, state_next;
    writeback_packet_t packet_q, packet_next;
    logic [31:0] dividend_q, dividend_next;
    logic [31:0] divisor_q, divisor_next;
    logic [31:0] quotient_q, quotient_next;
    logic [35:0] remainder_q, remainder_next;
    logic [2:0] iter_q, iter_next;
    logic quotient_negative_q, quotient_negative_next;
    logic remainder_negative_q, remainder_negative_next;
    logic special_q, special_next;
    logic [31:0] special_result_q, special_result_next;
    logic [3:0] qdigit;
    logic [35:0] trial_remainder;
    logic aborted;

    assign aborted = packet_q.valid && ((packet_q.branch_mask & abort_mask) != '0);
    assign issue_ready = state_q == DIV_IDLE;

    always_comb begin
        writeback = packet_q;
        writeback.valid = (state_q == DIV_DONE) && packet_q.valid && !aborted;
    end

    always_comb begin
        state_next = state_q;
        packet_next = packet_q;
        dividend_next = dividend_q;
        divisor_next = divisor_q;
        quotient_next = quotient_q;
        remainder_next = remainder_q;
        iter_next = iter_q;
        quotient_negative_next = quotient_negative_q;
        remainder_negative_next = remainder_negative_q;
        special_next = special_q;
        special_result_next = special_result_q;
        qdigit = 4'b0;
        trial_remainder = 36'b0;

        if (aborted) begin
            state_next = DIV_IDLE;
            packet_next = '0;
        end else begin
            unique case (state_q)
                DIV_IDLE: begin
                    if (issue_valid &&
                            ((issue_entry.branch_mask & abort_mask) == '0)) begin
                        packet_next = base_packet_for(issue_entry);
                        dividend_next = abs_operand(rs1_data, signed_lhs(issue_entry.ctrl.alu_op));
                        divisor_next = abs_operand(rs2_data, signed_rhs(issue_entry.ctrl.alu_op));
                        quotient_next = 32'b0;
                        remainder_next = 36'b0;
                        iter_next = 3'b0;
                        quotient_negative_next =
                            signed_lhs(issue_entry.ctrl.alu_op) &&
                            signed_rhs(issue_entry.ctrl.alu_op) &&
                            (rs1_data[31] ^ rs2_data[31]);
                        remainder_negative_next =
                            signed_lhs(issue_entry.ctrl.alu_op) && rs1_data[31];
                        special_next = special_case(issue_entry.ctrl.alu_op,
                            rs1_data, rs2_data, special_result_next);
                        state_next = DIV_RUN;
                    end
                end
                DIV_RUN: begin
                    if (special_q) begin
                        packet_next.data = special_result_q;
                        state_next = DIV_DONE;
                    end else begin
                        trial_remainder = {remainder_q[31:0], dividend_q[31:28]};
                        qdigit = select_qdigit(trial_remainder, divisor_q);
                        remainder_next = trial_remainder -
                            (36'(divisor_q) * 36'(qdigit));
                        quotient_next = {quotient_q[27:0], qdigit};
                        dividend_next = {dividend_q[27:0], 4'b0};
                        iter_next = iter_q + 3'd1;
                        if (iter_q == 3'd7) begin
                            packet_next.data = final_result(packet_q.instr,
                                {quotient_q[27:0], qdigit},
                                trial_remainder - (36'(divisor_q) * 36'(qdigit)),
                                quotient_negative_q, remainder_negative_q);
                            state_next = DIV_DONE;
                        end
                    end
                end
                DIV_DONE: begin
                    if (writeback_ready) begin
                        packet_next = '0;
                        state_next = DIV_IDLE;
                    end
                end
                default: begin
                    state_next = DIV_IDLE;
                    packet_next = '0;
                end
            endcase
        end
    end

    always_ff @(posedge clk or negedge rst_l) begin
        if (!rst_l) begin
            state_q <= DIV_IDLE;
            packet_q <= '0;
            dividend_q <= '0;
            divisor_q <= '0;
            quotient_q <= '0;
            remainder_q <= '0;
            iter_q <= '0;
            quotient_negative_q <= 1'b0;
            remainder_negative_q <= 1'b0;
            special_q <= 1'b0;
            special_result_q <= '0;
        end else begin
            state_q <= state_next;
            packet_q <= packet_next;
            dividend_q <= dividend_next;
            divisor_q <= divisor_next;
            quotient_q <= quotient_next;
            remainder_q <= remainder_next;
            iter_q <= iter_next;
            quotient_negative_q <= quotient_negative_next;
            remainder_negative_q <= remainder_negative_next;
            special_q <= special_next;
            special_result_q <= special_result_next;
        end
    end

    function automatic writeback_packet_t base_packet_for(input issue_entry_t entry);
        writeback_packet_t packet;
        packet = '0;
        packet.valid = 1'b1;
        packet.active_id = entry.active_id;
        packet.pc = entry.pc;
        packet.instr = entry.instr;
        packet.prd = entry.prd;
        packet.has_dest = entry.has_dest;
        packet.branch_mask = entry.branch_mask;
        return packet;
    endfunction

    function automatic logic signed_lhs(input alu_op_t op);
        signed_lhs = (op == ALU_DIV) || (op == ALU_REM);
    endfunction

    function automatic logic signed_rhs(input alu_op_t op);
        signed_rhs = (op == ALU_DIV) || (op == ALU_REM);
    endfunction

    function automatic logic [31:0] abs_operand(input logic [31:0] value,
            input logic is_signed);
        abs_operand = (is_signed && value[31]) ? (~value + 32'd1) : value;
    endfunction

    function automatic logic special_case(input alu_op_t op,
            input logic [31:0] lhs, input logic [31:0] rhs,
            output logic [31:0] result);
        special_case = 1'b0;
        result = 32'b0;
        if (rhs == 32'b0) begin
            special_case = 1'b1;
            if ((op == ALU_REM) || (op == ALU_REMU)) begin
                result = lhs;
            end else begin
                result = 32'hffff_ffff;
            end
        end else if (((op == ALU_DIV) || (op == ALU_REM)) &&
                (lhs == 32'h8000_0000) && (rhs == 32'hffff_ffff)) begin
            special_case = 1'b1;
            result = (op == ALU_DIV) ? 32'h8000_0000 : 32'b0;
        end
    endfunction

    function automatic logic [3:0] select_qdigit(input logic [35:0] remainder,
            input logic [31:0] divisor);
        logic [35:0] divisor_ext;
        select_qdigit = 4'd0;
        divisor_ext = 36'(divisor);
        for (int i = 0; i < 16; i += 1) begin
            if ((divisor_ext * 36'(i)) <= remainder) begin
                select_qdigit = 4'(i);
            end
        end
    endfunction

    function automatic logic [31:0] final_result(input logic [31:0] instr,
            input logic [31:0] quotient, input logic [35:0] remainder,
            input logic quotient_negative, input logic remainder_negative);
        alu_op_t op;
        logic [31:0] signed_quotient;
        logic [31:0] signed_remainder;
        op = alu_op_t'(instr_to_alu_op(instr));
        signed_quotient = quotient_negative ? (~quotient + 32'd1) : quotient;
        signed_remainder = remainder_negative ? (~remainder[31:0] + 32'd1) :
            remainder[31:0];
        unique case (op)
            ALU_DIV, ALU_DIVU: final_result = signed_quotient;
            default: final_result = signed_remainder;
        endcase
    endfunction

    function automatic alu_op_t instr_to_alu_op(input logic [31:0] instr);
        unique case (instr[14:12])
            3'b100: instr_to_alu_op = ALU_DIV;
            3'b101: instr_to_alu_op = ALU_DIVU;
            3'b110: instr_to_alu_op = ALU_REM;
            default: instr_to_alu_op = ALU_REMU;
        endcase
    endfunction

endmodule: ooo_div_unit
