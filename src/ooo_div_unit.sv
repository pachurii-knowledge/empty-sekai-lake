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
    input  logic [XLEN-1:0]   rs1_data,
    input  logic [XLEN-1:0]   rs2_data,
    input  branch_mask_t      abort_mask,
    // Precise-trap full flush: abandon any in-flight division so no stale
    // writeback lands on a reused active-list id after the pipeline is reset.
    input  logic              flush,
    input  logic              writeback_ready,
    output writeback_packet_t writeback
);

    // Radix-16 restoring division: 4 quotient bits per cycle.
    localparam int DIV_ITERS = XLEN / 4;
    localparam int ITER_W = $clog2(DIV_ITERS);
    localparam int REM_W = XLEN + 4;

    typedef enum logic [1:0] {
        DIV_IDLE,
        DIV_RUN,
        DIV_DONE
    } div_state_t;

    div_state_t state_q, state_next;
    writeback_packet_t packet_q, packet_next;
    alu_op_t op_q, op_next;
    logic [XLEN-1:0] dividend_q, dividend_next;
    logic [XLEN-1:0] divisor_q, divisor_next;
    logic [XLEN-1:0] quotient_q, quotient_next;
    logic [REM_W-1:0] remainder_q, remainder_next;
    logic [ITER_W-1:0] iter_q, iter_next;
    logic quotient_negative_q, quotient_negative_next;
    logic remainder_negative_q, remainder_negative_next;
    logic special_q, special_next;
    logic [XLEN-1:0] special_result_q, special_result_next;
    logic [3:0] qdigit;
    logic [REM_W-1:0] trial_remainder;
    logic aborted;
    logic [XLEN-1:0] eff_lhs, eff_rhs;

    assign aborted = packet_q.valid && ((packet_q.branch_mask & abort_mask) != '0);
    assign issue_ready = state_q == DIV_IDLE;

    always_comb begin
        writeback = packet_q;
        writeback.valid = (state_q == DIV_DONE) && packet_q.valid && !aborted &&
            !flush;
    end

    // W-form ops divide the low 32 bits of the operands; sign-/zero-extend them
    // to XLEN here so the full-width divider computes the correct 32-bit result
    // (the INT32_MIN/-1 overflow then resolves naturally: 2^31 is positive at
    // 64 bits, and the final sext32 of the quotient restores INT32_MIN).
    always_comb begin
        eff_lhs = rs1_data;
        eff_rhs = rs2_data;
        if (is_word_div(issue_entry.ctrl.alu_op)) begin
            if (unsigned_div(issue_entry.ctrl.alu_op)) begin
                eff_lhs = XLEN'(rs1_data[31:0]);
                eff_rhs = XLEN'(rs2_data[31:0]);
            end else begin
                eff_lhs = XLEN'($signed(rs1_data[31:0]));
                eff_rhs = XLEN'($signed(rs2_data[31:0]));
            end
        end
    end

    always_comb begin
        state_next = state_q;
        packet_next = packet_q;
        op_next = op_q;
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
        trial_remainder = '0;

        if (flush || aborted) begin
            state_next = DIV_IDLE;
            packet_next = '0;
        end else begin
            unique case (state_q)
                DIV_IDLE: begin
                    if (issue_valid &&
                            ((issue_entry.branch_mask & abort_mask) == '0)) begin
                        packet_next = base_packet_for(issue_entry);
                        op_next = issue_entry.ctrl.alu_op;
                        dividend_next = abs_operand(eff_lhs,
                            signed_div(issue_entry.ctrl.alu_op));
                        divisor_next = abs_operand(eff_rhs,
                            signed_div(issue_entry.ctrl.alu_op));
                        quotient_next = '0;
                        remainder_next = '0;
                        iter_next = '0;
                        quotient_negative_next =
                            signed_div(issue_entry.ctrl.alu_op) &&
                            (eff_lhs[XLEN-1] ^ eff_rhs[XLEN-1]);
                        remainder_negative_next =
                            signed_div(issue_entry.ctrl.alu_op) && eff_lhs[XLEN-1];
                        special_next = special_case(issue_entry.ctrl.alu_op,
                            eff_lhs, eff_rhs, special_result_next);
                        state_next = DIV_RUN;
                    end
                end
                DIV_RUN: begin
                    if (special_q) begin
                        packet_next.data = word_result(op_q, special_result_q);
                        state_next = DIV_DONE;
                    end else begin
                        trial_remainder = {remainder_q[XLEN-1:0],
                            dividend_q[XLEN-1:XLEN-4]};
                        qdigit = select_qdigit(trial_remainder, divisor_q);
                        remainder_next = trial_remainder -
                            (REM_W'(divisor_q) * REM_W'(qdigit));
                        quotient_next = {quotient_q[XLEN-5:0], qdigit};
                        dividend_next = {dividend_q[XLEN-5:0], 4'b0};
                        iter_next = iter_q + 1'b1;
                        if (iter_q == ITER_W'(DIV_ITERS - 1)) begin
                            packet_next.data = word_result(op_q,
                                final_result(op_q,
                                    {quotient_q[XLEN-5:0], qdigit},
                                    trial_remainder -
                                        (REM_W'(divisor_q) * REM_W'(qdigit)),
                                    quotient_negative_q, remainder_negative_q));
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
            op_q <= ALU_DIV;
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
            op_q <= op_next;
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

    function automatic logic is_word_div(input alu_op_t op);
        is_word_div = (op == ALU_DIVW) || (op == ALU_DIVUW) ||
            (op == ALU_REMW) || (op == ALU_REMUW);
    endfunction

    function automatic logic unsigned_div(input alu_op_t op);
        unsigned_div = (op == ALU_DIVU) || (op == ALU_REMU) ||
            (op == ALU_DIVUW) || (op == ALU_REMUW);
    endfunction

    function automatic logic signed_div(input alu_op_t op);
        signed_div = !unsigned_div(op);
    endfunction

    function automatic logic is_rem_op(input alu_op_t op);
        is_rem_op = (op == ALU_REM) || (op == ALU_REMU) ||
            (op == ALU_REMW) || (op == ALU_REMUW);
    endfunction

    // W-form results are the low 32 bits sign-extended to XLEN.
    function automatic logic [XLEN-1:0] word_result(input alu_op_t op,
            input logic [XLEN-1:0] value);
        word_result = is_word_div(op) ? XLEN'($signed(value[31:0])) : value;
    endfunction

    function automatic logic [XLEN-1:0] abs_operand(input logic [XLEN-1:0] value,
            input logic is_signed);
        abs_operand = (is_signed && value[XLEN-1]) ? (~value + 1'b1) : value;
    endfunction

    function automatic logic special_case(input alu_op_t op,
            input logic [XLEN-1:0] lhs, input logic [XLEN-1:0] rhs,
            output logic [XLEN-1:0] result);
        special_case = 1'b0;
        result = '0;
        if (rhs == '0) begin
            special_case = 1'b1;
            if (is_rem_op(op)) begin
                result = lhs;
            end else begin
                result = '1;
            end
        end else if (signed_div(op) && !is_word_div(op) &&
                (lhs == {1'b1, {(XLEN-1){1'b0}}}) && (rhs == '1)) begin
            // INT_MIN / -1 overflow (full-width ops only; W-form operands are
            // 32-bit values extended to XLEN and resolve through the divider).
            special_case = 1'b1;
            result = is_rem_op(op) ? '0 : {1'b1, {(XLEN-1){1'b0}}};
        end
    endfunction

    function automatic logic [3:0] select_qdigit(input logic [REM_W-1:0] remainder,
            input logic [XLEN-1:0] divisor);
        logic [REM_W-1:0] divisor_ext;
        select_qdigit = 4'd0;
        divisor_ext = REM_W'(divisor);
        for (int i = 0; i < 16; i += 1) begin
            if ((divisor_ext * REM_W'(i)) <= remainder) begin
                select_qdigit = 4'(i);
            end
        end
    endfunction

    function automatic logic [XLEN-1:0] final_result(input alu_op_t op,
            input logic [XLEN-1:0] quotient, input logic [REM_W-1:0] remainder,
            input logic quotient_negative, input logic remainder_negative);
        logic [XLEN-1:0] signed_quotient;
        logic [XLEN-1:0] signed_remainder;
        signed_quotient = quotient_negative ? (~quotient + 1'b1) : quotient;
        signed_remainder = remainder_negative ?
            (~remainder[XLEN-1:0] + 1'b1) : remainder[XLEN-1:0];
        final_result = is_rem_op(op) ? signed_remainder : signed_quotient;
    endfunction

endmodule: ooo_div_unit
