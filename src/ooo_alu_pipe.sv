`include "ooo_types.vh"

`default_nettype none

module ooo_alu_pipe
    import OOO_Types::*;
(
    input  logic             clk,
    input  logic             rst_l,
    input  logic             issue_valid,
    input  issue_entry_t     issue_entry,
    input  logic [31:0]      rs1_data,
    input  logic [31:0]      rs2_data,
    input  branch_mask_t     abort_mask,
    output writeback_packet_t writeback
);

    writeback_packet_t wb_next;

    always_comb begin
        wb_next = '0;
        if (issue_valid && ((issue_entry.branch_mask & abort_mask) == '0)) begin
            wb_next.valid = 1'b1;
            wb_next.active_id = issue_entry.active_id;
            wb_next.prd = issue_entry.prd;
            wb_next.has_dest = issue_entry.has_dest;
            wb_next.data = result_for(issue_entry, rs1_data, rs2_data);
            wb_next.branch_mask = issue_entry.branch_mask;
            wb_next.branch_valid = (issue_entry.ctrl.pc_source == PC_cond) ||
                (issue_entry.ctrl.pc_source == PC_uncond) ||
                (issue_entry.ctrl.pc_source == PC_indirect);
            wb_next.branch_id = issue_entry.branch_id;
            wb_next.branch_mispredict = wb_next.branch_valid &&
                (actual_target_for(issue_entry, rs1_data, rs2_data) != (issue_entry.pc + 32'd4));
            wb_next.redirect_pc = actual_target_for(issue_entry, rs1_data, rs2_data);
            wb_next.exception = issue_entry.ctrl.illegal_instr;
            wb_next.halted = issue_entry.ctrl.syscall && (rs1_data == 32'ha);
        end
    end

    always_ff @(posedge clk or negedge rst_l) begin
        if (!rst_l) begin
            writeback <= '0;
        end else if ((wb_next.branch_mask & abort_mask) != '0) begin
            writeback <= '0;
        end else begin
            writeback <= wb_next;
        end
    end

    function automatic logic [31:0] result_for(issue_entry_t entry,
            logic [31:0] src1, logic [31:0] src2);
        logic [31:0] alu_a;
        logic [31:0] alu_b;
        alu_a = entry.ctrl.usePC ? entry.pc : src1;
        alu_b = entry.ctrl.useImm ? entry.imm : src2;
        unique case (entry.ctrl.rd_source)
            RD_PC4: result_for = entry.pc + 32'd4;
            RD_IMM: result_for = entry.imm;
            RD_CMP: result_for = {31'b0, branch_cmp(src1,
                entry.ctrl.useImm ? entry.imm : src2, entry.ctrl.alu_op)};
            default: result_for = alu_result(alu_a, alu_b, entry.ctrl.alu_op);
        endcase
    endfunction

    function automatic logic [31:0] actual_target_for(issue_entry_t entry,
            logic [31:0] src1, logic [31:0] src2);
        if (entry.ctrl.pc_source == PC_uncond) begin
            actual_target_for = entry.pc + entry.imm;
        end else if (entry.ctrl.pc_source == PC_indirect) begin
            actual_target_for = (src1 + entry.imm) & 32'hffff_fffe;
        end else if ((entry.ctrl.pc_source == PC_cond) &&
                branch_cmp(src1, src2, entry.ctrl.alu_op)) begin
            actual_target_for = entry.pc + entry.imm;
        end else begin
            actual_target_for = entry.pc + 32'd4;
        end
    endfunction

    function automatic logic [31:0] alu_result(logic [31:0] a, logic [31:0] b,
            alu_op_t op);
        unique case (op)
            ALU_ADD: alu_result = a + b;
            ALU_SUB: alu_result = a - b;
            ALU_XOR: alu_result = a ^ b;
            ALU_OR:  alu_result = a | b;
            ALU_AND: alu_result = a & b;
            ALU_SLL: alu_result = a << b[4:0];
            ALU_SRL: alu_result = a >> b[4:0];
            ALU_SRA: alu_result = signed'(a) >>> b[4:0];
            ALU_SLT: alu_result = {31'b0, signed'(a) < signed'(b)};
            ALU_SLTU: alu_result = {31'b0, a < b};
            default: alu_result = 32'b0;
        endcase
    endfunction

    function automatic logic branch_cmp(logic [31:0] a, logic [31:0] b,
            alu_op_t op);
        unique case (op)
            ALU_BEQ: branch_cmp = (a == b);
            ALU_BNE: branch_cmp = (a != b);
            ALU_BLT: branch_cmp = signed'(a) < signed'(b);
            ALU_BGE: branch_cmp = signed'(a) >= signed'(b);
            ALU_BLTU: branch_cmp = a < b;
            ALU_BGEU: branch_cmp = a >= b;
            ALU_SLT: branch_cmp = signed'(a) < signed'(b);
            ALU_SLTU: branch_cmp = a < b;
            default: branch_cmp = 1'b0;
        endcase
    endfunction

endmodule: ooo_alu_pipe
