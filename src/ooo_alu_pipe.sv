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
    input  logic [31:0]      csr_rdata,
    input  logic             csr_illegal,
    input  branch_mask_t     abort_mask,
    output writeback_packet_t writeback
);

    writeback_packet_t wb_next;

    /* Simulation halt-on-ecall (a0 in {10,11}) is how the directed/ACT tests
     * signal pass/fail. A real OS boot can't use it: the kernel's SBI ecalls
     * legitimately carry a0==10 (console '\n') or 11, which would spuriously
     * halt. The +no_ecall_halt plusarg disables it so those ecalls trap to the
     * M-mode SBI firmware normally; the run instead terminates via the console
     * watch / HTIF tohost. Default (no plusarg) preserves the test behaviour. */
    logic ecall_halt_en;
    initial ecall_halt_en = !$test$plusargs("no_ecall_halt");

    always_comb begin
        wb_next = '0;
        if (issue_valid && ((issue_entry.branch_mask & abort_mask) == '0)) begin
            wb_next.valid = 1'b1;
            wb_next.active_id = issue_entry.active_id;
            wb_next.pc = issue_entry.pc;
            wb_next.instr = issue_entry.instr;
            wb_next.prd = issue_entry.prd;
            wb_next.has_dest = issue_entry.has_dest;
            wb_next.data = result_for(issue_entry, rs1_data, rs2_data);
            wb_next.branch_mask = issue_entry.branch_mask;
            wb_next.branch_valid = (issue_entry.ctrl.pc_source == PC_cond) ||
                (issue_entry.ctrl.pc_source == PC_uncond) ||
                (issue_entry.ctrl.pc_source == PC_indirect);
            wb_next.branch_id = issue_entry.branch_id;
            wb_next.branch_mispredict = wb_next.branch_valid &&
                branch_mispredict_for(issue_entry, rs1_data, rs2_data);
            wb_next.redirect_pc = actual_target_for(issue_entry, rs1_data, rs2_data);
            wb_next.control_predicted = issue_entry.control_predicted;
            wb_next.predicted_pc = issue_entry.predicted_pc;
            wb_next.predictor_info = issue_entry.predictor_info;
            wb_next.csr_write = issue_entry.ctrl.csr_write;
            wb_next.csr_addr = issue_entry.instr[31:20];
            wb_next.csr_wdata = csr_write_data_for(issue_entry, rs1_data,
                csr_rdata);
            wb_next.fp_write = issue_entry.ctrl.fp_writes_fpr &&
                !issue_entry.ctrl.memRead && !issue_entry.ctrl.memWrite;
            wb_next.fp_rd = issue_entry.fp_rd;
            wb_next.fp_data = fp_result_for(issue_entry, rs1_data);
            wb_next.exception = issue_entry.ctrl.illegal_instr ||
                ((issue_entry.ctrl.exec_class == EXEC_CSR) &&
                 (csr_illegal || (issue_entry.ctrl.csr_write &&
                  (issue_entry.instr[31:30] == 2'b11))));
            wb_next.exc_cause = 5'd2;   // EXC_ILLEGAL_INSTR
            wb_next.halted = ecall_halt_en && issue_entry.ctrl.syscall &&
                ((rs1_data == 32'ha) || (rs1_data == 32'hb));

            // Instruction-address-misaligned (IALIGN=32, no C extension): a
            // taken branch or jump whose target is not 4-byte aligned faults on
            // the control-transfer instruction itself (epc = its PC, tval =
            // target). JALR already forces target bit[0] to 0; JAL/branch
            // immediates are even, so the only way to misalign is target bit[1].
            // Reported here as an exception that the ROB takes precisely at
            // commit; fetch is kept on an aligned path meanwhile since the
            // misaligned target is never actually executed.
            if (control_taken(issue_entry, rs1_data, rs2_data) &&
                    actual_target_for(issue_entry, rs1_data, rs2_data)[1]) begin
                wb_next.exception = 1'b1;
                wb_next.exc_cause = 5'd0;   // EXC_INSTR_MISALIGNED
                wb_next.data =
                    actual_target_for(issue_entry, rs1_data, rs2_data);
                wb_next.redirect_pc = issue_entry.pc + 32'd4;
            end

            // Instruction-fetch page/access fault: the frontend translated this
            // PC and faulted, replacing the instruction with a forced NOP that
            // carries the fault. Report it precisely at commit with m/stval set
            // to the faulting virtual address (the PC). Takes priority over any
            // value decoded from the (discarded) NOP encoding.
            if (issue_entry.ctrl.fetch_fault) begin
                wb_next.exception = 1'b1;
                wb_next.exc_cause = issue_entry.ctrl.fetch_fault_cause;
                wb_next.data = issue_entry.pc;
            end
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
            default: begin
                if (entry.ctrl.exec_class == EXEC_CSR) begin
                    result_for = csr_rdata;
                end else if (entry.ctrl.exec_class == EXEC_FP) begin
                    result_for = fp_gpr_result_for(entry, src1);
                end else begin
                    result_for = alu_result(alu_a, alu_b, entry.ctrl.alu_op);
                end
            end
        endcase
    endfunction

    function automatic fp_reg_data_t fp_result_for(issue_entry_t entry,
            logic [31:0] int_src);
        real a;
        real b;
        real c;
        real r;
        logic [31:0] s_bits;
        logic [63:0] d_bits;
        a = entry.ctrl.fp_double ? $bitstoreal(entry.fp_src1_data) :
            fp32_to_real(entry.fp_src1_data[31:0]);
        b = entry.ctrl.fp_double ? $bitstoreal(entry.fp_src2_data) :
            fp32_to_real(entry.fp_src2_data[31:0]);
        c = entry.ctrl.fp_double ? $bitstoreal(entry.fp_src3_data) :
            fp32_to_real(entry.fp_src3_data[31:0]);
        unique case (entry.ctrl.fp_op)
            FP_ADD: r = a + b;
            FP_SUB: r = a - b;
            FP_MUL: r = a * b;
            FP_DIV: r = a / b;
            FP_SQRT: r = $sqrt(a);
            FP_MIN: r = (a < b) ? a : b;
            FP_MAX: r = (a > b) ? a : b;
            FP_MADD: r = (a * b) + c;
            FP_MSUB: r = (a * b) - c;
            FP_NMSUB: r = c - (a * b);
            FP_NMADD: r = -(a * b) - c;
            FP_CVT_F_W: r = real'(signed'(int_src));
            FP_CVT_F_WU: r = real'(int_src);
            FP_MV_F_X: begin
                fp_result_for = {32'hffff_ffff, int_src};
                return fp_result_for;
            end
            FP_SGNJ, FP_SGNJN, FP_SGNJX: begin
                fp_result_for = fp_sign_result(entry);
                return fp_result_for;
            end
            default: r = a;
        endcase
        if (entry.ctrl.fp_double) begin
            d_bits = $realtobits(r);
            fp_result_for = d_bits;
        end else begin
            s_bits = real_to_fp32(r);
            fp_result_for = {32'hffff_ffff, s_bits};
        end
    endfunction

    function automatic fp_reg_data_t fp_sign_result(issue_entry_t entry);
        logic [63:0] mag;
        logic sign_bit;
        if (entry.ctrl.fp_double) begin
            mag = {1'b0, entry.fp_src1_data[62:0]};
            unique case (entry.ctrl.fp_op)
                FP_SGNJ: sign_bit = entry.fp_src2_data[63];
                FP_SGNJN: sign_bit = ~entry.fp_src2_data[63];
                default: sign_bit = entry.fp_src1_data[63] ^ entry.fp_src2_data[63];
            endcase
            fp_sign_result = {sign_bit, mag[62:0]};
        end else begin
            mag = {32'hffff_ffff, 1'b0, entry.fp_src1_data[30:0]};
            unique case (entry.ctrl.fp_op)
                FP_SGNJ: sign_bit = entry.fp_src2_data[31];
                FP_SGNJN: sign_bit = ~entry.fp_src2_data[31];
                default: sign_bit = entry.fp_src1_data[31] ^ entry.fp_src2_data[31];
            endcase
            fp_sign_result = {32'hffff_ffff, sign_bit, mag[30:0]};
        end
    endfunction

    function automatic logic [31:0] fp_gpr_result_for(issue_entry_t entry,
            logic [31:0] int_src);
        real a;
        real b;
        a = entry.ctrl.fp_double ? $bitstoreal(entry.fp_src1_data) :
            fp32_to_real(entry.fp_src1_data[31:0]);
        b = entry.ctrl.fp_double ? $bitstoreal(entry.fp_src2_data) :
            fp32_to_real(entry.fp_src2_data[31:0]);
        unique case (entry.ctrl.fp_op)
            FP_CVT_W:  fp_gpr_result_for = 32'($rtoi(a));
            FP_CVT_WU: fp_gpr_result_for = 32'($rtoi(a));
            FP_MV_X:   fp_gpr_result_for = entry.fp_src1_data[31:0];
            FP_EQ:     fp_gpr_result_for = {31'b0, a == b};
            FP_LT:     fp_gpr_result_for = {31'b0, a < b};
            FP_LE:     fp_gpr_result_for = {31'b0, a <= b};
            FP_CLASS:  fp_gpr_result_for = fp_class(entry);
            default:   fp_gpr_result_for = int_src;
        endcase
    endfunction

    function automatic real fp32_to_real(input logic [31:0] bits);
        int exp;
        int frac;
        real mant;
        real value;
        exp = int'(bits[30:23]);
        frac = int'(bits[22:0]);
        if (exp == 0) begin
            mant = real'(frac) / 8388608.0;
            value = mant * pow2(-126);
        end else if (exp == 255) begin
            value = 0.0;
        end else begin
            mant = 1.0 + (real'(frac) / 8388608.0);
            value = mant * pow2(exp - 127);
        end
        fp32_to_real = bits[31] ? -value : value;
    endfunction

    function automatic logic [31:0] real_to_fp32(input real value);
        logic sign;
        int exp;
        int frac;
        real norm;
        real abs_value;
        logic [7:0] exp_bits;
        logic [22:0] frac_bits;
        if (value == 0.0) begin
            real_to_fp32 = 32'b0;
        end else begin
            sign = value < 0.0;
            abs_value = sign ? -value : value;
            exp = 127;
            norm = abs_value;
            while (norm >= 2.0) begin
                norm = norm / 2.0;
                exp += 1;
            end
            while (norm < 1.0) begin
                norm = norm * 2.0;
                exp -= 1;
            end
            frac = int'((norm - 1.0) * 8388608.0);
            if (frac >= 8388608) begin
                frac = 0;
                exp += 1;
            end
            exp_bits = 8'(exp);
            frac_bits = 23'(frac);
            real_to_fp32 = {sign, exp_bits, frac_bits};
        end
    endfunction

    function automatic real pow2(input int exponent);
        real value;
        value = 1.0;
        if (exponent >= 0) begin
            for (int i = 0; i < exponent; i += 1) begin
                value = value * 2.0;
            end
        end else begin
            for (int i = 0; i < -exponent; i += 1) begin
                value = value / 2.0;
            end
        end
        pow2 = value;
    endfunction

    function automatic logic [31:0] fp_class(issue_entry_t entry);
        logic sign;
        logic [10:0] exp_d;
        logic [51:0] frac_d;
        logic [7:0] exp_s;
        logic [22:0] frac_s;
        fp_class = 32'b0;
        if (entry.ctrl.fp_double) begin
            sign = entry.fp_src1_data[63];
            exp_d = entry.fp_src1_data[62:52];
            frac_d = entry.fp_src1_data[51:0];
            if (exp_d == 11'h7ff) begin
                fp_class[frac_d == 0 ? (sign ? 0 : 7) : 9] = 1'b1;
            end else if (exp_d == 0) begin
                fp_class[frac_d == 0 ? (sign ? 3 : 4) : (sign ? 2 : 5)] = 1'b1;
            end else begin
                fp_class[sign ? 1 : 6] = 1'b1;
            end
        end else begin
            sign = entry.fp_src1_data[31];
            exp_s = entry.fp_src1_data[30:23];
            frac_s = entry.fp_src1_data[22:0];
            if (exp_s == 8'hff) begin
                fp_class[frac_s == 0 ? (sign ? 0 : 7) : 9] = 1'b1;
            end else if (exp_s == 0) begin
                fp_class[frac_s == 0 ? (sign ? 3 : 4) : (sign ? 2 : 5)] = 1'b1;
            end else begin
                fp_class[sign ? 1 : 6] = 1'b1;
            end
        end
    endfunction

    function automatic logic [31:0] csr_write_data_for(issue_entry_t entry,
            logic [31:0] src1, logic [31:0] old_value);
        logic [31:0] operand;
        operand = entry.ctrl.useImm ? {27'b0, entry.instr[19:15]} : src1;
        unique case (entry.ctrl.csr_op)
            CSR_RW, CSR_RWI: csr_write_data_for = operand;
            CSR_RS, CSR_RSI: csr_write_data_for = old_value | operand;
            CSR_RC, CSR_RCI: csr_write_data_for = old_value & ~operand;
            default: csr_write_data_for = old_value;
        endcase
    endfunction

    function automatic logic branch_mispredict_for(issue_entry_t entry,
            logic [31:0] src1, logic [31:0] src2);
        if (entry.control_predicted) begin
            branch_mispredict_for = actual_target_for(entry, src1, src2) !=
                entry.predicted_pc;
        end else if ((entry.ctrl.pc_source == PC_uncond) ||
                (entry.ctrl.pc_source == PC_indirect)) begin
            // Unpredicted jumps stall fetch and must redirect on resolution.
            branch_mispredict_for = 1'b1;
        end else begin
            branch_mispredict_for = actual_target_for(entry, src1, src2) !=
                (entry.pc + 32'd4);
        end
    endfunction

    function automatic logic control_taken(issue_entry_t entry,
            logic [31:0] src1, logic [31:0] src2);
        control_taken = (entry.ctrl.pc_source == PC_uncond) ||
            (entry.ctrl.pc_source == PC_indirect) ||
            ((entry.ctrl.pc_source == PC_cond) &&
             branch_cmp(src1, src2, entry.ctrl.alu_op));
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
        logic signed [63:0] signed_product;
        logic signed [64:0] mixed_product;
        logic [63:0] unsigned_product;
        signed_product = signed'(a) * signed'(b);
        mixed_product = $signed({a[31], a}) * $signed({1'b0, b});
        unsigned_product = {32'b0, a} * {32'b0, b};
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
            ALU_MUL: alu_result = signed_product[31:0];
            ALU_MULH: alu_result = signed_product[63:32];
            ALU_MULHSU: alu_result = mixed_product[63:32];
            ALU_MULHU: alu_result = unsigned_product[63:32];
            ALU_DIV: begin
                if (b == 32'b0) begin
                    alu_result = 32'hffff_ffff;
                end else if ((a == 32'h8000_0000) && (b == 32'hffff_ffff)) begin
                    alu_result = 32'h8000_0000;
                end else begin
                    alu_result = signed'(a) / signed'(b);
                end
            end
            ALU_DIVU: alu_result = (b == 32'b0) ? 32'hffff_ffff : (a / b);
            ALU_REM: begin
                if (b == 32'b0) begin
                    alu_result = a;
                end else if ((a == 32'h8000_0000) && (b == 32'hffff_ffff)) begin
                    alu_result = 32'b0;
                end else begin
                    alu_result = signed'(a) % signed'(b);
                end
            end
            ALU_REMU: alu_result = (b == 32'b0) ? a : (a % b);
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
