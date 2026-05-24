`include "ooo_types.vh"

`default_nettype none

module niigo_fp_unit
    import OOO_Types::*;
(
    input  logic              clk,
    input  logic              rst_l,
    input  logic              issue_valid,
    output logic              issue_ready,
    input  issue_entry_t      issue_entry,
    input  logic [31:0]       rs1_data,
    input  logic [2:0]        frm,
    input  branch_mask_t      abort_mask,
    input  logic              writeback_ready,
    output writeback_packet_t writeback
);

    localparam int FP_LATENCY = 2;

    logic [FP_LATENCY-1:0] valid_q;
    writeback_packet_t pipe_q [FP_LATENCY];
    logic advance;
    logic output_aborted;

    assign output_aborted = valid_q[FP_LATENCY-1] &&
        ((pipe_q[FP_LATENCY-1].branch_mask & abort_mask) != '0);
    assign advance = writeback_ready || output_aborted || !valid_q[FP_LATENCY-1];
    assign issue_ready = advance;

    always_comb begin
        writeback = pipe_q[FP_LATENCY-1];
        writeback.valid = valid_q[FP_LATENCY-1] && !output_aborted;
    end

    always_ff @(posedge clk or negedge rst_l) begin
        if (!rst_l) begin
            valid_q <= '0;
            for (int i = 0; i < FP_LATENCY; i += 1) begin
                pipe_q[i] <= '0;
            end
        end else if (advance) begin
            for (int i = FP_LATENCY - 1; i > 0; i -= 1) begin
                valid_q[i] <= valid_q[i - 1] &&
                    ((pipe_q[i - 1].branch_mask & abort_mask) == '0);
                pipe_q[i] <= pipe_q[i - 1];
            end
            valid_q[0] <= issue_valid &&
                ((issue_entry.branch_mask & abort_mask) == '0);
            pipe_q[0] <= packet_for(issue_entry, rs1_data, frm);
        end
    end

    function automatic writeback_packet_t packet_for(input issue_entry_t entry,
            input logic [31:0] int_src, input logic [2:0] csr_frm);
        writeback_packet_t packet;
        logic [2:0] rm;
        packet = '0;
        rm = (entry.instr[14:12] == 3'b111) ? csr_frm : entry.instr[14:12];
        packet.valid = 1'b1;
        packet.active_id = entry.active_id;
        packet.pc = entry.pc;
        packet.instr = entry.instr;
        packet.prd = entry.prd;
        packet.has_dest = entry.has_dest;
        packet.branch_mask = entry.branch_mask;
        packet.data = fp_gpr_result_for(entry, int_src);
        packet.fp_write = entry.ctrl.fp_writes_fpr &&
            !entry.ctrl.memRead && !entry.ctrl.memWrite;
        packet.fp_rd = entry.fp_rd;
        packet.fp_data = fp_result_for(entry, int_src, rm);
        packet.fp_fflags_valid = fp_flags_write_valid(entry);
        packet.fp_fflags = fp_flags_for(entry, rm);
        return packet;
    endfunction

    function automatic logic fp_flags_write_valid(input issue_entry_t entry);
        fp_flags_write_valid = entry.ctrl.exec_class == EXEC_FP &&
            !entry.ctrl.memRead && !entry.ctrl.memWrite &&
            (entry.ctrl.fp_op != FP_MV_X) && (entry.ctrl.fp_op != FP_MV_F_X) &&
            (entry.ctrl.fp_op != FP_SGNJ) && (entry.ctrl.fp_op != FP_SGNJN) &&
            (entry.ctrl.fp_op != FP_SGNJX) && (entry.ctrl.fp_op != FP_CLASS);
    endfunction

    function automatic logic [4:0] fp_flags_for(input issue_entry_t entry,
            input logic [2:0] rm);
        logic [4:0] flags;
        logic src1_nan;
        logic src2_nan;
        logic src2_zero;
        flags = 5'b0;
        src1_nan = fp_is_nan(entry.ctrl.fp_double, entry.fp_src1_data);
        src2_nan = fp_is_nan(entry.ctrl.fp_double, entry.fp_src2_data);
        src2_zero = fp_is_zero(entry.ctrl.fp_double, entry.fp_src2_data);
        if ((rm > 3'b100) && (entry.instr[14:12] == 3'b111)) begin
            flags[4] = 1'b1;
        end
        if (src1_nan || src2_nan) begin
            flags[4] = (entry.ctrl.fp_op == FP_LT) || (entry.ctrl.fp_op == FP_LE) ||
                (entry.ctrl.fp_op == FP_MIN) || (entry.ctrl.fp_op == FP_MAX);
        end
        if ((entry.ctrl.fp_op == FP_DIV) && src2_zero &&
                !fp_is_zero(entry.ctrl.fp_double, entry.fp_src1_data)) begin
            flags[3] = 1'b1;
        end
        if ((entry.ctrl.fp_op == FP_SQRT) && fp_is_negative(entry.ctrl.fp_double,
                entry.fp_src1_data) && !fp_is_zero(entry.ctrl.fp_double,
                entry.fp_src1_data)) begin
            flags[4] = 1'b1;
        end
        fp_flags_for = flags;
    endfunction

    function automatic logic fp_is_nan(input logic is_double,
            input fp_reg_data_t value);
        if (is_double) begin
            fp_is_nan = (value[62:52] == 11'h7ff) && (value[51:0] != '0);
        end else begin
            fp_is_nan = (value[30:23] == 8'hff) && (value[22:0] != '0);
        end
    endfunction

    function automatic logic fp_is_zero(input logic is_double,
            input fp_reg_data_t value);
        if (is_double) begin
            fp_is_zero = value[62:0] == '0;
        end else begin
            fp_is_zero = value[30:0] == '0;
        end
    endfunction

    function automatic logic fp_is_negative(input logic is_double,
            input fp_reg_data_t value);
        fp_is_negative = is_double ? value[63] : value[31];
    endfunction

    function automatic fp_reg_data_t fp_result_for(issue_entry_t entry,
            logic [31:0] int_src, logic [2:0] rm);
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
            FP_CVT_F_F: begin
                fp_result_for = fp_convert_format(entry, rm);
                return fp_result_for;
            end
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
            s_bits = real_to_fp32(r, rm);
            fp_result_for = {32'hffff_ffff, s_bits};
        end
    endfunction

    function automatic fp_reg_data_t fp_convert_format(input issue_entry_t entry,
            input logic [2:0] rm);
        real value;
        if (entry.ctrl.fp_double) begin
            value = fp32_to_real(entry.fp_src1_data[31:0]);
            fp_convert_format = $realtobits(value);
        end else begin
            value = $bitstoreal(entry.fp_src1_data);
            fp_convert_format = {32'hffff_ffff, real_to_fp32(value, rm)};
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

    function automatic logic [31:0] real_to_fp32(input real value,
            input logic [2:0] rm);
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
            frac = rounded_fraction(norm, sign, rm);
            if (frac >= 8388608) begin
                frac = 0;
                exp += 1;
            end
            exp_bits = 8'(exp);
            frac_bits = 23'(frac);
            real_to_fp32 = {sign, exp_bits, frac_bits};
        end
    endfunction

    function automatic int rounded_fraction(input real norm, input logic sign,
            input logic [2:0] rm);
        real scaled;
        int trunc_value;
        scaled = (norm - 1.0) * 8388608.0;
        trunc_value = int'(scaled);
        unique case (rm)
            3'b010: rounded_fraction = trunc_value;
            3'b011: rounded_fraction = (!sign && (scaled > real'(trunc_value))) ?
                trunc_value + 1 : trunc_value;
            3'b100: rounded_fraction = (sign && (scaled > real'(trunc_value))) ?
                trunc_value + 1 : trunc_value;
            default: rounded_fraction = int'(scaled + 0.5);
        endcase
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

endmodule: niigo_fp_unit
