`include "ooo_types.vh"

`default_nettype none

module ooo_mul_unit
    import OOO_Types::*;
(
    input wire logic              clk,
    input wire logic              rst_l,
    input wire logic              issue_valid,
    output logic              issue_ready,
    input wire issue_entry_t      issue_entry,
    input wire logic [XLEN-1:0]   rs1_data,
    input wire logic [XLEN-1:0]   rs2_data,
    input wire branch_mask_t      abort_mask,
    // Resolved-branch checkpoint bits to clear from every in-flight stage's
    // branch_mask each cycle (see ooo_div_unit for the full rationale): a
    // multiply held across a branch resolution + a later reuse of that freed
    // checkpoint bit would otherwise carry a stale mask and be spuriously
    // aborted on the reused branch, dropping its writeback -> ROB-head deadlock.
    input wire branch_mask_t      reset_mask,
    // Precise-trap full flush: squash every in-flight stage so no stale
    // writeback lands after the pipeline is reset (the active-list id this op
    // targets may be reused by a younger instruction next cycle).
    input wire logic              flush,
    input wire logic              writeback_ready,
    output writeback_packet_t writeback
);

    localparam int MUL_LATENCY = 3;

    // Metadata pipeline (active_id/prd/branch_mask/...); pipe_q[MUL_LATENCY-1].data
    // holds the finalized result. The DATAPATH (the signed 64x64 multiply) is now
    // spread across the three stages instead of being computed combinationally in
    // stage 0 -- that single-cycle multiply (hand-written wallace_product + sign
    // fixups, fed by the abort->writeback->operand network) was the FB2b worst
    // path (abort_mask -> MulUnit/pipe_q[0][data], ~906 logic levels). Latency is
    // unchanged (still MUL_LATENCY cycles), so there is no IPC cost.
    logic [MUL_LATENCY-1:0] valid_q;
    writeback_packet_t pipe_q [MUL_LATENCY];
    // Stage 0 -> 1: operand magnitudes + product sign + op (registered).
    logic [XLEN-1:0]   s0_lhs_mag_q, s0_rhs_mag_q;
    logic              s0_neg_q;
    alu_op_t           s0_op_q;
    // Stage 1 -> 2: |lhs|*|rhs| (DSP-inferred from `*`) + sign + op (registered).
    logic [2*XLEN-1:0] s1_prod_mag_q;
    logic              s1_neg_q;
    alu_op_t           s1_op_q;

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
            s0_lhs_mag_q <= '0;
            s0_rhs_mag_q <= '0;
            s0_neg_q     <= 1'b0;
            s0_op_q      <= ALU_MUL;
            s1_prod_mag_q <= '0;
            s1_neg_q      <= 1'b0;
            s1_op_q       <= ALU_MUL;
        end else if (flush) begin
            valid_q <= '0;
        end else if (advance) begin
            // Stage 2 <- Stage 1: restore the product sign and select the result
            // half/width, registered into pipe_q[last].data (shallow output).
            pipe_q[MUL_LATENCY-1] <= pipe_q[MUL_LATENCY-2];
            pipe_q[MUL_LATENCY-1].data <=
                mul_select(s1_op_q, s1_neg_q, s1_prod_mag_q);
            // Age the advancing mask by this cycle's resolved-branch bits.
            pipe_q[MUL_LATENCY-1].branch_mask <=
                pipe_q[MUL_LATENCY-2].branch_mask & ~reset_mask;
            valid_q[MUL_LATENCY-1] <= valid_q[MUL_LATENCY-2] &&
                ((pipe_q[MUL_LATENCY-2].branch_mask & abort_mask) == '0);
            // Stage 1 <- Stage 0: magnitude multiply (zero-extended operands -> a
            // 2*XLEN unsigned product; Vivado infers a pipelined DSP cascade).
            pipe_q[1] <= pipe_q[0];
            pipe_q[1].branch_mask <= pipe_q[0].branch_mask & ~reset_mask;
            s1_prod_mag_q <= {{XLEN{1'b0}}, s0_lhs_mag_q} *
                             {{XLEN{1'b0}}, s0_rhs_mag_q};
            s1_neg_q <= s0_neg_q;
            s1_op_q  <= s0_op_q;
            valid_q[1] <= valid_q[0] &&
                ((pipe_q[0].branch_mask & abort_mask) == '0);
            // Stage 0 <- issue: capture operand magnitudes + product sign.
            pipe_q[0] <= meta_for(issue_entry);
            pipe_q[0].branch_mask <= issue_entry.branch_mask & ~reset_mask;
            s0_lhs_mag_q <= operand_mag(issue_entry.ctrl.alu_op, rs1_data, 1'b1);
            s0_rhs_mag_q <= operand_mag(issue_entry.ctrl.alu_op, rs2_data, 1'b0);
            s0_neg_q     <= product_negative(issue_entry.ctrl.alu_op,
                                             rs1_data, rs2_data);
            s0_op_q      <= issue_entry.ctrl.alu_op;
            valid_q[0] <= issue_valid &&
                ((issue_entry.branch_mask & abort_mask) == '0);
        end else begin
            // Stalled (a completed output is waiting for a writeback slot): the
            // stages hold, but the held branch_masks must still be aged by
            // reset_mask -- a 1-cycle reset pulse missed here leaves the output's
            // mask permanently stale and it would later false-abort on a reused
            // checkpoint bit (the same deadlock as the div unit).
            for (int i = 0; i < MUL_LATENCY; i += 1) begin
                pipe_q[i].branch_mask <= pipe_q[i].branch_mask & ~reset_mask;
            end
        end
    end

    function automatic writeback_packet_t meta_for(input issue_entry_t entry);
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

    // Per-operand signedness for each MUL form (matches the prior wallace path):
    // MUL/MULH/MULW sign both operands; MULHSU signs only the lhs; MULHU neither.
    function automatic logic [XLEN-1:0] operand_mag(input alu_op_t op,
            input logic [XLEN-1:0] val, input logic is_lhs);
        logic is_signed;
        if (is_lhs) begin
            is_signed = (op == ALU_MUL) || (op == ALU_MULH) ||
                (op == ALU_MULHSU) || (op == ALU_MULW);
        end else begin
            is_signed = (op == ALU_MUL) || (op == ALU_MULH) || (op == ALU_MULW);
        end
        operand_mag = (is_signed && val[XLEN-1]) ? (~val + 1'b1) : val;
    endfunction

    function automatic logic product_negative(input alu_op_t op,
            input logic [XLEN-1:0] lhs, input logic [XLEN-1:0] rhs);
        logic lhs_signed;
        logic rhs_signed;
        lhs_signed = (op == ALU_MUL) || (op == ALU_MULH) ||
            (op == ALU_MULHSU) || (op == ALU_MULW);
        rhs_signed = (op == ALU_MUL) || (op == ALU_MULH) || (op == ALU_MULW);
        product_negative = (lhs_signed && lhs[XLEN-1]) ^
            (rhs_signed && rhs[XLEN-1]);
    endfunction

    function automatic logic [XLEN-1:0] mul_select(input alu_op_t op,
            input logic neg, input logic [2*XLEN-1:0] prod_mag);
        logic [2*XLEN-1:0] product;
        product = neg ? (~prod_mag + 1'b1) : prod_mag;
        unique case (op)
            ALU_MUL:  mul_select = product[XLEN-1:0];
            // MULW: low 32 product bits, sign-extended to XLEN.
            ALU_MULW: mul_select = XLEN'($signed(product[31:0]));
            default:  mul_select = product[2*XLEN-1:XLEN];   // MULH/MULHSU/MULHU
        endcase
    endfunction

endmodule: ooo_mul_unit
