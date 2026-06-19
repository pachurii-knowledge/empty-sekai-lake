`include "ooo_types.vh"

`default_nettype none

module niigo_fp_unit
    import OOO_Types::*;
(
    input wire logic              clk,
    input wire logic              rst_l,
    input wire logic              issue_valid,
    output logic              issue_ready,
    input wire issue_entry_t      issue_entry,
    input wire logic [XLEN-1:0]   rs1_data,
    input wire logic [2:0]        frm,
    input wire branch_mask_t      abort_mask,
    // Precise-trap full flush: drop the pending request, the cvfpu internal
    // pipeline, and the output buffer so no stale writeback lands on a reused
    // active-list id after the pipeline is reset.
    input wire logic              flush,
    input wire logic              writeback_ready,
    output writeback_packet_t writeback
);

    // FB2b wall #2: after the LSQ head-translate pipeline cut, the binding placed
    // path became the CVFPU FMA (ADDMUL) arithmetic -> arbiter (~83 levels, 24
    // CARRY8). The FMA ran as ONE combinational block (PipeRegs=1, PipeConfig
    // BEFORE = a single register in front of the whole mul+add). Split the FMA
    // across 2 internal stages: ADDMUL PipeRegs=2 + PipeConfig DISTRIBUTED (spread
    // the registers THROUGH the unit instead of all before it). Other opgroups stay
    // at 1. FP is infrequent (xv6/Linux are integer-heavy), so the extra FMA
    // latency is a negligible IPC cost. The wrapper's abort-at-output + flush_i
    // (clears all internal stages) are pipeline-depth/structure agnostic.
    localparam fpnew_pkg::fpu_implementation_t NIIGO_CVFPU_IMPL = '{
        PipeRegs:   '{'{default: 2},   // ADDMUL (FMA): 2 distributed stages
                      '{default: 1},   // DIVSQRT
                      '{default: 1},   // NONCOMP
                      '{default: 1}},  // CONV
        UnitTypes:  '{'{default: fpnew_pkg::PARALLEL},
                      '{default: fpnew_pkg::MERGED},
                      '{default: fpnew_pkg::PARALLEL},
                      '{default: fpnew_pkg::MERGED}},
        PipeConfig: fpnew_pkg::DISTRIBUTED
    };

    issue_entry_t req_entry_q;
    logic [XLEN-1:0] req_int_src_q;
    logic         req_valid_q;
    logic         req_is_simple;
    logic         req_aborted;

    logic [2:0][63:0]         fpnew_operands;
    fpnew_pkg::roundmode_e    fpnew_rnd_mode;
    fpnew_pkg::operation_e    fpnew_op;
    logic                     fpnew_op_mod;
    fpnew_pkg::fp_format_e    fpnew_src_fmt;
    fpnew_pkg::fp_format_e    fpnew_dst_fmt;
    fpnew_pkg::int_format_e   fpnew_int_fmt;
    logic                     fpnew_in_valid;
    logic                     fpnew_in_ready;
    logic [63:0]              fpnew_result;
    fpnew_pkg::status_t       fpnew_status;
    issue_entry_t             fpnew_tag;
    logic                     fpnew_out_valid;
    logic                     fpnew_out_ready;
    logic                     fpnew_output_aborted;
    logic                     fpnew_buffer_valid_q;
    logic [63:0]              fpnew_buffer_result_q;
    fpnew_pkg::status_t       fpnew_buffer_status_q;
    issue_entry_t             fpnew_buffer_tag_q;
    logic                     fpnew_buffer_aborted;
    writeback_packet_t        simple_writeback;
    writeback_packet_t        fpnew_writeback;
    logic                     simple_writeback_valid;

    assign issue_ready = !req_valid_q;
    assign req_is_simple = is_simple_op(req_entry_q);
    assign req_aborted = req_valid_q && ((req_entry_q.branch_mask & abort_mask) != '0);
    assign simple_writeback_valid = req_valid_q && req_is_simple && !req_aborted;

    assign fpnew_in_valid = req_valid_q && !req_is_simple && !req_aborted;
    assign fpnew_output_aborted = fpnew_out_valid &&
        ((fpnew_tag.branch_mask & abort_mask) != '0);
    assign fpnew_buffer_aborted = fpnew_buffer_valid_q &&
        ((fpnew_buffer_tag_q.branch_mask & abort_mask) != '0);
    assign fpnew_out_ready = !fpnew_buffer_valid_q || fpnew_buffer_aborted ||
        (writeback_ready && !simple_writeback_valid);

    fpnew_top #(
`ifdef RV64
        .Features       ( fpnew_pkg::RV64D          ),
`else
        .Features       ( fpnew_pkg::RV32D          ),
`endif
        .Implementation ( NIIGO_CVFPU_IMPL          ),
        .DivSqrtSel     ( fpnew_pkg::THMULTI        ),
        .TagType        ( issue_entry_t             )
    ) cvfpu (
        .clk_i         ( clk                ),
        .rst_ni        ( rst_l              ),
        .operands_i    ( fpnew_operands     ),
        .rnd_mode_i    ( fpnew_rnd_mode     ),
        .op_i          ( fpnew_op           ),
        .op_mod_i      ( fpnew_op_mod       ),
        .src_fmt_i     ( fpnew_src_fmt      ),
        .dst_fmt_i     ( fpnew_dst_fmt      ),
        .int_fmt_i     ( fpnew_int_fmt      ),
        .vectorial_op_i( 1'b0               ),
        .tag_i         ( req_entry_q        ),
        .simd_mask_i   ( '1                 ),
        .in_valid_i    ( fpnew_in_valid     ),
        .in_ready_o    ( fpnew_in_ready     ),
        .flush_i       ( flush              ),
        .result_o      ( fpnew_result       ),
        .status_o      ( fpnew_status       ),
        .tag_o         ( fpnew_tag          ),
        .out_valid_o   ( fpnew_out_valid    ),
        .out_ready_i   ( fpnew_out_ready    ),
        .busy_o        ( /* unused */       ),
        .early_valid_o ( /* unused */       )
    );

    always_comb begin
        simple_writeback = packet_for_simple(req_entry_q, req_int_src_q);
        fpnew_writeback = packet_for_fpnew(fpnew_buffer_tag_q,
            fpnew_buffer_result_q, fpnew_buffer_status_q);
        writeback = '0;
        if (flush) begin
            writeback = '0;
        end else if (simple_writeback_valid) begin
            writeback = simple_writeback;
        end else if (fpnew_buffer_valid_q && !fpnew_buffer_aborted) begin
            writeback = fpnew_writeback;
        end
    end

    always_ff @(posedge clk or negedge rst_l) begin
        if (!rst_l) begin
            req_valid_q <= 1'b0;
            req_entry_q <= '0;
            req_int_src_q <= '0;
            fpnew_buffer_valid_q <= 1'b0;
            fpnew_buffer_result_q <= '0;
            fpnew_buffer_status_q <= '0;
            fpnew_buffer_tag_q <= '0;
        end else if (flush) begin
            req_valid_q <= 1'b0;
            fpnew_buffer_valid_q <= 1'b0;
        end else begin
            if (fpnew_buffer_valid_q &&
                    (fpnew_buffer_aborted ||
                     (writeback_ready && !simple_writeback_valid))) begin
                fpnew_buffer_valid_q <= 1'b0;
            end
            if (fpnew_out_valid && fpnew_out_ready && !fpnew_output_aborted) begin
                fpnew_buffer_valid_q <= 1'b1;
                fpnew_buffer_result_q <= fpnew_result;
                fpnew_buffer_status_q <= fpnew_status;
                fpnew_buffer_tag_q <= fpnew_tag;
            end

            if (req_valid_q) begin
                if (req_aborted ||
                        (req_is_simple && writeback_ready) ||
                        (!req_is_simple && fpnew_in_ready)) begin
                    req_valid_q <= 1'b0;
                end
            end

            if (issue_valid && issue_ready &&
                    ((issue_entry.branch_mask & abort_mask) == '0)) begin
                req_valid_q <= 1'b1;
                req_entry_q <= issue_entry;
                req_int_src_q <= rs1_data;
            end

        end
    end

    always_comb begin
        fpnew_operands = cvfpu_operands(req_entry_q, req_int_src_q);
        fpnew_rnd_mode = cvfpu_round_mode(req_entry_q, frm);
        fpnew_op = cvfpu_operation(req_entry_q.ctrl.fp_op);
        fpnew_op_mod = cvfpu_op_mod(req_entry_q.ctrl.fp_op);
        fpnew_src_fmt = cvfpu_src_format(req_entry_q);
        fpnew_dst_fmt = cvfpu_dst_format(req_entry_q);
        fpnew_int_fmt = is_l_cvt(req_entry_q.ctrl.fp_op) ?
            fpnew_pkg::INT64 : fpnew_pkg::INT32;
    end

    // RV64 L-form conversions: 64-bit integer operand/result.
    function automatic logic is_l_cvt(input fp_op_t op);
        is_l_cvt = (op == FP_CVT_L) || (op == FP_CVT_LU) ||
            (op == FP_CVT_F_L) || (op == FP_CVT_F_LU);
    endfunction

    function automatic logic is_simple_op(input issue_entry_t entry);
        is_simple_op = (entry.ctrl.fp_op == FP_MV_X) ||
            (entry.ctrl.fp_op == FP_MV_F_X);
    endfunction

    function automatic logic flags_write_valid(input issue_entry_t entry);
        flags_write_valid = entry.ctrl.exec_class == EXEC_FP &&
            !entry.ctrl.memRead && !entry.ctrl.memWrite &&
            (entry.ctrl.fp_op != FP_MV_X) &&
            (entry.ctrl.fp_op != FP_MV_F_X) &&
            (entry.ctrl.fp_op != FP_SGNJ) &&
            (entry.ctrl.fp_op != FP_SGNJN) &&
            (entry.ctrl.fp_op != FP_SGNJX) &&
            (entry.ctrl.fp_op != FP_CLASS);
    endfunction

    function automatic fpnew_pkg::roundmode_e cvfpu_round_mode(
            input issue_entry_t entry, input logic [2:0] csr_frm);
        logic [2:0] rm;
        rm = (entry.instr[14:12] == 3'b111) ? csr_frm : entry.instr[14:12];
        unique case (entry.ctrl.fp_op)
            FP_SGNJ:  cvfpu_round_mode = fpnew_pkg::RNE;
            FP_SGNJN: cvfpu_round_mode = fpnew_pkg::RTZ;
            FP_SGNJX: cvfpu_round_mode = fpnew_pkg::RDN;
            FP_MIN:   cvfpu_round_mode = fpnew_pkg::RNE;
            FP_MAX:   cvfpu_round_mode = fpnew_pkg::RTZ;
            FP_LE:    cvfpu_round_mode = fpnew_pkg::RNE;
            FP_LT:    cvfpu_round_mode = fpnew_pkg::RTZ;
            FP_EQ:    cvfpu_round_mode = fpnew_pkg::RDN;
            default: begin
                unique case (rm)
                    3'b000: cvfpu_round_mode = fpnew_pkg::RNE;
                    3'b001: cvfpu_round_mode = fpnew_pkg::RTZ;
                    3'b010: cvfpu_round_mode = fpnew_pkg::RDN;
                    3'b011: cvfpu_round_mode = fpnew_pkg::RUP;
                    3'b100: cvfpu_round_mode = fpnew_pkg::RMM;
                    default: cvfpu_round_mode = fpnew_pkg::RNE;
                endcase
            end
        endcase
    endfunction

    function automatic fpnew_pkg::operation_e cvfpu_operation(input fp_op_t op);
        unique case (op)
            FP_ADD,
            FP_SUB:     cvfpu_operation = fpnew_pkg::ADD;
            FP_MUL:     cvfpu_operation = fpnew_pkg::MUL;
            FP_DIV:     cvfpu_operation = fpnew_pkg::DIV;
            FP_SQRT:    cvfpu_operation = fpnew_pkg::SQRT;
            FP_SGNJ,
            FP_SGNJN,
            FP_SGNJX:   cvfpu_operation = fpnew_pkg::SGNJ;
            FP_MIN,
            FP_MAX:     cvfpu_operation = fpnew_pkg::MINMAX;
            FP_EQ,
            FP_LT,
            FP_LE:      cvfpu_operation = fpnew_pkg::CMP;
            FP_CLASS:   cvfpu_operation = fpnew_pkg::CLASSIFY;
            FP_CVT_W,
            FP_CVT_WU,
            FP_CVT_L,
            FP_CVT_LU:  cvfpu_operation = fpnew_pkg::F2I;
            FP_CVT_F_W,
            FP_CVT_F_WU,
            FP_CVT_F_L,
            FP_CVT_F_LU: cvfpu_operation = fpnew_pkg::I2F;
            FP_CVT_F_F: cvfpu_operation = fpnew_pkg::F2F;
            FP_MADD,
            FP_MSUB:    cvfpu_operation = fpnew_pkg::FMADD;
            FP_NMSUB,
            FP_NMADD:   cvfpu_operation = fpnew_pkg::FNMSUB;
            default:    cvfpu_operation = fpnew_pkg::ADD;
        endcase
    endfunction

    function automatic logic cvfpu_op_mod(input fp_op_t op);
        unique case (op)
            FP_SUB,
            FP_MSUB,
            FP_NMADD,
            FP_CVT_WU,
            FP_CVT_LU,
            FP_CVT_F_WU,
            FP_CVT_F_LU: cvfpu_op_mod = 1'b1;
            default:     cvfpu_op_mod = 1'b0;
        endcase
    endfunction

    function automatic fpnew_pkg::fp_format_e cvfpu_dst_format(
            input issue_entry_t entry);
        cvfpu_dst_format = entry.ctrl.fp_double ? fpnew_pkg::FP64 :
            fpnew_pkg::FP32;
    endfunction

    function automatic fpnew_pkg::fp_format_e cvfpu_src_format(
            input issue_entry_t entry);
        if (entry.ctrl.fp_op == FP_CVT_F_F) begin
            cvfpu_src_format = entry.ctrl.fp_double ? fpnew_pkg::FP32 :
                fpnew_pkg::FP64;
        end else begin
            cvfpu_src_format = cvfpu_dst_format(entry);
        end
    endfunction

    function automatic logic [2:0][63:0] cvfpu_operands(
            input issue_entry_t entry, input logic [XLEN-1:0] int_src);
        cvfpu_operands = '0;
        unique case (entry.ctrl.fp_op)
            FP_ADD,
            FP_SUB: begin
                cvfpu_operands[1] = entry.fp_src1_data;
                cvfpu_operands[2] = entry.fp_src2_data;
            end
            FP_MUL,
            FP_MADD,
            FP_MSUB,
            FP_NMSUB,
            FP_NMADD: begin
                cvfpu_operands[0] = entry.fp_src1_data;
                cvfpu_operands[1] = entry.fp_src2_data;
                cvfpu_operands[2] = entry.fp_src3_data;
            end
            FP_CVT_F_W,
            FP_CVT_F_WU: begin
                cvfpu_operands[0] = {32'b0, int_src[31:0]};
            end
            FP_CVT_F_L,
            FP_CVT_F_LU: begin
                cvfpu_operands[0] = 64'(int_src);
            end
            default: begin
                cvfpu_operands[0] = entry.fp_src1_data;
                cvfpu_operands[1] = entry.fp_src2_data;
            end
        endcase
    endfunction

    function automatic writeback_packet_t base_packet(input issue_entry_t entry);
        writeback_packet_t packet;
        packet = '0;
        packet.valid = 1'b1;
        packet.active_id = entry.active_id;
        packet.pc = entry.pc;
        packet.instr = entry.instr;
        packet.prd = entry.prd;
        packet.has_dest = entry.has_dest;
        packet.branch_mask = entry.branch_mask;
        packet.fp_write = entry.ctrl.fp_writes_fpr &&
            !entry.ctrl.memRead && !entry.ctrl.memWrite;
        packet.fp_rd = entry.fp_rd;
        packet.fp_fflags_valid = flags_write_valid(entry);
        return packet;
    endfunction

    function automatic writeback_packet_t packet_for_simple(
            input issue_entry_t entry, input logic [XLEN-1:0] int_src);
        writeback_packet_t packet;
        packet = base_packet(entry);
        unique case (entry.ctrl.fp_op)
            FP_MV_X: begin
                // FMV.X.D moves all 64 bits; FMV.X.W sign-extends bit 31
                // (identity at XLEN=32).
                packet.data = entry.ctrl.fp_double ?
                    XLEN'(entry.fp_src1_data) :
                    XLEN'($signed(entry.fp_src1_data[31:0]));
                packet.fp_write = 1'b0;
            end
            FP_MV_F_X: begin
                // FMV.D.X moves all 64 bits; FMV.W.X NaN-boxes the low word.
                packet.fp_data = entry.ctrl.fp_double ? 64'(int_src) :
                    {32'hffff_ffff, int_src[31:0]};
            end
            default: begin
            end
        endcase
        return packet;
    endfunction

    function automatic writeback_packet_t packet_for_fpnew(input issue_entry_t entry,
            input logic [63:0] result, input fpnew_pkg::status_t status);
        writeback_packet_t packet;
        packet = base_packet(entry);
        // 32-bit integer results (FCVT.W/WU, compares, classify) sign-extend
        // to XLEN per RV64; the L-form conversions return the full 64 bits.
        packet.data = is_l_cvt(entry.ctrl.fp_op) ?
            XLEN'(result) : XLEN'($signed(result[31:0]));
        packet.fp_data = result;
        packet.fp_fflags = {status.NV, status.DZ, status.OF, status.UF, status.NX};
        return packet;
    endfunction

endmodule: niigo_fp_unit
