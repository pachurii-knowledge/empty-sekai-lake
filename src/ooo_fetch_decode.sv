`include "ooo_types.vh"
`include "superscalar_types.vh"

`default_nettype none

module ooo_fetch_decode
    import OOO_Types::*;
(
    input  logic             rst_l,
    input  logic             fetch_valid,
    input  logic             instr_mem_excpt,
    input  logic [31:0]      fetch_pc,
    input  logic [3:0][31:0] instr,
    output decode_lane_t     decode_lanes [OOO_WIDTH]
);

    ctrl_signals_t lane_ctrl [OOO_WIDTH];
    logic [1:0] fetch_lane_offset;
    logic [OOO_WIDTH-1:0] prefix_unpredicted_control;
    logic [OOO_WIDTH-1:0][31:0] raw_decode_instr;
    logic [OOO_WIDTH-1:0][31:0] decode_instr;

    assign fetch_lane_offset = fetch_pc[3:2];

    genvar lane;
    generate
        for (lane = 0; lane < OOO_WIDTH; lane += 1) begin : decode_gen
            assign raw_decode_instr[lane] =
                (fetch_valid && !instr_mem_excpt &&
                 (int'(fetch_lane_offset) + lane < OOO_WIDTH)) ?
                instr[int'(fetch_lane_offset) + lane] : 32'h0000_0013;
            assign decode_instr[lane] = prefix_unpredicted_control[lane] ?
                32'h0000_0013 : raw_decode_instr[lane];

            riscv_decode DecodeLane (
                .rst_l,
                .instr(decode_instr[lane]),
                .ctrl_signals(lane_ctrl[lane])
            );
        end
    endgenerate

    always_comb begin
        prefix_unpredicted_control = '0;
        for (int i = 1; i < OOO_WIDTH; i += 1) begin
            prefix_unpredicted_control[i] =
                prefix_unpredicted_control[i - 1] ||
                (raw_decode_instr[i - 1][6:0] == RISCV_ISA::OP_JAL) ||
                (raw_decode_instr[i - 1][6:0] == RISCV_ISA::OP_JALR);
        end
    end

    always_comb begin
        for (int i = 0; i < OOO_WIDTH; i += 1) begin
            decode_lanes[i].valid = fetch_valid && !instr_mem_excpt &&
                (int'(fetch_lane_offset) + i < OOO_WIDTH) &&
                !prefix_unpredicted_control[i];
            decode_lanes[i].kill = 1'b0;
            decode_lanes[i].pc = fetch_pc + (i * 32'd4);
            decode_lanes[i].instr = decode_instr[i];
            decode_lanes[i].ctrl = lane_ctrl[i];
            decode_lanes[i].rs1 = lane_ctrl[i].syscall ? 5'd10 :
                decode_instr[i][19:15];
            decode_lanes[i].rs2 = decode_instr[i][24:20];
            decode_lanes[i].rd = decode_instr[i][11:7];
            decode_lanes[i].imm = immediate_for(lane_ctrl[i].imm_mode, decode_instr[i]);
            decode_lanes[i].uses_rs1 = uses_rs1(decode_instr[i]);
            decode_lanes[i].uses_rs2 = uses_rs2(decode_instr[i]);
        end
    end

    function automatic logic [31:0] immediate_for(imm_mode_t mode,
            logic [31:0] raw_instr);
        unique case (mode)
            IMM_I:  immediate_for = {{21{raw_instr[31]}}, raw_instr[30:20]};
            IMM_S:  immediate_for = {{21{raw_instr[31]}}, raw_instr[30:25],
                                      raw_instr[11:7]};
            IMM_SB: immediate_for = {{20{raw_instr[31]}}, raw_instr[7],
                                      raw_instr[30:25], raw_instr[11:8], 1'b0};
            IMM_U:  immediate_for = {raw_instr[31:12], 12'b0};
            IMM_UJ: immediate_for = {{12{raw_instr[31]}}, raw_instr[19:12],
                                      raw_instr[20], raw_instr[30:21], 1'b0};
            default: immediate_for = 32'b0;
        endcase
    endfunction

    function automatic logic uses_rs1(logic [31:0] raw_instr);
        logic [6:0] opcode;
        opcode = raw_instr[6:0];
        uses_rs1 = (opcode == RISCV_ISA::OP_OP) || (opcode == RISCV_ISA::OP_IMM) ||
            (opcode == RISCV_ISA::OP_LOAD) || (opcode == RISCV_ISA::OP_STORE) ||
            (opcode == RISCV_ISA::OP_BRANCH) || (opcode == RISCV_ISA::OP_JALR) ||
            (opcode == RISCV_ISA::OP_SYSTEM) || (opcode == RISCV_ISA::OP_LOAD_FP) ||
            (opcode == RISCV_ISA::OP_STORE_FP) || (opcode == RISCV_ISA::OP_AMO) ||
            ((opcode == RISCV_ISA::OP_FP) &&
             ((raw_instr[31:27] == 5'b11010) ||
              (raw_instr[31:27] == 5'b11110)));
    endfunction

    function automatic logic uses_rs2(logic [31:0] raw_instr);
        logic [6:0] opcode;
        opcode = raw_instr[6:0];
        uses_rs2 = (opcode == RISCV_ISA::OP_OP) || (opcode == RISCV_ISA::OP_STORE) ||
            (opcode == RISCV_ISA::OP_BRANCH) || (opcode == RISCV_ISA::OP_AMO);
    endfunction

endmodule: ooo_fetch_decode
