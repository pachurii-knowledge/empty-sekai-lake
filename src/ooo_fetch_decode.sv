`include "ooo_types.vh"
`include "superscalar_types.vh"

`default_nettype none

module ooo_fetch_decode
    import OOO_Types::*;
(
    input  logic             rst_l,
    input  logic             fetch_valid,
    input  logic             instr_mem_excpt,
    // Per-lane fetch page/access fault for the fetch group. Bit i corresponds to
    // decode lane i (PC = fetch_pc + 4*i). Translation/execute-permission faults
    // are page-granular (so they set lane 0 only), but PMP can differ per 4-byte
    // word within the 16-byte fetch block, so each lane is checked independently.
    input  logic [OOO_WIDTH-1:0] fetch_fault_lane,
    input  logic [4:0]       fetch_fault_cause,  // EXC_INSTR_PAGE_FAULT/ACCESS
    input  logic [XLEN-1:0]  fetch_pc,
    input  logic [3:0][31:0] instr,
    output decode_lane_t     decode_lanes [OOO_WIDTH]
);

    ctrl_signals_t lane_ctrl [OOO_WIDTH];
    logic [1:0] fetch_lane_offset;
    logic [OOO_WIDTH-1:0] prefix_unpredicted_control;
    logic [OOO_WIDTH-1:0] base_valid;        // lane fetched + not control-squashed
    logic [OOO_WIDTH-1:0] block_fault;       // lane validly faults
    logic [OOO_WIDTH-1:0] prefix_fault;      // some older lane in the group faults
    logic [OOO_WIDTH-1:0][31:0] raw_decode_instr;
    logic [OOO_WIDTH-1:0][31:0] decode_instr;

    assign fetch_lane_offset = fetch_pc[3:2];

    // base_valid / block_fault / prefix_fault depend only on raw_decode_instr and
    // the fault mask, so they are resolved before decode_instr (no comb cycle).
    always_comb begin
        prefix_unpredicted_control = '0;
        for (int i = 1; i < OOO_WIDTH; i += 1) begin
            prefix_unpredicted_control[i] =
                prefix_unpredicted_control[i - 1] ||
                (raw_decode_instr[i - 1][6:0] == RISCV_ISA::OP_JAL) ||
                (raw_decode_instr[i - 1][6:0] == RISCV_ISA::OP_JALR);
        end
        for (int i = 0; i < OOO_WIDTH; i += 1) begin
            base_valid[i] = fetch_valid &&
                (int'(fetch_lane_offset) + i < OOO_WIDTH) &&
                !prefix_unpredicted_control[i];
            block_fault[i] = base_valid[i] && fetch_fault_lane[i];
        end
        prefix_fault = '0;
        for (int i = 1; i < OOO_WIDTH; i += 1)
            prefix_fault[i] = prefix_fault[i - 1] || block_fault[i - 1];
    end

    genvar lane;
    generate
        for (lane = 0; lane < OOO_WIDTH; lane += 1) begin : decode_gen
            assign raw_decode_instr[lane] =
                (fetch_valid && !instr_mem_excpt &&
                 (int'(fetch_lane_offset) + lane < OOO_WIDTH)) ?
                instr[int'(fetch_lane_offset) + lane] : 32'h0000_0013;
            // NOP a lane that is control-squashed, that is the fault carrier, or
            // that follows an older faulting lane in the same group. The fault
            // carrier is re-stamped below to carry the fault to the ALU/commit.
            assign decode_instr[lane] =
                (prefix_unpredicted_control[lane] || block_fault[lane] ||
                 prefix_fault[lane]) ?
                32'h0000_0013 : raw_decode_instr[lane];

            riscv_decode DecodeLane (
                .rst_l,
                .instr(decode_instr[lane]),
                .ctrl_signals(lane_ctrl[lane])
            );
        end
    endgenerate

    always_comb begin
        for (int i = 0; i < OOO_WIDTH; i += 1) begin
            // A lane stays valid until (and including) the first faulting lane;
            // older lanes execute normally, younger lanes are squashed. The fault
            // carrier (block_fault & !prefix_fault) retires as a trapping NOP.
            decode_lanes[i].valid = fetch_valid && !instr_mem_excpt &&
                (int'(fetch_lane_offset) + i < OOO_WIDTH) &&
                !prefix_unpredicted_control[i] && !prefix_fault[i];
            decode_lanes[i].kill = 1'b0;
            decode_lanes[i].pc = fetch_pc + (i * 32'd4);
            decode_lanes[i].instr = decode_instr[i];
            decode_lanes[i].ctrl = lane_ctrl[i];
            decode_lanes[i].ctrl.fetch_fault =
                block_fault[i] && !prefix_fault[i];
            decode_lanes[i].ctrl.fetch_fault_cause = fetch_fault_cause;
            decode_lanes[i].rs1 = lane_ctrl[i].syscall ? 5'd10 :
                decode_instr[i][19:15];
            decode_lanes[i].rs2 = decode_instr[i][24:20];
            decode_lanes[i].rd = decode_instr[i][11:7];
            decode_lanes[i].imm = immediate_for(lane_ctrl[i].imm_mode, decode_instr[i]);
            decode_lanes[i].uses_rs1 = uses_rs1(decode_instr[i]);
            decode_lanes[i].uses_rs2 = uses_rs2(decode_instr[i]);
        end

        // The fault carrier remains valid even under instr_mem_excpt: the
        // discarded instruction bytes do not matter once the fetch has faulted.
        for (int i = 0; i < OOO_WIDTH; i += 1)
            if (block_fault[i] && !prefix_fault[i])
                decode_lanes[i].valid = fetch_valid;
    end

    function automatic logic [XLEN-1:0] immediate_for(imm_mode_t mode,
            logic [31:0] raw_instr);
        unique case (mode)
            IMM_I:  immediate_for = {{(XLEN-11){raw_instr[31]}}, raw_instr[30:20]};
            IMM_S:  immediate_for = {{(XLEN-11){raw_instr[31]}}, raw_instr[30:25],
                                      raw_instr[11:7]};
            IMM_SB: immediate_for = {{(XLEN-12){raw_instr[31]}}, raw_instr[7],
                                      raw_instr[30:25], raw_instr[11:8], 1'b0};
            IMM_U:  immediate_for = {{(XLEN-32){raw_instr[31]}}, raw_instr[31:12], 12'b0};
            IMM_UJ: immediate_for = {{(XLEN-20){raw_instr[31]}}, raw_instr[19:12],
                                      raw_instr[20], raw_instr[30:21], 1'b0};
            default: immediate_for = '0;
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
            // RV64 OP-32 (0x3B) and OP-IMM-32 (0x1B) W-form ops
            (opcode == 7'h3B) || (opcode == 7'h1B) ||
            ((opcode == RISCV_ISA::OP_FP) &&
             ((raw_instr[31:27] == 5'b11010) ||
              (raw_instr[31:27] == 5'b11110)));
    endfunction

    function automatic logic uses_rs2(logic [31:0] raw_instr);
        logic [6:0] opcode;
        opcode = raw_instr[6:0];
        uses_rs2 = (opcode == RISCV_ISA::OP_OP) || (opcode == RISCV_ISA::OP_STORE) ||
            (opcode == RISCV_ISA::OP_BRANCH) || (opcode == RISCV_ISA::OP_AMO) ||
            (opcode == 7'h3B);   // RV64 OP-32 register W-form ops
    endfunction

endmodule: ooo_fetch_decode
