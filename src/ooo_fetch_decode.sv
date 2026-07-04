`include "ooo_types.vh"
`include "superscalar_types.vh"

`default_nettype none

module ooo_fetch_decode
    import OOO_Types::*;
(
    input wire logic             rst_l,
    input wire logic             fetch_valid,
    input wire logic             instr_mem_excpt,
    // Per-lane fetch page/access fault for the fetch group. Bit i corresponds to
    // decode lane i (PC = fetch_pc + 4*i). Translation/execute-permission faults
    // are page-granular (so they set lane 0 only), but PMP can differ per 4-byte
    // word within the 16-byte fetch block, so each lane is checked independently.
    input wire logic [OOO_WIDTH-1:0] fetch_fault_lane,
    input wire logic [4:0]       fetch_fault_cause,  // EXC_INSTR_PAGE_FAULT/ACCESS
    input wire logic [XLEN-1:0]  fetch_pc,
    input wire logic [3:0][31:0] instr,
`ifdef RVC
    // RV64C: pre-aligned + expanded lanes from rvc_realign (2-wide). Under -DRVC
    // these fully replace the fixed 4-byte window: instr/fetch_pc/fetch_fault_lane
    // are ignored and the realigner supplies each lane's PC, canonical 32-bit
    // encoding, length flag, per-instruction fetch fault, and raw parcel.
    input wire logic [1:0]            rvc_valid,
    input wire logic [1:0][XLEN-1:0]  rvc_pc,
    input wire logic [1:0][31:0]      rvc_instr,
    input wire logic [1:0]            rvc_is_compressed,
    input wire logic [1:0]            rvc_fetch_fault,
    input wire logic [1:0]            rvc_fetch_fault_hi,
    input wire logic [1:0][4:0]       rvc_fault_cause,
    input wire logic [1:0][15:0]      rvc_parcel,
`endif
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

    // Per-lane sources, `ifdef-selected from the RVC realigner or the fixed
    // 4-byte fetch window. All downstream logic (control-squash chain, fault
    // chain, decode, immediate/rs/rd extraction) is shared and identical.
    logic [OOO_WIDTH-1:0]           lane_present;      // lane holds a real slot
    logic [OOO_WIDTH-1:0][XLEN-1:0] lane_pc;
    logic [OOO_WIDTH-1:0]           lane_fault;        // this lane's fetch fault
    logic [OOO_WIDTH-1:0][4:0]      lane_fault_cause_v;
`ifdef RVC
    logic [OOO_WIDTH-1:0]           lane_is_comp;
    logic [OOO_WIDTH-1:0]           lane_fault_hi;
    logic [OOO_WIDTH-1:0][15:0]     lane_parcel;
`endif

    assign fetch_lane_offset = fetch_pc[3:2];

    always_comb begin
        for (int i = 0; i < OOO_WIDTH; i += 1) begin
`ifdef RVC
            // Only lanes 0/1 exist (the realigner is two-wide); 2/3 are forced
            // invalid NOPs so the control/fault chains and dispatch accounting
            // stay well-defined against the 4-wide backend.
            if (i < 2) begin
                lane_present[i]       = rvc_valid[i];
                raw_decode_instr[i]   = rvc_valid[i] ? rvc_instr[i] : 32'h0000_0013;
                lane_pc[i]            = rvc_pc[i];
                lane_fault[i]         = rvc_fetch_fault[i];
                lane_fault_cause_v[i] = rvc_fault_cause[i];
                lane_is_comp[i]       = rvc_is_compressed[i];
                lane_fault_hi[i]      = rvc_fetch_fault_hi[i];
                lane_parcel[i]        = rvc_parcel[i];
            end else begin
                lane_present[i]       = 1'b0;
                raw_decode_instr[i]   = 32'h0000_0013;
                lane_pc[i]            = '0;
                lane_fault[i]         = 1'b0;
                lane_fault_cause_v[i] = 5'd0;
                lane_is_comp[i]       = 1'b0;
                lane_fault_hi[i]      = 1'b0;
                lane_parcel[i]        = 16'h0000;
            end
`else
            lane_present[i] = fetch_valid &&
                (int'(fetch_lane_offset) + i < OOO_WIDTH);
            raw_decode_instr[i] =
                (fetch_valid && !instr_mem_excpt &&
                 (int'(fetch_lane_offset) + i < OOO_WIDTH)) ?
                instr[int'(fetch_lane_offset) + i] : 32'h0000_0013;
            lane_pc[i]            = fetch_pc + (i * 32'd4);
            lane_fault[i]         = fetch_fault_lane[i];
            lane_fault_cause_v[i] = fetch_fault_cause;
`endif
        end

        // A JAL/JALR in an older lane squashes the younger lanes of the same
        // fetch group (their PCs are wrong-path). The expander turns c.j/c.jr/
        // c.jalr into JAL/JALR, so this chain is C-transparent.
        prefix_unpredicted_control = '0;
        for (int i = 1; i < OOO_WIDTH; i += 1) begin
            prefix_unpredicted_control[i] =
                prefix_unpredicted_control[i - 1] ||
                (raw_decode_instr[i - 1][6:0] == RISCV_ISA::OP_JAL) ||
                (raw_decode_instr[i - 1][6:0] == RISCV_ISA::OP_JALR);
        end
        for (int i = 0; i < OOO_WIDTH; i += 1) begin
            base_valid[i] = fetch_valid && lane_present[i] &&
                !prefix_unpredicted_control[i];
            block_fault[i] = base_valid[i] && lane_fault[i];
        end
        prefix_fault = '0;
        for (int i = 1; i < OOO_WIDTH; i += 1)
            prefix_fault[i] = prefix_fault[i - 1] || block_fault[i - 1];
    end

    genvar lane;
    generate
        for (lane = 0; lane < OOO_WIDTH; lane += 1) begin : decode_gen
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
                lane_present[i] &&
                !prefix_unpredicted_control[i] && !prefix_fault[i];
            decode_lanes[i].kill = 1'b0;
            decode_lanes[i].pc = lane_pc[i];
            decode_lanes[i].instr = decode_instr[i];
            decode_lanes[i].ctrl = lane_ctrl[i];
            decode_lanes[i].ctrl.fetch_fault =
                block_fault[i] && !prefix_fault[i];
            decode_lanes[i].ctrl.fetch_fault_cause = lane_fault_cause_v[i];
`ifdef RVC
            decode_lanes[i].ctrl.is_compressed = lane_is_comp[i];
            decode_lanes[i].ctrl.fetch_fault_hi =
                lane_fault_hi[i] && block_fault[i] && !prefix_fault[i];
            decode_lanes[i].ctrl.rvc_parcel = lane_parcel[i];
`endif
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
