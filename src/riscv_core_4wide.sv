/**
 * Conservative 4-wide in-order RV32I core for Phase 2 bring-up.
 *
 * This core deliberately keeps memory single-ported and commits only a valid
 * prefix of each fetch bundle. Independent ALU/control instructions can commit
 * four-wide; loads/stores serialize through the existing Lab 4 memory port.
 */

`include "riscv_abi.vh"
`include "riscv_isa.vh"
`include "riscv_uarch.vh"
`include "memory_segments.vh"
`include "internal_defines.vh"
`include "superscalar_types.vh"

`default_nettype none

module riscv_core_4wide (
    input  logic             clk, rst_l, instr_mem_excpt, data_mem_excpt,
    input  logic [3:0][31:0] instr, data_load,
    input  logic [29:0]      data_load_addr,
    input  logic             data_load_valid,
    output logic             data_load_en, halted,
    output logic [ 3:0]      data_store_mask,
    output logic [29:0]      instr_addr, data_addr,
    output logic             instr_stall, data_stall,
    output logic [31:0]      data_store
);

    import RISCV_ABI::ECALL_ARG_HALT;
    import RISCV_ISA::*;
    import MemorySegments::USER_TEXT_START;

    localparam int WAYS = 4;
    localparam logic [31:0] NOP = 32'h0000_0013;

    logic [31:0] pc_q, pc_next;
    logic halted_q, halt_pending_q, halt_pending_next;
    logic load_pending_q, load_pending_next;
    logic [4:0] load_rd_q, load_rd_next;
    ldst_mode_t load_mode_q, load_mode_next;
    logic [1:0] load_byte_q, load_byte_next;

    logic [1:0][31:0] fetch_pc_pipe_q, fetch_pc_pipe_next;
    logic [1:0] fetch_valid_pipe_q, fetch_valid_pipe_next;
    logic flush_fetch_next;
    logic stall_core;

    fetch_lane_t fetch_lanes[WAYS];
    decode_lane_t decode_lanes[WAYS];
    ctrl_signals_t lane_ctrl[WAYS];
    logic [WAYS-1:0][31:0] decode_instr;
    logic [1:0] fetch_lane_offset;
    logic [WAYS-1:0][4:0] rf_rs1, rf_rs2, rf_rd;
    logic [WAYS-1:0][31:0] rf_rs1_data, rf_rs2_data, rf_rd_data;
    logic [WAYS-1:0] rf_rd_we;

    logic [WAYS-1:0] issue_mask;
    logic [WAYS-1:0] lane_writes;
    logic [WAYS-1:0][31:0] lane_results;
    logic [WAYS-1:0][31:0] lane_rs1_value, lane_rs2_value;
    logic [WAYS-1:0] lane_redirects;
    logic [WAYS-1:0][31:0] lane_redirect_targets;
    logic [WAYS-1:0] lane_halts;
    logic [WAYS-1:0] lane_exceptions;
    logic [WAYS-1:0] lane_is_memory;
    logic [WAYS-1:0] raw_hazard_lane;
    logic [WAYS-1:0] waw_hazard_lane;
    logic [WAYS-1:0] structural_hazard_lane;
    logic stop_prefix;
    logic memory_seen;
    logic redirect_seen;
    logic replay_seen;
    logic [31:0] redirect_target;
    logic [31:0] replay_target;
    logic [31:0] sequential_next_pc;
    logic [31:0] load_data_formatted;

    assign halted = halted_q;
    assign data_stall = 1'b0;
    assign stall_core = load_pending_q && !data_load_valid;
    assign instr_stall = !rst_l || stall_core || halt_pending_q || halted_q;
    assign instr_addr = {pc_q[31:4], 2'b00};
    assign fetch_lane_offset = fetch_pc_pipe_q[1][3:2];
    assign sequential_next_pc = {pc_q[31:4], 4'b0} + 32'd16;

    register_file #(.WAYS(WAYS), .FORWARD(0)) RF (
        .clk,
        .rst_l,
        .halted,
        .rd_we  (rf_rd_we),
        .rs1    (rf_rs1),
        .rs2    (rf_rs2),
        .rd     (rf_rd),
        .rd_data(rf_rd_data),
        .rs1_data(rf_rs1_data),
        .rs2_data(rf_rs2_data)
    );

    genvar lane;
    generate
        for (lane = 0; lane < WAYS; lane += 1) begin : decode_gen
            riscv_decode DecodeLane (
                .rst_l,
                .instr(decode_instr[lane]),
                .ctrl_signals(lane_ctrl[lane])
            );
        end
    endgenerate

    generate
        for (lane = 0; lane < WAYS; lane += 1) begin : decode_input_gen
            assign decode_instr[lane] = (fetch_valid_pipe_q[1] && !instr_mem_excpt &&
                    (int'(fetch_lane_offset) + lane < WAYS)) ?
                instr[int'(fetch_lane_offset) + lane] : NOP;
        end
    endgenerate

    always_comb begin
        for (int i = 0; i < WAYS; i += 1) begin
            fetch_lanes[i].valid = fetch_valid_pipe_q[1] && !instr_mem_excpt &&
                (int'(fetch_lane_offset) + i < WAYS);
            fetch_lanes[i].kill = 1'b0;
            fetch_lanes[i].pc = fetch_pc_pipe_q[1] + (i * 32'd4);
            fetch_lanes[i].instr = decode_instr[i];

            decode_lanes[i].valid = fetch_lanes[i].valid;
            decode_lanes[i].kill = fetch_lanes[i].kill;
            decode_lanes[i].pc = fetch_lanes[i].pc;
            decode_lanes[i].instr = fetch_lanes[i].instr;
            decode_lanes[i].ctrl = lane_ctrl[i];
            decode_lanes[i].rs1 = lane_ctrl[i].syscall ? 5'd10 :
                fetch_lanes[i].instr[19:15];
            decode_lanes[i].rs2 = fetch_lanes[i].instr[24:20];
            decode_lanes[i].rd = fetch_lanes[i].instr[11:7];
            decode_lanes[i].imm = immediate_for(lane_ctrl[i].imm_mode,
                fetch_lanes[i].instr);
            decode_lanes[i].uses_rs1 = uses_rs1(fetch_lanes[i].instr[6:0]);
            decode_lanes[i].uses_rs2 = uses_rs2(fetch_lanes[i].instr[6:0]);

            rf_rs1[i] = decode_lanes[i].rs1;
            rf_rs2[i] = decode_lanes[i].rs2;
        end
    end

    always_comb begin
        issue_mask = '0;
        lane_writes = '0;
        lane_results = '0;
        lane_rs1_value = '0;
        lane_rs2_value = '0;
        lane_redirects = '0;
        lane_redirect_targets = '0;
        lane_halts = '0;
        lane_exceptions = '0;
        lane_is_memory = '0;
        raw_hazard_lane = '0;
        waw_hazard_lane = '0;
        structural_hazard_lane = '0;
        stop_prefix = 1'b0;
        memory_seen = 1'b0;
        redirect_seen = 1'b0;
        replay_seen = 1'b0;
        redirect_target = sequential_next_pc;
        replay_target = sequential_next_pc;
        rf_rd = '0;
        rf_rd_data = '0;
        rf_rd_we = '0;
        pc_next = pc_q;
        halt_pending_next = halt_pending_q;
        load_pending_next = load_pending_q;
        load_rd_next = load_rd_q;
        load_mode_next = load_mode_q;
        load_byte_next = load_byte_q;
        data_load_en = 1'b0;
        data_store_mask = 4'b0;
        data_store = 32'b0;
        data_addr = 30'b0;
        flush_fetch_next = 1'b0;
        fetch_pc_pipe_next = fetch_pc_pipe_q;
        fetch_valid_pipe_next = fetch_valid_pipe_q;

        if (halt_pending_q) begin
            halt_pending_next = 1'b0;
        end else if (load_pending_q) begin
            data_addr = data_load_addr;
            if (data_load_valid) begin
                rf_rd_we[0] = (load_rd_q != 5'd0);
                rf_rd[0] = load_rd_q;
                rf_rd_data[0] = load_data_formatted;
                load_pending_next = 1'b0;
            end
        end else if (!halted_q) begin
            pc_next = sequential_next_pc;

            for (int i = 0; i < WAYS; i += 1) begin
                lane_is_memory[i] = decode_lanes[i].ctrl.memRead ||
                    decode_lanes[i].ctrl.memWrite;
                raw_hazard_lane[i] = has_raw_hazard(i);
                waw_hazard_lane[i] = has_waw_hazard(i);
                structural_hazard_lane[i] = lane_is_memory[i] &&
                    (memory_seen || i != 0);

                if (decode_lanes[i].valid && !stop_prefix &&
                        !raw_hazard_lane[i] && !waw_hazard_lane[i] &&
                        !structural_hazard_lane[i]) begin
                    issue_mask[i] = 1'b1;
                    lane_rs1_value[i] = forwarded_operand(i, decode_lanes[i].rs1,
                        rf_rs1_data[i]);
                    lane_rs2_value[i] = forwarded_operand(i, decode_lanes[i].rs2,
                        rf_rs2_data[i]);
                    execute_lane(i);

                    if (lane_is_memory[i]) begin
                        memory_seen = 1'b1;
                        stop_prefix = 1'b1;
                        if (i < WAYS - 1) begin
                            replay_seen = 1'b1;
                            replay_target = decode_lanes[i + 1].pc;
                        end
                    end
                    if (lane_redirects[i]) begin
                        redirect_seen = 1'b1;
                        redirect_target = lane_redirect_targets[i];
                        stop_prefix = 1'b1;
                    end
                    if (lane_halts[i] || lane_exceptions[i]) begin
                        stop_prefix = 1'b1;
                    end
                end else if (decode_lanes[i].valid) begin
                    stop_prefix = 1'b1;
                    if (!replay_seen) begin
                        replay_seen = 1'b1;
                        replay_target = decode_lanes[i].pc;
                    end
                end
            end

            for (int i = 0; i < WAYS; i += 1) begin
                rf_rd_we[i] = lane_writes[i] && issue_mask[i];
                rf_rd[i] = decode_lanes[i].rd;
                rf_rd_data[i] = lane_results[i];
            end

            for (int i = 0; i < WAYS; i += 1) begin
                if (issue_mask[i] && (lane_halts[i] || lane_exceptions[i])) begin
                    halt_pending_next = 1'b1;
                end
            end

            if (redirect_seen) begin
                pc_next = redirect_target;
                flush_fetch_next = 1'b1;
            end else if (replay_seen) begin
                pc_next = replay_target;
                flush_fetch_next = 1'b1;
            end

            if (!fetch_valid_pipe_q[1] || (|issue_mask)) begin
                fetch_pc_pipe_next = {fetch_pc_pipe_q[0], pc_q};
                fetch_valid_pipe_next = {fetch_valid_pipe_q[0], 1'b1};
            end

            if (flush_fetch_next) begin
                fetch_valid_pipe_next = 2'b00;
                fetch_pc_pipe_next = '0;
            end
        end
    end

    riscv_load_unit LoadFormat (
        .data_load(data_load[0]),
        .data_byte(load_byte_q),
        .ldst_mode(load_mode_q),
        .ld_out(load_data_formatted)
    );

    always_ff @(posedge clk, negedge rst_l) begin
        if (!rst_l) begin
            pc_q <= USER_TEXT_START;
            halted_q <= 1'b0;
            halt_pending_q <= 1'b0;
            load_pending_q <= 1'b0;
            load_rd_q <= 5'b0;
            load_mode_q <= LDST_W;
            load_byte_q <= 2'b0;
            fetch_pc_pipe_q <= '0;
            fetch_valid_pipe_q <= 2'b00;
        end else begin
            if (halt_pending_q) begin
                halted_q <= 1'b1;
            end
            pc_q <= pc_next;
            halt_pending_q <= halt_pending_next;
            load_pending_q <= load_pending_next;
            load_rd_q <= load_rd_next;
            load_mode_q <= load_mode_next;
            load_byte_q <= load_byte_next;
            fetch_pc_pipe_q <= fetch_pc_pipe_next;
            fetch_valid_pipe_q <= fetch_valid_pipe_next;
        end
    end

    function automatic logic [31:0] immediate_for(imm_mode_t mode,
            logic [31:0] raw_instr);
        case (mode)
            IMM_I:  immediate_for = {{21{raw_instr[31]}}, raw_instr[30:20]};
            IMM_S:  immediate_for = {{20{raw_instr[31]}}, raw_instr[31:25],
                raw_instr[11:7]};
            IMM_SB: immediate_for = {{20{raw_instr[31]}}, raw_instr[7],
                raw_instr[30:25], raw_instr[11:8], 1'b0};
            IMM_U:  immediate_for = {raw_instr[31:12], 12'b0};
            IMM_UJ: immediate_for = {{13{raw_instr[31]}}, raw_instr[19:12],
                raw_instr[20], raw_instr[30:21], 1'b0};
            default: immediate_for = 32'b0;
        endcase
    endfunction

    function automatic logic uses_rs1(logic [6:0] opcode);
        uses_rs1 = (opcode == OP_OP) || (opcode == OP_IMM) ||
            (opcode == OP_LOAD) || (opcode == OP_STORE) ||
            (opcode == OP_BRANCH) || (opcode == OP_JALR);
    endfunction

    function automatic logic uses_rs2(logic [6:0] opcode);
        uses_rs2 = (opcode == OP_OP) || (opcode == OP_STORE) ||
            (opcode == OP_BRANCH);
    endfunction

    function automatic logic has_raw_hazard(int lane_idx);
        has_raw_hazard = 1'b0;
        for (int j = 0; j < lane_idx; j += 1) begin
            if (issue_mask[j] && decode_lanes[j].ctrl.memRead &&
                    (decode_lanes[j].rd != 5'd0) &&
                    ((decode_lanes[lane_idx].uses_rs1 &&
                        decode_lanes[lane_idx].rs1 == decode_lanes[j].rd) ||
                     (decode_lanes[lane_idx].uses_rs2 &&
                        decode_lanes[lane_idx].rs2 == decode_lanes[j].rd))) begin
                has_raw_hazard = 1'b1;
            end
        end
    endfunction

    function automatic logic has_waw_hazard(int lane_idx);
        has_waw_hazard = 1'b0;
    endfunction

    function automatic logic [31:0] forwarded_operand(int lane_idx,
            logic [4:0] src, logic [31:0] rf_value);
        forwarded_operand = rf_value;
        for (int j = 0; j < lane_idx; j += 1) begin
            if (issue_mask[j] && lane_writes[j] && (decode_lanes[j].rd != 5'd0) &&
                    (decode_lanes[j].rd == src)) begin
                forwarded_operand = lane_results[j];
            end
        end
    endfunction

    task automatic execute_lane(input int lane_idx);
        automatic logic [31:0] src1;
        automatic logic [31:0] src2;
        automatic logic [31:0] alu_out;
        automatic logic branch_taken;

        src1 = decode_lanes[lane_idx].ctrl.usePC ? decode_lanes[lane_idx].pc :
            lane_rs1_value[lane_idx];
        src2 = decode_lanes[lane_idx].ctrl.useImm ? decode_lanes[lane_idx].imm :
            lane_rs2_value[lane_idx];
        alu_out = alu_result(src1, src2, decode_lanes[lane_idx].ctrl.alu_op);
        branch_taken = branch_result(lane_rs1_value[lane_idx],
            lane_rs2_value[lane_idx], decode_lanes[lane_idx].ctrl.btype);

        lane_exceptions[lane_idx] = decode_lanes[lane_idx].ctrl.illegal_instr ||
            instr_mem_excpt;
        lane_halts[lane_idx] = decode_lanes[lane_idx].ctrl.syscall &&
            (decode_lanes[lane_idx].instr == 32'h0000_0073) &&
            (lane_rs1_value[lane_idx] == ECALL_ARG_HALT);

        if (decode_lanes[lane_idx].ctrl.memWrite) begin
            data_addr = alu_out[31:2];
            store_data_for(decode_lanes[lane_idx].ctrl.ldst_mode, alu_out[1:0],
                lane_rs2_value[lane_idx], data_store, data_store_mask);
        end else if (decode_lanes[lane_idx].ctrl.memRead) begin
            data_addr = alu_out[31:2];
            data_load_en = 1'b1;
            load_pending_next = 1'b1;
            load_rd_next = decode_lanes[lane_idx].rd;
            load_mode_next = decode_lanes[lane_idx].ctrl.ldst_mode;
            load_byte_next = alu_out[1:0];
        end else if (decode_lanes[lane_idx].ctrl.rfWrite) begin
            lane_writes[lane_idx] = decode_lanes[lane_idx].rd != 5'd0;
            case (decode_lanes[lane_idx].ctrl.rd_source)
                RD_PC4: lane_results[lane_idx] = decode_lanes[lane_idx].pc + 32'd4;
                RD_IMM: lane_results[lane_idx] = decode_lanes[lane_idx].imm;
                RD_CMP: lane_results[lane_idx] = {31'b0, alu_out[0]};
                default: lane_results[lane_idx] = alu_out;
            endcase
        end

        if (decode_lanes[lane_idx].ctrl.pc_source == PC_uncond) begin
            lane_redirects[lane_idx] = 1'b1;
            lane_redirect_targets[lane_idx] = decode_lanes[lane_idx].pc +
                decode_lanes[lane_idx].imm;
        end else if (decode_lanes[lane_idx].ctrl.pc_source == PC_indirect) begin
            lane_redirects[lane_idx] = 1'b1;
            lane_redirect_targets[lane_idx] = (lane_rs1_value[lane_idx] +
                decode_lanes[lane_idx].imm) & 32'hffff_fffe;
        end else if (decode_lanes[lane_idx].ctrl.pc_source == PC_cond &&
                branch_taken) begin
            lane_redirects[lane_idx] = 1'b1;
            lane_redirect_targets[lane_idx] = decode_lanes[lane_idx].pc +
                decode_lanes[lane_idx].imm;
        end
    endtask

    function automatic logic [31:0] alu_result(logic [31:0] a, logic [31:0] b,
            alu_op_t op);
        case (op)
            ALU_ADD:  alu_result = a + b;
            ALU_SUB:  alu_result = a - b;
            ALU_XOR:  alu_result = a ^ b;
            ALU_OR:   alu_result = a | b;
            ALU_AND:  alu_result = a & b;
            ALU_SLL:  alu_result = a << b[4:0];
            ALU_SRL:  alu_result = a >> b[4:0];
            ALU_SRA:  alu_result = signed'(a) >>> b[4:0];
            ALU_SLT:  alu_result = {31'b0, signed'(a) < signed'(b)};
            ALU_SLTU: alu_result = {31'b0, a < b};
            default:  alu_result = a + b;
        endcase
    endfunction

    function automatic logic branch_result(logic [31:0] a, logic [31:0] b,
            logic [2:0] btype);
        case (btype)
            FUNCT3_BEQ:  branch_result = a == b;
            FUNCT3_BNE:  branch_result = a != b;
            FUNCT3_BLT:  branch_result = signed'(a) < signed'(b);
            FUNCT3_BGE:  branch_result = signed'(a) >= signed'(b);
            FUNCT3_BLTU: branch_result = a < b;
            FUNCT3_BGEU: branch_result = a >= b;
            default:     branch_result = 1'b0;
        endcase
    endfunction

    task automatic store_data_for(input ldst_mode_t mode,
            input logic [1:0] byte_sel,
            input logic [31:0] value, output logic [31:0] store_value,
            output logic [3:0] store_mask);
        case (mode)
            LDST_W: begin
                store_value = value;
                store_mask = 4'b1111;
            end
            LDST_H: begin
                store_value = byte_sel[1] ? {value[15:0], 16'b0} : value[15:0];
                store_mask = byte_sel[1] ? 4'b1100 : 4'b0011;
            end
            LDST_B: begin
                store_value = value[7:0] << (8 * byte_sel);
                store_mask = 4'b0001 << byte_sel;
            end
            default: begin
                store_value = 32'b0;
                store_mask = 4'b0000;
            end
        endcase
    endtask

endmodule: riscv_core_4wide
