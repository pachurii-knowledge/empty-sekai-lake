`include "ooo_types.vh"
`include "superscalar_types.vh"
`include "riscv_abi.vh"
`include "memory_segments.vh"

`default_nettype none

module riscv_core_ooo (
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

    import OOO_Types::*;
    import RISCV_ABI::ECALL_ARG_HALT;
    import MemorySegments::USER_TEXT_START;

    localparam int PHYS_READ_PORTS = 2 + OOO_WIDTH;
    localparam int RAS_DEPTH = 128;
    localparam int RAS_INDEX_BITS = $clog2(RAS_DEPTH);
    localparam int RAS_COUNT_BITS = $clog2(RAS_DEPTH + 1);

    logic [31:0] pc_q, pc_next;
    logic [1:0][31:0] fetch_pc_pipe_q, fetch_pc_pipe_next;
    logic [1:0] fetch_valid_pipe_q, fetch_valid_pipe_next;
    logic halted_q, halted_next;
    logic terminal_pending_q, terminal_pending_next;
    logic control_pending_q, control_pending_next;
    branch_id_t control_pending_id_q, control_pending_id_next;
    logic frontend_stall;
    logic redirect_valid;
    logic [31:0] redirect_pc;
    logic [31:0] sequential_next_pc;

    decode_lane_t decode_lanes [OOO_WIDTH];
    logic [OOO_WIDTH-1:0] lane_valid;
    logic [OOO_WIDTH-1:0] lane_has_dest;
    logic [OOO_WIDTH-1:0] lane_is_branch;
    logic [OOO_WIDTH-1:0] lane_is_unpredicted_control;
    logic [OOO_WIDTH-1:0] lane_is_call;
    logic [OOO_WIDTH-1:0] lane_is_return;
    logic [OOO_WIDTH-1:0] lane_control_predicted;
    logic [OOO_WIDTH-1:0][31:0] lane_predicted_pc;
    logic [OOO_WIDTH-1:0] lane_is_memory;
    logic [OOO_WIDTH-1:0] lane_is_terminal;
    logic [OOO_WIDTH-1:0] lane_is_serializing;
    logic [OOO_WIDTH-1:0] dispatch_valid;
    logic [OOO_WIDTH-1:0] alloc_req;
    logic [OOO_WIDTH-1:0] map_has_dest;
    logic dispatch_stall;
    logic [2:0] dispatch_count;
    logic [2:0] valid_count;
    logic [2:0] lane_active_offset [OOO_WIDTH];
    logic partial_resume_valid;
    logic [2:0] partial_resume_lane;
    logic partial_resume_lane_is_branch;
    logic dispatched_unpredicted_control;
    logic ras_redirect_valid;
    logic [31:0] ras_redirect_pc;
    logic predictor_redirect_valid;
    logic [31:0] predictor_redirect_pc;

    logic [31:0] ras_stack_q [RAS_DEPTH];
    logic [31:0] ras_stack_next [RAS_DEPTH];
    logic [RAS_COUNT_BITS-1:0] ras_count_q, ras_count_next;
    logic [RAS_COUNT_BITS-1:0] ras_checkpoint_count_q [BRANCH_STACK_SIZE];
    logic [RAS_COUNT_BITS-1:0] ras_checkpoint_count_next [BRANCH_STACK_SIZE];
    logic [RAS_COUNT_BITS-1:0] ras_branch_snapshot_count;

    logic direct_lookup_valid;
    logic [31:0] direct_lookup_pc;
    logic direct_prediction;
    predictor_info_t direct_prediction_info;
    logic indirect_lookup_valid;
    logic [31:0] indirect_lookup_pc;
    logic indirect_prediction_valid;
    logic [31:0] indirect_prediction_target;
    predictor_info_t indirect_prediction_info;
    predictor_info_t lane_predictor_info [OOO_WIDTH];

    arch_reg_t map_rs1 [OOO_WIDTH];
    arch_reg_t map_rs2 [OOO_WIDTH];
    arch_reg_t map_rd [OOO_WIDTH];
    phys_reg_t map_prs1 [OOO_WIDTH];
    phys_reg_t map_prs2 [OOO_WIDTH];
    phys_reg_t map_old_prd [OOO_WIDTH];
    phys_reg_t map_snapshot [32];

    logic [OOO_WIDTH-1:0] free_alloc_valid;
    phys_reg_t free_alloc_prd [OOO_WIDTH];
    logic free_can_allocate;
    logic [$clog2(PHYS_REGS)-1:0] free_head_snapshot;
    logic [$clog2(PHYS_REGS)-1:0] free_tail_snapshot;
    logic [$clog2(PHYS_REGS+1)-1:0] free_count_snapshot;
    logic [OOO_WIDTH-1:0] active_free_valid;
    phys_reg_t active_free_prd [OOO_WIDTH];

    logic [OOO_WIDTH-1:0] busy_src1_ready;
    logic [OOO_WIDTH-1:0] busy_src2_ready;
    logic [OOO_WIDTH-1:0] wakeup_valid;
    phys_reg_t wakeup_prd [OOO_WIDTH];

    rename_packet_t rename_packets [OOO_WIDTH];
    issue_entry_t dispatch_issue_entries [OOO_WIDTH];
    logic [OOO_WIDTH-1:0] int_insert_valid;
    logic [OOO_WIDTH-1:0] mem_insert_valid;

    logic active_full;
    active_id_t active_tail;
    logic [OOO_WIDTH-1:0] active_commit_valid;
    commit_packet_t active_commit_packet [OOO_WIDTH];

    logic int_iq_full;
    logic [1:0] int_issue_valid;
    issue_entry_t int_issue_entry [2];

    phys_reg_t phys_rs1 [PHYS_READ_PORTS];
    phys_reg_t phys_rs2 [PHYS_READ_PORTS];
    logic [PHYS_READ_PORTS-1:0][31:0] phys_rs1_data;
    logic [PHYS_READ_PORTS-1:0][31:0] phys_rs2_data;
    logic [OOO_WIDTH-1:0][31:0] mem_insert_rs1_data;
    logic [OOO_WIDTH-1:0][31:0] mem_insert_rs2_data;
    logic [OOO_WIDTH-1:0] phys_write_valid;
    phys_reg_t phys_write_prd [OOO_WIDTH];
    logic [OOO_WIDTH-1:0][31:0] phys_write_data;

    writeback_packet_t alu0_writeback;
    writeback_packet_t alu1_writeback;
    writeback_packet_t load_writeback;
    writeback_packet_t branch_writeback;
    logic [OOO_WIDTH-1:0] writeback_valid;
    active_id_t writeback_active_id [OOO_WIDTH];
    phys_reg_t writeback_prd [OOO_WIDTH];
    logic [OOO_WIDTH-1:0][31:0] writeback_data;
    logic [OOO_WIDTH-1:0] writeback_has_dest;
    logic [OOO_WIDTH-1:0] writeback_fp_write;
    arch_reg_t writeback_fp_rd [OOO_WIDTH];
    fp_reg_data_t writeback_fp_data [OOO_WIDTH];
    logic [OOO_WIDTH-1:0] writeback_csr_write;
    logic [OOO_WIDTH-1:0][11:0] writeback_csr_addr;
    logic [OOO_WIDTH-1:0][31:0] writeback_csr_wdata;
    logic [OOO_WIDTH-1:0] writeback_exception;
    logic [OOO_WIDTH-1:0] writeback_halted;

    logic [31:0] csr_read_data [2];
    logic [1:0] csr_read_illegal;
    logic csr_commit_write;
    logic [11:0] csr_commit_addr;
    logic [31:0] csr_commit_wdata;
    logic csr_retire;

    fp_reg_data_t fp_regs_q [FP_REGS];
    fp_reg_data_t fp_regs_next [FP_REGS];
    logic serial_pending_q, serial_pending_next;

    logic branch_stack_full;
    logic branch_allocate;
    logic branch_allocate_valid;
    branch_id_t branch_allocate_id;
    active_id_t branch_active_tail_snapshot;
    logic [$clog2(PHYS_REGS)-1:0] branch_free_head_snapshot;
    logic [$clog2(PHYS_REGS)-1:0] branch_free_tail_snapshot;
    logic [$clog2(PHYS_REGS+1)-1:0] branch_free_count_snapshot;
    phys_reg_t branch_map_snapshot [32];
    branch_mask_t current_branch_mask;
    logic branch_restore_valid;
    active_id_t branch_restore_active_tail;
    logic [$clog2(PHYS_REGS)-1:0] branch_restore_free_head;
    logic [$clog2(PHYS_REGS)-1:0] branch_restore_free_tail;
    logic [$clog2(PHYS_REGS+1)-1:0] branch_restore_free_count;
    phys_reg_t branch_restore_map [32];
    branch_mask_t stack_reset_mask;
    branch_mask_t stack_abort_mask;
    logic branch_resolve_valid;
    branch_id_t branch_resolve_id;
    logic branch_resolve_mispredict;
    branch_mask_t reset_mask;
    branch_mask_t abort_mask;
    branch_mask_t abort_mask_q;
    branch_mask_t dispatch_branch_mask;

    logic mem_queue_full;
    logic mem_data_load_en;
    logic [29:0] mem_data_addr;
    logic [31:0] mem_data_store;
    logic [3:0] mem_data_store_mask;
    logic commit_store;
    active_id_t commit_store_id;

    logic [OOO_WIDTH-1:0] retire_valid;
    logic [OOO_WIDTH-1:0] commit_free_valid;
    phys_reg_t commit_free_prd [OOO_WIDTH];
    logic precise_halt;
    logic precise_exception;

    logic [OOO_WIDTH-1:0] arch_rd_we;
    logic [OOO_WIDTH-1:0][4:0] arch_rs1;
    logic [OOO_WIDTH-1:0][4:0] arch_rs2;
    logic [OOO_WIDTH-1:0][4:0] arch_rd;
    logic [OOO_WIDTH-1:0][31:0] arch_rd_data;
    logic [OOO_WIDTH-1:0][31:0] arch_rs1_data;
    logic [OOO_WIDTH-1:0][31:0] arch_rs2_data;

    assign halted = halted_q;
    assign data_stall = 1'b0;
    assign instr_stall = !rst_l || halted_q || frontend_stall ||
        dispatched_unpredicted_control;
    assign instr_addr = {pc_q[31:4], 2'b00};
    assign sequential_next_pc = {pc_q[31:4], 4'b0} + 32'd16;
    assign dispatch_branch_mask = current_branch_mask & ~reset_mask & ~abort_mask;

    ooo_fetch_decode FetchDecode (
        .rst_l,
        .fetch_valid(fetch_valid_pipe_q[1] && !halted_q),
        .instr_mem_excpt,
        .fetch_pc(fetch_pc_pipe_q[1]),
        .instr,
        .decode_lanes
    );

    rename_map_table MapTable (
        .clk,
        .rst_l,
        .restore_valid(branch_restore_valid),
        .restore_map(branch_restore_map),
        .rename_valid(dispatch_valid),
        .rs1(map_rs1),
        .rs2(map_rs2),
        .rd(map_rd),
        .rename_has_dest(lane_has_dest),
        .alloc_prd(free_alloc_prd),
        .prs1(map_prs1),
        .prs2(map_prs2),
        .old_prd(map_old_prd),
        .has_dest(map_has_dest),
        .snapshot_map(map_snapshot)
    );

    free_list FreeList (
        .clk,
        .rst_l,
        .restore_valid(branch_restore_valid),
        .restore_head(branch_restore_free_head),
        .restore_tail(branch_restore_free_tail),
        .restore_count(branch_restore_free_count),
        .alloc_req(alloc_req),
        .free_valid(active_free_valid),
        .free_prd(active_free_prd),
        .alloc_valid(free_alloc_valid),
        .alloc_prd(free_alloc_prd),
        .can_allocate(free_can_allocate),
        .snapshot_head(free_head_snapshot),
        .snapshot_tail(free_tail_snapshot),
        .snapshot_count(free_count_snapshot)
    );

    busy_table BusyTable (
        .clk,
        .rst_l,
        .allocate_valid(free_alloc_valid),
        .allocate_prd(free_alloc_prd),
        .writeback_valid(wakeup_valid),
        .writeback_prd(wakeup_prd),
        .src1_prd(map_prs1),
        .src2_prd(map_prs2),
        .src1_ready(busy_src1_ready),
        .src2_ready(busy_src2_ready)
    );

    rv32g_csr_file CSRFile (
        .clk,
        .rst_l,
        .retire(csr_retire),
        .write_valid(csr_commit_write),
        .write_addr(csr_commit_addr),
        .write_data(csr_commit_wdata),
        .read_addr0(int_issue_entry[0].instr[31:20]),
        .read_addr1(int_issue_entry[1].instr[31:20]),
        .read_data0(csr_read_data[0]),
        .read_data1(csr_read_data[1]),
        .read_illegal0(csr_read_illegal[0]),
        .read_illegal1(csr_read_illegal[1])
    );

    branch_stack BranchStack (
        .clk,
        .rst_l,
        .allocate(branch_allocate),
        .active_tail_snapshot(branch_active_tail_snapshot),
        .free_head_snapshot(branch_free_head_snapshot),
        .free_tail_snapshot(branch_free_tail_snapshot),
        .free_count_snapshot(branch_free_count_snapshot),
        .map_snapshot(branch_map_snapshot),
        .resolve(branch_resolve_valid),
        .resolve_id(branch_resolve_id),
        .mispredict(branch_resolve_mispredict),
        .full(branch_stack_full),
        .allocate_valid(branch_allocate_valid),
        .allocate_id(branch_allocate_id),
        .current_mask(current_branch_mask),
        .restore_valid(branch_restore_valid),
        .restore_active_tail(branch_restore_active_tail),
        .restore_free_head(branch_restore_free_head),
        .restore_free_tail(branch_restore_free_tail),
        .restore_free_count(branch_restore_free_count),
        .restore_map(branch_restore_map),
        .reset_mask(stack_reset_mask),
        .abort_mask(stack_abort_mask)
    );

    tage_sc_l_predictor DirectBranchPredictor (
        .clk,
        .rst_l,
        .lookup_valid(direct_lookup_valid),
        .lookup_pc(direct_lookup_pc),
        .prediction(direct_prediction),
        .prediction_info(direct_prediction_info),
        .update_valid(branch_writeback.valid && branch_writeback.branch_valid &&
            (branch_writeback.instr[6:0] == RISCV_ISA::OP_BRANCH)),
        .update_pc(branch_writeback.pc),
        .update_taken(branch_writeback.redirect_pc != (branch_writeback.pc + 32'd4)),
        .update_info(branch_writeback.predictor_info)
    );

    ittage_predictor IndirectBranchPredictor (
        .clk,
        .rst_l,
        .lookup_valid(indirect_lookup_valid),
        .lookup_pc(indirect_lookup_pc),
        .prediction_valid(indirect_prediction_valid),
        .prediction_target(indirect_prediction_target),
        .prediction_info(indirect_prediction_info),
        .update_valid(branch_writeback.valid && branch_writeback.branch_valid &&
            (branch_writeback.instr[6:0] == RISCV_ISA::OP_JALR) &&
            !((branch_writeback.instr[19:15] == 5'd1) &&
              (branch_writeback.instr[11:7] == 5'd0))),
        .update_target(branch_writeback.redirect_pc),
        .update_info(branch_writeback.predictor_info)
    );

    ooo_dispatch_control DispatchControl (
        .lane_valid,
        .lane_has_dest,
        .lane_is_branch,
        .lane_is_memory,
        .lane_is_terminal,
        .lane_is_serializing,
        .active_list_full(active_full),
        .int_iq_full(int_iq_full),
        .mem_queue_full(mem_queue_full),
        .branch_stack_full(branch_stack_full),
        .free_list_can_allocate(free_can_allocate),
        .free_list_available(free_count_snapshot),
        .suppress_dispatch(redirect_valid || terminal_pending_q ||
            control_pending_q || serial_pending_q || halted_q),
        .dispatch_valid,
        .dispatch_stall
    );

    active_list ActiveList (
        .clk,
        .rst_l,
        .restore_valid(branch_restore_valid),
        .restore_tail(branch_restore_active_tail),
        .allocate_valid(dispatch_valid),
        .allocate_packet(rename_packets),
        .writeback_valid(writeback_valid),
        .writeback_id(writeback_active_id),
        .writeback_data(writeback_data),
        .writeback_exception(writeback_exception),
        .writeback_halted(writeback_halted),
        .writeback_fp_write,
        .writeback_fp_rd,
        .writeback_fp_data,
        .writeback_csr_write,
        .writeback_csr_addr,
        .writeback_csr_wdata,
        .reset_mask,
        .abort_mask,
        .full(active_full),
        .tail(active_tail),
        .commit_valid(active_commit_valid),
        .commit_packet(active_commit_packet),
        .free_valid(active_free_valid),
        .free_prd(active_free_prd)
    );

    int_issue_queue IntIssueQueue (
        .clk,
        .rst_l,
        .insert_valid(int_insert_valid),
        .insert_entry(dispatch_issue_entries),
        .wakeup_valid,
        .wakeup_prd,
        .reset_mask,
        .abort_mask,
        .full(int_iq_full),
        .issue_valid(int_issue_valid),
        .issue_entry(int_issue_entry)
    );

    phys_reg_file #(.READ_PORTS(PHYS_READ_PORTS)) PhysRegFile (
        .clk,
        .rst_l,
        .rs1(phys_rs1),
        .rs2(phys_rs2),
        .write_valid(phys_write_valid),
        .write_prd(phys_write_prd),
        .write_data(phys_write_data),
        .rs1_data(phys_rs1_data),
        .rs2_data(phys_rs2_data)
    );

    ooo_alu_pipe ALU0 (
        .clk,
        .rst_l,
        .issue_valid(int_issue_valid[0]),
        .issue_entry(int_issue_entry[0]),
        .rs1_data(phys_rs1_data[0]),
        .rs2_data(phys_rs2_data[0]),
        .csr_rdata(csr_read_data[0]),
        .csr_illegal(csr_read_illegal[0]),
        .abort_mask,
        .writeback(alu0_writeback)
    );

    ooo_alu_pipe ALU1 (
        .clk,
        .rst_l,
        .issue_valid(int_issue_valid[1]),
        .issue_entry(int_issue_entry[1]),
        .rs1_data(phys_rs1_data[1]),
        .rs2_data(phys_rs2_data[1]),
        .csr_rdata(csr_read_data[1]),
        .csr_illegal(csr_read_illegal[1]),
        .abort_mask,
        .writeback(alu1_writeback)
    );

    load_store_queue LoadStoreQueue (
        .clk,
        .rst_l,
        .insert_valid(mem_insert_valid),
        .insert_entry(dispatch_issue_entries),
        .insert_rs1_data(mem_insert_rs1_data),
        .insert_rs2_data(mem_insert_rs2_data),
        .wakeup_valid,
        .wakeup_prd,
        .wakeup_data(writeback_data),
        .reset_mask,
        .abort_mask,
        .data_load_valid,
        .data_load(data_load[0]),
        .data_load_addr,
        .commit_store,
        .commit_store_id,
        .full(mem_queue_full),
        .data_load_en(mem_data_load_en),
        .data_addr(mem_data_addr),
        .data_store(mem_data_store),
        .data_store_mask(mem_data_store_mask),
        .load_writeback
    );

    ooo_writeback_bus WritebackBus (
        .alu0_writeback,
        .alu1_writeback,
        .load_writeback,
        .abort_mask_q,
        .writeback_valid,
        .writeback_active_id,
        .writeback_prd,
        .writeback_data,
        .writeback_has_dest,
        .writeback_fp_write,
        .writeback_fp_rd,
        .writeback_fp_data,
        .writeback_csr_write,
        .writeback_csr_addr,
        .writeback_csr_wdata,
        .writeback_exception,
        .writeback_halted,
        .branch_writeback
    );

    ooo_branch_recovery BranchRecovery (
        .branch_writeback,
        .stack_reset_mask(stack_reset_mask),
        .stack_abort_mask(stack_abort_mask),
        .stack_restore_valid(branch_restore_valid),
        .fetch_pc_plus4(sequential_next_pc),
        .resolve_valid(branch_resolve_valid),
        .resolve_id(branch_resolve_id),
        .resolve_mispredict(branch_resolve_mispredict),
        .reset_mask,
        .abort_mask,
        .redirect_valid,
        .redirect_pc
    );

    ooo_commit_unit CommitUnit (
        .commit_valid(active_commit_valid),
        .commit_packet(active_commit_packet),
        .store_port_busy(1'b0),
        .retire_valid,
        .free_valid(commit_free_valid),
        .free_prd(commit_free_prd),
        .commit_store,
        .commit_store_id,
        .precise_halt,
        .precise_exception
    );

    register_file #(.WAYS(OOO_WIDTH), .FORWARD(0)) ArchitecturalRF (
        .clk,
        .rst_l,
        .halted,
        .rd_we(arch_rd_we),
        .rs1(arch_rs1),
        .rs2(arch_rs2),
        .rd(arch_rd),
        .rd_data(arch_rd_data),
        .rs1_data(arch_rs1_data),
        .rs2_data(arch_rs2_data)
    );

    always_comb begin
        direct_lookup_valid = 1'b0;
        direct_lookup_pc = '0;
        indirect_lookup_valid = 1'b0;
        indirect_lookup_pc = '0;
        for (int i = 0; i < OOO_WIDTH; i += 1) begin
            if (decode_lanes[i].valid && !decode_lanes[i].kill &&
                    (decode_lanes[i].ctrl.pc_source == PC_cond) &&
                    !direct_lookup_valid) begin
                direct_lookup_valid = 1'b1;
                direct_lookup_pc = decode_lanes[i].pc;
            end
            if (decode_lanes[i].valid && !decode_lanes[i].kill &&
                    (decode_lanes[i].ctrl.pc_source == PC_indirect) &&
                    !((decode_lanes[i].rs1 == 5'd1) &&
                      (decode_lanes[i].rd == 5'd0)) &&
                    !indirect_lookup_valid) begin
                indirect_lookup_valid = 1'b1;
                indirect_lookup_pc = decode_lanes[i].pc;
            end
        end
    end

    always_comb begin
        lane_valid = '0;
        lane_has_dest = '0;
        lane_is_branch = '0;
        lane_is_unpredicted_control = '0;
        lane_is_call = '0;
        lane_is_return = '0;
        lane_control_predicted = '0;
        lane_is_memory = '0;
        lane_is_terminal = '0;
        lane_is_serializing = '0;
        alloc_req = '0;
        int_insert_valid = '0;
        mem_insert_valid = '0;
        branch_allocate = 1'b0;
        dispatch_count = '0;
        valid_count = '0;
        partial_resume_valid = 1'b0;
        partial_resume_lane = '0;
        partial_resume_lane_is_branch = 1'b0;
        dispatched_unpredicted_control = 1'b0;
        ras_redirect_valid = 1'b0;
        ras_redirect_pc = '0;
        predictor_redirect_valid = 1'b0;
        predictor_redirect_pc = '0;
        ras_stack_next = ras_stack_q;
        fp_regs_next = fp_regs_q;
        serial_pending_next = serial_pending_q;
        csr_retire = 1'b0;
        csr_commit_write = 1'b0;
        csr_commit_addr = '0;
        csr_commit_wdata = '0;
        ras_count_next = branch_restore_valid ?
            ras_checkpoint_count_q[branch_resolve_id] : ras_count_q;
        ras_checkpoint_count_next = ras_checkpoint_count_q;
        ras_branch_snapshot_count = ras_count_next;
        frontend_stall = dispatch_stall;
        branch_active_tail_snapshot = active_tail;
        branch_free_head_snapshot = free_head_snapshot;
        branch_free_tail_snapshot = free_tail_snapshot;
        branch_free_count_snapshot = free_count_snapshot;
        for (int i = 0; i < 32; i += 1) begin
            branch_map_snapshot[i] = map_snapshot[i];
        end

        for (int i = 0; i < OOO_WIDTH; i += 1) begin
            map_rs1[i] = decode_lanes[i].rs1;
            map_rs2[i] = decode_lanes[i].rs2;
            map_rd[i] = decode_lanes[i].rd;
            lane_valid[i] = decode_lanes[i].valid && !decode_lanes[i].kill;
            lane_has_dest[i] = lane_valid[i] && decode_lanes[i].ctrl.rfWrite &&
                (decode_lanes[i].rd != 5'd0);
            lane_is_branch[i] = lane_valid[i] &&
                ((decode_lanes[i].ctrl.pc_source == PC_cond) ||
                 (decode_lanes[i].ctrl.pc_source == PC_uncond) ||
                 (decode_lanes[i].ctrl.pc_source == PC_indirect));
            lane_is_unpredicted_control[i] = lane_valid[i] &&
                ((decode_lanes[i].ctrl.pc_source == PC_uncond) ||
                 (decode_lanes[i].ctrl.pc_source == PC_indirect));
            lane_is_call[i] = lane_valid[i] &&
                (decode_lanes[i].ctrl.pc_source == PC_uncond) &&
                (decode_lanes[i].rd != 5'd0);
            lane_is_return[i] = lane_valid[i] &&
                (decode_lanes[i].ctrl.pc_source == PC_indirect) &&
                (decode_lanes[i].rs1 == 5'd1) &&
                (decode_lanes[i].rd == 5'd0);
            lane_is_memory[i] = lane_valid[i] &&
                (decode_lanes[i].ctrl.memRead || decode_lanes[i].ctrl.memWrite);
            lane_is_terminal[i] = lane_valid[i] &&
                (decode_lanes[i].ctrl.syscall || decode_lanes[i].ctrl.illegal_instr);
            lane_is_serializing[i] = lane_valid[i] &&
                decode_lanes[i].ctrl.serializing;
            lane_predicted_pc[i] = decode_lanes[i].pc + 32'd4;
            lane_active_offset[i] = dispatch_count;
            if (lane_valid[i]) begin
                valid_count += 1'b1;
            end
            lane_predictor_info[i] = '0;
            if (dispatch_valid[i]) begin
                dispatch_count += 1'b1;
            end else if (lane_valid[i] && !partial_resume_valid) begin
                partial_resume_valid = 1'b1;
                partial_resume_lane = 3'(i);
                partial_resume_lane_is_branch = lane_is_branch[i];
            end
        end

        branch_active_tail_snapshot = active_tail + active_id_t'(dispatch_count);
        for (int i = 0; i < OOO_WIDTH; i += 1) begin
            if (dispatch_valid[i]) begin
                if (lane_is_call[i] &&
                        (ras_count_next < RAS_COUNT_BITS'(RAS_DEPTH))) begin
                    ras_stack_next[RAS_INDEX_BITS'(ras_count_next)] =
                        decode_lanes[i].pc + 32'd4;
                    ras_count_next = ras_count_next + 1'b1;
                end else if (lane_is_return[i] && (ras_count_next != '0)) begin
                    lane_control_predicted[i] = 1'b1;
                    lane_predicted_pc[i] =
                        ras_stack_next[RAS_INDEX_BITS'(ras_count_next - 1'b1)];
                    ras_redirect_valid = 1'b1;
                    ras_redirect_pc =
                        ras_stack_next[RAS_INDEX_BITS'(ras_count_next - 1'b1)];
                    ras_count_next = ras_count_next - 1'b1;
                end else if (decode_lanes[i].ctrl.pc_source == PC_cond) begin
                    lane_predictor_info[i] = direct_prediction_info;
                    if (direct_prediction) begin
                        lane_control_predicted[i] = 1'b1;
                        lane_predicted_pc[i] = decode_lanes[i].pc +
                            decode_lanes[i].imm;
                        predictor_redirect_valid = 1'b1;
                        predictor_redirect_pc = decode_lanes[i].pc +
                            decode_lanes[i].imm;
                    end
                end else if ((decode_lanes[i].ctrl.pc_source == PC_indirect) &&
                        !lane_is_return[i]) begin
                    lane_predictor_info[i] = indirect_prediction_info;
                    if (indirect_prediction_valid) begin
                        lane_control_predicted[i] = 1'b1;
                        lane_predicted_pc[i] = indirect_prediction_target;
                        predictor_redirect_valid = 1'b1;
                        predictor_redirect_pc = indirect_prediction_target;
                    end
                end
                if (lane_has_dest[i] && free_alloc_valid[i]) begin
                    branch_map_snapshot[decode_lanes[i].rd] = free_alloc_prd[i];
                    branch_free_head_snapshot = branch_free_head_snapshot + 1'b1;
                    branch_free_count_snapshot = branch_free_count_snapshot - 1'b1;
                end
                if (lane_is_branch[i]) begin
                    branch_active_tail_snapshot = active_tail +
                        active_id_t'(lane_active_offset[i]) + active_id_t'(1);
                    ras_branch_snapshot_count = ras_count_next;
                    break;
                end
            end
        end
        if (branch_allocate_valid) begin
            ras_checkpoint_count_next[branch_allocate_id] = ras_branch_snapshot_count;
        end
        for (int i = 0; i < OOO_WIDTH; i += 1) begin
            if (dispatch_valid[i] && lane_is_unpredicted_control[i] &&
                    !lane_control_predicted[i]) begin
                dispatched_unpredicted_control = 1'b1;
            end
        end

        for (int i = 0; i < OOO_WIDTH; i += 1) begin
            alloc_req[i] = dispatch_valid[i] && lane_has_dest[i];
            rename_packets[i] = '0;
            rename_packets[i].valid = dispatch_valid[i];
            rename_packets[i].pc = decode_lanes[i].pc;
            rename_packets[i].instr = decode_lanes[i].instr;
            rename_packets[i].ctrl = decode_lanes[i].ctrl;
            rename_packets[i].rs1 = decode_lanes[i].rs1;
            rename_packets[i].rs2 = decode_lanes[i].rs2;
            rename_packets[i].rd = decode_lanes[i].rd;
            rename_packets[i].prs1 = map_prs1[i];
            rename_packets[i].prs2 = map_prs2[i];
            rename_packets[i].prd = free_alloc_prd[i];
            rename_packets[i].old_prd = map_old_prd[i];
            rename_packets[i].src1_ready = !decode_lanes[i].uses_rs1 ||
                busy_src1_ready[i];
            rename_packets[i].src2_ready = !decode_lanes[i].uses_rs2 ||
                busy_src2_ready[i];
            rename_packets[i].has_dest = map_has_dest[i];
            rename_packets[i].imm = decode_lanes[i].imm;
            rename_packets[i].branch_mask = dispatch_branch_mask;
            rename_packets[i].branch_id = lane_is_branch[i] ?
                branch_allocate_id : '0;
            rename_packets[i].active_id = active_tail +
                active_id_t'(lane_active_offset[i]);
            rename_packets[i].control_predicted = lane_control_predicted[i];
            rename_packets[i].predicted_pc = lane_predicted_pc[i];
            rename_packets[i].predictor_info = lane_predictor_info[i];
            rename_packets[i].fp_rs1 = decode_lanes[i].rs1;
            rename_packets[i].fp_rs2 = decode_lanes[i].rs2;
            rename_packets[i].fp_rs3 = decode_lanes[i].instr[31:27];
            rename_packets[i].fp_rd = decode_lanes[i].rd;
            rename_packets[i].fp_src1_data = fp_regs_q[decode_lanes[i].rs1];
            rename_packets[i].fp_src2_data = fp_regs_q[decode_lanes[i].rs2];
            rename_packets[i].fp_src3_data =
                fp_regs_q[decode_lanes[i].instr[31:27]];

            dispatch_issue_entries[i] = '0;
            dispatch_issue_entries[i].valid = dispatch_valid[i];
            dispatch_issue_entries[i].pc = rename_packets[i].pc;
            dispatch_issue_entries[i].instr = rename_packets[i].instr;
            dispatch_issue_entries[i].ctrl = rename_packets[i].ctrl;
            dispatch_issue_entries[i].prs1 = rename_packets[i].prs1;
            dispatch_issue_entries[i].prs2 = rename_packets[i].prs2;
            dispatch_issue_entries[i].prd = rename_packets[i].prd;
            dispatch_issue_entries[i].src1_ready = rename_packets[i].src1_ready;
            dispatch_issue_entries[i].src2_ready = rename_packets[i].src2_ready;
            dispatch_issue_entries[i].has_dest = rename_packets[i].has_dest;
            dispatch_issue_entries[i].imm = rename_packets[i].imm;
            dispatch_issue_entries[i].branch_mask = rename_packets[i].branch_mask;
            dispatch_issue_entries[i].branch_id = rename_packets[i].branch_id;
            dispatch_issue_entries[i].active_id = rename_packets[i].active_id;
            dispatch_issue_entries[i].control_predicted =
                rename_packets[i].control_predicted;
            dispatch_issue_entries[i].predicted_pc = rename_packets[i].predicted_pc;
            dispatch_issue_entries[i].predictor_info =
                rename_packets[i].predictor_info;
            dispatch_issue_entries[i].fp_rs1 = rename_packets[i].fp_rs1;
            dispatch_issue_entries[i].fp_rs2 = rename_packets[i].fp_rs2;
            dispatch_issue_entries[i].fp_rs3 = rename_packets[i].fp_rs3;
            dispatch_issue_entries[i].fp_rd = rename_packets[i].fp_rd;
            dispatch_issue_entries[i].fp_src1_data =
                rename_packets[i].fp_src1_data;
            dispatch_issue_entries[i].fp_src2_data =
                rename_packets[i].fp_src2_data;
            dispatch_issue_entries[i].fp_src3_data =
                rename_packets[i].fp_src3_data;

            int_insert_valid[i] = dispatch_valid[i] && !lane_is_memory[i];
            mem_insert_valid[i] = dispatch_valid[i] && lane_is_memory[i];
            branch_allocate |= dispatch_valid[i] && lane_is_branch[i];
        end

        phys_rs1[0] = int_issue_entry[0].prs1;
        phys_rs2[0] = int_issue_entry[0].prs2;
        phys_rs1[1] = int_issue_entry[1].prs1;
        phys_rs2[1] = int_issue_entry[1].prs2;
        for (int i = 0; i < OOO_WIDTH; i += 1) begin
            phys_rs1[2 + i] = dispatch_issue_entries[i].prs1;
            phys_rs2[2 + i] = dispatch_issue_entries[i].prs2;
            mem_insert_rs1_data[i] = phys_rs1_data[2 + i];
            mem_insert_rs2_data[i] = phys_rs2_data[2 + i];
        end

        for (int i = 0; i < OOO_WIDTH; i += 1) begin
            wakeup_valid[i] = writeback_valid[i] && writeback_has_dest[i];
            wakeup_prd[i] = writeback_prd[i];
            phys_write_valid[i] = writeback_valid[i] && writeback_has_dest[i];
            phys_write_prd[i] = writeback_prd[i];
            phys_write_data[i] = writeback_data[i];
        end

        arch_rd_we = '0;
        arch_rd = '0;
        arch_rd_data = '0;
        arch_rs1 = '0;
        arch_rs2 = '0;
        for (int i = 0; i < OOO_WIDTH; i += 1) begin
            arch_rd_we[i] = retire_valid[i] && active_commit_packet[i].has_dest;
            arch_rd[i] = active_commit_packet[i].rd;
            arch_rd_data[i] = active_commit_packet[i].data;
        end

        data_load_en = mem_data_load_en;
        data_addr = mem_data_addr;
        data_store = mem_data_store;
        data_store_mask = mem_data_store_mask;

        pc_next = pc_q;
        fetch_pc_pipe_next = fetch_pc_pipe_q;
        fetch_valid_pipe_next = fetch_valid_pipe_q;
        halted_next = halted_q || precise_halt || precise_exception;
        terminal_pending_next = terminal_pending_q;
        control_pending_next = control_pending_q;
        control_pending_id_next = control_pending_id_q;
        if (branch_resolve_valid && control_pending_q &&
                ((branch_resolve_id == control_pending_id_q) ||
                 abort_mask[control_pending_id_q])) begin
            control_pending_next = 1'b0;
            control_pending_id_next = '0;
        end
        if (redirect_valid || precise_halt || precise_exception) begin
            terminal_pending_next = 1'b0;
        end
        for (int i = 0; i < OOO_WIDTH; i += 1) begin
            if (active_commit_valid[i] &&
                    (active_commit_packet[i].instr == 32'h0000_0073)) begin
                terminal_pending_next = 1'b0;
            end
        end

        if (redirect_valid) begin
            pc_next = redirect_pc;
            fetch_pc_pipe_next = '0;
            fetch_valid_pipe_next = '0;
        end else if (!frontend_stall && !halted_q) begin
            if (ras_redirect_valid) begin
                pc_next = ras_redirect_pc;
                fetch_pc_pipe_next = '0;
                fetch_valid_pipe_next = '0;
            end else if (predictor_redirect_valid) begin
                pc_next = predictor_redirect_pc;
                fetch_pc_pipe_next = '0;
                fetch_valid_pipe_next = '0;
            end else if (dispatched_unpredicted_control) begin
                fetch_pc_pipe_next = '0;
                fetch_valid_pipe_next = '0;
            end else if (fetch_valid_pipe_q[1] && (dispatch_count < valid_count)) begin
                pc_next = decode_lanes[2'(dispatch_count)].pc;
                fetch_pc_pipe_next = '0;
                fetch_valid_pipe_next = '0;
            end else begin
                pc_next = sequential_next_pc;
                fetch_pc_pipe_next[0] = pc_q;
                fetch_pc_pipe_next[1] = fetch_pc_pipe_q[0];
                fetch_valid_pipe_next[0] = rst_l;
                fetch_valid_pipe_next[1] = fetch_valid_pipe_q[0];
            end
        end

        for (int i = 0; i < OOO_WIDTH; i += 1) begin
            if (dispatch_valid[i] && lane_is_terminal[i] && !redirect_valid) begin
                terminal_pending_next = 1'b1;
            end
            if (dispatch_valid[i] && decode_lanes[i].ctrl.serializing) begin
                serial_pending_next = 1'b1;
            end
            if (dispatch_valid[i] && lane_is_unpredicted_control[i] &&
                    !lane_control_predicted[i]) begin
                control_pending_next = 1'b1;
                control_pending_id_next = branch_allocate_id;
            end
        end

        for (int i = 0; i < OOO_WIDTH; i += 1) begin
            if (retire_valid[i]) begin
                csr_retire = 1'b1;
                if (active_commit_packet[i].fp_write) begin
                    fp_regs_next[active_commit_packet[i].fp_rd] =
                        active_commit_packet[i].fp_data;
                end
                if (!csr_commit_write && active_commit_packet[i].csr_write) begin
                    csr_commit_write = 1'b1;
                    csr_commit_addr = active_commit_packet[i].csr_addr;
                    csr_commit_wdata = active_commit_packet[i].csr_wdata;
                end
                if (active_commit_packet[i].serializing) begin
                    serial_pending_next = 1'b0;
                end
            end
        end
    end

    always_ff @(posedge clk or negedge rst_l) begin
        if (!rst_l) begin
            pc_q <= USER_TEXT_START;
            fetch_pc_pipe_q <= '0;
            fetch_valid_pipe_q <= '0;
            halted_q <= 1'b0;
            terminal_pending_q <= 1'b0;
            control_pending_q <= 1'b0;
            control_pending_id_q <= '0;
            serial_pending_q <= 1'b0;
            abort_mask_q <= '0;
            ras_count_q <= '0;
            for (int i = 0; i < FP_REGS; i += 1) begin
                fp_regs_q[i] <= '0;
            end
            for (int i = 0; i < RAS_DEPTH; i += 1) begin
                ras_stack_q[i] <= '0;
            end
            for (int i = 0; i < BRANCH_STACK_SIZE; i += 1) begin
                ras_checkpoint_count_q[i] <= '0;
            end
        end else begin
            pc_q <= pc_next;
            fetch_pc_pipe_q <= fetch_pc_pipe_next;
            fetch_valid_pipe_q <= fetch_valid_pipe_next;
            halted_q <= halted_next;
            terminal_pending_q <= terminal_pending_next;
            control_pending_q <= control_pending_next;
            control_pending_id_q <= control_pending_id_next;
            serial_pending_q <= serial_pending_next;
            abort_mask_q <= abort_mask;
            ras_count_q <= ras_count_next;
            for (int i = 0; i < RAS_DEPTH; i += 1) begin
                ras_stack_q[i] <= ras_stack_next[i];
            end
            for (int i = 0; i < BRANCH_STACK_SIZE; i += 1) begin
                ras_checkpoint_count_q[i] <= ras_checkpoint_count_next[i];
            end
            for (int i = 0; i < FP_REGS; i += 1) begin
                fp_regs_q[i] <= fp_regs_next[i];
            end
        end
    end








`ifdef SIMULATION_18447
    localparam int PERF_STALL_BUCKETS = 8;
    localparam int PERF_STALL_BITS = $clog2(PERF_STALL_BUCKETS);

    logic [63:0] perf_cycle_counter;
    logic [63:0] perf_dispatch_counter;
    logic [63:0] perf_retire_counter;
    logic [63:0] perf_frontend_stall_cycles;
    logic [63:0] perf_branch_instructions;
    logic [63:0] perf_mispredicted_branches;
    logic [63:0] perf_alu_instructions;
    logic [63:0] perf_load_instructions;
    logic [63:0] perf_store_instructions;
    logic [63:0] perf_total_data_reads;
    logic [63:0] perf_total_data_writes;
    logic [63:0] perf_stall_instr [PERF_STALL_BUCKETS];
    logic [63:0] perf_branch_instr_counter [16];
    logic [63:0] perf_jal_instr_counter [8];
    logic [63:0] perf_jalr_instr_counter [8];
    logic [63:0] perf_jalr_predicted_correct;
    logic [63:0] perf_jalr_predicted_incorrect;
    logic [63:0] perf_jalr_unpredicted;
    logic [63:0] perf_return_predicted_correct;
    logic [63:0] perf_return_predicted_incorrect;
    logic [63:0] perf_return_unpredicted;
    logic [63:0] perf_last_dispatch_cycle;
    logic [63:0] perf_stall_cycles_prev;
    logic perf_first_dispatch;

    always_ff @(posedge clk or negedge rst_l) begin
        if (!rst_l) begin
            perf_cycle_counter = 64'b0;
            perf_dispatch_counter = 64'b0;
            perf_retire_counter = 64'b0;
            perf_frontend_stall_cycles = 64'b0;
            perf_branch_instructions = 64'b0;
            perf_mispredicted_branches = 64'b0;
            perf_alu_instructions = 64'b0;
            perf_load_instructions = 64'b0;
            perf_store_instructions = 64'b0;
            perf_total_data_reads = 64'b0;
            perf_total_data_writes = 64'b0;
            perf_jalr_predicted_correct = 64'b0;
            perf_jalr_predicted_incorrect = 64'b0;
            perf_jalr_unpredicted = 64'b0;
            perf_return_predicted_correct = 64'b0;
            perf_return_predicted_incorrect = 64'b0;
            perf_return_unpredicted = 64'b0;
            perf_last_dispatch_cycle = 64'b0;
            perf_stall_cycles_prev = 64'b0;
            perf_first_dispatch = 1'b1;
            for (int i = 0; i < PERF_STALL_BUCKETS; i += 1) begin
                perf_stall_instr[i] = 64'b0;
            end
            for (int i = 0; i < 16; i += 1) begin
                perf_branch_instr_counter[i] = 64'b0;
            end
            for (int i = 0; i < 8; i += 1) begin
                perf_jal_instr_counter[i] = 64'b0;
                perf_jalr_instr_counter[i] = 64'b0;
            end
        end else if (!halted_q) begin
            perf_cycle_counter = perf_cycle_counter + 64'd1;
            if (frontend_stall || dispatched_unpredicted_control) begin
                perf_frontend_stall_cycles = perf_frontend_stall_cycles + 64'd1;
            end
            if (data_load_en) begin
                perf_total_data_reads = perf_total_data_reads + 64'd1;
            end
            if (data_store_mask != 4'b0) begin
                perf_total_data_writes = perf_total_data_writes + 64'd1;
            end

            for (int i = 0; i < OOO_WIDTH; i += 1) begin
                if (dispatch_valid[i]) begin
                    perf_dispatch_counter = perf_dispatch_counter + 64'd1;
                    if (!perf_first_dispatch) begin
                        perf_stall_cycles_prev = perf_cycle_counter -
                            perf_last_dispatch_cycle - 64'd1;
                        if (perf_stall_cycles_prev < 64'(PERF_STALL_BUCKETS)) begin
                            perf_stall_instr[PERF_STALL_BITS'(
                                    perf_stall_cycles_prev)] =
                                perf_stall_instr[PERF_STALL_BITS'(
                                    perf_stall_cycles_prev)] + 64'd1;
                        end
                    end else begin
                        perf_first_dispatch = 1'b0;
                    end
                    perf_last_dispatch_cycle = perf_cycle_counter;
                end

                if (retire_valid[i]) begin
                    perf_retire_counter = perf_retire_counter + 64'd1;
                    unique case (RISCV_ISA::opcode_t'(active_commit_packet[i].instr[6:0]))
                        RISCV_ISA::OP_OP, RISCV_ISA::OP_IMM: begin
                            perf_alu_instructions = perf_alu_instructions + 64'd1;
                        end
                        RISCV_ISA::OP_LOAD: begin
                            perf_load_instructions = perf_load_instructions + 64'd1;
                        end
                        RISCV_ISA::OP_STORE: begin
                            perf_store_instructions = perf_store_instructions + 64'd1;
                        end
                        default: begin
                        end
                    endcase
                end
            end

            if (branch_writeback.valid && branch_writeback.branch_valid) begin
                perf_branch_instructions = perf_branch_instructions + 64'd1;
                if (branch_writeback.branch_mispredict) begin
                    perf_mispredicted_branches = perf_mispredicted_branches + 64'd1;
                end

                unique case (RISCV_ISA::opcode_t'(branch_writeback.instr[6:0]))
                    RISCV_ISA::OP_BRANCH: begin
                        logic [3:0] branch_idx;
                        branch_idx = {
                            branch_writeback.redirect_pc < branch_writeback.pc,
                            branch_writeback.redirect_pc != (branch_writeback.pc + 32'd4),
                            branch_writeback.control_predicted,
                            branch_writeback.branch_mispredict
                        };
                        perf_branch_instr_counter[branch_idx] =
                            perf_branch_instr_counter[branch_idx] + 64'd1;
                    end
                    RISCV_ISA::OP_JAL: begin
                        logic [2:0] jal_idx;
                        jal_idx = {
                            branch_writeback.instr[11:7] == 5'd1,
                            branch_writeback.control_predicted,
                            branch_writeback.branch_mispredict
                        };
                        perf_jal_instr_counter[jal_idx] =
                            perf_jal_instr_counter[jal_idx] + 64'd1;
                    end
                    RISCV_ISA::OP_JALR: begin
                        logic [2:0] jalr_idx;
                        logic is_return;
                        jalr_idx = {
                            branch_writeback.instr[19:15] == 5'd1,
                            branch_writeback.control_predicted,
                            branch_writeback.branch_mispredict
                        };
                        is_return = (branch_writeback.instr[19:15] == 5'd1) &&
                            (branch_writeback.instr[11:7] == 5'd0);
                        perf_jalr_instr_counter[jalr_idx] =
                            perf_jalr_instr_counter[jalr_idx] + 64'd1;
                        if (branch_writeback.control_predicted &&
                                !branch_writeback.branch_mispredict) begin
                            perf_jalr_predicted_correct =
                                perf_jalr_predicted_correct + 64'd1;
                            if (is_return) begin
                                perf_return_predicted_correct =
                                    perf_return_predicted_correct + 64'd1;
                            end
                        end else if (branch_writeback.control_predicted) begin
                            perf_jalr_predicted_incorrect =
                                perf_jalr_predicted_incorrect + 64'd1;
                            if (is_return) begin
                                perf_return_predicted_incorrect =
                                    perf_return_predicted_incorrect + 64'd1;
                            end
                        end else begin
                            perf_jalr_unpredicted = perf_jalr_unpredicted + 64'd1;
                            if (is_return) begin
                                perf_return_unpredicted =
                                    perf_return_unpredicted + 64'd1;
                            end
                        end
                    end
                    default: begin
                    end
                endcase
            end
        end
    end

    initial begin
        wait (halted);
        $display("FINAL OOO PERFORMANCE COUNTERS:");
        $display("Total cycles: %0d", perf_cycle_counter);
        $display("Instructions dispatched: %0d", perf_dispatch_counter);
        $display("Instructions retired: %0d", perf_retire_counter);
        $display("  ALU instructions: %0d", perf_alu_instructions);
        $display("  Load instructions: %0d", perf_load_instructions);
        $display("  Store instructions: %0d", perf_store_instructions);
        $display("Frontend stall cycles: %0d", perf_frontend_stall_cycles);
        for (int i = 0; i < PERF_STALL_BUCKETS; i += 1) begin
            $display("Dispatched instructions with %0d stalls: %0d", i,
                perf_stall_instr[i]);
        end
        for (int i = 0; i < 16; i += 1) begin
            $display("Branch inst (idx %0d):     %0d", i,
                perf_branch_instr_counter[i]);
        end
        for (int i = 0; i < 8; i += 1) begin
            $display("JAL inst (idx %0d):        %0d", i,
                perf_jal_instr_counter[i]);
            $display("JALR inst (idx %0d):       %0d", i,
                perf_jalr_instr_counter[i]);
        end
        $display("JALR predicted correct: %0d", perf_jalr_predicted_correct);
        $display("JALR predicted incorrect: %0d", perf_jalr_predicted_incorrect);
        $display("JALR unpredicted: %0d", perf_jalr_unpredicted);
        $display("Return predicted correct: %0d", perf_return_predicted_correct);
        $display("Return predicted incorrect: %0d", perf_return_predicted_incorrect);
        $display("Return unpredicted: %0d", perf_return_unpredicted);
        $display("Total data reads: %0d", perf_total_data_reads);
        $display("Total data writes: %0d", perf_total_data_writes);
        $display("Total control flow instructions: %0d", perf_branch_instructions);
        $display("Mispredicted control flow instructions: %0d",
            perf_mispredicted_branches);
    end
`endif /* SIMULATION_18447 */

endmodule: riscv_core_ooo
