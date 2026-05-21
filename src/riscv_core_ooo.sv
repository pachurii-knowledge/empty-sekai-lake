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

    logic [31:0] pc_q, pc_next;
    logic [1:0][31:0] fetch_pc_pipe_q, fetch_pc_pipe_next;
    logic [1:0] fetch_valid_pipe_q, fetch_valid_pipe_next;
    logic halted_q, halted_next;
    logic terminal_pending_q, terminal_pending_next;
    logic frontend_stall;
    logic redirect_valid;
    logic [31:0] redirect_pc;
    logic [31:0] sequential_next_pc;

    decode_lane_t decode_lanes [OOO_WIDTH];
    logic [OOO_WIDTH-1:0] lane_valid;
    logic [OOO_WIDTH-1:0] lane_has_dest;
    logic [OOO_WIDTH-1:0] lane_is_branch;
    logic [OOO_WIDTH-1:0] lane_is_memory;
    logic [OOO_WIDTH-1:0] lane_is_terminal;
    logic [OOO_WIDTH-1:0] dispatch_valid;
    logic [OOO_WIDTH-1:0] alloc_req;
    logic [OOO_WIDTH-1:0] map_has_dest;
    logic dispatch_stall;
    logic [2:0] dispatch_count;
    logic [2:0] valid_count;
    logic [2:0] lane_active_offset [OOO_WIDTH];

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
    logic [OOO_WIDTH-1:0] writeback_exception;
    logic [OOO_WIDTH-1:0] writeback_halted;

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
    assign instr_stall = !rst_l || halted_q || frontend_stall;
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

    ooo_dispatch_control DispatchControl (
        .lane_valid,
        .lane_has_dest,
        .lane_is_branch,
        .lane_is_memory,
        .lane_is_terminal,
        .active_list_full(active_full),
        .int_iq_full(int_iq_full),
        .mem_queue_full(mem_queue_full),
        .branch_stack_full(branch_stack_full),
        .free_list_can_allocate(free_can_allocate),
        .free_list_available(free_count_snapshot),
        .suppress_dispatch(redirect_valid || terminal_pending_q || halted_q),
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
        lane_valid = '0;
        lane_has_dest = '0;
        lane_is_branch = '0;
        lane_is_memory = '0;
        lane_is_terminal = '0;
        alloc_req = '0;
        int_insert_valid = '0;
        mem_insert_valid = '0;
        branch_allocate = 1'b0;
        dispatch_count = '0;
        valid_count = '0;
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
            lane_is_memory[i] = lane_valid[i] &&
                (decode_lanes[i].ctrl.memRead || decode_lanes[i].ctrl.memWrite);
            lane_is_terminal[i] = lane_valid[i] &&
                (decode_lanes[i].ctrl.syscall || decode_lanes[i].ctrl.illegal_instr);
            lane_active_offset[i] = dispatch_count;
            if (lane_valid[i]) begin
                valid_count += 1'b1;
            end
            if (dispatch_valid[i]) begin
                dispatch_count += 1'b1;
            end
        end

        branch_active_tail_snapshot = active_tail + active_id_t'(dispatch_count);
        for (int i = 0; i < OOO_WIDTH; i += 1) begin
            if (dispatch_valid[i]) begin
                if (lane_has_dest[i] && free_alloc_valid[i]) begin
                    branch_map_snapshot[decode_lanes[i].rd] = free_alloc_prd[i];
                    branch_free_head_snapshot = branch_free_head_snapshot + 1'b1;
                    branch_free_count_snapshot = branch_free_count_snapshot - 1'b1;
                end
                if (lane_is_branch[i]) begin
                    branch_active_tail_snapshot = active_tail +
                        active_id_t'(lane_active_offset[i] + 1'b1);
                    break;
                end
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
            if (fetch_valid_pipe_q[1] && (dispatch_count < valid_count)) begin
                pc_next = decode_lanes[dispatch_count].pc;
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
        end
    end

    always_ff @(posedge clk or negedge rst_l) begin
        if (!rst_l) begin
            pc_q <= USER_TEXT_START;
            fetch_pc_pipe_q <= '0;
            fetch_valid_pipe_q <= '0;
            halted_q <= 1'b0;
            terminal_pending_q <= 1'b0;
            abort_mask_q <= '0;
        end else begin
            pc_q <= pc_next;
            fetch_pc_pipe_q <= fetch_pc_pipe_next;
            fetch_valid_pipe_q <= fetch_valid_pipe_next;
            halted_q <= halted_next;
            terminal_pending_q <= terminal_pending_next;
            abort_mask_q <= abort_mask;
        end
    end

endmodule: riscv_core_ooo
