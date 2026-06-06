`include "ooo_types.vh"

`default_nettype none

module active_list
    import OOO_Types::*;
(
    input  logic                  clk,
    input  logic                  rst_l,
    input  logic                  restore_valid,
    input  active_id_t            restore_tail,
    // Full pipeline flush on a precise trap / interrupt / trap-return. Unlike a
    // branch restore (which rolls back to a checkpoint), this squashes every
    // entry still in flight behind the committing (trapping) instruction. The
    // trapping instruction itself still commits this cycle via the loop above;
    // `flush` then discards everything younger.
    input  logic                  flush,
    input  logic [OOO_WIDTH-1:0]  allocate_valid,
    input  rename_packet_t        allocate_packet [OOO_WIDTH],
    input  logic [OOO_WIDTH-1:0]  writeback_valid,
    input  active_id_t            writeback_id [OOO_WIDTH],
    input  logic [OOO_WIDTH-1:0][31:0] writeback_data,
    input  logic [OOO_WIDTH-1:0]  writeback_exception,
    input  logic [OOO_WIDTH-1:0][4:0] writeback_exc_cause,
    input  logic [OOO_WIDTH-1:0]  writeback_halted,
    input  logic [OOO_WIDTH-1:0]  writeback_fp_write,
    input  arch_reg_t             writeback_fp_rd [OOO_WIDTH],
    input  fp_reg_data_t          writeback_fp_data [OOO_WIDTH],
    input  logic [OOO_WIDTH-1:0]  writeback_csr_write,
    input  logic [OOO_WIDTH-1:0][11:0] writeback_csr_addr,
    input  logic [OOO_WIDTH-1:0][31:0] writeback_csr_wdata,
    input  logic [OOO_WIDTH-1:0]  writeback_fp_fflags_valid,
    input  logic [OOO_WIDTH-1:0][4:0] writeback_fp_fflags,
    input  branch_mask_t          reset_mask,
    input  branch_mask_t          abort_mask,
    output logic                  full,
    output logic                  empty,
    output active_id_t            tail,
    output logic [OOO_WIDTH-1:0]  commit_valid,
    output commit_packet_t        commit_packet [OOO_WIDTH],
    output logic [OOO_WIDTH-1:0]  free_valid,
    output phys_reg_t             free_prd [OOO_WIDTH]
);

    typedef struct packed {
        logic valid;
        logic done;
        logic exception;
        logic [4:0] exc_cause;
        logic halted;
        logic [31:0] pc;
        logic [31:0] instr;
        logic [31:0] data;
        logic fp_write;
        arch_reg_t fp_rd;
        fp_reg_data_t fp_data;
        logic csr_write;
        logic [11:0] csr_addr;
        logic [31:0] csr_wdata;
        logic fp_fflags_valid;
        logic [4:0] fp_fflags;
        logic serializing;
        arch_reg_t rd;
        phys_reg_t prd;
        phys_reg_t old_prd;
        logic has_dest;
        logic is_store;
        branch_mask_t branch_mask;
    } active_entry_t;

    typedef logic [$clog2(ACTIVE_LIST_SIZE+1)-1:0] active_count_t;

    active_entry_t entries_q [ACTIVE_LIST_SIZE];
    active_entry_t entries_next [ACTIVE_LIST_SIZE];
    active_id_t head_q, head_next;
    active_id_t tail_q, tail_next;
    active_count_t count_q, count_next;
    logic [$clog2(OOO_WIDTH+1)-1:0] alloc_count;
    int commit_count;

    assign tail = tail_q;
    assign full = (count_q > ACTIVE_LIST_SIZE - OOO_WIDTH);
    assign empty = (count_q == '0);

    always_comb begin
        alloc_count = '0;
        for (int i = 0; i < OOO_WIDTH; i += 1) begin
            alloc_count += allocate_valid[i];
        end
    end

    always_comb begin
        entries_next = entries_q;
        head_next = head_q;
        tail_next = restore_valid ? restore_tail : tail_q;
        count_next = restore_valid ? active_distance(head_q, restore_tail) : count_q;
        commit_valid = '0;
        free_valid = '0;
        commit_count = 0;
        for (int i = 0; i < OOO_WIDTH; i += 1) begin
            commit_packet[i] = '0;
            free_prd[i] = '0;
        end

        for (int i = 0; i < ACTIVE_LIST_SIZE; i += 1) begin
            if ((entries_next[i].branch_mask & abort_mask) != '0) begin
                entries_next[i] = '0;
            end else if (entries_next[i].valid) begin
                entries_next[i].branch_mask &= ~reset_mask;
            end
        end

        for (int i = 0; i < ACTIVE_LIST_SIZE; i += 1) begin
            if ((count_next != '0) && !entries_next[head_next].valid) begin
                head_next = head_next + 1'b1;
                count_next -= 1'b1;
            end
        end

        for (int i = 0; i < OOO_WIDTH; i += 1) begin
            if (writeback_valid[i]) begin
                entries_next[writeback_id[i]].done = 1'b1;
                entries_next[writeback_id[i]].data = writeback_data[i];
                entries_next[writeback_id[i]].fp_write = writeback_fp_write[i];
                entries_next[writeback_id[i]].fp_rd = writeback_fp_rd[i];
                entries_next[writeback_id[i]].fp_data = writeback_fp_data[i];
                entries_next[writeback_id[i]].csr_write = writeback_csr_write[i];
                entries_next[writeback_id[i]].csr_addr = writeback_csr_addr[i];
                entries_next[writeback_id[i]].csr_wdata = writeback_csr_wdata[i];
                entries_next[writeback_id[i]].fp_fflags_valid =
                    writeback_fp_fflags_valid[i];
                entries_next[writeback_id[i]].fp_fflags = writeback_fp_fflags[i];
                if (writeback_exception[i] && !entries_next[writeback_id[i]].exception)
                    entries_next[writeback_id[i]].exc_cause = writeback_exc_cause[i];
                entries_next[writeback_id[i]].exception |= writeback_exception[i];
                entries_next[writeback_id[i]].halted |= writeback_halted[i];
            end
        end

        for (int i = 0; i < OOO_WIDTH; i += 1) begin
            if ((count_next != '0) && entries_next[head_next].valid &&
                    entries_q[head_next].done) begin
                commit_valid[i] = 1'b1;
                commit_packet[i].valid = 1'b1;
                commit_packet[i].active_id = head_next;
                commit_packet[i].rd = entries_next[head_next].rd;
                commit_packet[i].prd = entries_next[head_next].prd;
                commit_packet[i].old_prd = entries_next[head_next].old_prd;
                commit_packet[i].has_dest = entries_next[head_next].has_dest;
                commit_packet[i].pc = entries_next[head_next].pc;
                commit_packet[i].instr = entries_next[head_next].instr;
                commit_packet[i].data = entries_next[head_next].data;
                commit_packet[i].fp_write = entries_next[head_next].fp_write;
                commit_packet[i].fp_rd = entries_next[head_next].fp_rd;
                commit_packet[i].fp_data = entries_next[head_next].fp_data;
                commit_packet[i].csr_write = entries_next[head_next].csr_write;
                commit_packet[i].csr_addr = entries_next[head_next].csr_addr;
                commit_packet[i].csr_wdata = entries_next[head_next].csr_wdata;
                commit_packet[i].fp_fflags_valid =
                    entries_next[head_next].fp_fflags_valid;
                commit_packet[i].fp_fflags = entries_next[head_next].fp_fflags;
                commit_packet[i].serializing = entries_next[head_next].serializing;
                commit_packet[i].is_store = entries_next[head_next].is_store;
                commit_packet[i].halted = entries_next[head_next].halted;
                commit_packet[i].exception = entries_next[head_next].exception;
                commit_packet[i].exc_cause = entries_next[head_next].exc_cause;
                // An excepting instruction discards its result: its rd keeps the
                // old mapping (restored from the architectural map on flush), so
                // old_prd must NOT be freed here. Its freshly allocated prd is
                // reclaimed by rolling the free-list head back to the committed
                // head instead.
                free_valid[i] = entries_next[head_next].has_dest &&
                    !entries_next[head_next].exception;
                free_prd[i] = entries_next[head_next].old_prd;
                entries_next[head_next] = '0;
                head_next = head_next + 1'b1;
                count_next -= 1'b1;
                commit_count += 1;
                if (commit_packet[i].halted || commit_packet[i].exception) begin
                    break;
                end
            end
        end

        if (!restore_valid && !full) begin
            for (int i = 0; i < OOO_WIDTH; i += 1) begin
                if (allocate_valid[i]) begin
                    entries_next[tail_next].valid = 1'b1;
                    entries_next[tail_next].done = 1'b0;
                    entries_next[tail_next].exception = 1'b0;
                    entries_next[tail_next].exc_cause = 5'd0;
                    entries_next[tail_next].halted = 1'b0;
                    entries_next[tail_next].data = '0;
                    entries_next[tail_next].fp_write = 1'b0;
                    entries_next[tail_next].fp_rd = allocate_packet[i].fp_rd;
                    entries_next[tail_next].fp_data = '0;
                    entries_next[tail_next].csr_write = 1'b0;
                    entries_next[tail_next].csr_addr = allocate_packet[i].instr[31:20];
                    entries_next[tail_next].csr_wdata = '0;
                    entries_next[tail_next].fp_fflags_valid = 1'b0;
                    entries_next[tail_next].fp_fflags = '0;
                    entries_next[tail_next].serializing =
                        allocate_packet[i].ctrl.serializing;
                    entries_next[tail_next].pc = allocate_packet[i].pc;
                    entries_next[tail_next].instr = allocate_packet[i].instr;
                    entries_next[tail_next].rd = allocate_packet[i].rd;
                    entries_next[tail_next].prd = allocate_packet[i].prd;
                    entries_next[tail_next].old_prd = allocate_packet[i].old_prd;
                    entries_next[tail_next].has_dest = allocate_packet[i].has_dest;
                    entries_next[tail_next].is_store = allocate_packet[i].ctrl.memWrite;
                    entries_next[tail_next].branch_mask = allocate_packet[i].branch_mask;
                    tail_next = tail_next + 1'b1;
                    count_next += 1'b1;
                end
            end
        end

        // Trap flush: discard everything still in flight behind the trapping
        // instruction (which already committed via the loop above, advancing
        // head_next past it). Reset the queue to empty at the post-commit head.
        if (flush) begin
            for (int i = 0; i < ACTIVE_LIST_SIZE; i += 1) begin
                entries_next[i] = '0;
            end
            tail_next = head_next;
            count_next = '0;
        end
    end

    function automatic active_count_t active_distance(input active_id_t from_id,
            input active_id_t to_id);
        if (to_id >= from_id) begin
            active_distance = active_count_t'(to_id - from_id);
        end else begin
            active_distance = active_count_t'(ACTIVE_LIST_SIZE - from_id + to_id);
        end
    endfunction

    always_ff @(posedge clk or negedge rst_l) begin
        if (!rst_l) begin
            head_q <= '0;
            tail_q <= '0;
            count_q <= '0;
            for (int i = 0; i < ACTIVE_LIST_SIZE; i += 1) begin
                entries_q[i] <= '0;
            end
        end else begin
            entries_q <= entries_next;
            head_q <= head_next;
            tail_q <= tail_next;
            count_q <= count_next;
        end
    end

endmodule: active_list
