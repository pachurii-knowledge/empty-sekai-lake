`include "ooo_types.vh"

`default_nettype none

module branch_stack
    import OOO_Types::*;
(
    input  logic                         clk,
    input  logic                         rst_l,
    input  logic                         allocate,
    input  active_id_t                   active_tail_snapshot,
    input  logic [$clog2(PHYS_REGS)-1:0] free_head_snapshot,
    input  logic [$clog2(PHYS_REGS)-1:0] free_tail_snapshot,
    input  logic [$clog2(PHYS_REGS+1)-1:0] free_count_snapshot,
    input  phys_reg_t                    map_snapshot [32],
    input  logic                         resolve,
    input  branch_id_t                   resolve_id,
    input  logic                         mispredict,
    // Precise-trap flush: clear all outstanding branch checkpoints (all are
    // younger than the trapping instruction). The architectural map/free state
    // is restored separately by the core's RRAT/committed-free-head rollback.
    input  logic                         flush,
    output logic                         full,
    output logic                         allocate_valid,
    output branch_id_t                   allocate_id,
    output branch_mask_t                 current_mask,
    output logic                         restore_valid,
    output active_id_t                   restore_active_tail,
    output logic [$clog2(PHYS_REGS)-1:0] restore_free_head,
    output logic [$clog2(PHYS_REGS)-1:0] restore_free_tail,
    output logic [$clog2(PHYS_REGS+1)-1:0] restore_free_count,
    output phys_reg_t                    restore_map [32],
    output branch_mask_t                 reset_mask,
    output branch_mask_t                 abort_mask
);

    typedef struct packed {
        logic valid;
        branch_mask_t branch_mask;
        active_id_t active_tail;
        logic [$clog2(PHYS_REGS)-1:0] free_head;
        logic [$clog2(PHYS_REGS)-1:0] free_tail;
        logic [$clog2(PHYS_REGS+1)-1:0] free_count;
    } branch_meta_t;

    branch_meta_t meta_q [BRANCH_STACK_SIZE];
    branch_meta_t meta_next [BRANCH_STACK_SIZE];
    phys_reg_t map_q [BRANCH_STACK_SIZE][32];
    phys_reg_t map_next [BRANCH_STACK_SIZE][32];
    branch_mask_t valid_mask_q, valid_mask_next;
    branch_mask_t resolve_abort_mask;

    assign full = &valid_mask_q;
    assign current_mask = valid_mask_q;

    always_comb begin
        meta_next = meta_q;
        map_next = map_q;
        valid_mask_next = valid_mask_q;
        resolve_abort_mask = '0;
        allocate_valid = 1'b0;
        allocate_id = '0;
        restore_valid = 1'b0;
        restore_active_tail = '0;
        restore_free_head = '0;
        restore_free_tail = '0;
        restore_free_count = '0;
        reset_mask = '0;
        abort_mask = '0;
        for (int i = 0; i < 32; i += 1) begin
            restore_map[i] = '0;
        end

        if (resolve && meta_q[resolve_id].valid) begin
            reset_mask[resolve_id] = 1'b1;
            resolve_abort_mask[resolve_id] = 1'b1;
            if (mispredict) begin
                restore_valid = 1'b1;
                restore_active_tail = meta_q[resolve_id].active_tail;
                restore_free_head = meta_q[resolve_id].free_head;
                restore_free_tail = meta_q[resolve_id].free_tail;
                restore_free_count = meta_q[resolve_id].free_count;
                for (int i = 0; i < 32; i += 1) begin
                    restore_map[i] = map_q[resolve_id][i];
                end
                for (int slot = 0; slot < BRANCH_STACK_SIZE; slot += 1) begin
                    if (meta_q[slot].valid && meta_q[slot].branch_mask[resolve_id]) begin
                        resolve_abort_mask[slot] = 1'b1;
                    end
                end
            end
            abort_mask = mispredict ? resolve_abort_mask : '0;
            for (int slot = 0; slot < BRANCH_STACK_SIZE; slot += 1) begin
                if (resolve_abort_mask[slot]) begin
                    valid_mask_next[slot] = 1'b0;
                    meta_next[slot].valid = 1'b0;
                end else if (meta_next[slot].valid) begin
                    meta_next[slot].branch_mask &= ~resolve_abort_mask;
                end
            end
        end

        if (allocate && !full) begin
            for (int slot = 0; slot < BRANCH_STACK_SIZE; slot += 1) begin
                if (!valid_mask_next[slot] && !allocate_valid) begin
                    allocate_valid = 1'b1;
                    allocate_id = branch_id_t'(slot);
                    meta_next[slot].valid = 1'b1;
                    meta_next[slot].branch_mask = valid_mask_next;
                    valid_mask_next[slot] = 1'b1;
                    meta_next[slot].active_tail = active_tail_snapshot;
                    meta_next[slot].free_head = free_head_snapshot;
                    meta_next[slot].free_tail = free_tail_snapshot;
                    meta_next[slot].free_count = free_count_snapshot;
                    for (int i = 0; i < 32; i += 1) begin
                        map_next[slot][i] = map_snapshot[i];
                    end
                end
            end
        end

        if (flush) begin
            valid_mask_next = '0;
            for (int slot = 0; slot < BRANCH_STACK_SIZE; slot += 1) begin
                meta_next[slot] = '0;
            end
            allocate_valid = 1'b0;
            allocate_id = '0;
            restore_valid = 1'b0;
        end
    end

    always_ff @(posedge clk or negedge rst_l) begin
        if (!rst_l) begin
            valid_mask_q <= '0;
            for (int slot = 0; slot < BRANCH_STACK_SIZE; slot += 1) begin
                meta_q[slot] <= '0;
                for (int i = 0; i < 32; i += 1) begin
                    map_q[slot][i] <= '0;
                end
            end
        end else begin
            valid_mask_q <= valid_mask_next;
            meta_q <= meta_next;
            map_q <= map_next;
        end
    end

endmodule: branch_stack
