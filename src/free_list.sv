`include "ooo_types.vh"

`default_nettype none

module free_list
    import OOO_Types::*;
(
    input  logic                         clk,
    input  logic                         rst_l,
    input  logic                         restore_valid,
    input  logic [$clog2(PHYS_REGS)-1:0] restore_head,
    input  logic [$clog2(PHYS_REGS)-1:0] restore_tail,
    input  logic [$clog2(PHYS_REGS+1)-1:0] restore_count,
    input  logic [OOO_WIDTH-1:0]         alloc_req,
    input  logic [OOO_WIDTH-1:0]         free_valid,
    input  phys_reg_t                    free_prd [OOO_WIDTH],
    output logic [OOO_WIDTH-1:0]         alloc_valid,
    output phys_reg_t                    alloc_prd [OOO_WIDTH],
    output logic                         can_allocate,
    output logic [$clog2(PHYS_REGS)-1:0] snapshot_head,
    output logic [$clog2(PHYS_REGS)-1:0] snapshot_tail,
    output logic [$clog2(PHYS_REGS+1)-1:0] snapshot_count
);

    phys_reg_t entries_q [PHYS_REGS];
    phys_reg_t entries_next [PHYS_REGS];
    logic [$clog2(PHYS_REGS)-1:0] head_q, head_next;
    logic [$clog2(PHYS_REGS)-1:0] tail_q, tail_next;
    logic [$clog2(PHYS_REGS+1)-1:0] count_q, count_next;
    logic [$clog2(PHYS_REGS+1)-1:0] restore_reclaim_count;
    logic [$clog2(OOO_WIDTH+1)-1:0] alloc_count;
    logic [$clog2(OOO_WIDTH+1)-1:0] free_count;

    assign snapshot_head = head_q;
    assign snapshot_tail = tail_q;
    assign snapshot_count = count_q;

    always_comb begin
        alloc_count = '0;
        free_count = '0;
        for (int i = 0; i < OOO_WIDTH; i += 1) begin
            alloc_count += alloc_req[i];
            free_count += free_valid[i];
        end
        restore_reclaim_count = free_distance(restore_head, head_q);
        can_allocate = (count_q >= OOO_WIDTH);
    end

    always_comb begin
        entries_next = entries_q;
        head_next = restore_valid ? restore_head : head_q;
        tail_next = tail_q;
        count_next = restore_valid ?
            (count_q + restore_reclaim_count) : count_q;
        alloc_valid = '0;
        for (int i = 0; i < OOO_WIDTH; i += 1) begin
            alloc_prd[i] = '0;
        end

        if (!restore_valid && can_allocate) begin
            for (int i = 0; i < OOO_WIDTH; i += 1) begin
                if (alloc_req[i]) begin
                    alloc_valid[i] = 1'b1;
                    alloc_prd[i] = entries_next[head_next];
                    head_next = head_next + 1'b1;
                    count_next = count_next - 1'b1;
                end
            end
        end

        for (int i = 0; i < OOO_WIDTH; i += 1) begin
            if (free_valid[i] && (free_prd[i] != '0)) begin
                entries_next[tail_next] = free_prd[i];
                tail_next = tail_next + 1'b1;
                count_next = count_next + 1'b1;
            end
        end
    end

    function automatic logic [$clog2(PHYS_REGS+1)-1:0] free_distance(
            input logic [$clog2(PHYS_REGS)-1:0] from_ptr,
            input logic [$clog2(PHYS_REGS)-1:0] to_ptr);
        if (to_ptr >= from_ptr) begin
            free_distance = logic'($clog2(PHYS_REGS+1))'(to_ptr - from_ptr);
        end else begin
            free_distance = logic'($clog2(PHYS_REGS+1))'(PHYS_REGS - from_ptr + to_ptr);
        end
    endfunction

    always_ff @(posedge clk or negedge rst_l) begin
        if (!rst_l) begin
            head_q <= '0;
            tail_q <= PHYS_REGS - 32;
            count_q <= PHYS_REGS - 32;
            for (int i = 0; i < PHYS_REGS; i += 1) begin
                entries_q[i] <= phys_reg_t'(i + 32);
            end
        end else begin
            entries_q <= entries_next;
            head_q <= head_next;
            tail_q <= tail_next;
            count_q <= count_next;
        end
    end

endmodule: free_list
