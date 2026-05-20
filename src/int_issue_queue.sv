`include "ooo_types.vh"

`default_nettype none

module int_issue_queue
    import OOO_Types::*;
(
    input  logic                 clk,
    input  logic                 rst_l,
    input  logic [OOO_WIDTH-1:0] insert_valid,
    input  issue_entry_t         insert_entry [OOO_WIDTH],
    input  logic [OOO_WIDTH-1:0] wakeup_valid,
    input  phys_reg_t            wakeup_prd [OOO_WIDTH],
    input  branch_mask_t         reset_mask,
    input  branch_mask_t         abort_mask,
    output logic                 full,
    output logic [1:0]           issue_valid,
    output issue_entry_t         issue_entry [2]
);

    issue_entry_t entries_q [INT_IQ_SIZE];
    issue_entry_t entries_next [INT_IQ_SIZE];
    logic [$clog2(INT_IQ_SIZE+1)-1:0] count_q, count_next;
    logic [INT_IQ_SIZE-1:0] selected;
    logic [$clog2(OOO_WIDTH+1)-1:0] insert_count;

    assign full = (count_q > INT_IQ_SIZE - OOO_WIDTH);

    always_comb begin
        insert_count = '0;
        for (int i = 0; i < OOO_WIDTH; i += 1) begin
            insert_count += insert_valid[i];
        end
    end

    always_comb begin
        entries_next = entries_q;
        count_next = count_q;
        issue_valid = '0;
        selected = '0;
        for (int i = 0; i < 2; i += 1) begin
            issue_entry[i] = '0;
        end

        for (int i = 0; i < INT_IQ_SIZE; i += 1) begin
            entries_next[i].branch_mask &= ~reset_mask;
            if ((entries_next[i].branch_mask & abort_mask) != '0) begin
                entries_next[i] = '0;
                count_next -= 1'b1;
            end else if (entries_next[i].valid) begin
                for (int w = 0; w < OOO_WIDTH; w += 1) begin
                    if (wakeup_valid[w]) begin
                        if (entries_next[i].prs1 == wakeup_prd[w]) begin
                            entries_next[i].src1_ready = 1'b1;
                        end
                        if (entries_next[i].prs2 == wakeup_prd[w]) begin
                            entries_next[i].src2_ready = 1'b1;
                        end
                    end
                end
            end
        end

        for (int port = 0; port < 2; port += 1) begin
            for (int i = 0; i < INT_IQ_SIZE; i += 1) begin
                if (!issue_valid[port] && entries_next[i].valid &&
                        entries_next[i].src1_ready && entries_next[i].src2_ready &&
                        !selected[i]) begin
                    issue_valid[port] = 1'b1;
                    issue_entry[port] = entries_next[i];
                    selected[i] = 1'b1;
                    entries_next[i] = '0;
                    count_next -= 1'b1;
                end
            end
        end

        if (!full) begin
            for (int lane = 0; lane < OOO_WIDTH; lane += 1) begin
                if (insert_valid[lane]) begin
                    for (int i = 0; i < INT_IQ_SIZE; i += 1) begin
                        if (!entries_next[i].valid) begin
                            entries_next[i] = insert_entry[lane];
                            entries_next[i].valid = 1'b1;
                            count_next += 1'b1;
                            break;
                        end
                    end
                end
            end
        end
    end

    always_ff @(posedge clk or negedge rst_l) begin
        if (!rst_l) begin
            count_q <= '0;
            for (int i = 0; i < INT_IQ_SIZE; i += 1) begin
                entries_q[i] <= '0;
            end
        end else begin
            entries_q <= entries_next;
            count_q <= count_next;
        end
    end

endmodule: int_issue_queue
