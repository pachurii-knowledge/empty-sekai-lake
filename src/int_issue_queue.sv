`include "ooo_types.vh"

`default_nettype none

module int_issue_queue
    import OOO_Types::*;
(
    input wire logic                 clk,
    input wire logic                 rst_l,
    input wire logic [OOO_WIDTH-1:0] insert_valid,
    input wire issue_entry_t         insert_entry [OOO_WIDTH],
    input wire logic [OOO_WIDTH-1:0] wakeup_valid,
    input wire phys_reg_t            wakeup_prd [OOO_WIDTH],
    // Speculative wakeup: an ALU producer in EXECUTE (S2) broadcasts its dest one
    // cycle before its writeback bus appears, so a dependent ALU consumer can issue
    // back-to-back (zero bubble) across the new select->execute pipeline register.
    // Applied to FU_ALU consumers ONLY (see the wakeup loop) -- they read operands
    // in their own S2 (select+1), where the producer's writeback is bypassed.
    input wire logic [ALU_ISSUE_PORTS-1:0] spec_wake_valid,
    input wire phys_reg_t            spec_wake_prd [ALU_ISSUE_PORTS],
    input wire logic [FU_ISSUE_PORTS-1:0] issue_ready,
    input wire branch_mask_t         reset_mask,
    input wire branch_mask_t         abort_mask,
    // Full pipeline flush on a precise trap / interrupt / trap-return: discard
    // every queued instruction (all are younger than the trapping instruction).
    input wire logic                 flush,
    output logic                 full,
    output logic [FU_ISSUE_PORTS-1:0] issue_valid,
    output issue_entry_t         issue_entry [FU_ISSUE_PORTS]
);

    issue_entry_t entries_q [INT_IQ_SIZE];
    issue_entry_t entries_next [INT_IQ_SIZE];
    logic [$clog2(INT_IQ_SIZE+1)-1:0] count_q, count_next;
    logic [INT_IQ_SIZE-1:0] selected;
    logic [$clog2(OOO_WIDTH+1)-1:0] insert_count;
    logic branch_issued;
    logic branch_issue_blocked;

    assign full = (count_q > INT_IQ_SIZE - OOO_WIDTH);

    always_comb begin
        insert_count = '0;
        for (int i = 0; i < OOO_WIDTH; i += 1) begin
            insert_count += insert_valid[i];
        end
    end

    always_comb begin
        entries_next = entries_q;
        issue_valid = '0;
        selected = '0;
        branch_issued = 1'b0;
        branch_issue_blocked = 1'b0;
        for (int i = 0; i < FU_ISSUE_PORTS; i += 1) begin
            issue_entry[i] = '0;
        end

        for (int i = 0; i < INT_IQ_SIZE; i += 1) begin
            if ((entries_next[i].branch_mask & abort_mask) != '0) begin
                entries_next[i] = '0;
            end else if (entries_next[i].valid) begin
                entries_next[i].branch_mask &= ~reset_mask;
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
                // Speculative ALU wakeup -> FU_ALU consumers only. An ALU consumer
                // reads its operands in its own execute stage (select+1), where the
                // spec-woken producer's writeback is already on the bus (bypassed in
                // the phys_reg_file). MUL/DIV/FP read operands at select (no execute
                // register) so the value is NOT yet available -> they wake only on
                // the completion wakeup above. spec_wake_prd is a freshly allocated
                // dest (has_dest, prd != 0), so it never spuriously matches prs == 0.
                if (entries_next[i].fu_class == FU_ALU) begin
                    for (int w = 0; w < ALU_ISSUE_PORTS; w += 1) begin
                        if (spec_wake_valid[w]) begin
                            if (entries_next[i].prs1 == spec_wake_prd[w]) begin
                                entries_next[i].src1_ready = 1'b1;
                            end
                            if (entries_next[i].prs2 == spec_wake_prd[w]) begin
                                entries_next[i].src2_ready = 1'b1;
                            end
                        end
                    end
                end
            end
        end

        for (int port = 0; port < FU_ISSUE_PORTS; port += 1) begin
            for (int i = 0; i < INT_IQ_SIZE; i += 1) begin
                if (issue_ready[port] && !issue_valid[port] &&
                        entries_next[i].valid &&
                        entries_next[i].src1_ready && entries_next[i].src2_ready &&
                        !selected[i] && port_accepts_entry(port, entries_next[i])) begin
                    if (is_control_flow(entries_next[i]) &&
                            (entries_next[i].branch_mask != '0)) begin
                        branch_issue_blocked = 1'b1;
                        continue;
                    end
                    if (branch_issued && is_control_flow(entries_next[i])) begin
                        branch_issue_blocked = 1'b1;
                        continue;
                    end
                    issue_valid[port] = 1'b1;
                    issue_entry[port] = entries_next[i];
                    branch_issued |= is_control_flow(entries_next[i]);
                    selected[i] = 1'b1;
                    entries_next[i] = '0;
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
                            break;
                        end
                    end
                end
            end
        end

        if (flush) begin
            for (int i = 0; i < INT_IQ_SIZE; i += 1) begin
                entries_next[i] = '0;
            end
            issue_valid = '0;
            for (int i = 0; i < FU_ISSUE_PORTS; i += 1) begin
                issue_entry[i] = '0;
            end
        end

        // Queue occupancy = popcount of the valid bits in the final next-state
        // (flush zeroes every entry -> 0). Replaces the serial -1/+1 RMW that was
        // threaded through squash/issue/insert, which put count_next on the deep
        // serial select->count chain (the abort_mask -> count_q worst path). The
        // sum of 1-bit valids synthesizes to a shallow popcount adder tree.
        count_next = '0;
        for (int i = 0; i < INT_IQ_SIZE; i += 1) begin
            count_next += entries_next[i].valid;
        end
    end

    function automatic logic is_control_flow(issue_entry_t entry);
        is_control_flow = (entry.ctrl.pc_source == PC_cond) ||
            (entry.ctrl.pc_source == PC_uncond) ||
            (entry.ctrl.pc_source == PC_indirect);
    endfunction

    function automatic logic port_accepts_entry(input int port,
            input issue_entry_t entry);
        unique case (port)
            ISSUE_ALU0, ISSUE_ALU1: port_accepts_entry = entry.fu_class == FU_ALU;
            ISSUE_MUL: port_accepts_entry = entry.fu_class == FU_MUL;
            ISSUE_DIV: port_accepts_entry = entry.fu_class == FU_DIV;
            ISSUE_FP: port_accepts_entry = entry.fu_class == FU_FP;
            default: port_accepts_entry = 1'b0;
        endcase
    endfunction

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
