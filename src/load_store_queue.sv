`include "ooo_types.vh"

`default_nettype none

module load_store_queue
    import OOO_Types::*;
(
    input  logic                 clk,
    input  logic                 rst_l,
    input  logic [OOO_WIDTH-1:0] insert_valid,
    input  issue_entry_t         insert_entry [OOO_WIDTH],
    input  logic [OOO_WIDTH-1:0][31:0] insert_rs1_data,
    input  logic [OOO_WIDTH-1:0][31:0] insert_rs2_data,
    input  logic [OOO_WIDTH-1:0] wakeup_valid,
    input  phys_reg_t            wakeup_prd [OOO_WIDTH],
    input  logic [OOO_WIDTH-1:0][31:0] wakeup_data,
    input  branch_mask_t         reset_mask,
    input  branch_mask_t         abort_mask,
    input  logic                 data_load_valid,
    input  logic [31:0]          data_load,
    input  logic                 commit_store,
    input  active_id_t           commit_store_id,
    output logic                 full,
    output logic                 data_load_en,
    output logic [29:0]          data_addr,
    output logic [31:0]          data_store,
    output logic [3:0]           data_store_mask,
    output writeback_packet_t    load_writeback
);

    typedef struct packed {
        issue_entry_t entry;
        logic addr_ready;
        logic data_ready;
        logic issued_load;
        logic [31:0] addr;
        logic [31:0] store_data;
        logic [3:0] store_mask;
    } mem_entry_t;

    mem_entry_t entries_q [MEM_Q_SIZE];
    mem_entry_t entries_next [MEM_Q_SIZE];
    logic [$clog2(MEM_Q_SIZE+1)-1:0] count_q, count_next;
    logic [$clog2(MEM_Q_SIZE)-1:0] head_q, head_next;
    logic [$clog2(MEM_Q_SIZE)-1:0] tail_q, tail_next;

    assign full = (count_q > MEM_Q_SIZE - OOO_WIDTH);

    always_comb begin
        entries_next = entries_q;
        count_next = count_q;
        head_next = head_q;
        tail_next = tail_q;
        data_load_en = 1'b0;
        data_addr = '0;
        data_store = '0;
        data_store_mask = '0;
        load_writeback = '0;

        for (int i = 0; i < MEM_Q_SIZE; i += 1) begin
            if ((entries_next[i].entry.branch_mask & abort_mask) != '0) begin
                entries_next[i] = '0;
            end else if (entries_next[i].entry.valid) begin
                entries_next[i].entry.branch_mask &= ~reset_mask;
                for (int w = 0; w < OOO_WIDTH; w += 1) begin
                    if (wakeup_valid[w]) begin
                        if (entries_next[i].entry.prs1 == wakeup_prd[w]) begin
                            entries_next[i].entry.src1_ready = 1'b1;
                            entries_next[i].addr_ready = 1'b1;
                            entries_next[i].addr = wakeup_data[w] +
                                entries_next[i].entry.imm;
                        end
                        if (entries_next[i].entry.prs2 == wakeup_prd[w]) begin
                            entries_next[i].entry.src2_ready = 1'b1;
                            entries_next[i].data_ready = 1'b1;
                            format_store(entries_next[i].entry.ctrl.ldst_mode,
                                entries_next[i].addr[1:0],
                                wakeup_data[w],
                                entries_next[i].store_data,
                                entries_next[i].store_mask);
                        end
                    end
                end
            end
        end

        for (int i = 0; i < MEM_Q_SIZE; i += 1) begin
            if ((count_next != '0) && !entries_next[head_next].entry.valid) begin
                head_next = head_next + 1'b1;
                count_next -= 1'b1;
            end
        end

        if (entries_next[head_next].entry.valid &&
                entries_next[head_next].entry.ctrl.memRead &&
                entries_next[head_next].entry.src1_ready && !entries_next[head_next].issued_load) begin
            data_load_en = 1'b1;
            data_addr = entries_next[head_next].addr[31:2];
            entries_next[head_next].issued_load = 1'b1;
        end

        if (entries_next[head_next].entry.valid &&
                entries_next[head_next].entry.ctrl.memWrite &&
                entries_next[head_next].entry.src1_ready &&
                entries_next[head_next].entry.src2_ready &&
                !entries_next[head_next].issued_load) begin
            load_writeback.valid = 1'b1;
            load_writeback.active_id = entries_next[head_next].entry.active_id;
            load_writeback.branch_mask = entries_next[head_next].entry.branch_mask;
            load_writeback.has_dest = 1'b0;
            entries_next[head_next].issued_load = 1'b1;
        end

        if (entries_next[head_next].entry.valid &&
                entries_next[head_next].entry.ctrl.memRead &&
                entries_next[head_next].issued_load && data_load_valid) begin
            load_writeback.valid = 1'b1;
            load_writeback.active_id = entries_next[head_next].entry.active_id;
            load_writeback.prd = entries_next[head_next].entry.prd;
            load_writeback.has_dest = entries_next[head_next].entry.has_dest;
            load_writeback.branch_mask = entries_next[head_next].entry.branch_mask;
            load_writeback.data = format_load(data_load,
                entries_next[head_next].addr[1:0],
                entries_next[head_next].entry.ctrl.ldst_mode);
            entries_next[head_next] = '0;
            head_next = head_next + 1'b1;
            count_next -= 1'b1;
        end

        if (commit_store && entries_next[head_next].entry.valid &&
                entries_next[head_next].entry.ctrl.memWrite &&
                (entries_next[head_next].entry.active_id == commit_store_id)) begin
            data_addr = entries_next[head_next].addr[31:2];
            data_store = entries_next[head_next].store_data;
            data_store_mask = entries_next[head_next].store_mask;
            entries_next[head_next] = '0;
            head_next = head_next + 1'b1;
            count_next -= 1'b1;
        end

        if (!full) begin
            for (int lane = 0; lane < OOO_WIDTH; lane += 1) begin
                if (insert_valid[lane]) begin
                    entries_next[tail_next].entry = insert_entry[lane];
                    entries_next[tail_next].entry.valid = 1'b1;
                    entries_next[tail_next].addr_ready = insert_entry[lane].src1_ready;
                    entries_next[tail_next].data_ready = insert_entry[lane].src2_ready;
                    entries_next[tail_next].issued_load = 1'b0;
                    if (insert_entry[lane].src1_ready) begin
                        entries_next[tail_next].addr = insert_rs1_data[lane] +
                            insert_entry[lane].imm;
                    end else begin
                        entries_next[tail_next].addr = '0;
                    end
                    if (insert_entry[lane].src2_ready) begin
                        format_store(insert_entry[lane].ctrl.ldst_mode,
                            entries_next[tail_next].addr[1:0],
                            insert_rs2_data[lane],
                            entries_next[tail_next].store_data,
                            entries_next[tail_next].store_mask);
                    end else begin
                        entries_next[tail_next].store_data = '0;
                        entries_next[tail_next].store_mask = '0;
                    end
                    tail_next = tail_next + 1'b1;
                    count_next += 1'b1;
                end
            end
        end
    end

    task automatic format_store(input ldst_mode_t mode,
            input logic [1:0] byte_sel,
            input logic [31:0] value,
            output logic [31:0] store_value,
            output logic [3:0] store_mask);
        store_value = 32'b0;
        store_mask = 4'b0;
        unique case (mode)
            LDST_W: begin
                store_value = value;
                store_mask = 4'b1111;
            end
            LDST_H, LDST_HU: begin
                store_value = byte_sel[1] ? {value[15:0], 16'b0} :
                    {16'b0, value[15:0]};
                store_mask = byte_sel[1] ? 4'b1100 : 4'b0011;
            end
            LDST_B, LDST_BU: begin
                store_value = {4{value[7:0]}} << ({3'b0, byte_sel} * 5'd8);
                store_mask = 4'b0001 << byte_sel;
            end
            default: begin
                store_value = 32'b0;
                store_mask = 4'b0;
            end
        endcase
    endtask

    function automatic logic [31:0] format_load(input logic [31:0] raw_word,
            input logic [1:0] byte_sel, input ldst_mode_t mode);
        unique case (mode)
            LDST_W: format_load = raw_word;
            LDST_H: format_load = byte_sel[1] ?
                {{16{raw_word[31]}}, raw_word[31:16]} :
                {{16{raw_word[15]}}, raw_word[15:0]};
            LDST_HU: format_load = byte_sel[1] ?
                {16'b0, raw_word[31:16]} : {16'b0, raw_word[15:0]};
            LDST_B: begin
                unique case (byte_sel)
                    2'd0: format_load = {{24{raw_word[7]}}, raw_word[7:0]};
                    2'd1: format_load = {{24{raw_word[15]}}, raw_word[15:8]};
                    2'd2: format_load = {{24{raw_word[23]}}, raw_word[23:16]};
                    default: format_load = {{24{raw_word[31]}}, raw_word[31:24]};
                endcase
            end
            LDST_BU: begin
                unique case (byte_sel)
                    2'd0: format_load = {24'b0, raw_word[7:0]};
                    2'd1: format_load = {24'b0, raw_word[15:8]};
                    2'd2: format_load = {24'b0, raw_word[23:16]};
                    default: format_load = {24'b0, raw_word[31:24]};
                endcase
            end
            default: format_load = 32'b0;
        endcase
    endfunction

    always_ff @(posedge clk or negedge rst_l) begin
        if (!rst_l) begin
            count_q <= '0;
            head_q <= '0;
            tail_q <= '0;
            for (int i = 0; i < MEM_Q_SIZE; i += 1) begin
                entries_q[i] <= '0;
            end
        end else begin
            count_q <= count_next;
            head_q <= head_next;
            tail_q <= tail_next;
            entries_q <= entries_next;
        end
    end

endmodule: load_store_queue
