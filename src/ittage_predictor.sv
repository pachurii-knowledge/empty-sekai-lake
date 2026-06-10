`include "ooo_types.vh"

`default_nettype none

module ittage_predictor
    import OOO_Types::*;
#(
    parameter int INDEX_BITS = 10,
    parameter int TAG_BITS = 10,
    parameter int HISTORY_BITS = 30
) (
    input  logic            clk,
    input  logic            rst_l,
    input  logic            lookup_valid,
    input  logic [XLEN-1:0] lookup_pc,
    output logic            prediction_valid,
    output logic [XLEN-1:0] prediction_target,
    output predictor_info_t prediction_info,
    input  logic            update_valid,
    input  logic [XLEN-1:0] update_target,
    input  predictor_info_t update_info
);

    localparam int ENTRIES = 1 << INDEX_BITS;

    typedef logic [1:0] confidence_counter_t;
    typedef logic [1:0] useful_counter_t;

    logic [HISTORY_BITS-1:0] history_q;
    logic [XLEN-1:0] base_target [ENTRIES];
    logic base_valid [ENTRIES];
    logic [XLEN-1:0] tage_target [3][ENTRIES];
    confidence_counter_t tage_confidence [3][ENTRIES];
    useful_counter_t tage_useful [3][ENTRIES];
    logic [TAG_BITS-1:0] tage_tag [3][ENTRIES];

    logic [INDEX_BITS-1:0] base_index;
    logic [INDEX_BITS-1:0] idx [3];
    logic [TAG_BITS-1:0] tag [3];
    logic hit [3];
    logic [1:0] provider;

    always_comb begin
        base_index = lookup_pc[INDEX_BITS+1:2];
        idx[0] = lookup_pc[INDEX_BITS+1:2] ^ history_q[INDEX_BITS-1:0];
        idx[1] = lookup_pc[INDEX_BITS+1:2] ^
            history_q[INDEX_BITS-1:0] ^ history_q[2*INDEX_BITS-1:INDEX_BITS];
        idx[2] = lookup_pc[INDEX_BITS+1:2] ^
            history_q[INDEX_BITS-1:0] ^ history_q[2*INDEX_BITS-1:INDEX_BITS] ^
            history_q[3*INDEX_BITS-1:2*INDEX_BITS];
        tag[0] = lookup_pc[TAG_BITS+1:2] ^ history_q[TAG_BITS-1:0];
        tag[1] = lookup_pc[TAG_BITS+1:2] ^
            history_q[TAG_BITS-1:0] ^ history_q[2*TAG_BITS-1:TAG_BITS];
        tag[2] = lookup_pc[TAG_BITS+1:2] ^
            history_q[TAG_BITS-1:0] ^ history_q[2*TAG_BITS-1:TAG_BITS] ^
            history_q[3*TAG_BITS-1:2*TAG_BITS];

        for (int i = 0; i < 3; i += 1) begin
            hit[i] = (tage_tag[i][idx[i]] == tag[i]) &&
                (tage_confidence[i][idx[i]] != 2'b00);
        end

        provider = 2'd0;
        if (hit[0]) begin
            provider = 2'd1;
        end
        if (hit[1]) begin
            provider = 2'd2;
        end
        if (hit[2]) begin
            provider = 2'd3;
        end

        prediction_valid = lookup_valid &&
            ((provider != 2'd0) || base_valid[base_index]);
        unique case (provider)
            2'd3: prediction_target = tage_target[2][idx[2]];
            2'd2: prediction_target = tage_target[1][idx[1]];
            2'd1: prediction_target = tage_target[0][idx[0]];
            default: prediction_target = base_target[base_index];
        endcase

        prediction_info = '0;
        prediction_info.valid = lookup_valid;
        prediction_info.predicted_taken = prediction_valid;
        prediction_info.predicted_target_valid = prediction_valid;
        prediction_info.predicted_target = prediction_target;
        prediction_info.provider = provider;
        prediction_info.base_index = base_index;
        prediction_info.index0 = idx[0];
        prediction_info.index1 = idx[1];
        prediction_info.index2 = idx[2];
        prediction_info.tag0 = tag[0];
        prediction_info.tag1 = tag[1];
        prediction_info.tag2 = tag[2];
    end

    always_ff @(posedge clk or negedge rst_l) begin
        if (!rst_l) begin
            history_q <= '0;
            for (int i = 0; i < ENTRIES; i += 1) begin
                base_target[i] <= '0;
                base_valid[i] <= 1'b0;
                for (int t = 0; t < 3; t += 1) begin
                    tage_target[t][i] <= '0;
                    tage_confidence[t][i] <= 2'b00;
                    tage_useful[t][i] <= 2'b00;
                    tage_tag[t][i] <= '0;
                end
            end
        end else if (update_valid && update_info.valid) begin
            base_target[update_info.base_index] <= update_target;
            base_valid[update_info.base_index] <= 1'b1;

            if (update_info.predicted_target_valid &&
                    (update_info.predicted_target == update_target)) begin
                increment_provider(update_info);
            end else begin
                train_target(update_info, update_target);
            end
            history_q <= {history_q[HISTORY_BITS-2:0],
                ^update_target[11:2] ^ update_info.base_index[0]};
        end
    end

    task automatic increment_provider(input predictor_info_t info);
        unique case (info.provider)
            2'd3: begin
                if (tage_confidence[2][info.index2] != 2'b11) begin
                    tage_confidence[2][info.index2] =
                        tage_confidence[2][info.index2] + 2'b01;
                end
                if (tage_useful[2][info.index2] != 2'b11) begin
                    tage_useful[2][info.index2] =
                        tage_useful[2][info.index2] + 2'b01;
                end
            end
            2'd2: begin
                if (tage_confidence[1][info.index1] != 2'b11) begin
                    tage_confidence[1][info.index1] =
                        tage_confidence[1][info.index1] + 2'b01;
                end
                if (tage_useful[1][info.index1] != 2'b11) begin
                    tage_useful[1][info.index1] =
                        tage_useful[1][info.index1] + 2'b01;
                end
            end
            2'd1: begin
                if (tage_confidence[0][info.index0] != 2'b11) begin
                    tage_confidence[0][info.index0] =
                        tage_confidence[0][info.index0] + 2'b01;
                end
                if (tage_useful[0][info.index0] != 2'b11) begin
                    tage_useful[0][info.index0] =
                        tage_useful[0][info.index0] + 2'b01;
                end
            end
            default: begin
            end
        endcase
    endtask

    task automatic train_target(input predictor_info_t info,
            input logic [XLEN-1:0] target);
        if (info.provider != 2'd0) begin
            decrement_provider(info);
        end

        if (info.provider < 2'd1 && tage_useful[0][info.index0] == 2'b00) begin
            allocate_entry(0, info.index0, info.tag0, target);
        end else if (info.provider < 2'd2 &&
                tage_useful[1][info.index1] == 2'b00) begin
            allocate_entry(1, info.index1, info.tag1, target);
        end else if (info.provider < 2'd3 &&
                tage_useful[2][info.index2] == 2'b00) begin
            allocate_entry(2, info.index2, info.tag2, target);
        end
    endtask

    task automatic decrement_provider(input predictor_info_t info);
        unique case (info.provider)
            2'd3: if (tage_confidence[2][info.index2] != 2'b00) begin
                tage_confidence[2][info.index2] =
                    tage_confidence[2][info.index2] - 2'b01;
            end
            2'd2: if (tage_confidence[1][info.index1] != 2'b00) begin
                tage_confidence[1][info.index1] =
                    tage_confidence[1][info.index1] - 2'b01;
            end
            2'd1: if (tage_confidence[0][info.index0] != 2'b00) begin
                tage_confidence[0][info.index0] =
                    tage_confidence[0][info.index0] - 2'b01;
            end
            default: begin
            end
        endcase
    endtask

    task automatic allocate_entry(input int table_id,
            input logic [INDEX_BITS-1:0] index,
            input logic [TAG_BITS-1:0] entry_tag,
            input logic [XLEN-1:0] target);
        tage_tag[table_id][index] = entry_tag;
        tage_target[table_id][index] = target;
        tage_confidence[table_id][index] = 2'b01;
        tage_useful[table_id][index] = 2'b01;
    endtask

endmodule: ittage_predictor
