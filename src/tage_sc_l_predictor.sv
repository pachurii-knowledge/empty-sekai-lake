`include "ooo_types.vh"

`default_nettype none

module tage_sc_l_predictor
    import OOO_Types::*;
#(
    parameter int INDEX_BITS = 10,
    parameter int TAG_BITS = 10,
    parameter int HISTORY_BITS = 30,
    parameter int SC_INDEX_BITS = 8
) (
    input  logic            clk,
    input  logic            rst_l,
    input  logic            lookup_valid,
    input  logic [31:0]     lookup_pc,
    input  logic [HISTORY_BITS-1:0] history,
    output logic            prediction,
    output predictor_info_t prediction_info,
    input  logic            update_valid,
    input  logic [31:0]     update_pc,
    input  logic            update_taken,
    input  predictor_info_t update_info
);

    localparam int ENTRIES = 1 << INDEX_BITS;
    localparam int SC_ENTRIES = 1 << SC_INDEX_BITS;

    typedef logic [1:0] direction_counter_t;
    typedef logic [1:0] useful_counter_t;

    direction_counter_t base_table [ENTRIES];
    direction_counter_t tage_counter [3][ENTRIES];
    useful_counter_t tage_useful [3][ENTRIES];
    logic [TAG_BITS-1:0] tage_tag [3][ENTRIES];
    logic signed [5:0] sc_bias [SC_ENTRIES];

    logic [INDEX_BITS-1:0] base_index;
    logic [INDEX_BITS-1:0] idx [3];
    logic [TAG_BITS-1:0] tag [3];
    logic hit [3];
    logic pred [4];
    logic [1:0] provider;
    logic tage_prediction;
    logic [SC_INDEX_BITS-1:0] sc_index;
    logic signed [5:0] sc_bias_val;
    logic signed [5:0] sc_threshold;
    logic sc_taken;
    logic sc_override;

    always_comb begin
        base_index = lookup_pc[INDEX_BITS+1:2];
        idx[0] = lookup_pc[INDEX_BITS+1:2] ^ history[INDEX_BITS-1:0];
        idx[1] = lookup_pc[INDEX_BITS+1:2] ^
            history[INDEX_BITS-1:0] ^ history[2*INDEX_BITS-1:INDEX_BITS];
        idx[2] = lookup_pc[INDEX_BITS+1:2] ^
            history[INDEX_BITS-1:0] ^ history[2*INDEX_BITS-1:INDEX_BITS] ^
            history[3*INDEX_BITS-1:2*INDEX_BITS];
        // Tag hashes must be decorrelated from the index hashes above,
        // otherwise any index collision is also a tag collision and the tag
        // can never reject an aliased entry. Mix in higher-order PC bits and a
        // different set of history bits than the matching index.
        tag[0] = lookup_pc[TAG_BITS+1:2] ^
            lookup_pc[2*TAG_BITS+1:TAG_BITS+2] ^
            history[2*TAG_BITS-1:TAG_BITS];
        tag[1] = lookup_pc[TAG_BITS+1:2] ^
            lookup_pc[2*TAG_BITS+1:TAG_BITS+2] ^
            history[2*TAG_BITS-1:TAG_BITS] ^
            history[3*TAG_BITS-1:2*TAG_BITS];
        tag[2] = lookup_pc[TAG_BITS+1:2] ^
            lookup_pc[2*TAG_BITS+1:TAG_BITS+2] ^
            history[3*TAG_BITS-1:2*TAG_BITS];

        pred[0] = base_table[base_index][1];
        for (int i = 0; i < 3; i += 1) begin
            hit[i] = tage_tag[i][idx[i]] == tag[i];
            pred[i + 1] = tage_counter[i][idx[i]][1];
        end

        provider = 2'd0;
        if (hit[0] && tage_useful[0][idx[0]] != 2'b00) begin
            provider = 2'd1;
        end
        if (hit[1] && tage_useful[1][idx[1]] != 2'b00) begin
            provider = 2'd2;
        end
        if (hit[2] && tage_useful[2][idx[2]] != 2'b00) begin
            provider = 2'd3;
        end

        unique case (provider)
            2'd3: tage_prediction = pred[3];
            2'd2: tage_prediction = pred[2];
            2'd1: tage_prediction = pred[1];
            default: tage_prediction = pred[0];
        endcase

        // TAGE is the primary predictor. The statistical corrector only flips
        // it when the SC bias accumulates evidence beyond a threshold, and a
        // tagged TAGE provider (higher confidence than the bimodal fallback)
        // demands much stronger SC evidence before being overridden. This
        // mirrors the reference, where SC overrides TAGE only past a learned
        // threshold gated by TAGE confidence.
        sc_index = lookup_pc[SC_INDEX_BITS+1:2] ^ history[SC_INDEX_BITS-1:0];
        sc_bias_val = sc_bias[sc_index];
        sc_taken = (sc_bias_val >= 0);
        sc_threshold = (provider != 2'd0) ? 6'sd12 : 6'sd4;
        sc_override = (sc_taken != tage_prediction) &&
            ((sc_bias_val >= sc_threshold) || (sc_bias_val <= -sc_threshold));
        prediction = lookup_valid && (sc_override ? sc_taken : tage_prediction);

        prediction_info = '0;
        prediction_info.valid = lookup_valid;
        prediction_info.predicted_taken = prediction;
        prediction_info.provider = provider;
        prediction_info.base_index = base_index;
        prediction_info.index0 = idx[0];
        prediction_info.index1 = idx[1];
        prediction_info.index2 = idx[2];
        prediction_info.tag0 = tag[0];
        prediction_info.tag1 = tag[1];
        prediction_info.tag2 = tag[2];
        prediction_info.sc_history = 10'(history[SC_INDEX_BITS-1:0]);
    end

    always_ff @(posedge clk or negedge rst_l) begin
        if (!rst_l) begin
            for (int i = 0; i < ENTRIES; i += 1) begin
                base_table[i] <= 2'b01;
                for (int t = 0; t < 3; t += 1) begin
                    tage_counter[t][i] <= 2'b01;
                    tage_useful[t][i] <= 2'b00;
                    tage_tag[t][i] <= '0;
                end
            end
            for (int i = 0; i < SC_ENTRIES; i += 1) begin
                sc_bias[i] <= '0;
            end
        end else if (update_valid && update_info.valid) begin
            if (update_info.provider == 2'd0) begin
                update_counter(base_table[update_info.base_index], update_taken);
            end
            unique case (update_info.provider)
                2'd3: begin
                    update_counter(tage_counter[2][update_info.index2], update_taken);
                    update_useful(tage_useful[2][update_info.index2],
                        update_info.predicted_taken == update_taken);
                end
                2'd2: begin
                    update_counter(tage_counter[1][update_info.index1], update_taken);
                    update_useful(tage_useful[1][update_info.index1],
                        update_info.predicted_taken == update_taken);
                end
                2'd1: begin
                    update_counter(tage_counter[0][update_info.index0], update_taken);
                    update_useful(tage_useful[0][update_info.index0],
                        update_info.predicted_taken == update_taken);
                end
                default: begin
                end
            endcase

            if (update_info.predicted_taken != update_taken) begin
                allocate_entry(update_info, update_taken);
            end
            update_sc(update_pc, update_info.sc_history[SC_INDEX_BITS-1:0],
                update_taken);
        end
    end

    task automatic update_counter(inout direction_counter_t counter,
            input logic taken);
        if (taken && (counter != 2'b11)) begin
            counter = counter + 2'b01;
        end else if (!taken && (counter != 2'b00)) begin
            counter = counter - 2'b01;
        end
    endtask

    task automatic update_useful(inout useful_counter_t useful,
            input logic correct);
        if (correct && (useful != 2'b11)) begin
            useful = useful + 2'b01;
        end else if (!correct && (useful != 2'b00)) begin
            useful = useful - 2'b01;
        end
    endtask

    task automatic allocate_entry(input predictor_info_t info,
            input logic taken);
        if (info.provider < 2'd1 && tage_useful[0][info.index0] == 2'b00) begin
            tage_tag[0][info.index0] = info.tag0;
            tage_counter[0][info.index0] = taken ? 2'b10 : 2'b01;
            tage_useful[0][info.index0] = 2'b01;
        end else if (info.provider < 2'd2 &&
                tage_useful[1][info.index1] == 2'b00) begin
            tage_tag[1][info.index1] = info.tag1;
            tage_counter[1][info.index1] = taken ? 2'b10 : 2'b01;
            tage_useful[1][info.index1] = 2'b01;
        end else if (info.provider < 2'd3 &&
                tage_useful[2][info.index2] == 2'b00) begin
            tage_tag[2][info.index2] = info.tag2;
            tage_counter[2][info.index2] = taken ? 2'b10 : 2'b01;
            tage_useful[2][info.index2] = 2'b01;
        end
    endtask

    task automatic update_sc(input logic [31:0] pc,
            input logic [SC_INDEX_BITS-1:0] sc_hist, input logic taken);
        logic [SC_INDEX_BITS-1:0] update_index;
        update_index = pc[SC_INDEX_BITS+1:2] ^ sc_hist;
        if (taken && (sc_bias[update_index] != 6'sd31)) begin
            sc_bias[update_index] = sc_bias[update_index] + 6'sd1;
        end else if (!taken && (sc_bias[update_index] != -6'sd32)) begin
            sc_bias[update_index] = sc_bias[update_index] - 6'sd1;
        end
    endtask

endmodule: tage_sc_l_predictor
