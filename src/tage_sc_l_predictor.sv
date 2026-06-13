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
    input wire logic            clk,
    input wire logic            rst_l,
    input wire logic            lookup_valid,
    input wire logic [XLEN-1:0] lookup_pc,
    input wire logic [HISTORY_BITS-1:0] history,
    output logic            prediction,
    output predictor_info_t prediction_info,
    input wire logic            update_valid,
    input wire logic [XLEN-1:0] update_pc,
    input wire logic            update_taken,
    input wire predictor_info_t update_info
);

    localparam int ENTRIES = 1 << INDEX_BITS;
    localparam int SC_ENTRIES = 1 << SC_INDEX_BITS;

    typedef logic [1:0] direction_counter_t;
    typedef logic [1:0] useful_counter_t;

    // FPGA-friendly storage: each tagged bank is a SEPARATE 1D array (a 3D
    // `[3][ENTRIES]` array + a variable bank index forces the whole table to
    // flip-flops; per-bank 1D arrays with a single write port each infer RAM).
    // No array-wide reset under SYNTHESIS (RAM powers up to 0, which is the reset
    // value for tags/useful/sc; counters/base just warm up from 0 vs 2'b01 -- a
    // self-correcting, best-effort difference). Simulation keeps the deterministic
    // reset and is bit-identical (the read/update logic is unchanged).
    direction_counter_t base_table [ENTRIES];
    direction_counter_t tage_counter_0 [ENTRIES];
    direction_counter_t tage_counter_1 [ENTRIES];
    direction_counter_t tage_counter_2 [ENTRIES];
    useful_counter_t    tage_useful_0  [ENTRIES];
    useful_counter_t    tage_useful_1  [ENTRIES];
    useful_counter_t    tage_useful_2  [ENTRIES];
    logic [TAG_BITS-1:0] tage_tag_0 [ENTRIES];
    logic [TAG_BITS-1:0] tage_tag_1 [ENTRIES];
    logic [TAG_BITS-1:0] tage_tag_2 [ENTRIES];
    logic signed [5:0]   sc_bias [SC_ENTRIES];

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

    // Saturating counter updates as pure functions (RAM-friendly: a function
    // returns the next value for a non-blocking write, unlike an inout task).
    function automatic direction_counter_t cnt_upd(input direction_counter_t c,
            input logic taken);
        if (taken && (c != 2'b11))       cnt_upd = c + 2'b01;
        else if (!taken && (c != 2'b00)) cnt_upd = c - 2'b01;
        else                             cnt_upd = c;
    endfunction
    function automatic useful_counter_t use_upd(input useful_counter_t u,
            input logic correct);
        if (correct && (u != 2'b11))       use_upd = u + 2'b01;
        else if (!correct && (u != 2'b00)) use_upd = u - 2'b01;
        else                               use_upd = u;
    endfunction
    function automatic logic signed [5:0] sc_upd(input logic signed [5:0] b,
            input logic taken);
        if (taken && (b != 6'sd31))        sc_upd = b + 6'sd1;
        else if (!taken && (b != -6'sd32)) sc_upd = b - 6'sd1;
        else                               sc_upd = b;
    endfunction

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

        // Per-bank reads (constant bank index -> RAM read port).
        pred[0] = base_table[base_index][1];
        hit[0] = tage_tag_0[idx[0]] == tag[0];  pred[1] = tage_counter_0[idx[0]][1];
        hit[1] = tage_tag_1[idx[1]] == tag[1];  pred[2] = tage_counter_1[idx[1]][1];
        hit[2] = tage_tag_2[idx[2]] == tag[2];  pred[3] = tage_counter_2[idx[2]][1];

        provider = 2'd0;
        if (hit[0] && tage_useful_0[idx[0]] != 2'b00) begin
            provider = 2'd1;
        end
        if (hit[1] && tage_useful_1[idx[1]] != 2'b00) begin
            provider = 2'd2;
        end
        if (hit[2] && tage_useful_2[idx[2]] != 2'b00) begin
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

    // SC update index (combinational, used by the write below).
    logic [SC_INDEX_BITS-1:0] sc_wr_index;
    assign sc_wr_index = update_pc[SC_INDEX_BITS+1:2] ^
                         update_info.sc_history[SC_INDEX_BITS-1:0];
    logic upd_correct;
    assign upd_correct = (update_info.predicted_taken == update_taken);
    logic do_alloc;
    assign do_alloc = update_valid && update_info.valid &&
                      (update_info.predicted_taken != update_taken);

    always_ff @(posedge clk
`ifndef SYNTHESIS
            or negedge rst_l
`endif
        ) begin
`ifndef SYNTHESIS
        if (!rst_l) begin
            for (int i = 0; i < ENTRIES; i += 1) begin
                base_table[i] <= 2'b01;
                tage_counter_0[i] <= 2'b01; tage_counter_1[i] <= 2'b01; tage_counter_2[i] <= 2'b01;
                tage_useful_0[i] <= 2'b00;  tage_useful_1[i] <= 2'b00;  tage_useful_2[i] <= 2'b00;
                tage_tag_0[i] <= '0;        tage_tag_1[i] <= '0;        tage_tag_2[i] <= '0;
            end
            for (int i = 0; i < SC_ENTRIES; i += 1) begin
                sc_bias[i] <= '0;
            end
        end else
`endif
        if (update_valid && update_info.valid) begin
            // Counter/useful update of the provider bank (each bank's element is
            // written here XOR by allocate below -- allocate always targets a bank
            // strictly longer than the provider -- so each array keeps one write
            // port at one address). base = provider 0.
            if (update_info.provider == 2'd0)
                base_table[update_info.base_index] <=
                    cnt_upd(base_table[update_info.base_index], update_taken);
            if (update_info.provider == 2'd1) begin
                tage_counter_0[update_info.index0] <=
                    cnt_upd(tage_counter_0[update_info.index0], update_taken);
                tage_useful_0[update_info.index0] <=
                    use_upd(tage_useful_0[update_info.index0], upd_correct);
            end
            if (update_info.provider == 2'd2) begin
                tage_counter_1[update_info.index1] <=
                    cnt_upd(tage_counter_1[update_info.index1], update_taken);
                tage_useful_1[update_info.index1] <=
                    use_upd(tage_useful_1[update_info.index1], upd_correct);
            end
            if (update_info.provider == 2'd3) begin
                tage_counter_2[update_info.index2] <=
                    cnt_upd(tage_counter_2[update_info.index2], update_taken);
                tage_useful_2[update_info.index2] <=
                    use_upd(tage_useful_2[update_info.index2], upd_correct);
            end

            // Allocate a new entry on a misprediction, in the first free (useful==0)
            // bank strictly longer than the provider.
            if (do_alloc) begin
                if (update_info.provider < 2'd1 &&
                        tage_useful_0[update_info.index0] == 2'b00) begin
                    tage_tag_0[update_info.index0]     <= update_info.tag0;
                    tage_counter_0[update_info.index0] <= update_taken ? 2'b10 : 2'b01;
                    tage_useful_0[update_info.index0]  <= 2'b01;
                end else if (update_info.provider < 2'd2 &&
                        tage_useful_1[update_info.index1] == 2'b00) begin
                    tage_tag_1[update_info.index1]     <= update_info.tag1;
                    tage_counter_1[update_info.index1] <= update_taken ? 2'b10 : 2'b01;
                    tage_useful_1[update_info.index1]  <= 2'b01;
                end else if (update_info.provider < 2'd3 &&
                        tage_useful_2[update_info.index2] == 2'b00) begin
                    tage_tag_2[update_info.index2]     <= update_info.tag2;
                    tage_counter_2[update_info.index2] <= update_taken ? 2'b10 : 2'b01;
                    tage_useful_2[update_info.index2]  <= 2'b01;
                end
            end

            sc_bias[sc_wr_index] <= sc_upd(sc_bias[sc_wr_index], update_taken);
        end
    end

endmodule: tage_sc_l_predictor

`default_nettype wire
