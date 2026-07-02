`include "ooo_types.vh"

`default_nettype none

module ittage_predictor
    import OOO_Types::*;
#(
    parameter int INDEX_BITS = 10,
    parameter int TAG_BITS = 10,
    parameter int HISTORY_BITS = 30
) (
    input wire logic            clk,
    input wire logic            rst_l,
    input wire logic            lookup_valid,
    input wire logic [XLEN-1:0] lookup_pc,
    output logic            prediction_valid,
    output logic [XLEN-1:0] prediction_target,
    output predictor_info_t prediction_info,
    input wire logic            update_valid,
    input wire logic [XLEN-1:0] update_target,
    input wire predictor_info_t update_info
);

    localparam int ENTRIES = 1 << INDEX_BITS;

    typedef logic [1:0] confidence_counter_t;
    typedef logic [1:0] useful_counter_t;

    // Per-bank 1D arrays (see tage_sc_l_predictor) so the tables infer RAM rather
    // than flip-flops; no array-wide reset under SYNTHESIS (RAM powers up to 0 =
    // the reset value). history_q is written in the update branch, so it also
    // powers up to 0. Simulation keeps the deterministic reset (bit-identical).
    logic [HISTORY_BITS-1:0] history_q;
    logic [XLEN-1:0] base_target [ENTRIES];
    logic            base_valid  [ENTRIES];
    logic [XLEN-1:0] tage_target_0 [ENTRIES];
    logic [XLEN-1:0] tage_target_1 [ENTRIES];
    logic [XLEN-1:0] tage_target_2 [ENTRIES];
    confidence_counter_t tage_confidence_0 [ENTRIES];
    confidence_counter_t tage_confidence_1 [ENTRIES];
    confidence_counter_t tage_confidence_2 [ENTRIES];
    useful_counter_t tage_useful_0 [ENTRIES];
    useful_counter_t tage_useful_1 [ENTRIES];
    useful_counter_t tage_useful_2 [ENTRIES];
    logic [TAG_BITS-1:0] tage_tag_0 [ENTRIES];
    logic [TAG_BITS-1:0] tage_tag_1 [ENTRIES];
    logic [TAG_BITS-1:0] tage_tag_2 [ENTRIES];

    logic [INDEX_BITS-1:0] base_index;
    logic [INDEX_BITS-1:0] idx [3];
    logic [TAG_BITS-1:0] tag [3];
    logic hit [3];
    logic [1:0] provider;

    function automatic confidence_counter_t conf_inc(input confidence_counter_t c);
        conf_inc = (c != 2'b11) ? (c + 2'b01) : c;
    endfunction
    function automatic confidence_counter_t conf_dec(input confidence_counter_t c);
        conf_dec = (c != 2'b00) ? (c - 2'b01) : c;
    endfunction
    function automatic useful_counter_t use_inc(input useful_counter_t u);
        use_inc = (u != 2'b11) ? (u + 2'b01) : u;
    endfunction

    // --- Stage 1 (combinational): index/tag hashes from lookup_pc + history_q ---
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
    end

    // --- Sync read: register the per-bank array reads + the metadata that must
    // travel with them (idx/tag/base_index) so prediction + prediction_info are a
    // self-consistent snapshot of the same (lookup_pc, history_q). Prediction is
    // available one cycle later; the frontend holds the group one cycle (see
    // riscv_core_ooo.sv predict_stall). The lookup reads confidence/target/tag +
    // base; tage_useful is read only in the update path (left async). ---
    logic [XLEN-1:0]       base_tgt_q;
    logic                  base_valid_q;
    logic [XLEN-1:0]       tgt_rd_q  [3];
    confidence_counter_t   conf_rd_q [3];
    useful_counter_t       use_rd_q  [3];       // carried for the write-only update allocate
    logic [TAG_BITS-1:0]   tag_rd_q  [3];       // stored tags (compare operand A)
    logic [INDEX_BITS-1:0] idx_q     [3];
    logic [TAG_BITS-1:0]   tag_q     [3];       // computed tags (compare operand B + info)
    logic [INDEX_BITS-1:0] base_index_q;
    logic                  lookup_valid_q;

    always_ff @(posedge clk or negedge rst_l) begin
        if (!rst_l) begin
            base_tgt_q     <= '0;
            base_valid_q   <= 1'b0;
            tgt_rd_q       <= '{default: '0};
            conf_rd_q      <= '{default: '0};
            use_rd_q       <= '{default: '0};
            tag_rd_q       <= '{default: '0};
            idx_q          <= '{default: '0};
            tag_q          <= '{default: '0};
            base_index_q   <= '0;
            lookup_valid_q <= 1'b0;
        end else begin
            base_tgt_q   <= base_target[base_index];
            base_valid_q <= base_valid[base_index];
            tgt_rd_q[0]  <= tage_target_0[idx[0]];
            tgt_rd_q[1]  <= tage_target_1[idx[1]];
            tgt_rd_q[2]  <= tage_target_2[idx[2]];
            conf_rd_q[0] <= tage_confidence_0[idx[0]];
            conf_rd_q[1] <= tage_confidence_1[idx[1]];
            conf_rd_q[2] <= tage_confidence_2[idx[2]];
            use_rd_q[0]  <= tage_useful_0[idx[0]];
            use_rd_q[1]  <= tage_useful_1[idx[1]];
            use_rd_q[2]  <= tage_useful_2[idx[2]];
            tag_rd_q[0]  <= tage_tag_0[idx[0]];
            tag_rd_q[1]  <= tage_tag_1[idx[1]];
            tag_rd_q[2]  <= tage_tag_2[idx[2]];
            idx_q[0]     <= idx[0];  idx_q[1] <= idx[1];  idx_q[2] <= idx[2];
            tag_q[0]     <= tag[0];  tag_q[1] <= tag[1];  tag_q[2] <= tag[2];
            base_index_q   <= base_index;
            lookup_valid_q <= lookup_valid;
        end
    end

    // --- Stage 2 (combinational): prediction from the registered reads ---
    always_comb begin
        hit[0] = (tag_rd_q[0] == tag_q[0]) && (conf_rd_q[0] != 2'b00);
        hit[1] = (tag_rd_q[1] == tag_q[1]) && (conf_rd_q[1] != 2'b00);
        hit[2] = (tag_rd_q[2] == tag_q[2]) && (conf_rd_q[2] != 2'b00);

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

        prediction_valid = lookup_valid_q &&
            ((provider != 2'd0) || base_valid_q);
        unique case (provider)
            2'd3: prediction_target = tgt_rd_q[2];
            2'd2: prediction_target = tgt_rd_q[1];
            2'd1: prediction_target = tgt_rd_q[0];
            default: prediction_target = base_tgt_q;
        endcase

        prediction_info = '0;
        prediction_info.valid = lookup_valid_q;
        prediction_info.predicted_taken = prediction_valid;
        prediction_info.predicted_target_valid = prediction_valid;
        prediction_info.predicted_target = prediction_target;
        prediction_info.provider = provider;
        prediction_info.base_index = base_index_q;
        prediction_info.index0 = idx_q[0];
        prediction_info.index1 = idx_q[1];
        prediction_info.index2 = idx_q[2];
        prediction_info.tag0 = tag_q[0];
        prediction_info.tag1 = tag_q[1];
        prediction_info.tag2 = tag_q[2];
        // Carry the read confidence (as ctr*) + useful so the update is write-only.
        prediction_info.ctr0 = conf_rd_q[0];
        prediction_info.ctr1 = conf_rd_q[1];
        prediction_info.ctr2 = conf_rd_q[2];
        prediction_info.use0 = use_rd_q[0];
        prediction_info.use1 = use_rd_q[1];
        prediction_info.use2 = use_rd_q[2];
    end

    // Update was "correct" (provider's target matched) -> increment; else train
    // (decrement the provider, allocate a longer bank).
    logic tgt_match;
    assign tgt_match = update_info.predicted_target_valid &&
                       (update_info.predicted_target == update_target);

    always_ff @(posedge clk
`ifndef SYNTHESIS
            or negedge rst_l
`endif
        ) begin
`ifndef SYNTHESIS
        if (!rst_l) begin
            history_q <= '0;
            for (int i = 0; i < ENTRIES; i += 1) begin
                base_target[i] <= '0; base_valid[i] <= 1'b0;
                tage_target_0[i] <= '0; tage_target_1[i] <= '0; tage_target_2[i] <= '0;
                tage_confidence_0[i] <= 2'b00; tage_confidence_1[i] <= 2'b00; tage_confidence_2[i] <= 2'b00;
                tage_useful_0[i] <= 2'b00; tage_useful_1[i] <= 2'b00; tage_useful_2[i] <= 2'b00;
                tage_tag_0[i] <= '0; tage_tag_1[i] <= '0; tage_tag_2[i] <= '0;
            end
        end else
`endif
        if (update_valid && update_info.valid) begin
            base_target[update_info.base_index] <= update_target;
            base_valid[update_info.base_index]  <= 1'b1;

            // Write-only update: old confidence/useful come from the carried lookup
            // snapshot (update_info.ctr*/use*), not an array read, so each array has
            // a single write address and (with the sync-read lookup) maps to SRAM.
            // Increment on a target match, else decrement the confidence. Each
            // bank's element is written here XOR by allocate below.
            if (update_info.provider == 2'd1) begin
                if (tgt_match) begin
                    tage_confidence_0[update_info.index0] <= conf_inc(update_info.ctr0);
                    tage_useful_0[update_info.index0]     <= use_inc(update_info.use0);
                end else begin
                    tage_confidence_0[update_info.index0] <= conf_dec(update_info.ctr0);
                end
            end
            if (update_info.provider == 2'd2) begin
                if (tgt_match) begin
                    tage_confidence_1[update_info.index1] <= conf_inc(update_info.ctr1);
                    tage_useful_1[update_info.index1]     <= use_inc(update_info.use1);
                end else begin
                    tage_confidence_1[update_info.index1] <= conf_dec(update_info.ctr1);
                end
            end
            if (update_info.provider == 2'd3) begin
                if (tgt_match) begin
                    tage_confidence_2[update_info.index2] <= conf_inc(update_info.ctr2);
                    tage_useful_2[update_info.index2]     <= use_inc(update_info.use2);
                end else begin
                    tage_confidence_2[update_info.index2] <= conf_dec(update_info.ctr2);
                end
            end

            // Train (mispredicted target): allocate in the first free (useful==0)
            // bank strictly longer than the provider (useful from the carried snapshot).
            if (!tgt_match) begin
                if (update_info.provider < 2'd1 &&
                        update_info.use0 == 2'b00) begin
                    tage_tag_0[update_info.index0]        <= update_info.tag0;
                    tage_target_0[update_info.index0]     <= update_target;
                    tage_confidence_0[update_info.index0] <= 2'b01;
                    tage_useful_0[update_info.index0]     <= 2'b01;
                end else if (update_info.provider < 2'd2 &&
                        update_info.use1 == 2'b00) begin
                    tage_tag_1[update_info.index1]        <= update_info.tag1;
                    tage_target_1[update_info.index1]     <= update_target;
                    tage_confidence_1[update_info.index1] <= 2'b01;
                    tage_useful_1[update_info.index1]     <= 2'b01;
                end else if (update_info.provider < 2'd3 &&
                        update_info.use2 == 2'b00) begin
                    tage_tag_2[update_info.index2]        <= update_info.tag2;
                    tage_target_2[update_info.index2]     <= update_target;
                    tage_confidence_2[update_info.index2] <= 2'b01;
                    tage_useful_2[update_info.index2]     <= 2'b01;
                end
            end

            history_q <= {history_q[HISTORY_BITS-2:0],
                ^update_target[11:2] ^ update_info.base_index[0]};
        end
    end

endmodule: ittage_predictor

`default_nettype wire
