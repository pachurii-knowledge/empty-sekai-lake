/*
 * btb.sv
 *
 * Branch Target Buffer — a fetch-directed predictor for block-ending taken
 * control transfers (P2 of plans/ooo-perf.md). Indexed by the 16-byte fetch-block
 * virtual PC; a hit predicts "the last useful instruction of this block is a
 * taken control transfer to `target`, ending at parcel `offset`". The frontend
 * uses a hit to steer the NEXT fetch to `target` instead of the sequential block,
 * one cycle after the block is fetched (sync-read latency). Every steer is
 * verified at decode against the real B2 prediction and, on a mispredict, at
 * execute — so the BTB is a pure performance hint: a wrong or stale entry only
 * costs cycles, never correctness.
 *
 * Structure: direct-mapped, SETS entries, sync-read 1R + write-only 1W (the read
 * registers the array output; the write is an independent port), matching the
 * ASAP7 sync-read TAGE/ITTAGE macro-mappable pattern. No hysteresis in this
 * version: allocate on a taken train, invalidate on a not-taken/mis-steer train
 * (a "last-taken-target" cache). Prediction-only, so this is bit-identical-safe:
 * nothing here is instantiated unless the frontend chooses to consume it.
 *
 * Same-index read/write in one cycle is a benign prediction-accuracy race (the
 * registered read sees the pre-write value); it is never a correctness hazard.
 */
module btb
    import RISCV_ISA::XLEN;
#(
    parameter int SETS      = 512,
    parameter int OFFSET_W  = 3,          // terminating parcel (0..7 within a 16B block)
    parameter int TYPE_W    = 2,          // 0=cond 1=jal 2=ret 3=indirect (informational)
    localparam int IDX_W    = $clog2(SETS),
    // Index/tag off the block PC (bit 4 up; low 4 bits are the in-block offset).
    localparam int IDX_LO   = 4,
    localparam int TAG_W    = 12          // partial tag — aliasing is decode-verified
) (
    input  wire logic                 clk,
    input  wire logic                 rst_l,

    // ---- lookup (sync-read): launch with a fetch; result registered next cycle
    input  wire logic                 lookup_valid,
    input  wire logic [XLEN-1:0]      lookup_pc,     // 16B block PC
    output logic                      pred_valid,    // registered: hit for last lookup_pc
    output logic [XLEN-1:0]           pred_target,
    output logic [OFFSET_W-1:0]       pred_offset,
    output logic [TYPE_W-1:0]         pred_type,

    // ---- train (write-only 1W): driven at decode-verify
    input  wire logic                 train_valid,
    input  wire logic                 train_taken,   // 1=allocate/refresh, 0=invalidate (mis-steer)
    input  wire logic [XLEN-1:0]      train_pc,      // 16B block PC of the branch
    input  wire logic [XLEN-1:0]      train_target,
    input  wire logic [OFFSET_W-1:0]  train_offset,
    input  wire logic [TYPE_W-1:0]    train_type
);
    function automatic logic [IDX_W-1:0] idx_of(logic [XLEN-1:0] pc);
        idx_of = pc[IDX_LO +: IDX_W];
    endfunction
    function automatic logic [TAG_W-1:0] tag_of(logic [XLEN-1:0] pc);
        tag_of = pc[IDX_LO + IDX_W +: TAG_W];
    endfunction

    // Storage arrays (sync-read: one registered read per cycle).
    logic                v_q     [SETS];
    logic [TAG_W-1:0]    tag_q   [SETS];
    logic [XLEN-1:0]     tgt_q   [SETS];
    logic [OFFSET_W-1:0] off_q   [SETS];
    logic [TYPE_W-1:0]   typ_q   [SETS];

    // Registered lookup snapshot (available the cycle after lookup_valid).
    logic                lk_valid_q;
    logic [TAG_W-1:0]    lk_tag_q;
    logic [IDX_W-1:0]    lk_idx_q;
    logic                rd_v_q;
    logic [TAG_W-1:0]    rd_tag_q;
    logic [XLEN-1:0]     rd_tgt_q;
    logic [OFFSET_W-1:0] rd_off_q;
    logic [TYPE_W-1:0]   rd_typ_q;

    always_ff @(posedge clk or negedge rst_l) begin
        if (!rst_l) begin
            for (int i = 0; i < SETS; i++) v_q[i] <= 1'b0;
            lk_valid_q <= 1'b0;
            lk_tag_q   <= '0;
            lk_idx_q   <= '0;
            rd_v_q     <= 1'b0;
            rd_tag_q   <= '0;
            rd_tgt_q   <= '0;
            rd_off_q   <= '0;
            rd_typ_q   <= '0;
        end else begin
            // Sync read: register the indexed entry for the launched lookup.
            lk_valid_q <= lookup_valid;
            lk_tag_q   <= tag_of(lookup_pc);
            lk_idx_q   <= idx_of(lookup_pc);
            rd_v_q     <= v_q  [idx_of(lookup_pc)];
            rd_tag_q   <= tag_q[idx_of(lookup_pc)];
            rd_tgt_q   <= tgt_q[idx_of(lookup_pc)];
            rd_off_q   <= off_q[idx_of(lookup_pc)];
            rd_typ_q   <= typ_q[idx_of(lookup_pc)];

            // Write-only train port (independent of the read).
            if (train_valid) begin
                if (train_taken) begin
                    v_q  [idx_of(train_pc)] <= 1'b1;
                    tag_q[idx_of(train_pc)] <= tag_of(train_pc);
                    tgt_q[idx_of(train_pc)] <= train_target;
                    off_q[idx_of(train_pc)] <= train_offset;
                    typ_q[idx_of(train_pc)] <= train_type;
                end else if (v_q[idx_of(train_pc)] &&
                             (tag_q[idx_of(train_pc)] == tag_of(train_pc))) begin
                    // Mis-steer / now-not-taken: drop the entry that mispredicted.
                    v_q[idx_of(train_pc)] <= 1'b0;
                end
            end
        end
    end

    // Registered outputs: a hit is a valid entry whose tag matches the lookup PC.
    assign pred_valid  = lk_valid_q && rd_v_q && (rd_tag_q == lk_tag_q);
    assign pred_target = rd_tgt_q;
    assign pred_offset = rd_off_q;
    assign pred_type   = rd_typ_q;

endmodule : btb
