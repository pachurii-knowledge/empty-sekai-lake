/**
 * rvc_realign.sv
 *
 * RV64C (compressed) two-wide expand-before-decode realign stage.
 *
 * Drains the already-buffered 16-byte fetch group (fgrp_*) as 8 x 16-bit
 * parcels and emits up to TWO canonical 32-bit instructions per cycle (true
 * 2-byte-granular PC + is_compressed + per-instruction fetch fault) into the
 * UNCHANGED riscv_decode path. A 32-bit instruction whose low half is the last
 * parcel of a block (parcel 7) straddles into the sequentially-next block; this
 * is carried by ONE 16-bit dangling-half latch. The memory/L1I interface is
 * untouched -- fetch stays one block-aligned, word-granular, in-order request;
 * the realigner only reshapes the buffered bytes in-core.
 *
 * Parcel accounting vs. architectural length: the drain pointer counts IN-BLOCK
 * parcels consumed (compressed = 1, in-block 32-bit = 2, straddle-completion =
 * 1 -- its low half came from the latch, so it consumes only parcel 0 of the
 * new block). The architectural ILEN (=4 for a 32-bit op, used for PC/link/RAS)
 * is a separate concern handled outside this module via ILEN_INC.
 *
 * Whole body `ifdef RVC-gated (the Makefile globs every src file into one
 * Verilator build; ungated it would compile into the non-C baseline).
 */
`ifdef RVC

`include "ooo_types.vh"   // brings in the `RVC_NLANES lane-count macro (P4)

`default_nettype none

module rvc_realign
    import OOO_Types::*;
    import RISCV_UArch::MEMORY_READ_WIDTH;
#(
    // Emitted lanes: 2 (default) or 4 under -DREALIGN4. Sized from the shared
    // macro so ports/body agree with ooo_fetch_decode + riscv_core_ooo.
    parameter int NL = `RVC_NLANES
)(
    input  wire logic                        clk,
    input  wire logic                        rst_l,

    // ---- presented fetch group (the fbuf/fgrp bypass in riscv_core_ooo) ----
    input  wire logic                        fgrp_valid,
    input  wire logic [XLEN-1:0]             fgrp_pc,     // 2-byte aligned; [3:1]=start parcel
    input  wire logic [MEMORY_READ_WIDTH-1:0][XLEN-1:0] fgrp_data,
    input  wire logic                        fgrp_excpt,  // memory bus exception (rare)
    input  wire logic [3:0]                  fgrp_fault_word,  // ABSOLUTE per-4B-word fetch fault
    input  wire logic [4:0]                  fgrp_fault_cause,

    // ---- backend feedback ----
    input  wire logic [2:0]                  dispatch_count, // emitted lanes actually dispatched (0..NL)
    input  wire logic                        frontend_hold,  // stall/halt: freeze, no consume
    input  wire logic                        fetch_flush,    // any redirect: reset drain + straddle

    // ---- P2b offset-precise BTB steer (tied 0 when BTB disabled) ----
    input  wire logic                        btb_hit_in,   // this block was BTB-steered to a target
    input  wire logic [2:0]                  btb_off_in,   // last in-block parcel of the predicted branch

    // ---- NL aligned + expanded lanes (NL = 2 default, 4 under -DREALIGN4) ----
    output logic [NL-1:0]                    out_valid,
    output logic [NL-1:0][XLEN-1:0]          out_pc,
    output logic [NL-1:0][31:0]              out_instr,      // canonical 32-bit
    output logic [NL-1:0]                    out_is_compressed,
    output logic [NL-1:0]                    out_fetch_fault,
    output logic [NL-1:0]                    out_fetch_fault_hi,
    output logic [NL-1:0][4:0]               out_fault_cause,
    output logic [NL-1:0][15:0]              out_rvc_parcel, // raw parcel (illegal-compressed mtval)

    // ---- frontend control ----
    output logic                             rvc_consume_block,   // pop the presented group
    output logic                             rvc_block_drained,   // P2a BTB: block-ending (flush-independent)
    output logic                             rvc_btb_terminate,   // P2b: block terminated AT btb_off this cycle
    output logic                             rvc_tail_straddle,   // P2b: drain ended in a tail straddle (case-E recover)
    output logic                             frontend_oldest_valid,
    output logic [XLEN-1:0]                  frontend_oldest_pc
`ifdef DFE_S1
    ,
    // ---- DFE S1a inert taps: live drain start + straddle-completion state so the
    // core's fetch-side branch predecode scans the SAME window origin the realigner
    // presents this cycle. Read only by the S1a equivalence assertion (no consumer). ----
    output logic [2:0]                       dfe_s0,
    output logic                             dfe_completing,
    output logic [XLEN-1:0]                  dfe_straddle_pc,
    output logic [15:0]                      dfe_straddle_half
`endif
);

    // ---------------- registered drain / straddle state ----------------
    logic [2:0]      align_ptr_q, align_ptr_n;
    logic            align_ptr_valid_q, align_ptr_valid_n;   // 0 => fresh head (use fgrp_pc[3:1])
    logic            straddle_valid_q, straddle_valid_n;
    logic [15:0]     straddle_half_q, straddle_half_n;        // low 16b from prev block parcel 7
    logic [XLEN-1:0] straddle_pc_q, straddle_pc_n;            // = prev block base + 14
    logic            straddle_fault_q, straddle_fault_n;      // low-half word fault
    logic [4:0]      straddle_cause_q, straddle_cause_n;

    // ---------------- block parcels (RV64: low 2 of the returned words) ----------------
    // parcel k (0..7): word = k[2], within-word 16-bit slice = k[1:0].
    logic [15:0] pblock [0:7];
    always_comb begin
        for (int k = 0; k < 8; k += 1)
            pblock[k] = fgrp_data[k[2]][ (k[1:0]) * 16 +: 16 ];
    end

    logic [XLEN-1:0] base;
    assign base = {fgrp_pc[XLEN-1:4], 4'b0};

    // start parcel of the (non-completing) drain this cycle
    logic [2:0] s0;
    assign s0 = align_ptr_valid_q ? align_ptr_q : fgrp_pc[3:1];
    // lane0 is a straddle completion iff a low half is latched and we are on a
    // fresh (block-aligned successor) head.
    logic completing;
    assign completing = straddle_valid_q && !align_ptr_valid_q;

    // ---------------- expanders (mux the start parcel, then expand) ----------------
    logic [15:0] lane0_par, lane1_par;
    logic [31:0] exp0_x, exp1_x;
    logic        exp0_ill, exp1_ill;   // (illegal already carried as {0,c} in exp*_x)
    rvc_expand exp0 (.c(lane0_par), .x(exp0_x), .illegal(exp0_ill));
    rvc_expand exp1 (.c(lane1_par), .x(exp1_x), .illegal(exp1_ill));
`ifdef REALIGN4
    logic [15:0] lane2_par, lane3_par;
    logic [31:0] exp2_x, exp3_x;
    logic        exp2_ill, exp3_ill;
    rvc_expand exp2 (.c(lane2_par), .x(exp2_x), .illegal(exp2_ill));
    rvc_expand exp3 (.c(lane3_par), .x(exp3_x), .illegal(exp3_ill));
`endif

    // bounded high-half fetch of a 32-bit instruction starting at parcel p
    function automatic logic [15:0] hi_parcel(input logic [2:0] p);
        logic [3:0] idx;
        idx = {1'b0, p} + 4'd1;
        hi_parcel = (idx < 4'd8) ? pblock[idx[2:0]] : 16'h0000;
    endfunction

    // ---------------- lane 0 ----------------
    logic        l0_is32;
    logic [3:0]  l0_next;      // start parcel of the following instruction (0..8)
    logic [1:0]  l0_parcels;   // IN-BLOCK parcels consumed
    logic        l0_tail;      // 32-bit low half at parcel 7 (straddles)
    logic [2:0]  l0_lo_w, l0_hi_w;
    logic [2:0]  l0_last;      // P2b: last in-block parcel of lane0's instruction
    logic        btb_term0;    // P2b: lane0 IS the BTB-predicted terminating branch

    assign lane0_par = completing ? pblock[0] : pblock[s0];
    always_comb begin
        l0_is32   = (lane0_par[1:0] == 2'b11);
        l0_lo_w   = s0 >> 1;
        l0_hi_w   = ({1'b0, s0} + 4'd1) >> 1;
        l0_tail   = 1'b0;
        out_valid[0]          = fgrp_valid && !fgrp_excpt;
        out_pc[0]             = completing ? straddle_pc_q : (base + {s0, 1'b0});
        out_instr[0]          = 32'h0000_0013;
        out_is_compressed[0]  = 1'b0;
        out_rvc_parcel[0]     = 16'h0000;
        out_fetch_fault[0]    = 1'b0;
        out_fetch_fault_hi[0] = 1'b0;
        out_fault_cause[0]    = fgrp_fault_cause;
        l0_parcels            = 2'd0;
        l0_next               = {1'b0, s0};

        if (completing) begin
            // straddle completion: {this-block parcel0 (high) | latched low}
            out_instr[0]          = {pblock[0], straddle_half_q};
            out_is_compressed[0]  = 1'b0;                       // hard-forced ilen=4
            l0_parcels            = 2'd1;                       // consumes parcel 0 only
            l0_next               = 4'd1;
            // low half (prev block word 3) OR high half (this block word 0)
            out_fetch_fault[0]    = straddle_fault_q | fgrp_fault_word[0];
            out_fetch_fault_hi[0] = fgrp_fault_word[0] & ~straddle_fault_q;
            out_fault_cause[0]    = straddle_fault_q ? straddle_cause_q : fgrp_fault_cause;
        end else if (l0_is32 && (s0 == 3'd7)) begin
            // 32-bit low half at parcel 7 -> cannot complete in this block
            out_valid[0] = 1'b0;
            l0_tail      = 1'b1;
            l0_parcels   = 2'd0;
        end else if (l0_is32) begin
            out_instr[0]          = {hi_parcel(s0), pblock[s0]};
            out_is_compressed[0]  = 1'b0;
            l0_parcels            = 2'd2;
            l0_next               = {1'b0, s0} + 4'd2;
            out_fetch_fault[0]    = fgrp_fault_word[l0_lo_w] | fgrp_fault_word[l0_hi_w];
            out_fetch_fault_hi[0] = fgrp_fault_word[l0_hi_w] & ~fgrp_fault_word[l0_lo_w];
        end else begin
            out_instr[0]          = exp0_x;
            out_is_compressed[0]  = 1'b1;
            out_rvc_parcel[0]     = lane0_par;
            l0_parcels            = 2'd1;
            l0_next               = {1'b0, s0} + 4'd1;
            out_fetch_fault[0]    = fgrp_fault_word[l0_lo_w];
        end
        // P2b: lane0 is the terminating branch iff its last in-block parcel matches
        // the BTB offset (a straddle completion is never a BTB entry -> !completing;
        // a parcel-7 32-bit low half has out_valid[0]=0 so it cannot match). Computed
        // IN-BLOCK, after out_valid[0]: a continuous assign reading out_valid[0] here
        // would form an UNOPTFLAT cycle with the lane1 block that consumes btb_term0.
        l0_last   = l0_is32 ? (s0 + 3'd1) : s0;
        btb_term0 = btb_hit_in && out_valid[0] && !completing &&
                    (l0_last == btb_off_in);
    end

    // ---------------- lane 1 ----------------
    logic        l1_is32;
    logic [1:0]  l1_parcels;
    logic        l1_tail;
    logic [3:0]  s1;            // 0..8; lane1 only valid when s1 <= 7
    logic [2:0]  s1b;           // s1[2:0], safe pblock index when s1 <= 7
    logic [2:0]  l1_lo_w, l1_hi_w;
    logic [2:0]  l1_last;       // P2b: last in-block parcel of lane1's instruction
    logic        btb_term1;     // P2b: lane1 IS the BTB-predicted terminating branch
    logic [3:0]  l1_next;       // P4: start parcel of the instr after lane1 (feeds lane2)

    assign s1  = l0_next;
    assign s1b = s1[2:0];
    assign lane1_par = pblock[s1b];
    always_comb begin
        l1_is32   = (lane1_par[1:0] == 2'b11);
        l1_lo_w   = s1[3:1];
        l1_hi_w   = ({1'b0, s1b} + 4'd1) >> 1;
        l1_tail   = 1'b0;
        out_valid[1]          = 1'b0;
        out_pc[1]             = base + {s1b, 1'b0};
        out_instr[1]          = 32'h0000_0013;
        out_is_compressed[1]  = 1'b0;
        out_rvc_parcel[1]     = 16'h0000;
        out_fetch_fault[1]    = 1'b0;
        out_fetch_fault_hi[1] = 1'b0;
        out_fault_cause[1]    = fgrp_fault_cause;
        l1_parcels            = 2'd0;
        l1_next               = s1;   // no-emit / tail: next lane sees no advance

        // lane1 only exists if lane0 emitted a real instruction and there is
        // room left in the block (s1 <= 7; s1 == 8 means the block is exhausted).
        // P2b: suppress lane1 when lane0 is the terminating branch -- everything
        // past the taken branch in this block is wrong-path.
        if (out_valid[0] && !btb_term0 && (s1 <= 4'd7) && fgrp_valid && !fgrp_excpt) begin
            if (l1_is32 && (s1b == 3'd7)) begin
                out_valid[1] = 1'b0;
                l1_tail      = 1'b1;
            end else if (l1_is32) begin
                out_valid[1]          = 1'b1;
                out_instr[1]          = {hi_parcel(s1b), pblock[s1b]};
                out_is_compressed[1]  = 1'b0;
                l1_parcels            = 2'd2;
                l1_next               = s1 + 4'd2;
                out_fetch_fault[1]    = fgrp_fault_word[l1_lo_w] | fgrp_fault_word[l1_hi_w];
                out_fetch_fault_hi[1] = fgrp_fault_word[l1_hi_w] & ~fgrp_fault_word[l1_lo_w];
            end else begin
                out_valid[1]          = 1'b1;
                out_instr[1]          = exp1_x;
                out_is_compressed[1]  = 1'b1;
                out_rvc_parcel[1]     = lane1_par;
                l1_parcels            = 2'd1;
                l1_next               = s1 + 4'd1;
                out_fetch_fault[1]    = fgrp_fault_word[l1_lo_w];
            end
        end
        // P2b: lane1 as the terminating branch (out_valid[1] is already 0 when
        // btb_term0, so at most one of term0/term1 is set). Computed IN-BLOCK for the
        // same UNOPTFLAT reason as btb_term0.
        l1_last   = l1_is32 ? (s1b + 3'd1) : s1b;
        btb_term1 = btb_hit_in && out_valid[1] && (l1_last == btb_off_in);
    end

`ifdef REALIGN4
    // ---------------- lane 2 (P4: mirrors lane1, start = l1_next) ----------------
    logic        l2_is32;
    logic [1:0]  l2_parcels;
    logic        l2_tail;
    logic [3:0]  s2;            // 0..8; lane2 only valid when s2 <= 7
    logic [2:0]  s2b;
    logic [2:0]  l2_lo_w, l2_hi_w;
    logic [2:0]  l2_last;
    logic        btb_term2;
    logic [3:0]  l2_next;       // start parcel of the instr after lane2 (feeds lane3)
    logic        term_before2;  // an older lane in this block already terminated

    assign s2  = l1_next;
    assign s2b = s2[2:0];
    assign lane2_par    = pblock[s2b];
    assign term_before2 = btb_term0 || btb_term1;
    always_comb begin
        l2_is32   = (lane2_par[1:0] == 2'b11);
        l2_lo_w   = s2[3:1];
        l2_hi_w   = ({1'b0, s2b} + 4'd1) >> 1;
        l2_tail   = 1'b0;
        out_valid[2]          = 1'b0;
        out_pc[2]             = base + {s2b, 1'b0};
        out_instr[2]          = 32'h0000_0013;
        out_is_compressed[2]  = 1'b0;
        out_rvc_parcel[2]     = 16'h0000;
        out_fetch_fault[2]    = 1'b0;
        out_fetch_fault_hi[2] = 1'b0;
        out_fault_cause[2]    = fgrp_fault_cause;
        l2_parcels            = 2'd0;
        l2_next               = s2;

        // lane2 exists only if lane1 emitted, no older lane terminated, and there
        // is room left in the block.
        if (out_valid[1] && !term_before2 && (s2 <= 4'd7) && fgrp_valid && !fgrp_excpt) begin
            if (l2_is32 && (s2b == 3'd7)) begin
                out_valid[2] = 1'b0;
                l2_tail      = 1'b1;
            end else if (l2_is32) begin
                out_valid[2]          = 1'b1;
                out_instr[2]          = {hi_parcel(s2b), pblock[s2b]};
                out_is_compressed[2]  = 1'b0;
                l2_parcels            = 2'd2;
                l2_next               = s2 + 4'd2;
                out_fetch_fault[2]    = fgrp_fault_word[l2_lo_w] | fgrp_fault_word[l2_hi_w];
                out_fetch_fault_hi[2] = fgrp_fault_word[l2_hi_w] & ~fgrp_fault_word[l2_lo_w];
            end else begin
                out_valid[2]          = 1'b1;
                out_instr[2]          = exp2_x;
                out_is_compressed[2]  = 1'b1;
                out_rvc_parcel[2]     = lane2_par;
                l2_parcels            = 2'd1;
                l2_next               = s2 + 4'd1;
                out_fetch_fault[2]    = fgrp_fault_word[l2_lo_w];
            end
        end
        l2_last   = l2_is32 ? (s2b + 3'd1) : s2b;
        btb_term2 = btb_hit_in && out_valid[2] && (l2_last == btb_off_in);
    end

    // ---------------- lane 3 (P4: mirrors lane1, start = l2_next; last lane) ----------------
    logic        l3_is32;
    logic [1:0]  l3_parcels;
    logic        l3_tail;
    logic [3:0]  s3;
    logic [2:0]  s3b;
    logic [2:0]  l3_lo_w, l3_hi_w;
    logic [2:0]  l3_last;
    logic        btb_term3;
    logic        term_before3;

    assign s3  = l2_next;
    assign s3b = s3[2:0];
    assign lane3_par    = pblock[s3b];
    assign term_before3 = btb_term0 || btb_term1 || btb_term2;
    always_comb begin
        l3_is32   = (lane3_par[1:0] == 2'b11);
        l3_lo_w   = s3[3:1];
        l3_hi_w   = ({1'b0, s3b} + 4'd1) >> 1;
        l3_tail   = 1'b0;
        out_valid[3]          = 1'b0;
        out_pc[3]             = base + {s3b, 1'b0};
        out_instr[3]          = 32'h0000_0013;
        out_is_compressed[3]  = 1'b0;
        out_rvc_parcel[3]     = 16'h0000;
        out_fetch_fault[3]    = 1'b0;
        out_fetch_fault_hi[3] = 1'b0;
        out_fault_cause[3]    = fgrp_fault_cause;
        l3_parcels            = 2'd0;

        if (out_valid[2] && !term_before3 && (s3 <= 4'd7) && fgrp_valid && !fgrp_excpt) begin
            if (l3_is32 && (s3b == 3'd7)) begin
                out_valid[3] = 1'b0;
                l3_tail      = 1'b1;
            end else if (l3_is32) begin
                out_valid[3]          = 1'b1;
                out_instr[3]          = {hi_parcel(s3b), pblock[s3b]};
                out_is_compressed[3]  = 1'b0;
                l3_parcels            = 2'd2;
                out_fetch_fault[3]    = fgrp_fault_word[l3_lo_w] | fgrp_fault_word[l3_hi_w];
                out_fetch_fault_hi[3] = fgrp_fault_word[l3_hi_w] & ~fgrp_fault_word[l3_lo_w];
            end else begin
                out_valid[3]          = 1'b1;
                out_instr[3]          = exp3_x;
                out_is_compressed[3]  = 1'b1;
                out_rvc_parcel[3]     = lane3_par;
                l3_parcels            = 2'd1;
                out_fetch_fault[3]    = fgrp_fault_word[l3_lo_w];
            end
        end
        l3_last   = l3_is32 ? (s3b + 3'd1) : s3b;
        btb_term3 = btb_hit_in && out_valid[3] && (l3_last == btb_off_in);
    end
`endif

    // The block TERMINATES -- rvc_block_drained asserts and the block consumes -- the
    // cycle the predicted branch actually dispatches (its lane is in the dispatched
    // set). Continuous: reads only the block-local term vars + dispatch_count.
    assign rvc_btb_terminate = (btb_term0 && (dispatch_count >= 3'd1)) ||
                               (btb_term1 && (dispatch_count >= 3'd2))
`ifdef REALIGN4
                            || (btb_term2 && (dispatch_count >= 3'd3))
                            || (btb_term3 && (dispatch_count >= 3'd4))
`endif
                               ;

    // ---------------- consume / hold / straddle-latch ----------------
    logic [3:0] parcels_dispatched;   // in-block parcels of the dispatched lanes (<= 8)
    logic [3:0] ptr_after;
    logic       tail_straddle;
    logic       tail_straddle_eff;    // P2b: straddle only if the block was NOT terminated
    logic       completion_dispatched;

    always_comb begin
        parcels_dispatched = 4'd0;
        if (dispatch_count >= 3'd1) parcels_dispatched += {2'b0, l0_parcels};
        if (dispatch_count >= 3'd2) parcels_dispatched += {2'b0, l1_parcels};
`ifdef REALIGN4
        if (dispatch_count >= 3'd3) parcels_dispatched += {2'b0, l2_parcels};
        if (dispatch_count >= 3'd4) parcels_dispatched += {2'b0, l3_parcels};
`endif
        ptr_after = {1'b0, s0} + parcels_dispatched;
        // A block-boundary straddle: the next instruction begins at parcel 7 and
        // is a 32-bit low half.
        tail_straddle = (ptr_after == 4'd7) && (pblock[7][1:0] == 2'b11);
        completion_dispatched = completing && (dispatch_count >= 3'd1);
    end

    // These two are CONTINUOUS assigns, deliberately outside the always_comb
    // above: rvc_block_drained (the block's useful parcels are fully drained this
    // cycle -- the P2a BTB "block-ending branch" signal) must depend only on the
    // drain pointer, NEVER on fetch_flush. If it shared an always_comb that reads
    // fetch_flush (as rvc_consume_block does), Verilator would tie block_drained to
    // fetch_flush at block granularity, forming a fetch_flush -> block_drained ->
    // btb_suppress -> fetch_flush UNOPTFLAT loop. The flush gate lives only on
    // rvc_consume_block. Consume once every in-block parcel is drained, or the
    // tail straddles (latch its low half); fgrp_excpt also discards the block.
    // P2b: a BTB-terminated block also drains (block ends AT the predicted branch),
    // and its would-be tail straddle is discarded -- fetch steered to the target,
    // so the parcel-7 low half is wrong-path (never orphaned to base+16). This is
    // the fix for the P2a straddle-vs-steer bug. rvc_btb_terminate depends only on
    // dispatch + the BTB inputs (never fetch_flush), so the UNOPTFLAT invariant holds.
    assign tail_straddle_eff = tail_straddle && !rvc_btb_terminate;
    assign rvc_block_drained = (ptr_after >= 4'd8) || tail_straddle_eff ||
        fgrp_excpt || rvc_btb_terminate;
    assign rvc_consume_block = fgrp_valid && !frontend_hold && !fetch_flush &&
        rvc_block_drained;
    assign rvc_tail_straddle = tail_straddle_eff;   // case-E recover: re-fetch base+14

    // ---------------- oldest un-dispatched frontend PC (interrupt EPC) ----------------
    assign frontend_oldest_valid = straddle_valid_q || (fgrp_valid && !fgrp_excpt);
    assign frontend_oldest_pc    = straddle_valid_q ? straddle_pc_q
                                                    : (base + {s0, 1'b0});

`ifdef DFE_S1
    assign dfe_s0            = s0;
    assign dfe_completing    = completing;
    assign dfe_straddle_pc   = straddle_pc_q;
    assign dfe_straddle_half = straddle_half_q;
`endif

    // ---------------- next state ----------------
    always_comb begin
        align_ptr_n       = align_ptr_q;
        align_ptr_valid_n = align_ptr_valid_q;
        straddle_valid_n  = straddle_valid_q;
        straddle_half_n   = straddle_half_q;
        straddle_pc_n     = straddle_pc_q;
        straddle_fault_n  = straddle_fault_q;
        straddle_cause_n  = straddle_cause_q;

        if (fetch_flush) begin
            // Any redirect: the drain pointer and any dangling half are wrong-path.
            align_ptr_valid_n = 1'b0;
            straddle_valid_n  = 1'b0;
        end else if (frontend_hold) begin
            // Stall/halt: freeze everything (no consume, no advance).
        end else if (fgrp_valid) begin
            if (rvc_consume_block) begin
                align_ptr_valid_n = 1'b0;                 // next block is fresh
                straddle_valid_n  = tail_straddle_eff;    // latch iff tail straddle (not if BTB-terminated)
                if (tail_straddle_eff) begin
                    straddle_half_n  = pblock[7];
                    straddle_pc_n    = base + {3'd7, 1'b0};   // base + 14
                    straddle_fault_n = fgrp_fault_word[3];    // word 3 = parcels 6/7
                    straddle_cause_n = fgrp_fault_cause;
                end
            end else begin
                // Partial drain: hold the block, advance the pointer.
                align_ptr_n = ptr_after[2:0];
                if (completing && !completion_dispatched) begin
                    // completion not yet taken -> stay fresh, re-present it
                    align_ptr_valid_n = 1'b0;
                    straddle_valid_n  = 1'b1;
                end else begin
                    align_ptr_valid_n = 1'b1;
                    straddle_valid_n  = 1'b0;              // completion (if any) taken
                end
            end
        end
    end

    always_ff @(posedge clk) begin
        if (!rst_l) begin
            align_ptr_q       <= 3'd0;
            align_ptr_valid_q <= 1'b0;
            straddle_valid_q  <= 1'b0;
            straddle_half_q   <= 16'h0000;
            straddle_pc_q     <= '0;
            straddle_fault_q  <= 1'b0;
            straddle_cause_q  <= 5'd0;
        end else begin
            align_ptr_q       <= align_ptr_n;
            align_ptr_valid_q <= align_ptr_valid_n;
            straddle_valid_q  <= straddle_valid_n;
            straddle_half_q   <= straddle_half_n;
            straddle_pc_q     <= straddle_pc_n;
            straddle_fault_q  <= straddle_fault_n;
            straddle_cause_q  <= straddle_cause_n;
        end
    end

    // Silence unused-signal lints for the expander illegal outputs (illegal is
    // already carried inside exp*_x as {16'h0000, parcel}) and the tail flags.
    logic _unused;
`ifdef REALIGN4
    assign _unused = &{1'b0, exp0_ill, exp1_ill, exp2_ill, exp3_ill,
                       l0_tail, l1_tail, l2_tail, l3_tail};
`else
    // 2-wide: l1_next is computed but has no lane2 consumer.
    assign _unused = &{1'b0, exp0_ill, exp1_ill, l0_tail, l1_tail, l1_next};
`endif

endmodule : rvc_realign

`default_nettype wire

`endif /* RVC */
