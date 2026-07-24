`ifndef OOO_TYPES_VH_
`define OOO_TYPES_VH_

`include "internal_defines.vh"
`include "riscv_isa.vh"

// RV64C realign lane count (P4). Default 2-wide expand-before-decode realigner;
// -DREALIGN4 widens the realigner and its decode/wire consumers to 4 lanes so the
// RVC frontend can feed all 4 backend dispatch slots. Preprocessor macro (not a
// package localparam) because it sizes module ports. Consumed by rvc_realign.sv,
// ooo_fetch_decode.sv, and riscv_core_ooo.sv. Default keeps the 2-wide path
// behaviourally bit-identical.
`ifdef REALIGN4
  `define RVC_NLANES 4
`else
  `define RVC_NLANES 2
`endif

// ---- Macro-op fusion umbrella (plans/dhry-direct-attacks.md Stage 1; full spec
// plans/dhry-attack-plan/shared-infra.md). Each lever is independently -D-gated +
// composable for A/B; FUSE_ANY = "any fusion compiled in" (shared detector +
// null-slave + born-done), FUSE_BRANCH = "a fused op resolves a branch" (master
// carries branch_id + predictor rewiring). Default unset => bit-identical.
`ifdef FUSE_UADDR
  `define FUSE_ANY
`endif
`ifdef FUSE_CMPBR
  `define FUSE_ANY
  `define FUSE_BRANCH
`endif
`ifdef FUSE_LDBR
  `define FUSE_ANY
  `define FUSE_BRANCH
`endif
// The branch fusions REQUIRE RVC (IALIGN=16 => the fused conditional branch's
// always-even target can never raise the misaligned-target fault, so the folded
// "second sub-op never faults" -- shared-infra §7). On a non-RVC build the
// FUSE_BRANCH payload/logic is compiled OUT (inert): strip the umbrella here so
// every downstream `ifdef FUSE_BRANCH site (all consumers include this header
// first; single-unit preprocessing) sees it undefined.
`ifndef RVC
`ifdef FUSE_BRANCH
  `undef FUSE_BRANCH
`endif
`endif
// FUSE_LDBR rides the FUSE_BRANCH payload (the slave branch's resolve/training
// identity), so on a non-RVC build (FUSE_BRANCH just stripped) the lever is
// fully inert: strip it too and every downstream `ifdef FUSE_LDBR site (all
// consumers include this header first) compiles out.
`ifndef FUSE_BRANCH
`ifdef FUSE_LDBR
  `undef FUSE_LDBR
`endif
`endif

package OOO_Types;

    // Pull in the shared control/ALU types from their package (not $unit) so the
    // helper functions below can name ALU_*/ctrl_signals_t without a package ->
    // $unit reference (Vivado Synth 8-10854).
    import Internal_Defines::*;

    // Re-export the ISA width as an OOO_Types localparam so modules that do
    // `import OOO_Types::*` see XLEN (a wildcard import does not chain the
    // symbols a package itself imported).
    localparam int XLEN = RISCV_ISA::XLEN;

    localparam int OOO_WIDTH = 4;
    // Physical register count (P6 window depth). Default 64 = 32 arch + 32 rename
    // (exactly 32 + ACTIVE_LIST_SIZE, the deadlock floor, so ZERO free-list headroom
    // over the ROB). -DDEEP_WINDOW grows it to 128 to give the free list burst slack:
    // under 4-wide dispatch (-DREALIGN4) the 2-stage commit frees regs late, so at 64
    // the free list starves before the ROB fills (P4 measured qsort freelist_stall 37%).
    // 128 (not 96) because free_list.sv is a power-of-2 ring buffer (bare +1 pointer wrap
    // + free_distance = PHYS_REGS - from + to); a non-pow2 size hands out out-of-range
    // regs. phys_reg_t auto-widens 6->7. ROB (ACTIVE_LIST_SIZE) intentionally unchanged
    // (growing it would raise phys-reg demand, not lower it). Default OFF = bit-identical.
`ifdef DEEP_WINDOW
    localparam int PHYS_REGS = 128;
`else
    localparam int PHYS_REGS = 64;
`endif
    localparam int FP_REGS = 32;
    // Window-depth structures (P6b). Defaults are bit-identical; each grows under
    // its own macro. ACTIVE_LIST (ROB) and MEM_Q (LSQ) are POWER-OF-2 RING BUFFERS
    // (bare +1 pointer wrap + ring distance), so they may ONLY take power-of-2 sizes
    // (48/24 would index out-of-array slots). INT_IQ is a collapsing slot queue, so
    // any size is legal. BIG_ROB=64 needs PHYS_REGS >= 32+64 = 96 (the free-list
    // deadlock floor), so it REQUIRES -DDEEP_WINDOW (PHYS_REGS=128), enforced by the
    // build flags. active_id_t / iq_idx_t / all counts auto-derive via $clog2.
`ifdef BIG_ROB
    localparam int ACTIVE_LIST_SIZE = 64;   // pow2 ring; requires DEEP_WINDOW
`else
    localparam int ACTIVE_LIST_SIZE = 32;
`endif
`ifdef BIG_IQ
    localparam int INT_IQ_SIZE = 24;        // collapsing queue; non-pow2 OK
`else
    localparam int INT_IQ_SIZE = 16;
`endif
`ifdef BIG_LSQ
    localparam int MEM_Q_SIZE = 32;         // pow2 ring
`else
    localparam int MEM_Q_SIZE = 16;
`endif
`ifdef BSTACK_SIZE
    // Explicit checkpoint-depth override (-DBSTACK_SIZE=N, via make BSTACK=N). Takes
    // precedence over BIG_BSTACK. Intended for the capacity-ceiling sweep: N=32 is the
    // "effectively infinite" pool that bounds the whole checkpoint-capacity /
    // stall-on-full lever class in one run. branch_mask_t and branch_id_t auto-widen
    // from this, as do the per-checkpoint rename-map/RAS/GHR copies -- so a large N is
    // functional-sim only (the abort_mask fanout + map-copy area scale with it).
    localparam int BRANCH_STACK_SIZE = `BSTACK_SIZE;
`elsif BIG_BSTACK
    localparam int BRANCH_STACK_SIZE = 8;   // P7 branch-checkpoint depth (doubles
                                            // branch_mask_t + the abort_mask fanout)
`else
    localparam int BRANCH_STACK_SIZE = 4;
`endif
`ifdef ALU4
    // 3rd integer ALU issue port. ISSUE_ALU0/ALU1 stay 0/1; ISSUE_ALU2=2 inserts
    // before MUL/DIV/FP (which shift up), and WB gets a 3rd ALU source. The ALU
    // ports occupy the lowest ALU_ISSUE_PORTS issue indices (the IQ pick relies on
    // this). WB_SOURCES 6->7 (3 ALU + load + MUL + DIV + FP). OFF keeps 2/5/6.
    localparam int ALU_ISSUE_PORTS = 3;
    localparam int FU_ISSUE_PORTS = 6;
`ifdef FUSE_LDBR
    // FUSE_LDBR: +1 WB-only source (the pend_fbr fused-branch resolve — WB_FBR
    // in ooo_writeback_bus, lowest priority, backpressurable, no issue port).
    localparam int WB_SOURCES = 8;
`else
    localparam int WB_SOURCES = 7;
`endif
    localparam int ISSUE_ALU0 = 0;
    localparam int ISSUE_ALU1 = 1;
    localparam int ISSUE_ALU2 = 2;
    localparam int ISSUE_MUL = 3;
    localparam int ISSUE_DIV = 4;
    localparam int ISSUE_FP = 5;
`else
    localparam int ALU_ISSUE_PORTS = 2;
    localparam int FU_ISSUE_PORTS = 5;
`ifdef FUSE_LDBR
    localparam int WB_SOURCES = 7;
`else
    localparam int WB_SOURCES = 6;
`endif
    localparam int ISSUE_ALU0 = 0;
    localparam int ISSUE_ALU1 = 1;
    localparam int ISSUE_MUL = 2;
    localparam int ISSUE_DIV = 3;
    localparam int ISSUE_FP = 4;
`endif

    localparam int ARCH_REG_BITS = 5;
    localparam int PHYS_REG_BITS = $clog2(PHYS_REGS);
    localparam int ACTIVE_ID_BITS = $clog2(ACTIVE_LIST_SIZE);
    localparam int BRANCH_ID_BITS = $clog2(BRANCH_STACK_SIZE);

`ifdef DISPATCH_STATS
    // Dispatch-stall / group-truncation reason codes (instrumentation only; no
    // datapath effect). ooo_dispatch_control emits these at the exact point its
    // combinational ladder fires, so they cannot drift from the logic the way a
    // re-derived classifier in the core would.
    //
    // DCUT_*: why stop_prefix first went 0->1 this cycle, i.e. why the dispatch
    // group was cut. Mirrors the ladder in ooo_dispatch_control.sv in order.
    localparam logic [3:0] DCUT_BR_TERM   = 4'd0;  // a DISPATCHED branch ends the group
    localparam logic [3:0] DCUT_2ND_BR    = 4'd1;  // 2nd branch in the group
    localparam logic [3:0] DCUT_2ND_MEM   = 4'd2;  // 2nd memory op
    localparam logic [3:0] DCUT_2ND_FP    = 4'd3;  // 2nd FP op
    localparam logic [3:0] DCUT_FP_BUSY   = 4'd4;  // FP source/WAW FPR still busy
    localparam logic [3:0] DCUT_FREELIST  = 4'd5;  // dest_seen >= free_list_available
    localparam logic [3:0] DCUT_TERM_PREV = 4'd6;  // previous lane is terminal
    localparam logic [3:0] DCUT_TERM_CUR  = 4'd7;  // this lane terminal, prefix dispatched
    localparam logic [3:0] DCUT_SER_PREV  = 4'd8;  // previous lane serializing
    localparam logic [3:0] DCUT_SER_CUR   = 4'd9;  // this lane serializing, prefix dispatched
    localparam logic [3:0] DCUT_BSTACK    = 4'd10; // branch + branch stack full
    localparam logic [3:0] DCUT_FUSE_BST  = 4'd11; // fused pre-branch + branch stack full
    localparam logic [3:0] DCUT_NONE      = 4'd15;

    // FF_*: which redirect arm won pc_next on a fetch_flush cycle. The redirect
    // always_comb is a SEQUENTIAL-OVERRIDE chain (the commit-scan / trap / fence.i-hold
    // arms overwrite the main ladder), so the LAST writer wins -- ff_src is assigned
    // beside every `fetch_flush = 1'b1` and its final value is the arm that actually
    // determined pc_next. Listed low->high priority.
    localparam logic [3:0] FF_NONE        = 4'd0;
    localparam logic [3:0] FF_REDIRECT    = 4'd1;  // branch recovery (mispredict/abort)
    localparam logic [3:0] FF_RAS         = 4'd2;  // return-address-stack redirect
    localparam logic [3:0] FF_PRED        = 4'd3;  // predictor-taken (TAGE/JAL) redirect
    localparam logic [3:0] FF_UNPRED      = 4'd4;  // unpredicted JAL/JALR dispatched
    localparam logic [3:0] FF_BTBMIS      = 4'd5;  // BTB steered wrong
    localparam logic [3:0] FF_PARTIAL     = 4'd6;  // non-RVC partial-dispatch refetch
    localparam logic [3:0] FF_FENCEI      = 4'd7;  // fence.i redirect
    localparam logic [3:0] FF_SFENCE      = 4'd8;  // sfence.vma
    localparam logic [3:0] FF_SATP        = 4'd9;  // satp write
    localparam logic [3:0] FF_TRAP        = 4'd10; // trap / interrupt / mret / sret
    localparam logic [3:0] FF_FENCEI_HOLD = 4'd11; // fence.i L1D-writeback hold
    localparam logic [3:0] FF_OTHER       = 4'd15; // UNCLASSIFIED -- must stay 0


    // DSTL_*: which structural term asserted dispatch_stall (the whole-group stall).
    // DSTL_SUPPRESS is decomposed further by the core, which owns those 11 signals.
    localparam logic [3:0] DSTL_SUPPRESS  = 4'd0;
    localparam logic [3:0] DSTL_ROB_FULL  = 4'd1;
    localparam logic [3:0] DSTL_IQ_FULL   = 4'd2;
    localparam logic [3:0] DSTL_MEMQ_FULL = 4'd3;
    localparam logic [3:0] DSTL_BSTACK    = 4'd4;  // branch + stack full (any lane, first cut)
    localparam logic [3:0] DSTL_NONE      = 4'd15;
`endif

`ifdef CSWHY
    // ================= CSWHY: why is the ROB head not retiring? =================
    // Two-axis (CLASS x STATE) decomposition of commit_starved_backend, binned per
    // existing cs arm. Every code is emitted by the module that OWNS the fact (the
    // DSTAT_CUT / ff_src rule) -- the core performs zero classification.
    // spec: plans/cslat-decomposition-spec.md
    localparam int CSWHY_NCLASS = 17;
    localparam int CSWHY_NSTATE = 34;

    // ---- CLASS axis. State classes FIRST: C_UNWRITTEN reads instr==0, so decoding
    // before the state test would mis-bill it.
    localparam logic [5:0] C_NOHEAD    = 6'd0;  // no live head entry
    localparam logic [5:0] C_UNWRITTEN = 6'd1;  // slot reserved, payload write deferred D->Q
    localparam logic [5:0] C_ABORTED   = 6'd2;  // valid but squashed this cycle (zeroing deferred)
    localparam logic [5:0] C_ZERO      = 6'd3;  // live head with instr==0 -- ALARM, predicted 0
    localparam logic [5:0] C_LOAD      = 6'd4;
    localparam logic [5:0] C_FLOAD     = 6'd5;
    localparam logic [5:0] C_STORE     = 6'd6;
    localparam logic [5:0] C_FSTORE    = 6'd7;
    localparam logic [5:0] C_ATOMIC    = 6'd8;
    localparam logic [5:0] C_BRANCH    = 6'd9;
    localparam logic [5:0] C_JUMP      = 6'd10;
    localparam logic [5:0] C_MUL       = 6'd11;
    localparam logic [5:0] C_DIV       = 6'd12;
    localparam logic [5:0] C_FP        = 6'd13;
    localparam logic [5:0] C_SER       = 6'd14; // SYSTEM / MISC-MEM
    localparam logic [5:0] C_ALU       = 6'd15;
    localparam logic [5:0] C_UNKNOWN   = 6'd16; // REACHABLE catch-all; nonzero is a finding

    // ---- STATE axis, non-memory. S_SELECT MUST outrank the IQ arms: the IQ clears a
    // slot only in entries_sel, so entries_wake still claims it on the select cycle.
    localparam logic [5:0] S_NA        = 6'd0;  // class in {NOHEAD,UNWRITTEN,ABORTED}
    localparam logic [5:0] S_DONE      = 6'd1;  // done, awaiting the present/commit stage
    localparam logic [5:0] S_SELECT    = 6'd2;  // picked by the IQ this cycle
    localparam logic [5:0] S_ALU_EXEC  = 6'd3;
    localparam logic [5:0] S_MUL_EXEC  = 6'd4;
    localparam logic [5:0] S_DIV_EXEC  = 6'd5;
    localparam logic [5:0] S_FP_EXEC   = 6'd6;
    localparam logic [5:0] S_FU_WB     = 6'd7;  // result at an FU output, WB bus not accepting
    localparam logic [5:0] S_IQ_PICK   = 6'd8;  // in IQ, READY, not picked = issue contention
    localparam logic [5:0] S_IQ_OPWAIT = 6'd9;  // in IQ, NOT ready = TRUE operand wait
    // The ALU REGISTERS its writeback (ooo_alu_pipe.sv:196-203) unlike mul/div, so the
    // cycle its WB flop drives the bus has no other claimant -- the ALU-side analogue of
    // M_DRAINED. Without this it falls to S_NOWHERE (measured 38% of qsort).
    localparam logic [5:0] S_ALU_WB    = 6'd10;
    localparam logic [5:0] S_NOWHERE   = 6'd11; // REACHABLE catch-all
    // ---- STATE axis, memory. Admitted ONLY when the LSQ head IS the ROB head.
    localparam logic [5:0] M_SKEW      = 6'd16; // arm #1: headq aims at a different entry
    localparam logic [5:0] M_RETIRING  = 6'd17;
    localparam logic [5:0] M_ADDRWAIT  = 6'd18; // NOT memory -- operand wait in a memory costume
    localparam logic [5:0] M_STDATA    = 6'd19; // NOT memory -- store-data operand wait
    localparam logic [5:0] M_PARK      = 6'd20; // completed store parked (expect ~0 at the ROB head)
    localparam logic [5:0] M_LOADWAIT  = 6'd21; // head's OWN outstanding load
    localparam logic [5:0] M_MLPWAIT   = 6'd22; // someone ELSE's outstanding load (the A5 contamination)
    localparam logic [5:0] M_XLATE_WALK= 6'd23; // PTW (provably 0 in bare mode)
    localparam logic [5:0] M_XLATE_REG = 6'd24; // the FB2b registered-translate stage
    localparam logic [5:0] M_PORT      = 6'd25; // L1D accept port busy
    localparam logic [5:0] M_XFAULT    = 6'd26;
    localparam logic [5:0] M_MEMOTHER  = 6'd27; // REACHABLE catch-all (now residual after the split)
    localparam logic [5:0] M_DRAINED   = 6'd28; // memory class but already popped the LSQ (legal)
    // M_MEMOTHER split (2026-07-23): the head is ready by every gate but the REGISTERED
    // ROB done bit is 0. These distinguish the causes that were lumped as MEMOTHER (29%).
    localparam logic [5:0] M_COMPLETING= 6'd29; // LSQ head_done=1 THIS cycle: writeback->done register skew
    localparam logic [5:0] M_STWAIT    = 6'd30; // store ready, awaiting the in-order ROB commit handshake
    localparam logic [5:0] M_LOADBLK   = 6'd31; // load ready + port-free but did not fire (MLP/translate throttle)
    localparam logic [5:0] M_TWOBEAT   = 6'd32; // 2nd beat of a two-beat store in flight
    localparam logic [5:0] M_LOADFIRE  = 6'd33; // load request-issue cycle (cycle 0 of its memory wait)

    // ---- ARM: which existing cs bucket. The existing 3-way ladder is FOLLOWED, not
    // modified. NOTE the ARM_LAT guard contains !active_full, which is CAUSED by the
    // stalls being measured, so cycles migrate LAT->WINDOW: ALWAYS report the
    // arm-summed cell, never ARM_LAT alone.
    localparam logic [1:0] ARM_GATED  = 2'd0;
    localparam logic [1:0] ARM_WINDOW = 2'd1;
    localparam logic [1:0] ARM_LAT    = 2'd2;

    // ONE pure opcode decoder, so a head-side and any future retire-side histogram can
    // never disagree. Decodes the architectural 32-bit word the ROB stored.
    function automatic logic [4:0] cswhy_class_for(input logic [31:0] instr);
        logic [6:0] op; logic [6:0] f7; logic [2:0] f3;
        op = instr[6:0]; f7 = instr[31:25]; f3 = instr[14:12];
        // Opcode BEFORE is_store: is_store = ctrl.memWrite, which SC and RMW AMO both
        // set and LR sets neither -- keying on it mis-bills every atomic.
        case (op)
            7'h03: cswhy_class_for = C_LOAD;
            7'h07: cswhy_class_for = C_FLOAD;
            7'h23: cswhy_class_for = C_STORE;
            7'h27: cswhy_class_for = C_FSTORE;
            7'h2F: cswhy_class_for = C_ATOMIC;
            7'h63: cswhy_class_for = C_BRANCH;
            7'h67, 7'h6F: cswhy_class_for = C_JUMP;
            7'h33, 7'h3B: cswhy_class_for = (f7 == 7'd1) ? (f3[2] ? C_DIV : C_MUL) : C_ALU;
            7'h53, 7'h43, 7'h47, 7'h4B, 7'h4F: cswhy_class_for = C_FP;
            7'h73, 7'h0F: cswhy_class_for = C_SER;
            7'h13, 7'h1B, 7'h37, 7'h17: cswhy_class_for = C_ALU;
            default: cswhy_class_for = C_UNKNOWN;
        endcase
    endfunction

    // XC-CLASS side B: does rename's ROUTING verdict (captured at dispatch) agree with
    // the architectural opcode the ROB stored? Different decoder, different time,
    // different path -- so this is NOT a tautology. Returns 1 when consistent.
    // NOTE exec_class is EXEC_INT for ALU *and* MUL/DIV, so this checks the
    // mem / cf / fp / amo / ser boundaries, not the mul-vs-div split.
    function automatic logic cswhy_xclass_ok(
            input logic [4:0] cls, input logic is_mem_r, input logic is_mem_w,
            input logic [2:0] xclass, input logic [2:0] pcsrc);
        case (cls)
            C_LOAD, C_FLOAD:   cswhy_xclass_ok = is_mem_r;
            C_STORE, C_FSTORE: cswhy_xclass_ok = is_mem_w;
            C_ATOMIC:          cswhy_xclass_ok = (xclass == 3'(EXEC_AMO));
            C_BRANCH:          cswhy_xclass_ok = (pcsrc == 3'(PC_cond));
            C_JUMP:            cswhy_xclass_ok = (pcsrc == 3'(PC_uncond)) ||
                                                 (pcsrc == 3'(PC_indirect));
            C_FP:              cswhy_xclass_ok = (xclass == 3'(EXEC_FP));
            C_SER:             cswhy_xclass_ok = (xclass == 3'(EXEC_CSR)) ||
                                                 (xclass == 3'(EXEC_FENCE));
            C_ALU, C_MUL, C_DIV: cswhy_xclass_ok = !is_mem_r && !is_mem_w &&
                                                 (pcsrc != 3'(PC_cond));
            default:           cswhy_xclass_ok = 1'b1;   // state classes: not checkable
        endcase
    endfunction
`endif

`ifdef FUSE_ANY
    // Fusion-kind encoding for a detected pair (shared-infra §1); the master's
    // fuse_kind tells its FU which folded second op to execute.
    localparam logic [1:0] FUSE_K_NONE  = 2'd0;
    localparam logic [1:0] FUSE_K_UADDR = 2'd1;
    localparam logic [1:0] FUSE_K_CMPBR = 2'd2;
    localparam logic [1:0] FUSE_K_LDBR  = 2'd3;
`endif

    typedef logic [ARCH_REG_BITS-1:0] arch_reg_t;
    typedef logic [PHYS_REG_BITS-1:0] phys_reg_t;
    typedef logic [ACTIVE_ID_BITS-1:0] active_id_t;
    typedef logic [BRANCH_STACK_SIZE-1:0] branch_mask_t;
    typedef logic [BRANCH_ID_BITS-1:0] branch_id_t;
    typedef logic [63:0] fp_reg_data_t;

    typedef enum logic [2:0] {
        FU_ALU,
        FU_MUL,
        FU_DIV,
        FU_FP,
        FU_MEM,
        FU_DC = 3'bxxx
    } fu_class_t;

    typedef struct packed {
        logic        valid;
        logic        predicted_taken;
        logic        predicted_target_valid;
        logic [XLEN-1:0] predicted_target;
        logic [1:0]  provider;
        logic [9:0]  base_index;
        logic [9:0]  index0;
        logic [9:0]  index1;
        logic [9:0]  index2;
        logic [9:0]  tag0;
        logic [9:0]  tag1;
        logic [9:0]  tag2;
        logic [9:0]  sc_history;
        // Carried counter/useful reads from the lookup snapshot, so the resolve
        // update can be WRITE-ONLY (no array read) -> the predictor tables map to
        // sync-read SRAM. All at the same indices the lookup read (update_pc ==
        // lookup_pc, carried indices/sc_history), so these are the values the
        // update's async read would have returned (modulo staleness, which the
        // self-correcting best-effort tables tolerate). TAGE uses base_ctr / ctr*
        // (counters) / use* / sc_bias; ITTAGE reuses ctr* as confidences + use*.
        logic [1:0]  base_ctr;
        logic [1:0]  ctr0;
        logic [1:0]  ctr1;
        logic [1:0]  ctr2;
        logic [1:0]  use0;
        logic [1:0]  use1;
        logic [1:0]  use2;
        logic signed [5:0] sc_bias;
    } predictor_info_t;

    typedef struct packed {
        logic          valid;
        logic [XLEN-1:0]   pc;
        logic [31:0]   instr;
        ctrl_signals_t ctrl;
        arch_reg_t     rs1;
        arch_reg_t     rs2;
        arch_reg_t     rd;
        phys_reg_t     prs1;
        phys_reg_t     prs2;
        phys_reg_t     prd;
        phys_reg_t     old_prd;
        logic          src1_ready;
        logic          src2_ready;
        logic          has_dest;
        logic [XLEN-1:0]   imm;
        branch_mask_t  branch_mask;
        branch_id_t    branch_id;
        active_id_t    active_id;
        logic          control_predicted;
        logic [XLEN-1:0]   predicted_pc;
        predictor_info_t predictor_info;
        arch_reg_t     fp_rs1;
        arch_reg_t     fp_rs2;
        arch_reg_t     fp_rs3;
        arch_reg_t     fp_rd;
        fp_reg_data_t   fp_src1_data;
        fp_reg_data_t   fp_src2_data;
        fp_reg_data_t   fp_src3_data;
        fu_class_t      fu_class;
`ifdef FUSE_ANY
        // Macro-op fusion payload (shared-infra §2). Master: this op executes a
        // folded second op; slave: born-done ROB NOP (no FU, no arch write).
        logic          is_fused;
        logic [1:0]    fuse_kind;     // FUSE_K_*
        logic          fused_slave;
        logic [XLEN-1:0] fuse_imm;    // slave imm: ADDI imm (UADDR) / SB target imm (CMP/LDBR)
        alu_op_t       fuse_alu_op;   // slave op: ADD/ADDW (UADDR) / branch cmp (CMP/LDBR)
`endif
`ifdef FUSE_UADDR
        // FUSE_UADDR (c): surviving load's AGU base is pc_auipc (= pc-4), not
        // rs1_data; its imm carries the folded hi+off (fuse-uaddr.md §3c/§4).
        logic          use_pc_base;
`endif
`ifdef FUSE_BRANCH
        // Branch-fusion payload: the master hosts the slave branch's resolve, so
        // it carries the slave's checkpoint id + prediction/training identity.
        logic          fuse_is_branch;      // master hosts a fused conditional-branch resolve
        logic [1:0]    fuse_pc_source;      // slave pc_source (always PC_cond for these levers)
        branch_id_t    fuse_branch_id;      // = slave's branch_allocate_id (checkpoint X)
        logic          fuse_control_predicted;
        logic [XLEN-1:0] fuse_predicted_pc; // slave's predicted target (mispredict compare)
        logic [XLEN-1:0] fuse_branch_pc;    // slave.pc — TAGE/BTB training + GHR "taken" test
        logic [31:0]   fuse_branch_instr;   // slave.instr — training keys on [6:0]==OP_BRANCH
        predictor_info_t fuse_predictor_info;
        // Slave branch's instruction length (FUSE_BRANCH implies RVC): the fused
        // fall-through + training "taken" tests need the SLAVE's ILEN, not the
        // master's (fuse-cmpbr.md §7a).
        logic          fuse_is_compressed;
`endif
`ifdef FUSE_CMPBR_LI
        // FUSE_CMPBR Case B (li fusion): the fused branch's 2nd compare operand
        // rides the master's (otherwise unused) rs2 port — the branch's OTHER
        // source register — instead of x0 (fuse-cmpbr.md §7b).
        logic          fuse_cmp_rs2;
`endif
`ifdef FUSE_LDBR
        // FUSE_LDBR: the slave branch's OWN ROB slot (distinct from the load
        // master's active_id) — the fused resolve retires it via the pend_fbr
        // writeback (fuse-ldbr.md §HP4; 2-slot model, instret untouched).
        active_id_t    fuse_br_active_id;
        // Set on the LDBR SLAVE lane: this folded slave is NOT born-done (its
        // ROB entry waits for the fused-resolve writeback so it can never
        // commit ahead of its checkpoint resolve — consumed in active_list).
        logic          fuse_slave_ldbr;
`endif
    } rename_packet_t;

    typedef struct packed {
        logic          valid;
        logic [XLEN-1:0]   pc;
        logic [31:0]   instr;
        ctrl_signals_t ctrl;
        phys_reg_t     prs1;
        phys_reg_t     prs2;
        phys_reg_t     prd;
        logic          src1_ready;
        logic          src2_ready;
        logic          has_dest;
        logic [XLEN-1:0]   imm;
        branch_mask_t  branch_mask;
        branch_id_t    branch_id;
        active_id_t    active_id;
        logic          control_predicted;
        logic [XLEN-1:0]   predicted_pc;
        predictor_info_t predictor_info;
        arch_reg_t     fp_rs1;
        arch_reg_t     fp_rs2;
        arch_reg_t     fp_rs3;
        arch_reg_t     fp_rd;
        fp_reg_data_t   fp_src1_data;
        fp_reg_data_t   fp_src2_data;
        fp_reg_data_t   fp_src3_data;
        fu_class_t      fu_class;
`ifdef FUSE_ANY
        // Macro-op fusion payload, mirrored from rename_packet_t (shared-infra §2).
        logic          is_fused;
        logic [1:0]    fuse_kind;
        logic          fused_slave;
        logic [XLEN-1:0] fuse_imm;
        alu_op_t       fuse_alu_op;
`endif
`ifdef FUSE_UADDR
        // Mirrored from rename_packet_t (FUSE_UADDR (c) LSQ AGU pc-base select).
        logic          use_pc_base;
`endif
`ifdef FUSE_BRANCH
        logic          fuse_is_branch;
        logic [1:0]    fuse_pc_source;
        branch_id_t    fuse_branch_id;
        logic          fuse_control_predicted;
        logic [XLEN-1:0] fuse_predicted_pc;
        logic [XLEN-1:0] fuse_branch_pc;
        logic [31:0]   fuse_branch_instr;
        predictor_info_t fuse_predictor_info;
        logic          fuse_is_compressed;  // slave branch's ILEN flag (RVC)
`endif
`ifdef FUSE_CMPBR_LI
        logic          fuse_cmp_rs2;        // Case B: compare vs rs2_data, not x0
`endif
`ifdef FUSE_LDBR
        // Mirrored from rename_packet_t: the slave branch's own ROB slot (the
        // LSQ's fused-resolve emission hands it to pend_fbr).
        active_id_t    fuse_br_active_id;
`endif
    } issue_entry_t;

    typedef struct packed {
        logic          valid;
        active_id_t    active_id;
        logic [XLEN-1:0]   pc;
        logic [31:0]   instr;
        phys_reg_t     prd;
        logic          has_dest;
        logic [XLEN-1:0]   data;
        branch_mask_t  branch_mask;
        logic          branch_valid;
        branch_id_t    branch_id;
        logic          branch_mispredict;
        logic [XLEN-1:0]   redirect_pc;
        logic          control_predicted;
        logic [XLEN-1:0]   predicted_pc;
        predictor_info_t predictor_info;
        logic          fp_write;
        arch_reg_t     fp_rd;
        fp_reg_data_t   fp_data;
        logic          csr_write;
        logic [11:0]   csr_addr;
        logic [XLEN-1:0]   csr_wdata;
        logic          fp_fflags_valid;
        logic [4:0]    fp_fflags;
        logic          exception;
        logic [4:0]    exc_cause;
        logic          halted;
`ifdef FUSE_BRANCH
        // Branch-fusion identity (shared-infra §2): when the master resolves a
        // fused branch its writeback drives the branch-resolve + predictor-train
        // paths with the SLAVE's identity, not the master's.
        logic          fuse_is_branch;
        logic [XLEN-1:0] fuse_branch_pc;
        logic [31:0]   fuse_branch_instr;
        logic          fuse_is_compressed;  // slave branch's ILEN flag (RVC)
`endif
`ifdef RVC
        // RV64C: instruction length flag for branch-predictor training (the
        // "taken" test compares redirect_pc against pc + ILEN, not pc + 4).
        logic          is_compressed;
`endif
    } writeback_packet_t;

    typedef struct packed {
        logic          valid;
        active_id_t    active_id;
        arch_reg_t     rd;
        phys_reg_t     prd;
        phys_reg_t     old_prd;
        logic          has_dest;
        logic [XLEN-1:0]   pc;
        logic [31:0]   instr;
        logic [XLEN-1:0]   data;
        logic          fp_write;
        arch_reg_t     fp_rd;
        fp_reg_data_t   fp_data;
        logic          csr_write;
        logic [11:0]   csr_addr;
        logic [XLEN-1:0]   csr_wdata;
        logic          fp_fflags_valid;
        logic [4:0]    fp_fflags;
        logic          serializing;
        logic          is_store;
        logic          is_sc;        // M4-S5b: store-conditional (memWrite + EXEC_AMO + AMO_SC)
        logic          is_amo;       // M4 #3: RMW atomic (EXEC_AMO, amo_op not LR/SC)
        logic          halted;
        logic          exception;
        logic [4:0]    exc_cause;
`ifdef RVC
        // RV64C: is_compressed feeds ILEN and rvc_parcel is the original 16-bit
        // parcel used for the illegal-compressed mtval (the .instr field stays
        // the canonical expanded 32-bit word). Both allocated at dispatch,
        // carried through the ROB like .instr.
        logic          is_compressed;
        logic [15:0]   rvc_parcel;
`endif
    } commit_packet_t;

    // FB1 debug-observability probe (FPGA bring-up). A pure tap off existing
    // commit-stage signals -- zero functional change. Carried as one struct
    // port from riscv_core_ooo up through niigo_soc to the OCL debug block,
    // only wired under FPGA_BUILD. Feeds: committed-PC ring + instret counter
    // (retire_*), a shadow architectural regfile (arch_*), and a trap log
    // (trap_*). `halted` surfaces the ECALL/tohost halt for STATUS.
    typedef struct packed {
        logic [OOO_WIDTH-1:0]                    retire_valid; // per-lane actual retire
        logic [OOO_WIDTH-1:0][XLEN-1:0]          retire_pc;    // committed PC per lane
        logic [OOO_WIDTH-1:0]                    arch_we;      // committed arch-reg write
        logic [OOO_WIDTH-1:0][ARCH_REG_BITS-1:0] arch_rd;      // destination arch reg
        logic [OOO_WIDTH-1:0][XLEN-1:0]          arch_data;    // committed value
        logic                                    trap_valid;   // precise trap taken
        logic                                    trap_is_int;  // interrupt vs exception
        logic [4:0]                              trap_cause;   // cause code
        logic [XLEN-1:0]                         trap_epc;     // faulting/return PC
        logic [XLEN-1:0]                         trap_tval;    // trap value
        logic                                    halted;       // core quiesced
        logic                                    hpm_l1i_miss; // cache event pulses
        logic                                    hpm_l1d_miss; // (counted in ocl_csr)
        logic                                    hpm_l1d_wb;
    } debug_probe_t;

    function automatic logic is_mul_op(input alu_op_t op);
        is_mul_op = (op == ALU_MUL) || (op == ALU_MULH) ||
            (op == ALU_MULHSU) || (op == ALU_MULHU) || (op == ALU_MULW);
    endfunction

    function automatic logic is_div_op(input alu_op_t op);
        is_div_op = (op == ALU_DIV) || (op == ALU_DIVU) ||
            (op == ALU_REM) || (op == ALU_REMU) ||
            (op == ALU_DIVW) || (op == ALU_DIVUW) ||
            (op == ALU_REMW) || (op == ALU_REMUW);
    endfunction

    function automatic fu_class_t fu_class_for(input ctrl_signals_t ctrl);
        if (ctrl.memRead || ctrl.memWrite) begin
            fu_class_for = FU_MEM;
        end else if (ctrl.exec_class == EXEC_FP) begin
            fu_class_for = FU_FP;
        end else if ((ctrl.exec_class == EXEC_INT) && is_mul_op(ctrl.alu_op)) begin
            fu_class_for = FU_MUL;
        end else if ((ctrl.exec_class == EXEC_INT) && is_div_op(ctrl.alu_op)) begin
            fu_class_for = FU_DIV;
        end else begin
            fu_class_for = FU_ALU;
        end
    endfunction

endpackage: OOO_Types

`endif /* OOO_TYPES_VH_ */
