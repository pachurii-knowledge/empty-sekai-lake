`include "ooo_types.vh"
`include "superscalar_types.vh"
`include "riscv_abi.vh"
`include "memory_segments.vh"
`include "riscv_priv.vh"

// The fence.i / halt write-back D-side flush handshake (defer the pc+4 redirect,
// raise dcache_flush_req, hold the frontend until dcache_flush_done, then
// invalidate the L1I and refetch) is needed by ANY write-back D-side: the L1D
// cache (L1D_CACHE) and the M3d grant-and-go MOESI agent (CCD_AGENT). Derive one
// guard so both get byte-identical behavior; the L1=0 / passthrough builds keep
// the immediate fence.i redirect. (undef'd after endmodule to avoid leakage.)
`ifdef L1D_CACHE
  `define NIIGO_DSIDE_WB
`elsif CCD_AGENT
  `define NIIGO_DSIDE_WB
`endif

`ifdef LOAD_SPEC_WAKE
`ifndef L1D_CACHE
  // LOAD_SPEC_WAKE requires the L1D's 1-cycle hit latency as its spec-wake
  // timing anchor (load-spec-wake.md §top): on the passthrough memsys the
  // fixed D_RESP_DELAY (>1) makes every spec-wake resolve as a miss.
  `error "LOAD_SPEC_WAKE requires L1D_CACHE (1-cycle hit anchor); use L1D=1 / PERF=1"
`endif
`endif

`default_nettype none

module riscv_core_ooo
    import OOO_Types::*;
    import RISCV_ISA::XLEN_BYTES;
    import RISCV_UArch::MEMORY_READ_WIDTH, RISCV_UArch::MEMORY_ADDR_WIDTH;
#(
    parameter logic [XLEN-1:0] HART_ID = '0,  // M4: per-core mhartid (default 0 = single core; XLEN via OOO_Types)
    parameter bit              COHERENT = 1'b0 // M4 B9: SC resolves rd at commit (multi-core); default 0 = single-core verbatim
)
(
    input wire logic             clk, rst_l,

    // Instruction-fetch port: one 16-byte block per request, handshaked.
    // Responses arrive in request order after an arbitrary >= 1 cycle
    // latency; the core associates them with requests by arrival order.
    output logic             ifetch_req_valid,
    input wire logic             ifetch_req_ready,
    output logic [MEMORY_ADDR_WIDTH-1:0] ifetch_req_addr,
    input wire logic             ifetch_resp_valid,
    input wire logic [MEMORY_READ_WIDTH-1:0][XLEN-1:0] ifetch_resp_data,
    input wire logic             ifetch_resp_excpt,

    // Data port: one word-granular load or store per request, handshaked.
    // Load responses arrive in order after an arbitrary >= 1 cycle latency
    // (the request word address is echoed for the device decode at
    // delivery); accepted stores are fire-and-forget.
    output logic             dmem_req_valid,
    input wire logic             dmem_req_ready,
    output logic             dmem_req_write,
    output logic [MEMORY_ADDR_WIDTH-1:0] dmem_req_addr,
    output logic [XLEN-1:0]  dmem_req_wdata,
    output logic [XLEN_BYTES-1:0]        dmem_req_wmask,
    // M3d Stage 2: typed memory-op code (LOAD/STORE/LR/AMO_RD) for the CCD L1D
    // agent; ignored by the L1D/passthrough memsys arms. See load_store_queue.sv.
    output logic [2:0]       dmem_req_op,
    // M4 #3: fine AMO sub-op accompanying a COP_AMO beat (agent-authoritative AMO).
    output logic [3:0]       dmem_req_amo,
    input wire logic             dmem_resp_valid,
    input wire logic [MEMORY_ADDR_WIDTH-1:0] dmem_resp_addr,
    input wire logic [XLEN-1:0]  dmem_resp_data,
    // M3d Stage 3 (S1): snoop-kill from the CCD agent (remote write -> reservation-kill /
    // spec-load squash in the LSQ). Constant 0 in non-CCD + single-core CCD builds.
    input wire logic             dmem_snoop_kill_valid,
    input wire logic [MEMORY_ADDR_WIDTH-1:0] dmem_snoop_kill_laddr,

    // MMU page-table-walk port: the walker's level req/ack word port (PTE
    // reads + A/D writebacks); rdata is valid in the ack cycle.
    output logic             ptw_mem_req,
    output logic             ptw_mem_we,
    output logic [MEMORY_ADDR_WIDTH-1:0] ptw_mem_addr_w,
    output logic [XLEN-1:0]  ptw_mem_wdata,
    input wire logic             ptw_mem_ack,
    input wire logic [XLEN-1:0]  ptw_mem_rdata,

    // fence.i: flash-invalidate the L1I (consumed by niigo_memsys at L1=1;
    // ignored by the L1=0 passthrough). Pulsed when a fence.i retires.
    output logic             ifetch_inval,
    // Cacheable/device split for the data port (L1D=1): high when the current
    // dmem request targets a memory-mapped device (CLINT/PLIC/UART) and must
    // bypass the L1D. Ignored by the L1=0/C1 passthrough.
    output logic             dmem_req_device,
    // L1D writeback flush handshake: the core raises dcache_flush_req on a
    // fence.i and holds the frontend until dcache_flush_done (memsys writes back
    // all dirty L1D lines). Under L1=0/C1 the memsys completes it immediately.
    output logic             dcache_flush_req,
    input wire logic             dcache_flush_done,
    // Cache event pulses from the memsys for mhpmcounter3-5 (phase C3).
    input wire logic             hpm_l1i_miss,
    input wire logic             hpm_l1d_miss,
    input wire logic             hpm_l1d_wb,
`ifdef NIIGO_EXT_DEVICES
    // M4 SMP: the per-core CLINT/PLIC/UART are lifted to ONE shared instance at
    // the SMP top. This core EXPORTS its committed-store snoop + device-load
    // query and IMPORTS the shared device's load result, mtime, and the four
    // per-hart interrupt lines. (NIIGO_EXT_DEVICES is set only by the SMP build
    // targets; the default build never sees these ports or the external arm.)
    output logic             dsnoop_store_en,
    output logic [MEMORY_ADDR_WIDTH-1:0] dsnoop_store_waddr,
    output logic [XLEN-1:0]  dsnoop_store_wdata,
    output logic [XLEN_BYTES-1:0] dsnoop_store_mask,
    output logic [MEMORY_ADDR_WIDTH-1:0] dsnoop_load_addr,
    output logic             dsnoop_load_en,
    output logic [$clog2(XLEN_BYTES)-1:0] dsnoop_load_off,
    input  wire logic        ext_load_hit,
    input  wire logic [XLEN-1:0] ext_load_data,
    input  wire logic [63:0] ext_mtime,
    input  wire logic        ext_irq_m_timer,
    input  wire logic        ext_irq_m_software,
    input  wire logic        ext_irq_m_external,
    input  wire logic        ext_irq_s_external,
`endif

    output logic             halted
`ifdef LSQ_MLP2
    ,
    // Track A: dmem transaction id, threaded LSQ->core->memsys->l1_dcache and back
    // so a data response can be matched to its outstanding slot (P3c parked
    // completion). Const-0 at P3a. Width via DMEM_ID_W (body localparam). Placed
    // last so it composes with the optional FPGA_BUILD port block.
    output logic [DMEM_ID_W-1:0] dmem_req_id,
    input  wire logic [DMEM_ID_W-1:0] dmem_resp_id
`endif
`ifdef FPGA_BUILD
    ,
    // ---- FB1: vUART byte streams (rerouted from the sim console) + a debug
    // observability probe (pure tap off the commit stage). Both inert unless
    // FPGA_BUILD; see niigo_soc.sv / cl_niigo.sv.
    output logic             vuart_tx_valid,
    output logic [7:0]       vuart_tx_byte,
    input  wire logic        vuart_rx_valid,
    input  wire logic [7:0]  vuart_rx_byte,
    output logic             vuart_rx_pop,
    output debug_probe_t     dbg_probe
`endif
);

    // Byte-address -> word-address shift for the word-granular memory bus.
    localparam int ADDR_SHIFT = $clog2(XLEN_BYTES);

`ifdef LSQ_MLP2
    // Track A dmem transaction-id width. LSQ_MLP in {1,2} (a coherent build pins 1)
    // => 1 bit; matches the LSQ's LSQ_ID_W and the memsys id ports. Revisit for MLP>2.
    localparam int DMEM_ID_W = 1;
`endif

    import RISCV_ABI::ECALL_ARG_HALT;
    import MemorySegments::USER_TEXT_START;

    localparam int PHYS_READ_PORTS = FU_ISSUE_PORTS + OOO_WIDTH;
    // Return-address stack depth. This is a pure prediction hint: a RAS
    // mispredict is caught and recovered at JALR resolve, so depth affects
    // accuracy (call-nesting reach) but never architectural correctness.
    // 128 was heavily over-provisioned -- typical call-nesting that the RAS
    // usefully predicts is shallow. 32 is a standard depth and cuts the
    // ras_stack flop array 4x (128->32 entries x XLEN) plus the fetch-cycle
    // 128:1 top-of-stack read mux down to 32:1 (ASAP7 area+timing; the array
    // maps to flops, not SRAM, since push/pop/checkpoint want parallel access).
    // Target-selected (results-identical hint sizing): ASAP7 = 32, FPGA/sim = 128
    // (the FPGA lineage's validated depth; BRAM/LUT area there is cheap).
`ifdef NIIGO_ASIC
    localparam int RAS_DEPTH = 32;    // ASAP7 7nm ASIC
`else
    localparam int RAS_DEPTH = 128;   // FPGA + functional sim
`endif
    localparam int RAS_INDEX_BITS = $clog2(RAS_DEPTH);
    localparam int RAS_COUNT_BITS = $clog2(RAS_DEPTH + 1);
    localparam int DIRECT_HISTORY_BITS = 30;

    logic [XLEN-1:0] pc_q, pc_next;
    logic [XLEN-1:0] fetch_pa;   // translated fetch address (driven in MMU section)
    // Fetch fault for pc_q's group, computed combinationally in the MMU
    // section and captured into the request metadata at issue.
    logic        fetch_fault;            // combinational: any lane of pc_q faults
    logic [OOO_WIDTH-1:0] fetch_fault_lane;  // per-lane fetch fault for pc_q's group
    logic [4:0]  fetch_fault_cause;

    // ---- Handshaked fetch frontend ----
    // Up to FETCH_DEPTH fetch requests may be in flight or buffered at once
    // (outstanding requests tracked by a small in-order metadata FIFO, plus a
    // 2-entry fetched-group buffer presented to decode). Responses arrive in
    // request order; redirect events mark every outstanding request killed
    // (its response is dropped on arrival) and clear the group buffer, so no
    // stale group is ever presented -- the role the old fixed-depth
    // fetch_pc/fetch_valid pipe clears played, without assuming a fixed
    // memory latency.
    localparam int FETCH_DEPTH = 2;
    typedef struct packed {
        logic                 valid;
        logic                 kill;
        logic [XLEN-1:0]      pc;
        logic [OOO_WIDTH-1:0] fault_lane;
        logic [4:0]           fault_cause;
`ifdef BTB
        // P2a: when this block was fetched, the BTB steered the NEXT fetch to
        // btb_tgt (btb_hit=1). Carried to decode so it can verify the steer
        // against B2's real prediction and suppress the flush on a block-ending
        // agree. btb_hit=0 => fetch continued sequentially after this block.
        // btb_off (P2b) = the predicted branch's last in-block parcel (the offset
        // at which the realigner terminates the block on a steered hit).
        logic                 btb_hit;
        logic [XLEN-1:0]      btb_tgt;
        logic [2:0]           btb_off;
`endif
    } fetch_meta_t;
    typedef struct packed {
        logic                 valid;
        logic [XLEN-1:0]      pc;
        logic [MEMORY_READ_WIDTH-1:0][XLEN-1:0] data;
        logic                 excpt;
        logic [OOO_WIDTH-1:0] fault_lane;
        logic [4:0]           fault_cause;
`ifdef BTB
        logic                 btb_hit;
        logic [XLEN-1:0]      btb_tgt;
        logic [2:0]           btb_off;
`endif
    } fetch_group_t;
    fetch_meta_t  fmeta_q [FETCH_DEPTH];
    fetch_meta_t  fmeta_next [FETCH_DEPTH];
    logic         fmeta_rd_q, fmeta_rd_next;   // FETCH_DEPTH == 2: 1-bit ptrs
    logic         fmeta_wr_q, fmeta_wr_next;
    logic [1:0]   fmeta_cnt_q, fmeta_cnt_next;
    fetch_group_t fbuf_q [2];                  // [0] = head (presented)
    fetch_group_t fbuf_next [2];
    logic [1:0]   fbuf_cnt_q, fbuf_cnt_next;
    // Presented group (head of the buffer, or the arriving response bypassed
    // when the buffer is empty).
    logic         fgrp_valid;
    logic [XLEN-1:0] fgrp_pc;
    logic [MEMORY_READ_WIDTH-1:0][XLEN-1:0] fgrp_data;
    logic         fgrp_excpt;
    logic [OOO_WIDTH-1:0] fgrp_fault_lane;
    logic [4:0]   fgrp_fault_cause;
    // Control strobes computed in the main combinational block.
    logic         fetch_flush;     // kill all outstanding + buffered fetches
`ifdef DISPATCH_STATS
    logic [3:0]   ff_src;          // stats only: which redirect arm won this flush
`endif
    logic         fetch_consume;   // pop the presented group
    logic         fetch_issue;     // issue a fetch for pc_q this cycle
    logic         fresp_take;      // a response arrives (pops metadata)
    logic         fresp_live;      // ...and its requester was not killed
    fetch_meta_t  fmeta_head;
    assign fmeta_head = fmeta_q[fmeta_rd_q];
    assign fresp_take = ifetch_resp_valid;
    assign fresp_live = fresp_take && fmeta_head.valid && !fmeta_head.kill;
    assign fgrp_valid       = (fbuf_cnt_q != 2'd0) ? fbuf_q[0].valid : fresp_live;
    assign fgrp_pc          = (fbuf_cnt_q != 2'd0) ? fbuf_q[0].pc    : fmeta_head.pc;
    assign fgrp_data        = (fbuf_cnt_q != 2'd0) ? fbuf_q[0].data  : ifetch_resp_data;
    assign fgrp_excpt       = (fbuf_cnt_q != 2'd0) ? fbuf_q[0].excpt : ifetch_resp_excpt;
    assign fgrp_fault_lane  = (fbuf_cnt_q != 2'd0) ? fbuf_q[0].fault_lane
                                                   : fmeta_head.fault_lane;
    assign fgrp_fault_cause = (fbuf_cnt_q != 2'd0) ? fbuf_q[0].fault_cause
                                                   : fmeta_head.fault_cause;
    // P2a: suppress a decode-stage redirect flush when the BTB already steered
    // fetch to the correct block-ending target. Declared unconditionally (it
    // gates the always-present redirect arms); tied 0 when the BTB is disabled.
    logic            btb_suppress;
`ifdef BTB
    logic            fgrp_btb_hit;
    logic [XLEN-1:0] fgrp_btb_tgt;
    logic [2:0]      fgrp_btb_off;
    assign fgrp_btb_hit = (fbuf_cnt_q != 2'd0) ? fbuf_q[0].btb_hit : fmeta_head.btb_hit;
    assign fgrp_btb_tgt = (fbuf_cnt_q != 2'd0) ? fbuf_q[0].btb_tgt : fmeta_head.btb_tgt;
    assign fgrp_btb_off = (fbuf_cnt_q != 2'd0) ? fbuf_q[0].btb_off : fmeta_head.btb_off;

    // P2a fetch-directed BTB. Looked up on pc_next (the block about to be fetched)
    // so its sync-read result is ready the cycle that block becomes pc_q: a hit
    // then steers this cycle's pc_next to the target (stream N->T, no wrong-path
    // N+16) and tags the block's fmeta entry. Every steer is verified at decode
    // (and, on a mispredict, at execute), so this is a pure performance hint.
    logic            btb_pred_valid;
    logic [XLEN-1:0] btb_pred_target;
    logic [2:0]      btb_pred_offset;
    logic [1:0]      btb_pred_type;
    logic [XLEN-1:0] btb_lk_blk_q;     // block(pc_next) looked up last cycle
    logic            btb_hit_now;      // BTB hit for the block being fetched (pc_q)
    logic            btb_train_valid, btb_train_taken;
    logic [XLEN-1:0] btb_train_pc, btb_train_target;
    logic [2:0]      btb_train_offset;
    logic [1:0]      btb_train_type;

    btb #(.SETS(512)) Btb (
        .clk(clk), .rst_l(rst_l),
        .lookup_valid(1'b1),
        .lookup_pc(pc_next),
        .pred_valid(btb_pred_valid),
        .pred_target(btb_pred_target),
        .pred_offset(btb_pred_offset),
        .pred_type(btb_pred_type),
        .train_valid(btb_train_valid),
        .train_taken(btb_train_taken),
        .train_pc(btb_train_pc),
        .train_target(btb_train_target),
        .train_offset(btb_train_offset),
        .train_type(btb_train_type)
    );
    always_ff @(posedge clk or negedge rst_l) begin
        if (!rst_l) btb_lk_blk_q <= '0;
        else        btb_lk_blk_q <= {pc_next[XLEN-1:4], 4'b0};
    end
    // Steer only if the registered prediction is for the block we are actually
    // fetching now (a redirect may have moved pc_q off the looked-up pc_next).
    assign btb_hit_now = btb_pred_valid &&
        (btb_lk_blk_q == {pc_q[XLEN-1:4], 4'b0});

    // ---- decode-time verify + train (drives B7's redirect arms + the BTB) ----
    // Scoped to predictor_redirect ONLY (TAGE-taken conditionals, P1 JALs,
    // ITTAGE-hit indirects). RETURNS (ras_redirect) are RAS-predicted with a
    // dynamic target -- caching them in the BTB pollutes it, so the ras arm below
    // stays un-suppressed and returns are never trained/steered.
    logic [XLEN-1:0] btb_seq_for_fgrp;    // sequential next block (base + 16)
    logic            btb_mis_steer;       // any steer that must flush + redirect
    logic [XLEN-1:0] btb_mis_recover_pc;  // where a mis-steer redirects to
    assign btb_seq_for_fgrp = {fgrp_pc[XLEN-1:4], 4'b0} + XLEN'(16);
`ifdef RVC
    // ---- P2b OFFSET-PRECISE verify: rvc_realign terminates the block AT btb_off
    // (the predicted branch's last parcel), so the branch and its target are the
    // whole in-stream picture -- no wrong-path tail parcels, no orphaned straddle.
    logic [2:0]      btb_branch_off;
    logic            btb_branch_straddle;  // 32-bit branch low half at parcel 7
    logic [XLEN-1:0] btb_fallthrough_pc;   // continue after a not-taken terminator
    logic            btb_mis_term;         // terminated, but resolves not-taken/non-branch
    logic            btb_mis_drain;        // steered + drained without matching btb_off
    assign btb_branch_off      = btb_branch_pc[3:1] + (btb_branch_is_c ? 3'd0 : 3'd1);
    assign btb_branch_straddle = !btb_branch_is_c && (btb_branch_pc[3:1] == 3'd7);
    // fall-through of the terminating branch = base + 2*(off + 1) (off is straddle-free
    // so off <= 7 and off+1 <= 8 => base .. base+16).
    assign btb_fallthrough_pc  = {fgrp_pc[XLEN-1:4], 4'b0} +
        XLEN'({1'b0, fgrp_btb_off, 1'b0} + 12'd2);
    // WIN: the realigner terminated AT the predicted branch and it resolves taken to
    // the BTB target -- the target is already streaming, so suppress the flush.
    // NOTE: every verify arm is gated on fgrp_valid. After a mis-steer flush the group
    // buffer empties (fgrp_valid=0) but fmeta_head/pblock keep STALE btb_hit + tail_straddle,
    // which would keep btb_mis_drain asserted -> perpetual flush -> fetch never re-issues ->
    // hang (observed at xv6 pc 8000041e). btb_suppress/btb_mis_term are implicitly gated
    // (rvc_btb_terminate <= out_valid[0] <= fgrp_valid) but carry it for clarity.
    assign btb_suppress = fgrp_valid && rvc_btb_terminate && predictor_redirect_valid &&
        (predictor_redirect_pc == fgrp_btb_tgt);
    // MIS-STEER (terminated wrong): the block terminated at btb_off but no redirect
    // fired (branch not-taken, or the offset instr is not a branch) -- recover to the
    // fall-through. A redirect to a DIFFERENT target needs no special case: btb_suppress
    // is 0, so the normal predictor_redirect arm flushes + redirects to the real target.
    assign btb_mis_term = fgrp_valid && rvc_btb_terminate && !predictor_redirect_valid &&
        !ras_redirect_valid && !dispatched_unpredicted_control;
    // MIS-STEER (aliased/stale): steered, but the block drained WITHOUT terminating at
    // btb_off (offset landed mid-instruction / the block never branches there) and no
    // redirect fired -- recover to base+16, or re-fetch base+14 to re-establish a
    // pending tail straddle the flush would otherwise orphan.
    assign btb_mis_drain = fgrp_valid && fgrp_btb_hit && rvc_block_drained && !rvc_btb_terminate &&
        !predictor_redirect_valid && !ras_redirect_valid && !dispatched_unpredicted_control;
    assign btb_mis_steer = btb_mis_term || btb_mis_drain;
    assign btb_mis_recover_pc = btb_mis_term ? btb_fallthrough_pc :
        (rvc_tail_straddle ? ({fgrp_pc[XLEN-1:4], 4'b0} + XLEN'(14)) : btb_seq_for_fgrp);
    // TRAIN: cache the taken branch's block + terminating offset; REFUSE a straddling
    // 32-bit branch (its high half lives in the next block -> cannot be a clean in-block
    // terminate; refusing it is the P2a straddle-bug fix). Invalidate on any mis-steer.
    assign btb_train_valid = (!dispatch_stall && !halted_q) &&
        ((predictor_redirect_valid && !btb_branch_straddle) || btb_mis_steer);
    assign btb_train_taken  = predictor_redirect_valid && !btb_branch_straddle;
    assign btb_train_pc     = fgrp_pc;
    assign btb_train_target = predictor_redirect_pc;
    assign btb_train_offset = btb_branch_off;
    assign btb_train_type   = 2'd0;
`else
    // ---- non-RVC: block-granular P2a. Fixed 4-byte insns => no realigner, no
    // straddle, so block-ending steering is correct as-is. (Mid-block non-RVC via a
    // tail mask is a future extension; block-ending covers loop back-edges.)
    logic btb_block_ending;
    assign btb_block_ending = fgrp_valid && (dispatch_count == valid_count);
    assign btb_suppress = predictor_redirect_valid && fgrp_btb_hit &&
        (fgrp_btb_tgt == predictor_redirect_pc) && btb_block_ending;
    assign btb_mis_steer = fgrp_btb_hit && btb_block_ending &&
        !predictor_redirect_valid && !ras_redirect_valid &&
        !dispatched_unpredicted_control && (fgrp_btb_tgt != btb_seq_for_fgrp);
    assign btb_mis_recover_pc = btb_seq_for_fgrp;
    assign btb_train_valid = (!dispatch_stall && !halted_q) &&
        ((predictor_redirect_valid && btb_block_ending) || btb_mis_steer);
    assign btb_train_taken  = predictor_redirect_valid && btb_block_ending;
    assign btb_train_pc     = fgrp_pc;
    assign btb_train_target = predictor_redirect_pc;
    assign btb_train_offset = 3'd0;
    assign btb_train_type   = 2'd0;
`endif
`else
    assign btb_suppress = 1'b0;
`endif
    logic halted_q, halted_next;
    logic terminal_pending_q, terminal_pending_next;
    logic control_pending_q, control_pending_next;
    branch_id_t control_pending_id_q, control_pending_id_next;
    logic frontend_stall;
    logic redirect_valid;
    logic [XLEN-1:0] redirect_pc;
    logic [XLEN-1:0] sequential_next_pc;

    decode_lane_t decode_lanes [OOO_WIDTH];
    logic [OOO_WIDTH-1:0] lane_valid;
    logic [OOO_WIDTH-1:0] lane_has_dest;
    logic [OOO_WIDTH-1:0] lane_is_branch;
`ifdef JAL_NO_CKPT
    logic [OOO_WIDTH-1:0] lane_needs_ckpt;  // lane_is_branch minus JAL (PC_uncond)
`endif
    logic [OOO_WIDTH-1:0] lane_is_unpredicted_control;
    logic [OOO_WIDTH-1:0] lane_is_call;
    logic [OOO_WIDTH-1:0] lane_is_return;
    logic [OOO_WIDTH-1:0] lane_control_predicted;
    logic [OOO_WIDTH-1:0][XLEN-1:0] lane_predicted_pc;
    logic [OOO_WIDTH-1:0] lane_is_memory;
    logic [OOO_WIDTH-1:0] lane_is_terminal;
    logic [OOO_WIDTH-1:0] lane_is_serializing;
    // P5b FP de-serialization scoreboard taps (driven 0 unless -DFP_OOO).
    logic [OOO_WIDTH-1:0] lane_is_fp;
    logic [OOO_WIDTH-1:0] lane_fp_src_busy;
    logic [OOO_WIDTH-1:0] lane_reads_fflags;
    logic                 fflags_drain_stall;
`ifdef FUSE_ANY
    // Macro-op fusion shared detector outputs (plans/dhry-attack-plan/shared-infra.md
    // §3): lane i is the older op of a fusable ADJACENT pair (i, i+1); lane i+1 is
    // the folded younger op. Decode-derived ONLY (never dispatch_valid) so no
    // combinational loop through dispatch_control is introduced.
    logic [OOO_WIDTH-1:0] fuse_master;
    logic [OOO_WIDTH-1:0] fuse_slave;
    logic [OOO_WIDTH-1:0][1:0] fuse_kind_lane;
`endif
`ifdef FUSE_LDBR
    // FUSE_LDBR: lane i is the SLAVE of an LDBR pair — NOT born-done (its ROB
    // entry retires via the pend_fbr fused-resolve writeback; active_list).
    logic [OOO_WIDTH-1:0] fuse_slave_ldbr;
    // pend_fbr: the 1-deep fused-branch-resolve buffer + its LSQ handshake.
    writeback_packet_t pend_fbr_q, pend_fbr_next;
    writeback_packet_t fbr_writeback;         // pend_fbr_q, drive-gated
    logic            pend_fbr_full;           // -> LSQ fused-issue gate
    logic            pend_fbr_drain_ready;    // WB bus accepted/aborted it
    logic            fused_resolve_valid;
    active_id_t      fused_resolve_active_id;
    branch_id_t      fused_resolve_branch_id;
    logic [XLEN-1:0] fused_resolve_pc;
    logic [31:0]     fused_resolve_instr;
    logic [XLEN-1:0] fused_resolve_redirect_pc;
    logic            fused_resolve_mispredict;
    logic            fused_resolve_ctrl_pred;
    logic [XLEN-1:0] fused_resolve_pred_pc;
    predictor_info_t fused_resolve_pred_info;
    branch_mask_t    fused_resolve_branch_mask;
    logic            fused_resolve_is_comp;
`endif
`ifdef FUSE_UADDR
    // FUSE_UADDR (c) survivor overrides (fuse-uaddr.md §2): the surviving
    // YOUNGER load keeps its lane but reads x0 for rs1 (map_rs1<-0), takes the
    // folded hi+off immediate, and selects the LSQ AGU pc-base path. The nulled
    // OLDER auipc rides the shared fuse_slave[] null path (born-done ROB slot).
    logic [OOO_WIDTH-1:0]           fu_zero_rs1;
    logic [OOO_WIDTH-1:0]           fu_imm_ovr;
    logic [OOO_WIDTH-1:0][XLEN-1:0] fu_imm;
    logic [OOO_WIDTH-1:0]           fu_pc_base;
`endif
`ifdef FUSE_BRANCH
    // Fuse-master whose folded slave is a branch (drives dispatch_control's
    // atomic-pair branch-stack stall, shared-infra §4b).
    logic [OOO_WIDTH-1:0] lane_fuse_pre_branch;
`endif
`ifdef FUSE_CMPBR_LI
    // FUSE_CMPBR Case B (li fusion, fuse-cmpbr.md §7b): lane i is a li master
    // whose folded branch compares against the branch's OTHER source register
    // (not x0). fu_caseb_src is that arch register; the master re-reads it via
    // its (otherwise unused) rs2 rename port (the li reads only x0).
    logic [OOO_WIDTH-1:0]           fu_caseb;
    logic [OOO_WIDTH-1:0][4:0]      fu_caseb_src;
`endif
    logic [OOO_WIDTH-1:0] dispatch_valid;
    logic [OOO_WIDTH-1:0] alloc_req;
    logic [OOO_WIDTH-1:0] map_has_dest;
    logic dispatch_stall;
`ifdef DISPATCH_STATS
    logic       dstat_cut_valid;
    logic [3:0] dstat_cut_reason;
    logic [$clog2(OOO_WIDTH+1)-1:0] dstat_cut_idx;
    logic [3:0] dstat_stall_reason;
`endif
    logic [2:0] dispatch_count;
    logic [2:0] valid_count;
    logic [2:0] lane_active_offset [OOO_WIDTH];
    logic partial_resume_valid;
    logic [2:0] partial_resume_lane;
    logic partial_resume_lane_is_branch;
    logic dispatched_unpredicted_control;
    logic ras_redirect_valid;
    logic [XLEN-1:0] ras_redirect_pc;
    logic predictor_redirect_valid;
    logic [XLEN-1:0] predictor_redirect_pc;
    // P2b: PC + is_compressed of the predictor-redirecting branch, captured in the
    // B2 predictor loop, so the BTB can train its terminating parcel offset.
    logic [XLEN-1:0] btb_branch_pc;
    logic            btb_branch_is_c;

    logic [XLEN-1:0] ras_stack_q [RAS_DEPTH];
    logic [XLEN-1:0] ras_stack_next [RAS_DEPTH];
    logic [RAS_COUNT_BITS-1:0] ras_count_q, ras_count_next;
    logic [RAS_COUNT_BITS-1:0] ras_checkpoint_count_q [BRANCH_STACK_SIZE];
    logic [RAS_COUNT_BITS-1:0] ras_checkpoint_count_next [BRANCH_STACK_SIZE];
    logic [RAS_COUNT_BITS-1:0] ras_branch_snapshot_count;

    logic [DIRECT_HISTORY_BITS-1:0] ghr_q, ghr_next;
    logic [DIRECT_HISTORY_BITS-1:0] ghr_checkpoint_q [BRANCH_STACK_SIZE];
    logic [DIRECT_HISTORY_BITS-1:0] ghr_checkpoint_next [BRANCH_STACK_SIZE];
    logic [DIRECT_HISTORY_BITS-1:0] ghr_branch_snapshot;

    // Predictor sync-read hold. The TAGE/ITTAGE tables are sync-read, so the
    // prediction for the presented group's lookup_pc is registered and available
    // the NEXT cycle. When a branch-bearing group first appears we hold it one
    // cycle (suppress dispatch -> also freezes fetch via frontend_stall, and
    // freezes pc_q/ghr_q) so B2 consumes the registered prediction matched to the
    // SAME (fgrp_pc, ghr_q). Architecturally inert: predictions are hints and
    // every branch is precisely recovered at resolve, so a mismatched/stale/late
    // prediction can only change cycle counts, never a committed result. The
    // (pc, ghr) key is exact because ghr_q cannot change during a non-flush hold
    // (the speculative push is under dispatch_valid; a restore implies a flush
    // that discards the held group).
    logic pred_ready_q;
    logic [XLEN-1:0] pred_key_pc_q;
    logic [DIRECT_HISTORY_BITS-1:0] pred_key_ghr_q;
    logic pred_launch, prediction_ready, predict_stall;
    // A branch-bearing group is presented and its sync-read lookup is in flight.
    // DFE_S2: the DIRECT (TAGE, conditional) read is combinational (async) under
    // -DDFE_S2 (tage_sc_l_predictor.sv), so direct_prediction is available the same
    // cycle the branch is at the dispatch head -- no hold needed. Only the INDIRECT
    // (ITTAGE) sync read still holds (S2b territory). Default: both terms (bit-identical).
    assign pred_launch = fgrp_valid && !halted_q &&
`ifdef DFE_S2
                         indirect_lookup_valid;
`else
                         (direct_lookup_valid || indirect_lookup_valid);
`endif
    // The registered prediction is ready and matches THIS held group's key.
    assign prediction_ready = pred_ready_q && (pred_key_pc_q == fgrp_pc) &&
                              (pred_key_ghr_q == ghr_q);
    assign predict_stall = pred_launch && !prediction_ready;

    logic direct_lookup_valid;
    logic [XLEN-1:0] direct_lookup_pc;
    logic direct_prediction;
    predictor_info_t direct_prediction_info;
    logic indirect_lookup_valid;
    logic [XLEN-1:0] indirect_lookup_pc;
    logic indirect_prediction_valid;
    logic [XLEN-1:0] indirect_prediction_target;
    predictor_info_t indirect_prediction_info;
    predictor_info_t lane_predictor_info [OOO_WIDTH];

    arch_reg_t map_rs1 [OOO_WIDTH];
    arch_reg_t map_rs2 [OOO_WIDTH];
    arch_reg_t map_rd [OOO_WIDTH];
    phys_reg_t map_prs1 [OOO_WIDTH];
    phys_reg_t map_prs2 [OOO_WIDTH];
    phys_reg_t map_old_prd [OOO_WIDTH];
    phys_reg_t map_snapshot [32];

    logic [OOO_WIDTH-1:0] free_alloc_valid;
    phys_reg_t free_alloc_prd [OOO_WIDTH];
    logic free_can_allocate;
    logic [$clog2(PHYS_REGS)-1:0] free_head_snapshot;
    logic [$clog2(PHYS_REGS)-1:0] free_tail_snapshot;
    logic [$clog2(PHYS_REGS+1)-1:0] free_count_snapshot;
    logic [OOO_WIDTH-1:0] active_free_valid;
    phys_reg_t active_free_prd [OOO_WIDTH];

    logic [OOO_WIDTH-1:0] busy_src1_ready;
    logic [OOO_WIDTH-1:0] busy_src2_ready;
    logic [OOO_WIDTH-1:0] wakeup_valid;
    phys_reg_t wakeup_prd [OOO_WIDTH];

    rename_packet_t rename_packets [OOO_WIDTH];
    issue_entry_t dispatch_issue_entries [OOO_WIDTH];
    logic [OOO_WIDTH-1:0] int_insert_valid;
    logic [OOO_WIDTH-1:0] mem_insert_valid;

    logic active_full;
    logic active_empty;
    active_id_t active_tail;
    logic [OOO_WIDTH-1:0] active_commit_valid;
    commit_packet_t active_commit_packet [OOO_WIDTH];

    logic int_iq_full;
    logic [FU_ISSUE_PORTS-1:0] int_issue_valid;
    logic [FU_ISSUE_PORTS-1:0] int_issue_ready;
    issue_entry_t int_issue_entry [FU_ISSUE_PORTS];

    // Select -> Execute pipeline register (S2) for the two ALU ports. The IQ select
    // (S1) registers the chosen entry here; the ALU pipe / regfile read / CSR read
    // all run from the registered copy in S2. Halves the single-cycle select+wakeup
    // +regread+execute critical cone. MUL/DIV/FP keep their combinational issue path
    // (own multi-cycle pipelines, not on the critical path). spec_wake_* broadcasts
    // an in-S2 ALU producer's dest one cycle early for zero-bubble ALU->ALU chains.
    logic [ALU_ISSUE_PORTS-1:0] alu_issue_valid_q;
    issue_entry_t alu_issue_entry_q [ALU_ISSUE_PORTS];
    logic [ALU_ISSUE_PORTS-1:0] spec_wake_valid;
    phys_reg_t spec_wake_prd [ALU_ISSUE_PORTS];

`ifdef LOAD_SPEC_WAKE
    // LOAD_SPEC_WAKE (plans/dhry-attack-plan/load-spec-wake.md): the LSQ's
    // hit-predicted load broadcast (wired LSQ -> IQ directly) plus the core's
    // one-cycle verdict. ld_spec_pending_q tracks a broadcast made last cycle;
    // a response this cycle is that load's HIT (sole-outstanding at broadcast),
    // its absence the MISS. Under LSQ_MLP2 the verdict id-matches
    // dmem_resp_id vs the broadcast's allocated inflight id (adversarial
    // finding 1: never trust raw dmem_resp_valid). alu_issue_ld_spec_q is the
    // S2 poison riding alongside alu_issue_valid_q; iq_issue_ld_spec is the
    // IQ's per-port pick poison.
    logic       lsq_load_spec_wake_valid;
    phys_reg_t  lsq_load_spec_wake_prd;
    logic       ld_spec_pending_q;
    logic       ld_spec_hit, ld_spec_miss;
    logic [ALU_ISSUE_PORTS-1:0] iq_issue_ld_spec;
    logic [ALU_ISSUE_PORTS-1:0] alu_issue_ld_spec_q;
`ifdef LSQ_MLP2
    logic [DMEM_ID_W-1:0] lsq_load_spec_wake_id;
    logic [DMEM_ID_W-1:0] ld_spec_id_q;
`endif
`endif

    phys_reg_t phys_rs1 [PHYS_READ_PORTS];
    phys_reg_t phys_rs2 [PHYS_READ_PORTS];
    logic [PHYS_READ_PORTS-1:0][XLEN-1:0] phys_rs1_data;
    logic [PHYS_READ_PORTS-1:0][XLEN-1:0] phys_rs2_data;
    logic [OOO_WIDTH-1:0][XLEN-1:0] mem_insert_rs1_data;
    logic [OOO_WIDTH-1:0][XLEN-1:0] mem_insert_rs2_data;
    logic [OOO_WIDTH-1:0] phys_write_valid;
    phys_reg_t phys_write_prd [OOO_WIDTH];
    logic [OOO_WIDTH-1:0][XLEN-1:0] phys_write_data;

    writeback_packet_t alu0_writeback;
    writeback_packet_t alu1_writeback;
`ifdef ALU4
    writeback_packet_t alu2_writeback;   // 3rd integer ALU issue port
`endif
    writeback_packet_t load_writeback;
    writeback_packet_t mul_writeback;
    writeback_packet_t div_writeback;
    writeback_packet_t fp_writeback;
    writeback_packet_t branch_writeback;
    logic mul_writeback_ready;
    logic div_writeback_ready;
    logic fp_writeback_ready;
    logic [OOO_WIDTH-1:0] writeback_valid;
    active_id_t writeback_active_id [OOO_WIDTH];
    phys_reg_t writeback_prd [OOO_WIDTH];
    logic [OOO_WIDTH-1:0][XLEN-1:0] writeback_data;
    logic [OOO_WIDTH-1:0] writeback_has_dest;
    logic [OOO_WIDTH-1:0] writeback_fp_write;
    arch_reg_t writeback_fp_rd [OOO_WIDTH];
    fp_reg_data_t writeback_fp_data [OOO_WIDTH];
    logic [OOO_WIDTH-1:0] writeback_csr_write;
    logic [OOO_WIDTH-1:0][11:0] writeback_csr_addr;
    logic [OOO_WIDTH-1:0][XLEN-1:0] writeback_csr_wdata;
    logic [OOO_WIDTH-1:0] writeback_fp_fflags_valid;
    logic [OOO_WIDTH-1:0][4:0] writeback_fp_fflags;
    logic [OOO_WIDTH-1:0] writeback_exception;
    logic [OOO_WIDTH-1:0][4:0] writeback_exc_cause;
    logic [OOO_WIDTH-1:0] writeback_halted;

    logic [XLEN-1:0] csr_read_data [2];
    logic [1:0] csr_read_illegal;
    logic csr_commit_write;
    logic [11:0] csr_commit_addr;
    logic [XLEN-1:0] csr_commit_wdata;
    logic csr_fp_fflags_valid;
    logic [4:0] csr_fp_fflags;
    logic [2:0] csr_frm;
    logic [2:0] retire_count;   // # instructions retired this cycle (0..OOO_WIDTH)

    // --- Privileged-ISA / trap state (Phase 3) ---
    // Architectural privilege + CSR state exposed by priv_csr_file.
    RISCV_Priv::priv_mode_t cur_priv;
    logic [XLEN-1:0] csr_mstatus, csr_medeleg, csr_mideleg, csr_mie, csr_mip;
    logic [XLEN-1:0] csr_mtvec, csr_stvec, csr_mepc, csr_sepc, csr_satp;
    logic [31:0] csr_pmpcfg_arr [4];
    logic [XLEN-1:0] csr_pmpaddr_arr [16];
    logic        csr_menvcfg_adue;
    logic [63:0] clint_mtime;
    logic        irq_mtimer, irq_msoft;
    logic        clint_load_hit;
    logic [XLEN-1:0] clint_load_data;
    logic        plic_load_hit;
    logic [XLEN-1:0] plic_load_data;
    logic        plic_m_ext, plic_s_ext;
    logic        uart_load_hit;
    logic [XLEN-1:0] uart_load_data;
    logic        uart_irq;
    logic [ADDR_SHIFT-1:0] dev_load_off;   // head load byte offset (from LSQ)

    // Commit-time trap evaluation (driven combinationally in the commit block).
    logic        commit_exc_valid;
    logic [4:0]  commit_exc_cause;
    logic [XLEN-1:0] commit_exc_tval;
    logic [XLEN-1:0] commit_trap_epc;
    logic        commit_take_trap, commit_take_ret, commit_ret_from_s;
    // trap_controller outputs
    logic        tc_trap_valid, tc_is_int;
    logic [4:0]  tc_cause;
    RISCV_Priv::priv_mode_t tc_target;
    logic [XLEN-1:0] tc_vector;
    logic [XLEN-1:0] trap_redirect_pc;

    // --- Precise interrupts via ROB drain (Phase 3b) ---
    // When an interrupt is pending+enabled we stop dispatching new instructions
    // and let the ROB drain. Once empty, the interrupt is taken with epc set to
    // the oldest undispatched instruction. All of this is gated by
    // irq_pending_now, which is identically zero whenever interrupts are
    // disabled (e.g. every RV32G test), so it has no effect there.
    logic        irq_pending_now;
    logic [31:0] irq_eff;
    logic        m_irq_en, s_irq_en;
    logic        irq_drain_q, irq_drain_next;
    logic        commit_take_int;
    logic [XLEN-1:0] commit_int_epc;
    // WFI: while wfi_wait_q the core idles (dispatch suppressed) until an
    // enabled interrupt is pending (wfi_wake), ignoring the global enable.
    logic        wfi_wait_q, wfi_wait_next, wfi_wait_set, wfi_wake;
    assign wfi_wake = (csr_mip & csr_mie) != 32'b0;

    fp_reg_data_t fp_regs_q [FP_REGS];
    fp_reg_data_t fp_regs_next [FP_REGS];
    logic serial_pending_q, serial_pending_next;

`ifdef FP_OOO
    // P5b: single-producer arch-FPR scoreboard. fpr_busy_q[x]=1 while an FP writer
    // to FPR x is in flight (dispatch->retire); a WAW dispatch-stall keeps at most
    // one writer per FPR live, so the producer's aged branch_mask alone suffices
    // for abort recovery (fpr_prod_mask_q, aged by ~reset_mask -- the same trick
    // niigo_fp_unit uses for its single in-flight op). No branch_stack snapshot.
    logic [FP_REGS-1:0] fpr_busy_q, fpr_busy_next;
    branch_mask_t       fpr_prod_mask_q   [FP_REGS];
    branch_mask_t       fpr_prod_mask_next [FP_REGS];
`endif

    logic branch_stack_full;
    logic branch_allocate;
    logic branch_allocate_valid;
    branch_id_t branch_allocate_id;
    active_id_t branch_active_tail_snapshot;
    logic [$clog2(PHYS_REGS)-1:0] branch_free_head_snapshot;
    logic [$clog2(PHYS_REGS)-1:0] branch_free_tail_snapshot;
    logic [$clog2(PHYS_REGS+1)-1:0] branch_free_count_snapshot;
    phys_reg_t branch_map_snapshot [32];
    branch_mask_t current_branch_mask;
    logic branch_restore_valid;
    active_id_t branch_restore_active_tail;
    logic [$clog2(PHYS_REGS)-1:0] branch_restore_free_head;
    logic [$clog2(PHYS_REGS)-1:0] branch_restore_free_tail;
    logic [$clog2(PHYS_REGS+1)-1:0] branch_restore_free_count;
    phys_reg_t branch_restore_map [32];
    branch_mask_t stack_reset_mask;
    branch_mask_t stack_abort_mask;
    logic branch_resolve_valid;
    branch_id_t branch_resolve_id;
    logic branch_resolve_mispredict;
    branch_mask_t reset_mask;
    // FB2b routed-WNS fix: abort_mask / abort_mask_q are the branch-recovery squash
    // broadcast -- they fan out to every speculative structure (IQ/LSQ/ROB/branch_
    // stack/FUs) across the die. The OOC route showed abort_mask_q -> DivUnit at 39 ns
    // route (90%), the routed worst path. max_fanout makes synth REPLICATE the driver
    // into local copies so Quick-place can put each near its consumers (short routes),
    // instead of one global net the router stretches across the chip. Synthesis-only
    // attribute -- Verilator ignores it, so the functional design is unchanged.
    (* max_fanout = 64 *) branch_mask_t abort_mask;
    (* max_fanout = 64 *) branch_mask_t abort_mask_q;
    branch_mask_t dispatch_branch_mask;

    logic mem_queue_full;
    logic mem_data_load_en;
    logic [MEMORY_ADDR_WIDTH-1:0] mem_data_addr;
    logic [XLEN-1:0] mem_data_store;
    logic [XLEN_BYTES-1:0] mem_data_store_mask;
    logic [2:0] mem_dmem_op;            // M3d Stage 2: LSQ typed op -> dmem_req_op
    logic [3:0] mem_dmem_amo;           // M4 #3: LSQ fine AMO op -> dmem_req_amo
`ifdef LSQ_MLP2
    logic [DMEM_ID_W-1:0] mem_dmem_id;  // Track A: LSQ-allocated dmem txn id -> dmem_req_id
    assign dmem_req_id = mem_dmem_id;
`endif
    logic lsq_store_second_beat;
    logic lsq_store_port_busy;
    logic lsq_sc_commit_done;           // M4-S5b: LSQ resolved the coherent SC -> release ROB retire
    logic [2:0] lsq_head_reason;        // P3-M0: LSQ head-blocking-reason (display-only perf)
    logic [2:0] lsq_fwd_class;          // P3 L2a: store->load forwarding opportunity (display-only)
    logic commit_store;
    active_id_t commit_store_id;

    logic [OOO_WIDTH-1:0] retire_valid;
    logic [OOO_WIDTH-1:0] commit_free_valid;
    phys_reg_t commit_free_prd [OOO_WIDTH];
    logic precise_halt;
    logic precise_exception;

    // ---- Precise-trap full flush for non-serializing faults (Phase 3c) ----
    // Memory access/page faults and instruction-fetch faults are NOT
    // serializing, so younger instructions are in flight behind them when the
    // fault reaches commit. Taking such a trap therefore squashes every
    // in-flight instruction (active list / issue queues / LSQ / branch stack /
    // multi-cycle FUs) in one cycle and rolls the rename map and free list back
    // to the committed architectural state. The multi-cycle units (mul/div/fp)
    // are flushed directly so no stale writeback lands on a reused active-list
    // id; single-cycle ALU writebacks and LSQ load completions are wiped by the
    // same-cycle active-list / LSQ flush, so no drain is required.
    //
    // arch_map_q is a committed (retirement) rename map (RRAT); arch_free_head_q
    // is the free-list head as of the committed point, so squashed speculative
    // allocations are reclaimed by rolling the free-list head back to it.
    logic        trap_take;         // exception committed this cycle -> full flush
    phys_reg_t   arch_map_q [32];
    phys_reg_t   arch_map_next [32];
    logic [$clog2(PHYS_REGS)-1:0] arch_free_head_q;
    logic [$clog2(PHYS_REGS)-1:0] arch_free_head_next;
    logic        map_restore_valid;
    phys_reg_t   map_restore_map [32];
    logic        free_restore_valid;
    logic [$clog2(PHYS_REGS)-1:0] free_restore_head;

    logic [OOO_WIDTH-1:0] arch_rd_we;
    // register_file declares its index ports as [$clog2(WIDTH)-1:0] (WIDTH=XLEN),
    // so on RV64 each selector is 6 bits, not 5. The index arrays must match that
    // per-way width or the packed-array connection misaligns lanes 1+ (writing
    // scrambled architectural register numbers).
    logic [OOO_WIDTH-1:0][$clog2(XLEN)-1:0] arch_rs1;
    logic [OOO_WIDTH-1:0][$clog2(XLEN)-1:0] arch_rs2;
    logic [OOO_WIDTH-1:0][$clog2(XLEN)-1:0] arch_rd;
    logic [OOO_WIDTH-1:0][XLEN-1:0] arch_rd_data;
    logic [OOO_WIDTH-1:0][XLEN-1:0] arch_rs1_data;
    logic [OOO_WIDTH-1:0][XLEN-1:0] arch_rs2_data;

    assign halted = halted_q;
    // A fetch request fires only if no same-cycle redirect/flush retargets the
    // PC (fetch_flush setters later in the commit logic override pc_next, so a
    // request issued for the stale pc_q would be a dead fetch -- suppress it).
    logic fetch_fire;
    assign fetch_fire = fetch_issue && !fetch_flush;
    assign ifetch_req_valid = fetch_fire;
    // The request address is the translated fetch block address (fetch_pa,
    // computed in the MMU section below; identity when paging is off). The
    // 16-byte fetch block is {fetch_pa[..:4]} as a word address (2 zeros for
    // RV32's 4-byte words, 1 for RV64's 8-byte words).
    assign ifetch_req_addr = {fetch_pa[XLEN-1:4], {(4-ADDR_SHIFT){1'b0}}};
    assign sequential_next_pc = {pc_q[XLEN-1:4], 4'b0} + XLEN'(16);
    assign dispatch_branch_mask = current_branch_mask & ~reset_mask & ~abort_mask;

    // Extract the 4 32-bit instructions of the 16-byte fetch block from the
    // word-granular memory read. RV32: one instruction per memory word. RV64:
    // two 32-bit instructions per 64-bit word, so the block is the low 2 words.
    logic [3:0][31:0] decode_fetch_instr;
    always_comb begin
        for (int j = 0; j < 4; j++)
`ifdef RV64
            decode_fetch_instr[j] = fgrp_data[j/2][ (j[0] ? 32 : 0) +: 32 ];
`else
            decode_fetch_instr[j] = fgrp_data[j];
`endif
    end

`ifdef DFE_S1
`ifdef RVC
    // DFE S1a realigner taps. RVC-only: DFE_S1 is OoO-only and the OoO frontend
    // requires RVC (a non-RVC OoO build does not elaborate -- pre-existing
    // is_compressed), and the predecode is RVC-parcel-based.
    logic [2:0]      dfe_s0;
    logic            dfe_completing;
    logic [XLEN-1:0] dfe_straddle_pc;
    logic [15:0]     dfe_straddle_half;
`endif
`endif

`ifdef RVC
    // RV64C: the two-wide expand-before-decode realign stage drains the
    // presented 16-byte group as 2-byte parcels and emits up to two canonical
    // 32-bit instructions/cycle with true 2-byte-granular PCs. Under -DRVC the
    // fgrp_fault_lane carried through the buffer is ABSOLUTE per-4B-word (see the
    // FetchPMP fork below), which the realigner maps to each instruction.
    logic [`RVC_NLANES-1:0]            rvc_lane_valid;
    logic [`RVC_NLANES-1:0][XLEN-1:0]  rvc_lane_pc;
    logic [`RVC_NLANES-1:0][31:0]      rvc_lane_instr;
    logic [`RVC_NLANES-1:0]            rvc_lane_is_comp;
    logic [`RVC_NLANES-1:0]            rvc_lane_fault;
    logic [`RVC_NLANES-1:0]            rvc_lane_fault_hi;
    logic [`RVC_NLANES-1:0][4:0]       rvc_lane_cause;
    logic [`RVC_NLANES-1:0][15:0]      rvc_lane_parcel;
    logic                  rvc_consume_block;
    logic                  rvc_block_drained;
    logic                  rvc_btb_terminate;
    logic                  rvc_tail_straddle;
    logic                  rvc_oldest_valid;
    logic [XLEN-1:0]       rvc_oldest_pc;
    // P2b BTB steer inputs to the realigner (tied 0 when BTB is disabled).
    logic                  realign_btb_hit;
    logic [2:0]            realign_btb_off;
`ifdef BTB
    assign realign_btb_hit = fgrp_btb_hit;
    assign realign_btb_off = fgrp_btb_off;
`else
    assign realign_btb_hit = 1'b0;
    assign realign_btb_off = 3'd0;
`endif

    rvc_realign RvcRealign (
        .clk,
        .rst_l,
        .fgrp_valid(fgrp_valid && !halted_q),
        .fgrp_pc,
        .fgrp_data,
        .fgrp_excpt,
        .fgrp_fault_word(fgrp_fault_lane & {OOO_WIDTH{fgrp_valid}}),
        .fgrp_fault_cause,
        .dispatch_count,
        .frontend_hold(dispatch_stall || halted_q),
        .fetch_flush,
        .btb_hit_in(realign_btb_hit),
        .btb_off_in(realign_btb_off),
        .out_valid(rvc_lane_valid),
        .out_pc(rvc_lane_pc),
        .out_instr(rvc_lane_instr),
        .out_is_compressed(rvc_lane_is_comp),
        .out_fetch_fault(rvc_lane_fault),
        .out_fetch_fault_hi(rvc_lane_fault_hi),
        .out_fault_cause(rvc_lane_cause),
        .out_rvc_parcel(rvc_lane_parcel),
        .rvc_consume_block,
        .rvc_block_drained,
        .rvc_btb_terminate,
        .rvc_tail_straddle,
        .frontend_oldest_valid(rvc_oldest_valid),
        .frontend_oldest_pc(rvc_oldest_pc)
`ifdef DFE_S1
        , .dfe_s0(dfe_s0)
        , .dfe_completing(dfe_completing)
        , .dfe_straddle_pc(dfe_straddle_pc)
        , .dfe_straddle_half(dfe_straddle_half)
`endif
    );
`endif

    ooo_fetch_decode FetchDecode (
        .rst_l,
        .fetch_valid(fgrp_valid && !halted_q),
        .instr_mem_excpt(fgrp_excpt),
        .fetch_fault_lane(fgrp_fault_lane & {OOO_WIDTH{fgrp_valid}}),
        .fetch_fault_cause(fgrp_fault_cause),
        .fetch_pc(fgrp_pc),
        .instr(decode_fetch_instr),
`ifdef RVC
        .rvc_valid(rvc_lane_valid),
        .rvc_pc(rvc_lane_pc),
        .rvc_instr(rvc_lane_instr),
        .rvc_is_compressed(rvc_lane_is_comp),
        .rvc_fetch_fault(rvc_lane_fault),
        .rvc_fetch_fault_hi(rvc_lane_fault_hi),
        .rvc_fault_cause(rvc_lane_cause),
        .rvc_parcel(rvc_lane_parcel),
`endif
        .decode_lanes
    );

    rename_map_table MapTable (
        .clk,
        .rst_l,
        .restore_valid(map_restore_valid),
        .restore_map(map_restore_map),
        .rename_valid(dispatch_valid),
        .rs1(map_rs1),
        .rs2(map_rs2),
        .rd(map_rd),
        .rename_has_dest(lane_has_dest),
        .alloc_prd(free_alloc_prd),
        .prs1(map_prs1),
        .prs2(map_prs2),
        .old_prd(map_old_prd),
        .has_dest(map_has_dest),
        .snapshot_map(map_snapshot)
    );

    free_list FreeList (
        .clk,
        .rst_l,
        .restore_valid(free_restore_valid),
        .restore_head(free_restore_head),
        .restore_tail(branch_restore_free_tail),
        .restore_count(branch_restore_free_count),
        .alloc_req(alloc_req),
        .free_valid(active_free_valid),
        .free_prd(active_free_prd),
        .alloc_valid(free_alloc_valid),
        .alloc_prd(free_alloc_prd),
        .can_allocate(free_can_allocate),
        .snapshot_head(free_head_snapshot),
        .snapshot_tail(free_tail_snapshot),
        .snapshot_count(free_count_snapshot)
    );

    busy_table BusyTable (
        .clk,
        .rst_l,
        .allocate_valid(free_alloc_valid),
        .allocate_prd(free_alloc_prd),
        .writeback_valid(wakeup_valid),
        .writeback_prd(wakeup_prd),
        .src1_prd(map_prs1),
        .src2_prd(map_prs2),
        .src1_ready(busy_src1_ready),
        .src2_ready(busy_src2_ready)
    );

    priv_csr_file #(.HART_ID(HART_ID)) CSRFile (
        .clk,
        .rst_l,
        .retire_cnt(retire_count),
        .mtime(clint_mtime),
        // ALU ports read CSRs in S2 (the execute stage), so the read address comes
        // from the registered issue entry, matching the ALU pipe's csr_rdata use.
        .read_addr(alu_issue_entry_q[0].instr[31:20]),
        .read_data(csr_read_data[0]),
        .read_illegal(csr_read_illegal[0]),
        .read_addr1(alu_issue_entry_q[1].instr[31:20]),
        .read_data1(csr_read_data[1]),
        .read_illegal1(csr_read_illegal[1]),
        .write_valid(csr_commit_write),
        .write_addr(csr_commit_addr),
        .write_data(csr_commit_wdata),
        .fp_fflags_valid(csr_fp_fflags_valid),
        .fp_fflags(csr_fp_fflags),
        .frm_value(csr_frm),
        .cache_ev_l1i_miss(hpm_l1i_miss),
        .cache_ev_l1d_miss(hpm_l1d_miss),
        .cache_ev_l1d_wb(hpm_l1d_wb),
        .irq_m_timer(irq_mtimer),
        .irq_m_software(irq_msoft),
        .irq_m_external(plic_m_ext),
        .irq_s_external(plic_s_ext),
        .trap_valid(commit_take_trap || commit_take_int),
        .trap_is_interrupt(tc_is_int),
        .trap_cause(tc_cause),
        .trap_epc(commit_take_int ? commit_int_epc : commit_trap_epc),
        .trap_tval(tc_is_int ? 32'b0 : commit_exc_tval),
        .trap_target_priv(tc_target),
        .ret_valid(commit_take_ret),
        .ret_from_s(commit_ret_from_s),
        .priv(cur_priv),
        .mstatus(csr_mstatus),
        .medeleg(csr_medeleg),
        .mideleg(csr_mideleg),
        .mie_csr(csr_mie),
        .mip_csr(csr_mip),
        .mtvec(csr_mtvec),
        .stvec(csr_stvec),
        .mepc(csr_mepc),
        .sepc(csr_sepc),
        .satp(csr_satp),
        .pmpcfg_o(csr_pmpcfg_arr),
        .pmpaddr_o(csr_pmpaddr_arr),
        .menvcfg_adue(csr_menvcfg_adue)
    );

    // Trap aggregation/delegation for the instruction being committed.
    trap_controller TrapCtrl (
        .priv(cur_priv),
        .mstatus(csr_mstatus),
        .mie_csr(csr_mie),
        .mip_csr(csr_mip),
        .medeleg(csr_medeleg),
        .mideleg(csr_mideleg),
        .mtvec(csr_mtvec),
        .stvec(csr_stvec),
        .exc_valid(commit_exc_valid),
        .exc_cause(commit_exc_cause),
        .trap_valid(tc_trap_valid),
        .trap_is_interrupt(tc_is_int),
        .trap_cause(tc_cause),
        .trap_target_priv(tc_target),
        .trap_vector(tc_vector)
    );

    // A store write beat is accepted by the memory subsystem this cycle. The
    // devices snoop ACCEPTED stores only: a write beat held while the port is
    // not ready stays presented for several cycles and must produce exactly
    // one device side effect.
    logic dmem_store_fire;
    assign dmem_store_fire = dmem_req_valid && dmem_req_write && dmem_req_ready;

    // Cacheable/device split (L1D=1): a dmem request to the CLINT/PLIC/UART
    // device hole bypasses the L1D. Byte PA = word addr << ADDR_SHIFT. Mirrors
    // NIIGO_Mem::is_device_pa; kept inline so the core needn't import the memsys
    // package. Harmless when the memsys ignores it (L1=0/C1).
    logic [XLEN-1:0] dmem_req_bpa;
    assign dmem_req_bpa = {dmem_req_addr, {ADDR_SHIFT{1'b0}}};
    assign dmem_req_device =
        ((dmem_req_bpa >= XLEN'('h0200_0000)) && (dmem_req_bpa < XLEN'('h0201_0000))) ||
        ((dmem_req_bpa >= XLEN'('h0C00_0000)) && (dmem_req_bpa < XLEN'('h1000_0000))) ||
        ((dmem_req_bpa >= XLEN'('h0D00_0000)) && (dmem_req_bpa < XLEN'('h0D00_1000)));

    // fence.i L1D-writeback hold (L1D=1 only). On a fence.i retire the core
    // defers the pc+4 redirect, raises dcache_flush_req, and holds the frontend
    // until the memsys has written back every dirty L1D line (dcache_flush_done);
    // only then does it invalidate the L1I and refetch -- so the modified code is
    // in memory before the I-side refills it. On C1/L1=0 the fence.i path keeps
    // its immediate redirect and this is inert.
`ifdef NIIGO_DSIDE_WB
    logic        fencei_pending_q, fencei_pending_next;
    logic [XLEN-1:0] fencei_pc_q, fencei_pc_next;
    assign dcache_flush_req = fencei_pending_q;
`else
    assign dcache_flush_req = 1'b0;
`endif
    logic fencei_block;
`ifdef NIIGO_DSIDE_WB
    assign fencei_block = fencei_pending_q;
`else
    assign fencei_block = 1'b0;
`endif
    logic unused_flush_done;
    assign unused_flush_done = dcache_flush_done;

    // ============================ Device bus ============================
    // By default each core owns private CLINT/PLIC/UART instances. With
    // NIIGO_EXT_DEVICES (M4 SMP) they are lifted to ONE shared instance at the
    // SMP top: this core exports its snoop/load-query ports and imports the
    // shared device result/mtime/IRQs. The external result rides the clint_*
    // carriers so the data_load mux and the priv_csr_file wiring are unchanged.
`ifdef NIIGO_EXT_DEVICES
    assign dsnoop_store_en    = dmem_store_fire;
    assign dsnoop_store_waddr = dmem_req_addr;
    assign dsnoop_store_wdata = dmem_req_wdata;
    assign dsnoop_store_mask  = dmem_req_wmask;
    assign dsnoop_load_addr   = dmem_resp_addr;
    assign dsnoop_load_en     = dmem_resp_valid;
    assign dsnoop_load_off    = dev_load_off;
    // Shared-device load result rides the clint_* carrier (plic/uart unused here).
    assign clint_load_hit  = ext_load_hit;
    assign clint_load_data = ext_load_data;
    assign plic_load_hit   = 1'b0;  assign plic_load_data = '0;
    assign uart_load_hit   = 1'b0;  assign uart_load_data = '0;
    assign uart_irq        = 1'b0;
    assign clint_mtime     = ext_mtime;
    assign irq_mtimer      = ext_irq_m_timer;
    assign irq_msoft       = ext_irq_m_software;
    assign plic_m_ext      = ext_irq_m_external;
    assign plic_s_ext      = ext_irq_s_external;
`else
    // Minimal CLINT: snoops committed stores for mtimecmp / msip.
    clint Clint (
        .clk,
        .rst_l,
        .store_en(dmem_store_fire),
        .store_waddr(dmem_req_addr),
        .store_wdata(dmem_req_wdata),
        .store_mask(dmem_req_wmask),
        // Look up against the returned-load address so a CLINT hit lines up with
        // the load result the LSQ consumes (loads complete with latency).
        .load_addr(dmem_resp_addr),
        .load_hit(clint_load_hit),
        .load_data(clint_load_data),
        .irq_m_timer(irq_mtimer),
        .irq_m_software(irq_msoft),
        .mtime_out(clint_mtime)
    );

    // PLIC: external-interrupt controller (ctx0 = M-external, ctx1 = S-external).
    // No device sources are wired yet (src_irq = 0); software injects pending via
    // a write to the pending word. Drives mip.MEIP / mip.SEIP.
    // UART interrupt drives PLIC source 10 (the conventional NS16550 line); all
    // other device sources are still software-injected via the pending word.
    logic [31:0] plic_src;
    always_comb begin
        plic_src = 32'b0;
        plic_src[10] = uart_irq;
    end
    plic Plic (
        .clk,
        .rst_l,
        .src_irq(plic_src),
        .store_en(dmem_store_fire),
        .store_waddr(dmem_req_addr),
        .store_wdata(dmem_req_wdata),
        .store_mask(dmem_req_wmask),
        .load_addr(dmem_resp_addr),
        .load_en(dmem_resp_valid),
        .load_off(dev_load_off),
        .load_hit(plic_load_hit),
        .load_data(plic_load_data),
        .irq_m_external(plic_m_ext),
        .irq_s_external(plic_s_ext)
    );

    // NS16550-subset UART -> simulation console (base 0x0D00_0000, in the device
    // hole; 0x1000_0000 is arch-test RAM). Snoops the data store port like the
    // CLINT/PLIC; its loads mux into the LSQ writeback.
    uart Uart (
        .clk,
        .rst_l,
        .store_en(dmem_store_fire),
        .store_waddr(dmem_req_addr),
        .store_wdata(dmem_req_wdata),
        .store_mask(dmem_req_wmask),
        .load_addr(dmem_resp_addr),
        .load_en(dmem_resp_valid),
        .load_off(dev_load_off),
        .load_hit(uart_load_hit),
        .load_data(uart_load_data),
        .irq(uart_irq)
`ifdef FPGA_BUILD
        ,
        .vuart_tx_valid, .vuart_tx_byte,
        .vuart_rx_valid, .vuart_rx_byte, .vuart_rx_pop
`endif
    );
`endif

    // ===================== Sv32 MMU (Phase 4) =====================
    // satp / mstatus-derived translation context. Identical to the scalar core.
    logic        satp_mode;
    logic [RISCV_Priv::VM_PPN_W-1:0]  satp_ppn;
    logic [RISCV_Priv::VM_ASID_W-1:0] satp_asid;
    logic        mstatus_mprv, mstatus_sum, mstatus_mxr;
    RISCV_Priv::priv_mode_t mpp_mode, priv_data;
    logic        paging_fetch, paging_data;

`ifdef RV64
    // Sv39 satp layout: MODE[63:60] (8 = Sv39), ASID[59:44], PPN[43:0].
    assign satp_mode    = (csr_satp[63:60] == 4'd8);
    assign satp_ppn     = csr_satp[43:0];
    assign satp_asid    = csr_satp[59:44];
`else
    assign satp_mode    = csr_satp[31];
    assign satp_ppn     = csr_satp[21:0];
    assign satp_asid    = csr_satp[30:22];
`endif
    assign mstatus_mprv = csr_mstatus[RISCV_Priv::MSTATUS_MPRV_BIT];
    assign mstatus_sum  = csr_mstatus[RISCV_Priv::MSTATUS_SUM_BIT];
    assign mstatus_mxr  = csr_mstatus[RISCV_Priv::MSTATUS_MXR_BIT];
    assign mpp_mode     = RISCV_Priv::priv_mode_t'(csr_mstatus[RISCV_Priv::MSTATUS_MPP_LO+:2]);
    assign priv_data    = mstatus_mprv ? mpp_mode : cur_priv;
    assign paging_fetch = satp_mode && (cur_priv  != RISCV_Priv::PRIV_M);
    assign paging_data  = satp_mode && (priv_data != RISCV_Priv::PRIV_M);

    // TLB flush: SFENCE.VMA or any satp write (driven in the commit block).
    logic tlb_flush;

    // Compute an XLEN-capped physical byte address from a leaf translation
    // found at the given level (level > 0 substitutes the VA's low VPN slices
    // into the superpage's PPN).
    function automatic logic [XLEN-1:0] make_pa(
            input logic [RISCV_Priv::VM_PPN_W-1:0] ppn,
            input logic [1:0] level, input logic [XLEN-1:0] va);
`ifdef RV64
        unique case (level)
            2'd2:    make_pa = {8'b0, ppn[43:18], va[29:0]};  // 1 GiB
            2'd1:    make_pa = {8'b0, ppn[43:9],  va[20:0]};  // 2 MiB
            default: make_pa = {8'b0, ppn,        va[11:0]};  // 4 KiB
        endcase
`else
        if (level != 2'd0) make_pa = {ppn[19:10], va[21:0]};  // 4 MiB
        else               make_pa = {ppn[19:0],  va[11:0]};  // 4 KiB
`endif
    endfunction

    // Leaf-PTE permission fault (excludes A/D, which trigger a re-walk).
    function automatic logic perm_bad(input logic [7:0] perm,
            input logic [1:0] acc, input RISCV_Priv::priv_mode_t pr,
            input logic sum, input logic mxr);
        logic fail;
        fail = 1'b0;
        unique case (acc)
            2'd0: if (!perm[RISCV_Priv::PTE_X]) fail = 1'b1;
            2'd1: if (!(perm[RISCV_Priv::PTE_R] ||
                       (perm[RISCV_Priv::PTE_X] && mxr))) fail = 1'b1;
            2'd2: if (!perm[RISCV_Priv::PTE_W]) fail = 1'b1;
            default: ;
        endcase
        if (pr == RISCV_Priv::PRIV_U) begin
            if (!perm[RISCV_Priv::PTE_U]) fail = 1'b1;
        end else if (pr == RISCV_Priv::PRIV_S) begin
            if (perm[RISCV_Priv::PTE_U]) begin
                if (acc == 2'd0) fail = 1'b1;
                else if (!sum)   fail = 1'b1;
            end
        end
        perm_bad = fail;
    endfunction

    // --- Data-side translation request exposed by the load/store queue ---
    logic        mem_req_valid;
    logic [XLEN-1:0] mem_req_vaddr;
    logic        mem_req_store;
    logic [1:0]  data_acc;
    assign data_acc = mem_req_store ? 2'd2 : 2'd1;

    logic        data_noncanon, fetch_noncanon;
`ifdef RV64
    // Sv39 canonical check: VA bits [63:39] must all equal bit 38. A TLB
    // lookup truncates to VPN bits, so a non-canonical VA could falsely hit a
    // canonical entry -- gate the hit and walk the (faulting) access instead.
    assign data_noncanon  = paging_data &&
        (mem_req_vaddr[XLEN-1:39] != {(XLEN-39){mem_req_vaddr[38]}});
    assign fetch_noncanon = paging_fetch &&
        (pc_q[XLEN-1:39] != {(XLEN-39){pc_q[38]}});
`else
    assign data_noncanon  = 1'b0;
    assign fetch_noncanon = 1'b0;
`endif

    // --- DTLB ---
    logic        dtlb_hit;
    logic [1:0]  dtlb_level;
    logic [RISCV_Priv::VM_PPN_W-1:0] dtlb_ppn;
    logic [7:0]  dtlb_perm;
    logic        d_need_ad, dtlb_usable;
    assign d_need_ad   = !dtlb_perm[RISCV_Priv::PTE_A] ||
                         ((data_acc == 2'd2) && !dtlb_perm[RISCV_Priv::PTE_D]);
    // A non-canonical Sv39 VA must not use a (VPN-truncated) TLB hit; it goes
    // to the walker, which faults it.
    assign dtlb_usable = dtlb_hit && !d_need_ad && !data_noncanon;

    // --- Page-table walker (shared; data has priority over fetch) ---
    logic        ptw_req, ptw_done, ptw_fault, ptw_busy;
    logic [1:0]  ptw_level;
    logic        ptw_fault_access;       // PTW fault is a PMP-on-PTE access fault
    logic        ptw_pte_pmp_fault;      // PMP denies the in-flight PTE access
    logic        ptw_mem_is_write;       // in-flight PTE access is an A/D write
    logic [RISCV_Priv::VM_PPN_W-1:0] ptw_ppn;
    logic [7:0]  ptw_perm;
    logic [RISCV_Priv::VM_VPN_W-1:0] ptw_vpn;
    logic [RISCV_Priv::VM_VPN_W-1:0] ptw_walk_vpn; // VPN the walk was launched for
    logic        ptw_walk_is_data; // that walk is a data (vs fetch) access
    RISCV_Priv::priv_mode_t ptw_walk_priv;          // privilege the walk launched under
    logic [RISCV_Priv::VM_PPN_W-1:0] ptw_walk_satp; // satp.PPN the walk launched under
    logic [1:0]  ptw_access;
    RISCV_Priv::priv_mode_t ptw_priv;
    logic [XLEN-1:0] ptw_mem_addr;

    logic        itlb_hit;
    logic [1:0]  itlb_level;
    logic [RISCV_Priv::VM_PPN_W-1:0] itlb_ppn;
    logic [7:0]  itlb_perm;
    logic        data_need_walk, fetch_need_walk;
    logic        ptw_for_data;
    logic        itlb_usable;
    assign itlb_usable = itlb_hit && !fetch_noncanon;
    assign data_need_walk  = paging_data && mem_req_valid && !dtlb_usable;
    assign fetch_need_walk = paging_fetch && !itlb_usable;
    // Data accesses have priority over instruction fetch for the shared walker.
    assign ptw_for_data = data_need_walk;
    assign ptw_req    = data_need_walk || fetch_need_walk;
    assign ptw_vpn    = ptw_for_data ?
        mem_req_vaddr[RISCV_Priv::VM_VPN_W+11:12] :
        pc_q[RISCV_Priv::VM_VPN_W+11:12];
    assign ptw_access = ptw_for_data ? data_acc : 2'd0;
    assign ptw_priv   = ptw_for_data ? priv_data : cur_priv;

    ptw PTW (
        .clk, .rst_l,
        .req_valid(ptw_req),
        .req_vpn(ptw_vpn),
        .satp_ppn(satp_ppn),
        .req_access(ptw_access),
        .req_priv(ptw_priv),
        .mstatus_sum(mstatus_sum),
        .mstatus_mxr(mstatus_mxr),
        .adue(csr_menvcfg_adue),
        .req_noncanonical(ptw_for_data ? data_noncanon : fetch_noncanon),
        .mem_req(ptw_mem_req),
        .mem_we(ptw_mem_we),
        .mem_is_write(ptw_mem_is_write),
        .mem_addr(ptw_mem_addr),
        .mem_wdata(ptw_mem_wdata),
        .mem_ack(ptw_mem_ack),
        .mem_rdata(ptw_mem_rdata),
        .pte_pmp_fault(ptw_pte_pmp_fault),
        .busy(ptw_busy),
        .done(ptw_done),
        .fault(ptw_fault),
        .fault_access(ptw_fault_access),
        .ppn(ptw_ppn),
        .perm(ptw_perm),
        .leaf_level(ptw_level),
        .walk_vpn(ptw_walk_vpn),
        .walk_is_data(ptw_walk_is_data),
        .walk_priv(ptw_walk_priv),
        .walk_satp(ptw_walk_satp)
    );
    // A completed walk's result (resolved PA or fault) may be consumed only by an
    // access whose full translation request -- (VPN, satp.PPN, privilege) --
    // matches the one the walk was launched for. Fetches/loads are speculative, so
    // a walk can outlive its request: a mispredicted fetch of an unmapped VA
    // launches a walk that faults at the page-table root, and without the VPN
    // check the architectural fetch of a *different, valid* VPN in the same
    // address space adopts that root fault -> spurious page fault (this crashed
    // /sh at 0xd8c, the insn after the sret from its first syscall). walk_priv /
    // walk_satp additionally reject a walk launched in another mode / address
    // space (e.g. a speculative S-mode fetch of a user VA in a trap window,
    // walking the kernel page table). The TLB *fills* already key on walk_vpn;
    // this extends the same precision to the immediate PA/fault consumption.
    logic ptw_ctx_fetch, ptw_ctx_data;
    assign ptw_ctx_fetch =
        (ptw_walk_vpn  == pc_q[RISCV_Priv::VM_VPN_W+11:12]) &&
        (ptw_walk_priv == cur_priv) && (ptw_walk_satp == satp_ppn);
    assign ptw_ctx_data  =
        (ptw_walk_vpn  == mem_req_vaddr[RISCV_Priv::VM_VPN_W+11:12]) &&
        (ptw_walk_priv == priv_data) && (ptw_walk_satp == satp_ppn);
    assign ptw_mem_addr_w = ptw_mem_addr[XLEN-1:ADDR_SHIFT];

    // PMP on the implicit PTE access. Per the priv spec these accesses are checked
    // as Supervisor (reads need R, A/D writes need W); a violation aborts the walk
    // and surfaces as an access fault of the original access type (handled below).
    pmp_checker PtwPMP (
        .paddr(ptw_mem_addr),
        .access(ptw_mem_is_write ? 2'd2 : 2'd1),
        .priv(RISCV_Priv::PRIV_S),
        .pmpcfg(csr_pmpcfg_arr),
        .pmpaddr(csr_pmpaddr_arr),
        .fault(ptw_pte_pmp_fault)
    );

    mmu_tlb #(.ENTRIES(16)) ITLB (
        .clk, .rst_l,
        .lookup_en(paging_fetch),
        .lookup_vpn(pc_q[RISCV_Priv::VM_VPN_W+11:12]),
        .lookup_asid(satp_asid),
        .hit(itlb_hit), .hit_ppn(itlb_ppn), .hit_perm(itlb_perm),
        .hit_level(itlb_level),
        // Fill against the VPN the PTW actually walked, gated by the walk's
        // latched class -- not the live fetch/data head, which may have moved.
        .fill_en(ptw_done && !ptw_fault && !ptw_walk_is_data),
        .fill_vpn(ptw_walk_vpn),
        .fill_asid(satp_asid),
        .fill_ppn(ptw_ppn), .fill_perm(ptw_perm), .fill_level(ptw_level),
        .flush_en(tlb_flush)
    );

    mmu_tlb #(.ENTRIES(16)) DTLB (
        .clk, .rst_l,
        .lookup_en(paging_data && mem_req_valid),
        .lookup_vpn(mem_req_vaddr[RISCV_Priv::VM_VPN_W+11:12]),
        .lookup_asid(satp_asid),
        .hit(dtlb_hit), .hit_ppn(dtlb_ppn), .hit_perm(dtlb_perm),
        .hit_level(dtlb_level),
        .fill_en(ptw_done && !ptw_fault && ptw_walk_is_data),
        .fill_vpn(ptw_walk_vpn),
        .fill_asid(satp_asid),
        .fill_ppn(ptw_ppn), .fill_perm(ptw_perm), .fill_level(ptw_level),
        .flush_en(tlb_flush)
    );

    // --- Resolve the data physical address + fault/stall for the LSQ ---
    logic        data_from_ptw;
    logic [XLEN-1:0] data_pa;
    logic        data_perm_fault;
    logic        lsq_xlate_stall, lsq_xlate_fault;
    logic [4:0]  lsq_xlate_cause;
    // Consume the completed walk's result by its *latched* class, exactly as
    // the TLB fills do (ptw_walk_is_data, not the combinational ptw_for_data).
    // ptw_for_data tracks the *current* head's want; if a fetch walk completes
    // on a cycle when a data access has *newly* started wanting a walk,
    // ptw_for_data=1 but the result belongs to the fetch walk. Gating on the
    // latched class keeps the data load from grabbing a fetch walk's PPN (which
    // mistranslated VA 0x800096f0 -> PA 0x800076f0 and corrupted a syscall ptr).
    assign data_from_ptw = ptw_done && !ptw_fault && ptw_walk_is_data && ptw_ctx_data;
    always_comb begin
        if (!paging_data)        data_pa = mem_req_vaddr;
        else if (dtlb_usable)    data_pa = make_pa(dtlb_ppn, dtlb_level, mem_req_vaddr);
        else if (data_from_ptw)  data_pa = make_pa(ptw_ppn, ptw_level, mem_req_vaddr);
        else                     data_pa = mem_req_vaddr;
    end
    assign data_perm_fault =
        (dtlb_usable && perm_bad(dtlb_perm, data_acc, priv_data,
            mstatus_sum, mstatus_mxr)) ||
        (data_need_walk && ptw_walk_is_data && ptw_done && ptw_fault && ptw_ctx_data);
    // PMP on the resolved data physical address (checked in any mode, on the
    // post-translation PA). Only meaningful once the PA is resolved -- during a
    // walk data_pa is the (stale) VA fallback, so gate the PMP fault on a
    // resolved PA. A translation page fault takes priority over a PMP fault.
    logic        data_pa_resolved, pmp_data_fault;
    assign data_pa_resolved = !paging_data || dtlb_usable || data_from_ptw;
    pmp_checker DataPMP (
        .paddr(data_pa),
        .access(data_acc),
        .priv(priv_data),
        .pmpcfg(csr_pmpcfg_arr),
        .pmpaddr(csr_pmpaddr_arr),
        .fault(pmp_data_fault)
    );
    assign lsq_xlate_fault = mem_req_valid &&
        ((paging_data && data_perm_fault) ||
         (data_pa_resolved && pmp_data_fault));
    assign lsq_xlate_stall = paging_data && mem_req_valid &&
        !dtlb_usable && !data_from_ptw && !data_perm_fault;
    // A PMP fault on a PTE access during a data walk is an access fault, not a
    // page fault (priv spec); it overrides the page-fault cause below.
    logic data_ptw_access_fault;
    assign data_ptw_access_fault =
        data_need_walk && ptw_walk_is_data && ptw_done && ptw_fault && ptw_fault_access &&
        ptw_ctx_data;
    // Page fault (translation) reported ahead of a PMP access fault.
    assign lsq_xlate_cause =
        (paging_data && data_perm_fault && !data_ptw_access_fault) ?
            (mem_req_store ? RISCV_Priv::EXC_STORE_PAGE_FAULT
                           : RISCV_Priv::EXC_LOAD_PAGE_FAULT) :
            (mem_req_store ? RISCV_Priv::EXC_STORE_ACCESS
                           : RISCV_Priv::EXC_LOAD_ACCESS);
    // A returning load no longer needs address matching against the head: the
    // LSQ allows exactly one load outstanding at the data port and tracks it
    // (mem_inflight / mem_inflight_kill in load_store_queue.sv), so a response
    // belongs to that single request by construction, for any memory latency.
    // This replaces the old DMEMORY_READ_DELAY-deep VA match pipe, which baked
    // the fixed magic-memory latency into the core.

    // --- Resolve the fetch physical address + stall during a fetch walk ---
    logic        fetch_from_ptw, ptw_fetch_done;
    logic        fetch_xlate_stall;
    // Latched-class gating (mirrors the ITLB fill), not combinational ptw_for_data:
    // a fetch walk's completion must be consumed by the fetch path even on a cycle
    // when a data access is concurrently requesting a walk.
    assign fetch_from_ptw = ptw_done && !ptw_fault && !ptw_walk_is_data && ptw_ctx_fetch;
    assign ptw_fetch_done = ptw_done && !ptw_walk_is_data && ptw_ctx_fetch;
    always_comb begin
        if (!paging_fetch)       fetch_pa = pc_q;
        else if (itlb_usable)    fetch_pa = make_pa(itlb_ppn, itlb_level, pc_q);
        else if (fetch_from_ptw) fetch_pa = make_pa(ptw_ppn, ptw_level, pc_q);
        else                     fetch_pa = pc_q;
    end
    // Freeze the frontend while the walker resolves the fetch translation.
    assign fetch_xlate_stall = fetch_need_walk && !ptw_fetch_done;

    // Instruction-fetch page fault: an ITLB hit lacking execute permission for
    // the current privilege, or a fetch page-table walk that faulted. Reported
    // precisely at commit on the faulting PC (see ooo_fetch_decode / alu_pipe);
    // mtval = faulting VA.
    logic fetch_perm_fault;
    assign fetch_perm_fault =
        (itlb_usable && perm_bad(itlb_perm, 2'd0, cur_priv,
            mstatus_sum, mstatus_mxr)) ||
        (fetch_need_walk && !ptw_walk_is_data && ptw_done && ptw_fault && ptw_ctx_fetch);
    // A PMP fault on a PTE access during a fetch walk is an instruction *access*
    // fault, not a page fault (priv spec) -- selects the cause below.
    logic fetch_ptw_access_fault;
    assign fetch_ptw_access_fault =
        fetch_need_walk && !ptw_walk_is_data && ptw_done && ptw_fault && ptw_fault_access &&
        ptw_ctx_fetch;

    // PMP on the resolved fetch PA (any mode). Gated on a resolved PA so the
    // (stale VA) fallback during a walk is not checked; page fault wins over PMP.
    // A 16-byte fetch block can span multiple PMP regions, so each of the up-to-4
    // fetched words is PMP-checked independently (translation/exec-permission are
    // page-granular and identical across the block). fetch_fault_lane[i] is the
    // fault for decode lane i (VA pc_q + 4*i; same page, so PA = fetch_pa + 4*i).
    logic fetch_pa_resolved;
    logic [OOO_WIDTH-1:0] pmp_fetch_fault_lane;
    logic [OOO_WIDTH-1:0] fetch_lane_in_block;
    assign fetch_pa_resolved = !paging_fetch || itlb_usable || fetch_from_ptw;
    genvar fpl;
    generate
        for (fpl = 0; fpl < OOO_WIDTH; fpl += 1) begin : fetch_pmp_gen
            pmp_checker FetchPMP (
`ifdef RVC
                // RV64C: check the ABSOLUTE 4-byte word fpl of the 16-byte block
                // (block-aligned base), so fault_lane[w] is word w regardless of
                // the entry offset -- the realigner maps parcel s -> word s>>1.
                .paddr({fetch_pa[XLEN-1:4], 4'b0} + (fpl * 32'd4)),
`else
                .paddr(fetch_pa + (fpl * 32'd4)),
`endif
                .access(2'd0),
                .priv(cur_priv),
                .pmpcfg(csr_pmpcfg_arr),
                .pmpaddr(csr_pmpaddr_arr),
                .fault(pmp_fetch_fault_lane[fpl])
            );
            // Lane fpl is part of this block only if it lies within the 16-byte
            // fetch window starting at pc_q's offset; otherwise it is a different
            // block and must not contribute a fault.
            assign fetch_lane_in_block[fpl] =
                (int'(pc_q[3:2]) + fpl) < OOO_WIDTH;
        end
    endgenerate

    // Page/exec-permission faults are page-granular -> collapse onto lane 0;
    // PMP faults are per word. fetch_fault retains its group-level meaning (any
    // lane faults) for the existing frontend-stall/redirect logic.
    always_comb begin
`ifdef RVC
        // Absolute per-4B-word fault for the whole 16-byte block. A page/exec
        // fault covers the entire (single-page) block -> all words fault, so
        // whichever parcel the entry instruction sits at is caught; PMP faults
        // are already per absolute word (no entry-offset mask under RVC).
        if (paging_fetch && fetch_perm_fault)
            fetch_fault_lane = {OOO_WIDTH{1'b1}};
        else if (fetch_pa_resolved)
            fetch_fault_lane = pmp_fetch_fault_lane;
        else
            fetch_fault_lane = '0;
`else
        if (paging_fetch && fetch_perm_fault)
            fetch_fault_lane = {{(OOO_WIDTH-1){1'b0}}, 1'b1};
        else if (fetch_pa_resolved)
            fetch_fault_lane = pmp_fetch_fault_lane & fetch_lane_in_block;
        else
            fetch_fault_lane = '0;
`endif
    end
    assign fetch_fault = |fetch_fault_lane;
    assign fetch_fault_cause =
        (paging_fetch && fetch_perm_fault && !fetch_ptw_access_fault) ?
        RISCV_Priv::EXC_INSTR_PAGE_FAULT : RISCV_Priv::EXC_INSTR_ACCESS;

    branch_stack BranchStack (
        .clk,
        .rst_l,
        .allocate(branch_allocate),
        .active_tail_snapshot(branch_active_tail_snapshot),
        .free_head_snapshot(branch_free_head_snapshot),
        .free_tail_snapshot(branch_free_tail_snapshot),
        .free_count_snapshot(branch_free_count_snapshot),
        .map_snapshot(branch_map_snapshot),
        .resolve(branch_resolve_valid),
        .resolve_id(branch_resolve_id),
        .mispredict(branch_resolve_mispredict),
        .flush(trap_take),
        .full(branch_stack_full),
        .allocate_valid(branch_allocate_valid),
        .allocate_id(branch_allocate_id),
        .current_mask(current_branch_mask),
        .restore_valid(branch_restore_valid),
        .restore_active_tail(branch_restore_active_tail),
        .restore_free_head(branch_restore_free_head),
        .restore_free_tail(branch_restore_free_tail),
        .restore_free_count(branch_restore_free_count),
        .restore_map(branch_restore_map),
        .reset_mask(stack_reset_mask),
        .abort_mask(stack_abort_mask)
    );

    tage_sc_l_predictor #(.HISTORY_BITS(DIRECT_HISTORY_BITS)) DirectBranchPredictor (
        .clk,
        .rst_l,
        .lookup_valid(direct_lookup_valid),
        .lookup_pc(direct_lookup_pc),
        .history(ghr_q),
        .prediction(direct_prediction),
        .prediction_info(direct_prediction_info),
        .update_valid(branch_writeback.valid && branch_writeback.branch_valid &&
`ifdef FUSE_BRANCH
            // A fused master resolves the folded slave branch: train with the
            // SLAVE's identity (the master's .instr is not a branch encoding).
            ((branch_writeback.instr[6:0] == RISCV_ISA::OP_BRANCH) ||
             (branch_writeback.fuse_is_branch &&
              branch_writeback.fuse_branch_instr[6:0] == RISCV_ISA::OP_BRANCH))),
`else
            (branch_writeback.instr[6:0] == RISCV_ISA::OP_BRANCH)),
`endif
`ifdef FUSE_BRANCH
        .update_pc(branch_writeback.fuse_is_branch ?
            branch_writeback.fuse_branch_pc : branch_writeback.pc),
        .update_taken(branch_writeback.redirect_pc !=
            ((branch_writeback.fuse_is_branch ?
                branch_writeback.fuse_branch_pc : branch_writeback.pc)
             + `ILEN_INC(branch_writeback.fuse_is_branch ?
                 branch_writeback.fuse_is_compressed :
                 branch_writeback.is_compressed))),
`else
        .update_pc(branch_writeback.pc),
        .update_taken(branch_writeback.redirect_pc !=
            (branch_writeback.pc + `ILEN_INC(branch_writeback.is_compressed))),
`endif
        .update_info(branch_writeback.predictor_info)
    );

    ittage_predictor IndirectBranchPredictor (
        .clk,
        .rst_l,
        .lookup_valid(indirect_lookup_valid),
        .lookup_pc(indirect_lookup_pc),
        .prediction_valid(indirect_prediction_valid),
        .prediction_target(indirect_prediction_target),
        .prediction_info(indirect_prediction_info),
        .update_valid(branch_writeback.valid && branch_writeback.branch_valid &&
            (branch_writeback.instr[6:0] == RISCV_ISA::OP_JALR) &&
            !((branch_writeback.instr[19:15] == 5'd1) &&
              (branch_writeback.instr[11:7] == 5'd0))),
        .update_target(branch_writeback.redirect_pc),
        .update_info(branch_writeback.predictor_info)
    );

`ifdef FUSE_BRANCH
    // Fused-pair atomicity (shared-infra §4b): a fuse-master whose folded slave
    // is a branch (CMPBR/LDBR kinds) must stall WITH the branch-stack-full slave
    // so the pair dispatches together or not at all. Decode-derived (fuse_master/
    // fuse_kind_lane read decode_lanes only) — no dispatch_valid feedback loop.
    for (genvar fuse_gi = 0; fuse_gi < OOO_WIDTH; fuse_gi += 1) begin : gen_fuse_pre_branch
        assign lane_fuse_pre_branch[fuse_gi] = fuse_master[fuse_gi] &&
            ((fuse_kind_lane[fuse_gi] == FUSE_K_CMPBR) ||
             (fuse_kind_lane[fuse_gi] == FUSE_K_LDBR));
    end
`endif

    ooo_dispatch_control DispatchControl (
        .lane_valid,
        .lane_has_dest,
        .lane_is_branch,
`ifdef JAL_NO_CKPT
        .lane_needs_ckpt,
`endif
`ifdef FUSE_BRANCH
        .lane_fuse_pre_branch,
`endif
        .lane_is_memory,
        .lane_is_terminal,
        .lane_is_serializing,
        .lane_is_fp,
        .lane_fp_src_busy,
        .active_list_full(active_full),
        .int_iq_full(int_iq_full),
        .mem_queue_full(mem_queue_full),
        .branch_stack_full(branch_stack_full),
        .free_list_can_allocate(free_can_allocate),
        .free_list_available(free_count_snapshot),
        .suppress_dispatch(redirect_valid || terminal_pending_q ||
            control_pending_q || serial_pending_q || halted_q || irq_drain_q ||
            wfi_wait_q || commit_take_trap || fencei_block || predict_stall ||
            fflags_drain_stall),
        .dispatch_valid,
`ifdef DISPATCH_STATS
        .dstat_cut_valid,
        .dstat_cut_reason,
        .dstat_cut_idx,
        .dstat_stall_reason,
`endif
        .dispatch_stall
    );

    active_list ActiveList (
`ifdef CSWHY
        .cswhy_head_id, .cswhy_head_present, .cswhy_head_class,
        .cswhy_head_done, .cswhy_head_pending, .cswhy_head_xclass_ok,
        .cswhy_head_count,
`endif
        .clk,
        .rst_l,
        .restore_valid(branch_restore_valid),
        .restore_tail(branch_restore_active_tail),
        .flush(trap_take),
        .allocate_valid(dispatch_valid),
        .allocate_packet(rename_packets),
        .writeback_valid(writeback_valid),
        .writeback_id(writeback_active_id),
        .writeback_data(writeback_data),
        .writeback_exception(writeback_exception),
        .writeback_exc_cause(writeback_exc_cause),
        .writeback_halted(writeback_halted),
        .writeback_fp_write,
        .writeback_fp_rd,
        .writeback_fp_data,
        .writeback_csr_write,
        .writeback_csr_addr,
        .writeback_csr_wdata,
        .writeback_fp_fflags_valid,
        .writeback_fp_fflags,
        .reset_mask,
        .abort_mask,
        .commit_taken(retire_valid),
        .full(active_full),
        .empty(active_empty),
        .tail(active_tail),
        .commit_valid(active_commit_valid),
        .commit_packet(active_commit_packet),
        .free_valid(active_free_valid),
        .free_prd(active_free_prd)
    );

    int_issue_queue IntIssueQueue (
`ifdef CSWHY
        .cswhy_probe_valid, .cswhy_probe_id(cswhy_head_id),
        .cswhy_iq_present, .cswhy_iq_ready, .cswhy_iq_picked, .cswhy_iq_multi,
`endif
        .clk,
        .rst_l,
        .insert_valid(int_insert_valid),
        .insert_entry(dispatch_issue_entries),
        .wakeup_valid,
        .wakeup_prd,
        .spec_wake_valid,
        .spec_wake_prd,
        .issue_ready(int_issue_ready),
        .reset_mask,
        .abort_mask,
        .flush(trap_take),
        .full(int_iq_full),
        .issue_valid(int_issue_valid),
        .issue_entry(int_issue_entry)
`ifdef LOAD_SPEC_WAKE
        ,
        .load_spec_wake_valid(lsq_load_spec_wake_valid),
        .load_spec_wake_prd  (lsq_load_spec_wake_prd),
        .ld_spec_hit,
        .ld_spec_miss,
        .issue_ld_spec(iq_issue_ld_spec)
`endif
    );

    // ---- Select -> Execute pipeline register (S2) for the ALU ports ----
    // The IQ select output (S1) is registered here; the ALU pipes, their phys-reg
    // reads, and the CSR reads all run from this registered copy (S2). An entry
    // aborted at its select cycle never enters S2 (branch_mask & abort_mask gate);
    // an entry aborted while in S2 is squashed by the ALU pipe's own abort handling
    // and by the spec_wake abort gate below. A pipeline flush (trap_take) forces
    // int_issue_valid to 0 in the IQ, which clears these registers next cycle.
    always_ff @(posedge clk or negedge rst_l) begin
        if (!rst_l) begin
            alu_issue_valid_q <= '0;
`ifdef LOAD_SPEC_WAKE
            alu_issue_ld_spec_q <= '0;
`endif
            for (int p = 0; p < ALU_ISSUE_PORTS; p += 1) begin
                alu_issue_entry_q[p] <= '0;
            end
        end else begin
            for (int p = 0; p < ALU_ISSUE_PORTS; p += 1) begin
                if (!trap_take && int_issue_valid[p] &&
                        ((int_issue_entry[p].branch_mask & abort_mask) == '0)) begin
                    alu_issue_valid_q[p] <= 1'b1;
                    alu_issue_entry_q[p] <= int_issue_entry[p];
`ifdef LOAD_SPEC_WAKE
                    alu_issue_ld_spec_q[p] <= iq_issue_ld_spec[p];
`endif
                end else begin
                    alu_issue_valid_q[p] <= 1'b0;
                    alu_issue_entry_q[p] <= '0;
`ifdef LOAD_SPEC_WAKE
                    alu_issue_ld_spec_q[p] <= 1'b0;
`endif
                end
            end
        end
    end

    // Speculative ALU wakeup: an ALU op executing in S2 broadcasts its dest a cycle
    // before its writeback bus appears (gated by abort -- a squashed producer must
    // not wake anyone). has_dest implies prd != 0.
    always_comb begin
        for (int p = 0; p < ALU_ISSUE_PORTS; p += 1) begin
`ifdef LOAD_SPEC_WAKE
            // LOAD_SPEC_WAKE grandchild suppression: a spec-woken consumer
            // whose load MISSED must not spec-wake a grandchild off stale data
            // this cycle -- the cascade is bounded to depth 1 by construction.
            // On a HIT the consumer spec-wakes normally (ALU chaining kept).
            spec_wake_valid[p] = alu_issue_valid_q[p] &&
                alu_issue_entry_q[p].has_dest &&
                ((alu_issue_entry_q[p].branch_mask & abort_mask) == '0) &&
                !(alu_issue_ld_spec_q[p] && ld_spec_miss);
`else
            spec_wake_valid[p] = alu_issue_valid_q[p] &&
                alu_issue_entry_q[p].has_dest &&
                ((alu_issue_entry_q[p].branch_mask & abort_mask) == '0);
`endif
            spec_wake_prd[p] = alu_issue_entry_q[p].prd;
        end
    end

`ifdef LOAD_SPEC_WAKE
    // LOAD_SPEC_WAKE: 1-deep broadcast tracker + the global hit/miss verdict.
    // Sole-outstanding at broadcast => a response this cycle is THIS load's
    // (a hit); its absence is the miss. Under trap_take the whole pipe flushes,
    // so cancel the pending bit (the S2 consumers are flushed too).
    always_ff @(posedge clk or negedge rst_l) begin
        if (!rst_l) begin
            ld_spec_pending_q <= 1'b0;
`ifdef LSQ_MLP2
            ld_spec_id_q <= '0;
`endif
        end else begin
            ld_spec_pending_q <= lsq_load_spec_wake_valid && !trap_take;
`ifdef LSQ_MLP2
            if (lsq_load_spec_wake_valid) begin
                ld_spec_id_q <= lsq_load_spec_wake_id;
            end
`endif
        end
    end
`ifdef LSQ_MLP2
    // Adversarial finding 1: resolve by the id the response NAMES, not raw
    // dmem_resp_valid -- a response for any other transaction must never read
    // as this load's hit. (Safe-by-construction today: the broadcast is gated
    // on inflight_count_q==0 and only one load issues per cycle, so the X+1
    // response can only be this load's -- the assertion below pins that LSQ
    // invariant so a future LSQ change cannot silently reintroduce the P3b
    // stale-data class.)
    assign ld_spec_hit  = ld_spec_pending_q && dmem_resp_valid &&
        (dmem_resp_id == ld_spec_id_q);
    assign ld_spec_miss = ld_spec_pending_q &&
        !(dmem_resp_valid && (dmem_resp_id == ld_spec_id_q));
`ifndef SYNTHESIS
    always_ff @(posedge clk) begin
        if (rst_l && ld_spec_pending_q && dmem_resp_valid) begin
            assert (dmem_resp_id == ld_spec_id_q) else
                $error("LOAD_SPEC_WAKE: dmem_resp_id %0d != spec id %0d (sole-outstanding invariant broken)",
                    dmem_resp_id, ld_spec_id_q);
        end
    end
`endif
`else
    // Base arm (single outstanding via mem_inflight_q): raw dmem_resp_valid.
    assign ld_spec_hit  = ld_spec_pending_q && dmem_resp_valid;
    assign ld_spec_miss = ld_spec_pending_q && !dmem_resp_valid;
`endif
`endif

    phys_reg_file #(.READ_PORTS(PHYS_READ_PORTS)) PhysRegFile (
        .clk,
        .rst_l,
        .rs1(phys_rs1),
        .rs2(phys_rs2),
        .write_valid(phys_write_valid),
        .write_prd(phys_write_prd),
        .write_data(phys_write_data),
        .rs1_data(phys_rs1_data),
        .rs2_data(phys_rs2_data)
    );

    ooo_alu_pipe ALU0 (
        .clk,
        .rst_l,
        .issue_valid(alu_issue_valid_q[0]),
        .issue_entry(alu_issue_entry_q[0]),
        .rs1_data(phys_rs1_data[0]),
        .rs2_data(phys_rs2_data[0]),
        .csr_rdata(csr_read_data[0]),
        .csr_illegal(csr_read_illegal[0]),
        .abort_mask,
        .flush(trap_take),
`ifdef LOAD_SPEC_WAKE
        .spec_squash(alu_issue_ld_spec_q[0] && ld_spec_miss),
`endif
        .writeback(alu0_writeback)
    );

    ooo_alu_pipe ALU1 (
        .clk,
        .rst_l,
        .issue_valid(alu_issue_valid_q[1]),
        .issue_entry(alu_issue_entry_q[1]),
        .rs1_data(phys_rs1_data[1]),
        .rs2_data(phys_rs2_data[1]),
        .csr_rdata(csr_read_data[1]),
        .csr_illegal(csr_read_illegal[1]),
        .abort_mask,
        .flush(trap_take),
`ifdef LOAD_SPEC_WAKE
        .spec_squash(alu_issue_ld_spec_q[1] && ld_spec_miss),
`endif
        .writeback(alu1_writeback)
    );

    assign int_issue_ready[ISSUE_ALU0] = 1'b1;
    assign int_issue_ready[ISSUE_ALU1] = 1'b1;

`ifdef ALU4
    // 3rd integer ALU issue port. Mirrors ALU0/ALU1 (S2-registered issue + phys
    // read at index ISSUE_ALU2, auto-routed by the parameterized fan-outs); CSR is
    // tied off since the priv CSR file has only 2 read ports (the IQ N-pick keeps
    // CSR ops on ALU0/ALU1 via sel_csr).
    ooo_alu_pipe ALU2 (
        .clk,
        .rst_l,
        .issue_valid(alu_issue_valid_q[ISSUE_ALU2]),
        .issue_entry(alu_issue_entry_q[ISSUE_ALU2]),
        .rs1_data(phys_rs1_data[ISSUE_ALU2]),
        .rs2_data(phys_rs2_data[ISSUE_ALU2]),
        .csr_rdata('0),
        .csr_illegal(1'b0),
        .abort_mask,
        .flush(trap_take),
`ifdef LOAD_SPEC_WAKE
        .spec_squash(alu_issue_ld_spec_q[ISSUE_ALU2] && ld_spec_miss),
`endif
        .writeback(alu2_writeback)
    );
    assign int_issue_ready[ISSUE_ALU2] = 1'b1;
`endif

    ooo_mul_unit MulUnit (
`ifdef CSWHY
        .cswhy_probe_valid, .cswhy_probe_id(cswhy_head_id),
        .cswhy_fu_busy(cswhy_mul_busy), .cswhy_fu_wb(cswhy_mul_wb),
`endif
        .clk,
        .rst_l,
        .issue_valid(int_issue_valid[ISSUE_MUL]),
        .issue_ready(int_issue_ready[ISSUE_MUL]),
        .issue_entry(int_issue_entry[ISSUE_MUL]),
        .rs1_data(phys_rs1_data[ISSUE_MUL]),
        .rs2_data(phys_rs2_data[ISSUE_MUL]),
        .abort_mask,
        .reset_mask,
        .flush(trap_take),
        .writeback_ready(mul_writeback_ready),
        .writeback(mul_writeback)
    );

    ooo_div_unit DivUnit (
`ifdef CSWHY
        .cswhy_probe_valid, .cswhy_probe_id(cswhy_head_id),
        .cswhy_fu_busy(cswhy_div_busy), .cswhy_fu_wb(cswhy_div_wb),
`endif
        .clk,
        .rst_l,
        .issue_valid(int_issue_valid[ISSUE_DIV]),
        .issue_ready(int_issue_ready[ISSUE_DIV]),
        .issue_entry(int_issue_entry[ISSUE_DIV]),
        .rs1_data(phys_rs1_data[ISSUE_DIV]),
        .rs2_data(phys_rs2_data[ISSUE_DIV]),
        .abort_mask,
        .reset_mask,
        .flush(trap_take),
        .writeback_ready(div_writeback_ready),
        .writeback(div_writeback)
    );

    niigo_fp_unit FpUnit (
`ifdef CSWHY
        .cswhy_probe_valid, .cswhy_probe_id(cswhy_head_id),
        .cswhy_fu_busy(cswhy_fp_busy), .cswhy_fu_wb(cswhy_fp_wb),
`endif
        .clk,
        .rst_l,
        .issue_valid(int_issue_valid[ISSUE_FP]),
        .issue_ready(int_issue_ready[ISSUE_FP]),
        .issue_entry(int_issue_entry[ISSUE_FP]),
        .rs1_data(phys_rs1_data[ISSUE_FP]),
        .frm(csr_frm),
        .abort_mask,
        .reset_mask,
        .flush(trap_take),
        .writeback_ready(fp_writeback_ready),
        .writeback(fp_writeback)
    );

`ifdef FUSE_LDBR
    // ---- FUSE_LDBR: pend_fbr — the 1-deep fused-branch-resolve buffer ----
    // The LSQ emits fused_resolve_* combinationally in the fused load's
    // final-beat retire; the fused-issue gate (load_store_queue) guarantees
    // pend_fbr is EMPTY at that moment, so the capture never clobbers. The
    // buffered resolve seats onto the writeback bus as the WB_FBR source ONLY
    // when no ALU branch resolves (branch_stack has ONE resolve port) and a WB
    // lane is free — completion + resolve + drain are atomic (spec blocker 3).
    always_comb begin
        pend_fbr_next = pend_fbr_q;
        // Spec blocker 1: age the held mask by reset_mask EVERY cycle (the
        // ooo_mul_unit.sv:86-87 contract) — a missed 1-cycle reset pulse would
        // leave a stale bit that later false-aborts on a reused checkpoint.
        pend_fbr_next.branch_mask = pend_fbr_q.branch_mask & ~reset_mask;
        // Drain: seated+resolved (or abort-dropped) by the WB bus this cycle.
        if (pend_fbr_drain_ready)
            pend_fbr_next = '0;
        // Capture (slot guaranteed free by the LSQ fused-issue gate). The
        // packet retires the BRANCH's own ROB slot (2-slot model; has_dest=0)
        // and drives the resolve with the slave's prediction/training identity
        // (the fuse_* fields, like the FUSE_CMPBR master's writeback).
        if (fused_resolve_valid) begin
            pend_fbr_next                  = '0;
            pend_fbr_next.valid            = 1'b1;
            pend_fbr_next.active_id        = fused_resolve_active_id;
            pend_fbr_next.pc               = fused_resolve_pc;
            pend_fbr_next.instr            = fused_resolve_instr; // OP_BRANCH => trains
            pend_fbr_next.has_dest         = 1'b0;
            pend_fbr_next.branch_mask      = fused_resolve_branch_mask & ~reset_mask;
            pend_fbr_next.branch_valid     = 1'b1;
            pend_fbr_next.branch_id        = fused_resolve_branch_id;
            pend_fbr_next.branch_mispredict= fused_resolve_mispredict;
            pend_fbr_next.redirect_pc      = fused_resolve_redirect_pc;
            pend_fbr_next.control_predicted= fused_resolve_ctrl_pred;
            pend_fbr_next.predicted_pc     = fused_resolve_pred_pc;
            pend_fbr_next.predictor_info   = fused_resolve_pred_info;
            pend_fbr_next.exception        = 1'b0;
            pend_fbr_next.fuse_is_branch     = 1'b1;
            pend_fbr_next.fuse_branch_pc     = fused_resolve_pc;
            pend_fbr_next.fuse_branch_instr  = fused_resolve_instr;
            pend_fbr_next.fuse_is_compressed = fused_resolve_is_comp;
            pend_fbr_next.is_compressed      = fused_resolve_is_comp;
        end
        // Spec blocker 3: abort a buffered (or just-captured) resolve whose
        // older branch mispredicts this cycle (comb abort_mask — the same
        // contract the mul/div/FP in-flight ops follow).
        if (pend_fbr_next.valid &&
                ((pend_fbr_next.branch_mask & abort_mask) != '0))
            pend_fbr_next = '0;
        // Spec blocker 2: a precise trap flushes everything (ooo_mul_unit.sv:77
        // takes flush=trap_take) — a stale resolve would corrupt the trap's
        // full-pipeline flush.
        if (trap_take)
            pend_fbr_next = '0;
    end

    always_ff @(posedge clk or negedge rst_l) begin
        if (!rst_l) pend_fbr_q <= '0;
        else        pend_fbr_q <= pend_fbr_next;
    end

    assign pend_fbr_full = pend_fbr_q.valid;
    // Drive gate (mirrors the ALU pipe's internal abort/flush suppression): a
    // same-cycle-aborted or trap-flushed resolve never seats/resolves; the
    // next-state logic above drops it.
    always_comb begin
        fbr_writeback = pend_fbr_q;
        if (((pend_fbr_q.branch_mask & abort_mask) != '0) || trap_take)
            fbr_writeback.valid = 1'b0;
    end
`endif

    load_store_queue #(.COHERENT(COHERENT)) LoadStoreQueue (
`ifdef CSWHY
        .cswhy_lsq_head_id, .cswhy_lsq_head_valid, .cswhy_lsq_reason,
`endif
        .clk,
        .rst_l,
        .insert_valid(mem_insert_valid),
        .insert_entry(dispatch_issue_entries),
        .insert_rs1_data(mem_insert_rs1_data),
        .insert_rs2_data(mem_insert_rs2_data),
        .wakeup_valid,
        .wakeup_prd,
        .wakeup_data(writeback_data),
        .reset_mask,
        .abort_mask,
        .flush(trap_take),
        .data_load_valid(dmem_resp_valid),
        // A load that hits a memory-mapped device (decoded against the echoed
        // response address) returns the device register value instead of the
        // (out-of-window) DRAM result.
        .data_load(clint_load_hit ? clint_load_data :
                   plic_load_hit  ? plic_load_data  :
                   uart_load_hit  ? uart_load_data  : dmem_resp_data),
        .dmem_req_ready(dmem_req_ready),
        .snoop_kill_valid(dmem_snoop_kill_valid),
        .snoop_kill_laddr(dmem_snoop_kill_laddr),
        .commit_store,
        .commit_store_id,
        .paging_data(paging_data),
        .xlate_stall(lsq_xlate_stall),
        .xlate_fault(lsq_xlate_fault),
        .xlate_cause(lsq_xlate_cause),
        .xlate_pa(data_pa),
        .mem_req_valid(mem_req_valid),
        .mem_req_vaddr(mem_req_vaddr),
        .mem_req_store(mem_req_store),
        .full(mem_queue_full),
        .data_load_en(mem_data_load_en),
        .data_addr(mem_data_addr),
        .data_store(mem_data_store),
        .data_store_mask(mem_data_store_mask),
        .dmem_req_op(mem_dmem_op),
        .dmem_req_amo(mem_dmem_amo),
`ifdef LSQ_MLP2
        .dmem_req_id(mem_dmem_id),
        .dmem_resp_id(dmem_resp_id),
`endif
        .store_second_beat(lsq_store_second_beat),
        .store_port_busy(lsq_store_port_busy),
        .head_load_off(dev_load_off),
        .sc_commit_done(lsq_sc_commit_done),
        .load_writeback,
`ifdef FUSE_LDBR
        .pend_fbr_full,
        .fused_resolve_valid,
        .fused_resolve_active_id,
        .fused_resolve_branch_id,
        .fused_resolve_pc,
        .fused_resolve_instr,
        .fused_resolve_redirect_pc,
        .fused_resolve_mispredict,
        .fused_resolve_ctrl_pred,
        .fused_resolve_pred_pc,
        .fused_resolve_pred_info,
        .fused_resolve_branch_mask,
        .fused_resolve_is_comp,
`endif
        .lsq_head_reason(lsq_head_reason),
        .lsq_fwd_class(lsq_fwd_class)
`ifdef LOAD_SPEC_WAKE
        ,
        .load_spec_wake_valid(lsq_load_spec_wake_valid),
        .load_spec_wake_prd  (lsq_load_spec_wake_prd)
`ifdef LSQ_MLP2
        ,
        .load_spec_wake_id   (lsq_load_spec_wake_id)
`endif
`endif
    );

    ooo_writeback_bus WritebackBus (
        .alu0_writeback,
        .alu1_writeback,
`ifdef ALU4
        .alu2_writeback,
`endif
        .load_writeback,
        .mul_writeback,
        .div_writeback,
        .fp_writeback,
`ifdef FUSE_LDBR
        .fbr_writeback,
`endif
        .abort_mask_q,
        .mul_writeback_ready,
        .div_writeback_ready,
        .fp_writeback_ready,
`ifdef FUSE_LDBR
        .fbr_writeback_ready(pend_fbr_drain_ready),
`endif
        .writeback_valid,
        .writeback_active_id,
        .writeback_prd,
        .writeback_data,
        .writeback_has_dest,
        .writeback_fp_write,
        .writeback_fp_rd,
        .writeback_fp_data,
        .writeback_csr_write,
        .writeback_csr_addr,
        .writeback_csr_wdata,
        .writeback_fp_fflags_valid,
        .writeback_fp_fflags,
        .writeback_exception,
        .writeback_exc_cause,
        .writeback_halted,
        .branch_writeback
    );

    ooo_branch_recovery BranchRecovery (
        .branch_writeback,
        .stack_reset_mask(stack_reset_mask),
        .stack_abort_mask(stack_abort_mask),
        .stack_restore_valid(branch_restore_valid),
        .fetch_pc_plus4(sequential_next_pc),
        .resolve_valid(branch_resolve_valid),
        .resolve_id(branch_resolve_id),
        .resolve_mispredict(branch_resolve_mispredict),
        .reset_mask,
        .abort_mask,
        .redirect_valid,
        .redirect_pc
    );

    ooo_commit_unit #(.COHERENT(COHERENT)) CommitUnit (
        .commit_valid(active_commit_valid),
        .commit_packet(active_commit_packet),
        .store_port_busy(lsq_store_port_busy),
        .store_port_ready(dmem_req_ready),
        .sc_commit_done(lsq_sc_commit_done),
        .retire_valid,
        .free_valid(commit_free_valid),
        .free_prd(commit_free_prd),
        .commit_store,
        .commit_store_id,
        .precise_halt,
        .precise_exception
    );

    // Shadow architectural register file: write-only (read addrs tied to 0, read
    // data unused) -- it exists purely for the sim register-file dump. Excluded
    // from synthesis (the lab register_file.sv carries a `string`-based dump that
    // is not synthesizable, and the shadow state has no datapath function).
`ifndef SYNTHESIS
    register_file #(.WAYS(OOO_WIDTH), .FORWARD(0)) ArchitecturalRF (
        .clk,
        .rst_l,
        .halted,
        .rd_we(arch_rd_we),
        .rs1(arch_rs1),
        .rs2(arch_rs2),
        .rd(arch_rd),
        .rd_data(arch_rd_data),
        .rs1_data(arch_rs1_data),
        .rs2_data(arch_rs2_data)
    );
`else
    assign arch_rs1_data = '0;
    assign arch_rs2_data = '0;
`endif

    always_comb begin
        direct_lookup_valid = 1'b0;
        direct_lookup_pc = '0;
        indirect_lookup_valid = 1'b0;
        indirect_lookup_pc = '0;
        for (int i = 0; i < OOO_WIDTH; i += 1) begin
            if (decode_lanes[i].valid && !decode_lanes[i].kill &&
                    (decode_lanes[i].ctrl.pc_source == PC_cond) &&
                    !direct_lookup_valid) begin
                direct_lookup_valid = 1'b1;
                direct_lookup_pc = decode_lanes[i].pc;
            end
            if (decode_lanes[i].valid && !decode_lanes[i].kill &&
                    (decode_lanes[i].ctrl.pc_source == PC_indirect) &&
                    !((decode_lanes[i].rs1 == 5'd1) &&
                      (decode_lanes[i].rd == 5'd0)) &&
                    !indirect_lookup_valid) begin
                indirect_lookup_valid = 1'b1;
                indirect_lookup_pc = decode_lanes[i].pc;
            end
        end
    end

`ifdef DFE_S1
`ifdef RVC
    // ============ DFE S1a: fetch-side branch predecode (INERT, no consumer) ============
    // Independent raw-block scan that reproduces WHICH in-block instruction is the first
    // controlling conditional / first non-return indirect of the drain window the
    // realigner presents this cycle. Compared one-directionally against the dispatch-
    // directed lookup PCs by the assertion below. A different traversal of the same bytes
    // -- reusing rvc_lane_pc would be a tautology. RVC-only (DFE_S1 is OoO-only, OoO
    // requires RVC).
    logic            pd_cond_valid;  logic [XLEN-1:0] pd_cond_pc;
    logic            pd_ind_valid;   logic [XLEN-1:0] pd_ind_pc;

    always_comb begin
        logic [15:0]     pd_par [8];
        logic [XLEN-1:0] pd_base, pd_pc;
        logic [3:0]      pd_cur;
        logic            pd_stop;
        logic [15:0]     pd_p, pd_hi;
        logic [4:0]      pd_rs1, pd_rd;
        logic            pd_is32, pd_is_cond, pd_is_jalr, pd_is_jal, pd_is_ret;

        for (int k = 0; k < 8; k += 1)
            pd_par[k] = fgrp_data[k[2]][(k[1:0]) * 16 +: 16];   // == rvc_realign pblock
        pd_base = {fgrp_pc[XLEN-1:4], 4'b0};
        pd_cond_valid = 1'b0; pd_cond_pc = '0;
        pd_ind_valid  = 1'b0; pd_ind_pc  = '0;
        pd_stop = 1'b0; pd_cur = {1'b0, dfe_s0};
        pd_pc='0; pd_p='0; pd_hi='0; pd_rs1='0; pd_rd='0;
        pd_is32=1'b0; pd_is_cond=1'b0; pd_is_jalr=1'b0; pd_is_jal=1'b0; pd_is_ret=1'b0;

        // Straddle completion: the prev block's 32-bit low half is this cycle's lane0.
        if (dfe_completing) begin
            if (dfe_straddle_half[6:2] == 5'b11000) begin
                pd_cond_valid = 1'b1; pd_cond_pc = dfe_straddle_pc;   // 32-bit BRANCH
            end else if (dfe_straddle_half[6:2] == 5'b11001) begin
                // 32-bit JALR straddling parcel 7. rs1's upper 4 bits are in the
                // not-yet-arrived high half, so a return cannot be disambiguated here.
                // OVER-detect the indirect pick (Finding #1): the assertion is one-
                // directional and decode excludes ONLY the true return (rs1==1 && rd==0),
                // whose indirect_lookup_valid is already 0.
                pd_ind_valid = 1'b1; pd_ind_pc = dfe_straddle_pc;
                pd_stop = 1'b1;
            end else if (dfe_straddle_half[6:2] == 5'b11011) begin
                pd_stop = 1'b1;                                       // 32-bit JAL
            end
            pd_cur = 4'd1;                                           // parcel0 high half consumed
        end

        for (int step = 0; step < 8; step += 1) begin
            if (!pd_stop && (pd_cur <= 4'd7) && fgrp_valid) begin
                pd_p   = pd_par[pd_cur[2:0]];
                pd_hi  = (pd_cur < 4'd7) ? pd_par[pd_cur[2:0] + 3'd1] : 16'h0000;
                pd_pc  = pd_base + {pd_cur[2:0], 1'b0};
                pd_is32= (pd_p[1:0] == 2'b11);
                pd_rd  = pd_p[11:7];
                pd_rs1 = pd_is32 ? {pd_hi[3:0], pd_p[15]} : pd_p[11:7];
                pd_is_cond = (pd_is32 && (pd_p[6:2]==5'b11000)) ||
                             ((pd_p[1:0]==2'b01) && ((pd_p[15:13]==3'b110)||(pd_p[15:13]==3'b111)));
                pd_is_jal  = (pd_is32 && (pd_p[6:2]==5'b11011)) ||
                             ((pd_p[1:0]==2'b01) && (pd_p[15:13]==3'b101));
                pd_is_jalr = (pd_is32 && (pd_p[6:2]==5'b11001)) ||
                             ((pd_p[1:0]==2'b10) && (pd_p[15:13]==3'b100) &&
                              (pd_p[6:2]==5'd0) && (pd_p[11:7]!=5'd0));
                pd_is_ret  = pd_is_jalr &&
                             ( (pd_is32 && (pd_rs1==5'd1) && (pd_rd==5'd0)) ||
                               (!pd_is32 && (pd_p[12]==1'b0) && (pd_p[11:7]==5'd1)) );

                if (pd_is32 && (pd_cur == 4'd7)) begin
                    pd_stop = 1'b1;                    // parcel-7 low half: caught via completing
                end else if (pd_is_jal || pd_is_ret) begin
                    pd_stop = 1'b1;                    // squashes younger, not a lookup pick
                end else if (pd_is_jalr) begin
                    if (!pd_ind_valid) begin pd_ind_valid=1'b1; pd_ind_pc=pd_pc; end
                    pd_stop = 1'b1;                    // non-return indirect squashes younger
                end else if (pd_is_cond) begin
                    if (!pd_cond_valid) begin pd_cond_valid=1'b1; pd_cond_pc=pd_pc; end
                    pd_cur = pd_cur + (pd_is32 ? 4'd2 : 4'd1);   // cond does NOT squash younger
                end else begin
                    pd_cur = pd_cur + (pd_is32 ? 4'd2 : 4'd1);
                end
            end
        end
    end

`ifndef SYNTHESIS
    // #1 KILL SIGNAL: the fetch-side predecode must name the SAME conditional/indirect
    // branch PC the dispatch-directed TAGE/ITTAGE lookup keys on (identical prediction
    // input key). One-directional: lookup_valid => predecode names it here. A mismatch
    // means the FTQ would predict from a different key => accuracy would diverge =>
    // the universal-no-regression premise is false. Hard stop.
    always_ff @(posedge clk) begin
        if (rst_l && fgrp_valid && !halted_q && !fetch_flush) begin
            if (direct_lookup_valid)
                assert (pd_cond_valid && (pd_cond_pc == direct_lookup_pc))
                  else $error("DFE_S1 cond mismatch: pd=%h disp=%h (fgrp_pc=%h s0=%0d compl=%b)",
                              pd_cond_pc, direct_lookup_pc, fgrp_pc, dfe_s0, dfe_completing);
            if (indirect_lookup_valid)
                assert (pd_ind_valid && (pd_ind_pc == indirect_lookup_pc))
                  else $error("DFE_S1 ind mismatch: pd=%h disp=%h (fgrp_pc=%h s0=%0d compl=%b)",
                              pd_ind_pc, indirect_lookup_pc, fgrp_pc, dfe_s0, dfe_completing);
        end
    end
`endif
`endif
`endif

    // ---- Phys-reg read-address fan-out (FB2b false-loop break) ----
    // Extracted from the monolithic dispatch always_comb so that the read of
    // int_issue_entry / alu_issue_entry_q here does NOT alias into that block's
    // hundreds of unrelated outputs (lane_valid, dispatch_valid, commit_exc_valid,
    // branch_allocate ...). Verilator/Vivado analyze dependencies at whole-block
    // granularity, so an int_issue_entry read inside the dispatch block drew a
    // spurious edge to every output of it, closing the issue/wakeup/abort_mask
    // false combinational loops. With this as its own block the dependency graph is
    // the real (acyclic) one. Value-identical -- pure code motion.
    always_comb begin
        for (int i = 0; i < FU_ISSUE_PORTS; i += 1) begin
            // ALU ports read the phys regfile in S2 (from the registered issue
            // entry); MUL/DIV/FP read at select (combinational issue), unchanged.
            if (i < ALU_ISSUE_PORTS) begin
                phys_rs1[i] = alu_issue_entry_q[i].prs1;
                phys_rs2[i] = alu_issue_entry_q[i].prs2;
            end else begin
                phys_rs1[i] = int_issue_entry[i].prs1;
                phys_rs2[i] = int_issue_entry[i].prs2;
            end
        end
        for (int i = 0; i < OOO_WIDTH; i += 1) begin
            phys_rs1[FU_ISSUE_PORTS + i] = dispatch_issue_entries[i].prs1;
            phys_rs2[FU_ISSUE_PORTS + i] = dispatch_issue_entries[i].prs2;
        end
    end

    // ---- Mem-operand data read + writeback fan-out (wakeup / phys-write) ----
    // Reads the phys-regfile OUTPUT (phys_rs*_data) and the writeback bus, so it
    // is a SEPARATE block from the read-address fan-out above (which writes
    // phys_rs*) -- keeping them split puts the regfile flop between them in the
    // dependency graph (addr -> regfile -> data) instead of a same-block apparent
    // cycle. wakeup/phys_write are pure writeback-bus fan-out. Value-identical.
    always_comb begin
        for (int i = 0; i < OOO_WIDTH; i += 1) begin
            mem_insert_rs1_data[i] = phys_rs1_data[FU_ISSUE_PORTS + i];
            mem_insert_rs2_data[i] = phys_rs2_data[FU_ISSUE_PORTS + i];
        end
        for (int i = 0; i < OOO_WIDTH; i += 1) begin
            wakeup_valid[i] = writeback_valid[i] && writeback_has_dest[i];
            wakeup_prd[i] = writeback_prd[i];
            phys_write_valid[i] = writeback_valid[i] && writeback_has_dest[i];
            phys_write_prd[i] = writeback_prd[i];
            phys_write_data[i] = writeback_data[i];
        end
    end

    // frontend_stall: pure combinational of two module-input stalls; a
    // continuous assign keeps it out of the procedural dispatch blocks.
    assign frontend_stall = dispatch_stall || fetch_xlate_stall;

    // ================================================================
    // Dispatch pipeline, decomposed into single-purpose comb blocks
    // (FB2b Wall A). The former 700-line monolith drew false
    // combinational-loop edges between every input it read and every
    // output it wrote (Verilator/Vivado analyze dependencies at
    // whole-block granularity), corrupting the post-place STA. Splitting
    // it into the real dataflow stages -- LaneDecode -> BranchPredict ->
    // Rename, and CommitTrap -> InterruptDrain -> FrontendControl --
    // gives the tool the true (acyclic) graph. Each block is
    // value-identical pure code motion: every signal has exactly one
    // driver block, and every cross-block read is of a settled value
    // (no earlier block reads a later block's output). Inits live with
    // their owning block.
    // ================================================================

`ifdef FUSE_ANY
    // ---- B0-fuse: intra-group ADJACENT fusion-pair detector (shared-infra §3).
    // Placed BEFORE B1a: B1a reads fuse_slave[] to force the folded lane's
    // lane_has_dest=0, and B3 reads fuse_master[] to build the payload. Reads
    // decode_lanes ONLY (valid, rs1/rs2/rd, ctrl, uses_rs1, uses_rs2, imm, pc,
    // instr) -- NOT dispatch_valid -- so it introduces no combinational cycle
    // through dispatch_control (the lane_has_dest -> dispatch_control ->
    // dispatch_valid loop would corrupt STA; shared-infra §4c/§5). ----
    always_comb begin
        fuse_master = '0;
        fuse_slave  = '0;
        for (int i = 0; i < OOO_WIDTH; i += 1) fuse_kind_lane[i] = FUSE_K_NONE;
`ifdef FUSE_LDBR
        fuse_slave_ldbr = '0;
`endif
`ifdef FUSE_UADDR
        fu_zero_rs1 = '0;
        fu_imm_ovr  = '0;
        fu_imm      = '0;
        fu_pc_base  = '0;
`endif
`ifdef FUSE_CMPBR_LI
        fu_caseb     = '0;
        fu_caseb_src = '0;
`endif

        // Left-to-right disjoint pairing: a lane already claimed as a slave cannot
        // also be a master (no chained triple-fusion). i only ranges 0..OOO_WIDTH-2
        // (intra-group ADJACENT); cross-group pairs need the carried fusion-prefix
        // latch (an XL) and are out of scope.
        for (int i = 0; i < OOO_WIDTH-1; i += 1) begin
            automatic logic prod_ok, cons_ok, struct_ok;
`ifdef FUSE_UADDR
            automatic logic c_pair, first_mem;
`endif
`ifdef FUSE_CMPBR_LI
            automatic logic struct_ok_li, caseb_ok;
`endif
            // --- structural (shared) ---
            struct_ok =
                decode_lanes[i].valid && !decode_lanes[i].kill &&
                decode_lanes[i+1].valid && !decode_lanes[i+1].kill &&
                !fuse_slave[i] &&                                   // i not already a slave
                decode_lanes[i].ctrl.rfWrite && (decode_lanes[i].rd != 5'd0) && // master rd live
                // master must not itself terminate/steer the group before the slave:
                !decode_lanes[i].ctrl.serializing &&
                !decode_lanes[i].ctrl.syscall && !decode_lanes[i].ctrl.illegal_instr &&
                (decode_lanes[i].ctrl.pc_source == PC_plus4) &&
                // slave's SOLE gpr source is the master's rd (adjacency guarantees the
                // master is the only in-group producer between them):
                decode_lanes[i+1].uses_rs1 &&
                (decode_lanes[i+1].rs1 == decode_lanes[i].rd);

            // --- per-lever producer/consumer opcode predicate (hooks; the levers
            //     refine these — shown here as the reference predicates) ---
            prod_ok = 1'b0; cons_ok = 1'b0;
`ifdef FUSE_UADDR
            c_pair = 1'b0;
            // LUI/AUIPC  ->  ADDI/ADDIW rd, rd, imm (constant / pc-rel address build).
            // Slave must NOT use rs2 (I-type), so master.rd is the sole source.
            // fetch_fault on either lane excludes the pair (nulling a fault
            // carrier would drop its trap); !illegal_instr on the slave keeps an
            // RV32 0x1B (OP-IMM-32, reserved at RV32) encoding from being nulled.
            if ((decode_lanes[i].instr[6:0] == RISCV_ISA::OP_LUI ||
                 decode_lanes[i].instr[6:0] == RISCV_ISA::OP_AUIPC) &&
                (decode_lanes[i+1].instr[6:0] == RISCV_ISA::OP_IMM ||
                 decode_lanes[i+1].instr[6:0] == 7'h1B /*OP-IMM-32*/) &&
                (decode_lanes[i+1].instr[14:12] == 3'b000) &&   // ADDI/ADDIW funct3
                !decode_lanes[i+1].uses_rs2 &&
                (decode_lanes[i+1].rd == decode_lanes[i].rd) &&
                !decode_lanes[i].ctrl.fetch_fault &&
                !decode_lanes[i+1].ctrl.fetch_fault &&
                !decode_lanes[i+1].ctrl.illegal_instr) begin
                prod_ok = 1'b1; cons_ok = 1'b1;
                if (struct_ok && !fuse_master[i]) fuse_kind_lane[i] = FUSE_K_UADDR;
            end
            // (c) AUIPC -> integer LOAD rd, off(rd) (canonical PIC/GOT idiom):
            // keep the YOUNGER load, null the OLDER auipc (fuse-uaddr.md §1).
            // load.rd == auipc.rd makes the auipc's intermediate provably dead;
            // G1 (first_mem: no older valid mem lane) + G2 (whole-group dests fit
            // the free list) make the kill-older direction dispatch-atomic — with
            // them no per-lane stop can land between lanes i and i+1.
            else if ((decode_lanes[i].instr[6:0] == RISCV_ISA::OP_AUIPC) &&
                (decode_lanes[i+1].instr[6:0] == RISCV_ISA::OP_LOAD) &&
                (decode_lanes[i+1].rd == decode_lanes[i].rd) &&   // base provably dead
                !decode_lanes[i+1].ctrl.illegal_instr &&
                !decode_lanes[i].ctrl.fetch_fault &&
                !decode_lanes[i+1].ctrl.fetch_fault) begin
                // G1: the load is the group's first memory (mirrors
                // dispatch_control's memory_seen, which folds lane_valid).
                first_mem = 1'b1;
                for (int j = 0; j < OOO_WIDTH; j += 1)
                    if ((j < i) && decode_lanes[j].valid && !decode_lanes[j].kill &&
                        (decode_lanes[j].ctrl.memRead || decode_lanes[j].ctrl.memWrite))
                        first_mem = 1'b0;
                // G2: free-list headroom for the whole group's dests (no
                // dest-block stop possible at any lane this cycle).
                if (first_mem &&
                        (free_count_snapshot >= ($clog2(PHYS_REGS+1))'(OOO_WIDTH))) begin
                    prod_ok = 1'b1; cons_ok = 1'b1; c_pair = 1'b1;
                end
            end
`endif
`ifdef FUSE_CMPBR
            // FUSE_CMPBR Inc 1 — Case A reg-reg (fuse-cmpbr.md §3/§10): a
            // single-cycle integer ALU producer writing rd, followed by a
            // conditional branch testing rd against x0 (beqz/bnez/bltz/bgez rd;
            // rs2==x0 + struct_ok's rs1==rd). branch_cmp(result, '0, op) covers
            // all six cond ops value-identically, so the consumer alu_op needs
            // no restriction. Producer exclusions: MEM (loads/stores decode
            // EXEC_INT too — a load master would route to the LSQ and never
            // resolve the folded branch), MUL/DIV (long-latency FUs), CSR/FP/
            // AMO/FENCE (exec_class != EXEC_INT), AUIPC (usePC), immediates
            // (Inc 1 is reg-reg only; lifted under FUSE_CMPBR_LI = Case A-imm),
            // and any fetch-fault carrier. struct_ok already enforces rfWrite,
            // rd!=0, PC_plus4, and non-serializing/syscall/illegal on the
            // master, plus slave.uses_rs1 && slave.rs1 == master.rd.
            if ((decode_lanes[i].ctrl.exec_class == EXEC_INT) &&
                !is_mul_op(decode_lanes[i].ctrl.alu_op) &&
                !is_div_op(decode_lanes[i].ctrl.alu_op) &&
                !decode_lanes[i].ctrl.memRead &&
                !decode_lanes[i].ctrl.memWrite &&
                !decode_lanes[i].ctrl.usePC &&
`ifndef FUSE_CMPBR_LI
                !decode_lanes[i].ctrl.useImm &&   // Inc 1: reg-reg producers only
`endif
                !decode_lanes[i].ctrl.fetch_fault &&
                !decode_lanes[i+1].ctrl.fetch_fault &&
                (decode_lanes[i+1].ctrl.pc_source == PC_cond) &&
                (decode_lanes[i+1].rs2 == 5'd0)) begin       // branch vs x0
                prod_ok = 1'b1; cons_ok = 1'b1;
                if (struct_ok && !fuse_master[i]) fuse_kind_lane[i] = FUSE_K_CMPBR;
            end
`endif
`ifdef FUSE_CMPBR_LI
            // FUSE_CMPBR Inc 2 Case B (fuse-cmpbr.md §7b): li rt,imm (single-
            // instruction OP_IMM ADDI with rs1==x0 — c.li expands to the same
            // form) followed by beq/bne comparing a register against rt. The
            // branch's OTHER operand (not the li's rd) is re-read through the
            // li master's unused rs2 port; BEQ/BNE are commutative so the
            // operand order is irrelevant. struct_ok_li mirrors struct_ok but
            // allows the li's rd at EITHER branch source. NB: the
            // struct_ok-form `li rt; beqz rt` is already claimed by the Inc 1
            // arm above (useImm is lifted under FUSE_CMPBR_LI); this arm takes
            // only the pairs struct_ok rejects.
            struct_ok_li =
                decode_lanes[i].valid && !decode_lanes[i].kill &&
                decode_lanes[i+1].valid && !decode_lanes[i+1].kill &&
                !fuse_slave[i] &&
                decode_lanes[i].ctrl.rfWrite && (decode_lanes[i].rd != 5'd0) &&
                !decode_lanes[i].ctrl.serializing &&
                !decode_lanes[i].ctrl.syscall && !decode_lanes[i].ctrl.illegal_instr &&
                (decode_lanes[i].ctrl.pc_source == PC_plus4) &&
                ((decode_lanes[i+1].rs1 == decode_lanes[i].rd) ||
                 (decode_lanes[i+1].rs2 == decode_lanes[i].rd));
            caseb_ok =
                (decode_lanes[i].instr[6:0] == RISCV_ISA::OP_IMM) &&
                (decode_lanes[i].instr[14:12] == 3'b000) &&   // ADDI
                (decode_lanes[i].rs1 == 5'd0) &&               // li form
                decode_lanes[i].ctrl.useImm &&
                !decode_lanes[i].ctrl.fetch_fault &&
                !decode_lanes[i+1].ctrl.fetch_fault &&
                (decode_lanes[i+1].ctrl.pc_source == PC_cond) &&
                ((decode_lanes[i+1].ctrl.alu_op == ALU_BEQ) ||
                 (decode_lanes[i+1].ctrl.alu_op == ALU_BNE));
`endif
`ifdef FUSE_LDBR
            // FUSE_LDBR — a plain integer LOAD writing rd, followed by a
            // conditional branch testing rd against x0 (beqz/bnez/bltz/bgez;
            // branch_cmp covers all six cond ops value-identically, so the
            // consumer alu_op is unrestricted). exec_class==EXEC_INT excludes
            // AMO/LR (EXEC_AMO — a fused LR would lose its reservation
            // writeback arm and never resolve) and FP loads (EXEC_FP — the
            // result is an FPR, not a GPR the branch reads); fetch_fault on
            // either lane excludes the pair (a faulting load takes its trap
            // before resolving; the folded slave would wait on a resolve that
            // never comes). struct_ok already enforces rfWrite, rd!=0,
            // PC_plus4, non-serializing/syscall/illegal on the master, plus
            // slave.uses_rs1 && slave.rs1 == master.rd. v1 scope: rs2==x0
            // only — a dynamic 2nd operand (the li;beq enum idiom) is the
            // fuse_cmp_rs2-style extension, out of scope.
            if (decode_lanes[i].ctrl.memRead && !decode_lanes[i].ctrl.memWrite &&
                (decode_lanes[i].ctrl.exec_class == EXEC_INT) &&
                !decode_lanes[i].ctrl.fetch_fault &&
                !decode_lanes[i+1].ctrl.fetch_fault &&
                (decode_lanes[i+1].ctrl.pc_source == PC_cond) &&
                (decode_lanes[i+1].rs2 == 5'd0)) begin
                prod_ok = 1'b1; cons_ok = 1'b1;
                if (struct_ok && !fuse_master[i]) fuse_kind_lane[i] = FUSE_K_LDBR;
            end
`endif
            if (struct_ok && prod_ok && cons_ok && !fuse_master[i]) begin
`ifdef FUSE_UADDR
                if (c_pair) begin
                    // (c) AUIPC+LOAD: null the OLDER auipc (born-done ROB slot);
                    // the younger load survives with the pc-base AGU override,
                    // the folded hi+off immediate, and rs1 no longer read. Not
                    // marked fuse_master/fuse_kind — the load executes no folded
                    // ALU op (its fold is the AGU base mux in the LSQ).
                    fuse_slave[i]    = 1'b1;
                    fu_zero_rs1[i+1] = 1'b1;
                    fu_imm_ovr[i+1]  = 1'b1;
                    fu_imm[i+1]      = decode_lanes[i].imm + decode_lanes[i+1].imm;
                    fu_pc_base[i+1]  = 1'b1;
                end else
`endif
                begin
                    fuse_master[i]   = 1'b1;
                    fuse_slave[i+1]  = 1'b1;   // consumed; loop's !fuse_slave[i] blocks i+1 master
`ifdef FUSE_LDBR
                    fuse_slave_ldbr[i+1] = (fuse_kind_lane[i] == FUSE_K_LDBR);
`endif
                end
            end
`ifdef FUSE_CMPBR_LI
            // Case B claim (struct_ok rejected the pair because the li's rd is
            // not the branch's rs1): fold the branch into the li master.
            if (struct_ok_li && caseb_ok && !fuse_master[i]) begin
                fuse_master[i]   = 1'b1;
                fuse_slave[i+1]  = 1'b1;
                fuse_kind_lane[i] = FUSE_K_CMPBR;
                fu_caseb[i]      = 1'b1;
                fu_caseb_src[i]  = (decode_lanes[i+1].rs1 == decode_lanes[i].rd) ?
                    decode_lanes[i+1].rs2 : decode_lanes[i+1].rs1;
            end
`endif
        end
    end
`endif

    // ---- B1a: lane decode (per-lane attributes + rename source regs + valid
    // count). Reads decode_lanes ONLY -- NOT dispatch_valid -- so lane_valid (and
    // the lane_is_* attrs) no longer whole-block-alias dispatch_valid (the false
    // dispatch_valid -> lane_valid -> dispatch_control -> dispatch_valid loop edge).
    // Value-identical: the dispatch-count / partial-resume half moved to B1b. ----
    always_comb begin
        lane_valid = '0;
        lane_has_dest = '0;
        lane_is_branch = '0;
        lane_is_unpredicted_control = '0;
        lane_is_call = '0;
        lane_is_return = '0;
        lane_is_memory = '0;
        lane_is_terminal = '0;
        lane_is_serializing = '0;
        lane_is_fp = '0;
        lane_fp_src_busy = '0;
        lane_reads_fflags = '0;
        valid_count = '0;

        for (int i = 0; i < OOO_WIDTH; i += 1) begin
`ifdef FUSE_UADDR
            // FUSE_UADDR (c): the surviving load's base is pc_auipc (AGU
            // pc-base), so it no longer reads rs1 -- map x0 (prs1=0,
            // busy-ready) instead of the nulled auipc's stale mapping.
            map_rs1[i] = fu_zero_rs1[i] ? 5'd0 : decode_lanes[i].rs1;
`else
            map_rs1[i] = decode_lanes[i].rs1;
`endif
`ifdef FUSE_CMPBR_LI
            // FUSE_CMPBR Case B: the li master re-reads the folded branch's
            // OTHER source register through its unused rs2 port (its own rs2
            // field is not a source — OP_IMM uses_rs2==0). prs2 and the busy
            // lookup follow the override automatically (fu_zero_rs1 pattern).
            map_rs2[i] = fu_caseb[i] ? fu_caseb_src[i] : decode_lanes[i].rs2;
`else
            map_rs2[i] = decode_lanes[i].rs2;
`endif
            map_rd[i] = decode_lanes[i].rd;
            lane_valid[i] = decode_lanes[i].valid && !decode_lanes[i].kill;
`ifdef FUSE_ANY
            // Folded fusion slave does no arch write (its value folds to
            // master.prd); transitively kills its free-list allocate.
            lane_has_dest[i] = lane_valid[i] && decode_lanes[i].ctrl.rfWrite &&
                (decode_lanes[i].rd != 5'd0) && !fuse_slave[i];
`else
            lane_has_dest[i] = lane_valid[i] && decode_lanes[i].ctrl.rfWrite &&
                (decode_lanes[i].rd != 5'd0);
`endif
            lane_is_branch[i] = lane_valid[i] &&
                ((decode_lanes[i].ctrl.pc_source == PC_cond) ||
                 (decode_lanes[i].ctrl.pc_source == PC_uncond) ||
                 (decode_lanes[i].ctrl.pc_source == PC_indirect));
`ifdef JAL_NO_CKPT
            lane_needs_ckpt[i] = lane_is_branch[i] &&
                (decode_lanes[i].ctrl.pc_source != PC_uncond);
`endif
            lane_is_unpredicted_control[i] = lane_valid[i] &&
                ((decode_lanes[i].ctrl.pc_source == PC_uncond) ||
                 (decode_lanes[i].ctrl.pc_source == PC_indirect));
            lane_is_call[i] = lane_valid[i] &&
                (decode_lanes[i].ctrl.pc_source == PC_uncond) &&
                (decode_lanes[i].rd != 5'd0);
            lane_is_return[i] = lane_valid[i] &&
                (decode_lanes[i].ctrl.pc_source == PC_indirect) &&
                (decode_lanes[i].rs1 == 5'd1) &&
                (decode_lanes[i].rd == 5'd0);
            lane_is_memory[i] = lane_valid[i] &&
                (decode_lanes[i].ctrl.memRead || decode_lanes[i].ctrl.memWrite);
            lane_is_terminal[i] = lane_valid[i] &&
                (decode_lanes[i].ctrl.syscall || decode_lanes[i].ctrl.illegal_instr);
            lane_is_serializing[i] = lane_valid[i] &&
                decode_lanes[i].ctrl.serializing;
`ifdef FP_OOO
            // P5b: an FP lane touches an FPR (source or dest). lane_fp_src_busy
            // folds the source-read stall AND the WAW dest stall (one in-flight
            // writer per FPR) -- both read the REGISTERED busy_q so a producer's
            // fp_regs_q write and its busy-clear land on the same edge (a reader
            // that sees the bit cleared reads the freshly-committed value).
            lane_is_fp[i] = lane_valid[i] &&
                (decode_lanes[i].ctrl.fp_writes_fpr ||
                 decode_lanes[i].ctrl.fp_uses_rs1 ||
                 decode_lanes[i].ctrl.fp_uses_rs2 ||
                 decode_lanes[i].ctrl.fp_uses_rs3);
            lane_fp_src_busy[i] = lane_valid[i] &&
                ((decode_lanes[i].ctrl.fp_uses_rs1 &&
                    fpr_busy_q[decode_lanes[i].rs1]) ||
                 (decode_lanes[i].ctrl.fp_uses_rs2 &&
                    fpr_busy_q[decode_lanes[i].rs2]) ||
                 (decode_lanes[i].ctrl.fp_uses_rs3 &&
                    fpr_busy_q[decode_lanes[i].instr[31:27]]) ||
                 (decode_lanes[i].ctrl.fp_writes_fpr &&
                    fpr_busy_q[decode_lanes[i].rd]));
            // A CSR op reading fflags(0x001)/frm(0x002)/fcsr(0x003) must see all
            // older FP fflags-producers' accumulated flags, which land only at
            // their commit -> hold it until the ROB drains (rare; see below).
            lane_reads_fflags[i] = lane_valid[i] &&
                (decode_lanes[i].ctrl.exec_class == EXEC_CSR) &&
                (decode_lanes[i].instr[31:20] inside {12'h001, 12'h002, 12'h003});
`endif
            if (lane_valid[i]) begin
                valid_count += 1'b1;
            end
        end
    end

`ifdef FP_OOO
    // fflags-read drain interlock: while any presented lane reads fflags/fcsr and
    // the ROB is non-empty of older ops, suppress the whole group's dispatch. The
    // ROB drains by commit (its ops are all older than the not-yet-dispatched
    // reader), so this is deadlock-free; fflags reads are rare (FP exception
    // checks / context save), so the conservatism is free.
    assign fflags_drain_stall = (|(lane_valid & lane_reads_fflags)) && !active_empty;
`else
    assign fflags_drain_stall = 1'b0;
`endif

    // ---- B1b: dispatch-count + active-list offset + partial-resume. Reads
    // dispatch_valid (and the B1a lane attrs), kept separate from the decode above
    // so dispatch_valid does not reach lane_valid. Same running-count semantics
    // (lane_active_offset[i] = dispatch_count before this lane's +1). ----
    always_comb begin
        dispatch_count = '0;
        partial_resume_valid = 1'b0;
        partial_resume_lane = '0;
        partial_resume_lane_is_branch = 1'b0;

        for (int i = 0; i < OOO_WIDTH; i += 1) begin
            lane_active_offset[i] = dispatch_count;
            if (dispatch_valid[i]) begin
                dispatch_count += 1'b1;
            end else if (lane_valid[i] && !partial_resume_valid) begin
                partial_resume_valid = 1'b1;
                partial_resume_lane = 3'(i);
                partial_resume_lane_is_branch = lane_is_branch[i];
            end
        end
    end

    // ---- B2: branch prediction + checkpoint (RAS/GHR/predictor
    // redirects, branch-stack snapshots, control-predicted lane attrs) ----
    always_comb begin
        // Per-lane control-predicted / predicted-PC / predictor-info
        // defaults (overridden below for predicted lanes).
        lane_control_predicted = '0;
        dispatched_unpredicted_control = 1'b0;
        for (int i = 0; i < OOO_WIDTH; i += 1) begin
            lane_predicted_pc[i] = decode_lanes[i].pc +
                `ILEN_INC(decode_lanes[i].ctrl.is_compressed);
            lane_predictor_info[i] = '0;
        end
        ras_redirect_valid = 1'b0;
        ras_redirect_pc = '0;
        predictor_redirect_valid = 1'b0;
        predictor_redirect_pc = '0;
        btb_branch_pc = '0;
        btb_branch_is_c = 1'b0;
        ras_stack_next = ras_stack_q;
        ras_count_next = branch_restore_valid ?
            ras_checkpoint_count_q[branch_resolve_id] : ras_count_q;
        ras_branch_snapshot_count = ras_count_next;
        // Speculative global history: on a misprediction restore the branch's
        // pre-push checkpoint, then re-push the resolved direction if the
        // resolving branch was conditional (mirrors the RAS recovery above).
        ghr_next = branch_restore_valid ?
            ghr_checkpoint_q[branch_resolve_id] : ghr_q;
`ifdef FUSE_BRANCH
        if (branch_restore_valid && branch_resolve_valid &&
                ((branch_writeback.instr[6:0] == RISCV_ISA::OP_BRANCH) ||
                 (branch_writeback.fuse_is_branch &&
                  branch_writeback.fuse_branch_instr[6:0] == RISCV_ISA::OP_BRANCH))) begin
            // Same taken test, on the SLAVE branch's pc when the resolve came
            // from a fused master.
            ghr_next = {ghr_next[DIRECT_HISTORY_BITS-2:0],
                (branch_writeback.redirect_pc !=
                    (branch_writeback.fuse_is_branch ?
                        branch_writeback.fuse_branch_pc : branch_writeback.pc) +
                    `ILEN_INC(branch_writeback.fuse_is_branch ?
                        branch_writeback.fuse_is_compressed :
                        branch_writeback.is_compressed))};
        end
`else
        if (branch_restore_valid && branch_resolve_valid &&
                (branch_writeback.instr[6:0] == RISCV_ISA::OP_BRANCH)) begin
            ghr_next = {ghr_next[DIRECT_HISTORY_BITS-2:0],
                (branch_writeback.redirect_pc !=
                    branch_writeback.pc + `ILEN_INC(branch_writeback.is_compressed))};
        end
`endif
        ghr_branch_snapshot = ghr_next;
        branch_active_tail_snapshot = active_tail;
        branch_free_head_snapshot = free_head_snapshot;
        branch_free_tail_snapshot = free_tail_snapshot;
        branch_free_count_snapshot = free_count_snapshot;
        for (int i = 0; i < 32; i += 1) begin
            branch_map_snapshot[i] = map_snapshot[i];
        end

        branch_active_tail_snapshot = active_tail + active_id_t'(dispatch_count);
        for (int i = 0; i < OOO_WIDTH; i += 1) begin
            if (dispatch_valid[i]) begin
                if (decode_lanes[i].ctrl.pc_source == PC_uncond) begin
                    // P1: a JAL / c.j / c.jal is a static direct jump — target is
                    // pc+imm (IMM_UJ) and it is unconditionally taken. Predict it
                    // at dispatch (control_predicted) so it steers fetch here and
                    // never falls into the unpredicted-control freeze (control_pending
                    // + pc-hold). At resolve the ALU compares actual_target (pc+imm)
                    // against predicted_pc (pc+imm) -> never mispredicts, checkpoint
                    // frees cleanly. A call (rd!=x0) additionally pushes the RAS; a
                    // call with a full RAS is still predicted here (just no push).
                    lane_control_predicted[i] = 1'b1;
                    lane_predicted_pc[i] = decode_lanes[i].pc + decode_lanes[i].imm;
                    predictor_redirect_valid = 1'b1;
                    predictor_redirect_pc = decode_lanes[i].pc + decode_lanes[i].imm;
                    btb_branch_pc   = decode_lanes[i].pc;
`ifdef RVC
                    btb_branch_is_c = decode_lanes[i].ctrl.is_compressed;
`else
                    btb_branch_is_c = 1'b0;
`endif
                    if (lane_is_call[i] &&
                            (ras_count_next < RAS_COUNT_BITS'(RAS_DEPTH))) begin
                        ras_stack_next[RAS_INDEX_BITS'(ras_count_next)] =
                            decode_lanes[i].pc +
                            `ILEN_INC(decode_lanes[i].ctrl.is_compressed);
                        ras_count_next = ras_count_next + 1'b1;
                    end
                end else if (lane_is_return[i] && (ras_count_next != '0)) begin
                    lane_control_predicted[i] = 1'b1;
                    lane_predicted_pc[i] =
                        ras_stack_next[RAS_INDEX_BITS'(ras_count_next - 1'b1)];
                    ras_redirect_valid = 1'b1;
                    ras_redirect_pc =
                        ras_stack_next[RAS_INDEX_BITS'(ras_count_next - 1'b1)];
                    ras_count_next = ras_count_next - 1'b1;
                end else if (decode_lanes[i].ctrl.pc_source == PC_cond) begin
                    lane_predictor_info[i] = direct_prediction_info;
                    if (direct_prediction) begin
                        lane_control_predicted[i] = 1'b1;
                        lane_predicted_pc[i] = decode_lanes[i].pc +
                            decode_lanes[i].imm;
                        predictor_redirect_valid = 1'b1;
                        predictor_redirect_pc = decode_lanes[i].pc +
                            decode_lanes[i].imm;
                        btb_branch_pc   = decode_lanes[i].pc;
`ifdef RVC
                        btb_branch_is_c = decode_lanes[i].ctrl.is_compressed;
`else
                        btb_branch_is_c = 1'b0;
`endif
                    end
                end else if ((decode_lanes[i].ctrl.pc_source == PC_indirect) &&
                        !lane_is_return[i]) begin
                    lane_predictor_info[i] = indirect_prediction_info;
                    if (indirect_prediction_valid) begin
                        lane_control_predicted[i] = 1'b1;
                        lane_predicted_pc[i] = indirect_prediction_target;
                        predictor_redirect_valid = 1'b1;
                        predictor_redirect_pc = indirect_prediction_target;
                        btb_branch_pc   = decode_lanes[i].pc;
`ifdef RVC
                        btb_branch_is_c = decode_lanes[i].ctrl.is_compressed;
`else
                        btb_branch_is_c = 1'b0;
`endif
                    end
                end
                if (lane_has_dest[i] && free_alloc_valid[i]) begin
                    branch_map_snapshot[decode_lanes[i].rd] = free_alloc_prd[i];
                    branch_free_head_snapshot = branch_free_head_snapshot + 1'b1;
                    branch_free_count_snapshot = branch_free_count_snapshot - 1'b1;
                end
                if (lane_is_branch[i]) begin
                    branch_active_tail_snapshot = active_tail +
                        active_id_t'(lane_active_offset[i]) + active_id_t'(1);
                    ras_branch_snapshot_count = ras_count_next;
                    ghr_branch_snapshot = ghr_next;
                    if (decode_lanes[i].ctrl.pc_source == PC_cond) begin
                        ghr_next = {ghr_next[DIRECT_HISTORY_BITS-2:0],
                            direct_prediction};
                    end
                    break;
                end
            end
        end
        for (int i = 0; i < OOO_WIDTH; i += 1) begin
            if (dispatch_valid[i] && lane_is_unpredicted_control[i] &&
                    !lane_control_predicted[i]) begin
                dispatched_unpredicted_control = 1'b1;
            end
        end
    end

    // ---- B2c: per-branch RAS/GHR checkpoint write. Split out so B2 no longer
    // reads branch_allocate_valid/id, which closed the false branch_allocate_valid
    // -> branch_free_head_snapshot loop edge (B2 computes the snapshot independently
    // of branch_allocate_*; it only read them for this checkpoint store). The
    // snapshot values come from B2; value-identical. ----
    always_comb begin
        ras_checkpoint_count_next = ras_checkpoint_count_q;
        ghr_checkpoint_next = ghr_checkpoint_q;
        if (branch_allocate_valid) begin
            ras_checkpoint_count_next[branch_allocate_id] = ras_branch_snapshot_count;
            ghr_checkpoint_next[branch_allocate_id] = ghr_branch_snapshot;
        end
    end

    // ---- B3: rename packets + issue-queue insert payloads ----
    always_comb begin
        alloc_req = '0;
        int_insert_valid = '0;
        mem_insert_valid = '0;
        branch_allocate = 1'b0;
        for (int i = 0; i < OOO_WIDTH; i += 1) begin
            alloc_req[i] = dispatch_valid[i] && lane_has_dest[i];
            rename_packets[i] = '0;
            rename_packets[i].valid = dispatch_valid[i];
            rename_packets[i].pc = decode_lanes[i].pc;
            rename_packets[i].instr = decode_lanes[i].instr;
            rename_packets[i].ctrl = decode_lanes[i].ctrl;
            rename_packets[i].rs1 = decode_lanes[i].rs1;
            rename_packets[i].rs2 = decode_lanes[i].rs2;
            rename_packets[i].rd = decode_lanes[i].rd;
            rename_packets[i].prs1 = map_prs1[i];
            rename_packets[i].prs2 = map_prs2[i];
            rename_packets[i].prd = free_alloc_prd[i];
            rename_packets[i].old_prd = map_old_prd[i];
            rename_packets[i].src1_ready = !decode_lanes[i].uses_rs1 ||
                busy_src1_ready[i];
`ifdef FUSE_CMPBR_LI
            // Case B master's rs2 is the folded branch's other source (a real
            // read): gate readiness purely on the busy lookup — the li's own
            // uses_rs2==0 would force src2_ready=1 while rs is still in flight.
            rename_packets[i].src2_ready = fu_caseb[i] ? busy_src2_ready[i] :
                (!decode_lanes[i].uses_rs2 || busy_src2_ready[i]);
`else
            rename_packets[i].src2_ready = !decode_lanes[i].uses_rs2 ||
                busy_src2_ready[i];
`endif
            rename_packets[i].has_dest = map_has_dest[i];
`ifdef FUSE_UADDR
            // FUSE_UADDR (c): surviving load's offset = folded hi+off
            // (bit-exact lo<0 borrow fold — IMM_U[11:0]==0, one XLEN add).
            rename_packets[i].imm = fu_imm_ovr[i] ? fu_imm[i] : decode_lanes[i].imm;
`else
            rename_packets[i].imm = decode_lanes[i].imm;
`endif
            rename_packets[i].branch_mask = dispatch_branch_mask;
`ifdef JAL_NO_CKPT
            // JAL (PC_uncond) allocates no checkpoint, so it carries branch_id 0.
            // Redundant with EDIT 1 (a branch terminates the group => at most one
            // dispatched branch => a JAL group has branch_allocate==0 => id 0 already),
            // but makes EDIT 2's safety independent of group-termination.
            rename_packets[i].branch_id = (lane_is_branch[i] &&
                (decode_lanes[i].ctrl.pc_source != PC_uncond)) ?
                branch_allocate_id : '0;
`else
            rename_packets[i].branch_id = lane_is_branch[i] ?
                branch_allocate_id : '0;
`endif
            rename_packets[i].active_id = active_tail +
                active_id_t'(lane_active_offset[i]);
            rename_packets[i].control_predicted = lane_control_predicted[i];
            rename_packets[i].predicted_pc = lane_predicted_pc[i];
            rename_packets[i].predictor_info = lane_predictor_info[i];
            rename_packets[i].fp_rs1 = decode_lanes[i].rs1;
            rename_packets[i].fp_rs2 = decode_lanes[i].rs2;
            rename_packets[i].fp_rs3 = decode_lanes[i].instr[31:27];
            rename_packets[i].fp_rd = decode_lanes[i].rd;
            rename_packets[i].fp_src1_data = fp_regs_q[decode_lanes[i].rs1];
            rename_packets[i].fp_src2_data = fp_regs_q[decode_lanes[i].rs2];
            rename_packets[i].fp_src3_data =
                fp_regs_q[decode_lanes[i].instr[31:27]];
            rename_packets[i].fu_class = fu_class_for(decode_lanes[i].ctrl);
`ifdef FUSE_ANY
            // Fusion payload (shared-infra §4a/§5): mark the folded slave
            // born-done (active_list completes it at allocate; no IQ entry), and
            // carry the slave's op on the master for its FU to execute folded.
            rename_packets[i].is_fused    = fuse_master[i];
            rename_packets[i].fuse_kind   = fuse_kind_lane[i];
            rename_packets[i].fused_slave = fuse_slave[i];
`ifdef FUSE_LDBR
            rename_packets[i].fuse_slave_ldbr = fuse_slave_ldbr[i];
`endif
`ifdef FUSE_UADDR
            // FUSE_UADDR (c): surviving load's LSQ AGU base select.
            rename_packets[i].use_pc_base = fu_pc_base[i];
`endif
`ifdef FUSE_CMPBR_LI
            // Case B li master: fused branch compares against rs2_data.
            rename_packets[i].fuse_cmp_rs2 = fu_caseb[i];
`endif
            // The slave payload reads lane i+1; a master is never the last lane
            // (the detector pairs ADJACENT lanes 0..OOO_WIDTH-2), so guard the
            // index and leave lane OOO_WIDTH-1's payload at the '0 reset above.
            if (i < OOO_WIDTH-1) begin
                rename_packets[i].fuse_imm    = decode_lanes[i+1].imm;
                rename_packets[i].fuse_alu_op = decode_lanes[i+1].ctrl.alu_op;
`ifdef FUSE_BRANCH
                // Branch-fusion payload: the master hosts the slave branch's
                // resolve (checkpoint id X + the slave's prediction/training id).
                rename_packets[i].fuse_pc_source         = decode_lanes[i+1].ctrl.pc_source; // PC_cond
                rename_packets[i].fuse_branch_id         = branch_allocate_id; // slave's checkpoint X
                rename_packets[i].fuse_control_predicted = lane_control_predicted[i+1];
                rename_packets[i].fuse_predicted_pc      = lane_predicted_pc[i+1];
                rename_packets[i].fuse_branch_pc         = decode_lanes[i+1].pc;
                rename_packets[i].fuse_branch_instr      = decode_lanes[i+1].instr;
                rename_packets[i].fuse_predictor_info    = lane_predictor_info[i+1];
                // Slave branch's ILEN flag (FUSE_CMPBR: fused fall-through +
                // training taken-tests need the slave's ILEN, not the master's).
                rename_packets[i].fuse_is_compressed     =
                    decode_lanes[i+1].ctrl.is_compressed;
`endif
`ifdef FUSE_LDBR
                // The slave branch's OWN ROB slot (distinct from the load
                // master's): the fused resolve retires it via pend_fbr.
                rename_packets[i].fuse_br_active_id =
                    active_tail + active_id_t'(lane_active_offset[i+1]);
`endif
            end
`endif
`ifdef FUSE_BRANCH
            rename_packets[i].fuse_is_branch = fuse_master[i] &&
                (fuse_kind_lane[i] == FUSE_K_CMPBR || fuse_kind_lane[i] == FUSE_K_LDBR);
`endif

            dispatch_issue_entries[i] = '0;
            dispatch_issue_entries[i].valid = dispatch_valid[i];
            dispatch_issue_entries[i].pc = rename_packets[i].pc;
            dispatch_issue_entries[i].instr = rename_packets[i].instr;
            dispatch_issue_entries[i].ctrl = rename_packets[i].ctrl;
            dispatch_issue_entries[i].prs1 = rename_packets[i].prs1;
            dispatch_issue_entries[i].prs2 = rename_packets[i].prs2;
            dispatch_issue_entries[i].prd = rename_packets[i].prd;
            dispatch_issue_entries[i].src1_ready = rename_packets[i].src1_ready;
            dispatch_issue_entries[i].src2_ready = rename_packets[i].src2_ready;
            dispatch_issue_entries[i].has_dest = rename_packets[i].has_dest;
            dispatch_issue_entries[i].imm = rename_packets[i].imm;
            dispatch_issue_entries[i].branch_mask = rename_packets[i].branch_mask;
            dispatch_issue_entries[i].branch_id = rename_packets[i].branch_id;
            dispatch_issue_entries[i].active_id = rename_packets[i].active_id;
            dispatch_issue_entries[i].control_predicted =
                rename_packets[i].control_predicted;
            dispatch_issue_entries[i].predicted_pc = rename_packets[i].predicted_pc;
            dispatch_issue_entries[i].predictor_info =
                rename_packets[i].predictor_info;
            dispatch_issue_entries[i].fp_rs1 = rename_packets[i].fp_rs1;
            dispatch_issue_entries[i].fp_rs2 = rename_packets[i].fp_rs2;
            dispatch_issue_entries[i].fp_rs3 = rename_packets[i].fp_rs3;
            dispatch_issue_entries[i].fp_rd = rename_packets[i].fp_rd;
            dispatch_issue_entries[i].fp_src1_data =
                rename_packets[i].fp_src1_data;
            dispatch_issue_entries[i].fp_src2_data =
                rename_packets[i].fp_src2_data;
            dispatch_issue_entries[i].fp_src3_data =
                rename_packets[i].fp_src3_data;
            dispatch_issue_entries[i].fu_class = rename_packets[i].fu_class;
`ifdef FUSE_ANY
            // Mirror the fusion payload into the master's issue entry (its FU
            // reads the folded op from here).
            dispatch_issue_entries[i].is_fused    = rename_packets[i].is_fused;
            dispatch_issue_entries[i].fuse_kind   = rename_packets[i].fuse_kind;
            dispatch_issue_entries[i].fused_slave = rename_packets[i].fused_slave;
            dispatch_issue_entries[i].fuse_imm    = rename_packets[i].fuse_imm;
            dispatch_issue_entries[i].fuse_alu_op = rename_packets[i].fuse_alu_op;
`endif
`ifdef FUSE_UADDR
            dispatch_issue_entries[i].use_pc_base = rename_packets[i].use_pc_base;
`endif
`ifdef FUSE_CMPBR_LI
            dispatch_issue_entries[i].fuse_cmp_rs2 = rename_packets[i].fuse_cmp_rs2;
`endif
`ifdef FUSE_BRANCH
            dispatch_issue_entries[i].fuse_is_branch = rename_packets[i].fuse_is_branch;
            dispatch_issue_entries[i].fuse_pc_source = rename_packets[i].fuse_pc_source;
            dispatch_issue_entries[i].fuse_branch_id = rename_packets[i].fuse_branch_id;
            dispatch_issue_entries[i].fuse_control_predicted =
                rename_packets[i].fuse_control_predicted;
            dispatch_issue_entries[i].fuse_predicted_pc =
                rename_packets[i].fuse_predicted_pc;
            dispatch_issue_entries[i].fuse_branch_pc = rename_packets[i].fuse_branch_pc;
            dispatch_issue_entries[i].fuse_branch_instr =
                rename_packets[i].fuse_branch_instr;
            dispatch_issue_entries[i].fuse_predictor_info =
                rename_packets[i].fuse_predictor_info;
            dispatch_issue_entries[i].fuse_is_compressed =
                rename_packets[i].fuse_is_compressed;
`endif
`ifdef FUSE_LDBR
            dispatch_issue_entries[i].fuse_br_active_id =
                rename_packets[i].fuse_br_active_id;
`endif

`ifdef FUSE_ANY
            // The folded slave does not execute (no IQ entry; born-done ROB slot).
            int_insert_valid[i] = dispatch_valid[i] && !lane_is_memory[i] &&
                !fuse_slave[i];
            mem_insert_valid[i] = dispatch_valid[i] && lane_is_memory[i] &&
                !fuse_slave[i];
`else
            int_insert_valid[i] = dispatch_valid[i] && !lane_is_memory[i];
            mem_insert_valid[i] = dispatch_valid[i] && lane_is_memory[i];
`endif
`ifdef JAL_NO_CKPT
            // JAL (PC_uncond) is always correctly predicted (predicted_pc==pc+imm==
            // target), so its checkpoint could never be restored -- do not allocate it.
            // lane_is_branch stays true for a JAL (dispatch-group termination + the B2
            // predictor break are unchanged); only the checkpoint allocate is dropped.
            branch_allocate |= dispatch_valid[i] && lane_is_branch[i] &&
                (decode_lanes[i].ctrl.pc_source != PC_uncond);
`else
            branch_allocate |= dispatch_valid[i] && lane_is_branch[i];
`endif
        end
    end

    // ---- B4: in-order commit -- architectural regfile / RRAT update,
    // commit-time trap/return detection, precise-trap flush + rollback ----
    always_comb begin
        commit_exc_valid = 1'b0;
        commit_exc_cause = 5'd0;
        commit_exc_tval = 32'd0;
        commit_trap_epc = 32'd0;
        commit_take_trap = 1'b0;
        commit_take_ret = 1'b0;
        commit_ret_from_s = 1'b0;
        wfi_wait_set = 1'b0;
        arch_rd_we = '0;
        arch_rd = '0;
        arch_rd_data = '0;
        arch_rs1 = '0;
        arch_rs2 = '0;
        for (int i = 0; i < OOO_WIDTH; i += 1) begin
            arch_rd_we[i] = retire_valid[i] && active_commit_packet[i].has_dest;
            arch_rd[i] = active_commit_packet[i].rd;
            arch_rd_data[i] = active_commit_packet[i].data;
        end

        // ---- Commit-time trap / return detection (Phase 3 privileged ISA) ----
        // ecall/ebreak/illegal/csr-illegal raise synchronous exceptions; mret /
        // sret return from a trap. All of these are serializing, so the
        // triggering instruction commits in isolation (any older instruction in
        // a lower lane has already updated architectural state above). The
        // architectural state transition is applied by priv_csr_file, driven by
        // commit_take_trap / commit_take_ret below.
        for (int i = 0; i < OOO_WIDTH; i += 1) begin
            if (retire_valid[i] && !commit_exc_valid && !commit_take_ret &&
                    !active_commit_packet[i].halted) begin
                unique case (active_commit_packet[i].instr)
                    32'h3020_0073: begin                       // MRET
                        // MRET is legal only from M-mode; otherwise it is an
                        // illegal instruction.
                        if (cur_priv != RISCV_Priv::PRIV_M) begin
                            commit_exc_valid = 1'b1;
                            commit_exc_cause = RISCV_Priv::EXC_ILLEGAL_INSTR;
                            commit_exc_tval  = active_commit_packet[i].instr;
                            commit_trap_epc  = active_commit_packet[i].pc;
                        end else begin
                            commit_take_ret   = 1'b1;
                            commit_ret_from_s = 1'b0;
                            commit_trap_epc   = active_commit_packet[i].pc;
                        end
                    end
                    32'h1020_0073: begin                       // SRET
                        // SRET is illegal from U-mode, and illegal from S-mode
                        // when mstatus.TSR=1 (Trap SRET). M-mode may always SRET.
                        if ((cur_priv == RISCV_Priv::PRIV_U) ||
                            ((cur_priv == RISCV_Priv::PRIV_S) &&
                             csr_mstatus[RISCV_Priv::MSTATUS_TSR_BIT])) begin
                            commit_exc_valid = 1'b1;
                            commit_exc_cause = RISCV_Priv::EXC_ILLEGAL_INSTR;
                            commit_exc_tval  = active_commit_packet[i].instr;
                            commit_trap_epc  = active_commit_packet[i].pc;
                        end else begin
                            commit_take_ret   = 1'b1;
                            commit_ret_from_s = 1'b1;
                            commit_trap_epc   = active_commit_packet[i].pc;
                        end
                    end
                    32'h0000_0073: begin                       // ECALL
                        commit_exc_valid = 1'b1;
                        commit_exc_cause = (cur_priv == RISCV_Priv::PRIV_M) ?
                                RISCV_Priv::EXC_ECALL_M :
                            (cur_priv == RISCV_Priv::PRIV_S) ?
                                RISCV_Priv::EXC_ECALL_S : RISCV_Priv::EXC_ECALL_U;
                        commit_trap_epc  = active_commit_packet[i].pc;
                    end
                    32'h0010_0073: begin                       // EBREAK
                        commit_exc_valid = 1'b1;
                        commit_exc_cause = RISCV_Priv::EXC_BREAKPOINT;
                        commit_trap_epc  = active_commit_packet[i].pc;
                        // A breakpoint exception reports the PC of the EBREAK in
                        // m/stval (unlike most synchronous exceptions, which
                        // report zero or a faulting data address).
                        commit_exc_tval  = active_commit_packet[i].pc;
                    end
                    32'h1050_0073: begin                       // WFI
                        // mstatus.TW=1 makes WFI illegal in any less-privileged
                        // mode (S or U); with TW=0 it is legal and simply waits.
                        // M-mode may always WFI.
                        if ((cur_priv != RISCV_Priv::PRIV_M) &&
                            csr_mstatus[RISCV_Priv::MSTATUS_TW_BIT]) begin
                            commit_exc_valid = 1'b1;
                            commit_exc_cause = RISCV_Priv::EXC_ILLEGAL_INSTR;
                            commit_exc_tval  = active_commit_packet[i].instr;
                            commit_trap_epc  = active_commit_packet[i].pc;
                        end else if (!wfi_wake) begin
                            // Legal WFI, no enabled interrupt pending yet: idle
                            // the frontend until one arrives. (mret/sret/ecall
                            // semantics are unaffected; WFI itself retires here.)
                            wfi_wait_set = 1'b1;
                        end
                    end
                    default: begin
                        // SFENCE.VMA is illegal from U-mode, and illegal from
                        // S-mode when mstatus.TVM=1 (Trap Virtual Memory). It is
                        // encoded as SYSTEM / funct3=000 / funct7=0001001.
                        if ((active_commit_packet[i].instr[6:0] ==
                                 RISCV_ISA::OP_SYSTEM) &&
                            (active_commit_packet[i].instr[14:12] == 3'b000) &&
                            (active_commit_packet[i].instr[31:25] == 7'b0001001) &&
                            ((cur_priv == RISCV_Priv::PRIV_U) ||
                             ((cur_priv == RISCV_Priv::PRIV_S) &&
                              csr_mstatus[RISCV_Priv::MSTATUS_TVM_BIT]))) begin
                            commit_exc_valid = 1'b1;
                            commit_exc_cause = RISCV_Priv::EXC_ILLEGAL_INSTR;
                            commit_exc_tval  = active_commit_packet[i].instr;
                            commit_trap_epc  = active_commit_packet[i].pc;
                        end else if (active_commit_packet[i].exception) begin
                            commit_exc_valid = 1'b1;
                            commit_exc_cause = active_commit_packet[i].exc_cause;
                            // Illegal-instruction faults report the instruction in
                            // mtval; memory faults report the faulting address,
                            // which the LSQ/fetch placed in the commit data field.
                            // RV64C: an illegal COMPRESSED op reports the 16-bit
                            // parcel (zero-extended), not the expanded 32-bit word
                            // (.instr must stay canonical -- it is a live CSR/
                            // fence/predictor decode input).
                            commit_exc_tval  =
                                (active_commit_packet[i].exc_cause ==
                                    RISCV_Priv::EXC_ILLEGAL_INSTR) ?
`ifdef RVC
                                    (active_commit_packet[i].is_compressed ?
                                        {{(XLEN-16){1'b0}},
                                         active_commit_packet[i].rvc_parcel} :
                                        active_commit_packet[i].instr) :
`else
                                        active_commit_packet[i].instr :
`endif
                                        active_commit_packet[i].data;
                            commit_trap_epc  = active_commit_packet[i].pc;
                        end
                    end
                endcase
                if (commit_exc_valid || commit_take_ret) begin
                    arch_rd_we[i] = 1'b0;
                end
            end
        end
        commit_take_trap = commit_exc_valid;

        // ---- Precise-trap full flush + architectural rollback (Phase 3c) ----
        // A committed exception is taken precisely: every younger in-flight
        // instruction is squashed this cycle (active list / issue queues / LSQ /
        // branch stack / multi-cycle FUs all see `trap_take`) and the speculative
        // rename map and free-list head are restored to the committed (RRAT)
        // state. The faulting instruction does not architecturally complete --
        // arch_rd_we is already cleared for it above -- so it neither updates the
        // RRAT nor advances the architectural free-list head, and the physical
        // register it speculatively allocated is reclaimed by the head rollback.
        // Interrupts already wait for an empty ROB and returns are serializing,
        // so only exceptions need this; serializing exceptions simply see an
        // empty younger window and the rollback is a no-op.
        trap_take = commit_take_trap;

        arch_free_head_next = arch_free_head_q;
        for (int i = 0; i < 32; i += 1) begin
            arch_map_next[i] = arch_map_q[i];
        end
        for (int i = 0; i < OOO_WIDTH; i += 1) begin
            if (arch_rd_we[i]) begin
                arch_map_next[arch_rd[i]] = active_commit_packet[i].prd;
                arch_free_head_next = arch_free_head_next + 1'b1;
            end
        end

        // Restore muxes: a trap flush rolls back to the architectural state and
        // takes priority over a branch misprediction recovery (the mispredicting
        // branch is necessarily younger than the trapping head, so it is part of
        // the squashed window).
        map_restore_valid  = trap_take || branch_restore_valid;
        free_restore_valid = trap_take || branch_restore_valid;
        free_restore_head  = trap_take ? arch_free_head_next :
                                         branch_restore_free_head;
        for (int i = 0; i < 32; i += 1) begin
            map_restore_map[i] = trap_take ? arch_map_next[i] :
                                             branch_restore_map[i];
        end
    end

    // ---- B5: precise interrupt drain (ROB-empty) + WFI idle FSM ----
    always_comb begin
        commit_take_int = 1'b0;
        commit_int_epc = 32'd0;
        irq_drain_next = irq_drain_q;

        // ---- Precise interrupt handling via ROB drain (Phase 3b) ----
        // Mirror trap_controller's interrupt-enable evaluation so we can stop
        // dispatch the moment an interrupt becomes deliverable.
        irq_eff = csr_mip & csr_mie;
        m_irq_en = (cur_priv != RISCV_Priv::PRIV_M) ||
            csr_mstatus[RISCV_Priv::MSTATUS_MIE_BIT];
        s_irq_en = (cur_priv == RISCV_Priv::PRIV_U) ||
            ((cur_priv == RISCV_Priv::PRIV_S) &&
             csr_mstatus[RISCV_Priv::MSTATUS_SIE_BIT]);
        irq_pending_now =
            (m_irq_en && ((irq_eff & ~csr_mideleg) != 32'b0)) ||
            (s_irq_en && (( irq_eff &  csr_mideleg) != 32'b0));

        // Take the interrupt only once the machine has drained to a precise
        // point (ROB empty) and the next architectural instruction is known
        // (a valid, un-dispatched decode lane). epc is that instruction's PC.
        // Once drained (ROB empty) the interrupt is precise. epc is the oldest
        // instruction still in the frozen frontend (decode is oldest, then the
        // fetch pipeline, then pc_q if the frontend is completely empty).
        if (irq_drain_q && irq_pending_now && active_empty &&
                !commit_take_trap && !commit_take_ret && !fencei_block) begin
            commit_take_int = 1'b1;
            // epc = the oldest unexecuted instruction in the frozen frontend:
            // the presented group (decode is oldest), then the second buffered
            // group, then any still-outstanding fetch (oldest-first), then
            // pc_q if the frontend is completely empty.
            commit_int_epc  =
`ifdef RVC
                // RV64C: the realigner's oldest un-dispatched PC covers a pending
                // straddle completion and a partially-drained head block (a
                // block-aligned fgrp_pc would skip a mid-block instruction).
                rvc_oldest_valid ? rvc_oldest_pc :
`else
                lane_valid[0] ? decode_lanes[0].pc :
                fgrp_valid    ? fgrp_pc :
`endif
                (fbuf_cnt_q == 2'd2) ? fbuf_q[1].pc :
                (fmeta_q[fmeta_rd_q].valid && !fmeta_q[fmeta_rd_q].kill) ?
                    fmeta_q[fmeta_rd_q].pc :
                (fmeta_q[~fmeta_rd_q].valid && !fmeta_q[~fmeta_rd_q].kill) ?
                    fmeta_q[~fmeta_rd_q].pc :
                pc_q;
        end

        // Drain FSM: enter on a pending interrupt, leave once it is taken (or it
        // is no longer deliverable, e.g. software cleared mie before draining).
        if (halted_q || commit_take_int) begin
            irq_drain_next = 1'b0;
        end else if (irq_pending_now) begin
            irq_drain_next = 1'b1;
        end else begin
            irq_drain_next = 1'b0;
        end

        // WFI idle FSM: set when a legal WFI retires with no enabled interrupt
        // pending; cleared once an enabled interrupt arrives (wfi_wake) or on any
        // pipeline flush (trap/interrupt/branch redirect). While set, dispatch is
        // suppressed so the core idles until woken.
        wfi_wait_next = wfi_wait_q;
        if (wfi_wait_set) wfi_wait_next = 1'b1;
        if (wfi_wake || trap_take || halted_q) wfi_wait_next = 1'b0;
    end

    // ---- B6: data-memory port request drive ----
    always_comb begin
        // Drive the data port request. Apply Sv32 translation at the memory
        // port: the LSQ works in virtual addresses; the physical word address
        // is computed by the MMU above. The store write beats are the
        // exception: the LSQ already drives the captured physical word
        // address, so bypass the (stale, head-VA based) translation mux.
        dmem_req_valid = mem_data_load_en || (mem_data_store_mask != '0);
        dmem_req_write = (mem_data_store_mask != '0);
        dmem_req_addr = (paging_data && !lsq_store_second_beat) ? data_pa[XLEN-1:ADDR_SHIFT]
                                                                : mem_data_addr;
        dmem_req_wdata = mem_data_store;
        dmem_req_wmask = mem_data_store_mask;
        dmem_req_op    = mem_dmem_op;
        dmem_req_amo   = mem_dmem_amo;
    end

    // ---- B7: frontend control + commit-driven redirects (pc_next,
    // fetch flush/consume/issue, ifetch invalidate, pending interlocks,
    // CSR/FP commit writes, fence.i / sfence / satp / trap redirects) ----
    always_comb begin
        fp_regs_next = fp_regs_q;
        serial_pending_next = serial_pending_q;
        retire_count = '0;
        csr_commit_write = 1'b0;
        csr_commit_addr = '0;
        csr_commit_wdata = '0;
        csr_fp_fflags_valid = 1'b0;
        csr_fp_fflags = '0;
        tlb_flush = 1'b0;

        pc_next = pc_q;
        fetch_flush = 1'b0;
`ifdef DISPATCH_STATS
        // Last-writer-wins: the redirect chain below is sequential-override, so whichever
        // arm assigns ff_src last is the one that actually determined pc_next. FF_OTHER
        // catches a fetch_flush raised by a site this instrumentation does not know about
        // -- it is a REACHABLE catch-all, so a nonzero ff_other means a missed setter.
        ff_src = FF_OTHER;
`endif
        fetch_consume = 1'b0;
        fetch_issue = 1'b0;
        ifetch_inval = 1'b0;
`ifdef NIIGO_DSIDE_WB
        fencei_pending_next = fencei_pending_q;
        fencei_pc_next = fencei_pc_q;
`endif
        // Synchronous exceptions now redirect to a trap handler instead of
        // halting the core (see commit-time trap detection above). precise_halt
        // (ecall a0=10/11) remains the simulation's clean stop condition.
        halted_next = halted_q || precise_halt;
        terminal_pending_next = terminal_pending_q;
        control_pending_next = control_pending_q;
        control_pending_id_next = control_pending_id_q;
        if (branch_resolve_valid && control_pending_q &&
                ((branch_resolve_id == control_pending_id_q) ||
                 abort_mask[control_pending_id_q])) begin
            control_pending_next = 1'b0;
            control_pending_id_next = '0;
        end
        if (redirect_valid || precise_halt || precise_exception) begin
            terminal_pending_next = 1'b0;
        end
        if (redirect_valid) begin
            serial_pending_next = 1'b0;
        end
        // A precise-trap flush squashes every younger in-flight instruction, so
        // any frontend "pending" interlock those squashed instructions raised
        // (a not-yet-committed serializing op, an unresolved unpredicted control
        // transfer, or a pending terminal ecall) must be released here -- the
        // instruction that would have cleared it no longer exists.
        if (trap_take) begin
            serial_pending_next = 1'b0;
            control_pending_next = 1'b0;
            control_pending_id_next = '0;
            terminal_pending_next = 1'b0;
        end
        for (int i = 0; i < OOO_WIDTH; i += 1) begin
            if (active_commit_valid[i] &&
                    (active_commit_packet[i].instr == 32'h0000_0073)) begin
                terminal_pending_next = 1'b0;
            end
        end

        if (redirect_valid) begin
            // Branch recovery: every outstanding/buffered fetch is wrong-path.
            pc_next = redirect_pc;
            fetch_flush = 1'b1;
            `ifdef DISPATCH_STATS
            ff_src = FF_REDIRECT;
            `endif
        end else begin
            // Group consumption and dispatch-time prediction redirects. Gated
            // on !dispatch_stall (zero lanes dispatch under a stall, so the
            // presented group must be held and re-presented). Deliberately NOT
            // gated on fetch_xlate_stall: consumption is an explicit event
            // here, so a group that fully dispatches while pc_q's translation
            // is still walking leaves the buffer and cannot dispatch twice
            // (the old frozen-pipe scheme would re-present it).
            if (!dispatch_stall && !halted_q) begin
                // Returns (ras) always redirect -- the BTB does not handle them.
                // !btb_suppress: when the BTB already steered fetch to a
                // block-ending predictor-taken target, skip the redirect+flush
                // (the target is the next block in the stream), fall to consume.
                if (ras_redirect_valid) begin
                    pc_next = ras_redirect_pc;
                    fetch_flush = 1'b1;
                    `ifdef DISPATCH_STATS
                    ff_src = FF_RAS;
                    `endif
                end else if (predictor_redirect_valid && !btb_suppress) begin
                    pc_next = predictor_redirect_pc;
                    fetch_flush = 1'b1;
                    `ifdef DISPATCH_STATS
                    ff_src = FF_PRED;
                    `endif
                end else if (dispatched_unpredicted_control) begin
                    // Unpredicted JAL/JALR dispatched: younger fetches are
                    // dead; pc holds until the control transfer resolves.
                    fetch_flush = 1'b1;
                    `ifdef DISPATCH_STATS
                    ff_src = FF_UNPRED;
                    `endif
`ifdef BTB
                end else if (btb_mis_steer) begin
                    // BTB steered wrong: flush the wrongly steered target stream and
                    // redirect to the recovered PC (fall-through of a not-taken
                    // terminator, or base+16 / base+14 for an aliased/stale steer).
                    pc_next = btb_mis_recover_pc;
                    fetch_flush = 1'b1;
                    `ifdef DISPATCH_STATS
                    ff_src = FF_BTBMIS;
                    `endif
`endif
`ifdef RVC
                end else begin
                    // RV64C: rvc_realign drains the 16-byte group over multiple
                    // cycles and pops the group (fetch_consume) only once every
                    // in-block parcel is drained or the block tail-straddles.
                    // Partial dispatch HOLDS the group (align_ptr advances inside
                    // the realigner) -- no pc rewind, no flush, no double-decode.
                    // Use the flush-INDEPENDENT rvc_block_drained (not rvc_consume_block,
                    // which gates on !fetch_flush): here !fetch_flush and !frontend_hold
                    // are already implied (no redirect arm fired; inside !dispatch_stall
                    // && !halted_q), so this equals rvc_consume_block exactly, but reading
                    // the flush-independent signal breaks the fetch_flush -> rvc_consume_block
                    // -> this-always -> fetch_flush UNOPTFLAT combinational loop.
                    fetch_consume = fgrp_valid && rvc_block_drained;
                end
`else
                end else if (fgrp_valid && (dispatch_count < valid_count)) begin
                    // Partial dispatch: discard the group and refetch from the
                    // first undispatched instruction.
                    pc_next = decode_lanes[2'(dispatch_count)].pc;
                    fetch_flush = 1'b1;
                    `ifdef DISPATCH_STATS
                    ff_src = FF_PARTIAL;
                    `endif
                    fetch_consume = 1'b1;
                end else if (fgrp_valid) begin
                    // Fully dispatched (or no decodable lanes): pop it.
                    fetch_consume = 1'b1;
                end
`endif
            end
            // Fetch issue: one 16-byte block per cycle while the frontend is
            // unstalled, translation for pc_q is resolved (fetch_xlate_stall
            // is part of frontend_stall), and there is space for the response
            // (outstanding requests + buffered groups bounded by FETCH_DEPTH,
            // crediting a group consumed this very cycle -- without that
            // credit, back-to-back streaming would stall one cycle in three).
            // A same-cycle commit redirect (fence.i/sfence/satp/trap, applied
            // below) suppresses the request via fetch_fire's !fetch_flush.
            if (!frontend_stall && !halted_q && rst_l && !fetch_flush &&
                    ifetch_req_ready &&
                    (({1'b0, fmeta_cnt_q} + {1'b0, fbuf_cnt_q}
                      - {2'b0, fetch_consume}) < 3'(FETCH_DEPTH))) begin
                fetch_issue = 1'b1;
`ifdef BTB
                // A BTB hit for pc_q's block steers the next fetch to the target
                // (verified at decode); otherwise fetch continues sequentially.
                pc_next = btb_hit_now ? btb_pred_target : sequential_next_pc;
`else
                pc_next = sequential_next_pc;
`endif
            end
        end

        for (int i = 0; i < OOO_WIDTH; i += 1) begin
            if (dispatch_valid[i] && lane_is_terminal[i] && !redirect_valid) begin
                terminal_pending_next = 1'b1;
            end
            if (dispatch_valid[i] && decode_lanes[i].ctrl.serializing) begin
                serial_pending_next = 1'b1;
            end
            if (dispatch_valid[i] && lane_is_unpredicted_control[i] &&
                    !lane_control_predicted[i]) begin
                control_pending_next = 1'b1;
                control_pending_id_next = branch_allocate_id;
            end
        end

        for (int i = 0; i < OOO_WIDTH; i += 1) begin
            if (retire_valid[i]) begin
                retire_count = retire_count + 3'd1;
                if (active_commit_packet[i].fp_write) begin
                    fp_regs_next[active_commit_packet[i].fp_rd] =
                        active_commit_packet[i].fp_data;
                end
                if (!csr_commit_write && active_commit_packet[i].csr_write &&
                        !active_commit_packet[i].exception) begin
                    csr_commit_write = 1'b1;
                    csr_commit_addr = active_commit_packet[i].csr_addr;
                    csr_commit_wdata = active_commit_packet[i].csr_wdata;
                end
                if (active_commit_packet[i].fp_fflags_valid) begin
                    csr_fp_fflags_valid = 1'b1;
                    csr_fp_fflags |= active_commit_packet[i].fp_fflags;
                end
                if (active_commit_packet[i].serializing) begin
                    serial_pending_next = 1'b0;
                end
                if ((active_commit_packet[i].instr[6:0] == RISCV_ISA::OP_MISC_MEM) &&
                        (active_commit_packet[i].instr[14:12] == 3'b001)) begin
`ifdef NIIGO_DSIDE_WB
                    // Defer the redirect: first write back the L1D (the modified
                    // code may be dirty there), then invalidate the L1I and
                    // refetch (handled in the fence.i-hold block below).
                    if (!fencei_pending_q) begin
                        fencei_pending_next = 1'b1;
                        fencei_pc_next = active_commit_packet[i].pc + 32'd4;
                    end
`else
                    pc_next = active_commit_packet[i].pc + 32'd4;
                    fetch_flush = 1'b1;
                    `ifdef DISPATCH_STATS
                    ff_src = FF_FENCEI;
                    `endif
                    // Flash-invalidate the L1I so the refetch past this fence.i
                    // sees instruction memory as modified by prior stores. The
                    // refetch is naturally >= 1 cycle later (redirect to pc+4),
                    // by which point the 1-cycle invalidate has applied.
                    ifetch_inval = 1'b1;
`endif
                end
                // SFENCE.VMA: flush both TLBs (modeled as a full flush) and
                // refetch the next instruction so younger fetches re-translate
                // against the new page tables. SFENCE.VMA is serializing, so it
                // commits in isolation and nothing younger is in flight.
                if ((active_commit_packet[i].instr[6:0] == RISCV_ISA::OP_SYSTEM) &&
                        (active_commit_packet[i].instr[14:12] == 3'b000) &&
                        (active_commit_packet[i].instr[31:25] == 7'b0001001) &&
                        !commit_take_trap) begin
                    tlb_flush = 1'b1;
                    pc_next = active_commit_packet[i].pc + 32'd4;
                    fetch_flush = 1'b1;
                    `ifdef DISPATCH_STATS
                    ff_src = FF_SFENCE;
                    `endif
                end
                // A satp write switches address space; flush both TLBs and
                // refetch pc+4 so younger fetches re-translate, without needing a
                // separate SFENCE.VMA. The CSR write is serializing (nothing
                // younger is in flight), so a fetch-pipe flush is sufficient.
                if (active_commit_packet[i].csr_write &&
                        (active_commit_packet[i].csr_addr == RISCV_Priv::CSR_SATP) &&
                        !active_commit_packet[i].exception && !commit_take_trap) begin
                    tlb_flush = 1'b1;
                    pc_next = active_commit_packet[i].pc + 32'd4;
                    fetch_flush = 1'b1;
                    `ifdef DISPATCH_STATS
                    ff_src = FF_SATP;
                    `endif
                end
            end
        end

        // Trap / return redirect overrides any sequential or fence redirect.
        // The trapping/returning instruction is serializing, so no younger
        // speculative work is in flight; flushing the fetch pipe is sufficient.
        trap_redirect_pc = commit_take_ret ?
            (commit_ret_from_s ? csr_sepc : csr_mepc) : tc_vector;
        if (commit_take_trap || commit_take_int || commit_take_ret) begin
            pc_next = trap_redirect_pc;
            fetch_flush = 1'b1;
            `ifdef DISPATCH_STATS
            ff_src = FF_TRAP;
            `endif
        end

`ifdef NIIGO_DSIDE_WB
        // fence.i L1D-writeback hold (highest priority while active; fence.i is
        // serializing and interrupts are gated above, so nothing competes).
        // Hold pc + keep the fetch pipe flushed until the memsys finishes
        // writing back every dirty L1D line, then invalidate the L1I and redirect
        // to the instruction after the fence.i.
        if (fencei_pending_q) begin
            fetch_flush = 1'b1;
            `ifdef DISPATCH_STATS
            ff_src = FF_FENCEI_HOLD;
            `endif
            if (dcache_flush_done) begin
                pc_next = fencei_pc_q;
                ifetch_inval = 1'b1;
                fencei_pending_next = 1'b0;
            end else begin
                pc_next = pc_q;
            end
        end
`endif
    end

    // ---- Fetch-side next-state (metadata FIFO + group buffer) ----
    // Driven by the strobes computed above: a response pops its metadata and
    // (if live) lands in the group buffer or is presented by bypass; a consume
    // pops the presented group; an issued fetch appends metadata; a flush
    // kills every outstanding request and empties the buffer.
    always_comb begin
        logic [1:0] bcnt;
        logic       bypass_consumed;
        fmeta_next = fmeta_q;
        fmeta_rd_next = fmeta_rd_q;
        fmeta_wr_next = fmeta_wr_q;
        fmeta_cnt_next = fmeta_cnt_q;
        fbuf_next = fbuf_q;
        bcnt = fbuf_cnt_q;

        // 1. Response arrival pops the oldest metadata entry.
        if (fresp_take) begin
            fmeta_next[fmeta_rd_q].valid = 1'b0;
            fmeta_rd_next = ~fmeta_rd_q;
            fmeta_cnt_next = fmeta_cnt_q - 2'd1;
        end

        // 2. Consume the presented group: pop the buffer head, or absorb the
        //    bypassed response without ever buffering it.
        bypass_consumed = 1'b0;
        if (fetch_consume) begin
            if (fbuf_cnt_q != 2'd0) begin
                fbuf_next[0] = fbuf_q[1];
                fbuf_next[1] = '0;
                bcnt = bcnt - 2'd1;
            end else begin
                bypass_consumed = 1'b1;
            end
        end

        // 3. A live, unconsumed response lands in the buffer.
        if (fresp_live && !bypass_consumed) begin
            fbuf_next[bcnt[0]].valid = 1'b1;
            fbuf_next[bcnt[0]].pc = fmeta_head.pc;
            fbuf_next[bcnt[0]].data = ifetch_resp_data;
            fbuf_next[bcnt[0]].excpt = ifetch_resp_excpt;
            fbuf_next[bcnt[0]].fault_lane = fmeta_head.fault_lane;
            fbuf_next[bcnt[0]].fault_cause = fmeta_head.fault_cause;
`ifdef BTB
            fbuf_next[bcnt[0]].btb_hit = fmeta_head.btb_hit;
            fbuf_next[bcnt[0]].btb_tgt = fmeta_head.btb_tgt;
            fbuf_next[bcnt[0]].btb_off = fmeta_head.btb_off;
`endif
            bcnt = bcnt + 2'd1;
        end

        // 4. An issued fetch appends metadata (fetch_fire excludes same-cycle
        //    flushes, so a fresh entry is never created just to be killed).
        if (fetch_fire) begin
            fmeta_next[fmeta_wr_q].valid = 1'b1;
            fmeta_next[fmeta_wr_q].kill = 1'b0;
            fmeta_next[fmeta_wr_q].pc = pc_q;
            fmeta_next[fmeta_wr_q].fault_lane = fetch_fault_lane;
            fmeta_next[fmeta_wr_q].fault_cause = fetch_fault_cause;
`ifdef BTB
            // Tag the block with the steer decided this cycle (btb_hit_now =>
            // the next fetch was steered to btb_pred_target). btb_pred_offset is
            // the terminating parcel the realigner will stop this block at.
            fmeta_next[fmeta_wr_q].btb_hit = btb_hit_now;
            fmeta_next[fmeta_wr_q].btb_tgt = btb_pred_target;
            fmeta_next[fmeta_wr_q].btb_off = btb_pred_offset;
`endif
            fmeta_wr_next = ~fmeta_wr_q;
            fmeta_cnt_next = fmeta_cnt_next + 2'd1;
        end

        // 5. Flush: mark every still-outstanding request killed (its response
        //    is discarded on arrival) and empty the group buffer.
        if (fetch_flush) begin
            for (int i = 0; i < FETCH_DEPTH; i += 1) begin
                if (fmeta_next[i].valid) begin
                    fmeta_next[i].kill = 1'b1;
                end
            end
            for (int i = 0; i < 2; i += 1) begin
                fbuf_next[i] = '0;
            end
            bcnt = 2'd0;
        end

        fbuf_cnt_next = bcnt;
    end

    always_ff @(posedge clk or negedge rst_l) begin
        if (!rst_l) begin
            pc_q <= USER_TEXT_START;
            for (int i = 0; i < FETCH_DEPTH; i += 1) begin
                fmeta_q[i] <= '0;
            end
            fmeta_rd_q <= 1'b0;
            fmeta_wr_q <= 1'b0;
            fmeta_cnt_q <= '0;
            for (int i = 0; i < 2; i += 1) begin
                fbuf_q[i] <= '0;
            end
            fbuf_cnt_q <= '0;
            halted_q <= 1'b0;
            terminal_pending_q <= 1'b0;
            control_pending_q <= 1'b0;
            control_pending_id_q <= '0;
            serial_pending_q <= 1'b0;
            irq_drain_q <= 1'b0;
            wfi_wait_q <= 1'b0;
`ifdef NIIGO_DSIDE_WB
            fencei_pending_q <= 1'b0;
            fencei_pc_q <= '0;
`endif
            abort_mask_q <= '0;
            ras_count_q <= '0;
            for (int i = 0; i < FP_REGS; i += 1) begin
                fp_regs_q[i] <= '0;
            end
            for (int i = 0; i < RAS_DEPTH; i += 1) begin
                ras_stack_q[i] <= '0;
            end
            for (int i = 0; i < BRANCH_STACK_SIZE; i += 1) begin
                ras_checkpoint_count_q[i] <= '0;
            end
            ghr_q <= '0;
            pred_ready_q <= 1'b0;
            pred_key_pc_q <= '0;
            pred_key_ghr_q <= '0;
            for (int i = 0; i < BRANCH_STACK_SIZE; i += 1) begin
                ghr_checkpoint_q[i] <= '0;
            end
            // RRAT / architectural free-list head mirror the rename map table and
            // free list reset state (identity map; head at 0).
            for (int i = 0; i < 32; i += 1) begin
                arch_map_q[i] <= phys_reg_t'(i);
            end
            arch_free_head_q <= '0;
        end else begin
            pc_q <= pc_next;
            fmeta_q <= fmeta_next;
            fmeta_rd_q <= fmeta_rd_next;
            fmeta_wr_q <= fmeta_wr_next;
            fmeta_cnt_q <= fmeta_cnt_next;
            fbuf_q <= fbuf_next;
            fbuf_cnt_q <= fbuf_cnt_next;
            halted_q <= halted_next;
            terminal_pending_q <= terminal_pending_next;
            control_pending_q <= control_pending_next;
            control_pending_id_q <= control_pending_id_next;
            serial_pending_q <= serial_pending_next;
            irq_drain_q <= irq_drain_next;
            wfi_wait_q <= wfi_wait_next;
`ifdef NIIGO_DSIDE_WB
            fencei_pending_q <= fencei_pending_next;
            fencei_pc_q <= fencei_pc_next;
`endif
            abort_mask_q <= abort_mask;
            ras_count_q <= ras_count_next;
            for (int i = 0; i < RAS_DEPTH; i += 1) begin
                ras_stack_q[i] <= ras_stack_next[i];
            end
            for (int i = 0; i < BRANCH_STACK_SIZE; i += 1) begin
                ras_checkpoint_count_q[i] <= ras_checkpoint_count_next[i];
            end
            ghr_q <= ghr_next;
            // Predictor sync-read key: remember the group/history the in-flight
            // lookup was launched for, so next cycle prediction_ready can confirm
            // the registered prediction matches the (still-held) group. Cleared on
            // any redirect (the held group is discarded) as belt-and-suspenders.
            pred_ready_q <= redirect_valid ? 1'b0 : pred_launch;
            pred_key_pc_q <= fgrp_pc;
            pred_key_ghr_q <= ghr_q;
            for (int i = 0; i < 32; i += 1) begin
                arch_map_q[i] <= arch_map_next[i];
            end
            arch_free_head_q <= arch_free_head_next;
            for (int i = 0; i < BRANCH_STACK_SIZE; i += 1) begin
                ghr_checkpoint_q[i] <= ghr_checkpoint_next[i];
            end
            for (int i = 0; i < FP_REGS; i += 1) begin
                fp_regs_q[i] <= fp_regs_next[i];
            end
        end
    end

`ifdef FP_OOO
    // P5b FPR scoreboard next-state. Priority (last write wins): age every
    // producer mask by ~reset_mask; clear a busy bit whose (aged) producer mask
    // intersects abort_mask (wrong-path squash) or whose producer retires (commit
    // fp_write -> fp_rd); clear-all on a precise-trap flush; then a fresh
    // dispatch-set re-arms. The WAW dispatch-stall guarantees one producer per
    // FPR, so retire-by-fp_rd and mask-abort are unambiguous.
    always_comb begin
        fpr_busy_next = fpr_busy_q;
        for (int x = 0; x < FP_REGS; x += 1) begin
            fpr_prod_mask_next[x] = fpr_prod_mask_q[x] & ~reset_mask;
            if (fpr_busy_q[x] && ((fpr_prod_mask_q[x] & abort_mask) != '0)) begin
                fpr_busy_next[x] = 1'b0;
            end
        end
        for (int i = 0; i < OOO_WIDTH; i += 1) begin
            if (retire_valid[i] && active_commit_packet[i].fp_write) begin
                fpr_busy_next[active_commit_packet[i].fp_rd] = 1'b0;
            end
        end
        if (trap_take) begin
            fpr_busy_next = '0;
        end
        for (int i = 0; i < OOO_WIDTH; i += 1) begin
            if (dispatch_valid[i] && decode_lanes[i].ctrl.fp_writes_fpr) begin
                fpr_busy_next[decode_lanes[i].rd] = 1'b1;
                // dispatch_branch_mask is already & ~reset_mask & ~abort_mask.
                fpr_prod_mask_next[decode_lanes[i].rd] = dispatch_branch_mask;
            end
        end
    end

    always_ff @(posedge clk or negedge rst_l) begin
        if (!rst_l) begin
            fpr_busy_q <= '0;
            for (int x = 0; x < FP_REGS; x += 1) fpr_prod_mask_q[x] <= '0;
        end else begin
            fpr_busy_q <= fpr_busy_next;
            for (int x = 0; x < FP_REGS; x += 1)
                fpr_prod_mask_q[x] <= fpr_prod_mask_next[x];
        end
    end
`endif


`ifdef SIMULATION_18447
    localparam int PERF_STALL_BUCKETS = 8;
    localparam int PERF_STALL_BITS = $clog2(PERF_STALL_BUCKETS);

    logic [63:0] perf_cycle_counter;
    logic [63:0] perf_dispatch_counter;
    logic [63:0] perf_retire_counter;
    logic [63:0] perf_frontend_stall_cycles;
    logic [63:0] perf_branch_instructions;
    logic [63:0] perf_mispredicted_branches;
    logic [63:0] perf_alu_instructions;
    logic [63:0] perf_load_instructions;
    logic [63:0] perf_store_instructions;
`ifdef FUSE_UADDR
    // FUSE_UADDR engagement counters (fuse-uaddr.md §6): dispatched fused pairs
    // by class — (a)/(b) master fires, (c) surviving pc-base load fires.
    logic [63:0] perf_fu_fire_ab;
    logic [63:0] perf_fu_fire_c;
`endif
`ifdef FUSE_CMPBR
    // FUSE_CMPBR engagement counter (fuse-cmpbr.md §9): dispatched fused
    // compute+branch pairs (master fires; spec predicts ~8/Dhrystone iter).
    logic [63:0] perf_fu_fire_cmpbr;
`endif
`ifdef FUSE_LDBR
    // FUSE_LDBR engagement counter (fuse-ldbr.md §10): dispatched fused
    // load->branch pairs (load master fires; spec predicts ~6-9/Dhrystone iter).
    logic [63:0] perf_fu_fire_ldbr;
`endif
`ifdef LOAD_SPEC_WAKE
    // LOAD_SPEC_WAKE engagement counters (load-spec-wake.md §8.6): prove the
    // mechanism fires. broadcast = LSQ spec-wake broadcasts; consumers = IQ
    // spec-reliant ALU picks; hit/miss = the one-cycle verdicts; squash =
    // consumer writebacks zeroed by spec_squash on a miss.
    logic [63:0] perf_ldspec_broadcast;
    logic [63:0] perf_ldspec_consumers;
    logic [63:0] perf_ldspec_hit;
    logic [63:0] perf_ldspec_miss;
    logic [63:0] perf_ldspec_squash;
`endif
    logic [63:0] perf_total_data_reads;
    logic [63:0] perf_total_data_writes;
    logic [63:0] perf_stall_instr [PERF_STALL_BUCKETS];
    logic [63:0] perf_branch_instr_counter [16];
    logic [63:0] perf_jal_instr_counter [8];
    logic [63:0] perf_jalr_instr_counter [8];
    logic [63:0] perf_jalr_predicted_correct;
    logic [63:0] perf_jalr_predicted_incorrect;
    logic [63:0] perf_jalr_unpredicted;
    logic [63:0] perf_return_predicted_correct;
    logic [63:0] perf_return_predicted_incorrect;
    logic [63:0] perf_return_unpredicted;
    logic [63:0] perf_last_dispatch_cycle;
    logic [63:0] perf_stall_cycles_prev;
    logic perf_first_dispatch;

    // --- Extended microarchitectural counters (asap7-synth perf study) ---
    // All read-only taps off existing datapath signals; no functional effect
    // (single-core / COHERENT=0 path stays bit-identical). Gated by the
    // always-on SIMULATION_18447 define; absent from the synthesis tops.
    logic [63:0] perf_rob_full_cycles;        // active list (ROB) full
    logic [63:0] perf_rob_empty_cycles;       // ROB empty (drained -> frontend-bound)
    logic [63:0] perf_iq_full_cycles;         // integer issue queue full
    logic [63:0] perf_memq_full_cycles;       // load/store queue full
    logic [63:0] perf_bstack_full_cycles;     // branch checkpoint stack full
    logic [63:0] perf_freelist_stall_cycles;  // no free physreg available
    logic [63:0] perf_dispatch_stall_cycles;  // any-reason dispatch stall
    logic [63:0] perf_bstack_branch_block;    // cycles a branch was blocked by full stack
    logic [63:0] perf_branch_presented;       // cycles a branch was presented at dispatch
`ifdef DFE_STATS
    // Decoupled-frontend (FTQ) S0 instrumentation: how much predict_stall is actually
    // CONVERTIBLE by hiding the sync-read predictor latency. ps_raw = all predict_stall
    // cycles; ps_sole = cycles where predict_stall is the ONLY dispatch inhibitor AND the
    // backend is not full (the truly recoverable count -- other suppress/structural terms
    // would stall anyway); ps_sole_res = ps_sole with a fetched block already buffered
    // (fbuf_cnt!=0), i.e. run-ahead has work (predictor-latency-bound, not ifetch-bound).
    logic [63:0] perf_ps_raw;
    logic [63:0] perf_ps_sole;
    logic [63:0] perf_ps_sole_res;
`endif
`ifdef DUAL_BRANCH_COUNT
    // Upper bound on the dispatch throughput a 2nd-branch-per-group (DUAL_BRANCH) could
    // recover: cycles where a dispatched branch left a VALID younger lane held behind it
    // (grp_branch_cut), and the subset where that held lane was itself a branch
    // (grp_2nd_branch). Measurement only, no datapath effect.
    logic [63:0] perf_grp_branch_cut;
    logic [63:0] perf_grp_2nd_branch;
`endif
    logic [63:0] perf_commit_starved_be;      // 0-retire w/ non-empty ROB (backend/latency)
    logic [63:0] perf_commit_starved_fe;      // 0-retire w/ empty ROB (frontend)
`ifdef ROBHEAD_STATS
    // Decompose commit_starved_backend (0-retire, non-empty ROB) by WHY the ROB head is not
    // retiring -- distinguishes the small-window case from commit-gating / head-op latency.
    logic [63:0] perf_cs_gated;    // head DONE (commit_valid[0]) but retire blocked (store port / SC-AMO / commit gate)
    logic [63:0] perf_cs_window;   // head NOT done AND ROB full -> can't dispatch behind it = SMALL WINDOW
    logic [63:0] perf_cs_latency;  // head NOT done AND ROB has room -> head waiting on its op/deps (ILP / FU latency)
`endif

`ifdef CSWHY
    // ---- CSWHY probe wires (all driven by the module that OWNS the fact) ----
    active_id_t  cswhy_head_id;
    logic        cswhy_head_present, cswhy_head_done, cswhy_head_pending, cswhy_head_xclass_ok;
    logic [5:0]  cswhy_head_class;
    logic        cswhy_iq_present, cswhy_iq_ready, cswhy_iq_picked, cswhy_iq_multi;
    logic        cswhy_mul_busy, cswhy_mul_wb, cswhy_div_busy, cswhy_div_wb;
    logic        cswhy_fp_busy,  cswhy_fp_wb;
    active_id_t  cswhy_lsq_head_id;
    logic        cswhy_lsq_head_valid;
    logic [5:0]  cswhy_lsq_reason;
    logic [$clog2(ACTIVE_LIST_SIZE+1)-1:0] cswhy_head_count;
    // ALU S2: the head is in an ALU execute stage. The issue bus is the core's own
    // fact, so this one term is legitimately computed here.
    logic        cswhy_alu_s2_head;
    // The ALU REGISTERS its writeback (ooo_alu_pipe.sv:196-203), unlike mul/div which
    // drive combinationally off the last pipe stage. So there is one cycle where the
    // head's result is on the bus but: its IQ slot was freed at select, alu_issue_valid_q
    // is already 0, and entries_q.done is not set until NEXT cycle. Nothing claimed it.
    logic        cswhy_alu_wb_head;
    // XC-2: the LSQ head must be an in-flight ROB entry, never OLDER than the ROB
    // head. Ring distance from the ROB head must be inside the live window. The
    // YOUNGER direction is legal (a load pops the LSQ on its writeback cycle while
    // the ROB done write is still in entries_next) and shows up as M_DRAINED.
    logic        cswhy_lsq_in_window;
    always_comb begin
        cswhy_alu_s2_head = 1'b0;
        for (int p = 0; p < ALU_ISSUE_PORTS; p += 1) begin
            if (alu_issue_valid_q[p] &&
                    (alu_issue_entry_q[p].active_id == cswhy_head_id))
                cswhy_alu_s2_head = 1'b1;
        end
        cswhy_alu_wb_head = cswhy_probe_valid &&
            ((alu0_writeback.valid && (alu0_writeback.active_id == cswhy_head_id)) ||
             (alu1_writeback.valid && (alu1_writeback.active_id == cswhy_head_id))
`ifdef ALU4
             || (alu2_writeback.valid && (alu2_writeback.active_id == cswhy_head_id))
`endif
            );
        cswhy_lsq_in_window =
            ({1'b0, ACTIVE_ID_BITS'(cswhy_lsq_head_id - cswhy_head_id)} <
             {1'b0, cswhy_head_count});
    end
    // The probe is only meaningful when there IS a live head entry.
    wire         cswhy_probe_valid = cswhy_head_present;
`endif
`ifdef CSWHY
    // The histogram: [arm][class][state]. Printed nonzero-only with an RTL-emitted
    // legend so the dump keys cannot drift from the enum.
    logic [63:0] cswhy_cell [3][CSWHY_NCLASS][CSWHY_NSTATE];
    logic [63:0] cswhy_visits [CSWHY_NCLASS];
    logic [63:0] cswhy_depart_retire, cswhy_depart_other;
    logic [63:0] cswhy_xc_class, cswhy_xc_multi, cswhy_xc_mem_outside, cswhy_xc_pending_bad;
    // Visit tracking closes at DEPARTURE on the REGISTERED class, with explicit
    // present rise/fall so drain-to-empty -> refill-at-the-same-index is caught.
    logic        cswhy_seen_q, cswhy_retired_q;
    active_id_t  cswhy_hid_q;
    logic [4:0]  cswhy_hcls_q;
`endif
    logic [63:0] perf_retire_hist [OOO_WIDTH+1];
    logic [63:0] perf_dispatch_hist [OOO_WIDTH+1];
`ifdef DISPATCH_STATS
    // Decompose dispatch_hist[0] (zero-dispatch cycles) into mutually exclusive
    // reasons. INVARIANT, checked at end of sim: sum(all buckets below) ==
    // dispatch_hist_0. `ds_*` = dispatch_stall asserted (whole-group stall);
    // `dn_*` = NOT stalled yet nothing dispatched (frontend starve / lane-0 cut).
    logic [63:0] perf_ds_redirect, perf_ds_trap, perf_ds_bstack_late, perf_ds_wfi;
    logic [63:0] perf_ds_irqdrain, perf_ds_terminal, perf_ds_control, perf_ds_serial;
    logic [63:0] perf_ds_fencei, perf_ds_predict, perf_ds_fflags, perf_ds_supp_other;
    logic [63:0] perf_ds_robfull, perf_ds_iqfull, perf_ds_memqfull, perf_ds_bstack;
    logic [63:0] perf_ds_other, perf_ds_supp_shadowed;
    logic [63:0] perf_dn_nolanes, perf_dn_other;
    logic [63:0] perf_dn_cut [16];   // zero-dispatch, no stall, by DCUT_* code
    // Group truncation: partial groups (1..W-1 dispatched) -- the width that is
    // silently lost today. lost = popcount(lane_valid & ~dispatch_valid).
    logic [63:0] perf_dt_cut [16];               // by DCUT_* code
    logic [63:0] perf_dt_lost [OOO_WIDTH+1];     // cycles by #lanes lost
    logic [63:0] perf_dt_slots;                  // total lost dispatch slots
    // Fetch-flush accounting. ff[] partitions the flush CYCLES by winning redirect arm;
    // The decision metric is ff_dead + ff_hold (NOT ff_dead alone):
    // ff_dead[] charges each flush the FRONTEND-STARVED
    // cycles that follow it (zero dispatch, no backend stall, no valid lanes), i.e. the
    // refetch bubble it actually cost; ff_hold[] charges the cycles where the flush's OWN
    // recovery suppress term (control_pending/fencei/serial/predict_stall) held dispatch
    // with no queue full. Only genuine backend pressure (a full queue) is uncharged. sum(ff_dead) <= dn_nolanes, and the difference is
    // frontend starvation NOT caused by a flush (cold miss, fetch-credit, xlate).
    logic [63:0] perf_ff [16];
    logic [63:0] perf_ff_dead [16];
    logic [63:0] perf_ff_hold [16];
    logic [63:0] perf_ff_total;
    logic [63:0] perf_btb_suppress;   // predictor redirects the BTB already steered (no flush)
    logic [63:0] perf_pred_raw;       // raw predictor_redirect_valid events
    logic        perf_ffp_valid;      // a flush is awaiting frontend recovery
    logic [3:0]  perf_ffp_src;        // ...and which arm caused it
`endif
    logic [63:0] perf_compressed_retired;     // RVC (16-bit) instructions retired
    logic [63:0] perf_l1i_miss_cnt, perf_l1d_miss_cnt, perf_l1d_wb_cnt;
    logic [63:0] perf_load_lat_sum, perf_load_lat_count;  // avg load accept->resp latency
    logic        perf_load_pending_q;
    logic [63:0] perf_load_start_cycle;
    logic [63:0] perf_lsq_hr [7];             // P3-M0: LSQ head-blocking reason histogram
    logic [63:0] perf_lsq_fwd [5];            // P3 L2a: store->load forwarding-opportunity histogram

    always_ff @(posedge clk or negedge rst_l) begin
        if (!rst_l) begin
            perf_cycle_counter = 64'b0;
            perf_dispatch_counter = 64'b0;
            perf_retire_counter = 64'b0;
            perf_frontend_stall_cycles = 64'b0;
            perf_branch_instructions = 64'b0;
            perf_mispredicted_branches = 64'b0;
            perf_alu_instructions = 64'b0;
            perf_load_instructions = 64'b0;
            perf_store_instructions = 64'b0;
`ifdef FUSE_UADDR
            perf_fu_fire_ab = 64'b0;
            perf_fu_fire_c = 64'b0;
`endif
`ifdef FUSE_CMPBR
            perf_fu_fire_cmpbr = 64'b0;
`endif
`ifdef FUSE_LDBR
            perf_fu_fire_ldbr = 64'b0;
`endif
`ifdef LOAD_SPEC_WAKE
            perf_ldspec_broadcast = 64'b0;
            perf_ldspec_consumers = 64'b0;
            perf_ldspec_hit = 64'b0;
            perf_ldspec_miss = 64'b0;
            perf_ldspec_squash = 64'b0;
`endif
            perf_total_data_reads = 64'b0;
            perf_total_data_writes = 64'b0;
            perf_jalr_predicted_correct = 64'b0;
            perf_jalr_predicted_incorrect = 64'b0;
            perf_jalr_unpredicted = 64'b0;
            perf_return_predicted_correct = 64'b0;
            perf_return_predicted_incorrect = 64'b0;
            perf_return_unpredicted = 64'b0;
            perf_last_dispatch_cycle = 64'b0;
            perf_stall_cycles_prev = 64'b0;
            perf_first_dispatch = 1'b1;
            perf_rob_full_cycles = 64'b0;
            perf_rob_empty_cycles = 64'b0;
            perf_iq_full_cycles = 64'b0;
            perf_memq_full_cycles = 64'b0;
            perf_bstack_full_cycles = 64'b0;
            perf_freelist_stall_cycles = 64'b0;
            perf_dispatch_stall_cycles = 64'b0;
            perf_bstack_branch_block = 64'b0;
            perf_branch_presented = 64'b0;
`ifdef DFE_STATS
            perf_ps_raw = 64'b0;
            perf_ps_sole = 64'b0;
            perf_ps_sole_res = 64'b0;
`endif
`ifdef DUAL_BRANCH_COUNT
            perf_grp_branch_cut = 64'b0;
            perf_grp_2nd_branch = 64'b0;
`endif
            perf_commit_starved_be = 64'b0;
            perf_commit_starved_fe = 64'b0;
`ifdef ROBHEAD_STATS
            perf_cs_gated = 64'b0; perf_cs_window = 64'b0; perf_cs_latency = 64'b0;
`endif
            perf_compressed_retired = 64'b0;
            perf_l1i_miss_cnt = 64'b0;
            perf_l1d_miss_cnt = 64'b0;
            perf_l1d_wb_cnt = 64'b0;
            perf_load_lat_sum = 64'b0;
            perf_load_lat_count = 64'b0;
            perf_load_pending_q = 1'b0;
            perf_load_start_cycle = 64'b0;
            for (int i = 0; i < 7; i += 1) perf_lsq_hr[i] = 64'b0;
            for (int i = 0; i < 5; i += 1) perf_lsq_fwd[i] = 64'b0;
            for (int i = 0; i <= OOO_WIDTH; i += 1) begin
                perf_retire_hist[i] = 64'b0;
                perf_dispatch_hist[i] = 64'b0;
            end
`ifdef CSWHY
            for (int a = 0; a < 3; a += 1)
                for (int c = 0; c < CSWHY_NCLASS; c += 1)
                    for (int st = 0; st < CSWHY_NSTATE; st += 1) cswhy_cell[a][c][st] = 64'b0;
            for (int c = 0; c < CSWHY_NCLASS; c += 1) cswhy_visits[c] = 64'b0;
            cswhy_depart_retire = 64'b0; cswhy_depart_other = 64'b0;
            cswhy_xc_class = 64'b0; cswhy_xc_multi = 64'b0;
            cswhy_xc_mem_outside = 64'b0; cswhy_xc_pending_bad = 64'b0;
            cswhy_seen_q = 1'b0; cswhy_retired_q = 1'b0;
            cswhy_hid_q = '0; cswhy_hcls_q = C_NOHEAD;
`endif
`ifdef DISPATCH_STATS
            perf_ds_redirect = 64'b0; perf_ds_trap     = 64'b0; perf_ds_bstack_late = 64'b0;
            perf_ds_wfi      = 64'b0; perf_ds_irqdrain = 64'b0; perf_ds_terminal = 64'b0;
            perf_ds_control  = 64'b0; perf_ds_serial   = 64'b0; perf_ds_fencei = 64'b0;
            perf_ds_predict  = 64'b0; perf_ds_fflags   = 64'b0; perf_ds_supp_other = 64'b0;
            perf_ds_robfull  = 64'b0; perf_ds_iqfull   = 64'b0; perf_ds_memqfull = 64'b0;
            perf_ds_bstack   = 64'b0; perf_ds_other    = 64'b0; perf_ds_supp_shadowed = 64'b0;
            perf_dn_nolanes  = 64'b0; perf_dn_other    = 64'b0; perf_dt_slots  = 64'b0;
            for (int i = 0; i < 16; i += 1) begin
                perf_dn_cut[i] = 64'b0;
                perf_dt_cut[i] = 64'b0;
            end
            for (int i = 0; i <= OOO_WIDTH; i += 1) perf_dt_lost[i] = 64'b0;
            for (int i = 0; i < 16; i += 1) begin
                perf_ff[i] = 64'b0;
                perf_ff_dead[i] = 64'b0;
                perf_ff_hold[i] = 64'b0;
            end
            perf_ff_total = 64'b0; perf_btb_suppress = 64'b0; perf_pred_raw = 64'b0;
            perf_ffp_valid = 1'b0; perf_ffp_src = FF_NONE;
`endif
            for (int i = 0; i < PERF_STALL_BUCKETS; i += 1) begin
                perf_stall_instr[i] = 64'b0;
            end
            for (int i = 0; i < 16; i += 1) begin
                perf_branch_instr_counter[i] = 64'b0;
            end
            for (int i = 0; i < 8; i += 1) begin
                perf_jal_instr_counter[i] = 64'b0;
                perf_jalr_instr_counter[i] = 64'b0;
            end
        end else if (!halted_q) begin
            perf_cycle_counter = perf_cycle_counter + 64'd1;
            perf_lsq_hr[lsq_head_reason] = perf_lsq_hr[lsq_head_reason] + 64'd1;
            perf_lsq_fwd[lsq_fwd_class] = perf_lsq_fwd[lsq_fwd_class] + 64'd1;
            if (frontend_stall || dispatched_unpredicted_control) begin
                perf_frontend_stall_cycles = perf_frontend_stall_cycles + 64'd1;
            end
            if (dmem_req_valid && !dmem_req_write && dmem_req_ready) begin
                perf_total_data_reads = perf_total_data_reads + 64'd1;
            end
            if (dmem_req_valid && dmem_req_write && dmem_req_ready) begin
                perf_total_data_writes = perf_total_data_writes + 64'd1;
            end

            // --- Structural-hazard occupancy (per-cycle) ---
            if (active_full)        perf_rob_full_cycles       = perf_rob_full_cycles + 64'd1;
            if (active_empty)       perf_rob_empty_cycles      = perf_rob_empty_cycles + 64'd1;
            if (int_iq_full)        perf_iq_full_cycles        = perf_iq_full_cycles + 64'd1;
            if (mem_queue_full)     perf_memq_full_cycles      = perf_memq_full_cycles + 64'd1;
            if (branch_stack_full)  perf_bstack_full_cycles    = perf_bstack_full_cycles + 64'd1;
            if (!free_can_allocate) perf_freelist_stall_cycles = perf_freelist_stall_cycles + 64'd1;
            if (dispatch_stall)     perf_dispatch_stall_cycles = perf_dispatch_stall_cycles + 64'd1;
            if (hpm_l1i_miss)       perf_l1i_miss_cnt          = perf_l1i_miss_cnt + 64'd1;
            if (hpm_l1d_miss)       perf_l1d_miss_cnt          = perf_l1d_miss_cnt + 64'd1;
            if (hpm_l1d_wb)         perf_l1d_wb_cnt            = perf_l1d_wb_cnt + 64'd1;

            // Branch presented at dispatch, and whether the full branch stack
            // blocked it (dispatch_control forces a stall on branch+stack-full).
            begin
                logic branch_presented_c;
                branch_presented_c = 1'b0;
                for (int i = 0; i < OOO_WIDTH; i += 1) begin
                    if (lane_valid[i] && lane_is_branch[i]) branch_presented_c = 1'b1;
                end
                if (branch_presented_c) begin
                    perf_branch_presented = perf_branch_presented + 64'd1;
                    if (branch_stack_full)
                        perf_bstack_branch_block = perf_bstack_branch_block + 64'd1;
                end
            end
`ifdef DFE_STATS
            if (predict_stall) begin
                perf_ps_raw = perf_ps_raw + 64'd1;
                // Sole-cause: predict_stall is the only reason dispatch is held this cycle,
                // and no backend structure is full -- so hiding it would actually convert.
                if (!redirect_valid && !terminal_pending_q && !control_pending_q &&
                    !serial_pending_q && !halted_q && !irq_drain_q && !wfi_wait_q &&
                    !commit_take_trap && !fencei_block && !fflags_drain_stall &&
                    !active_full && !int_iq_full && !mem_queue_full && !fetch_xlate_stall) begin
                    perf_ps_sole = perf_ps_sole + 64'd1;
                    if (fbuf_cnt_q != 2'd0)
                        perf_ps_sole_res = perf_ps_sole_res + 64'd1;
                end
            end
`endif
`ifdef DUAL_BRANCH_COUNT
            // A dispatched branch always terminates its group, so a valid younger lane
            // that did NOT dispatch was cut off by that branch -- the addressable event
            // for widening the group past the first branch.
            begin
                logic cut_c, second_br_c;
                cut_c = 1'b0; second_br_c = 1'b0;
                for (int i = 0; i < OOO_WIDTH; i += 1) begin
                    if (dispatch_valid[i] && lane_is_branch[i]) begin
                        for (int j = i + 1; j < OOO_WIDTH; j += 1) begin
                            if (lane_valid[j] && !dispatch_valid[j]) begin
                                cut_c = 1'b1;
                                if (lane_is_branch[j]) second_br_c = 1'b1;
                            end
                        end
                    end
                end
                if (cut_c)       perf_grp_branch_cut = perf_grp_branch_cut + 64'd1;
                if (second_br_c) perf_grp_2nd_branch = perf_grp_2nd_branch + 64'd1;
            end
`endif

            // Commit-bandwidth histogram + starvation decomposition.
            perf_retire_hist[retire_count] = perf_retire_hist[retire_count] + 64'd1;
            if (retire_count == 3'd0) begin
                if (active_empty) perf_commit_starved_fe = perf_commit_starved_fe + 64'd1;
                else begin
                    perf_commit_starved_be = perf_commit_starved_be + 64'd1;
`ifdef ROBHEAD_STATS
                    // WHY the ROB head did not retire this cycle (mutually exclusive):
                    if (active_commit_valid[0]) perf_cs_gated   = perf_cs_gated   + 64'd1; // head ready, retire gated
                    else if (active_full)       perf_cs_window  = perf_cs_window  + 64'd1; // head stalled + ROB full
                    else                        perf_cs_latency = perf_cs_latency + 64'd1; // head stalled, window has room
`endif
`ifdef CSWHY
                    // ---- CSWHY bin. The 3-way ladder above is FOLLOWED, not modified.
                    // The core performs ZERO classification: every term below is a
                    // reason bit emitted by the module that owns the fact.
                    begin
                        logic [1:0] cswhy_arm;
                        logic [5:0] cswhy_state;
                        logic       is_mem_c, lsq_is_head;
                        cswhy_arm = active_commit_valid[0] ? ARM_GATED :
                                    (active_full ? ARM_WINDOW : ARM_LAT);
                        is_mem_c = (cswhy_head_class == C_LOAD)  || (cswhy_head_class == C_FLOAD) ||
                                   (cswhy_head_class == C_STORE) || (cswhy_head_class == C_FSTORE) ||
                                   (cswhy_head_class == C_ATOMIC);
                        // Memory states are admitted ONLY when the LSQ head IS the ROB
                        // head -- they are different queues with independent pointers.
                        lsq_is_head = cswhy_lsq_head_valid && (cswhy_lsq_head_id == cswhy_head_id);

                        if ((cswhy_head_class == C_NOHEAD) || (cswhy_head_class == C_UNWRITTEN) ||
                                (cswhy_head_class == C_ABORTED))          cswhy_state = S_NA;
                        else if (cswhy_head_done)                          cswhy_state = S_DONE;
                        // S_SELECT MUST outrank the IQ arms: the IQ clears a slot only
                        // in entries_sel, so entries_wake still claims it on the select
                        // cycle and the cycle would be mis-billed as IQ residency.
                        else if (cswhy_iq_picked)                          cswhy_state = S_SELECT;
                        else if (is_mem_c && lsq_is_head)                  cswhy_state = cswhy_lsq_reason;
                        else if (is_mem_c)                                 cswhy_state = M_DRAINED;
                        else if (cswhy_alu_s2_head)                        cswhy_state = S_ALU_EXEC;
                        else if (cswhy_alu_wb_head)                        cswhy_state = S_ALU_WB;
                        else if (cswhy_mul_wb || cswhy_div_wb || cswhy_fp_wb) cswhy_state = S_FU_WB;
                        else if (cswhy_mul_busy)                           cswhy_state = S_MUL_EXEC;
                        else if (cswhy_div_busy)                           cswhy_state = S_DIV_EXEC;
                        else if (cswhy_fp_busy)                            cswhy_state = S_FP_EXEC;
                        else if (cswhy_iq_present && cswhy_iq_ready)       cswhy_state = S_IQ_PICK;
                        else if (cswhy_iq_present)                         cswhy_state = S_IQ_OPWAIT;
                        else                                               cswhy_state = S_NOWHERE;

                        cswhy_cell[cswhy_arm][cswhy_head_class][cswhy_state] =
                            cswhy_cell[cswhy_arm][cswhy_head_class][cswhy_state] + 64'd1;

                        // ---- cross-checks (counted, NEVER used as guards: as a guard
                        // XC-1 would structurally kill C_NOHEAD/C_UNWRITTEN/S_DONE).
                        if (!cswhy_head_xclass_ok) cswhy_xc_class = cswhy_xc_class + 64'd1;
                        if (cswhy_iq_multi) cswhy_xc_multi = cswhy_xc_multi + 64'd1;
                        else begin
                            logic [3:0] claims;
                            // The IQ still claims the slot on the select cycle, so it is
                            // excluded when the select term fires (else multi != 0 by
                            // construction).
                            claims = 4'(cswhy_iq_present && !cswhy_iq_picked) +
                                     4'(cswhy_alu_s2_head) + 4'(cswhy_alu_wb_head) +
                                     4'(cswhy_mul_busy || cswhy_mul_wb) +
                                     4'(cswhy_div_busy || cswhy_div_wb) +
                                     4'(cswhy_fp_busy || cswhy_fp_wb) + 4'(lsq_is_head);
                            if (claims > 4'd1) cswhy_xc_multi = cswhy_xc_multi + 64'd1;
                        end
                        if (cswhy_lsq_head_valid && !lsq_is_head &&
                                !cswhy_lsq_in_window) cswhy_xc_mem_outside = cswhy_xc_mem_outside + 64'd1;
                        if ((cswhy_arm == ARM_LAT) && !cswhy_head_pending)
                            cswhy_xc_pending_bad = cswhy_xc_pending_bad + 64'd1;
                    end
`endif
                end
            end
`ifdef CSWHY
            // ---- visit accounting: closes at DEPARTURE on the REGISTERED class, with
            // explicit present rise/fall so a drain-to-empty then refill at the same
            // index is not silently merged into one visit.
            if (cswhy_seen_q && (!cswhy_head_present || (cswhy_head_id != cswhy_hid_q))) begin
                cswhy_visits[cswhy_hcls_q] = cswhy_visits[cswhy_hcls_q] + 64'd1;
                // Departure is observed the cycle AFTER the retire, so the cause is the
                // twice-registered retire flag.
                // head_postskip is PRE-pop, so a retire at N advances the head at N+1 and
                // the departure edge is seen at N+1 -- the cause is retire(N), which is
                // exactly cswhy_retired_q here (this read precedes its update below).
                // Using the twice-registered flag misfiled every ISOLATED retire.
                if (cswhy_retired_q) cswhy_depart_retire = cswhy_depart_retire + 64'd1;
                else                  cswhy_depart_other  = cswhy_depart_other  + 64'd1;
            end
            cswhy_seen_q     = cswhy_head_present;
            cswhy_hid_q      = cswhy_head_id;
            cswhy_hcls_q     = cswhy_head_class;
            cswhy_retired_q  = (retire_count != 3'd0);
`endif
            begin
                logic [2:0] dcnt;
                dcnt = 3'd0;
                for (int i = 0; i < OOO_WIDTH; i += 1) begin
                    if (dispatch_valid[i]) dcnt = dcnt + 3'd1;
                end
                perf_dispatch_hist[dcnt] = perf_dispatch_hist[dcnt] + 64'd1;
`ifdef DISPATCH_STATS
                begin
                    logic [2:0] lost_c;
                    logic       any_valid_c;
                    lost_c = 3'd0; any_valid_c = 1'b0;
                    for (int i = 0; i < OOO_WIDTH; i += 1) begin
                        if (lane_valid[i]) begin
                            any_valid_c = 1'b1;
                            if (!dispatch_valid[i]) lost_c = lost_c + 3'd1;
                        end
                    end

                    if (dcnt == 3'd0) begin
                        if (dispatch_stall) begin
                            // Whole-group stall. dstat_stall_reason gives the structural
                            // term; DSTL_SUPPRESS is decomposed here in the OR order of
                            // the suppress_dispatch expression at the instantiation.
                            case (dstat_stall_reason)
                            DSTL_ROB_FULL:  perf_ds_robfull  = perf_ds_robfull  + 64'd1;
                            DSTL_IQ_FULL:   perf_ds_iqfull   = perf_ds_iqfull   + 64'd1;
                            DSTL_MEMQ_FULL: perf_ds_memqfull = perf_ds_memqfull + 64'd1;
                            DSTL_BSTACK:    perf_ds_bstack   = perf_ds_bstack   + 64'd1;
                            // Stall asserted by a branch-stack arm firing on a lane that
                            // an EARLIER cut had already killed -- the branch stack was
                            // not the real cause (that lane could not dispatch anyway).
                            // The true cause is dstat_cut_reason; kept as its own bucket
                            // so sum(ds_*) == dispatch_stall_cycles still holds exactly.
                            DSTL_NONE:      perf_ds_bstack_late = perf_ds_bstack_late + 64'd1;
                            DSTL_SUPPRESS: begin
                                // Deliberate severity priority (machine-recovering first),
                                // NOT the textual OR order of suppress_dispatch -- the
                                // terms co-assert, so the order picks which one is blamed.
                                // halted_q is absent on purpose: this whole block is gated
                                // on !halted_q, so halt cycles are not measured at all.
                                if      (redirect_valid)     perf_ds_redirect = perf_ds_redirect + 64'd1;
                                else if (commit_take_trap)   perf_ds_trap     = perf_ds_trap     + 64'd1;
                                else if (wfi_wait_q)         perf_ds_wfi      = perf_ds_wfi      + 64'd1;
                                else if (irq_drain_q)        perf_ds_irqdrain = perf_ds_irqdrain + 64'd1;
                                else if (terminal_pending_q) perf_ds_terminal = perf_ds_terminal + 64'd1;
                                else if (control_pending_q)  perf_ds_control  = perf_ds_control  + 64'd1;
                                else if (serial_pending_q)   perf_ds_serial   = perf_ds_serial   + 64'd1;
                                else if (fencei_block)       perf_ds_fencei   = perf_ds_fencei   + 64'd1;
                                else if (predict_stall)      perf_ds_predict  = perf_ds_predict  + 64'd1;
                                else if (fflags_drain_stall) perf_ds_fflags   = perf_ds_fflags   + 64'd1;
                                else                         perf_ds_supp_other = perf_ds_supp_other + 64'd1;
                                // Co-occurrence: a suppress term got the credit, but a
                                // structural queue was ALSO full, so removing the suppress
                                // cause alone would NOT have converted this cycle. Without
                                // this the ds_* split reads as more recoverable than it is.
                                if (active_full || int_iq_full || mem_queue_full)
                                    perf_ds_supp_shadowed = perf_ds_supp_shadowed + 64'd1;
                            end
                            default:        perf_ds_other    = perf_ds_other    + 64'd1;
                            endcase
                        end else if (!any_valid_c) begin
                            // Frontend delivered nothing -- NOT backend pressure.
                            perf_dn_nolanes = perf_dn_nolanes + 64'd1;
                        end else if (dstat_cut_valid) begin
                            // Valid lanes present but stop_prefix cut at/before the
                            // first one, so the group produced nothing.
                            perf_dn_cut[dstat_cut_reason] = perf_dn_cut[dstat_cut_reason] + 64'd1;
                        end else begin
                            perf_dn_other = perf_dn_other + 64'd1;
                        end
                    end else if (lost_c != 3'd0) begin
                        // Partial group: some lanes went, some valid lanes were cut.
                        perf_dt_lost[lost_c] = perf_dt_lost[lost_c] + 64'd1;
                        perf_dt_slots = perf_dt_slots + 64'(lost_c);
                        if (dstat_cut_valid)
                            perf_dt_cut[dstat_cut_reason] = perf_dt_cut[dstat_cut_reason] + 64'd1;
                    end

                    // ---- fetch-flush accounting (shares any_valid_c / dcnt above) ----
                    if (predictor_redirect_valid) perf_pred_raw = perf_pred_raw + 64'd1;
                    if (btb_suppress)             perf_btb_suppress = perf_btb_suppress + 64'd1;
                    if (fetch_flush) begin
                        perf_ff[ff_src] = perf_ff[ff_src] + 64'd1;
                        perf_ff_total   = perf_ff_total + 64'd1;
                        // (Re-)arm the bubble charge against the NEWEST flush: a later
                        // redirect supersedes an older one's refetch.
                        perf_ffp_valid  = 1'b1;
                        perf_ffp_src    = ff_src;
                    end else if (perf_ffp_valid) begin
                        if (dcnt != 3'd0) begin
                            perf_ffp_valid = 1'b0;      // frontend recovered; stop charging
                        end else if (!dispatch_stall && !any_valid_c) begin
                            // Zero dispatch, backend willing, no instructions delivered:
                            // this cycle is the refetch bubble of that flush.
                            perf_ff_dead[perf_ffp_src] = perf_ff_dead[perf_ffp_src] + 64'd1;
                        end else if (dispatch_stall && !active_full && !int_iq_full &&
                                     !mem_queue_full) begin
                            // `dispatch_stall` is NOT a clean "backend's fault" guard: it
                            // CONTAINS the suppress terms a flush arm itself raises. The
                            // worst case is FF_UNPRED -- dispatched_unpredicted_control
                            // raises the flush AND sets control_pending_q, which is a
                            // suppress term, so the whole JALR freeze is dispatch_stall=1
                            // and ff_dead charges it NOTHING; then the freeze always exits
                            // via a forced mispredict (ooo_alu_pipe.sv:481-484 hardcodes
                            // branch_mispredict_for=1 for unpredicted PC_uncond/PC_indirect)
                            // which re-arms to FF_REDIRECT and bills it the whole bubble.
                            // So: when the machine is held by a SUPPRESS term and no
                            // structural queue is full, charge the flush a HOLD cycle.
                            // (Same sole-cause shape as perf_ps_sole.)
                            perf_ff_hold[perf_ffp_src] = perf_ff_hold[perf_ffp_src] + 64'd1;
                        end
                        // else: a structural queue is full -- genuinely backend pressure,
                        // the cycle was lost regardless of the flush. Stay armed.
                    end
                end
`endif
            end

            // Average cacheable-load latency (accept -> response); one load is
            // outstanding at a time, so req/resp pair cleanly. Devices excluded.
            if (perf_load_pending_q && dmem_resp_valid) begin
                perf_load_lat_sum = perf_load_lat_sum +
                    (perf_cycle_counter - perf_load_start_cycle);
                perf_load_lat_count = perf_load_lat_count + 64'd1;
                perf_load_pending_q = 1'b0;
            end
            if (!perf_load_pending_q && dmem_req_valid && !dmem_req_write &&
                    dmem_req_ready && !dmem_req_device) begin
                perf_load_pending_q = 1'b1;
                perf_load_start_cycle = perf_cycle_counter;
            end

`ifdef LOAD_SPEC_WAKE
            // LOAD_SPEC_WAKE engagement: broadcast / spec-reliant picks / the
            // one-cycle verdict / miss squashes.
            if (lsq_load_spec_wake_valid)
                perf_ldspec_broadcast = perf_ldspec_broadcast + 64'd1;
            for (int p = 0; p < ALU_ISSUE_PORTS; p += 1) begin
                if (iq_issue_ld_spec[p])
                    perf_ldspec_consumers = perf_ldspec_consumers + 64'd1;
                if (alu_issue_ld_spec_q[p] && ld_spec_miss)
                    perf_ldspec_squash = perf_ldspec_squash + 64'd1;
            end
            if (ld_spec_hit)  perf_ldspec_hit  = perf_ldspec_hit  + 64'd1;
            if (ld_spec_miss) perf_ldspec_miss = perf_ldspec_miss + 64'd1;
`endif

            for (int i = 0; i < OOO_WIDTH; i += 1) begin
                if (dispatch_valid[i]) begin
`ifdef FUSE_UADDR
                    if (fuse_master[i] && (fuse_kind_lane[i] == FUSE_K_UADDR))
                        perf_fu_fire_ab = perf_fu_fire_ab + 64'd1;
                    if (fu_pc_base[i])
                        perf_fu_fire_c = perf_fu_fire_c + 64'd1;
`endif
`ifdef FUSE_CMPBR
                    if (fuse_master[i] && (fuse_kind_lane[i] == FUSE_K_CMPBR))
                        perf_fu_fire_cmpbr = perf_fu_fire_cmpbr + 64'd1;
`endif
`ifdef FUSE_LDBR
                    if (fuse_master[i] && (fuse_kind_lane[i] == FUSE_K_LDBR))
                        perf_fu_fire_ldbr = perf_fu_fire_ldbr + 64'd1;
`endif
                    perf_dispatch_counter = perf_dispatch_counter + 64'd1;
                    if (!perf_first_dispatch) begin
                        perf_stall_cycles_prev = perf_cycle_counter -
                            perf_last_dispatch_cycle - 64'd1;
                        if (perf_stall_cycles_prev < 64'(PERF_STALL_BUCKETS)) begin
                            perf_stall_instr[PERF_STALL_BITS'(
                                    perf_stall_cycles_prev)] =
                                perf_stall_instr[PERF_STALL_BITS'(
                                    perf_stall_cycles_prev)] + 64'd1;
                        end
                    end else begin
                        perf_first_dispatch = 1'b0;
                    end
                    perf_last_dispatch_cycle = perf_cycle_counter;
                end

                if (retire_valid[i]) begin
                    perf_retire_counter = perf_retire_counter + 64'd1;
`ifdef RVC
                    if (active_commit_packet[i].is_compressed)
                        perf_compressed_retired = perf_compressed_retired + 64'd1;
`endif
                    unique case (RISCV_ISA::opcode_t'(active_commit_packet[i].instr[6:0]))
                        RISCV_ISA::OP_OP, RISCV_ISA::OP_IMM: begin
                            perf_alu_instructions = perf_alu_instructions + 64'd1;
                        end
                        RISCV_ISA::OP_LOAD: begin
                            perf_load_instructions = perf_load_instructions + 64'd1;
                        end
                        RISCV_ISA::OP_STORE: begin
                            perf_store_instructions = perf_store_instructions + 64'd1;
                        end
                        default: begin
                        end
                    endcase
                end
            end

            if (branch_writeback.valid && branch_writeback.branch_valid) begin
                perf_branch_instructions = perf_branch_instructions + 64'd1;
                if (branch_writeback.branch_mispredict) begin
                    perf_mispredicted_branches = perf_mispredicted_branches + 64'd1;
                end

                unique case (RISCV_ISA::opcode_t'(branch_writeback.instr[6:0]))
                    RISCV_ISA::OP_BRANCH: begin
                        logic [3:0] branch_idx;
                        branch_idx = {
                            branch_writeback.redirect_pc < branch_writeback.pc,
                            branch_writeback.redirect_pc !=
                                (branch_writeback.pc +
                                 `ILEN_INC(branch_writeback.is_compressed)),
                            branch_writeback.control_predicted,
                            branch_writeback.branch_mispredict
                        };
                        perf_branch_instr_counter[branch_idx] =
                            perf_branch_instr_counter[branch_idx] + 64'd1;
                    end
                    RISCV_ISA::OP_JAL: begin
                        logic [2:0] jal_idx;
                        jal_idx = {
                            branch_writeback.instr[11:7] == 5'd1,
                            branch_writeback.control_predicted,
                            branch_writeback.branch_mispredict
                        };
                        perf_jal_instr_counter[jal_idx] =
                            perf_jal_instr_counter[jal_idx] + 64'd1;
                    end
                    RISCV_ISA::OP_JALR: begin
                        logic [2:0] jalr_idx;
                        logic is_return;
                        jalr_idx = {
                            branch_writeback.instr[19:15] == 5'd1,
                            branch_writeback.control_predicted,
                            branch_writeback.branch_mispredict
                        };
                        is_return = (branch_writeback.instr[19:15] == 5'd1) &&
                            (branch_writeback.instr[11:7] == 5'd0);
                        perf_jalr_instr_counter[jalr_idx] =
                            perf_jalr_instr_counter[jalr_idx] + 64'd1;
                        if (branch_writeback.control_predicted &&
                                !branch_writeback.branch_mispredict) begin
                            perf_jalr_predicted_correct =
                                perf_jalr_predicted_correct + 64'd1;
                            if (is_return) begin
                                perf_return_predicted_correct =
                                    perf_return_predicted_correct + 64'd1;
                            end
                        end else if (branch_writeback.control_predicted) begin
                            perf_jalr_predicted_incorrect =
                                perf_jalr_predicted_incorrect + 64'd1;
                            if (is_return) begin
                                perf_return_predicted_incorrect =
                                    perf_return_predicted_incorrect + 64'd1;
                            end
                        end else begin
                            perf_jalr_unpredicted = perf_jalr_unpredicted + 64'd1;
                            if (is_return) begin
                                perf_return_unpredicted =
                                    perf_return_unpredicted + 64'd1;
                            end
                        end
                    end
                    default: begin
                    end
                endcase
            end
        end
    end

    // Dump at end-of-sim ($finish): fires on the ECALL-halt path (benchmarks)
    // AND on a +maxcyc cap (xv6, which never halts). The legacy stdout lines are
    // preserved verbatim so scripts/run_447_benchmarks.py keeps parsing them; the
    // extended metrics follow, and +perf_out=<path> writes a key=value file.
    final begin
        string perf_out_path;
        int    pfd;
        $display("FINAL OOO PERFORMANCE COUNTERS:");
        $display("Total cycles: %0d", perf_cycle_counter);
        $display("Instructions dispatched: %0d", perf_dispatch_counter);
        $display("Instructions retired: %0d", perf_retire_counter);
        $display("  ALU instructions: %0d", perf_alu_instructions);
        $display("  Load instructions: %0d", perf_load_instructions);
        $display("  Store instructions: %0d", perf_store_instructions);
`ifdef FUSE_UADDR
        $display("  FUSE_UADDR (a)/(b) pairs fused: %0d", perf_fu_fire_ab);
        $display("  FUSE_UADDR (c) pc-base loads:   %0d", perf_fu_fire_c);
`endif
`ifdef FUSE_CMPBR
        $display("  FUSE_CMPBR pairs fused:         %0d", perf_fu_fire_cmpbr);
`endif
`ifdef FUSE_LDBR
        $display("  FUSE_LDBR pairs fused:          %0d", perf_fu_fire_ldbr);
`endif
`ifdef LOAD_SPEC_WAKE
        $display("  LDSPEC broadcasts:              %0d", perf_ldspec_broadcast);
        $display("  LDSPEC spec-woken consumers:    %0d", perf_ldspec_consumers);
        $display("  LDSPEC hits:                    %0d", perf_ldspec_hit);
        $display("  LDSPEC misses:                  %0d", perf_ldspec_miss);
        $display("  LDSPEC squashed writebacks:     %0d", perf_ldspec_squash);
`endif
        $display("Frontend stall cycles: %0d", perf_frontend_stall_cycles);
        for (int i = 0; i < PERF_STALL_BUCKETS; i += 1) begin
            $display("Dispatched instructions with %0d stalls: %0d", i,
                perf_stall_instr[i]);
        end
        for (int i = 0; i < 16; i += 1) begin
            $display("Branch inst (idx %0d):     %0d", i,
                perf_branch_instr_counter[i]);
        end
        for (int i = 0; i < 8; i += 1) begin
            $display("JAL inst (idx %0d):        %0d", i,
                perf_jal_instr_counter[i]);
            $display("JALR inst (idx %0d):       %0d", i,
                perf_jalr_instr_counter[i]);
        end
        $display("JALR predicted correct: %0d", perf_jalr_predicted_correct);
        $display("JALR predicted incorrect: %0d", perf_jalr_predicted_incorrect);
        $display("JALR unpredicted: %0d", perf_jalr_unpredicted);
        $display("Return predicted correct: %0d", perf_return_predicted_correct);
        $display("Return predicted incorrect: %0d", perf_return_predicted_incorrect);
        $display("Return unpredicted: %0d", perf_return_unpredicted);
        $display("Total data reads: %0d", perf_total_data_reads);
        $display("Total data writes: %0d", perf_total_data_writes);
        $display("Total control flow instructions: %0d", perf_branch_instructions);
        $display("Mispredicted control flow instructions: %0d",
            perf_mispredicted_branches);
        // --- extended microarchitectural metrics ---
        $display("EXT ROB full cycles: %0d", perf_rob_full_cycles);
        $display("EXT ROB empty cycles: %0d", perf_rob_empty_cycles);
        $display("EXT IQ full cycles: %0d", perf_iq_full_cycles);
        $display("EXT MemQ full cycles: %0d", perf_memq_full_cycles);
        $display("EXT BranchStack full cycles: %0d", perf_bstack_full_cycles);
        $display("EXT Freelist stall cycles: %0d", perf_freelist_stall_cycles);
        $display("EXT Dispatch stall cycles: %0d", perf_dispatch_stall_cycles);
        $display("EXT Branch presented cycles: %0d", perf_branch_presented);
        $display("EXT Branch blocked by full stack cycles: %0d", perf_bstack_branch_block);
        $display("EXT Commit-starved backend cycles: %0d", perf_commit_starved_be);
        $display("EXT Commit-starved frontend cycles: %0d", perf_commit_starved_fe);
        $display("EXT Compressed retired: %0d", perf_compressed_retired);
        $display("EXT L1I misses: %0d", perf_l1i_miss_cnt);
        $display("EXT L1D misses: %0d", perf_l1d_miss_cnt);
        $display("EXT L1D writebacks: %0d", perf_l1d_wb_cnt);
        $display("EXT Load latency sum: %0d", perf_load_lat_sum);
        $display("EXT Load latency count: %0d", perf_load_lat_count);
        for (int i = 0; i <= OOO_WIDTH; i += 1)
            $display("EXT Retire hist[%0d]: %0d", i, perf_retire_hist[i]);
        for (int i = 0; i <= OOO_WIDTH; i += 1)
            $display("EXT Dispatch hist[%0d]: %0d", i, perf_dispatch_hist[i]);

        if ($value$plusargs("perf_out=%s", perf_out_path)) begin
            pfd = $fopen(perf_out_path, "w");
            if (pfd != 0) begin
                $fdisplay(pfd, "cycles=%0d", perf_cycle_counter);
                $fdisplay(pfd, "dispatched=%0d", perf_dispatch_counter);
                $fdisplay(pfd, "retired=%0d", perf_retire_counter);
                $fdisplay(pfd, "alu=%0d", perf_alu_instructions);
                $fdisplay(pfd, "load=%0d", perf_load_instructions);
                $fdisplay(pfd, "store=%0d", perf_store_instructions);
`ifdef FUSE_UADDR
                $fdisplay(pfd, "fu_fire_ab=%0d", perf_fu_fire_ab);
                $fdisplay(pfd, "fu_fire_c=%0d", perf_fu_fire_c);
`endif
`ifdef FUSE_CMPBR
                $fdisplay(pfd, "fu_fire_cmpbr=%0d", perf_fu_fire_cmpbr);
`endif
`ifdef FUSE_LDBR
                $fdisplay(pfd, "fu_fire_ldbr=%0d", perf_fu_fire_ldbr);
`endif
`ifdef LOAD_SPEC_WAKE
                $fdisplay(pfd, "ldspec_broadcast=%0d", perf_ldspec_broadcast);
                $fdisplay(pfd, "ldspec_consumers=%0d", perf_ldspec_consumers);
                $fdisplay(pfd, "ldspec_hit=%0d", perf_ldspec_hit);
                $fdisplay(pfd, "ldspec_miss=%0d", perf_ldspec_miss);
                $fdisplay(pfd, "ldspec_squash=%0d", perf_ldspec_squash);
`endif
                $fdisplay(pfd, "frontend_stall_cycles=%0d", perf_frontend_stall_cycles);
                $fdisplay(pfd, "data_reads=%0d", perf_total_data_reads);
                $fdisplay(pfd, "data_writes=%0d", perf_total_data_writes);
                $fdisplay(pfd, "control_flow=%0d", perf_branch_instructions);
                $fdisplay(pfd, "mispredicts=%0d", perf_mispredicted_branches);
                $fdisplay(pfd, "jalr_pred_correct=%0d", perf_jalr_predicted_correct);
                $fdisplay(pfd, "jalr_pred_incorrect=%0d", perf_jalr_predicted_incorrect);
                $fdisplay(pfd, "jalr_unpredicted=%0d", perf_jalr_unpredicted);
                $fdisplay(pfd, "return_pred_correct=%0d", perf_return_predicted_correct);
                $fdisplay(pfd, "return_pred_incorrect=%0d", perf_return_predicted_incorrect);
                $fdisplay(pfd, "return_unpredicted=%0d", perf_return_unpredicted);
                $fdisplay(pfd, "rob_full_cycles=%0d", perf_rob_full_cycles);
                $fdisplay(pfd, "rob_empty_cycles=%0d", perf_rob_empty_cycles);
                $fdisplay(pfd, "iq_full_cycles=%0d", perf_iq_full_cycles);
                $fdisplay(pfd, "memq_full_cycles=%0d", perf_memq_full_cycles);
                $fdisplay(pfd, "bstack_full_cycles=%0d", perf_bstack_full_cycles);
                $fdisplay(pfd, "freelist_stall_cycles=%0d", perf_freelist_stall_cycles);
                $fdisplay(pfd, "dispatch_stall_cycles=%0d", perf_dispatch_stall_cycles);
                $fdisplay(pfd, "branch_presented_cycles=%0d", perf_branch_presented);
                $fdisplay(pfd, "bstack_branch_block_cycles=%0d", perf_bstack_branch_block);
`ifdef DFE_STATS
                $fdisplay(pfd, "predict_stall_raw=%0d", perf_ps_raw);
                $fdisplay(pfd, "predict_stall_sole=%0d", perf_ps_sole);
                $fdisplay(pfd, "predict_stall_sole_resident=%0d", perf_ps_sole_res);
`endif
`ifdef DUAL_BRANCH_COUNT
                $fdisplay(pfd, "grp_branch_cut_cycles=%0d", perf_grp_branch_cut);
                $fdisplay(pfd, "grp_2nd_branch_cycles=%0d", perf_grp_2nd_branch);
`endif
                $fdisplay(pfd, "commit_starved_backend=%0d", perf_commit_starved_be);
                $fdisplay(pfd, "commit_starved_frontend=%0d", perf_commit_starved_fe);
`ifdef ROBHEAD_STATS
                $fdisplay(pfd, "cs_gated=%0d", perf_cs_gated);
                $fdisplay(pfd, "cs_window=%0d", perf_cs_window);
                $fdisplay(pfd, "cs_latency=%0d", perf_cs_latency);
`endif
                $fdisplay(pfd, "compressed_retired=%0d", perf_compressed_retired);
                $fdisplay(pfd, "l1i_miss=%0d", perf_l1i_miss_cnt);
                $fdisplay(pfd, "l1d_miss=%0d", perf_l1d_miss_cnt);
                $fdisplay(pfd, "l1d_wb=%0d", perf_l1d_wb_cnt);
                $fdisplay(pfd, "load_lat_sum=%0d", perf_load_lat_sum);
                $fdisplay(pfd, "load_lat_count=%0d", perf_load_lat_count);
                for (int i = 0; i <= OOO_WIDTH; i += 1)
                    $fdisplay(pfd, "retire_hist_%0d=%0d", i, perf_retire_hist[i]);
                for (int i = 0; i <= OOO_WIDTH; i += 1)
                    $fdisplay(pfd, "dispatch_hist_%0d=%0d", i, perf_dispatch_hist[i]);
`ifdef DISPATCH_STATS
                $fdisplay(pfd, "ds_redirect=%0d", perf_ds_redirect);
                $fdisplay(pfd, "ds_trap=%0d", perf_ds_trap);
                $fdisplay(pfd, "ds_bstack_late=%0d", perf_ds_bstack_late);
                $fdisplay(pfd, "ds_wfi=%0d", perf_ds_wfi);
                $fdisplay(pfd, "ds_irqdrain=%0d", perf_ds_irqdrain);
                $fdisplay(pfd, "ds_terminal=%0d", perf_ds_terminal);
                $fdisplay(pfd, "ds_control=%0d", perf_ds_control);
                $fdisplay(pfd, "ds_serial=%0d", perf_ds_serial);
                $fdisplay(pfd, "ds_fencei=%0d", perf_ds_fencei);
                $fdisplay(pfd, "ds_predict=%0d", perf_ds_predict);
                $fdisplay(pfd, "ds_fflags=%0d", perf_ds_fflags);
                $fdisplay(pfd, "ds_supp_other=%0d", perf_ds_supp_other);
                $fdisplay(pfd, "ds_robfull=%0d", perf_ds_robfull);
                $fdisplay(pfd, "ds_iqfull=%0d", perf_ds_iqfull);
                $fdisplay(pfd, "ds_memqfull=%0d", perf_ds_memqfull);
                $fdisplay(pfd, "ds_bstack=%0d", perf_ds_bstack);
                $fdisplay(pfd, "ds_other=%0d", perf_ds_other);
                $fdisplay(pfd, "ds_supp_shadowed=%0d", perf_ds_supp_shadowed);
                $fdisplay(pfd, "dn_nolanes=%0d", perf_dn_nolanes);
                $fdisplay(pfd, "dn_other=%0d", perf_dn_other);
                for (int i = 0; i < 16; i += 1)
                    if (perf_dn_cut[i] != 64'd0)
                        $fdisplay(pfd, "dn_cut_%0d=%0d", i, perf_dn_cut[i]);
                for (int i = 0; i < 16; i += 1)
                    if (perf_dt_cut[i] != 64'd0)
                        $fdisplay(pfd, "dt_cut_%0d=%0d", i, perf_dt_cut[i]);
                for (int i = 0; i <= OOO_WIDTH; i += 1)
                    $fdisplay(pfd, "dt_lost_%0d=%0d", i, perf_dt_lost[i]);
                $fdisplay(pfd, "dt_slots=%0d", perf_dt_slots);
`ifdef CSWHY
                // RTL-emitted legend so the dump keys can never drift from the enum.
                $fdisplay(pfd, "cswhy_classes=NOHEAD,UNWRITTEN,ABORTED,ZERO,LOAD,FLOAD,STORE,FSTORE,ATOMIC,BRANCH,JUMP,MUL,DIV,FP,SER,ALU,UNKNOWN");
                $fdisplay(pfd, "cswhy_states=NA,DONE,SELECT,ALUEXEC,MULEXEC,DIVEXEC,FPEXEC,FUWB,IQPICK,IQOPWAIT,ALUWB,NOWHERE,-,-,-,-,SKEW,RETIRING,ADDRWAIT,STDATA,PARK,LOADWAIT,MLPWAIT,XLATEWALK,XLATEREG,PORT,XFAULT,MEMOTHER,DRAINED,COMPLETING,STWAIT,LOADBLK,TWOBEAT,LOADFIRE");
                $fdisplay(pfd, "cswhy_arms=GATED,WINDOW,LAT");
                for (int a = 0; a < 3; a += 1)
                    for (int c = 0; c < CSWHY_NCLASS; c += 1)
                        for (int st = 0; st < CSWHY_NSTATE; st += 1)
                            if (cswhy_cell[a][c][st] != 64'd0)
                                $fdisplay(pfd, "cswhy_%0d_%0d_%0d=%0d", a, c, st, cswhy_cell[a][c][st]);
                for (int c = 0; c < CSWHY_NCLASS; c += 1)
                    if (cswhy_visits[c] != 64'd0)
                        $fdisplay(pfd, "cswhy_visits_%0d=%0d", c, cswhy_visits[c]);
                $fdisplay(pfd, "cswhy_depart_retire=%0d", cswhy_depart_retire);
                $fdisplay(pfd, "cswhy_depart_other=%0d", cswhy_depart_other);
                // Cross-checks. XC-CLASS and XC-2 compare DIFFERENT modules'
                // derivations, so they are not tautologies; all must be 0.
                $fdisplay(pfd, "cswhy_xc_class=%0d", cswhy_xc_class);
                $fdisplay(pfd, "cswhy_xc_multi=%0d", cswhy_xc_multi);
                $fdisplay(pfd, "cswhy_xc_mem_outside=%0d", cswhy_xc_mem_outside);
                $fdisplay(pfd, "cswhy_xc_pending_bad=%0d", cswhy_xc_pending_bad);
                // XC-3: head-pointer edge count vs the commit unit's own retire
                // histogram -- different blocks, neither derived from the other.
                $fdisplay(pfd, "cswhy_xc3_lhs=%0d", cswhy_depart_retire);
                $fdisplay(pfd, "cswhy_xc3_rhs=%0d", perf_cycle_counter - perf_retire_hist[0]);
`endif
                // Fetch-flush by winning redirect arm: ff_<code> = flush cycles,
                // ffdead_<code> = frontend-starved cycles charged to it (the real cost).
                for (int i = 0; i < 16; i += 1)
                    if (perf_ff[i] != 64'd0) $fdisplay(pfd, "ff_%0d=%0d", i, perf_ff[i]);
                for (int i = 0; i < 16; i += 1)
                    if (perf_ff_dead[i] != 64'd0) $fdisplay(pfd, "ffdead_%0d=%0d", i, perf_ff_dead[i]);
                for (int i = 0; i < 16; i += 1)
                    if (perf_ff_hold[i] != 64'd0) $fdisplay(pfd, "ffhold_%0d=%0d", i, perf_ff_hold[i]);
                for (int i = 0; i < 16; i += 1)
                    if ((perf_ff_dead[i] + perf_ff_hold[i]) != 64'd0)
                        $fdisplay(pfd, "ffcost_%0d=%0d", i, perf_ff_dead[i] + perf_ff_hold[i]);
                $fdisplay(pfd, "ff_total=%0d", perf_ff_total);
                $fdisplay(pfd, "ff_other=%0d", perf_ff[FF_OTHER]);
                $fdisplay(pfd, "btb_suppress_cnt=%0d", perf_btb_suppress);
                $fdisplay(pfd, "pred_redirect_raw=%0d", perf_pred_raw);
                begin
                    logic [63:0] fsum, fdsum, fhsum;
                    fsum = 64'd0; fdsum = 64'd0; fhsum = 64'd0;
                    for (int i = 0; i < 16; i += 1) begin
                        fsum  = fsum  + perf_ff[i];
                        fdsum = fdsum + perf_ff_dead[i];
                        fhsum = fhsum + perf_ff_hold[i];
                    end
                    // ff_other MUST be 0 (a nonzero value means a fetch_flush site this
                    // instrumentation does not know about -- a REACHABLE catch-all, unlike
                    // dstat_zero_residual). ffdead_sum must not exceed dn_nolanes.
                    $fdisplay(pfd, "ff_sum=%0d", fsum);
                    $fdisplay(pfd, "ffdead_sum=%0d", fdsum);
                    $fdisplay(pfd, "ffhold_sum=%0d", fhsum);
                    $fdisplay(pfd, "ffcost_sum=%0d", fdsum + fhsum);
                    $fdisplay(pfd, "ffdead_vs_nolanes=%0d", perf_dn_nolanes - fdsum);
                end
                // Self-checks. NOTE: dstat_zero_residual alone is a TAUTOLOGY -- the
                // ladder has catch-alls on every arm, so it sums to dispatch_hist_0 by
                // construction and can never flag a misattribution. The load-bearing
                // check is dstat_stall_residual, which compares sum(ds_*) against
                // perf_dispatch_stall_cycles -- a counter incremented independently
                // from the raw dispatch_stall signal. Both must be 0.
                begin
                    logic [63:0] ssum;
                    ssum = perf_ds_redirect + perf_ds_trap + perf_ds_bstack_late
                         + perf_ds_wfi + perf_ds_irqdrain + perf_ds_terminal
                         + perf_ds_control + perf_ds_serial + perf_ds_fencei
                         + perf_ds_predict + perf_ds_fflags + perf_ds_supp_other
                         + perf_ds_robfull + perf_ds_iqfull + perf_ds_memqfull
                         + perf_ds_bstack + perf_ds_other;
                    $fdisplay(pfd, "dstat_stall_sum=%0d", ssum);
                    $fdisplay(pfd, "dstat_stall_residual=%0d",
                              perf_dispatch_stall_cycles - ssum);
                end
                begin
                    logic [63:0] zsum;
                    zsum = perf_ds_redirect + perf_ds_trap + perf_ds_bstack_late + perf_ds_wfi
                         + perf_ds_irqdrain + perf_ds_terminal + perf_ds_control
                         + perf_ds_serial + perf_ds_fencei + perf_ds_predict
                         + perf_ds_fflags + perf_ds_supp_other + perf_ds_robfull
                         + perf_ds_iqfull + perf_ds_memqfull + perf_ds_bstack
                         + perf_ds_other + perf_dn_nolanes + perf_dn_other;
                    for (int i = 0; i < 16; i += 1) zsum = zsum + perf_dn_cut[i];
                    $fdisplay(pfd, "dstat_zero_sum=%0d", zsum);
                    $fdisplay(pfd, "dstat_zero_residual=%0d",
                              perf_dispatch_hist[0] - zsum);
                end
`endif
                // P3-M0: LSQ head-blocking reason histogram (0 empty, 1 ready,
                // 2 loadwait, 3 storepark, 4 xlate, 5 memport, 6 other).
                for (int i = 0; i < 7; i += 1)
                    $fdisplay(pfd, "lsq_hr_%0d=%0d", i, perf_lsq_hr[i]);
                // P3 L2a: 0 na, 1 noload, 2 nomatch, 3 partial, 4 full (forwardable)
                for (int i = 0; i < 5; i += 1)
                    $fdisplay(pfd, "lsq_fwd_%0d=%0d", i, perf_lsq_fwd[i]);
                $fclose(pfd);
                $display("PERF: wrote %s", perf_out_path);
            end
        end
    end
`endif /* SIMULATION_18447 */


`ifdef AGENT_DEBUG
    integer dbg_cyc = 0;
    always_ff @(posedge clk) begin
        if (rst_l) begin
            dbg_cyc <= dbg_cyc + 1;
            for (int i = 0; i < OOO_WIDTH; i += 1) begin
                if (retire_valid[i])
                    $display("[%0d] retire[%0d] pc=%h instr=%h rd=%0d wr=%b data=%h exc=%b cause=%0d",
                        dbg_cyc, i, active_commit_packet[i].pc,
                        active_commit_packet[i].instr,
                        active_commit_packet[i].rd,
                        active_commit_packet[i].has_dest,
                        active_commit_packet[i].data,
                        active_commit_packet[i].exception,
                        active_commit_packet[i].exc_cause);
            end
            if (commit_take_trap)
                $display("[%0d] TRAP cause=%0d epc=%h tval=%h priv=%0d->vec=%h",
                    dbg_cyc, commit_exc_cause, commit_trap_epc, commit_exc_tval,
                    cur_priv, tc_vector);
            if (commit_take_int)
                $display("[%0d] INT  epc=%h mip=%h mie=%h priv=%0d->vec=%h",
                    dbg_cyc, commit_int_epc, csr_mip, csr_mie, cur_priv, tc_vector);
            if (commit_take_ret)
                $display("[%0d] RET  from_s=%b epc=%h", dbg_cyc,
                    commit_ret_from_s, trap_redirect_pc);
            if (satp_mode && (dbg_cyc % 1 == 0))
                $display("[%0d] PG pc=%h pgD=%b mrq=%b mva=%h dh=%b du=%b ptwB=%b D=%b F=%b xs=%b xf=%b dle=%b da=%h",
                    dbg_cyc, pc_q, paging_data, mem_req_valid,
                    mem_req_vaddr, dtlb_hit, dtlb_usable, ptw_busy, ptw_done,
                    ptw_fault, lsq_xlate_stall, lsq_xlate_fault, mem_data_load_en,
                    dmem_req_addr);
        end
    end
`endif

`ifdef FPGA_BUILD
    // FB1 debug observability probe: a pure combinational tap off the commit
    // stage. No functional effect -- only wired out under FPGA_BUILD. The OCL
    // debug block (ocl_csr.sv) clocks these into the PC ring / instret counter,
    // the shadow architectural regfile, and the trap log.
    always_comb begin
        dbg_probe = '0;
        for (int i = 0; i < OOO_WIDTH; i++) begin
            dbg_probe.retire_valid[i] = retire_valid[i];
            dbg_probe.retire_pc[i]    = active_commit_packet[i].pc;
            dbg_probe.arch_we[i]      = arch_rd_we[i];
            dbg_probe.arch_rd[i]      = arch_rd[i][ARCH_REG_BITS-1:0];
            dbg_probe.arch_data[i]    = arch_rd_data[i];
        end
        dbg_probe.trap_valid  = commit_take_trap || commit_take_int;
        dbg_probe.trap_is_int = tc_is_int;
        dbg_probe.trap_cause  = tc_cause;
        dbg_probe.trap_epc    = commit_take_int ? commit_int_epc : commit_trap_epc;
        dbg_probe.trap_tval   = tc_is_int ? '0 : commit_exc_tval;
        dbg_probe.halted      = halted_q;
        dbg_probe.hpm_l1i_miss = hpm_l1i_miss;
        dbg_probe.hpm_l1d_miss = hpm_l1d_miss;
        dbg_probe.hpm_l1d_wb   = hpm_l1d_wb;
    end
`endif

endmodule: riscv_core_ooo

`ifdef NIIGO_DSIDE_WB
`undef NIIGO_DSIDE_WB
`endif
