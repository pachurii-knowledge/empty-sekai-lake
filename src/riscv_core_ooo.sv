`include "ooo_types.vh"
`include "superscalar_types.vh"
`include "riscv_abi.vh"
`include "memory_segments.vh"
`include "riscv_priv.vh"

`default_nettype none

module riscv_core_ooo
    import OOO_Types::*;
    import RISCV_ISA::XLEN_BYTES;
    import RISCV_UArch::MEMORY_READ_WIDTH, RISCV_UArch::MEMORY_ADDR_WIDTH;
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
    input wire logic             dmem_resp_valid,
    input wire logic [MEMORY_ADDR_WIDTH-1:0] dmem_resp_addr,
    input wire logic [XLEN-1:0]  dmem_resp_data,

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

    output logic             halted
);

    // Byte-address -> word-address shift for the word-granular memory bus.
    localparam int ADDR_SHIFT = $clog2(XLEN_BYTES);

    import RISCV_ABI::ECALL_ARG_HALT;
    import MemorySegments::USER_TEXT_START;

    localparam int PHYS_READ_PORTS = FU_ISSUE_PORTS + OOO_WIDTH;
    localparam int RAS_DEPTH = 128;
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
    } fetch_meta_t;
    typedef struct packed {
        logic                 valid;
        logic [XLEN-1:0]      pc;
        logic [MEMORY_READ_WIDTH-1:0][XLEN-1:0] data;
        logic                 excpt;
        logic [OOO_WIDTH-1:0] fault_lane;
        logic [4:0]           fault_cause;
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
    logic [OOO_WIDTH-1:0] lane_is_unpredicted_control;
    logic [OOO_WIDTH-1:0] lane_is_call;
    logic [OOO_WIDTH-1:0] lane_is_return;
    logic [OOO_WIDTH-1:0] lane_control_predicted;
    logic [OOO_WIDTH-1:0][XLEN-1:0] lane_predicted_pc;
    logic [OOO_WIDTH-1:0] lane_is_memory;
    logic [OOO_WIDTH-1:0] lane_is_terminal;
    logic [OOO_WIDTH-1:0] lane_is_serializing;
    logic [OOO_WIDTH-1:0] dispatch_valid;
    logic [OOO_WIDTH-1:0] alloc_req;
    logic [OOO_WIDTH-1:0] map_has_dest;
    logic dispatch_stall;
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
    logic csr_retire;

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
    branch_mask_t abort_mask;
    branch_mask_t abort_mask_q;
    branch_mask_t dispatch_branch_mask;

    logic mem_queue_full;
    logic mem_data_load_en;
    logic [MEMORY_ADDR_WIDTH-1:0] mem_data_addr;
    logic [XLEN-1:0] mem_data_store;
    logic [XLEN_BYTES-1:0] mem_data_store_mask;
    logic lsq_store_second_beat;
    logic lsq_store_port_busy;
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

    ooo_fetch_decode FetchDecode (
        .rst_l,
        .fetch_valid(fgrp_valid && !halted_q),
        .instr_mem_excpt(fgrp_excpt),
        .fetch_fault_lane(fgrp_fault_lane & {OOO_WIDTH{fgrp_valid}}),
        .fetch_fault_cause(fgrp_fault_cause),
        .fetch_pc(fgrp_pc),
        .instr(decode_fetch_instr),
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

    priv_csr_file CSRFile (
        .clk,
        .rst_l,
        .retire(csr_retire),
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
`ifdef L1D_CACHE
    logic        fencei_pending_q, fencei_pending_next;
    logic [XLEN-1:0] fencei_pc_q, fencei_pc_next;
    assign dcache_flush_req = fencei_pending_q;
`else
    assign dcache_flush_req = 1'b0;
`endif
    logic fencei_block;
`ifdef L1D_CACHE
    assign fencei_block = fencei_pending_q;
`else
    assign fencei_block = 1'b0;
`endif
    logic unused_flush_done;
    assign unused_flush_done = dcache_flush_done;

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
    );

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
                .paddr(fetch_pa + (fpl * 32'd4)),
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
        if (paging_fetch && fetch_perm_fault)
            fetch_fault_lane = {{(OOO_WIDTH-1){1'b0}}, 1'b1};
        else if (fetch_pa_resolved)
            fetch_fault_lane = pmp_fetch_fault_lane & fetch_lane_in_block;
        else
            fetch_fault_lane = '0;
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
            (branch_writeback.instr[6:0] == RISCV_ISA::OP_BRANCH)),
        .update_pc(branch_writeback.pc),
        .update_taken(branch_writeback.redirect_pc != (branch_writeback.pc + 32'd4)),
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

    ooo_dispatch_control DispatchControl (
        .lane_valid,
        .lane_has_dest,
        .lane_is_branch,
        .lane_is_memory,
        .lane_is_terminal,
        .lane_is_serializing,
        .active_list_full(active_full),
        .int_iq_full(int_iq_full),
        .mem_queue_full(mem_queue_full),
        .branch_stack_full(branch_stack_full),
        .free_list_can_allocate(free_can_allocate),
        .free_list_available(free_count_snapshot),
        .suppress_dispatch(redirect_valid || terminal_pending_q ||
            control_pending_q || serial_pending_q || halted_q || irq_drain_q ||
            wfi_wait_q || commit_take_trap || fencei_block),
        .dispatch_valid,
        .dispatch_stall
    );

    active_list ActiveList (
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
            for (int p = 0; p < ALU_ISSUE_PORTS; p += 1) begin
                alu_issue_entry_q[p] <= '0;
            end
        end else begin
            for (int p = 0; p < ALU_ISSUE_PORTS; p += 1) begin
                if (!trap_take && int_issue_valid[p] &&
                        ((int_issue_entry[p].branch_mask & abort_mask) == '0)) begin
                    alu_issue_valid_q[p] <= 1'b1;
                    alu_issue_entry_q[p] <= int_issue_entry[p];
                end else begin
                    alu_issue_valid_q[p] <= 1'b0;
                    alu_issue_entry_q[p] <= '0;
                end
            end
        end
    end

    // Speculative ALU wakeup: an ALU op executing in S2 broadcasts its dest a cycle
    // before its writeback bus appears (gated by abort -- a squashed producer must
    // not wake anyone). has_dest implies prd != 0.
    always_comb begin
        for (int p = 0; p < ALU_ISSUE_PORTS; p += 1) begin
            spec_wake_valid[p] = alu_issue_valid_q[p] &&
                alu_issue_entry_q[p].has_dest &&
                ((alu_issue_entry_q[p].branch_mask & abort_mask) == '0);
            spec_wake_prd[p] = alu_issue_entry_q[p].prd;
        end
    end

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
        .writeback(alu1_writeback)
    );

    assign int_issue_ready[ISSUE_ALU0] = 1'b1;
    assign int_issue_ready[ISSUE_ALU1] = 1'b1;

    ooo_mul_unit MulUnit (
        .clk,
        .rst_l,
        .issue_valid(int_issue_valid[ISSUE_MUL]),
        .issue_ready(int_issue_ready[ISSUE_MUL]),
        .issue_entry(int_issue_entry[ISSUE_MUL]),
        .rs1_data(phys_rs1_data[ISSUE_MUL]),
        .rs2_data(phys_rs2_data[ISSUE_MUL]),
        .abort_mask,
        .flush(trap_take),
        .writeback_ready(mul_writeback_ready),
        .writeback(mul_writeback)
    );

    ooo_div_unit DivUnit (
        .clk,
        .rst_l,
        .issue_valid(int_issue_valid[ISSUE_DIV]),
        .issue_ready(int_issue_ready[ISSUE_DIV]),
        .issue_entry(int_issue_entry[ISSUE_DIV]),
        .rs1_data(phys_rs1_data[ISSUE_DIV]),
        .rs2_data(phys_rs2_data[ISSUE_DIV]),
        .abort_mask,
        .flush(trap_take),
        .writeback_ready(div_writeback_ready),
        .writeback(div_writeback)
    );

    niigo_fp_unit FpUnit (
        .clk,
        .rst_l,
        .issue_valid(int_issue_valid[ISSUE_FP]),
        .issue_ready(int_issue_ready[ISSUE_FP]),
        .issue_entry(int_issue_entry[ISSUE_FP]),
        .rs1_data(phys_rs1_data[ISSUE_FP]),
        .frm(csr_frm),
        .abort_mask,
        .flush(trap_take),
        .writeback_ready(fp_writeback_ready),
        .writeback(fp_writeback)
    );

    load_store_queue LoadStoreQueue (
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
        .store_second_beat(lsq_store_second_beat),
        .store_port_busy(lsq_store_port_busy),
        .head_load_off(dev_load_off),
        .load_writeback
    );

    ooo_writeback_bus WritebackBus (
        .alu0_writeback,
        .alu1_writeback,
        .load_writeback,
        .mul_writeback,
        .div_writeback,
        .fp_writeback,
        .abort_mask_q,
        .mul_writeback_ready,
        .div_writeback_ready,
        .fp_writeback_ready,
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

    ooo_commit_unit CommitUnit (
        .commit_valid(active_commit_valid),
        .commit_packet(active_commit_packet),
        .store_port_busy(lsq_store_port_busy),
        .store_port_ready(dmem_req_ready),
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
        valid_count = '0;

        for (int i = 0; i < OOO_WIDTH; i += 1) begin
            map_rs1[i] = decode_lanes[i].rs1;
            map_rs2[i] = decode_lanes[i].rs2;
            map_rd[i] = decode_lanes[i].rd;
            lane_valid[i] = decode_lanes[i].valid && !decode_lanes[i].kill;
            lane_has_dest[i] = lane_valid[i] && decode_lanes[i].ctrl.rfWrite &&
                (decode_lanes[i].rd != 5'd0);
            lane_is_branch[i] = lane_valid[i] &&
                ((decode_lanes[i].ctrl.pc_source == PC_cond) ||
                 (decode_lanes[i].ctrl.pc_source == PC_uncond) ||
                 (decode_lanes[i].ctrl.pc_source == PC_indirect));
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
            if (lane_valid[i]) begin
                valid_count += 1'b1;
            end
        end
    end

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
            lane_predicted_pc[i] = decode_lanes[i].pc + 32'd4;
            lane_predictor_info[i] = '0;
        end
        ras_redirect_valid = 1'b0;
        ras_redirect_pc = '0;
        predictor_redirect_valid = 1'b0;
        predictor_redirect_pc = '0;
        ras_stack_next = ras_stack_q;
        ras_count_next = branch_restore_valid ?
            ras_checkpoint_count_q[branch_resolve_id] : ras_count_q;
        ras_branch_snapshot_count = ras_count_next;
        // Speculative global history: on a misprediction restore the branch's
        // pre-push checkpoint, then re-push the resolved direction if the
        // resolving branch was conditional (mirrors the RAS recovery above).
        ghr_next = branch_restore_valid ?
            ghr_checkpoint_q[branch_resolve_id] : ghr_q;
        if (branch_restore_valid && branch_resolve_valid &&
                (branch_writeback.instr[6:0] == RISCV_ISA::OP_BRANCH)) begin
            ghr_next = {ghr_next[DIRECT_HISTORY_BITS-2:0],
                (branch_writeback.redirect_pc != branch_writeback.pc + 32'd4)};
        end
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
                if (lane_is_call[i] &&
                        (ras_count_next < RAS_COUNT_BITS'(RAS_DEPTH))) begin
                    ras_stack_next[RAS_INDEX_BITS'(ras_count_next)] =
                        decode_lanes[i].pc + 32'd4;
                    ras_count_next = ras_count_next + 1'b1;
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
                    end
                end else if ((decode_lanes[i].ctrl.pc_source == PC_indirect) &&
                        !lane_is_return[i]) begin
                    lane_predictor_info[i] = indirect_prediction_info;
                    if (indirect_prediction_valid) begin
                        lane_control_predicted[i] = 1'b1;
                        lane_predicted_pc[i] = indirect_prediction_target;
                        predictor_redirect_valid = 1'b1;
                        predictor_redirect_pc = indirect_prediction_target;
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
            rename_packets[i].src2_ready = !decode_lanes[i].uses_rs2 ||
                busy_src2_ready[i];
            rename_packets[i].has_dest = map_has_dest[i];
            rename_packets[i].imm = decode_lanes[i].imm;
            rename_packets[i].branch_mask = dispatch_branch_mask;
            rename_packets[i].branch_id = lane_is_branch[i] ?
                branch_allocate_id : '0;
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

            int_insert_valid[i] = dispatch_valid[i] && !lane_is_memory[i];
            mem_insert_valid[i] = dispatch_valid[i] && lane_is_memory[i];
            branch_allocate |= dispatch_valid[i] && lane_is_branch[i];
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
                            commit_exc_tval  =
                                (active_commit_packet[i].exc_cause ==
                                    RISCV_Priv::EXC_ILLEGAL_INSTR) ?
                                        active_commit_packet[i].instr :
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
                lane_valid[0] ? decode_lanes[0].pc :
                fgrp_valid    ? fgrp_pc :
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
    end

    // ---- B7: frontend control + commit-driven redirects (pc_next,
    // fetch flush/consume/issue, ifetch invalidate, pending interlocks,
    // CSR/FP commit writes, fence.i / sfence / satp / trap redirects) ----
    always_comb begin
        fp_regs_next = fp_regs_q;
        serial_pending_next = serial_pending_q;
        csr_retire = 1'b0;
        csr_commit_write = 1'b0;
        csr_commit_addr = '0;
        csr_commit_wdata = '0;
        csr_fp_fflags_valid = 1'b0;
        csr_fp_fflags = '0;
        tlb_flush = 1'b0;

        pc_next = pc_q;
        fetch_flush = 1'b0;
        fetch_consume = 1'b0;
        fetch_issue = 1'b0;
        ifetch_inval = 1'b0;
`ifdef L1D_CACHE
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
        end else begin
            // Group consumption and dispatch-time prediction redirects. Gated
            // on !dispatch_stall (zero lanes dispatch under a stall, so the
            // presented group must be held and re-presented). Deliberately NOT
            // gated on fetch_xlate_stall: consumption is an explicit event
            // here, so a group that fully dispatches while pc_q's translation
            // is still walking leaves the buffer and cannot dispatch twice
            // (the old frozen-pipe scheme would re-present it).
            if (!dispatch_stall && !halted_q) begin
                if (ras_redirect_valid) begin
                    pc_next = ras_redirect_pc;
                    fetch_flush = 1'b1;
                end else if (predictor_redirect_valid) begin
                    pc_next = predictor_redirect_pc;
                    fetch_flush = 1'b1;
                end else if (dispatched_unpredicted_control) begin
                    // Unpredicted JAL/JALR dispatched: younger fetches are
                    // dead; pc holds until the control transfer resolves.
                    fetch_flush = 1'b1;
                end else if (fgrp_valid && (dispatch_count < valid_count)) begin
                    // Partial dispatch: discard the group and refetch from the
                    // first undispatched instruction.
                    pc_next = decode_lanes[2'(dispatch_count)].pc;
                    fetch_flush = 1'b1;
                    fetch_consume = 1'b1;
                end else if (fgrp_valid) begin
                    // Fully dispatched (or no decodable lanes): pop it.
                    fetch_consume = 1'b1;
                end
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
                pc_next = sequential_next_pc;
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
                csr_retire = 1'b1;
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
`ifdef L1D_CACHE
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
        end

`ifdef L1D_CACHE
        // fence.i L1D-writeback hold (highest priority while active; fence.i is
        // serializing and interrupts are gated above, so nothing competes).
        // Hold pc + keep the fetch pipe flushed until the memsys finishes
        // writing back every dirty L1D line, then invalidate the L1I and redirect
        // to the instruction after the fence.i.
        if (fencei_pending_q) begin
            fetch_flush = 1'b1;
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
`ifdef L1D_CACHE
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
`ifdef L1D_CACHE
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
            if (frontend_stall || dispatched_unpredicted_control) begin
                perf_frontend_stall_cycles = perf_frontend_stall_cycles + 64'd1;
            end
            if (dmem_req_valid && !dmem_req_write && dmem_req_ready) begin
                perf_total_data_reads = perf_total_data_reads + 64'd1;
            end
            if (dmem_req_valid && dmem_req_write && dmem_req_ready) begin
                perf_total_data_writes = perf_total_data_writes + 64'd1;
            end

            for (int i = 0; i < OOO_WIDTH; i += 1) begin
                if (dispatch_valid[i]) begin
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
                            branch_writeback.redirect_pc != (branch_writeback.pc + 32'd4),
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

    initial begin
        wait (halted);
        $display("FINAL OOO PERFORMANCE COUNTERS:");
        $display("Total cycles: %0d", perf_cycle_counter);
        $display("Instructions dispatched: %0d", perf_dispatch_counter);
        $display("Instructions retired: %0d", perf_retire_counter);
        $display("  ALU instructions: %0d", perf_alu_instructions);
        $display("  Load instructions: %0d", perf_load_instructions);
        $display("  Store instructions: %0d", perf_store_instructions);
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

endmodule: riscv_core_ooo
