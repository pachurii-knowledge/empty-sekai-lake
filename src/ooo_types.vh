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
    localparam int ACTIVE_LIST_SIZE = 32;
    localparam int INT_IQ_SIZE = 16;
    localparam int MEM_Q_SIZE = 16;
    localparam int BRANCH_STACK_SIZE = 4;
    localparam int ALU_ISSUE_PORTS = 2;
    localparam int FU_ISSUE_PORTS = 5;
    localparam int WB_SOURCES = 6;
    localparam int ISSUE_ALU0 = 0;
    localparam int ISSUE_ALU1 = 1;
    localparam int ISSUE_MUL = 2;
    localparam int ISSUE_DIV = 3;
    localparam int ISSUE_FP = 4;

    localparam int ARCH_REG_BITS = 5;
    localparam int PHYS_REG_BITS = $clog2(PHYS_REGS);
    localparam int ACTIVE_ID_BITS = $clog2(ACTIVE_LIST_SIZE);
    localparam int BRANCH_ID_BITS = $clog2(BRANCH_STACK_SIZE);

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
