`ifndef OOO_TYPES_VH_
`define OOO_TYPES_VH_

`include "internal_defines.vh"
`include "riscv_isa.vh"

package OOO_Types;

    // Re-export the ISA width as an OOO_Types localparam so modules that do
    // `import OOO_Types::*` see XLEN (a wildcard import does not chain the
    // symbols a package itself imported).
    localparam int XLEN = RISCV_ISA::XLEN;

    localparam int OOO_WIDTH = 4;
    localparam int PHYS_REGS = 64;
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
        logic          halted;
        logic          exception;
        logic [4:0]    exc_cause;
    } commit_packet_t;

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
