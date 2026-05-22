`ifndef OOO_TYPES_VH_
`define OOO_TYPES_VH_

`include "internal_defines.vh"

package OOO_Types;

    localparam int OOO_WIDTH = 4;
    localparam int PHYS_REGS = 64;
    localparam int FP_REGS = 32;
    localparam int ACTIVE_LIST_SIZE = 32;
    localparam int INT_IQ_SIZE = 16;
    localparam int MEM_Q_SIZE = 16;
    localparam int BRANCH_STACK_SIZE = 4;

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

    typedef struct packed {
        logic        valid;
        logic        predicted_taken;
        logic        predicted_target_valid;
        logic [31:0] predicted_target;
        logic [1:0]  provider;
        logic [9:0]  base_index;
        logic [9:0]  index0;
        logic [9:0]  index1;
        logic [9:0]  index2;
        logic [9:0]  tag0;
        logic [9:0]  tag1;
        logic [9:0]  tag2;
    } predictor_info_t;

    typedef struct packed {
        logic          valid;
        logic [31:0]   pc;
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
        logic [31:0]   imm;
        branch_mask_t  branch_mask;
        branch_id_t    branch_id;
        active_id_t    active_id;
        logic          control_predicted;
        logic [31:0]   predicted_pc;
        predictor_info_t predictor_info;
        arch_reg_t     fp_rs1;
        arch_reg_t     fp_rs2;
        arch_reg_t     fp_rs3;
        arch_reg_t     fp_rd;
        fp_reg_data_t   fp_src1_data;
        fp_reg_data_t   fp_src2_data;
        fp_reg_data_t   fp_src3_data;
    } rename_packet_t;

    typedef struct packed {
        logic          valid;
        logic [31:0]   pc;
        logic [31:0]   instr;
        ctrl_signals_t ctrl;
        phys_reg_t     prs1;
        phys_reg_t     prs2;
        phys_reg_t     prd;
        logic          src1_ready;
        logic          src2_ready;
        logic          has_dest;
        logic [31:0]   imm;
        branch_mask_t  branch_mask;
        branch_id_t    branch_id;
        active_id_t    active_id;
        logic          control_predicted;
        logic [31:0]   predicted_pc;
        predictor_info_t predictor_info;
        arch_reg_t     fp_rs1;
        arch_reg_t     fp_rs2;
        arch_reg_t     fp_rs3;
        arch_reg_t     fp_rd;
        fp_reg_data_t   fp_src1_data;
        fp_reg_data_t   fp_src2_data;
        fp_reg_data_t   fp_src3_data;
    } issue_entry_t;

    typedef struct packed {
        logic          valid;
        active_id_t    active_id;
        logic [31:0]   pc;
        logic [31:0]   instr;
        phys_reg_t     prd;
        logic          has_dest;
        logic [31:0]   data;
        branch_mask_t  branch_mask;
        logic          branch_valid;
        branch_id_t    branch_id;
        logic          branch_mispredict;
        logic [31:0]   redirect_pc;
        logic          control_predicted;
        logic [31:0]   predicted_pc;
        predictor_info_t predictor_info;
        logic          fp_write;
        arch_reg_t     fp_rd;
        fp_reg_data_t   fp_data;
        logic          csr_write;
        logic [11:0]   csr_addr;
        logic [31:0]   csr_wdata;
        logic          exception;
        logic          halted;
    } writeback_packet_t;

    typedef struct packed {
        logic          valid;
        active_id_t    active_id;
        arch_reg_t     rd;
        phys_reg_t     prd;
        phys_reg_t     old_prd;
        logic          has_dest;
        logic [31:0]   pc;
        logic [31:0]   instr;
        logic [31:0]   data;
        logic          fp_write;
        arch_reg_t     fp_rd;
        fp_reg_data_t   fp_data;
        logic          csr_write;
        logic [11:0]   csr_addr;
        logic [31:0]   csr_wdata;
        logic          serializing;
        logic          is_store;
        logic          halted;
        logic          exception;
    } commit_packet_t;

endpackage: OOO_Types

`endif /* OOO_TYPES_VH_ */
