/**
 * riscv_core.sv
 *
 * RISC-V 32-bit Processor
 *
 * ECE 18-447
 * Carnegie Mellon University
 *
 * This is the core part of the processor, and is responsible for executing the
 * instructions and updating the CPU state appropriately.
 *
 * This is where you can start to add code and make modifications to fully
 * implement the processor. You can add any additional files or change and
 * delete files as you need to implement the processor, provided that they are
 * under the src directory. You may not change any files outside the src
 * directory. The only requirement is that there is a riscv_core module with the
 * interface defined below, with the same port names as below.
 *
 * The Makefile will automatically find any files you add, provided they are
 * under the src directory and have either a *.v, *.vh, or *.sv extension. The
 * files may be nested in subdirectories under the src directory as well.
 * Additionally, the build system sets up the include paths so that you can
 * place header files (*.vh) in any subdirectory in the src directory, and
 * include them from anywhere else inside the src directory.
 *
 * The compiler and synthesis tools support both Verilog and System Verilog
 * constructs and syntax, so you can write either Verilog or System Verilog
 * code, or mix both as you please.
 **/

/*----------------------------------------------------------------------------*
 *  You may edit this file and add or change any files in the src directory.  *
 *----------------------------------------------------------------------------*/

// RISC-V Includes
`include "riscv_abi.vh"             // ABI registers and definitions
`include "riscv_isa.vh"             // RISC-V ISA definitions
`include "riscv_priv.vh"            // Privileged ISA (CSR addrs, causes, Sv32)
`include "memory_segments.vh"       // Memory segment starting addresses

// Local Includes
`include "internal_defines.vh"      // Control signals struct, ALU ops

/* A quick switch to enable/disable tracing. Comment out to disable. Please
 * comment this out before submitting your code. You'll also want to comment
 * this out for longer tests, as it will make them run much faster. */
// `define TRACE

// Force the compiler to throw an error if any variables are undeclared
`default_nettype none

/**
 * The core of the RISC-V processor, everything except main memory.
 *
 * This is the RISC-V processor, which, each cycle, fetches the next
 * instruction, executes it, and then updates the register file, memory,
 * and register file appropriately.
 *
 * The memory that the processor interacts with is dual-ported with a
 * single-cycle synchronous write and combinational read. One port is used to
 * fetch instructions, while the other is for loading and storing data.
 *
 * Inputs:
 *  - clk               The global clock for the processor.
 *  - rst_l             The asynchronous, active low reset for the processor.
 *  - instr_mem_excpt   Indicates that an invalid instruction address was given
 *                      to memory.
 *  - data_mem_excpt    Indicates that an invalid address was given to the data
 *                      memory during a load and/or store operation.
 *  - instr             The instruction loaded loaded from the instr_addr
 *                      address in memory.
 *  - data_load         The data loaded from the data_addr address in memory.
 *  - data_load_addr    The address of the data that is being returned on da_load
 *  - data_load_valid   Indicates that the data_load is valid data from memory
 *                      (As opposed to not having initiated a load 8 cycles ago)
 *
 * Outputs:
 *  - data_load_en      Indicates that data from the data_addr address in
 *                      memory should be loaded.
 *  - halted            Indicates that the processor has stopped because of a
 *                      syscall or exception. Used to indicate to the testbench
 *                      to end simulation. Must be held until next clock cycle.
 *  - data_store_mask   Byte-enable bit mask  signal indicating which bytes of data_store
 *                      should be written to the data_addr address in memory.
 *  - instr_addr        The address of the instruction to load from memory.
 *  - instr_stall       stall instruction load from memory if multicycle.
 *  - data_addr         The address of the data to load or store from memory.
 *  - data_stall        stall data load from memory if multicycle.
 *  - data_store        The data to store to the data_addr address in memory.
 **/
module riscv_core_scalar
    import RISCV_ISA::XLEN, RISCV_ISA::XLEN_BYTES;
    import RISCV_UArch::MEMORY_ADDR_WIDTH, RISCV_UArch::MEMORY_READ_WIDTH;
(
    input  logic             clk, rst_l, instr_mem_excpt, data_mem_excpt,
    input  logic [MEMORY_READ_WIDTH-1:0][XLEN-1:0] instr, data_load,
    input  logic [MEMORY_ADDR_WIDTH-1:0]           data_load_addr,
    input  logic             data_load_valid,
    output logic             data_load_en, halted,
    output logic [XLEN_BYTES-1:0]        data_store_mask,
    output logic [MEMORY_ADDR_WIDTH-1:0] instr_addr, data_addr,
    output logic             instr_stall, data_stall,
    output logic [XLEN-1:0]  data_store,
    // MMU page-table-walk port
    output logic [MEMORY_ADDR_WIDTH-1:0] ptw_addr,
    output logic             ptw_we,
    output logic [XLEN-1:0]  ptw_wdata,
    input  logic [XLEN-1:0]  ptw_rdata
);

    // Byte-address -> word-address shift for the word-granular memory bus
    // (2 for RV32's 4-byte words, 3 for RV64's 8-byte words).
    localparam int ADDR_SHIFT = $clog2(XLEN_BYTES);

    assign data_stall = 1'b0;

   /* Import the ISA field types, and the argument to ecall to halt the
     * simulator, and the start of the user text segment. */
    import RISCV_ISA::*;
    import RISCV_ABI::ECALL_ARG_HALT;
    import MemorySegments::USER_TEXT_START;

    //===============================IF1 start=========================//
    // stall for memory 
    logic mem_stall;
    logic dsu_mem_stall;   // raw stall from the data-stall unit
    // ptw_stall (declared in the MMU section) freezes the pipeline during a walk
    // flush signal from EX stage
    logic flush_E;
    // Manage the value of the PC, don't increment if the processor is halted
    logic [XLEN-1:0] pc_F1, npc_plus4_F1, npc_offset_M1, next_pc_F1, branch_target_D, branch_target_E;
    logic 	        branch_taken_D;
    logic PCSrc_M1;
    assign instr_stall = (~rst_l) || mem_stall;


    adder #(XLEN) Next_PC_Adder(.A(pc_F1), .B('d4), .cin(1'b0),
            .sum(npc_plus4_F1), .cout());

    register #(XLEN, USER_TEXT_START) PC_Register(.clk, .rst_l, .en(~halted & ~mem_stall), .clear(1'b0), .D(next_pc_F1),
            .Q(pc_F1));
    // Fetch address is translated by the MMU (identity when paging is off).
    assign instr_addr       = phys_pc[XLEN-1:ADDR_SHIFT];

    //===============================BP IF Logic =========================//

    // BTB Instance
    logic BTB_we; 
    logic [1:0] BTB_history_F1, pred_state_E;
    logic [29:0] tagPC;
    logic [61:0] BTB_write_data, BTB_read_data;
    logic [6:0] BTB_read_addr, BTB_write_addr;
    logic [1:0] BTB_history_E;
    logic [XLEN-1:0] npc_plus4_E;
    logic incorrect_addr_F1;


    // read from BTB
    // assign BTB_read_addr = instr_addr;
    // assign tagPC = BTB_read_data[61:32];
    // assign BTB_history_F1 = BTB_read_data[31:30];
    // // // PC update logic with branch prediction
    // always_comb begin
    //   next_pc_F1 = npc_plus4_F1; 
    //   if (flush_E && ((BTB_history_E == PRED_TAKEN_W) || (BTB_history_E == PRED_TAKEN_S))) begin
    //     next_pc_F1 = incorrect_addr_F1 ? {branch_target_E[31:1], 1'b0} : npc_plus4_E; 
    //   end
    //   else if (flush_E) begin
    //     next_pc_F1 = {branch_target_E[31:1], 1'b0};
    //   end
    //   else begin
    //     if (tagPC == instr_addr) begin
    //         if (BTB_history_F1 == PRED_TAKEN_W || BTB_history_F1 == PRED_TAKEN_S) begin
    //           next_pc_F1 = {BTB_read_data[29:0], 2'd0};
    //         end 
    //         else begin
    //           next_pc_F1 = npc_plus4_F1;
    //         end
    //     end
    //   end
    // end

    // Privileged trap/return redirect (computed in the EX-stage trap block).
    logic        trap_redirect_E;
    logic [XLEN-1:0] trap_target_pc;

    always_comb begin
      next_pc_F1 = npc_plus4_F1;
      if (trap_redirect_E) begin
        next_pc_F1 = trap_target_pc;
      end else if (flush_E) begin
        next_pc_F1 = {branch_target_E[XLEN-1:1], 1'b0};
      end
    end



    // sram_1r_1w BTB (.clk, .rst_l, .we(BTB_we), .read_addr(BTB_read_addr), 
    //           .write_addr(BTB_write_addr), .write_data(BTB_write_data), 
    //           .read_data(BTB_read_data));   


    logic actual_taken_E;
    // Hysteresis counter for the branch predictor
    hysteresis_counter BP_COUNTER (.clk(clk), .rst_l(rst_l), .taken(actual_taken_E), .cur_state(BTB_history_E), .pred_state(pred_state_E));


    //===============================IF1/IF2 Pipeline Registers=========================//
    logic [XLEN-1:0] pc_F2, npc_plus4_F2, next_pc_F2;
    logic [1:0] BTB_history_F2;

    register #(XLEN, USER_TEXT_START) NPC_PLUS4_IF1_IF2_REG(.clk(clk), .rst_l(rst_l), .en(~halted & ~mem_stall), .clear(1'b0), .D(npc_plus4_F1), .Q(npc_plus4_F2));
    register #(2, 2'b0) BTB_HIS_IF1_IF2_REG(.clk(clk), .rst_l(rst_l), .en(~halted & ~mem_stall), .clear(1'b0), .D(BTB_history_F1), .Q(BTB_history_F2));

    register #(XLEN, USER_TEXT_START) PC_IF1_IF2_REG(.clk, .rst_l, .en(~halted & ~mem_stall), .clear(1'b0), .D(pc_F1),
            .Q(pc_F2));

    register #(XLEN, USER_TEXT_START) NXT_PC_IF1_IF2_REG(.clk, .rst_l, .en(~halted & ~mem_stall), .clear(1'b0), .D(next_pc_F1),
            .Q(next_pc_F2));

    
    //===============================IF2/ID Pipeline Registers=========================//

    logic [31:0] instr_D;
    logic [XLEN-1:0] pc_D, next_pc_D;
    logic [1:0]  BTB_history_D;
    logic [XLEN-1:0] npc_plus4_D;
    register #(XLEN, USER_TEXT_START) PC_IF2_ID_REG(.clk, .rst_l, .en(~halted & ~mem_stall), .clear(1'b0), .D(pc_F2), .Q(pc_D));
    register #(2, 2'b0) BTB_HIS_IF2_ID_REG(.clk(clk), .rst_l(rst_l), .en(~halted & ~mem_stall), .clear(1'b0), .D(BTB_history_F2), .Q(BTB_history_D));
    register #(XLEN, USER_TEXT_START) NXT_PC_IF2_D_REG(.clk, .rst_l, .en(~halted & ~mem_stall), .clear(1'b0), .D(next_pc_F2), .Q(next_pc_D));
    register #(XLEN, USER_TEXT_START) NPC_PLUS4_IF2_D_REG(.clk(clk), .rst_l(rst_l), .en(~halted & ~mem_stall), .clear(1'b0), .D(npc_plus4_F2), .Q(npc_plus4_D));

    //===============================ID start=========================//
    // Decode the instruction and generate the control signals.
    // RV64 packs two 32-bit instructions per 64-bit memory word; select the
    // half the (word-aligned) fetch PC points at. RV32: the word is the instr.
    logic [XLEN-1:0] fetch_word_D;
    assign fetch_word_D = instr[0];
`ifdef RV64
    assign instr_D = pc_D[2] ? fetch_word_D[63:32] : fetch_word_D[31:0];
`else
    assign instr_D = fetch_word_D[31:0];
`endif
    ctrl_signals_t ctrl_signals_D;
    logic en;
    riscv_decode Decoder(.rst_l(rst_l), .instr(instr_D), .ctrl_signals(ctrl_signals_D));
    logic x10_halt;

    logic [4:0]     rs1_D, rs2_D, rd_D, rd_W;
    logic [XLEN-1:0] rs1_data_D, rs2_data_D, boffset_D, rd_in_W, se_immediate_D;
    logic [6:0]     opcode_D;
    logic RegWrite_W, rs1_use_D, rs2_use_D;
    assign  rs1_D = ctrl_signals_D.syscall ? X10 :instr_D[19:15];
    assign  opcode_D = instr_D[6:0];
    assign  rs2_D = instr_D[24:20];
    assign  rd_D = instr_D[11:7];
    //rs1 is used if opcode indicates R-type, I-type, S-type, or B-type
    //  (incl. RV64 OP-32 0x3B and OP-IMM-32 0x1B; harmless in RV32 -- unused)
    assign  rs1_use_D = opcode_D == OP_OP || opcode_D == OP_IMM || opcode_D == OP_STORE || opcode_D == OP_BRANCH || opcode_D == OP_SYSTEM || opcode_D == OP_JALR || opcode_D == OP_LOAD || opcode_D == 7'h3B || opcode_D == 7'h1B;
    //rs2 is used if opcode indicates R-type, S-type, or B-type (incl. OP-32)
    assign  rs2_use_D = opcode_D == OP_OP || opcode_D == OP_STORE || opcode_D == OP_BRANCH || opcode_D == 7'h3B;
    always_comb begin
      case(ctrl_signals_D.imm_mode)
        IMM_I: se_immediate_D = {{(XLEN-11){instr_D[31]}}, instr_D[30:20]};
        IMM_S: se_immediate_D = {{(XLEN-12){instr_D[31]}}, instr_D[31:25], instr_D[11:7]};
        IMM_U: se_immediate_D = {{(XLEN-32){instr_D[31]}}, instr_D[31:12], 12'b0};
        IMM_UJ: se_immediate_D = {{(XLEN-20){instr_D[31]}},instr_D[19:12],instr_D[20],instr_D[30:21],1'b0};
        IMM_SB: se_immediate_D = {{(XLEN-12){instr_D[31]}},instr_D[7],instr_D[30:25],instr_D[11:8],1'b0};
        default: se_immediate_D = 'x;
      endcase
    end
    register_file #(.FORWARD(1)) RF(.clk(clk), .rst_l(rst_l), .halted(halted), .rd_we(RegWrite_W),
            .rs1(rs1_D), .rs2(rs2_D), .rd(rd_W), .rd_data(rd_in_W), .rs1_data(rs1_data_D), .rs2_data(rs2_data_D));

    //===============================ID/EX Pipeline Registers=========================//
    logic [XLEN-1:0] pc_E, next_pc_E;
    logic RegWrite_E;
    assign RegWrite_E = ctrl_signals_E.rfWrite;
    logic [XLEN-1:0] rs1_data_E, rs2_data_E, se_immediate_E, boffset_E;
    logic [6:0] opcode_E;
    logic [4:0] rd_E, rs1_E, rs2_E;
    logic rs1_use_E, rs2_use_E, flush_M1, flush_W;
    ctrl_signals_t ctrl_signals_E;
    
    register #(XLEN, USER_TEXT_START) PC_ID_E_REG(.clk, .rst_l, .en(~halted & ~flush_M1 & ~flush_W & ~mem_stall), .clear(flush_E || flush_M1 || flush_W), .D(pc_D), .Q(pc_E));
    register #(XLEN, '0) RS1_DATA_ID_E_REG(.clk, .rst_l, .en(~halted & ~flush_M1 & ~flush_W & ~mem_stall), .clear(flush_E || flush_M1 || flush_W), .D(rs1_data_D), .Q(rs1_data_E));
    register #(XLEN, '0) RS2_DATA_ID_E_REG(.clk, .rst_l, .en(~halted & ~flush_M1 & ~flush_W & ~mem_stall), .clear(flush_E || flush_M1 || flush_W), .D(rs2_data_D), .Q(rs2_data_E));
    register #(XLEN, '0) SE_IMM_ID_E_REG(.clk, .rst_l, .en(~halted & ~flush_M1 & ~flush_W & ~mem_stall), .clear(flush_E || flush_M1 || flush_W), .D(se_immediate_D), .Q(se_immediate_E));
    register #(XLEN, '0) BOFFSET_ID_E_REG(.clk, .rst_l, .en(~halted & ~flush_M1 & ~flush_W & ~mem_stall), .clear(flush_E || flush_M1 || flush_W), .D(boffset_D), .Q(boffset_E));
    register #(5, 5'b0) RD_ID_E_REG(.clk, .rst_l, .en(~halted & ~flush_M1 & ~flush_W & ~mem_stall), .clear(flush_E || flush_M1 || flush_W), .D(rd_D), .Q(rd_E));
    register #($bits(ctrl_signals_D), '0) CTRL_ID_E_REG(.clk, .rst_l, .en(~halted & ~ctrl_signals_D.illegal_instr & ~flush_M1 & ~flush_W & ~mem_stall), .clear(flush_E || flush_M1 || flush_W), .D(ctrl_signals_D), .Q(ctrl_signals_E));
    register #($bits(opcode_D), '0) OPCODE_ID_E_REG(.clk, .rst_l, .en(~halted & ~ctrl_signals_D.illegal_instr & ~mem_stall), .clear(flush_E || flush_M1 || flush_W), .D(opcode_D), .Q(opcode_E));
    register #(5, 5'b0) RS1_ID_E_REG(.clk, .rst_l, .en(~halted & ~flush_M1 & ~flush_W & ~mem_stall), .clear(flush_E || flush_M1 || flush_W), .D(rs1_D), .Q(rs1_E));
    register #(5, 5'b0) RS2_ID_E_REG(.clk, .rst_l, .en(~halted & ~flush_M1 & ~flush_W & ~mem_stall), .clear(flush_E || flush_M1 || flush_W), .D(rs2_D), .Q(rs2_E));
    register #(1, 1'b0) RS1_USE_ID_E_REG(.clk, .rst_l, .en(~halted & ~flush_M1 & ~flush_W & ~mem_stall), .clear(flush_E || flush_M1 || flush_W), .D(rs1_use_D), .Q(rs1_use_E));
    register #(1, 1'b0) RS2_USE_ID_E_REG(.clk, .rst_l, .en(~halted & ~flush_M1 & ~flush_W & ~mem_stall), .clear(flush_E || flush_M1 || flush_W), .D(rs2_use_D), .Q(rs2_use_E));
    register #(2, 2'b0) BTB_HIS_ID_E_REG(.clk(clk), .rst_l(rst_l), .en(~halted & ~flush_M1 & ~flush_W & ~mem_stall), .clear(flush_E || flush_M1 || flush_W), .D(BTB_history_D), .Q(BTB_history_E));
    register #(XLEN, USER_TEXT_START) NXT_PC_D_E_REG(.clk, .rst_l, .en(~halted & ~flush_M1 & ~flush_W & ~mem_stall), .clear(flush_E || flush_M1 || flush_W), .D(next_pc_D), .Q(next_pc_E));
    register #(XLEN, USER_TEXT_START) NPC_PLUS4_D_E_REG(.clk(clk), .rst_l(rst_l), .en(~halted & ~flush_M1 & ~flush_W & ~mem_stall), .clear(flush_E || flush_M1 || flush_W), .D(npc_plus4_D), .Q(npc_plus4_E));

    // Privileged ISA: pipeline the raw instruction and a validity bit into EX so
    // the trap/CSR datapath can act at the (in-order) execute stage.
    logic [31:0] instr_E;
    logic        valid_E;
    register #(32, 32'b0) INSTR_ID_E_REG(.clk, .rst_l, .en(~halted & ~flush_M1 & ~flush_W & ~mem_stall), .clear(flush_E || flush_M1 || flush_W), .D(instr_D), .Q(instr_E));
    register #(1, 1'b0) VALID_ID_E_REG(.clk, .rst_l, .en(~halted & ~flush_M1 & ~flush_W & ~mem_stall), .clear(flush_E || flush_M1 || flush_W), .D(1'b1), .Q(valid_E));

    //===============================EX start=========================//

    logic [XLEN-1:0] base_branch_addr_E, alu_out_E, alu_src1_E, alu_src2_E, rs1_data_muxed_E, rs2_data_muxed_E, alu_out_M1;
    logic [XLEN-1:0] rd_data_M1;
    logic MemRead_E, MemRead_M1;
    assign MemRead_E = ctrl_signals_E.memRead;
    logic [1:0] rs1_fwd_sel_E, rs2_fwd_sel_E, alu_src2_sel_E;
    logic branch_src_sel;
    logic [XLEN-1:0] rd_in_M1;

    assign alu_src1_E = ctrl_signals_E.usePC ? pc_E : (rs1_use_E ? rs1_data_muxed_E : '0);
    assign x10_halt = ctrl_signals_E.syscall && rs1_data_E == ECALL_ARG_HALT;

    logic cmp_out_E, alu_cout_E;
    alu_src2_sel_unit ALU_SRC2_SEL_UNIT(.*);
    mux #(3, XLEN) Rs1_data_Mux (.in({rd_in_W, rd_data_M1, rs1_data_E}), .sel(rs1_fwd_sel_E), .out(rs1_data_muxed_E));
    mux #(3, XLEN) Rs2_data_Mux (.in({rd_in_W, rd_data_M1, rs2_data_E}), .sel(rs2_fwd_sel_E), .out(rs2_data_muxed_E));
    mux #(4, XLEN) Alu_Src2_Mux (.in({rd_in_W, rd_data_M1, rs2_data_E, se_immediate_E}), .sel(alu_src2_sel_E), .out(alu_src2_E));
    riscv_alu ALU(.alu_src1(alu_src1_E), .alu_src2(alu_src2_E), .alu_op(ctrl_signals_E.alu_op), .alu_out(alu_out_E));
    assign cmp_out_E = alu_out_E[0];
    mux #(2, XLEN) Branch_Src_Mux (.in({rs1_data_muxed_E, pc_E}), .sel(branch_src_sel), .out(base_branch_addr_E));
    adder #(XLEN) Branch_Target_Adder(.A(base_branch_addr_E), .B(se_immediate_E), .cin(1'b0), .sum(branch_target_E), .cout());

    //===============================BP EX Logic=========================//
    // write back to BTB and counter 
    // assign BTB_we = ctrl_signals_E.isBranch;
    assign actual_taken_E = cmp_out_E || (ctrl_signals_E.isBranch && (ctrl_signals_E.imm_mode == IMM_UJ || ctrl_signals_E.imm_mode == IMM_I)); // either taken, or not taken but jal or jalr
    // assign BTB_write_data = {pc_E[31:2], pred_state_E, branch_target_E[31:2]};
    // assign BTB_write_addr = pc_E[31:2];

    // select branch source
    always_comb begin
      if (ctrl_signals_E.imm_mode == IMM_I) begin
        branch_src_sel = 1'b1;
      end
      else begin
        branch_src_sel = 1'b0;
      end
    end

    // flush upstream logic if prediction is incorrect
    // always_comb begin
    //   flush_E = 1'b0;
    //   incorrect_addr_F1 = 1'b0; 
    //   if ((~mem_stall) && ctrl_signals_E.isBranch && (~flush_M1 && ~flush_W)) begin
    //     if (actual_taken_E) begin
    //       if (BTB_history_E == PRED_NOT_TAKEN_S || BTB_history_E == PRED_NOT_TAKEN_W) begin
    //         flush_E = 1'b1;
    //       end
    //       if (branch_target_E != next_pc_E) begin
    //         flush_E = 1'b1;
    //         incorrect_addr_F1 = 1'b1; 
    //       end
    //     end
    //     else begin
    //       if (BTB_history_E == PRED_TAKEN_S || BTB_history_E == PRED_TAKEN_W) begin
    //         flush_E = 1'b1;
    //       end
    //     end
    //   end
    // end

    always_comb begin
      flush_E = 1'b0;
      incorrect_addr_F1 = 1'b0; 
      if ((~mem_stall) && ctrl_signals_E.isBranch && (~flush_M1 && ~flush_W)) begin
        if (actual_taken_E) begin
          flush_E = 1'b1;
          incorrect_addr_F1 = 1'b1; 
        end
      end
      // A trap or trap-return squashes the pipeline behind EX using the same
      // single-stage flush machinery the branch resolution uses.
      if (trap_redirect_E) begin
        flush_E = 1'b1;
        incorrect_addr_F1 = 1'b1;
      end
    end

    //===============================Privileged ISA (EX stage)=========================//
    // CSR file, trap controller, and CLINT. Synchronous exceptions and trap
    // returns are resolved at EX (the in-order control-resolution point), so the
    // existing flush machinery squashes younger instructions precisely.

    RISCV_Priv::priv_mode_t cur_priv;
    logic [XLEN-1:0] csr_mstatus, csr_medeleg, csr_mideleg, csr_mie, csr_mip;
    logic [XLEN-1:0] csr_mtvec, csr_stvec, csr_mepc, csr_sepc, csr_satp;
    logic        csr_menvcfg_adue;
    logic [XLEN-1:0] csr_read_data_E;
    logic        csr_read_illegal_E;
    logic [2:0]  csr_frm_E;

    logic [XLEN-1:0] clint_load_data;
    logic        clint_load_hit;
    logic        irq_mtimer, irq_msoft;
    logic [63:0] clint_mtime;

    // CSR access fields for the EX instruction
    logic [11:0] csr_addr_E;
    logic [XLEN-1:0] csr_operand_E;
    logic [XLEN-1:0] csr_wval_E;
    logic        csr_is_E;
    logic        csr_does_write_E;
    logic        csr_we_E;

    assign csr_addr_E   = instr_E[31:20];
    assign csr_is_E     = (ctrl_signals_E.exec_class == EXEC_CSR);
    assign csr_operand_E = ctrl_signals_E.useImm ?
        XLEN'(instr_E[19:15]) : rs1_data_muxed_E;
    assign csr_does_write_E = ctrl_signals_E.csr_write;

    always_comb begin
        unique case (ctrl_signals_E.csr_op)
            CSR_RW, CSR_RWI: csr_wval_E = csr_operand_E;
            CSR_RS, CSR_RSI: csr_wval_E = csr_read_data_E | csr_operand_E;
            CSR_RC, CSR_RCI: csr_wval_E = csr_read_data_E & ~csr_operand_E;
            default:         csr_wval_E = csr_read_data_E;
        endcase
    end

    // Exception detection at EX
    logic exc_illegal_E, exc_ecall_E, exc_ebreak_E;
    logic ecall_halt_E;
    logic priv_illegal_E;        // privileged instruction used at too-low priv
    logic exc_valid_E;
    logic [4:0] exc_cause_E;
    logic [XLEN-1:0] exc_tval_E;

    // mret legal only in M; sret legal in S/M and not when mstatus.TSR && S
    assign priv_illegal_E = valid_E && (
        (ctrl_signals_E.is_mret && (cur_priv != RISCV_Priv::PRIV_M)) ||
        (ctrl_signals_E.is_sret && ((cur_priv == RISCV_Priv::PRIV_U) ||
            ((cur_priv == RISCV_Priv::PRIV_S) &&
             csr_mstatus[RISCV_Priv::MSTATUS_TSR_BIT]))) ||
        (csr_is_E && (csr_read_illegal_E ||
            (csr_does_write_E && (csr_addr_E[11:10] == 2'b11)))));

    assign ecall_halt_E = ctrl_signals_E.syscall &&
        ((rs1_data_muxed_E == 32'ha) || (rs1_data_muxed_E == 32'hb));

    assign exc_illegal_E = valid_E &&
        (ctrl_signals_E.illegal_instr || priv_illegal_E);
    assign exc_ebreak_E  = valid_E && ctrl_signals_E.is_ebreak;
    assign exc_ecall_E   = valid_E && ctrl_signals_E.syscall && !ecall_halt_E;

    // Data-side fault signals (driven by the MMU block below).
    logic data_pagefault_E;
    logic data_access_fault_E;
    logic ifault_E, ifault_acc_E;

    always_comb begin
        exc_valid_E = 1'b0;
        exc_cause_E = 5'd0;
        exc_tval_E  = 32'b0;
        if (ifault_E) begin
            // Instruction fetch fault is associated with this instruction.
            exc_valid_E = 1'b1;
            exc_cause_E = ifault_acc_E ? RISCV_Priv::EXC_INSTR_ACCESS
                                       : RISCV_Priv::EXC_INSTR_PAGE_FAULT;
            exc_tval_E  = pc_E;
        end else if (exc_illegal_E) begin
            exc_valid_E = 1'b1;
            exc_cause_E = RISCV_Priv::EXC_ILLEGAL_INSTR;
            exc_tval_E  = instr_E;
        end else if (exc_ebreak_E) begin
            exc_valid_E = 1'b1;
            exc_cause_E = RISCV_Priv::EXC_BREAKPOINT;
            exc_tval_E  = pc_E;
        end else if (exc_ecall_E) begin
            exc_valid_E = 1'b1;
            unique case (cur_priv)
                RISCV_Priv::PRIV_U: exc_cause_E = RISCV_Priv::EXC_ECALL_U;
                RISCV_Priv::PRIV_S: exc_cause_E = RISCV_Priv::EXC_ECALL_S;
                default:            exc_cause_E = RISCV_Priv::EXC_ECALL_M;
            endcase
        end else if (data_pagefault_E) begin
            exc_valid_E = 1'b1;
            exc_cause_E = ctrl_signals_E.memWrite ?
                RISCV_Priv::EXC_STORE_PAGE_FAULT : RISCV_Priv::EXC_LOAD_PAGE_FAULT;
            exc_tval_E  = alu_out_E;
        end else if (data_access_fault_E) begin
            exc_valid_E = 1'b1;
            exc_cause_E = ctrl_signals_E.memWrite ?
                RISCV_Priv::EXC_STORE_ACCESS : RISCV_Priv::EXC_LOAD_ACCESS;
            exc_tval_E  = alu_out_E;
        end
    end

    // Trap controller decision
    logic        tc_trap_valid, tc_is_int;
    logic [4:0]  tc_cause;
    RISCV_Priv::priv_mode_t tc_target;
    logic [XLEN-1:0] tc_vector;
    logic        gate_E;     // EX instruction is committable this cycle

    assign gate_E = (~mem_stall) && (~flush_M1) && (~flush_W) && (~halted);

    trap_controller TrapCtrl (
        .priv(cur_priv),
        .mstatus(csr_mstatus),
        .mie_csr(csr_mie),
        .mip_csr(csr_mip),
        .medeleg(csr_medeleg),
        .mideleg(csr_mideleg),
        .mtvec(csr_mtvec),
        .stvec(csr_stvec),
        .exc_valid(exc_valid_E && gate_E),
        .exc_cause(exc_cause_E),
        .trap_valid(tc_trap_valid),
        .trap_is_interrupt(tc_is_int),
        .trap_cause(tc_cause),
        .trap_target_priv(tc_target),
        .trap_vector(tc_vector)
    );

    // An interrupt is only injected when a real instruction occupies EX.
    logic take_trap_E, take_ret_E;
    assign take_trap_E = gate_E && tc_trap_valid &&
        (tc_is_int ? valid_E : 1'b1);
    assign take_ret_E  = gate_E && !take_trap_E && valid_E &&
        (ctrl_signals_E.is_mret || ctrl_signals_E.is_sret) && !priv_illegal_E;

    assign trap_redirect_E = take_trap_E || take_ret_E;
    always_comb begin
        if (take_trap_E) begin
            trap_target_pc = tc_vector;
        end else if (take_ret_E) begin
            trap_target_pc = ctrl_signals_E.is_sret ? csr_sepc : csr_mepc;
        end else begin
            trap_target_pc = 32'b0;
        end
    end

    // CSR write commits at EX when the CSR instruction is not trapping.
    assign csr_we_E = gate_E && valid_E && csr_is_E && csr_does_write_E &&
        !take_trap_E && !exc_valid_E;

    priv_csr_file CSRFile (
        .clk,
        .rst_l,
        .retire(gate_E && valid_E && !take_trap_E),
        .mtime(clint_mtime),
        .read_addr(csr_addr_E),
        .read_data(csr_read_data_E),
        .read_illegal(csr_read_illegal_E),
        .read_addr1(12'b0),
        .read_data1(),
        .read_illegal1(),
        .write_valid(csr_we_E),
        .write_addr(csr_addr_E),
        .write_data(csr_wval_E),
        .fp_fflags_valid(1'b0),
        .fp_fflags(5'b0),
        .frm_value(csr_frm_E),
        .irq_m_timer(irq_mtimer),
        .irq_m_software(irq_msoft),
        .irq_m_external(1'b0),
        .irq_s_external(1'b0),
        .trap_valid(take_trap_E),
        .trap_is_interrupt(tc_is_int),
        .trap_cause(tc_cause),
        .trap_epc(pc_E),
        .trap_tval(tc_is_int ? 32'b0 : exc_tval_E),
        .trap_target_priv(tc_target),
        .ret_valid(take_ret_E),
        .ret_from_s(ctrl_signals_E.is_sret),
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
        .pmpcfg_o(pmpcfg_arr),
        .pmpaddr_o(pmpaddr_arr),
        .menvcfg_adue(csr_menvcfg_adue)
    );

    clint Clint (
        .clk,
        .rst_l,
        .store_en(ctrl_signals_M1.memWrite && ~mem_stall),
        .store_waddr(data_addr),
        .store_wdata(data_store),
        .store_mask(data_store_mask),
        .load_addr(cache_addr),
        .load_hit(clint_load_hit),
        .load_data(clint_load_data),
        .irq_m_timer(irq_mtimer),
        .irq_m_software(irq_msoft),
        .mtime_out(clint_mtime)
    );

    // Pipeline the CSR read result to M1 for the rd writeback path.
    logic [XLEN-1:0] csr_read_data_M1;
    register #(XLEN, '0) CSR_RDATA_E_M1_REG(.clk, .rst_l, .en(~halted & ~mem_stall), .clear(1'b0), .D(csr_read_data_E), .Q(csr_read_data_M1));

    //===============================Sv32 MMU=========================//
    // satp / mstatus-derived translation context
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
    // Effective privilege for data accesses honours MPRV; instruction fetch
    // always uses the current privilege.
    assign priv_data    = mstatus_mprv ? mpp_mode : cur_priv;
    assign paging_fetch = satp_mode && (cur_priv  != RISCV_Priv::PRIV_M);
    assign paging_data  = satp_mode && (priv_data != RISCV_Priv::PRIV_M);

    // Compute a physical byte address from a leaf translation found at the
    // given level (level > 0 substitutes the VA's low VPN slices into the
    // superpage's PPN).
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
            2'd0: if (!perm[RISCV_Priv::PTE_X]) fail = 1'b1;                       // fetch
            2'd1: if (!(perm[RISCV_Priv::PTE_R] ||
                       (perm[RISCV_Priv::PTE_X] && mxr))) fail = 1'b1;             // load
            2'd2: if (!perm[RISCV_Priv::PTE_W]) fail = 1'b1;                       // store
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

    // --- shared PTW <-> memory port ---
    logic        ptw_req, ptw_done, ptw_fault, ptw_busy;
    logic [1:0]  ptw_level;
    logic [RISCV_Priv::VM_PPN_W-1:0] ptw_ppn;
    logic [7:0]  ptw_perm;
    logic [RISCV_Priv::VM_VPN_W-1:0] ptw_vpn;
    logic [1:0]  ptw_access;
    RISCV_Priv::priv_mode_t ptw_priv;
    logic [XLEN-1:0] ptw_mem_addr;

    // --- DTLB lookup (data side, at EX) ---
    logic        dmem_op_E;
    logic [1:0]  data_acc;
    logic        dtlb_hit;
    logic [RISCV_Priv::VM_PPN_W-1:0] dtlb_ppn;
    logic [7:0]  dtlb_perm;
    logic [1:0]  dtlb_level;
    logic        d_need_ad;
    logic        dtlb_usable;
    logic        data_need_walk;
    logic        data_noncanon, fetch_noncanon;

    assign dmem_op_E = (ctrl_signals_E.memRead || ctrl_signals_E.memWrite) && valid_E;
    assign data_acc  = ctrl_signals_E.memWrite ? 2'd2 : 2'd1;
    assign d_need_ad = !dtlb_perm[RISCV_Priv::PTE_A] ||
                       ((data_acc == 2'd2) && !dtlb_perm[RISCV_Priv::PTE_D]);
`ifdef RV64
    // Sv39 canonical check: VA bits [63:39] must equal bit 38. A truncated
    // VPN lookup could falsely hit the TLB, so the access is sent to the
    // walker, which page-faults it without walking.
    assign data_noncanon  = paging_data &&
        (alu_out_E[XLEN-1:39] != {(XLEN-39){alu_out_E[38]}});
    assign fetch_noncanon = paging_fetch &&
        (pc_F1[XLEN-1:39] != {(XLEN-39){pc_F1[38]}});
`else
    assign data_noncanon  = 1'b0;
    assign fetch_noncanon = 1'b0;
`endif
    assign dtlb_usable  = dtlb_hit && !d_need_ad && !data_noncanon;
    assign data_need_walk = paging_data && dmem_op_E && !dtlb_usable &&
        (~flush_M1) && (~flush_W);

    // --- ITLB lookup (fetch side, at F1) ---
    logic        itlb_hit, itlb_usable;
    logic [RISCV_Priv::VM_PPN_W-1:0] itlb_ppn;
    logic [7:0]  itlb_perm;
    logic [1:0]  itlb_level;
    logic        fetch_need_walk;
    logic [XLEN-1:0] phys_pc;

    assign itlb_usable = itlb_hit && !fetch_noncanon;
    assign fetch_need_walk = paging_fetch && !itlb_usable;

    // --- PTW arbitration: data side has priority over fetch ---
    logic ptw_for_data;
    assign ptw_for_data = data_need_walk;
    assign ptw_req    = data_need_walk || fetch_need_walk;
    assign ptw_vpn    = ptw_for_data ?
        alu_out_E[RISCV_Priv::VM_VPN_W+11:12] :
        pc_F1[RISCV_Priv::VM_VPN_W+11:12];
    assign ptw_access = ptw_for_data ? data_acc : 2'd0;
    assign ptw_priv   = ptw_for_data ? priv_data : cur_priv;

    // SFENCE.VMA flushes both TLBs (modeled as a full flush) at EX.
    logic tlb_flush;
    assign tlb_flush = valid_E && ctrl_signals_E.is_sfence_vma && gate_E;

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
        .mem_req(),
        .mem_we(ptw_we),
        .mem_is_write(),
        .mem_addr(ptw_mem_addr),
        .mem_wdata(ptw_wdata),
        .mem_ack(1'b1),
        .mem_rdata({ptw_rdata}),
        .pte_pmp_fault(1'b0),   // scalar prototype: PMP-on-PTE not modelled
        .busy(ptw_busy),
        .done(ptw_done),
        .fault(ptw_fault),
        .fault_access(),
        .ppn(ptw_ppn),
        .perm(ptw_perm),
        .leaf_level(ptw_level)
    );
    assign ptw_addr = ptw_mem_addr[XLEN-1:ADDR_SHIFT];

    mmu_tlb #(.ENTRIES(16)) ITLB (
        .clk, .rst_l,
        .lookup_en(paging_fetch),
        .lookup_vpn(pc_F1[RISCV_Priv::VM_VPN_W+11:12]),
        .lookup_asid(satp_asid),
        .hit(itlb_hit), .hit_ppn(itlb_ppn), .hit_perm(itlb_perm),
        .hit_level(itlb_level),
        .fill_en(ptw_done && !ptw_fault && !ptw_for_data),
        .fill_vpn(pc_F1[RISCV_Priv::VM_VPN_W+11:12]),
        .fill_asid(satp_asid),
        .fill_ppn(ptw_ppn), .fill_perm(ptw_perm), .fill_level(ptw_level),
        .flush_en(tlb_flush)
    );

    mmu_tlb #(.ENTRIES(16)) DTLB (
        .clk, .rst_l,
        .lookup_en(paging_data && dmem_op_E),
        .lookup_vpn(alu_out_E[RISCV_Priv::VM_VPN_W+11:12]),
        .lookup_asid(satp_asid),
        .hit(dtlb_hit), .hit_ppn(dtlb_ppn), .hit_perm(dtlb_perm),
        .hit_level(dtlb_level),
        .fill_en(ptw_done && !ptw_fault && ptw_for_data),
        .fill_vpn(alu_out_E[RISCV_Priv::VM_VPN_W+11:12]),
        .fill_asid(satp_asid),
        .fill_ppn(ptw_ppn), .fill_perm(ptw_perm), .fill_level(ptw_level),
        .flush_en(tlb_flush)
    );

    // PTW stall freezes the entire pipeline during a walk (deasserts on done).
    logic ptw_stall;
    assign ptw_stall = ptw_req && !ptw_done;

    // --- Resolve the data physical address at EX ---
    logic [XLEN-1:0] pa_E;
    logic        data_from_ptw;
    assign data_from_ptw = ptw_for_data && ptw_done && !ptw_fault;
    always_comb begin
        if (!paging_data) begin
            pa_E = alu_out_E;
        end else if (dtlb_usable) begin
            pa_E = make_pa(dtlb_ppn, dtlb_level, alu_out_E);
        end else if (data_from_ptw) begin
            pa_E = make_pa(ptw_ppn, ptw_level, alu_out_E);
        end else begin
            pa_E = alu_out_E;     // walk in progress; address unused until done
        end
    end
    // Data page fault: a usable TLB hit that fails permission, or a faulting walk
    always_comb begin
        data_pagefault_E = 1'b0;
        if (paging_data && dmem_op_E) begin
            if (dtlb_usable && perm_bad(dtlb_perm, data_acc, priv_data,
                    mstatus_sum, mstatus_mxr))
                data_pagefault_E = 1'b1;
            else if (ptw_for_data && ptw_done && ptw_fault)
                data_pagefault_E = 1'b1;
        end
    end

    // --- PMP on the data physical address ---
    logic [31:0] pmpcfg_arr [4];
    logic [XLEN-1:0] pmpaddr_arr [16];
    logic        data_pmp_fault;
    pmp_checker DataPMP (
        .paddr(pa_E), .access(data_acc), .priv(priv_data),
        .pmpcfg(pmpcfg_arr), .pmpaddr(pmpaddr_arr), .fault(data_pmp_fault)
    );
    assign data_access_fault_E = dmem_op_E &&
        (!paging_data ? data_pmp_fault
                      : (dtlb_usable || data_from_ptw) && data_pmp_fault);

    // --- Resolve the fetch physical address at F1 ---
    logic        fetch_from_ptw;
    assign fetch_from_ptw = (!ptw_for_data) && ptw_done && !ptw_fault;
    always_comb begin
        if (!paging_fetch) begin
            phys_pc = pc_F1;
        end else if (itlb_usable) begin
            phys_pc = make_pa(itlb_ppn, itlb_level, pc_F1);
        end else if (fetch_from_ptw) begin
            phys_pc = make_pa(ptw_ppn, ptw_level, pc_F1);
        end else begin
            phys_pc = pc_F1;
        end
    end
    // Instruction page/access fault detected at fetch, latched for the pipe.
    logic ifault_F1, ifault_acc_F1;
    always_comb begin
        ifault_F1     = 1'b0;
        ifault_acc_F1 = 1'b0;
        if (paging_fetch) begin
            if (itlb_usable && perm_bad(itlb_perm, 2'd0, cur_priv,
                    mstatus_sum, mstatus_mxr))
                ifault_F1 = 1'b1;
            else if ((!ptw_for_data) && ptw_done && ptw_fault)
                ifault_F1 = 1'b1;
        end
    end

    // Pipeline the instruction fetch fault alongside the PC (F1 -> F2 -> D -> E).
    logic ifault_F2, ifault_D;
    register #(1, 1'b0) IFAULT_IF1_IF2(.clk, .rst_l, .en(~halted & ~mem_stall), .clear(1'b0), .D(ifault_F1), .Q(ifault_F2));
    register #(1, 1'b0) IFAULT_IF2_ID(.clk, .rst_l, .en(~halted & ~mem_stall), .clear(1'b0), .D(ifault_F2), .Q(ifault_D));
    register #(1, 1'b0) IFAULT_ID_E(.clk, .rst_l, .en(~halted & ~flush_M1 & ~flush_W & ~mem_stall), .clear(flush_E || flush_M1 || flush_W), .D(ifault_D), .Q(ifault_E));
    assign ifault_acc_E = 1'b0;   // (PMP-on-fetch access faults not yet split out)

    // Pipeline the data physical address to M1 for the memory access.
    logic [XLEN-1:0] pa_M1;
    register #(XLEN, '0) PA_E_M1_REG(.clk, .rst_l, .en(~halted & ~mem_stall), .clear(1'b0), .D(pa_E), .Q(pa_M1));

    //===============================EX/MEM1 Pipeline Registers=========================//

    logic [XLEN-1:0] rs1_data_M1, rs2_data_M1, branch_target_M1, pc_M1;
    logic cmp_out_M1;
    ctrl_signals_t ctrl_signals_M1;
    logic [4:0] rd_M1;
    logic bcond_M1;;


    register #(XLEN, '0) RS1_DATA_E_M1_REG(.clk, .rst_l, .en(~halted & ~mem_stall), .clear(1'b0), .D(rs1_data_muxed_E), .Q(rs1_data_M1));
    register #(XLEN, '0) ALU_OUT_E_M1_REG(.clk, .rst_l, .en(~halted & ~mem_stall), .clear(1'b0), .D(alu_out_E), .Q(alu_out_M1));
    register #(XLEN, '0) RS2_DATA_E_M1_REG(.clk, .rst_l, .en(~halted & ~mem_stall), .clear(1'b0), .D(rs2_data_muxed_E), .Q(rs2_data_M1));
    register #(XLEN, '0) BRANCH_TARGET_E_M1_REG(.clk, .rst_l, .en(~halted & ~mem_stall), .clear(1'b0), .D(branch_target_E), .Q(branch_target_M1));
    register #(XLEN, '0) PC_E_M1_REG(.clk, .rst_l, .en(~halted & ~mem_stall), .clear(1'b0), .D(pc_E), .Q(pc_M1));
    // A trapping/returning instruction is converted to a bubble as it advances
    // to M1 so it commits no architectural side effects (rf/mem/CSR write).
    register #($bits(ctrl_signals_E), 'b0) CTRL_E_M1_REG(.clk, .rst_l, .en(~halted & ~mem_stall), .clear(trap_redirect_E), .D(ctrl_signals_E), .Q(ctrl_signals_M1));
    register #(5, 5'b0) RD_E_M1_REG(.clk, .rst_l, .en(~halted & ~mem_stall), .clear(1'b0), .D(rd_E), .Q(rd_M1));
    register #(1, 1'b0) CMP_OUT_E_M1_REG(.clk, .rst_l, .en(~halted & ~mem_stall), .clear(1'b0), .D(cmp_out_E), .Q(bcond_M1));
    register #(1, 1'b0) FLUSH_E_M1_REG(.clk, .rst_l, .en(~halted & ~mem_stall), .clear(1'b0), .D(flush_E), .Q(flush_M1));

    register #(1, 1'b0) MEMREAD_E_M1_REG(.clk, .rst_l, .en(~halted & ~mem_stall), .clear(mem_stall), .D(MemRead_E), .Q(MemRead_M1));
    
    

    //===============================MEM1 start=========================//

    logic [XLEN-1:0] mem_read_data_M1, final_data_M1, cache_read_data_selected_M1;
    logic [3:0][XLEN-1:0] cache_write_data_M1;
    logic [XLEN-1:0] cache_read_data_M1;
    logic RegWrite_M1, Mem2Reg_M1, syscall_M1, illegal_instr_M1;
    logic [2:0] rd_src_M1;
    logic [ADDR_SHIFT-1:0] data_byte_M1;   // byte offset within the XLEN-byte word
    logic [1:0] data_offset;               // word offset within the 4-word line
    logic cache_en;
    logic read_hit_M1, read_miss_M1, is_eviction_M1, mem_cache_sel_M1;
    logic [MEMORY_ADDR_WIDTH-1:0] cache_addr, data_addr_aligned;
    ldst_mode_t ldst_mode_M1;
    assign npc_offset_M1 = branch_target_M1;
    assign PCSrc_M1 = ctrl_signals_M1.pc_source == PC_cond && bcond_M1;
    assign data_offset = pa_M1[ADDR_SHIFT+1 : ADDR_SHIFT];
    assign data_addr_aligned = {pa_M1[XLEN-1 : ADDR_SHIFT+2], 2'b0};
    assign data_addr = MemRead_M1 ? data_addr_aligned : pa_M1[XLEN-1 : ADDR_SHIFT];
    assign data_load_en = MemRead_M1 && read_miss_M1;
    assign cache_en = data_load_valid || ctrl_signals_M1.memWrite;
    assign RegWrite_M1 = ctrl_signals_M1.rfWrite;
    assign Mem2Reg_M1 = ctrl_signals_M1.mem2RF;
    assign syscall_M1 = ctrl_signals_M1.syscall;
    assign ldst_mode_M1 = ctrl_signals_M1.ldst_mode;
    assign illegal_instr_M1 = ctrl_signals_M1.illegal_instr;
    assign data_byte_M1 = pa_M1[ADDR_SHIFT-1 : 0];
    assign rd_src_M1 = ctrl_signals_M1.rd_source;
    riscv_store_unit Store_Unit(.memWrite(ctrl_signals_M1.memWrite), .ldst_mode(ctrl_signals_M1.ldst_mode), .data_byte(data_byte_M1), .rs2_data(rs2_data_M1), .data_store_mask(data_store_mask), .data_store(data_store));
    

    logic [3:0] write_data_valid_M1;

    // write data valid signal
    always_comb begin
        write_data_valid_M1 = 4'b0000;
        if (ctrl_signals_M1.memWrite) begin
            write_data_valid_M1[data_offset] = 1'b1;
        end else if (data_load_valid) begin
            write_data_valid_M1 = 4'b1111;
        end
    end

    assign cache_addr = MemRead_M1 ? pa_M1[XLEN-1 : ADDR_SHIFT] : data_addr_aligned;

    logic cache_flush_M1;

    // cache read write flush logic: a store that does not write a whole cache
    // word flushes the line (the cache merges only at word granularity).
    always_comb begin
        cache_flush_M1 = 1'b0;
        if (ctrl_signals_M1.memWrite) begin
`ifdef RV64
            // RV64 sub-word stores are byte-merged into the cached word (below),
            // so the cache stays coherent without a flush -- which is what keeps
            // a following load's same-cycle cache forwarding working (the data
            // memory has an 8-cycle read latency, so a flush+memory-miss would
            // read stale data).
            cache_flush_M1 = 1'b0;
`else
            case(ctrl_signals_M1.ldst_mode)
                LDST_W: cache_flush_M1 = 1'b0;
                LDST_H: cache_flush_M1 = 1'b1;
                LDST_B: cache_flush_M1 = 1'b1;
                default: cache_flush_M1 = 1'b0;
            endcase
`endif
        end
    end

    // RV64: byte-merge a sub-word store into the current cached word so the
    // whole-word cache write preserves the untouched bytes. (RV32 stores a whole
    // 32-bit word, so data_store is used directly.)
    logic [XLEN-1:0] cache_store_word;
`ifdef RV64
    logic [XLEN-1:0] store_mask_bits;
    always_comb begin
        for (int bi = 0; bi < XLEN_BYTES; bi++)
            store_mask_bits[bi*8 +: 8] = {8{data_store_mask[bi]}};
    end
    assign cache_store_word = (cache_read_data_M1 & ~store_mask_bits)
                            | (data_store & store_mask_bits);
`else
    assign cache_store_word = data_store;
`endif

    // cache write data select logic
    mux #(2, XLEN) MEM_CACHE_DATA_SEL_0(.in({data_load[0], cache_store_word}), .sel(data_load_valid), .out(cache_write_data_M1[0]));
    mux #(2, XLEN) MEM_CACHE_DATA_SEL_1(.in({data_load[1], cache_store_word}), .sel(data_load_valid), .out(cache_write_data_M1[1]));
    mux #(2, XLEN) MEM_CACHE_DATA_SEL_2(.in({data_load[2], cache_store_word}), .sel(data_load_valid), .out(cache_write_data_M1[2]));
    mux #(2, XLEN) MEM_CACHE_DATA_SEL_3(.in({data_load[3], cache_store_word}), .sel(data_load_valid), .out(cache_write_data_M1[3]));


    cache447 #(.WAYS(2), .POLICY(1), .INDEX_BITS(5), .WORD_SIZE(XLEN), .ADDRESS_SIZE(MEMORY_ADDR_WIDTH)) DataCache (
        .clk(clk), .rst_l(rst_l), .address(cache_addr), 
        .enable(MemRead_M1 | ctrl_signals_M1.memWrite | data_load_valid), .flush(cache_flush_M1), 
        .write_data(cache_write_data_M1), .read_data(cache_read_data_M1), .rd_wr(MemRead_M1), 
        .write_data_valid(write_data_valid_M1), .read_hit(read_hit_M1), .read_miss(read_miss_M1), 
        .is_eviction(is_eviction_M1));   

    // cache data and memory data select signal
    always_comb begin
      if (read_hit_M1) begin
        mem_cache_sel_M1 = 1'b1;
      end
      else begin
        mem_cache_sel_M1 = 1'b0;
      end
    end
    
    // CSR reads write the old CSR value into rd; other non-memory ops use the
    // ALU result (or PC+4 for link).
    assign rd_in_M1 = (rd_src_M1 == RD_PC4) ? (pc_M1 + XLEN'(4)) :
                      (ctrl_signals_M1.exec_class == EXEC_CSR) ? csr_read_data_M1 :
                      (alu_out_M1);


    riscv_load_unit Load_Unit_MEM(.data_load(data_load[data_offset]), .ldst_mode(ldst_mode_M1), .data_byte(data_byte_M1), .ld_out(mem_read_data_M1));
    riscv_load_unit Load_Unit_CACHE(.data_load(cache_read_data_M1), .ldst_mode(ldst_mode_M1), .data_byte(data_byte_M1), .ld_out(cache_read_data_selected_M1));
    logic [XLEN-1:0] mem_final_data_M1;
    mux #(2, XLEN) MEM_CACHE_SEL_DATA(.in({cache_read_data_selected_M1, mem_read_data_M1}), .sel(mem_cache_sel_M1), .out(mem_final_data_M1));
    // A load that hits the memory-mapped CLINT returns its register value.
    assign final_data_M1 = (MemRead_M1 && clint_load_hit) ? clint_load_data :
                           mem_final_data_M1;
    assign rd_data_M1 = Mem2Reg_M1 ? final_data_M1 : rd_in_M1;


    //===============================MEM1/WB Pipeline Registers=========================//

    logic [XLEN-1:0] rs1_data_W, rd_data_W, mem_read_data_W;
    logic [2:0] rd_src_W;
    logic [ADDR_SHIFT-1:0] data_byte_W;
    logic syscall_W, illegal_instr_W, Mem2Reg_W;
    ldst_mode_t ldst_mode_W;
    register #(XLEN, '0) RS1_DATA_M1_WB_REG(.clk, .rst_l, .en(~halted & ~mem_stall), .clear(1'b0), .D(rs1_data_M1), .Q(rs1_data_W));
    register #(XLEN, '0) RD_DATA_M1_WB_REG(.clk, .rst_l, .en(~halted & ~mem_stall), .clear(1'b0), .D(rd_in_M1), .Q(rd_data_W));
    register #(5, 5'b0) RD_M1_WB_REG(.clk, .rst_l, .en(~halted & ~mem_stall), .clear(1'b0), .D(rd_M1), .Q(rd_W));
    register #(ADDR_SHIFT, '0) DATA_BYTE_M1_WB_REG(.clk, .rst_l, .en(~halted & ~mem_stall), .clear(1'b0), .D(data_byte_M1), .Q(data_byte_W));
    register #(1, 1'b0) MEM2REG_M1_WB_REG(.clk, .rst_l, .en(~halted & ~mem_stall), .clear(1'b0), .D(Mem2Reg_M1), .Q(Mem2Reg_W));
    register #(1, 1'b0) REGWRITE_M1_WB_REG(.clk, .rst_l, .en(~halted & ~mem_stall), .clear(1'b0), .D(RegWrite_M1), .Q(RegWrite_W));
    register #(1, 1'b0) SYSCALL_M1_WB_REG(.clk, .rst_l, .en(~halted & ~mem_stall), .clear(1'b0), .D(syscall_M1), .Q(syscall_W));
    register #(1, 1'b0) ILLEGAL_INSTR_M1_WB_REG(.clk, .rst_l, .en(~halted & ~mem_stall), .clear(1'b0), .D(illegal_instr_M1), .Q(illegal_instr_W));
    register #($bits(ldst_mode_M1), LDST_DC) LDST_MODE_M1_WB_REG(.clk, .rst_l, .en(~halted & ~mem_stall), .clear(1'b0), .D(ldst_mode_M1), .Q(ldst_mode_W));
    register #(XLEN, '0) RD_IN_M1_WB_REG(.clk, .rst_l, .en(~halted & ~mem_stall), .clear(1'b0), .D(rd_data_M1), .Q(rd_in_W));
    register #(1, 1'b0) FLUSH_M1_W_REG(.clk, .rst_l, .en(~halted & ~mem_stall), .clear(1'b0), .D(flush_M1), .Q(flush_W));




    //===============================WB start=========================//

    //Handle syscall halt here
    logic syscall_halt, exception_halt, read_miss;
    // A committed ecall with a0 in {10,11} halts the sim (pass/fail), matching the
    // OoO core. +no_ecall_halt disables it for a real OS boot, where the kernel's
    // SBI ecalls legitimately carry those values (see ooo_alu_pipe.sv / Phase H4).
    logic ecall_halt_en;
    initial ecall_halt_en = !$test$plusargs("no_ecall_halt");
    assign syscall_halt = ecall_halt_en && syscall_W &&
                          ((rs1_data_W == ECALL_ARG_HALT) || (rs1_data_W == 'd11));
    /* Illegal instructions now raise a precise trap at EX instead of halting.
     * Raw fetch/data memory exceptions remain a hard halt for now (the MMU adds
     * precise instruction/load/store faults in Phase 2). */
    assign exception_halt   = instr_mem_excpt | data_mem_excpt;
    assign halted = rst_l & (syscall_halt | exception_halt);

    data_stall_unit DSU(.clk, .rst_l, .read_miss_M1, .data_load_valid, .mem_stall(dsu_mem_stall));
    assign mem_stall = dsu_mem_stall | ptw_stall;
    


    riscv_forwarding_unit Forwarding_Unit(.*);

    //===============================WB end=========================//



    

`ifdef SIMULATION_18447
    always_ff @(posedge clk) begin
        if (rst_l && instr_mem_excpt) begin
            $display("Instruction memory exception at address 0x%08x.", instr_addr << 2);
        end
        if (rst_l && data_mem_excpt) begin
            $display("Data memory exception at address 0x%08x.", data_addr << 2);
        end
        if (rst_l && syscall_halt) begin
            $display("ECALL invoked with halt argument. Terminating simulation at 0x%08x.", pc_F1);
        end
    end

    // Performance counters
    parameter stall_cycle = 8;
    logic [63:0] cycle_counter;
    logic [63:0] instr_counter;
    logic [63:0] instr_counter_correct; // Instructions executed no wrong path
    logic [63:0] stall_id_counter;
    logic [63:0] last_pc_update;
    logic [63:0] stall_cycles_prev;

    logic [63:0] alu_instructions;         // ALU type instructions
    logic [63:0] load_instructions;        // Load instructions
    logic [63:0] store_instructions;       // Store instructions
    logic [63:0] branch_instructions;      // Branch instructions
    logic [63:0] mispredicted_branches;    // Mispredicted branches
    
    logic first_inst;

    // 2d list of stall cycles per instruction
    // stall_instr[0] counts instructions that experienced 0 stalls, etc.
    logic [63:0] stall_instr [0:stall_cycle-1];

    // Branch instructions: 4 dimensions → 2×2×2×2 = 16 counters.
    // Index is computed as: { (backward_offset), (taken), (BTB hit), (rewind required) }
    logic [63:0] branch_instr_counter [15:0];

    // JAL instructions: 3 dimensions → 2×2×2 = 8 counters.
    // Index is computed as: { (rd != x1), (BTB hit), (rewind required) }
    logic [63:0] jal_instr_counter [7:0];

    // JALR instructions: 3 dimensions → 2×2×2 = 8 counters.
    // Index is computed as: { (rs1 != x1), (BTB hit), (rewind required) }
    logic [63:0] jalr_instr_counter [7:0];

    // counters for lab4
    logic [63:0] total_cache_reads;

    logic [63:0] cache_read_misses;

    logic [63:0] cache_read_hits;

    logic [63:0] total_cache_writes;

    logic [63:0] total_evictions;

    always_ff @(posedge clk or negedge rst_l) begin
        if (!rst_l) begin
            cycle_counter   <= 0;
            instr_counter   <= 0;
            instr_counter_correct <= 0;
            stall_id_counter <= 0;
            last_pc_update  <= 0;
            first_inst      <= 1;

            alu_instructions<= 0;
            load_instructions<= 0;
            store_instructions<= 0;
            branch_instructions<= 0;
            mispredicted_branches <= 0;
            for (int i = 0; i < stall_cycle; i++) begin
                stall_instr[i] <= 0;
            end
            for (int i = 0; i < 16; i++) begin
                branch_instr_counter[i] <= 0;
            end
            for (int j = 0; j < 8; j++) begin
                jal_instr_counter[j]  <= 0;
                jalr_instr_counter[j] <= 0;
            end

            total_cache_reads <= 64'b0;
            cache_read_misses <= 64'b0;
            cache_read_hits <= 64'b0;
            total_cache_writes <= 64'b0;
            total_evictions <= 64'b0;

        end else if (~halted) begin
            cycle_counter <= cycle_counter + 1;


            total_cache_reads <= (MemRead_M1) ? total_cache_reads + 1 : total_cache_reads;
            cache_read_misses <= (MemRead_M1 & read_miss_M1) ? cache_read_misses + 1 : cache_read_misses;
            cache_read_hits <= (MemRead_M1 & read_hit_M1) ? cache_read_hits + 1 : cache_read_hits;
            total_cache_writes <= ((ctrl_signals_M1.memWrite & ~cache_flush_M1) | data_load_valid) ? total_cache_writes + 1 : total_cache_writes;
            total_evictions <= (is_eviction_M1) ? total_evictions + 1 : total_evictions;
            // If the ID stage is stalled, increment stall_id_counter
            if (mem_stall) begin
                stall_id_counter <= stall_id_counter + 1;
            end
            if (flush_E) begin
              mispredicted_branches <= mispredicted_branches + 1;
            end
            // When there is no stall, a new instruction is being fetched
            if (~mem_stall) begin                
                instr_counter <= instr_counter + 1;
                // wrong path instructions are not counted
                if (~flush_E && ~flush_M1 && ~flush_W) begin
                  instr_counter_correct <= instr_counter_correct + 1;
                  case (opcode_E) 
                    OP_OP, OP_IMM: begin
                      // ALU instructions
                      alu_instructions <= alu_instructions + 1;
                    end
                    OP_LOAD: begin
                      // Load instructions
                      load_instructions <= load_instructions + 1;
                    end
                    OP_STORE: begin
                      // Store instructions
                      store_instructions <= store_instructions + 1;
                    end
                    OP_BRANCH: begin
                      // Branch instructions
                      logic[15:0] branch_idx;
                      branch_idx = {(branch_target_E < pc_E) ? 1'b1 : 1'b0,
                                    actual_taken_E,
                                    branch_target_E == next_pc_E,
                                    flush_E};
                      branch_instr_counter[branch_idx] <= branch_instr_counter[branch_idx] + 1;
                      branch_instructions <= branch_instructions + 1;
                    end
                    OP_JAL: begin
                      // JAL instructions
                      logic[7:0] jal_idx;
                      jal_idx = {rd_E == X1,
                                    branch_target_E == next_pc_E,
                                    flush_E};
                      jal_instr_counter[jal_idx] <= jal_instr_counter[jal_idx] + 1;
                      branch_instructions <= branch_instructions + 1;
                    end
                    OP_JALR: begin
                      // JALR instructions
                      logic[7:0] jalr_idx;
                      jalr_idx = {rs1_E == X1,
                                    branch_target_E == next_pc_E,
                                    flush_E};
                      jalr_instr_counter[jalr_idx] <= jalr_instr_counter[jalr_idx] + 1;
                      branch_instructions <= branch_instructions + 1;
                    end
                  endcase
                end
            
                // For all but the first instruction, compute the stall 
                // cycles of the previous instruction 
                if (!first_inst) begin
                    stall_cycles_prev = cycle_counter - last_pc_update - 1;
                    stall_instr[stall_cycles_prev] <= stall_instr[stall_cycles_prev] + 1;
                end else begin
                    first_inst <= 0;
                end

                last_pc_update <= cycle_counter;
            end
        end
    end

    // Display the performance counters at the end of the simulation
    initial begin
        wait (halted);
        $display("FINAL PERFORMANCE COUNTERS:");
        $display("Total cycles: %0d", cycle_counter);
        $display("Instructions fetched: %0d", instr_counter);
        $display("Instructions executed (no wrong path or bubbles): %0d", instr_counter_correct);
        $display("  ALU instructions: %0d", alu_instructions);
        $display("  Load instructions: %0d", load_instructions);
        $display("  Store instructions: %0d", store_instructions);
        $display("ID stall cycles: %0d", stall_id_counter);
        for (logic [3:0] i = 4'b0; i < stall_cycle; i++) begin
            $display("Instructions with %0d stalls: %0d", i, stall_instr[i]);
        end
        for (int i = 0; i < 16; i++) begin
            $display("Branch inst (idx %0d):     %0d", i, branch_instr_counter[i]);
        end
        for (int j = 0; j < 8; j++) begin
            $display("JAL inst (idx %0d):        %0d", j, jal_instr_counter[j]);
            $display("JALR inst (idx %0d):       %0d", j, jalr_instr_counter[j]);
        end
        $display("Total cache reads: %0d", total_cache_reads);
        $display("Total cache read misses: %0d", cache_read_misses);
        $display("Total cache read hits: %0d", cache_read_hits);
        $display("Total cache writes: %0d", total_cache_writes);
        $display("Total evictions: %0d", total_evictions);
        $display("Total control flow instructions: %0d", branch_instructions);
        $display("Mispredicted control flow instructions: %0d", mispredicted_branches);
    end
`endif /* SIMULATION_18447 */

    /* When the design is compiled for simulation, the Makefile defines
     * SIMULATION_18447. You can use this to have code that is there for
     * simulation, but is discarded when the design is synthesized. Useful
     * for constructs that can't be synthesized. */
`ifdef SIMULATION_18447
`ifdef TRACE

    // opcode_t opcode;
    // funct7_t funct7;
    // rtype_funct3_t rtype_funct3;
    // itype_int_funct3_t itype_int_funct3;
    // assign opcode           = opcode_t'(instr[6:0]);
    // assign funct7           = funct7_t'(instr[31:25]);
    // assign rtype_funct3     = rtype_funct3_t'(instr[14:12]);
    // assign itype_int_funct3 = itype_int_funct3_t'(instr[14:12]);

    // /* Cycle-by-cycle trace messages. You'll want to comment this out for
    //  * longer tests, or they will take much, much longer to run. Be sure to
    //  * comment this out before submitting your code, so tests can be run
    //  * quickly. */
    // always_ff @(posedge clk) begin
    //     if (rst_l) begin
    //         $display({"\n", {80{"-"}}});
    //         $display("- Simulation Cycle %0d", $time);
    //         $display({{80{"-"}}, "\n"});

    //         $display("\tPC: 0x%08x", pc);
    //         $display("\tInstruction: 0x%08x\n", instr);

    //         $display("\tInstruction Memory Exception: %0b", instr_mem_excpt);
    //         $display("\tData Memory Exception: %0b", data_mem_excpt);
    //         $display("\tIllegal Instruction Exception: %0b", ctrl_signals.illegal_instr);
    //         $display("\tHalted: %0b\n", halted);

    //         $display("\tOpcode: 0x%02x (%s)", opcode, opcode.name);
    //         $display("\tFunct3: 0x%01x (%s | %s)", rtype_funct3, rtype_funct3.name, itype_int_funct3.name);
    //         $display("\tFunct7: 0x%02x (%s)", funct7, funct7.name);
    //         $display("\trs1: %0d", rs1);
    //         $display("\trs2: %0d", rs2);
    //         $display("\trd: %0d", rd);
    //         $display("\tSign Extended Immediate: %0d", se_immediate);
    //     end
    // end

`endif /* TRACE */
`endif /* SIMULATION_18447 */

endmodule: riscv_core_scalar

/**
 * The arithmetic-logic unit (ALU) for the RISC-V processor.
 *
 * The ALU handles executing the current instruction, producing the
 * appropriate output based on the ALU operation specified to it by the
 * decoder.
 *
 * Inputs:
 *  - alu_src1      The first operand to the ALU.
 *  - alu_src2      The second operand to the ALU.
 *  - alu_op        The ALU operation to perform.
 * Outputs:
 *  - alu_out       The result of the ALU operation on the two sources.
 **/
module riscv_alu
    import RISCV_ISA::XLEN;
(
    input  logic [XLEN-1:0] alu_src1,
    input  logic [XLEN-1:0] alu_src2,
    input  alu_op_t     alu_op,
    output logic [XLEN-1:0] alu_out
);

  logic [XLEN-1:0] sum, or_res, xor_res, and_res;
  logic [XLEN-1:0] sll_res, srl_res, sra_res;

  logic eq, ne, sltu, slt, geu, ge;

  adder #($bits(alu_src1)) ALU_Adder(.A(alu_src1),
                                    .B((alu_op==ALU_SUB)?(~alu_src2):alu_src2),
                                    .cin(alu_op==ALU_SUB),
                                    .sum, .cout());

  assign or_res = alu_src1 | alu_src2;
  assign xor_res = alu_src1 ^ alu_src2;
  assign and_res = alu_src1 & alu_src2;

  // Shift amount: 5 bits for RV32, 6 for RV64 ($clog2(XLEN)).
  assign sll_res = alu_src1 << alu_src2[$clog2(XLEN)-1:0];
  assign srl_res = alu_src1 >> alu_src2[$clog2(XLEN)-1:0];
  assign sra_res = $signed(alu_src1) >>> alu_src2[$clog2(XLEN)-1:0];

  assign eq = ~|xor_res;
  assign ne = |xor_res;
  assign sltu = alu_src1 < alu_src2;
  // assign slt = $signed(alu_src1) < $signed(alu_src2);
  assign slt = (alu_src1[XLEN-1] == alu_src2[XLEN-1]) ? (alu_src1 < alu_src2) : (alu_src1[XLEN-1] > alu_src2[XLEN-1]);
  assign geu = ~sltu;
  assign ge = ~slt;

  logic [XLEN-1:0] arith_res, logic_res, shift_res, cmp_res;

  assign arith_res = sum;

  assign logic_res =
       (alu_op == ALU_OR ) ? or_res
     : (alu_op == ALU_XOR) ? xor_res
     : and_res;

  assign shift_res =
       (alu_op == ALU_SLL) ? sll_res
     : (alu_op == ALU_SRL) ? srl_res
     : sra_res;

  assign cmp_res =
       (alu_op == ALU_BEQ ) ? {{(XLEN-1){1'b0}}, eq}
     : (alu_op == ALU_BNE ) ? {{(XLEN-1){1'b0}}, ne}
     : (alu_op == ALU_BLT ) ? {{(XLEN-1){1'b0}}, slt}
     : (alu_op == ALU_BGE ) ? {{(XLEN-1){1'b0}}, ge}
     : (alu_op == ALU_BLTU) ? {{(XLEN-1){1'b0}}, sltu}
     : (alu_op == ALU_BGEU) ? {{(XLEN-1){1'b0}}, geu}
     : (alu_op == ALU_SLT ) ? {{(XLEN-1){1'b0}}, slt}
     : (alu_op == ALU_SLTU) ? {{(XLEN-1){1'b0}}, sltu}
     : 'x;

  logic [1:0] group_sel;
  assign group_sel =
       (alu_op == ALU_ADD || alu_op == ALU_SUB) ? 2'b00
     : (alu_op == ALU_OR  || alu_op == ALU_XOR || alu_op == ALU_AND) ? 2'b01
     : (alu_op == ALU_SLL || alu_op == ALU_SRL || alu_op == ALU_SRA) ? 2'b10
     : 2'b11;

  // RV64 W-form ops: operate on the low 32 bits, sign-extend bit 31 to XLEN.
  // (Never selected in RV32 -- the decoder only emits them under -DRV64.)
  logic [31:0] w_res32;
  logic        is_w_op;
  assign is_w_op = (alu_op == ALU_ADDW) || (alu_op == ALU_SUBW)
                || (alu_op == ALU_SLLW) || (alu_op == ALU_SRLW)
                || (alu_op == ALU_SRAW);
  always_comb begin
    unique case (alu_op)
      ALU_ADDW: w_res32 = alu_src1[31:0] +  alu_src2[31:0];
      ALU_SUBW: w_res32 = alu_src1[31:0] -  alu_src2[31:0];
      ALU_SLLW: w_res32 = alu_src1[31:0] << alu_src2[4:0];
      ALU_SRLW: w_res32 = alu_src1[31:0] >> alu_src2[4:0];
      ALU_SRAW: w_res32 = $signed(alu_src1[31:0]) >>> alu_src2[4:0];
      default:  w_res32 = 'x;
    endcase
  end

  always_comb begin
    if (is_w_op)
      alu_out = {{(XLEN-32){w_res32[31]}}, w_res32};
    else case (group_sel)
      2'b00: alu_out = arith_res;
      2'b01: alu_out = logic_res;
      2'b10: alu_out = shift_res;
      default: alu_out = cmp_res;
    endcase
  end

endmodule


// The forwarding unit for the RISC-V processor
module riscv_forwarding_unit
  (input  logic [4:0]  rs1_E, rs2_E, rd_E, 
   input  logic        RegWrite_E, RegWrite_M1, RegWrite_W,
   input  logic [4:0]  rd_M1, rd_W, 
   output logic [1:0]  rs1_fwd_sel_E, rs2_fwd_sel_E);

  // Forwarding logic for rs1
  always_comb begin
    if (rs1_E != 0 && rs1_E == rd_M1 && RegWrite_M1) begin
      rs1_fwd_sel_E = 2'd1;
    end else if (rs1_E != 0 && rs1_E == rd_W && RegWrite_W) begin
      rs1_fwd_sel_E = 2'd2;
    end else begin
      rs1_fwd_sel_E = 2'd0;
    end
  end

  // Forwarding logic for rs2
  always_comb begin
    if (rs2_E != 0 && rs2_E == rd_M1 && RegWrite_M1) begin
      rs2_fwd_sel_E = 2'd1;
    end else if (rs2_E != 0 && rs2_E == rd_W && RegWrite_W) begin
      rs2_fwd_sel_E = 2'd2;
    end else begin
      rs2_fwd_sel_E = 2'd0;
    end
  end

endmodule: riscv_forwarding_unit

// The ALU source selection unit for the RISC-V processor
module alu_src2_sel_unit
  (input  logic [4:0]  rs2_E, rd_E, 
   input  logic        RegWrite_E, RegWrite_M1, RegWrite_W, 
   input  ctrl_signals_t ctrl_signals_E,
   input  logic [4:0]  rd_M1, rd_W, 
   output logic [1:0]  alu_src2_sel_E);

  // logic for alu_src2_sel
  always_comb begin
    if (ctrl_signals_E.useImm) begin
      alu_src2_sel_E = 2'd0;
    end else if (rs2_E != 0 && rs2_E == rd_M1 && RegWrite_M1) begin
      alu_src2_sel_E = 2'd2;
    end else if (rs2_E != 0 && rs2_E == rd_W && RegWrite_W) begin
      alu_src2_sel_E = 2'd3;
    end else begin
      alu_src2_sel_E = 2'd1;
    end
  end

endmodule: alu_src2_sel_unit

// The store unit for the RISC-V processor
// XLEN-generic store formatter: place the size's low bytes of rs2 at the byte
// offset within the (XLEN-byte) memory word, and build the byte-enable mask.
module riscv_store_unit
    import RISCV_ISA::XLEN, RISCV_ISA::XLEN_BYTES;
    (input  logic        memWrite,
     input  ldst_mode_t  ldst_mode,
     input  logic [$clog2(XLEN_BYTES)-1:0] data_byte,
     input  logic [XLEN-1:0] rs2_data,
     output logic [XLEN_BYTES-1:0] data_store_mask,
     output logic [XLEN-1:0] data_store);
    logic [XLEN-1:0]       base_data;   // size's bytes in the low position
    logic [XLEN_BYTES-1:0] base_mask;   // size's byte mask in the low position
    always_comb begin
      case(ldst_mode)
        LDST_B: begin base_data = {{(XLEN-8){1'b0}},  rs2_data[7:0]};  base_mask = XLEN_BYTES'('h1); end
        LDST_H: begin base_data = {{(XLEN-16){1'b0}}, rs2_data[15:0]}; base_mask = XLEN_BYTES'('h3); end
        LDST_W: begin base_data = {{(XLEN-32){1'b0}}, rs2_data[31:0]}; base_mask = XLEN_BYTES'('hF); end
        LDST_D: begin base_data = rs2_data;                           base_mask = '1;              end
        default:begin base_data = 'x;                                 base_mask = '0;              end
      endcase
    end
    assign data_store      = base_data << {data_byte, 3'b0};
    assign data_store_mask = memWrite ? (base_mask << data_byte) : '0;
endmodule: riscv_store_unit


// XLEN-generic load formatter: shift the target field to bit 0, then sign- or
// zero-extend to XLEN per the size.
module riscv_load_unit
    import RISCV_ISA::XLEN, RISCV_ISA::XLEN_BYTES;
    (input  logic [XLEN-1:0] data_load,
     input  logic [$clog2(XLEN_BYTES)-1:0] data_byte,
     input  ldst_mode_t     ldst_mode,
     output logic [XLEN-1:0] ld_out);
    logic [XLEN-1:0] shifted;
    assign shifted = data_load >> {data_byte, 3'b0};
    always_comb begin
      case(ldst_mode)
        LDST_B:  ld_out = {{(XLEN-8){shifted[7]}},   shifted[7:0]};
        LDST_BU: ld_out = {{(XLEN-8){1'b0}},         shifted[7:0]};
        LDST_H:  ld_out = {{(XLEN-16){shifted[15]}}, shifted[15:0]};
        LDST_HU: ld_out = {{(XLEN-16){1'b0}},        shifted[15:0]};
        LDST_W:  ld_out = {{(XLEN-32){shifted[31]}}, shifted[31:0]};
        LDST_WU: ld_out = {{(XLEN-32){1'b0}},        shifted[31:0]};
        LDST_D:  ld_out = data_load;
        default: ld_out = 'x;
      endcase
    end
endmodule: riscv_load_unit



// 2 bit hyseresis counter, taken should be 1 if the branch is actually taken, 
// pred_taken is be 1 if the branch is predicted taken
module hysteresis_counter
  (input  logic        clk, rst_l,
   input  logic        taken,
   input  logic  [1:0] cur_state,
   output logic  [1:0] pred_state);

  // always_comb begin
  //   case (cur_state) 
  //     PRED_TAKEN_S: pred_state = taken ? PRED_TAKEN_S : PRED_TAKEN_W;
  //     PRED_TAKEN_W: pred_state = taken ? PRED_TAKEN_S : PRED_NOT_TAKEN_W;
  //     PRED_NOT_TAKEN_S: pred_state = taken ? PRED_NOT_TAKEN_W : PRED_NOT_TAKEN_S;
  //     PRED_NOT_TAKEN_W: pred_state = taken ? PRED_TAKEN_W : PRED_NOT_TAKEN_S;
  //     default: pred_state = PRED_NOT_TAKEN_S; 
  //   endcase
  // end


  // 0 bit
  assign pred_state = PRED_NOT_TAKEN_S;
  // 1 bit
  // assign pred_state = taken ? PRED_TAKEN_S : PRED_NOT_TAKEN_S;

endmodule: hysteresis_counter

module data_stall_unit
  (input logic clk, rst_l, 
   input logic read_miss_M1, data_load_valid,
   output logic mem_stall);

  logic mem_stall_internal;

  always_ff @(posedge clk or negedge rst_l) begin
    if (~rst_l) begin
      mem_stall_internal <= 1'b0;
    end else if (mem_stall_internal) begin
        if (data_load_valid) begin
            mem_stall_internal <= 1'b0;
        end
    end else begin
        if (read_miss_M1) begin
            mem_stall_internal <= 1'b1;
        end else if (data_load_valid) begin
            mem_stall_internal <= 1'b0;
        end
    end
  end

  always_comb begin
    mem_stall = 1'b0;
    if ((mem_stall_internal || read_miss_M1) && (~data_load_valid)) begin
        mem_stall = 1'b1;
    end
  end
  

endmodule: data_stall_unit




