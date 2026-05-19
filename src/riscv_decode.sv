/**
 * riscv_decode.sv
 *
 * RISC-V 32-bit Processor
 *
 * ECE 18-447
 * Carnegie Mellon University
 *
 * This file contains the implementation of the RISC-V decoder.
 *
 * This takes in information about the current RISC-V instruction and produces
 * the appropriate control signals to get the processor to execute the current
 * instruction.
 **/

/*----------------------------------------------------------------------------*
 *  You may edit this file and add or change any files in the src directory.  *
 *----------------------------------------------------------------------------*/

// RISC-V Includes
`include "riscv_isa.vh"             // RISC-V ISA definitions

// Local Includes
`include "internal_defines.vh"      // Control signals struct, ALU ops

// Force the compiler to throw an error if any variables are undeclared
`default_nettype none

// Define a macro that prints for simulation, does nothing for synthesis
`ifdef SIMULATION_18447
`define display(print, format, arg) \
    do begin \
        if (print) begin \
            $display(format, arg); \
        end \
    end while (0)
`else
`define display(print, format, arg)
`endif /* SIMULATION_18447 */

/**
 * The instruction decoder for the RISC-V processor.
 *
 * This module processes the current instruction, determines what instruction it
 * is, and sets the control signals for the processor appropriately.
 *
 * Inputs:
 *  - rst_l             The asynchronous, active low reset for the processor.
 *  - instr             The current instruction being executed by the processor.
 *
 * Outputs:
 *  - ctrl_signals      The control signals needed to execute the given
 *                      instruction correctly.
 **/
module riscv_decode
    (input  logic           rst_l,
     input  logic [31:0]    instr,
     output ctrl_signals_t  ctrl_signals);

    // Import all of the ISA types and enums (opcodes, functions codes, etc.)
    import RISCV_ISA::*;

    // The various fields of an instruction
    opcode_t            opcode;
    funct7_t            funct7;
    rtype_funct3_t      rtype_funct3;
    itype_int_funct3_t  itype_int_funct3;
    itype_load_funct3_t itype_load_funct3;
    stype_funct3_t      stype_funct3;
    sbtype_funct3_t     sbtype_funct3;
    itype_funct12_t     itype_funct12;

    // Decode the opcode and various function codes for the instruction
    assign opcode           = opcode_t'(instr[6:0]);
    assign funct7           = funct7_t'(instr[31:25]);
    assign rtype_funct3     = rtype_funct3_t'(instr[14:12]);
    assign itype_int_funct3 = itype_int_funct3_t'(instr[14:12]);
    assign itype_load_funct3= itype_load_funct3_t'(instr[14:12]);
    assign stype_funct3     = stype_funct3_t'(instr[14:12]);
    assign sbtype_funct3    = sbtype_funct3_t'(instr[14:12]);
    assign itype_funct12    = itype_funct12_t'(instr[31:20]);

        always_comb begin
            // Default initialization to prevent latch inference
            ctrl_signals = '{
                useImm: 1'b0,
                usePC: 1'b0,
                doCmp: 1'b0,
                rfWrite: 1'b0,
                mem2RF: 1'b0,
                pc2RF: 1'b0,
                memRead: 1'b0,
                memWrite: 1'b0,
                imm_mode: IMM_DC,
                alu_op: ALU_DC,
                ldst_mode: LDST_DC,
                pc_source: PC_DC,
                rd_source: RD_DC,
                isBranch: 1'b0,

                btype: sbtype_funct3,
                syscall: 1'b0,
                illegal_instr: 1'b0
            };
            
            if(~rst_l) begin
                ctrl_signals = '{
                    useImm: 1'b0,
                    usePC: 1'b0,
                    doCmp: 1'b0,
                    rfWrite: 1'b0,  // never don't care
                    mem2RF: 1'b0,
                    pc2RF: 1'b0,
                    memRead: 1'b0,  // never don't care
                    memWrite: 1'b0, // never don't care
                    imm_mode: IMM_DC,
                    alu_op: ALU_DC,
                    ldst_mode: LDST_DC,
                    pc_source: PC_DC,
                    rd_source: RD_DC,
                    isBranch: 1'b0,

                    btype: sbtype_funct3,
                    syscall: 1'b0,
                    illegal_instr: 1'b0
                };
            end
        
            unique case (opcode)
                OP_OP: begin
                    ctrl_signals.useImm = 1'b0;
                    ctrl_signals.rfWrite = 1'b1;
                    ctrl_signals.pc2RF = 1'b0;
                    ctrl_signals.mem2RF = 1'b0;
                    ctrl_signals.memRead = 1'b0;
                    ctrl_signals.memWrite = 1'b0;
                    ctrl_signals.usePC = 1'b0;
                    ctrl_signals.imm_mode  = IMM_DC;
                    ctrl_signals.ldst_mode = LDST_DC;
                    ctrl_signals.pc_source = PC_plus4;

                    unique case (rtype_funct3)
                        // 3-bit function code for add or subtract
                        FUNCT3_ADD_SUB: begin
                            unique case (funct7)
                                // 7-bit function code for typical integer instructions
                                FUNCT7_INT: begin
                                    ctrl_signals.alu_op = ALU_ADD;
                                    ctrl_signals.rd_source = RD_ALU;
                                end
                                FUNCT7_ALT_INT: begin
                                    ctrl_signals.alu_op = ALU_SUB;
                                    ctrl_signals.rd_source = RD_ALU;
                                end
                                default: begin
                                    `display(rst_l, "Encountered unknown/unimplemented 7-bit function code 0x%02x.",
                                            funct7);
                                    ctrl_signals.illegal_instr = 1'b1;
                                end
                            endcase
                        end
                        FUNCT3_SLL: begin
                            ctrl_signals.alu_op = ALU_SLL;
                            ctrl_signals.rd_source = RD_ALU;
                        end
                        FUNCT3_SLT: begin
                            ctrl_signals.alu_op = ALU_SLT;
                            ctrl_signals.rd_source = RD_CMP;
                        end
                        FUNCT3_SLTU: begin
                            ctrl_signals.alu_op = ALU_SLTU;
                            ctrl_signals.rd_source = RD_CMP;
                        end
                        FUNCT3_XOR: begin
                            ctrl_signals.alu_op = ALU_XOR;
                            ctrl_signals.rd_source = RD_ALU;
                        end
                        FUNCT3_SRL_SRA: begin
                            unique case (funct7)
                                FUNCT7_INT: begin
                                    ctrl_signals.alu_op = ALU_SRL;
                                    ctrl_signals.rd_source = RD_ALU;
                                end
                                FUNCT7_ALT_INT: begin
                                    ctrl_signals.alu_op = ALU_SRA;
                                    ctrl_signals.rd_source = RD_ALU;
                                end
                                default: begin
                                    `display(rst_l, "Encountered unknown/unimplemented 7-bit function code 0x%02x.",
                                            funct7);
                                    ctrl_signals.illegal_instr = 1'b1;
                                end
                            endcase
                        end
                        FUNCT3_OR: begin
                            ctrl_signals.alu_op = ALU_OR;
                            ctrl_signals.rd_source = RD_ALU;
                        end
                        FUNCT3_AND: begin
                            ctrl_signals.alu_op = ALU_AND;
                            ctrl_signals.rd_source = RD_ALU;
                        end
                        default: begin
                            `display(rst_l, "Encountered unknown/unimplemented 3-bit rtype integer function code 0x%01x.",
                                    rtype_funct3);
                            ctrl_signals.illegal_instr = 1'b1;
                        end
                    endcase
                end

                // General I-type arithmetic operation
                OP_IMM: begin
                    ctrl_signals.useImm = 1'b1;
                    ctrl_signals.rfWrite = 1'b1;
                    ctrl_signals.pc2RF = 1'b0;
                    ctrl_signals.mem2RF = 1'b0;
                    ctrl_signals.memRead = 1'b0;
                    ctrl_signals.memWrite = 1'b0;
                    ctrl_signals.usePC = 1'b0;
                    ctrl_signals.imm_mode = IMM_I;
                    ctrl_signals.ldst_mode = LDST_DC;
                    ctrl_signals.pc_source = PC_plus4;

                    unique case (itype_int_funct3)
                        FUNCT3_ADDI: begin
                            ctrl_signals.rd_source = RD_ALU;
                            ctrl_signals.alu_op = ALU_ADD;
                        end

                        FUNCT3_SLTI: begin
                            ctrl_signals.rd_source = RD_CMP;
                            ctrl_signals.alu_op = ALU_SLT;
                        end

                        FUNCT3_SLTIU: begin
                            ctrl_signals.rd_source = RD_CMP;
                            ctrl_signals.alu_op = ALU_SLTU;
                        end

                        FUNCT3_XORI: begin
                            ctrl_signals.rd_source = RD_ALU;
                            ctrl_signals.alu_op = ALU_XOR;
                        end

                        FUNCT3_ORI: begin
                            ctrl_signals.rd_source = RD_ALU;
                            ctrl_signals.alu_op = ALU_OR;
                        end

                        FUNCT3_ANDI: begin
                            ctrl_signals.rd_source = RD_ALU;
                            ctrl_signals.alu_op = ALU_AND;
                        end

                        FUNCT3_SLLI: begin
                            ctrl_signals.rd_source = RD_ALU;
                            ctrl_signals.alu_op = ALU_SLL;
                        end

                        FUNCT3_SRLI_SRAI: begin
                            unique case (funct7)
                                FUNCT7_INT: begin
                                    ctrl_signals.rd_source = RD_ALU;
                                    ctrl_signals.alu_op = ALU_SRL;
                                end
                                FUNCT7_ALT_INT: begin
                                    ctrl_signals.rd_source = RD_ALU;
                                    ctrl_signals.alu_op = ALU_SRA;
                                end
                                default: begin
                                    `display(rst_l, "Encountered unknown/unimplemented 7-bit function code 0x%02x.",
                                            funct7);
                                    ctrl_signals.illegal_instr = 1'b1;
                                end
                            endcase
                        end

                        default: begin
                            `display(rst_l, "Encountered unknown/unimplemented 3-bit itype integer function code 0x%01x.",
                                    itype_int_funct3);
                            ctrl_signals.illegal_instr = 1'b1;
                        end
                    endcase
                end

                // load instructions (i-type)
                OP_LOAD: begin
                    ctrl_signals.useImm = 1'b1;
                    ctrl_signals.rfWrite = 1'b1;
                    ctrl_signals.mem2RF = 1'b1;
                    ctrl_signals.rd_source = RD_MMU;
                    ctrl_signals.pc2RF = 1'b0;
                    ctrl_signals.usePC = 1'b0;
                    ctrl_signals.memRead = 1'b1;
                    ctrl_signals.memWrite = 1'b0;
                    ctrl_signals.imm_mode = IMM_I;
                    ctrl_signals.alu_op = ALU_ADD;
                    ctrl_signals.pc_source = PC_plus4;
    
                    unique case (itype_load_funct3)
                        FUNCT3_LW: begin
                            ctrl_signals.ldst_mode = LDST_W;
                        end

                        FUNCT3_LH: begin
                            ctrl_signals.ldst_mode = LDST_H;
                        end

                        FUNCT3_LHU: begin
                            ctrl_signals.ldst_mode = LDST_HU;
                        end

                        FUNCT3_LB: begin
                            ctrl_signals.ldst_mode = LDST_B;
                        end

                        FUNCT3_LBU: begin
                            ctrl_signals.ldst_mode = LDST_BU;
                        end

                        default: begin
                            `display(rst_l, "Encountered unknown/unimplemented 3-bit itype load code 0x%01x.",
                                    itype_load_funct3);
                            ctrl_signals.illegal_instr = 1'b1;
                        end
                    endcase
                end
        
                // store instructions (s-type)
                OP_STORE: begin
                    ctrl_signals.useImm = 1'b1;
                    ctrl_signals.usePC = 1'b0;
                    ctrl_signals.rfWrite = 1'b0;
                    ctrl_signals.pc2RF = 1'b0;
                    ctrl_signals.memRead = 1'b0;
                    ctrl_signals.memWrite = 1'b1;
                    ctrl_signals.imm_mode = IMM_S;
                    ctrl_signals.alu_op = ALU_ADD;
                    ctrl_signals.pc_source = PC_plus4;

                    unique case (stype_funct3)
                        FUNCT3_SW: begin
                            ctrl_signals.ldst_mode = LDST_W;
                        end

                        FUNCT3_SH: begin
                            ctrl_signals.ldst_mode = LDST_H;
                        end

                        FUNCT3_SB: begin
                            ctrl_signals.ldst_mode = LDST_B;
                        end

                        default: begin
                            `display(rst_l, "Encountered unknown/unimplemented 3-bit stype store code 0x%01x.",
                                    stype_funct3);
                            ctrl_signals.illegal_instr = 1'b1;
                        end
                    endcase
                end
                // branch instructions (b-type)
                OP_BRANCH: begin
                    ctrl_signals.isBranch = 1'b1;
                    ctrl_signals.useImm = 1'b0;
                    ctrl_signals.rfWrite = 1'b0;
                    // ctrl_signals.mem2RF=1'bx;
                    ctrl_signals.usePC = 1'b0;
                    ctrl_signals.pc2RF = 1'bx;
                    ctrl_signals.memRead = 1'b0;
                    ctrl_signals.memWrite = 1'b0;
                    ctrl_signals.imm_mode = IMM_SB;
                    ctrl_signals.alu_op = ALU_SUB;
                    ctrl_signals.ldst_mode = LDST_DC;
                    ctrl_signals.pc_source = PC_cond;
                    unique case (sbtype_funct3)
                        FUNCT3_BEQ: begin
                        ctrl_signals.alu_op = ALU_BEQ;
                        end
                    FUNCT3_BNE: begin
                        ctrl_signals.alu_op = ALU_BNE;
                        end
                        FUNCT3_BLT: begin
                        ctrl_signals.alu_op = ALU_BLT;
                        end

                        FUNCT3_BGE: begin
                        ctrl_signals.alu_op = ALU_BGE;
                        end

                        FUNCT3_BLTU: begin
                        ctrl_signals.alu_op = ALU_BLTU;
                        end

                        FUNCT3_BGEU: begin
                        ctrl_signals.alu_op = ALU_BGEU;
                        end

                        default: begin
                        `display(rst_l, "Encountered unknown/unimplemented 3-bit sbtype branch code 0x%01x.",
                                sbtype_funct3);
                        ctrl_signals.illegal_instr = 1'b1;
                        end
                    endcase
                end

                OP_AUIPC: begin
                    ctrl_signals.isBranch = 1'b0;
                    ctrl_signals.useImm = 1'b1;
                    ctrl_signals.rfWrite = 1'b1;
                    ctrl_signals.pc2RF = 1'b0;
                    ctrl_signals.memRead = 1'b0;
                    ctrl_signals.memWrite = 1'b0;
                    ctrl_signals.usePC = 1'b1;
                    ctrl_signals.imm_mode = IMM_U;
                    ctrl_signals.alu_op = ALU_ADD;
                    ctrl_signals.ldst_mode = LDST_DC;
                    ctrl_signals.pc_source = PC_plus4;
                    ctrl_signals.rd_source = RD_ALU;
                end

                OP_JAL: begin
                    ctrl_signals.isBranch = 1'b1;
                    ctrl_signals.useImm = 1'b1;
                    ctrl_signals.rfWrite = 1'b1;
                    ctrl_signals.pc2RF = 1'b1;
                    ctrl_signals.memRead = 1'b0;
                    ctrl_signals.memWrite = 1'b0;
                    ctrl_signals.usePC = 1'b1;
                    ctrl_signals.imm_mode = IMM_UJ;
                    ctrl_signals.alu_op = ALU_ADD;
                    ctrl_signals.ldst_mode = LDST_DC;
                    ctrl_signals.pc_source = PC_uncond;
                    ctrl_signals.rd_source = RD_PC4;
                end

                OP_JALR: begin
                    ctrl_signals.isBranch = 1'b1;
                    ctrl_signals.useImm = 1'b1;
                    ctrl_signals.rfWrite = 1'b1;
                    ctrl_signals.pc2RF = 1'b1;
                    ctrl_signals.memRead = 1'b0;
                    ctrl_signals.memWrite = 1'b0;
                    ctrl_signals.usePC = 1'b0;
                    ctrl_signals.imm_mode = IMM_I;
                    ctrl_signals.alu_op = ALU_ADD;
                    ctrl_signals.ldst_mode = LDST_DC;
                    ctrl_signals.pc_source = PC_indirect;
                    ctrl_signals.rd_source = RD_PC4;
                end

                OP_LUI: begin
                    ctrl_signals.useImm = 1'b1;
                    ctrl_signals.rfWrite = 1'b1;
                    ctrl_signals.pc2RF = 1'b0;
                    ctrl_signals.memRead = 1'b0;
                    ctrl_signals.memWrite = 1'b0;
                    ctrl_signals.usePC = 1'b0;
                    ctrl_signals.imm_mode = IMM_U;
                    ctrl_signals.alu_op = ALU_ADD;
                    ctrl_signals.ldst_mode = LDST_DC;
                    ctrl_signals.pc_source = PC_plus4;
                    ctrl_signals.rd_source = RD_IMM;
                end
                // General system operation
                OP_SYSTEM: begin
                    unique case (itype_funct12)
                        FUNCT12_ECALL: begin
                            ctrl_signals.syscall = 1'b1;
                            ctrl_signals.useImm = 1'b0;
                            ctrl_signals.rfWrite = 1'b0;
                            ctrl_signals.pc2RF = 1'b0;
                            ctrl_signals.memRead = 1'b0;
                            ctrl_signals.memWrite = 1'b0;
                            ctrl_signals.usePC = 1'b0;
                            ctrl_signals.imm_mode = IMM_DC;
                            ctrl_signals.alu_op = ALU_ADD;
                            ctrl_signals.ldst_mode = LDST_DC;
                            ctrl_signals.pc_source = PC_plus4;
                            ctrl_signals.rd_source = RD_DC;
                        end

                        default: begin
                            `display(rst_l, "Encountered unknown/unimplemented 12-bit itype function code 0x%03x.",
                                    itype_funct12);
                            ctrl_signals.illegal_instr = 1'b1;
                        end
                    endcase
                end

                default: begin
                    `display(rst_l, "Encountered unknown/unimplemented opcode 0x%02x.", opcode);
                    ctrl_signals.illegal_instr = 1'b1;
                end
            endcase

            // Only assert the illegal instruction exception after reset
            ctrl_signals.illegal_instr &= (rst_l & (instr != 'h0));
        end


endmodule: riscv_decode