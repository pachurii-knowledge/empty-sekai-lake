/**
 * rvc_expand.sv
 *
 * RV64C (compressed) 16-bit -> canonical 32-bit instruction expander.
 *
 * Pure combinational. Relocates every shuffled/scaled RVC immediate into the
 * STANDARD 32-bit field positions and emits the exact canonical base-ISA
 * encoding, so the UNCHANGED riscv_decode / immediate_for / positional
 * rs1[19:15]/rs2[24:20]/rd[11:7] path reproduces the RVC semantics with zero
 * changes downstream. Target is RV64GC: the RV32C-only encoding slots
 * (C.FLW/C.FSW/C.JAL/C.FLWSP/C.FSWSP) take their RV64 meaning instead and are
 * never decoded as FP-word / JAL.
 *
 * A parcel with c[1:0]==2'b11 is a 32-bit instruction and NEVER reaches here
 * (the realigner handles it via {hi,lo} concat). Illegal/reserved compressed
 * encodings emit x = {16'h0000, c} (which riscv_decode flags illegal, since
 * c[1:0]!=11 is not a valid base opcode) and assert illegal=1; the caller
 * carries the original parcel c for the mtval.
 *
 * The whole file body is `ifdef RVC-gated: with RVC unset it contributes zero
 * modules (the Makefile globs every src file into one Verilator build, so an
 * ungated body would compile in the non-C baseline).
 */
`ifdef RVC

`default_nettype none

module rvc_expand (
    input  wire logic [15:0] c,
    output logic [31:0]      x,
    output logic             illegal
);
`ifndef RV64
    // RVC is RV64GC-only in this core; the RV32C-divergent slots are not
    // implemented. The build-flag guard (verilator.mk) also enforces this.
    initial $error("rvc_expand: RVC requires RV64 (RV64GC only)");
`endif

    // -------- base opcodes (7-bit) --------
    localparam logic [6:0] OP_LOAD    = 7'b0000011;
    localparam logic [6:0] OP_LOAD_FP = 7'b0000111;
    localparam logic [6:0] OP_IMM     = 7'b0010011;
    localparam logic [6:0] OP_IMM_32  = 7'b0011011;   // *W I-form (RV64)
    localparam logic [6:0] OP_STORE   = 7'b0100011;
    localparam logic [6:0] OP_STORE_FP= 7'b0100111;
    localparam logic [6:0] OP_OP      = 7'b0110011;
    localparam logic [6:0] OP_OP_32   = 7'b0111011;   // *W R-form (RV64)
    localparam logic [6:0] OP_LUI     = 7'b0110111;
    localparam logic [6:0] OP_BRANCH  = 7'b1100011;
    localparam logic [6:0] OP_JALR    = 7'b1100111;
    localparam logic [6:0] OP_JAL     = 7'b1101111;

    // -------- standard-format packers (verify-once immediate placement) --------
    function automatic logic [31:0] enc_r(logic [6:0] f7, logic [4:0] rs2,
            logic [4:0] rs1, logic [2:0] f3, logic [4:0] rd, logic [6:0] op);
        enc_r = {f7, rs2, rs1, f3, rd, op};
    endfunction
    function automatic logic [31:0] enc_i(logic [11:0] imm, logic [4:0] rs1,
            logic [2:0] f3, logic [4:0] rd, logic [6:0] op);
        enc_i = {imm, rs1, f3, rd, op};
    endfunction
    function automatic logic [31:0] enc_s(logic [11:0] imm, logic [4:0] rs2,
            logic [4:0] rs1, logic [2:0] f3, logic [6:0] op);
        enc_s = {imm[11:5], rs2, rs1, f3, imm[4:0], op};
    endfunction
    function automatic logic [31:0] enc_b(logic [12:0] imm, logic [4:0] rs2,
            logic [4:0] rs1, logic [2:0] f3, logic [6:0] op);
        enc_b = {imm[12], imm[10:5], rs2, rs1, f3, imm[4:1], imm[11], op};
    endfunction
    function automatic logic [31:0] enc_u(logic [19:0] imm20, logic [4:0] rd,
            logic [6:0] op);
        enc_u = {imm20, rd, op};                       // imm20 = instr[31:12]
    endfunction
    function automatic logic [31:0] enc_j(logic [20:0] imm, logic [4:0] rd,
            logic [6:0] op);
        enc_j = {imm[20], imm[10:1], imm[11], imm[19:12], rd, op};
    endfunction

    // -------- decoded compressed fields --------
    logic [1:0] quad;   logic [2:0] f3;
    logic [4:0] rd_rs1; logic [4:0] rs2_c;            // full 5-bit fields (C2)
    logic [4:0] rp_hi, rp_lo;                         // x8+.. primed regs
    assign quad   = c[1:0];
    assign f3     = c[15:13];
    assign rd_rs1 = c[11:7];
    assign rs2_c  = c[6:2];
    assign rp_hi  = {2'b01, c[9:7]};                  // rd'/rs1'  (x8..x15)
    assign rp_lo  = {2'b01, c[4:2]};                  // rs2'/rd'  (x8..x15)

    // -------- immediates (shuffled/scaled -> value) --------
    logic [5:0]  imm6;          // {c12,c6:2}
    logic [11:0] simm6;         // sext(imm6)
    logic [5:0]  shamt6;        // {c12,c6:2}
    logic [11:0] addi4spn_u;    // C.ADDI4SPN nzuimm (zero-ext)
    logic [11:0] addi16sp_s;    // C.ADDI16SP nzimm (sext)
    logic [19:0] lui_u;         // C.LUI instr[31:12]
    logic [11:0] uimm_cld;      // C.FLD/C.FSD/C.LD/C.SD  (8b)
    logic [11:0] uimm_clw;      // C.LW/C.SW              (7b)
    logic [11:0] uimm_ldsp;     // C.FLDSP/C.LDSP         (9b)
    logic [11:0] uimm_lwsp;     // C.LWSP                 (8b)
    logic [11:0] uimm_sdsp;     // C.FSDSP/C.SDSP         (9b)
    logic [11:0] uimm_swsp;     // C.SWSP                 (8b)
    logic [20:0] cj_off;        // C.J   (sext, x2)
    logic [12:0] cb_off;        // C.BEQZ/C.BNEZ (sext, x2)

    assign imm6   = {c[12], c[6:2]};
    assign simm6  = {{6{imm6[5]}}, imm6};
    assign shamt6 = {c[12], c[6:2]};
    // C.ADDI4SPN: nzuimm[9:6]=c[10:7], [5:4]=c[12:11], [3]=c[5], [2]=c[6]
    assign addi4spn_u = {2'b00, c[10:7], c[12:11], c[5], c[6], 2'b00};
    // C.ADDI16SP: nzimm[9]=c12,[8:7]=c[4:3],[6]=c5,[5]=c2,[4]=c6, sext
    assign addi16sp_s = {{2{c[12]}}, c[12], c[4:3], c[5], c[2], c[6], 4'b0000};
    // C.LUI: instr[31:12] = sext({c12,c6:2}) with the 6-bit value at [17:12]
    assign lui_u = {{14{c[12]}}, c[12], c[6:2]};
    // scaled load/store offsets (zero-extended)
    assign uimm_cld  = {4'b0000, c[6:5], c[12:10], 3'b000};
    assign uimm_clw  = {5'b00000, c[5], c[12:10], c[6], 2'b00};
    assign uimm_ldsp = {3'b000, c[4:2], c[12], c[6:5], 3'b000};
    assign uimm_lwsp = {4'b0000, c[3:2], c[12], c[6:4], 2'b00};
    assign uimm_sdsp = {3'b000, c[9:7], c[12:10], 3'b000};
    assign uimm_swsp = {4'b0000, c[8:7], c[12:9], 2'b00};
    // C.J: imm[11]=c12,[10]=c8,[9:8]=c[10:9],[7]=c6,[6]=c7,[5]=c2,[4]=c11,[3:1]=c[5:3]
    assign cj_off = {{9{c[12]}}, c[12], c[8], c[10:9], c[6], c[7], c[2],
                     c[11], c[5:3], 1'b0};
    // C.Bxx: imm[8]=c12,[7:6]=c[6:5],[5]=c2,[4:3]=c[11:10],[2:1]=c[4:3]
    assign cb_off = {{4{c[12]}}, c[12], c[6:5], c[2], c[11:10], c[4:3], 1'b0};

    always_comb begin
        x       = {16'h0000, c};     // default: illegal carrier (parcel in low 16)
        illegal = 1'b1;

        unique case (quad)
        // ============================ C0 ============================
        2'b00: unique case (f3)
            3'b000: begin                                   // C.ADDI4SPN
                if (c[12:5] == 8'd0) begin illegal = 1'b1; x = {16'h0000, c}; end
                else begin
                    x = enc_i(addi4spn_u, 5'd2, 3'b000, rp_lo, OP_IMM);
                    illegal = 1'b0;
                end
            end
            3'b001: begin                                   // C.FLD
                x = enc_i(uimm_cld, rp_hi, 3'b011, rp_lo, OP_LOAD_FP);
                illegal = 1'b0;
            end
            3'b010: begin                                   // C.LW
                x = enc_i(uimm_clw, rp_hi, 3'b010, rp_lo, OP_LOAD);
                illegal = 1'b0;
            end
            3'b011: begin                                   // C.LD (RV64)
                x = enc_i(uimm_cld, rp_hi, 3'b011, rp_lo, OP_LOAD);
                illegal = 1'b0;
            end
            3'b100: begin illegal = 1'b1; x = {16'h0000, c}; end   // reserved
            3'b101: begin                                   // C.FSD
                x = enc_s(uimm_cld, rp_lo, rp_hi, 3'b011, OP_STORE_FP);
                illegal = 1'b0;
            end
            3'b110: begin                                   // C.SW
                x = enc_s(uimm_clw, rp_lo, rp_hi, 3'b010, OP_STORE);
                illegal = 1'b0;
            end
            3'b111: begin                                   // C.SD (RV64)
                x = enc_s(uimm_cld, rp_lo, rp_hi, 3'b011, OP_STORE);
                illegal = 1'b0;
            end
        endcase
        // ============================ C1 ============================
        2'b01: unique case (f3)
            3'b000: begin                                   // C.ADDI / C.NOP / HINT
                x = enc_i(simm6, rd_rs1, 3'b000, rd_rs1, OP_IMM);
                illegal = 1'b0;                             // rd==0/imm==0 -> HINT/NOP
            end
            3'b001: begin                                   // C.ADDIW (RV64)
                if (rd_rs1 == 5'd0) begin illegal = 1'b1; x = {16'h0000, c}; end
                else begin
                    x = enc_i(simm6, rd_rs1, 3'b000, rd_rs1, OP_IMM_32);
                    illegal = 1'b0;
                end
            end
            3'b010: begin                                   // C.LI (rd==0 HINT)
                x = enc_i(simm6, 5'd0, 3'b000, rd_rs1, OP_IMM);
                illegal = 1'b0;
            end
            3'b011: begin
                if (rd_rs1 == 5'd2) begin                   // C.ADDI16SP
                    if (imm6 == 6'd0) begin illegal = 1'b1; x = {16'h0000, c}; end
                    else begin
                        x = enc_i(addi16sp_s, 5'd2, 3'b000, 5'd2, OP_IMM);
                        illegal = 1'b0;
                    end
                end else begin                              // C.LUI (rd==0 HINT)
                    if (imm6 == 6'd0) begin illegal = 1'b1; x = {16'h0000, c}; end
                    else begin
                        x = enc_u(lui_u, rd_rs1, OP_LUI);
                        illegal = 1'b0;
                    end
                end
            end
            3'b100: begin                                   // minor ALU
                unique case (c[11:10])
                    2'b00: begin                            // C.SRLI
                        x = enc_i({6'b000000, shamt6}, rp_hi, 3'b101, rp_hi, OP_IMM);
                        illegal = 1'b0;
                    end
                    2'b01: begin                            // C.SRAI
                        x = enc_i({6'b010000, shamt6}, rp_hi, 3'b101, rp_hi, OP_IMM);
                        illegal = 1'b0;
                    end
                    2'b10: begin                            // C.ANDI
                        x = enc_i(simm6, rp_hi, 3'b111, rp_hi, OP_IMM);
                        illegal = 1'b0;
                    end
                    2'b11: begin                            // reg-reg
                        unique case ({c[12], c[6:5]})
                            3'b000: begin x = enc_r(7'b0100000, rp_lo, rp_hi, 3'b000, rp_hi, OP_OP);    illegal = 1'b0; end // SUB
                            3'b001: begin x = enc_r(7'b0000000, rp_lo, rp_hi, 3'b100, rp_hi, OP_OP);    illegal = 1'b0; end // XOR
                            3'b010: begin x = enc_r(7'b0000000, rp_lo, rp_hi, 3'b110, rp_hi, OP_OP);    illegal = 1'b0; end // OR
                            3'b011: begin x = enc_r(7'b0000000, rp_lo, rp_hi, 3'b111, rp_hi, OP_OP);    illegal = 1'b0; end // AND
                            3'b100: begin x = enc_r(7'b0100000, rp_lo, rp_hi, 3'b000, rp_hi, OP_OP_32); illegal = 1'b0; end // SUBW (RV64)
                            3'b101: begin x = enc_r(7'b0000000, rp_lo, rp_hi, 3'b000, rp_hi, OP_OP_32); illegal = 1'b0; end // ADDW (RV64)
                            default: begin illegal = 1'b1; x = {16'h0000, c}; end                                          // reserved
                        endcase
                    end
                endcase
            end
            3'b101: begin                                   // C.J
                x = enc_j(cj_off, 5'd0, OP_JAL);
                illegal = 1'b0;
            end
            3'b110: begin                                   // C.BEQZ
                x = enc_b(cb_off, 5'd0, rp_hi, 3'b000, OP_BRANCH);
                illegal = 1'b0;
            end
            3'b111: begin                                   // C.BNEZ
                x = enc_b(cb_off, 5'd0, rp_hi, 3'b001, OP_BRANCH);
                illegal = 1'b0;
            end
        endcase
        // ============================ C2 ============================
        2'b10: unique case (f3)
            3'b000: begin                                   // C.SLLI (rd==0/shamt==0 HINT)
                x = enc_i({6'b000000, shamt6}, rd_rs1, 3'b001, rd_rs1, OP_IMM);
                illegal = 1'b0;
            end
            3'b001: begin                                   // C.FLDSP
                x = enc_i(uimm_ldsp, 5'd2, 3'b011, rd_rs1, OP_LOAD_FP);
                illegal = 1'b0;
            end
            3'b010: begin                                   // C.LWSP (rd==0 reserved)
                if (rd_rs1 == 5'd0) begin illegal = 1'b1; x = {16'h0000, c}; end
                else begin
                    x = enc_i(uimm_lwsp, 5'd2, 3'b010, rd_rs1, OP_LOAD);
                    illegal = 1'b0;
                end
            end
            3'b011: begin                                   // C.LDSP (RV64, rd==0 reserved)
                if (rd_rs1 == 5'd0) begin illegal = 1'b1; x = {16'h0000, c}; end
                else begin
                    x = enc_i(uimm_ldsp, 5'd2, 3'b011, rd_rs1, OP_LOAD);
                    illegal = 1'b0;
                end
            end
            3'b100: begin
                if (c[12] == 1'b0) begin
                    if (rs2_c == 5'd0) begin                // C.JR (rs1==0 reserved)
                        if (rd_rs1 == 5'd0) begin illegal = 1'b1; x = {16'h0000, c}; end
                        else begin
                            x = enc_i(12'd0, rd_rs1, 3'b000, 5'd0, OP_JALR);
                            illegal = 1'b0;
                        end
                    end else begin                          // C.MV (rd==0 HINT)
                        x = enc_r(7'b0000000, rs2_c, 5'd0, 3'b000, rd_rs1, OP_OP);
                        illegal = 1'b0;
                    end
                end else begin
                    if (rs2_c == 5'd0) begin
                        if (rd_rs1 == 5'd0) begin           // C.EBREAK
                            x = 32'h0010_0073;
                            illegal = 1'b0;
                        end else begin                      // C.JALR
                            x = enc_i(12'd0, rd_rs1, 3'b000, 5'd1, OP_JALR);
                            illegal = 1'b0;
                        end
                    end else begin                          // C.ADD (rd==0 HINT)
                        x = enc_r(7'b0000000, rs2_c, rd_rs1, 3'b000, rd_rs1, OP_OP);
                        illegal = 1'b0;
                    end
                end
            end
            3'b101: begin                                   // C.FSDSP
                x = enc_s(uimm_sdsp, rs2_c, 5'd2, 3'b011, OP_STORE_FP);
                illegal = 1'b0;
            end
            3'b110: begin                                   // C.SWSP
                x = enc_s(uimm_swsp, rs2_c, 5'd2, 3'b010, OP_STORE);
                illegal = 1'b0;
            end
            3'b111: begin                                   // C.SDSP (RV64)
                x = enc_s(uimm_sdsp, rs2_c, 5'd2, 3'b011, OP_STORE);
                illegal = 1'b0;
            end
        endcase
        // c[1:0]==11 is a 32-bit instruction; must never reach the expander.
        2'b11: begin illegal = 1'b1; x = {16'h0000, c}; end
        endcase
    end

endmodule : rvc_expand

`default_nettype wire

`endif /* RVC */
