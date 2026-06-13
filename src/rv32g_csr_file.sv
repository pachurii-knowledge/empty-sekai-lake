`default_nettype none

module rv32g_csr_file (
    input wire logic        clk,
    input wire logic        rst_l,
    input wire logic        retire,
    input wire logic        write_valid,
    input wire logic [11:0] write_addr,
    input wire logic [31:0] write_data,
    input wire logic        fp_fflags_valid,
    input wire logic [4:0]  fp_fflags,
    input wire logic [11:0] read_addr0,
    input wire logic [11:0] read_addr1,
    output logic [31:0] read_data0,
    output logic [31:0] read_data1,
    output logic        read_illegal0,
    output logic        read_illegal1,
    output logic [2:0]  frm_value
);

    localparam logic [11:0] CSR_FFLAGS   = 12'h001;
    localparam logic [11:0] CSR_FRM      = 12'h002;
    localparam logic [11:0] CSR_FCSR     = 12'h003;
    localparam logic [11:0] CSR_CYCLE    = 12'hC00;
    localparam logic [11:0] CSR_TIME     = 12'hC01;
    localparam logic [11:0] CSR_INSTRET  = 12'hC02;
    localparam logic [11:0] CSR_CYCLEH   = 12'hC80;
    localparam logic [11:0] CSR_TIMEH    = 12'hC81;
    localparam logic [11:0] CSR_INSTRETH = 12'hC82;

    logic [63:0] cycle_q;
    logic [63:0] instret_q;
    logic [4:0] fflags_q;
    logic [2:0] frm_q;
    logic [4:0] fflags_next;
    logic [2:0] frm_next;

    assign frm_value = frm_q;

    always_comb begin
        read_csr(read_addr0, read_data0, read_illegal0);
        read_csr(read_addr1, read_data1, read_illegal1);
    end

    task automatic read_csr(input logic [11:0] addr,
            output logic [31:0] data, output logic illegal);
        illegal = 1'b0;
        unique case (addr)
            CSR_FFLAGS:   data = {27'b0, fflags_q};
            CSR_FRM:      data = {29'b0, frm_q};
            CSR_FCSR:     data = {24'b0, frm_q, fflags_q};
            CSR_CYCLE,
            CSR_TIME:     data = cycle_q[31:0];
            CSR_INSTRET:  data = instret_q[31:0];
            CSR_CYCLEH,
            CSR_TIMEH:    data = cycle_q[63:32];
            CSR_INSTRETH: data = instret_q[63:32];
            default: begin
                data = 32'b0;
                illegal = 1'b1;
            end
        endcase
    endtask

    always_ff @(posedge clk or negedge rst_l) begin
        if (!rst_l) begin
            cycle_q <= 64'b0;
            instret_q <= 64'b0;
            fflags_q <= 5'b0;
            frm_q <= 3'b0;
        end else begin
            cycle_q <= cycle_q + 64'd1;
            if (retire) begin
                instret_q <= instret_q + 64'd1;
            end
            fflags_q <= fflags_next;
            frm_q <= frm_next;
        end
    end

    always_comb begin
        fflags_next = fflags_q;
        frm_next = frm_q;
        if (write_valid) begin
            unique case (write_addr)
                CSR_FFLAGS: fflags_next = write_data[4:0];
                CSR_FRM:    frm_next = write_data[2:0];
                CSR_FCSR: begin
                    fflags_next = write_data[4:0];
                    frm_next = write_data[7:5];
                end
                default: begin
                end
            endcase
        end
        if (fp_fflags_valid) begin
            fflags_next |= fp_fflags;
        end
    end



endmodule: rv32g_csr_file
