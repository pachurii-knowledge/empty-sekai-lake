`include "ooo_types.vh"
`include "riscv_abi.vh"
`include "memory_segments.vh"

`default_nettype none

module phys_reg_file
    import OOO_Types::*;
    import RISCV_ABI::SP, RISCV_ABI::GP;
    import MemorySegments::STACK_END, MemorySegments::USER_DATA_START;
#(
    parameter int READ_PORTS = OOO_WIDTH
)
(
    input  logic                 clk,
    input  logic                 rst_l,
    input  phys_reg_t            rs1 [READ_PORTS],
    input  phys_reg_t            rs2 [READ_PORTS],
    input  logic [OOO_WIDTH-1:0] write_valid,
    input  phys_reg_t            write_prd [OOO_WIDTH],
    input  logic [OOO_WIDTH-1:0][XLEN-1:0] write_data,
    output logic [READ_PORTS-1:0][XLEN-1:0] rs1_data,
    output logic [READ_PORTS-1:0][XLEN-1:0] rs2_data
);

    logic [PHYS_REGS-1:0][XLEN-1:0] registers;

    always_comb begin
        for (int i = 0; i < READ_PORTS; i += 1) begin
            rs1_data[i] = (rs1[i] == '0) ? '0 : registers[rs1[i]];
            rs2_data[i] = (rs2[i] == '0) ? '0 : registers[rs2[i]];
            for (int w = 0; w < OOO_WIDTH; w += 1) begin
                if (write_valid[w] && (write_prd[w] != '0)) begin
                    if (write_prd[w] == rs1[i]) begin
                        rs1_data[i] = write_data[w];
                    end
                    if (write_prd[w] == rs2[i]) begin
                        rs2_data[i] = write_data[w];
                    end
                end
            end
        end
    end

    always_ff @(posedge clk or negedge rst_l) begin
        if (!rst_l) begin
            registers <= '0;
            registers[SP] <= STACK_END;
            registers[GP] <= USER_DATA_START;
        end else begin
            for (int i = 0; i < OOO_WIDTH; i += 1) begin
                if (write_valid[i] && (write_prd[i] != '0)) begin
                    registers[write_prd[i]] <= write_data[i];
                end
            end
        end
    end

endmodule: phys_reg_file
