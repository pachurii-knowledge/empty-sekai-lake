`include "ooo_types.vh"

`default_nettype none

module rename_map_table
    import OOO_Types::*;
(
    input  logic                         clk,
    input  logic                         rst_l,
    input  logic                         restore_valid,
    input  phys_reg_t                    restore_map [32],
    input  logic [OOO_WIDTH-1:0]         rename_valid,
    input  arch_reg_t                    rs1 [OOO_WIDTH],
    input  arch_reg_t                    rs2 [OOO_WIDTH],
    input  arch_reg_t                    rd [OOO_WIDTH],
    input  phys_reg_t                    alloc_prd [OOO_WIDTH],
    output phys_reg_t                    prs1 [OOO_WIDTH],
    output phys_reg_t                    prs2 [OOO_WIDTH],
    output phys_reg_t                    old_prd [OOO_WIDTH],
    output logic [OOO_WIDTH-1:0]         has_dest,
    output phys_reg_t                    snapshot_map [32]
);

    phys_reg_t map_q [32];
    phys_reg_t map_next [32];

    always_comb begin
        for (int i = 0; i < 32; i += 1) begin
            map_next[i] = restore_valid ? restore_map[i] : map_q[i];
            snapshot_map[i] = map_q[i];
        end

        for (int lane = 0; lane < OOO_WIDTH; lane += 1) begin
            prs1[lane] = (rs1[lane] == 5'd0) ? '0 : map_next[rs1[lane]];
            prs2[lane] = (rs2[lane] == 5'd0) ? '0 : map_next[rs2[lane]];
            old_prd[lane] = (rd[lane] == 5'd0) ? '0 : map_next[rd[lane]];
            has_dest[lane] = rename_valid[lane] && (rd[lane] != 5'd0);

            if (has_dest[lane]) begin
                map_next[rd[lane]] = alloc_prd[lane];
            end
        end
    end

    always_ff @(posedge clk or negedge rst_l) begin
        if (!rst_l) begin
            for (int i = 0; i < 32; i += 1) begin
                map_q[i] <= phys_reg_t'(i);
            end
        end else begin
            for (int i = 0; i < 32; i += 1) begin
                map_q[i] <= map_next[i];
            end
        end
    end

endmodule: rename_map_table
