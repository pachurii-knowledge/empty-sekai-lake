`include "ooo_types.vh"

`default_nettype none

module rename_map_table
    import OOO_Types::*;
(
    input wire logic                         clk,
    input wire logic                         rst_l,
    input wire logic                         restore_valid,
    input wire phys_reg_t                    restore_map [32],
    input wire logic [OOO_WIDTH-1:0]         rename_valid,
    input wire arch_reg_t                    rs1 [OOO_WIDTH],
    input wire arch_reg_t                    rs2 [OOO_WIDTH],
    input wire arch_reg_t                    rd [OOO_WIDTH],
    input wire logic [OOO_WIDTH-1:0]         rename_has_dest,
    input wire phys_reg_t                    alloc_prd [OOO_WIDTH],
    output phys_reg_t                    prs1 [OOO_WIDTH],
    output phys_reg_t                    prs2 [OOO_WIDTH],
    output phys_reg_t                    old_prd [OOO_WIDTH],
    output logic [OOO_WIDTH-1:0]         has_dest,
    output phys_reg_t                    snapshot_map [32]
);

    phys_reg_t map_q [32];
    phys_reg_t map_next [32];
    // Base map for this cycle: the registered map (or the restore snapshot). The
    // per-lane intra-group RAW bypass reads THIS (registered) plus explicit
    // prior-lane overrides -- NOT a combinationally-updated map_next. The former
    // code read+wrote map_next[var] in one always_comb (variable read & write
    // indices), which Vivado models as a combinational read-after-write on one
    // memory -> a (false) structural loop (the LUTLP-1 5713-LUT loop, merged with
    // free_list/active_list under -flatten_hierarchy rebuilt; "timing analysis may
    // not be accurate"). The explicit bypass is value-identical (last prior in-
    // group writer of an arch reg wins) and loop-free: map_next is write-only here
    // (consumed only by the register), so there is no combinational map_next read.
    phys_reg_t base_map [32];

    always_comb begin
        for (int i = 0; i < 32; i += 1) begin
            base_map[i] = restore_valid ? restore_map[i] : map_q[i];
            snapshot_map[i] = map_q[i];
        end

        for (int lane = 0; lane < OOO_WIDTH; lane += 1) begin
            has_dest[lane] = rename_valid[lane] && rename_has_dest[lane] &&
                (rd[lane] != 5'd0);
        end

        // Source/old-dest rename: registered base, overridden by the latest
        // earlier in-group lane that writes the same arch reg (j increasing ->
        // higher j wins, matching the former sequential map_next overwrite).
        for (int lane = 0; lane < OOO_WIDTH; lane += 1) begin
            prs1[lane] = (rs1[lane] == 5'd0) ? '0 : base_map[rs1[lane]];
            prs2[lane] = (rs2[lane] == 5'd0) ? '0 : base_map[rs2[lane]];
            old_prd[lane] = (rd[lane] == 5'd0) ? '0 : base_map[rd[lane]];
            for (int j = 0; j < OOO_WIDTH; j += 1) begin
                if (j < lane && has_dest[j]) begin
                    if ((rs1[lane] != 5'd0) && (rd[j] == rs1[lane])) begin
                        prs1[lane] = alloc_prd[j];
                    end
                    if ((rs2[lane] != 5'd0) && (rd[j] == rs2[lane])) begin
                        prs2[lane] = alloc_prd[j];
                    end
                    if ((rd[lane] != 5'd0) && (rd[j] == rd[lane])) begin
                        old_prd[lane] = alloc_prd[j];
                    end
                end
            end
        end

        // Map update for registration only (write-only into map_next; later
        // lanes override earlier -> last writer of an arch reg wins).
        for (int i = 0; i < 32; i += 1) begin
            map_next[i] = base_map[i];
        end
        for (int lane = 0; lane < OOO_WIDTH; lane += 1) begin
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
