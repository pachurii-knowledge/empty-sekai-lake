`include "ooo_types.vh"
`include "superscalar_types.vh"

`default_nettype none

module tb_ooo_structures;
    import OOO_Types::*;

    logic clk;
    logic rst_l;

    phys_reg_t restore_map [32];
    phys_reg_t snapshot_map [32];
    arch_reg_t rs1 [OOO_WIDTH];
    arch_reg_t rs2 [OOO_WIDTH];
    arch_reg_t rd [OOO_WIDTH];
    phys_reg_t alloc_prd [OOO_WIDTH];
    phys_reg_t prs1 [OOO_WIDTH];
    phys_reg_t prs2 [OOO_WIDTH];
    phys_reg_t old_prd [OOO_WIDTH];
    logic [OOO_WIDTH-1:0] lane_valid;
    logic [OOO_WIDTH-1:0] has_dest;

    logic [$clog2(PHYS_REGS)-1:0] free_head;
    logic [$clog2(PHYS_REGS)-1:0] free_tail;
    logic [$clog2(PHYS_REGS+1)-1:0] free_count;
    logic [OOO_WIDTH-1:0] free_valid;
    phys_reg_t free_prd [OOO_WIDTH];
    logic [OOO_WIDTH-1:0] alloc_valid;
    logic free_can_allocate;

    logic [OOO_WIDTH-1:0] src1_ready;
    logic [OOO_WIDTH-1:0] src2_ready;

    branch_mask_t current_mask;
    branch_mask_t reset_mask;
    branch_mask_t abort_mask;
    branch_id_t allocate_id;
    logic branch_full;
    logic branch_allocate_valid;
    logic restore_valid;
    active_id_t restore_active_tail;

    always #5 clk = ~clk;

    rename_map_table MapTable (
        .clk,
        .rst_l,
        .restore_valid(1'b0),
        .restore_map,
        .rename_valid(lane_valid),
        .rs1,
        .rs2,
        .rd,
        .alloc_prd,
        .prs1,
        .prs2,
        .old_prd,
        .has_dest,
        .snapshot_map
    );

    free_list FreeList (
        .clk,
        .rst_l,
        .restore_valid(1'b0),
        .restore_head('0),
        .restore_tail('0),
        .restore_count('0),
        .alloc_req(has_dest),
        .free_valid,
        .free_prd,
        .alloc_valid,
        .alloc_prd,
        .can_allocate(free_can_allocate),
        .snapshot_head(free_head),
        .snapshot_tail(free_tail),
        .snapshot_count(free_count)
    );

    busy_table BusyTable (
        .clk,
        .rst_l,
        .allocate_valid(alloc_valid),
        .allocate_prd(alloc_prd),
        .writeback_valid(free_valid),
        .writeback_prd(free_prd),
        .src1_prd(prs1),
        .src2_prd(prs2),
        .src1_ready,
        .src2_ready
    );

    branch_stack BranchStack (
        .clk,
        .rst_l,
        .allocate(1'b1),
        .active_tail_snapshot('0),
        .free_head_snapshot(free_head),
        .free_tail_snapshot(free_tail),
        .free_count_snapshot(free_count),
        .map_snapshot(snapshot_map),
        .resolve(1'b0),
        .resolve_id('0),
        .mispredict(1'b0),
        .full(branch_full),
        .allocate_valid(branch_allocate_valid),
        .allocate_id,
        .current_mask,
        .restore_valid,
        .restore_active_tail,
        .restore_free_head(),
        .restore_free_tail(),
        .restore_free_count(),
        .restore_map(),
        .reset_mask,
        .abort_mask
    );

    initial begin
        clk = 1'b0;
        rst_l = 1'b0;
        lane_valid = 4'b0000;
        free_valid = 4'b0000;
        for (int i = 0; i < 32; i += 1) begin
            restore_map[i] = phys_reg_t'(i);
        end
        for (int i = 0; i < OOO_WIDTH; i += 1) begin
            rs1[i] = '0;
            rs2[i] = '0;
            rd[i] = '0;
            free_prd[i] = '0;
        end

        #12 rst_l = 1'b1;
        lane_valid = 4'b0011;
        rs1[0] = 5'd1;
        rs2[0] = 5'd2;
        rd[0] = 5'd3;
        rs1[1] = 5'd3;
        rs2[1] = 5'd4;
        rd[1] = 5'd5;
        #10;
        assert(has_dest[0] && has_dest[1]) else $fatal("rename dest detection failed");
        assert(prs1[1] == alloc_prd[0]) else $fatal("same-cycle rename bypass failed");
        assert(free_can_allocate) else $fatal("free list allocation unexpectedly blocked");
        #20;
        $finish;
    end

endmodule: tb_ooo_structures
