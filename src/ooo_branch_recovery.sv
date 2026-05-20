`include "ooo_types.vh"

`default_nettype none

module ooo_branch_recovery
    import OOO_Types::*;
(
    input  writeback_packet_t branch_writeback,
    input  branch_mask_t      stack_reset_mask,
    input  branch_mask_t      stack_abort_mask,
    input  logic              stack_restore_valid,
    input  logic [31:0]       fetch_pc_plus4,
    output logic              resolve_valid,
    output branch_id_t        resolve_id,
    output logic              resolve_mispredict,
    output branch_mask_t      reset_mask,
    output branch_mask_t      abort_mask,
    output logic              redirect_valid,
    output logic [31:0]       redirect_pc
);

    always_comb begin
        resolve_valid = branch_writeback.valid && branch_writeback.branch_valid;
        resolve_id = branch_writeback.branch_id;
        resolve_mispredict = branch_writeback.branch_mispredict;
        reset_mask = stack_reset_mask;
        abort_mask = stack_abort_mask;
        redirect_valid = stack_restore_valid ||
            (resolve_valid && branch_writeback.branch_mispredict);
        redirect_pc = redirect_valid ? branch_writeback.redirect_pc : fetch_pc_plus4;
    end

endmodule: ooo_branch_recovery
