`include "ooo_types.vh"

`default_nettype none

module ooo_commit_unit
    import OOO_Types::*;
#(
    // M4-S5b: when set, a store-conditional is resolved by the LSQ across an agent
    // round trip (COP_SC). The commit unit engages it (commit_store) and HOLDS it at
    // the ROB head until the LSQ reports the outcome (sc_commit_done), then retires.
    // Default 0 => single-core: the SC retires like any store, netlist-identical.
    parameter bit COHERENT = 1'b0
)
(
    input wire logic [OOO_WIDTH-1:0] commit_valid,
    input wire commit_packet_t       commit_packet [OOO_WIDTH],
    input wire logic                 store_port_busy,
    // M4-S5b: the LSQ has resolved the coherent SC at the ROB head (agent sc_ok +
    // rd written) -- release its retire. Constant 0 when COHERENT=0.
    input wire logic                 sc_commit_done,
    // The memory subsystem can accept the store's write beat this cycle. A
    // store may only retire when its write can be driven and accepted in the
    // same cycle (the LSQ entry is cleared at commit, so the write cannot be
    // replayed later).
    input wire logic                 store_port_ready,
    output logic [OOO_WIDTH-1:0] retire_valid,
    output logic [OOO_WIDTH-1:0] free_valid,
    output phys_reg_t            free_prd [OOO_WIDTH],
    output logic                 commit_store,
    output active_id_t           commit_store_id,
    output logic                 precise_halt,
    output logic                 precise_exception
);

    logic stop_prefix;

    always_comb begin
        retire_valid = '0;
        free_valid = '0;
        for (int i = 0; i < OOO_WIDTH; i += 1) begin
            free_prd[i] = '0;
        end
        commit_store = 1'b0;
        commit_store_id = '0;
        precise_halt = 1'b0;
        precise_exception = 1'b0;
        stop_prefix = 1'b0;

        for (int i = 0; i < OOO_WIDTH; i += 1) begin
            if (commit_valid[i] && !stop_prefix) begin
                if (COHERENT && commit_packet[i].is_sc) begin
                    // M4-S5b: coherent store-conditional. Engage the LSQ (commit_store
                    // carries the active_id; the LSQ self-gates the COP_SC issue on its
                    // own port readiness) and HOLD at the ROB head until the LSQ reports
                    // the agent's verdict. retire_valid waits for sc_commit_done; rd is
                    // written by the LSQ's resolve writeback that same cycle.
                    if (sc_commit_done) begin
                        retire_valid[i] = 1'b1;
                        free_valid[i] = commit_packet[i].has_dest;
                        free_prd[i] = commit_packet[i].old_prd;
                    end else begin
                        commit_store = 1'b1;
                        commit_store_id = commit_packet[i].active_id;
                        stop_prefix = 1'b1;
                    end
                end else if (commit_packet[i].is_store &&
                        (store_port_busy || !store_port_ready)) begin
                    stop_prefix = 1'b1;
                end else begin
                    retire_valid[i] = 1'b1;
                    free_valid[i] = commit_packet[i].has_dest;
                    free_prd[i] = commit_packet[i].old_prd;

                    if (commit_packet[i].is_store) begin
                        commit_store = 1'b1;
                        commit_store_id = commit_packet[i].active_id;
                    end
                    if (commit_packet[i].halted) begin
                        precise_halt = 1'b1;
                        stop_prefix = 1'b1;
                    end
                    if (commit_packet[i].exception) begin
                        precise_exception = 1'b1;
                        stop_prefix = 1'b1;
                    end
                end
            end
        end
    end

endmodule: ooo_commit_unit
