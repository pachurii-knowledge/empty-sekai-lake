// tb_btb.sv — standalone unit test for src/btb.sv (P2 foundation).
// Drives the sync-read lookup + write-only train ports and checks allocate/hit/
// miss/invalidate and the 1-cycle sync-read timing. Expect "TB-BTB: ALL PASSED".
`timescale 1ns/1ps
`include "riscv_isa.vh"
module tb_btb;
    import RISCV_ISA::XLEN;
    localparam int OFFSET_W = 3, TYPE_W = 2;

    logic clk = 0, rst_l = 0;
    always #5 clk = ~clk;

    logic                lookup_valid, pred_valid;
    logic [XLEN-1:0]     lookup_pc, pred_target;
    logic [OFFSET_W-1:0] pred_offset;
    logic [TYPE_W-1:0]   pred_type;
    logic                train_valid, train_taken;
    logic [XLEN-1:0]     train_pc, train_target;
    logic [OFFSET_W-1:0] train_offset;
    logic [TYPE_W-1:0]   train_type;

    btb #(.SETS(512)) dut (.*);

    int errors = 0;
    task automatic check(string name, logic cond);
        if (!cond) begin $display("  FAIL: %s", name); errors++; end
        else            $display("  ok:   %s", name);
    endtask

    // Issue a lookup on cycle N; the registered result is valid on cycle N+1.
    task automatic do_lookup(logic [XLEN-1:0] pc);
        @(negedge clk); lookup_valid = 1; lookup_pc = pc;
        @(negedge clk); lookup_valid = 0;   // result now registered, sample it
    endtask
    task automatic do_train(logic taken, logic [XLEN-1:0] pc, logic [XLEN-1:0] tgt,
                            logic [OFFSET_W-1:0] off, logic [TYPE_W-1:0] typ);
        @(negedge clk);
        train_valid = 1; train_taken = taken; train_pc = pc;
        train_target = tgt; train_offset = off; train_type = typ;
        @(negedge clk); train_valid = 0;
    endtask

    initial begin
        lookup_valid = 0; lookup_pc = 0;
        train_valid = 0; train_taken = 0; train_pc = 0; train_target = 0;
        train_offset = 0; train_type = 0;
        repeat (3) @(negedge clk); rst_l = 1; @(negedge clk);

        // 1) cold lookup => miss
        do_lookup(64'h8000_0040);
        check("cold lookup misses", pred_valid === 1'b0);

        // 2) allocate a taken entry, then it hits with the right payload
        do_train(1'b1, 64'h8000_0040, 64'h8000_1200, 3'd6, 2'd1);
        do_lookup(64'h8000_0040);
        check("hit after allocate",        pred_valid === 1'b1);
        check("hit target correct",        pred_target === 64'h8000_1200);
        check("hit offset correct",        pred_offset === 3'd6);
        check("hit type correct",          pred_type   === 2'd1);

        // 3) a different tag in the same set => miss (tag check works)
        //    same index (bits [12:4]) but different tag bits (>= bit 16)
        do_lookup(64'h8010_0040);
        check("tag-mismatch same-set miss", pred_valid === 1'b0);

        // 4) invalidate on a not-taken/mis-steer train => now misses
        do_train(1'b0, 64'h8000_0040, 64'h0, 3'd0, 2'd0);
        do_lookup(64'h8000_0040);
        check("miss after invalidate",     pred_valid === 1'b0);

        // 5) re-allocate with a new target => hit with the updated target
        do_train(1'b1, 64'h8000_0040, 64'h8000_9abc, 3'd7, 2'd2);
        do_lookup(64'h8000_0040);
        check("re-alloc hit",              pred_valid  === 1'b1);
        check("re-alloc new target",       pred_target === 64'h8000_9abc);

        // 6) lookup_valid low => pred_valid low regardless of a live entry
        @(negedge clk); lookup_valid = 0; @(negedge clk);
        check("no lookup => no pred",      pred_valid === 1'b0);

        if (errors == 0) $display("TB-BTB: ALL PASSED");
        else             $display("TB-BTB: %0d FAILED", errors);
        $finish;
    end
endmodule
