/**
 * testbench.sv
 *
 * RISC-V 32-bit Processor
 *
 * ECE 18-447
 * Carnegie Mellon University
 *
 * This is the testbench (top module) used for processor simulation.
 *
 * This top module is intended only for simulation, and is not synthesizable. It
 * is responsible for running and managing the riscv_core module. See top.sv for
 * the synthesizable top module.
 *
 * The top module handles connecting the memory simulation model to the
 * processor core, and terminating simulation when requested by the core. It
 * also generates the clock, and keeps track of basic information, such as
 * the cycle count and PC value.
 *
 * Authors:
 *  - 2016 - 2017: Brandon Perez
 **/

/*----------------------------------------------------------------------------*
 *                          DO NOT MODIFY THIS FILE!                          *
 *          You should only add or change files in the src directory!         *
 *----------------------------------------------------------------------------*/

// This module is only included when we are running simulation
`ifdef SIMULATION_18447

// Force the compiler to throw an error if any variables are undeclared
`default_nettype none

// RISC-V Includes
`include "riscv_isa.vh"  // Definition of XLEN
`include "riscv_uarch.vh"  // Definition of main memory parameters
`include "memory_segments.vh"  // Definition of memory segments array

/*----------------------------------------------------------------------------
 * Simulation Top Module
 *----------------------------------------------------------------------------*/

/**
 * The top module for the RISC-V core used for simulation.
 *
 * For simulation, a non-synthesizable, but behaviorally accurate model is
 * used for memory. Additionally, the top module for simulation handles
 * clock generation, resetting the processor, keeping track of cycles, and
 * terminating the simulation when prompted.
 **/
module top;

    // Import the parameters needed to define main memory
    import RISCV_ISA::XLEN, RISCV_ISA::XLEN_BYTES;
    import RISCV_UArch::MEMORY_NUM_PORTS, RISCV_UArch::MEMORY_ADDR_WIDTH;
    import RISCV_UArch::SUPERSCALAR_WAYS;
    import RISCV_UArch::MEMORY_READ_WIDTH;
    import RISCV_UArch::IMEMORY_READ_DELAY;
    import RISCV_UArch::DMEMORY_READ_DELAY;
    import MemorySegments::SEGMENT_WORDS;

    // Import the clock to use for the processor in simulation
    import RISCV_UArch::CLOCK_HALF_PERIOD;

    // Internal variables
    int cycle_count;

    // Processor and memory interface signals
    logic clk, rst_l, instr_mem_excpt_M, instr_mem_excpt_P;
    logic data_mem_excpt_M, data_mem_excpt_P;
    logic data_load_en, halted;
    logic instr_stall, data_stall;
    logic [XLEN_BYTES-1:0] data_store_mask;
    logic [MEMORY_ADDR_WIDTH-1:0] instr_addr, data_addr, data_addr_P;
    logic [XLEN-1:0] data_store, pc;
    logic [MEMORY_READ_WIDTH-1:0][XLEN-1:0] mem_data_load_M, mem_data_load_P;
    logic [MEMORY_READ_WIDTH-1:0][XLEN-1:0] instr_M, instr_P;
    logic data_valid_P;

    // MMU page-table-walk port (core <-> memory)
    logic [MEMORY_ADDR_WIDTH-1:0] ptw_addr;
    logic                         ptw_we;
    logic [XLEN-1:0]              ptw_wdata, ptw_rdata;

    // Handle resetting the processor when simulation begins
    initial begin
        rst_l = 1'b1;
        #1 rst_l = 1'b0;
        #(2 * CLOCK_HALF_PERIOD + 1) rst_l = 1'b1;
    end

    // The global clock for the design
    clock #(.HALF_PERIOD(CLOCK_HALF_PERIOD)) Clock (.clk);

    // The Phase 1 core uses the Lab 4 memory handshake unconditionally.
    riscv_core RISCV_Core (
        .clk            (clk),
        .rst_l          (rst_l),
        .instr_mem_excpt(instr_mem_excpt_P),
        .data_mem_excpt (data_mem_excpt_P),
        .instr          (instr_P),
        .data_load      (mem_data_load_P),
        .data_load_en   (data_load_en),
        .halted         (halted),
        .data_store_mask(data_store_mask),
        .instr_stall    (instr_stall),
        .data_stall     (data_stall),
        .instr_addr     (instr_addr),
        .data_addr      (data_addr),
        .data_store     (data_store),
        .data_load_addr (data_addr_P),
        .data_load_valid(data_valid_P),
        .ptw_addr       (ptw_addr),
        .ptw_we         (ptw_we),
        .ptw_wdata      (ptw_wdata),
        .ptw_rdata      (ptw_rdata)
    );

    // Delay buffer to simulate multi-cycle pipelined memory
    delay_buffer #(
        .DATA_WIDTH(1 + MEMORY_READ_WIDTH * XLEN),
        .DELAY     (IMEMORY_READ_DELAY),
        .RESET_VAL (({1'b0, {MEMORY_READ_WIDTH{32'h13}}}))
    ) InstrDelayBuffer (
        .clk,
        .rst_l,
        .stall   (instr_stall),
        .data_in ({instr_mem_excpt_M, instr_M}),
        .data_out({instr_mem_excpt_P, instr_P})
    );

    delay_buffer #(
        .DATA_WIDTH(MEMORY_ADDR_WIDTH + 2 + MEMORY_READ_WIDTH * XLEN),
        .DELAY     (DMEMORY_READ_DELAY),
        .RESET_VAL (({{MEMORY_ADDR_WIDTH{1'h0}}, 1'b0, 1'b0, {MEMORY_READ_WIDTH{32'h0}}}))
    ) DataDelayBuffer (
        .clk,
        .rst_l,
        .stall   (data_stall),
        .data_in ({data_addr, data_load_en, data_mem_excpt_M, mem_data_load_M}),
        .data_out({data_addr_P, data_valid_P, data_mem_excpt_P, mem_data_load_P})
    );

    // The main memory for the processor
    main_memory #(
        .NUM_PORTS    (MEMORY_NUM_PORTS),
        .LOAD_WORDS   (MEMORY_READ_WIDTH),
        .WORD_BYTES   (XLEN_BYTES),
        .ADDR_WIDTH   (MEMORY_ADDR_WIDTH),
        .SEGMENT_WORDS(SEGMENT_WORDS)
    ) Memory (
        .clk,
        .rst_l,
        .load_ens   ({data_load_en, 1'b1}),
        .store_masks({data_store_mask, 4'b0}),
        .addrs      ({data_addr, instr_addr}),
        .store_data ({data_store, 32'dx}),
        .mem_excpts ({data_mem_excpt_M, instr_mem_excpt_M}),
        .load_data  ({mem_data_load_M, instr_M}),
        .ptw_addr   (ptw_addr),
        .ptw_we     (ptw_we),
        .ptw_wdata  (ptw_wdata),
        .ptw_rdata  (ptw_rdata)
    );

    // Keep a count of the cycles that have passed, and the current PC value
    assign pc = {instr_addr, 2'b00};
    always_ff @(posedge clk) begin
        if (!rst_l) begin
            cycle_count = 0;
        end
        else begin
            cycle_count += 1;
        end
    end

    // Handle terminating simulation whenever halted is asserted
    always @(posedge clk) begin
        #0;  // Allow all other tasks to finish
        if (rst_l && halted) begin
            $finish;
        end
    end

    /* HTIF tohost monitor used by the RISC-V privileged / Sv32 test suites.
     * Enable by passing +tohost=<hex byte address>; a non-zero store to that
     * address ends simulation. By convention a payload of 1 means PASS, and any
     * other value V means FAIL where (V >> 1) is the failing test number. When
     * the plusarg is absent this monitor is inert, preserving the existing
     * ECALL-halt + register-dump flow. */
    logic [31:0] htif_tohost_addr;
    logic        htif_enabled;
    initial begin
        if ($value$plusargs("tohost=%h", htif_tohost_addr)) begin
            htif_enabled = 1'b1;
        end else begin
            htif_tohost_addr = 32'b0;
            htif_enabled = 1'b0;
        end
    end
    always @(posedge clk) begin
        if (rst_l && htif_enabled && (data_store_mask != 4'b0) &&
                (data_addr == htif_tohost_addr[31:2])) begin
            if (data_store == 32'h1) begin
                $display("HTIF: tohost = 1 (PASS)");
            end else begin
                $display("HTIF: tohost = %0d (FAIL test %0d)", data_store,
                    data_store >> 1);
            end
            $finish;
        end
    end

    /* Architectural-signature dumper for RISCOF/arch-test style verification.
     * Enable by passing +sig_begin=<hex byte addr> +sig_end=<hex byte addr>
     * +sig_out=<path>. At the end of simulation the words in [begin, end) are
     * written one 32-bit word per line (lowercase hex) so the result can be
     * diffed directly against the golden reference .sig. Reads the flat backing
     * store in the (DO-NOT-MODIFY) memory model hierarchically; uninitialized
     * words read back as zero, matching the model's own read semantics. */
    logic [31:0] sig_begin_addr, sig_end_addr;
    string       sig_out_path;
    logic        sig_enabled;
    initial begin
        if ($value$plusargs("sig_begin=%h", sig_begin_addr) &&
            $value$plusargs("sig_end=%h", sig_end_addr) &&
            $value$plusargs("sig_out=%s", sig_out_path)) begin
            sig_enabled = 1'b1;
        end else begin
            sig_enabled = 1'b0;
        end
    end
    final begin
        if (sig_enabled) begin
            int unsigned waddr;
            int          fd;
            logic [29:0] idx;
            fd = $fopen(sig_out_path, "w");
            if (fd == 0) begin
                $display("SIG: unable to open %s for writing", sig_out_path);
            end else begin
                for (waddr = sig_begin_addr; waddr < sig_end_addr;
                        waddr += 4) begin
                    idx = waddr[31:2];
                    if (Memory.memory.exists(idx))
                        $fdisplay(fd, "%08x", Memory.memory[idx]);
                    else
                        $fdisplay(fd, "%08x", 32'h0);
                end
                $fclose(fd);
                $display("SIG: wrote signature [%08h,%08h) to %s",
                    sig_begin_addr, sig_end_addr, sig_out_path);
            end
        end
    end

endmodule : top

/*----------------------------------------------------------------------------
 * Clock Module
 *----------------------------------------------------------------------------*/

/**
 * The generator for the global clock used for the processor.
 *
 * This outputs the global clock for the design, and is parameterized by
 * the clock's half period, so the actual period is double that.
 *
 * Parameters:
 *  - HALF_PERIOD   Half of the generated clock's period.
 *
 * Outputs:
 *  - clk           The global clock for the design, with a period of
 *                  2*HALF_PERIOD.
 **/
module clock #(
    parameter HALF_PERIOD = 0
) (
    output logic clk
);

    initial begin
        clk = 1;

        forever #HALF_PERIOD clk = ~clk;
    end

endmodule : clock

/**
 * Delay data_in to data_out by parameterized number of clock edges. 
 * Data_out is 0 during reset.  DELAY=0 means combinational.
 * 
 * Parameters:
 *  - DATA_WIDTH    width of data value 
 *  - DELAY         number of clock edges; 0 means combinational
 *
 * Input:
 *  - data_in       input data
 *  - clk           clock
 *  - rst_l         active low reset
 *
 * Outputs:
 *  - data_out      output data
 **/
module delay_buffer #(
    parameter DATA_WIDTH = 0,
              DELAY      = 0,
              RESET_VAL  = 0
) (
    clk,
    rst_l,
    stall,
    data_in,
    data_out
);

    output [DATA_WIDTH-1:0] data_out;
    reg [DATA_WIDTH-1:0] data_out;

    input [DATA_WIDTH-1:0] data_in;
    input clk, rst_l, stall;
    reg     [DATA_WIDTH-1:0] data_q[DELAY:0];  // only upto data_q[DELAY-1] is used

    integer                  i;

    always @(posedge clk) begin
        if (!rst_l) begin
            for (i = 0; i < DELAY; i = i + 1) begin
                data_q[i] <= RESET_VAL;
            end
        end
        else begin
            if (!stall) begin
                data_q[0] <= data_in;

                for (i = 1; i < DELAY; i = i + 1) begin
                    data_q[i] <= data_q[i-1];
                end
            end
        end
    end  // always@ (posedge clk)                                                                                                                                                                                                      

    always @(*) begin
        if (!rst_l) begin
            data_out = RESET_VAL;
        end
        else if (DELAY == 0) begin
            data_out = data_in;
        end
        else begin
            data_out = data_q[(DELAY!=0)?DELAY-1 : 0];
        end
    end

endmodule


`endif  /* SIMULATION_18447 */
