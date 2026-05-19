/**
 * sram.sv
 *
 * RISC-V 32-bit Processor
 *
 * ECE 18-447
 * Carnegie Mellon University
 *
 * This is an SRAM module that can be used by the main processor.
 *
 * Authors:
 *  - 2017: James Hoe
 *  - 2017: Brandon Perez
 *  - 2025: Dane Engman and Varun Rajesh
 **/

/*----------------------------------------------------------------------------*
 *                          DO NOT MODIFY THIS FILE!                          *
 *          You should only add or change files in the src directory!         *
 *----------------------------------------------------------------------------*/

// RISC-V Includes
`include "riscv_uarch.vh"  // Definition of BTB default parameters

// Force the compiler to throw an error if any variables are undeclared
`default_nettype none

`ifdef SIMULATION_18447

/*----------------------------------------------------------------------------
 * SRAM Modules
 *----------------------------------------------------------------------------*/

/**
 * A parameterized static random access memory (SRAM) used by the processor.
 * 2 ports - 1 independent read and 1 independent write port
 *
 * This is a synchronous write, combinational (asynchronous) read SRAM. The
 * SRAM has a single read port and a single write port. Writes do not appear
 * in memory until the next cycle. The SRAM is parameterized by the size of
 * its words, the number of words, and the value it takes on reset.
 *
 * Parameters:
 *  - NUM_WORDS     The number of words present in the SRAM memory.
 *  - WORD_WIDTH    The number of bits that each word in memory has.
 *  - RESET_VAL     The value that all memory locations hold after a reset.
 *
 * Inputs:
 *  - clk           The clock to use for the SRAM.
 *  - rst_l         The asynchronous active-low reset for the SRAM.
 *  - we            Indicates that write_data data should be written to the
 *                  write_addr address in the SRAM.
 *  - read_addr     The address from which to read the value.
 *  - write_addr    The address to which to write write_data.
 *  - write_data    The data to write to the write_addr address.
 *
 * Outputs:
 *  - read_data     The data at the read_addr address in memory.
 **/

module sram_1r_1w #(
    parameter                        NUM_WORDS  = RISCV_UArch::BTB_NUM_WORDS,
    parameter                        WORD_WIDTH = RISCV_UArch::BTB_WORD_WIDTH,
    parameter logic [WORD_WIDTH-1:0] RESET_VAL  = 'b0
) (
    input  logic                         clk,
    input  logic                         rst_l,
    input  logic                         we,
    input  logic [$clog2(NUM_WORDS)-1:0] read_addr,
    input  logic [$clog2(NUM_WORDS)-1:0] write_addr,
    input  logic [       WORD_WIDTH-1:0] write_data,
    output logic [       WORD_WIDTH-1:0] read_data
);

    // The memory for the SRAM
    logic [WORD_WIDTH-1:0] memory[NUM_WORDS-1:0];

    // Handle initialization and writing to the memory
    always_ff @(posedge clk, negedge rst_l) begin
        if (!rst_l) begin
            memory <= '{default: RESET_VAL};
        end
        else if (we) begin
            memory[write_addr] <= write_data;
        end
    end

    // Handle reading from memory
    assign read_data = memory[read_addr];

endmodule : sram_1r_1w


/**
 * A parameterized static random access memory (SRAM) used by the processor.
 * 1 port - shared read and write access with separate data buses
 *
 * This is a synchronous write, combinational (asynchronous) read SRAM with a 
 * single read/write port. Writes do not appear in memory until 
 * the next cycle. The SRAM is parameterized by the size of its words, the 
 * number of words, and the value it takes on reset.
 *
 * Parameters:
 *  - NUM_WORDS     The number of words present in the SRAM memory.
 *  - WORD_WIDTH    The number of bits that each word in memory has.
 *  - RESET_VAL     The value that all memory locations hold after a reset.
 *
 * Inputs:
 *  - clk           The clock to use for the SRAM.
 *  - rst_l         The asynchronous active-low reset for the SRAM.
 *  - we            Indicates that `write_data` should be written to `addr` in memory.
 *  - addr          The address for both read and write operations.
 *  - write_data    The data to be written to memory when `we` is high.
 *
 * Outputs:
 *  - read_data     The data at `addr` in memory. Invalid when `we` is active.
 **/


module sram_1rw #(
    parameter                        NUM_WORDS  = RISCV_UArch::BTB_NUM_WORDS,
    parameter                        WORD_WIDTH = RISCV_UArch::BTB_WORD_WIDTH,
    parameter logic [WORD_WIDTH-1:0] RESET_VAL  = 'b0
) (
    input  logic                         clk,
    input  logic                         rst_l,
    input  logic                         we,
    input  logic [$clog2(NUM_WORDS)-1:0] addr,
    input  logic [       WORD_WIDTH-1:0] write_data,
    output logic [       WORD_WIDTH-1:0] read_data
);

    // The memory for the SRAM
    logic [WORD_WIDTH-1:0] memory[NUM_WORDS-1:0];

    // Handle initialization and writing to the memory
    always_ff @(posedge clk, negedge rst_l) begin
        if (!rst_l) begin
            memory <= '{default: RESET_VAL};
        end
        else if (we) begin
            memory[addr] <= write_data;
        end
    end

    assign read_data = (we) ? 'x : memory[addr];

endmodule : sram_1rw


/**
 * A parameterized static random access memory (SRAM) used by the processor.
 * 2 ports - 1 read/write port and 1 independent read-only port
 *
 * This is a synchronous write, combinational (asynchronous) read SRAM with 
 * two access ports:
 *  - Port A: A read/write port using a `write_data_a` and `read_data_a`
 *  - Port B: A read-only port that continuously outputs data at `read_data_b`.
 *
 * Writes occur only on Port A when `we` is asserted. The SRAM is parameterized
 * by the size of its words, the number of words, and the value it takes on reset.
 *
 * Parameters:
 *  - NUM_WORDS     The number of words present in the SRAM memory.
 *  - WORD_WIDTH    The number of bits that each word in memory has.
 *  - RESET_VAL     The value that all memory locations hold after a reset.
 *
 * Inputs:
 *  - clk           The clock to use for the SRAM.
 *  - rst_l         The asynchronous active-low reset for the SRAM.
 *  - we            Indicates that `write_data_a` should be written to `addr_a`.
 *  - addr_a        The address for read/write operations on Port A.
 *  - addr_b        The address for read-only access on Port B.
 *
 * Inputs:
 *  - write_data_a  The data to be written to the memory at `addr_a` on Port A.
 *
 * Outputs:
 *  - read_data_a   The data at `addr_a` in memory. Invalid when `we` is active.
 *  - read_data_b   The data at `addr_b` in memory (read-only).
 **/


module sram_1rw_1r #(
    parameter                        NUM_WORDS  = RISCV_UArch::BTB_NUM_WORDS,
    parameter                        WORD_WIDTH = RISCV_UArch::BTB_WORD_WIDTH,
    parameter logic [WORD_WIDTH-1:0] RESET_VAL  = 'b0
) (
    input  logic                         clk,
    input  logic                         rst_l,
    input  logic                         we,
    input  logic [$clog2(NUM_WORDS)-1:0] addr_a,
    input  logic [$clog2(NUM_WORDS)-1:0] addr_b,
    input  logic [       WORD_WIDTH-1:0] write_data_a,
    output logic [       WORD_WIDTH-1:0] read_data_a,
    output logic [       WORD_WIDTH-1:0] read_data_b
);

    logic [WORD_WIDTH-1:0] memory[NUM_WORDS-1:0];

    always_ff @(posedge clk, negedge rst_l) begin
        if (!rst_l) begin
            memory <= '{default: RESET_VAL};
        end
        else if (we) begin
            memory[addr_a] <= write_data_a;
        end
    end

    assign read_data_a = (we) ? 'x : memory[addr_a];
    assign read_data_b = memory[addr_b];

endmodule : sram_1rw_1r



/**
 * A parameterized static random access memory (SRAM) used by the processor.
 * 2 port - 2x shared read and write access
 *
 * This is a synchronous write, combinational (asynchronous) read SRAM with two 
 * independent read/write ports. Writes do not appear in memory until 
 * the next cycle. The SRAM is parameterized by the size of its words, the 
 * number of words, and the value it takes on reset. Note that in the case of a
 * write collision, port A takes precedence.
 *
 * Parameters:
 *  - NUM_WORDS     The number of words present in the SRAM memory.
 *  - WORD_WIDTH    The number of bits that each word in memory has.
 *  - RESET_VAL     The value that all memory locations hold after a reset.
 *
 * Inputs:
 *  - clk           The clock to use for the SRAM.
 *  - rst_l         The asynchronous active-low reset for the SRAM.
 *  - we_a          Indicates that `write_data_a` should be written to `addr_a` in memory.
 *  - we_b          Indicates that `write_data_b` should be written to `addr_b` in memory.
 *  - addr_a        The address for both read and write operations on Port A.
 *  - addr_b        The address for both read and write operations on Port B.
 *  - write_data_a  The data to be written to memory at `addr_a` when `we_a` is high.
 *  - write_data_b  The data to be written to memory at `addr_b` when `we_b` is high.
 *
 * Outputs:
 *  - read_data_a   The data read from memory at `addr_a` when `we_a` is low. Invalid when `we_a` is active.
 *  - read_data_b   The data read from memory at `addr_b` when `we_b` is low. Invalid when `we_b` is active.
 **/


module sram_2rw #(
    parameter                        NUM_WORDS  = RISCV_UArch::BTB_NUM_WORDS,
    parameter                        WORD_WIDTH = RISCV_UArch::BTB_WORD_WIDTH,
    parameter logic [WORD_WIDTH-1:0] RESET_VAL  = 'b0
) (
    input  logic                         clk,
    input  logic                         rst_l,
    input  logic                         we_a,
    input  logic                         we_b,
    input  logic [$clog2(NUM_WORDS)-1:0] addr_a,
    input  logic [$clog2(NUM_WORDS)-1:0] addr_b,
    input  logic [       WORD_WIDTH-1:0] write_data_a,
    input  logic [       WORD_WIDTH-1:0] write_data_b,
    output logic [       WORD_WIDTH-1:0] read_data_a,
    output logic [       WORD_WIDTH-1:0] read_data_b
);

    // The memory for the SRAM
    logic [WORD_WIDTH-1:0] memory[NUM_WORDS-1:0];

    // Handle initialization and writing to the memory
    always_ff @(posedge clk, negedge rst_l) begin
        if (!rst_l) begin
            memory <= '{default: RESET_VAL};
        end
        else begin
            if (we_a) begin
                memory[addr_a] <= write_data_a;
            end
            if (we_b && !(we_a && (addr_a == addr_b))) begin
                memory[addr_b] <= write_data_b;
            end
        end
    end

    assign read_data_a = (we_a) ? 'x : memory[addr_a];
    assign read_data_b = (we_b) ? 'x : memory[addr_b];

endmodule : sram_2rw


`endif


