/**
 * main_memory.sv
 *
 * RISC-V 32-bit Processor
 *
 * ECE 18-447
 * Carnegie Mellon University
 *
 * This is the main memory used by the processor.
 *
 * The processor's main memory is a slightly non-standard DRAM. The memory a
 * synchronous write, combination (asynchronous) read DRAM. By default, the
 * memory is dual-ported, with each port supporting simultaneous reads and
 * writes to the same address. Also, the memory is word-addressable, so by
 * default the values read out of memory are always 4-bytes. However,
 * there is a write mask that allows users to selectively update certain bytes
 * of a memory word.
 *
 * The main memory is a segmented memory, and as such is divided into 5 segments
 * in total, by default. There are text and data segments for both user and
 * kernel code, and a shared stack segment. The module gets the data for each
 * section from the corresponding binary files that contain the data in the
 * section.
 *
 * The memory module is not synthesizable, and is intended to only be used for
 * simulation.
 *
 * A note about the segment_t structure. The memory array is nested inside a
 * structure field to deal with a not yet implemented error when compiling with
 * VCS. If a structure contains an unpacked array field, then it cannot be
 * passed as a reference into a function when it is selected from an array
 * (e.g. array[i]). This causes VCS to throw en error during compilation.
 *
 * Authors:
 *  - 2016 - 2017: Brandon Perez
 **/

/*----------------------------------------------------------------------------*
 *                          DO NOT MODIFY THIS FILE!                          *
 *          You should only add or change files in the src directory!         *
 *----------------------------------------------------------------------------*/

// This module is only included when we are running simulation.
`ifdef SIMULATION_18447

// RISC-V Includes
`include "riscv_isa.vh"             // Definition of BYTE_WIDTH
`include "memory_segments.vh"       // Definition of memory segment types

// Force the compiler to throw an error if any variables are undeclared
`default_nettype none

/*----------------------------------------------------------------------------
 * Main Memory Module
 *----------------------------------------------------------------------------*/

/**
 * A behaviorally correct, but non-synthesizable model for main memory.
 *
 * This is a synchronous write, combinational (asynchronous) read DRAM. The
 * memory is dual-ported, with each port supporting simultaneous reads and
 * writes to the same address. Writes do no appear in memory until the next
 * cycle, so a simultaneous read will not see the same value. If there is a
 * write conflict between ports, the highest numbered port will be the one to
 * update the memory location. The memory also allows for multi-word reads on a
 * memory load, but only supports updating a single word in memory at a time.
 *
 * This module is parameterized by the number of ports it has, and the size of
 * the words in memory in bytes. The memory is word-addressable, so the width
 * of the addresses is determined by the number of bytes in each memory word.
 * The memory is also parameterized by the number of consecutive words that are
 * read on a load. There are no alignment restrictions on the addresses.
 *
 * Parameters:
 *  - NUM_PORTS     The number of ports that the memory has.
 *  - LOAD_WORDS    The number of consecutive memory words read on a load.
 *  - WORD_BYTES    The number of bytes in a memory word.
 *  - ADDR_WIDTH    The number of bits used for memory addresses.
 *  - SEGMENT_WORDS The size of the memory segments in number of words.
 *  - SEGMENTS      A list of the parameters for all the segments into which the
 *                  address space is divided.
 *
 * Inputs:
 *  - clk           The clock to use for the main memory.
 *  - rst_l         The asynchronous active-low reset for the main memory.
 *  - store_masks   Byte-enable bit mask signal indicating which bytes of store_data should
 *                  be written to the addrs address in memory for each port.
 *  - addrs         The address to read from and/or write to for each port.
 *  - store_data    The data to store at the given address for each port.
 *
 * Outputs:
 *  - mem_excpts    Signal indicating that an invalid memory address was
 *                  specified, for each port. This signal is only asserted when
 *                  a memory load or store occurs.
 *  - load_data     The data at the given address in memory for each port.
 **/
module main_memory
    #(parameter                 NUM_PORTS=0, LOAD_WORDS=0, WORD_BYTES=0,
      parameter                 ADDR_WIDTH=0, SEGMENT_WORDS=0,
      localparam                BYTE_WIDTH = RISCV_ISA::BYTE_WIDTH,
      localparam                WORD_WIDTH = WORD_BYTES * BYTE_WIDTH)
    (input  logic                                                   clk, rst_l,
     input  logic [NUM_PORTS-1:0]                                   load_ens,
     input  logic [NUM_PORTS-1:0][WORD_BYTES-1:0]                   store_masks,
     input  logic [NUM_PORTS-1:0][ADDR_WIDTH-1:0]                   addrs,
     input  logic [NUM_PORTS-1:0][WORD_WIDTH-1:0]                   store_data,
     output logic [NUM_PORTS-1:0]                                   mem_excpts,
     output logic [NUM_PORTS-1:0][LOAD_WORDS-1:0][WORD_WIDTH-1:0]   load_data);

    /*------------------------------------------------------------------------
     * Definitions
     *------------------------------------------------------------------------*/

    // The parameter to $fseek that indicates to seek from the end of file
    localparam SEEK_END                             = 2;

    // The prefix for the segment data files to which extensions are appended
    localparam string SEGMENT_FILE_PREFIX           = "mem";

    // The number of segments
    localparam NUM_SEGMENTS                         = MemorySegments::NUM_SEGMENTS;

    // The amount to shift addresses, or the alignment of memory addresses
    localparam ADDR_SHIFT                           = $clog2(WORD_BYTES);

    // Convenient typedefs for various types used by the memory system
    typedef logic  [ADDR_WIDTH-1:0]                 addr_t;
    typedef logic  [$clog2(NUM_SEGMENTS)-1:0]       index_t;
    typedef logic  [WORD_BYTES-1:0]                 store_mask_t;
    typedef logic  [WORD_BYTES-1:0][BYTE_WIDTH-1:0] word_t;
    typedef word_t                                  memory_t[SEGMENT_WORDS];

    // The representation of a memory segment
    typedef struct {
        addr_t base_addr;       // Start of the segment in memory
        struct {
            memory_t memory;    // The actual physical memory for the segment
        } m;
    } segment_t;
    typedef segment_t           segments_t[NUM_SEGMENTS];

    /* The representation of a segment index, used to determine which segment to
     * use for an operation on a given address. */
    typedef struct {
        logic valid;            // Indicates this is a valid index
        index_t index;          // Index of the segment in the array
    } segment_index_t;

    /* The physical memory, indexed by absolute word address, along with the
     * segment index for each memory port's address. */
    word_t                      memory[addr_t];
    segment_index_t             segment_indices[NUM_PORTS];

    /*------------------------------------------------------------------------
     * Core Logic
     *------------------------------------------------------------------------*/

    // Handle finding the corresponding segment for each port's address
    always_comb begin
        segment_index_loop: foreach (segment_indices[i]) begin
            segment_indices[i] = find_segment(addrs[i], load_ens[i],
                store_masks[i]);
        end
    end

    // Handle initialization of the memory segments, and writing to memory
    always_ff @(posedge clk, negedge rst_l) begin
        if (!rst_l) begin
            initialize_memory();
        end else begin
            data_store_loop: foreach (store_data[i]) begin
                if (segment_indices[i].valid) begin
                    segment_write(addrs[i], store_data[i], store_masks[i],
                            segment_indices[i].index);
                end
            end
        end
    end

    // Handle reading from memory
    always_comb begin
        load_data = 'bx;
        data_load_loop: foreach (load_data[i,j]) begin
            if (segment_indices[i].valid) begin
                load_data[i][j] = segment_read(addrs[i] + j,
                        segment_indices[i].index);
            end
        end
    end

    // Handle generating memory exceptions if the address is invalid
    always_comb begin
        memory_exception_loop: foreach (mem_excpts[i]) begin
            mem_excpts[i] = rst_l & ((store_masks[i] != 'b0) | load_ens[i]) &
                ~segment_indices[i].valid;
        end
    end

    /*------------------------------------------------------------------------
     * Memory Helper Functions
     *------------------------------------------------------------------------*/

    /* Finds the segment corresponding to the given address, if one exists,
     * returning the index structure of this segment in the array. */
    function segment_index_t find_segment(addr_t addr, logic load_en,
            store_mask_t store_mask);

        // Only search for a segment when a load or store operation is happening
        if (!load_en && store_mask == 4'b0) begin
            return '{valid: 1'b0, index: 'bx};
        end

        // Iterate over each segment, and check if the address lies within it
        for (int i = 0; i < NUM_SEGMENTS; i += 1) begin
            addr_t start_addr, end_addr;
            start_addr = addr_t'(segment_base_addr(index_t'(i)) >> ADDR_SHIFT);
            end_addr = start_addr + SEGMENT_WORDS;

            if (start_addr <= addr && addr < end_addr) begin
                return '{valid: 1'b1, index: i};
            end
        end

        return '{valid: 1'b0, index: 'bx};
    endfunction: find_segment

    /* Computes the offset into the given segment of the address. Assumes that
     * the address is present in the segment. */
    function addr_t segment_addr(addr_t addr, index_t index);

        return addr - addr_t'(segment_base_addr(index) >> ADDR_SHIFT);
    endfunction: segment_addr

    /* Writes the value to the given memory segment in little-endian order.
     * Note that the segments array is not passed in, because non-blocking
     * assignments can only occur to module-level variables. */
    function void segment_write(addr_t addr, word_t data,
            store_mask_t store_mask, index_t index);

        addr_t offset;

        // If the write mask is not asserted, then nothing needs to be done
        if (store_mask == 'b0) begin
            return;
        end

        // Iterate over the bytes of the memory, updating as per the write mask
        offset = segment_addr(addr, index);
        foreach (store_mask[i]) begin
            if (store_mask[i]) begin
                if (!memory.exists(addr)) begin
                    memory[addr] = '{default: 'hde};
                end
                memory[addr][i] = data[i];
            end
        end

        return;
    endfunction: segment_write

    // Reads the value out from the given memory segment in little-endian order.
    function word_t segment_read(addr_t addr, index_t index);

        if (memory.exists(addr)) begin
            return memory[addr];
        end
        return '{default: 'hde};
    endfunction: segment_read

    /*------------------------------------------------------------------------
     * Initialization Helper Functions
     *------------------------------------------------------------------------*/

    // Initializes all the segments in memory, loads the data from file
    function void initialize_memory();

        memory.delete();
        initialize_segment(0);
        initialize_segment(1);
        initialize_segment(2);
        initialize_segment(3);
        initialize_segment(4);
        return;
    endfunction: initialize_memory

    /* Initializes a segment of memory, and loading it with the values from
     * the corresponding data file. */
    function void initialize_segment(index_t index);

        string segment_path;
        int segment_fd, file_size;

        // If the segment doesn't have a data file, we don't need to do anything.
        if (segment_extension(index).len() == 0) begin
            return;
        end

        // Attempt to open the binary file for this segment
        segment_path = {SEGMENT_FILE_PREFIX, segment_extension(index)};
        segment_fd = $fopen(segment_path, "rb");
        if (segment_fd == 0) begin
            file_error(segment_fd, segment_path, "Unable to open file");
        end

        // Determine the size of the memory segment data file
        assert($fseek(segment_fd, 0, SEEK_END) == 0) else $fatal;
        file_size = $ftell(segment_fd);
        assert($rewind(segment_fd) == 0) else $fatal;

        // Load the memory segment if the file size does not exceed its size
        if (file_size > MemorySegments::SEGMENT_SIZE) begin
            $fatal("Error: %s: File is too large for memory segment.",
                    segment_path);
        end

        load_mem_segment(segment_fd, segment_path, index);
        $fclose(segment_fd);
        return;
    endfunction: initialize_segment

    function longint segment_base_addr(index_t index);
        case (index)
            0: return MemorySegments::USER_TEXT_START;
            1: return MemorySegments::USER_DATA_START;
            2: return MemorySegments::STACK_START;
            3: return MemorySegments::KERNEL_TEXT_START;
            4: return MemorySegments::KERNEL_DATA_START;
            default: return '0;
        endcase
    endfunction: segment_base_addr

    function string segment_extension(index_t index);
        case (index)
            0: return ".text.bin";
            1: return ".data.bin";
            2: return "";
            3: return ".ktext.bin";
            4: return ".kdata.bin";
            default: return "";
        endcase
    endfunction: segment_extension

    function void load_mem_segment(int segment_fd, string segment_path,
            index_t index);

        int word_index, byte_index;
        addr_t base_addr;

        word_index = 0;
        byte_index = 0;
        base_addr = addr_t'(segment_base_addr(index) >> ADDR_SHIFT);

        /* Since $fread always reads data in big-endian order, load the segment
         * one byte at a time to ensure little-endian ordering. */
        while (1) begin
            string placeholder;
            logic [BYTE_WIDTH-1:0] data_byte;
            int bytes_read;

            // Read the next byte from the file
            bytes_read = $fread(data_byte, segment_fd);
            if (bytes_read == 0 && $ferror(segment_fd, placeholder) != 0) begin
                file_error(segment_fd, segment_path, "Unable to read file");
            end else if (bytes_read == 0) begin
                break;
            end

            // Write the byte to the segment, and advance the indices as needed
            memory[base_addr + word_index][byte_index] = data_byte;
            byte_index += 1;
            if (byte_index == WORD_BYTES) begin
                byte_index = 0;
                word_index += 1;
            end
        end

        return;
    endfunction: load_mem_segment

    // Reports an error after a file I/O operation fails, and exits
    function void file_error(int fd, string file_path, string msg);
        string error_cause;
        assert($ferror(fd, error_cause) != 0) else $fatal;

        $fatal("Error: %s: %s: %s.\n", file_path, msg, error_cause);
    endfunction: file_error

endmodule: main_memory

`endif /* SIMULATION_18447 */
