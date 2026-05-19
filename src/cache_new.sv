/**
 * A set-associative cache module for handling memory requests.
 *
 * This module implements a set-associative cache with a configurable number 
 * of sets and ways. It supports read and write operations, with hit/miss 
 * detection for cache accesses.
 *
 * Parameters:
 *  - INDEX_BITS        Number of bits in the address that form the set index.
 *  - BLOCK_OFFSET_BITS Number of bits in the address that determine the block offset.
 *  - BLOCK_SIZE        Number of words per block (derived from BLOCK_OFFSET_BITS).
 *  - WAYS              The associativity of the cache (number of ways per set).
 *  - WORD_SIZE         The size of a single word in bits.
 *  - ADDRESS_SIZE      The size of the memory address in bits.
 *  - POLICY            The cache eviction policy switch. (0 -> DIRECT, 1 -> LRU, 2 -> MRU, etc.)      
 *
 * Inputs:
 *  - clk               The clock.
 *  - rst_l             Synchronous active-low reset signal.
 *  - address           The memory address for cache operations in the format of [TAG_BITS, INDEX_BITS, BLOCK_OFFSET_BITS]
 *  - enable            Enables cache read, write, or flush operations.
 *  - rd_wr             Determines the operation type: 1 for read, 0 for write.
 *  - flush             Invalidates all ways at the set index specified by address. Flush takes priority over rd_wr.
 *  - write_data_valid  Bit-wise valid signal for each word in the write block.
 *  - write_data        The data to be written to the cache.
 *
 * Outputs:
 *  - read_data         The requested word from the cache if found.
 *  - read_hit          Indicates if the requested word was found in the cache.
 *  - read_miss         Indicates if the requested word was not found in the cache.
 *  - is_eviction       Indicates if a write triggered an eviction.
 *
 *
 * Authors:
 *     - 2025: Varun Rajesh 
 */

 /*---------------------------------------------------------------------------*
 *                          DO NOT MODIFY THIS FILE!                          *
 *          You should only add or change files in the src directory!         *
 *                                                                            *
 *          For part B, if you want to modify the RTL of the cache,           *
 *          (which we encourage!), make a copy of this in your src            *
 *          and CHANGE THE NAME of the module. When we test your              *
 *          submission, we will only copy files in your src directory,        *
 *          and keep all other files (inluding this one, unmodified).         *
 *----------------------------------------------------------------------------*/

module cache_op #(
    parameter INDEX_BITS        = 4,
    parameter BLOCK_OFFSET_BITS = 2,
    parameter BLOCK_SIZE        = 2 ** BLOCK_OFFSET_BITS,

    parameter WAYS = 4,

    parameter WORD_SIZE    = 32,
    parameter ADDRESS_SIZE = 30,

    parameter POLICY = -1
) (
    input logic clk,
    input logic rst_l,

    input logic [ADDRESS_SIZE - 1 : 0] address,
    input logic                        enable,
    input logic                        rd_wr,
    input logic                        flush,

    input logic [BLOCK_SIZE - 1 : 0]                    write_data_valid,
    input logic [BLOCK_SIZE - 1 : 0][WORD_SIZE - 1 : 0] write_data,

    output logic [WORD_SIZE - 1 : 0] read_data,
    output logic                     read_hit,
    output logic                     read_miss,
    output logic                     is_eviction
);

    // --- METADATA PARAMETERS ---
    // TODO: Modify as needed
    localparam METADATA_BITS_PER_WAY = (POLICY == 0) ? 1 : (POLICY == 1) ? $clog2(WAYS) : (POLICY == 2) ? 1 : 1;
    localparam METADATA_BITS = METADATA_BITS_PER_WAY * WAYS;

    initial begin
        if (WAYS != (2 ** $clog2(WAYS))) 
            $fatal(1, "WAYS must be a power of 2");

        if (WAYS <= 0) 
            $fatal(1, "WAYS must be greater than 0");

        if ((POLICY == 0) != (WAYS == 1)) 
            $fatal(1, "POLICY must be DIRECT if and only if WAYS is 1");
    end


    // --- LOCAL PARAMETERS ---
    localparam TAG_BITS = ADDRESS_SIZE - INDEX_BITS - BLOCK_OFFSET_BITS;

    localparam CACHE_DEPTH = 2 ** INDEX_BITS;

    // --- DATA STRUCTURES
    typedef struct packed {
        logic [BLOCK_SIZE - 1 : 0]                   valid;
        logic [TAG_BITS - 1 : 0]                     tag;
        logic [BLOCK_SIZE - 1 : 0][WORD_SIZE- 1 : 0] line;
    } cache_block_t;

    typedef struct packed {
        logic [METADATA_BITS - 1 : 0] metadata;
        cache_block_t [WAYS - 1 : 0]  blocks;
    } cache_set_t;

    typedef struct packed {
        logic [TAG_BITS - 1 : 0]          tag;
        logic [INDEX_BITS - 1 : 0]        index;
        logic [BLOCK_OFFSET_BITS - 1 : 0] block_offset;
    } address_t;


    // --- ADDRESSING ---
    address_t decoded_address;
    assign decoded_address = address;


    // --- CACHE DATA PORTS ---
    cache_set_t                cache_write_data;
    cache_set_t                cache_read_data;


    // --- CACHE SET INFORMATION ---
    logic       [WAYS - 1 : 0] way_hit;
    logic       [WAYS - 1 : 0] way_block_hit;
    logic       [WAYS - 1 : 0] way_valid;
    logic       [WAYS - 1 : 0] way_tag_match;


    // --- CACHE WRITE SIGNALS ---
    logic       [WAYS - 1 : 0] way_write_enable;


    // --- SRAM INSTANCE ---
    sram_1r_1w #(
        .NUM_WORDS (CACHE_DEPTH),
        .WORD_WIDTH($bits(cache_set_t)),
        .RESET_VAL (1'b0)
    ) cache_money (
        .clk       (clk),
        .rst_l     (rst_l),
        .we        (enable),
        .read_addr (decoded_address.index),
        .write_addr(decoded_address.index),
        .write_data(cache_write_data),
        .read_data (cache_read_data)
    );


    // --- CACHE HIT/MISS LOGIC
    always_comb begin
        for (int way = 0; way < WAYS; way++) begin
            way_tag_match[way] = (cache_read_data.blocks[way].tag == decoded_address.tag);
            way_valid[way] = |cache_read_data.blocks[way].valid;

            way_block_hit[way] = cache_read_data.blocks[way].valid[decoded_address.block_offset] && way_tag_match[way];
            way_hit[way] = way_valid[way] && way_tag_match[way];
        end
    end

    // --- METADATA UPDATE ---
    always_comb begin
        if (enable) begin
            // Flush case
            if (flush) begin
                cache_write_data.metadata = '0;
            end
            // Read case
            else if (rd_wr) begin
                // Update metadata if cache hit
                if (|way_block_hit) begin
                    cache_write_data.metadata = calculate_metadata(cache_read_data.metadata, way_block_hit);
                end

                // No change if no hit
                else begin
                    cache_write_data.metadata = cache_read_data.metadata;
                end
            end
            // Write case
            else begin
                // Something always has to get written on a write
                cache_write_data.metadata = calculate_metadata(cache_read_data.metadata, way_write_enable);
            end
        end
        else begin
            // No change if not enabled
            cache_write_data.metadata = cache_read_data.metadata;
        end
    end


    assign is_eviction = (enable && ~rd_wr)  // Enable write
        && (~|way_hit)  // No tag matches that is being used
        && (&way_valid)  // All ways are being used
        && (|cache_read_data.metadata);  // Metadata is populated; should never have an eviction on unpopulated metadata (why?)


    // --- WRITE ENABLE LOGIC ---
    always_comb begin
        if (enable && ~rd_wr) begin
            // If a tag matches and way is being used, use that for the write
            if (|way_hit) begin
                way_write_enable = way_hit;
            end
            // If no tag matches that is being used
            else begin
                // If all ways are being used
                if (&way_valid) begin

                    // This is the only true eviction
                    if (|cache_read_data.metadata) begin
                        way_write_enable = select_eviction_target(cache_read_data.metadata);
                    end

                    // If no metadata set yet, default to first way
                    else begin
                        way_write_enable = WAYS'(1);
                    end
                end

                // If at least one way is not being used
                else begin
                    way_write_enable = 1'b0;

                    // Select first way that is not being used
                    for (int way = 0; way < WAYS; way++) begin
                        if (way_valid[way] == 1'b0) begin
                            way_write_enable[way] = 1'b1;
                            break;
                        end
                    end
                end
            end
        end
        else begin
            way_write_enable = '0;
        end
    end


    // --- WRITE DATA GENERATION ---
    always_comb begin
        for (int way = 0; way < WAYS; way++) begin
            if (flush && enable) begin
                // Eviction case, reset valid bits
                cache_write_data.blocks[way].tag   = 'x;
                cache_write_data.blocks[way].valid = '0;
                cache_write_data.blocks[way].line  = 'x;
            end
            else if (way_write_enable[way]) begin
                if (is_eviction) begin
                    // Nuke cache line and fully replace it with what's being written
                    cache_write_data.blocks[way].tag   = decoded_address.tag;
                    cache_write_data.blocks[way].valid = write_data_valid;
                    cache_write_data.blocks[way].line  = write_data;
                end
                else begin
                    // Selectively write valid words
                    cache_write_data.blocks[way].tag = decoded_address.tag;
                    cache_write_data.blocks[way].valid = cache_read_data.blocks[way].valid | write_data_valid; // Combine valid bits
                    for (int block = 0; block < BLOCK_SIZE; block++) begin
                        if (write_data_valid[block]) begin
                            cache_write_data.blocks[way].line[block] = write_data[block];
                        end
                        else begin
                            cache_write_data.blocks[way].line[block] = cache_read_data.blocks[way].line[block];
                        end
                    end
                end
            end
            else begin
                cache_write_data.blocks[way] = cache_read_data.blocks[way];
            end
        end
    end


    // --- READ WORD MUX ---
    always_comb begin
        read_data = '0;

        for (int way = 0; way < WAYS; way++) begin
            read_data |= way_block_hit[way] ? cache_read_data.blocks[way].line[decoded_address.block_offset] : '0;
        end
    end

    // --- READ SIGNALS --- 
    always_comb begin
        if (enable && rd_wr) begin
            read_hit  = |way_block_hit;
            read_miss = !(|way_block_hit);
        end
        else begin
            read_hit  = 1'b0;
            read_miss = 1'b0;
        end
    end


    // ##################################### CUSTOMIZABLE CACHE POLICY #####################################

    // --- METADATA UPDATE ---
    // TODO: Extend as needed
    // See note above if you want to modify
    // (Do not modify without making a copy in src and renaming the module)

    function automatic logic [METADATA_BITS - 1 : 0] calculate_metadata_direct(
        logic [METADATA_BITS - 1 : 0] current_metadata, logic [WAYS - 1 : 0] update_index);

        // just return 1
        return '1;
    endfunction

    function automatic logic [METADATA_BITS - 1 : 0] calculate_metadata_mru(
        logic [METADATA_BITS - 1 : 0] current_metadata, logic [WAYS - 1 : 0] update_index);

        // Update metadata to match index being updated
        return update_index;
    endfunction

    function automatic logic [METADATA_BITS - 1 : 0] calculate_metadata_lru(
        logic [METADATA_BITS - 1 : 0] current_metadata, logic [WAYS - 1 : 0] update_index);

        logic [METADATA_BITS_PER_WAY - 1 : 0] metadata_array        [WAYS];
        logic [METADATA_BITS_PER_WAY - 1 : 0] metadata_mux_output;
        logic [METADATA_BITS_PER_WAY - 1 : 0] updated_metadata_array[WAYS];
        logic [        METADATA_BITS - 1 : 0] updated_metadata;

        // Unpack metadata for easier handling
        for (int way = 0; way < WAYS; way++) begin
            metadata_array[way] = current_metadata[METADATA_BITS_PER_WAY*way+:METADATA_BITS_PER_WAY];
        end

        // Initialize metadata (sets LRU as last way)
        if (current_metadata == '0) begin
            for (int way = 0; way < WAYS; way++) begin
                updated_metadata_array[way] = way;
            end
        end

        else begin
            // Figure out metadata value of index being updated
            metadata_mux_output = '0;
            for (int way = 0; way < WAYS; way++) begin
                metadata_mux_output |= update_index[way] ? metadata_array[way] : '0;
            end

            // Loop through indices
            // If index is less than the update index, increase the metadata <-> make more LRU
            // If index is greater than the update index, no change
            // if index matches the update index, reset it to 0 <-> makes the least LRU
            for (int way = 0; way < WAYS; way++) begin
                if (metadata_array[way] < metadata_mux_output) begin
                    updated_metadata_array[way] = metadata_array[way] + 1;
                end
                else if (metadata_array[way] > metadata_mux_output) begin
                    updated_metadata_array[way] = metadata_array[way];
                end
                else begin
                    updated_metadata_array[way] = 0;
                end
            end
        end

        // Repack metadata for return
        for (int way = 0; way < WAYS; way++) begin
            updated_metadata[METADATA_BITS_PER_WAY*way+:METADATA_BITS_PER_WAY] = updated_metadata_array[way];
        end

        return updated_metadata;
    endfunction

    function automatic logic [METADATA_BITS - 1 : 0] calculate_metadata(logic [METADATA_BITS - 1 : 0] current_metadata,
                                                                        logic [WAYS - 1 : 0] update_index);
        if (POLICY == 0) begin
            return calculate_metadata_direct(current_metadata, update_index);
        end
        else if (POLICY == 1) begin
            return calculate_metadata_lru(current_metadata, update_index);
        end
        else if (POLICY == 2) begin
            return calculate_metadata_mru(current_metadata, update_index);
        end
        else begin
            $fatal(1, "Unimplemented cache policy %d", POLICY);
            return '0;
        end
    endfunction



    // --- WRITE ENABLE LOGIC ---
    // TODO: Extend as needed

    function automatic logic [WAYS - 1 : 0] select_eviction_target_direct(logic [METADATA_BITS - 1 : 0] current_metadata);

        // just return 1
        return '1;
    endfunction

    function automatic logic [WAYS - 1 : 0] select_eviction_target_mru(logic [METADATA_BITS - 1 : 0] current_metadata);

        // Metadata stores MRU in one-hot, so we can use that as the eviction target
        return current_metadata;
    endfunction

    function automatic logic [WAYS - 1 : 0] select_eviction_target_lru(logic [METADATA_BITS - 1 : 0] current_metadata);
        logic [WAYS - 1 : 0] eviction_target;

        eviction_target = '0;
        for (int way = 0; way < WAYS; way++) begin
            // Find eviction target that is all 1s <-> this is the LRU candidate
            eviction_target[way] = &current_metadata[METADATA_BITS_PER_WAY*way+:METADATA_BITS_PER_WAY];
        end

        return eviction_target;
    endfunction

    function automatic logic [WAYS - 1 : 0] select_eviction_target(logic [METADATA_BITS - 1 : 0] current_metadata);
        if (POLICY == 0) begin
            return select_eviction_target_direct(current_metadata);
        end
        else if (POLICY == 1) begin
            return select_eviction_target_lru(current_metadata);
        end
        else if (POLICY == 2) begin
            return select_eviction_target_mru(current_metadata);
        end
        else begin
            $fatal(1, "Unimplemented eviction policy %s", POLICY);
            return '0;
        end
    endfunction

    // ################################################## CUSTOMIZABLE CACHE POLICY #####################################

endmodule : cache_op
