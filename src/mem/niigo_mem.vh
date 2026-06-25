/**
 * niigo_mem.vh
 *
 * Shared definitions for the niigo memory subsystem (phase C1+): the NMI
 * line-granular internal bus (the future L2 / coherence seam) and the L1
 * cache geometry. See plans/fpga-memsys.md §4 and §6.
 *
 * NOTE on addressing: the NMI carries WORD addresses (byte PA >>
 * log2(XLEN_BYTES)), matching the core's memory ports and main_memory. The
 * AXI bridge (phase X1) converts to a byte address at the boundary. The plan
 * text describes byte PAs; this is the deliberate implementation choice that
 * keeps the whole memsys word-addressed end to end.
 */

`ifndef NIIGO_MEM_VH_
`define NIIGO_MEM_VH_

`include "riscv_isa.vh"
`include "riscv_uarch.vh"

package NIIGO_Mem;

    import RISCV_ISA::XLEN, RISCV_ISA::XLEN_BYTES;
    import RISCV_UArch::MEMORY_READ_WIDTH, RISCV_UArch::MEMORY_ADDR_WIDTH;

    // ---- Line geometry (DD-1: one 64 B line == one 512 b AXI beat) ----
    localparam int LINE_BYTES     = 64;
    localparam int LINE_BITS      = LINE_BYTES * 8;            // 512
    localparam int MEM_ADDR_SHIFT = $clog2(XLEN_BYTES);        // 2 (RV32) / 3 (RV64)
    localparam int LINE_WORDS     = LINE_BYTES / XLEN_BYTES;   // 16 / 8
    localparam int LINE_WORD_BITS = $clog2(LINE_WORDS);        // 4 / 3
    // Number of MEMORY_READ_WIDTH-wide read beats to cover one line on the
    // sim adapter (4 at RV32, 2 at RV64).
    localparam int LINE_RD_BEATS  = LINE_WORDS / MEMORY_READ_WIDTH;

    // ---- L1 cache geometry (L1I/L1D share; 16 KiB, 4-way, 64 B lines) ----
    localparam int L1_SETS       = 64;
    localparam int L1_WAYS       = 4;
    localparam int L1_INDEX_BITS = $clog2(L1_SETS);            // 6
    localparam int L1_WAY_BITS   = $clog2(L1_WAYS);            // 2
    localparam int L1_TAG_BITS   = MEMORY_ADDR_WIDTH - L1_INDEX_BITS - LINE_WORD_BITS;

    // ---- VIPT alias-free invariant (M2; plans/multicore-ccd.md §V "DD-CCD-VIPT") ----
    // The L1s are virtually-indexed, physically-tagged. For VIPT to be SYNONYM-FREE the index
    // must lie entirely within the page offset, i.e. way_size <= page_size. Then for every VA
    // that maps a PA under 4 KiB paging, VA[index] == PA[index]: a core's (VA-sourced) index and
    // a coherence snoop's (PA-sourced) index select the SAME set, with no synonym search. This is
    // the single load-bearing invariant of the whole CCD L1 — `L1_VIPT_ALIAS_FREE` makes it a
    // machine-checked property (the caches `initial assert` it), so growing L1_SETS/LINE_BYTES
    // past a page can never silently break VIPT or coherence. way_size = SETS*LINE (total/ways).
    localparam int PAGE_BYTES         = 4096;                  // Sv32/Sv39 base page
    localparam int L1_WAY_BYTES       = L1_SETS * LINE_BYTES;  // = 4096 (== one page, exactly)
    localparam bit L1_VIPT_ALIAS_FREE = (L1_WAY_BYTES <= PAGE_BYTES);

    // Word-address field extraction (a word address is MEMORY_ADDR_WIDTH bits):
    //   [ tag | index | line_word_offset ]
    function automatic logic [L1_INDEX_BITS-1:0]
            l1_index(input logic [MEMORY_ADDR_WIDTH-1:0] wa);
        l1_index = wa[LINE_WORD_BITS +: L1_INDEX_BITS];
    endfunction
    function automatic logic [L1_TAG_BITS-1:0]
            l1_tag(input logic [MEMORY_ADDR_WIDTH-1:0] wa);
        l1_tag = wa[MEMORY_ADDR_WIDTH-1 : L1_INDEX_BITS + LINE_WORD_BITS];
    endfunction
    function automatic logic [LINE_WORD_BITS-1:0]
            l1_word_off(input logic [MEMORY_ADDR_WIDTH-1:0] wa);
        l1_word_off = wa[LINE_WORD_BITS-1:0];
    endfunction
    // Line-base word address (low LINE_WORD_BITS cleared).
    function automatic logic [MEMORY_ADDR_WIDTH-1:0]
            l1_line_base(input logic [MEMORY_ADDR_WIDTH-1:0] wa);
        l1_line_base = {wa[MEMORY_ADDR_WIDTH-1 : LINE_WORD_BITS],
                        {LINE_WORD_BITS{1'b0}}};
    endfunction

    // ---- AXI4 geometry (phase X1): one 64 B line == one 512 b beat ----
    localparam int AXI_ADDR_W = 64;
    localparam int AXI_DATA_W = LINE_BITS;       // 512
    localparam int AXI_ID_W   = 4;
    localparam int AXI_STRB_W = AXI_DATA_W / 8;  // 64
    localparam logic [2:0] AXI_SIZE_LINE = 3'd6; // 2^6 = 64 bytes
    localparam logic [1:0] AXI_BURST_INCR = 2'b01;

    // ---- NMI op + id encodings (plan §4) ----
    typedef enum logic [2:0] {
        NMI_RD_LINE  = 3'd0,
        NMI_WR_LINE  = 3'd1,
        NMI_RD_WORDS = 3'd2,
        NMI_WR_WORD  = 3'd3,
        NMI_PROBE    = 3'd4   // reserved for a future multicore snoop channel
    } nmi_op_e;

    localparam logic [1:0] NMI_SRC_IFILL  = 2'd0;
    localparam logic [1:0] NMI_SRC_DFILL  = 2'd1;
    localparam logic [1:0] NMI_SRC_DWB    = 2'd2;
    localparam logic [1:0] NMI_SRC_LEGACY = 2'd3;

    // Request (valid/ready handshake). Word-addressed (see header note).
    typedef struct packed {
        logic                          valid;
        nmi_op_e                       op;
        logic [MEMORY_ADDR_WIDTH-1:0]  waddr;   // line ops: line-aligned word addr
        logic [3:0]                    id;      // {src[1:0], gen[1:0]}
        logic [LINE_BITS-1:0]          wdata;   // WR_LINE full line / WR_WORD word in lane 0
        logic [XLEN_BYTES-1:0]         wmask;   // WR_WORD byte strobes
    } nmi_req_t;

    // Response (valid only; requester always sinks, <=1 outstanding per id).
    typedef struct packed {
        logic                  valid;
        logic [3:0]            id;
        logic [LINE_BITS-1:0]  rdata;   // RD_LINE full line / RD_WORDS words in low lanes
        logic                  err;
    } nmi_resp_t;

    // ---- Device decode (shared helper; byte PA). Cacheable == NOT a device.
    // CLINT [0x0200_0000,+64K) PLIC [0x0C00_0000,+64M) UART [0x0D00_0000,+4K).
    function automatic logic is_device_pa(input logic [XLEN-1:0] pa);
        logic clint, plic, uart;
        clint = (pa >= XLEN'('h0200_0000)) && (pa < XLEN'('h0201_0000));
        plic  = (pa >= XLEN'('h0C00_0000)) && (pa < XLEN'('h1000_0000));
        uart  = (pa >= XLEN'('h0D00_0000)) && (pa < XLEN'('h0D00_1000));
        is_device_pa = clint || plic || uart;
    endfunction

endpackage : NIIGO_Mem

`endif /* NIIGO_MEM_VH_ */
