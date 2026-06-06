`include "ooo_types.vh"
`include "riscv_priv.vh"

`default_nettype none

module load_store_queue
    import OOO_Types::*;
(
    input  logic                 clk,
    input  logic                 rst_l,
    input  logic [OOO_WIDTH-1:0] insert_valid,
    input  issue_entry_t         insert_entry [OOO_WIDTH],
    input  logic [OOO_WIDTH-1:0][31:0] insert_rs1_data,
    input  logic [OOO_WIDTH-1:0][31:0] insert_rs2_data,
    input  logic [OOO_WIDTH-1:0] wakeup_valid,
    input  phys_reg_t            wakeup_prd [OOO_WIDTH],
    input  logic [OOO_WIDTH-1:0][31:0] wakeup_data,
    input  branch_mask_t         reset_mask,
    input  branch_mask_t         abort_mask,
    // Full pipeline flush on a precise trap / interrupt / trap-return: discard
    // every queued memory op (all are younger than the trapping instruction).
    input  logic                 flush,
    input  logic                 data_load_valid,
    input  logic [31:0]          data_load,
    input  logic [29:0]          data_load_addr,
    input  logic                 commit_store,
    input  active_id_t           commit_store_id,
    // Sv32 data-side translation (driven by the core's MMU). When paging_data is
    // low the queue behaves exactly as before (identity mapping). When high, the
    // head's virtual address is exposed for the DTLB lookup, and the core feeds
    // back whether the translation is still walking (xlate_stall) or faulted.
    input  logic                 paging_data,
    input  logic                 xlate_stall,
    input  logic                 xlate_fault,
    input  logic [4:0]           xlate_cause,
    // Resolved physical address for the current mem_req_vaddr (driven by the
    // core MMU). Captured during the high-word probe of a cross-word store so
    // the second (fire-and-forget) write beat can target the correct PA after
    // the entry has retired.
    input  logic [31:0]          xlate_pa,
    output logic                 mem_req_valid,
    output logic [31:0]          mem_req_vaddr,
    output logic                 mem_req_store,
    output logic                 full,
    output logic                 data_load_en,
    output logic [29:0]          data_addr,
    output logic [31:0]          data_store,
    output logic [3:0]           data_store_mask,
    // High while driving the second beat of a split store: the data_addr output
    // already carries the captured physical word address, so the core port must
    // bypass the (head-VA based) translation mux.
    output logic                 store_second_beat,
    // Hold the commit stage off for one cycle while the second store beat drains
    // so a younger store cannot collide on the single memory write port.
    output logic                 store_port_busy,
    output writeback_packet_t    load_writeback
);

    typedef struct packed {
        issue_entry_t entry;
        logic addr_ready;
        logic data_ready;
        logic issued_load;
        logic load_complete;
        logic double_low_valid;
        logic [31:0] addr;
        logic [31:0] load_low_word;
        logic [31:0] store_data;
        logic [31:0] store_data_upper;
        logic [3:0] store_mask;
        // Raw (unshifted) rs2 value for integer stores. The byte-offset shift
        // baked into store_data/store_mask depends on addr[1:0], which may not
        // be resolved when the data operand arrives; keeping the raw value lets
        // the formatted fields be re-derived once the address is known.
        logic [31:0] store_raw;
        // High word of a two-beat store (cross-word misaligned, or the upper
        // word of an FP double). store_mask_hi is zero for single-beat stores.
        logic [31:0] store_data_hi;
        logic [3:0]  store_mask_hi;
        // Captured physical word address of the high beat (from xlate_pa during
        // the high-word probe). Used to drive the second write after retire.
        logic [31:0] store_hi_pa;
    } mem_entry_t;

    mem_entry_t entries_q [MEM_Q_SIZE];
    mem_entry_t entries_next [MEM_Q_SIZE];
    logic [$clog2(MEM_Q_SIZE+1)-1:0] count_q, count_next;
    logic [$clog2(MEM_Q_SIZE)-1:0] head_q, head_next;
    logic [$clog2(MEM_Q_SIZE)-1:0] tail_q, tail_next;
    logic head_match, head_xlate_ok, head_xlate_flt;
    logic reservation_valid_q, reservation_valid_next;
    logic [31:0] reservation_addr_q, reservation_addr_next;
    logic double_store_pending_q, double_store_pending_next;
    logic [29:0] double_store_addr_q, double_store_addr_next;
    logic [31:0] double_store_data_q, double_store_data_next;
    logic [3:0]  double_store_mask_q, double_store_mask_next;
    // High while the head store is probing its high-word translation (cross-word
    // store / FP double). Steers mem_req_vaddr up one word so the second page is
    // translated and verified before either word is written (store atomicity).
    logic        store_probe_hi_q, store_probe_hi_next;

    assign full = (count_q > MEM_Q_SIZE - OOO_WIDTH);
    assign store_port_busy = double_store_pending_q;

    // Expose the registered head's virtual address so the core can translate it
    // (DTLB lookup / PTW). Only meaningful once the address operand is resolved.
    assign mem_req_valid = entries_q[head_q].entry.valid &&
        entries_q[head_q].addr_ready &&
        (entries_q[head_q].entry.ctrl.memRead ||
         entries_q[head_q].entry.ctrl.memWrite);
    // While probing the high word of a two-beat store, expose the high-word VA
    // (one word up) so the MMU translates/faults the second page. Adding 4
    // increments the word index and preserves the byte offset.
    assign mem_req_vaddr = entries_q[head_q].addr +
        (store_probe_hi_q ? 32'd4 : 32'd0);
    // AMOs read and write; treat as a store so the walker checks W permission.
    assign mem_req_store = entries_q[head_q].entry.ctrl.memWrite;

    always_comb begin
        entries_next = entries_q;
        count_next = count_q;
        head_next = head_q;
        tail_next = tail_q;
        data_load_en = 1'b0;
        data_addr = '0;
        data_store = '0;
        data_store_mask = '0;
        load_writeback = '0;
        store_second_beat = 1'b0;
        reservation_valid_next = reservation_valid_q;
        reservation_addr_next = reservation_addr_q;
        double_store_pending_next = 1'b0;
        double_store_addr_next = '0;
        double_store_data_next = '0;
        double_store_mask_next = '0;
        store_probe_hi_next = 1'b0;

        if (double_store_pending_q) begin
            data_addr = double_store_addr_q;
            data_store = double_store_data_q;
            data_store_mask = double_store_mask_q;
            // data_addr already holds the captured physical word address; tell
            // the core port to use it directly (skip the head-VA translation).
            store_second_beat = 1'b1;
        end

        for (int i = 0; i < MEM_Q_SIZE; i += 1) begin
            if ((entries_next[i].entry.branch_mask & abort_mask) != '0) begin
                entries_next[i] = '0;
            end else if (entries_next[i].entry.valid) begin
                entries_next[i].entry.branch_mask &= ~reset_mask;
                for (int w = 0; w < OOO_WIDTH; w += 1) begin
                    if (wakeup_valid[w]) begin
                        if (entries_next[i].entry.prs1 == wakeup_prd[w]) begin
                            entries_next[i].entry.src1_ready = 1'b1;
                            entries_next[i].addr_ready = 1'b1;
                            entries_next[i].addr = wakeup_data[w] +
                                ((entries_next[i].entry.ctrl.exec_class == EXEC_AMO) ?
                                 32'b0 : entries_next[i].entry.imm);
                        end
                        if ((entries_next[i].entry.prs2 == wakeup_prd[w]) &&
                                !((entries_next[i].entry.ctrl.exec_class == EXEC_FP) &&
                                  entries_next[i].entry.ctrl.memWrite)) begin
                            entries_next[i].entry.src2_ready = 1'b1;
                            entries_next[i].data_ready = 1'b1;
                            entries_next[i].store_raw = wakeup_data[w];
                            format_store_split(
                                entries_next[i].entry.ctrl.ldst_mode,
                                entries_next[i].addr[1:0],
                                wakeup_data[w],
                                entries_next[i].store_data,
                                entries_next[i].store_mask,
                                entries_next[i].store_data_hi,
                                entries_next[i].store_mask_hi);
                        end
                    end
                end
            end
        end

        // Re-derive the byte-offset-dependent store fields for plain integer
        // stores once both the address and data operands are resolved. This is
        // idempotent and corrects the case where the data arrived (and was
        // formatted) before the address, so addr[1:0] was not yet known. AMO/SC
        // stores are word-aligned (offset always 0, and their store_data may be
        // an amo_result) and FP stores are full-word, so both are left alone.
        for (int i = 0; i < MEM_Q_SIZE; i += 1) begin
            if (entries_next[i].entry.valid &&
                    entries_next[i].entry.ctrl.memWrite &&
                    !entries_next[i].entry.ctrl.memRead &&
                    (entries_next[i].entry.ctrl.exec_class != EXEC_FP) &&
                    (entries_next[i].entry.ctrl.exec_class != EXEC_AMO) &&
                    entries_next[i].addr_ready &&
                    entries_next[i].data_ready) begin
                format_store_split(
                    entries_next[i].entry.ctrl.ldst_mode,
                    entries_next[i].addr[1:0],
                    entries_next[i].store_raw,
                    entries_next[i].store_data,
                    entries_next[i].store_mask,
                    entries_next[i].store_data_hi,
                    entries_next[i].store_mask_hi);
            end
        end

        for (int i = 0; i < MEM_Q_SIZE; i += 1) begin
            if ((count_next != '0) && !entries_next[head_next].entry.valid) begin
                head_next = head_next + 1'b1;
                count_next -= 1'b1;
            end
        end

        // ---- Sv32 data translation gating ----
        // Under paging, a head memory op may only touch memory once its
        // translation is established (DTLB hit, walk done, no fault) and the
        // registered head matches the entry currently processed (so the DTLB
        // lookup driven from entries_q[head_q] corresponds to this access).
        head_match     = (head_next == head_q);
        head_xlate_ok  = !paging_data ||
            (head_match && mem_req_valid && !xlate_stall && !xlate_fault);
        head_xlate_flt = paging_data && head_match && mem_req_valid && xlate_fault;

        // Faulting access: retire it with an exception instead of touching memory.
        if (head_xlate_flt && !double_store_pending_q &&
                entries_next[head_next].entry.valid &&
                (entries_next[head_next].entry.ctrl.memRead ||
                 entries_next[head_next].entry.ctrl.memWrite) &&
                entries_next[head_next].entry.src1_ready &&
                !entries_next[head_next].issued_load) begin
            load_writeback.valid = 1'b1;
            load_writeback.active_id = entries_next[head_next].entry.active_id;
            load_writeback.prd = entries_next[head_next].entry.prd;
            load_writeback.has_dest = entries_next[head_next].entry.has_dest;
            load_writeback.branch_mask = entries_next[head_next].entry.branch_mask;
            load_writeback.exception = 1'b1;
            load_writeback.exc_cause = xlate_cause;
            load_writeback.data = entries_next[head_next].addr;   // mtval = VA
            entries_next[head_next] = '0;
            head_next = head_next + 1'b1;
            count_next -= 1'b1;
        end

        // Misaligned AMO / LR / SC: the implementation does not support
        // misaligned atomics (MISALIGNED_AMO=false, LR/SC "always raise access
        // fault"), so an unaligned address retires with an access fault and
        // never touches memory or the reservation. Gated by head_match so it
        // only acts on the registered head (and never clobbers a writeback the
        // fault block above already produced for a different entry).
        if (head_match && !double_store_pending_q &&
                entries_next[head_next].entry.valid &&
                (entries_next[head_next].entry.ctrl.exec_class == EXEC_AMO) &&
                entries_next[head_next].entry.src1_ready &&
                !entries_next[head_next].issued_load &&
                (entries_next[head_next].addr[1:0] != 2'b00)) begin
            load_writeback.valid = 1'b1;
            load_writeback.active_id = entries_next[head_next].entry.active_id;
            load_writeback.prd = entries_next[head_next].entry.prd;
            load_writeback.has_dest = entries_next[head_next].entry.has_dest;
            load_writeback.branch_mask = entries_next[head_next].entry.branch_mask;
            load_writeback.exception = 1'b1;
            load_writeback.exc_cause =
                (entries_next[head_next].entry.ctrl.amo_op == AMO_LR) ?
                    RISCV_Priv::EXC_LOAD_ACCESS : RISCV_Priv::EXC_STORE_ACCESS;
            load_writeback.data = entries_next[head_next].addr;   // mtval = VA
            entries_next[head_next] = '0;
            head_next = head_next + 1'b1;
            count_next -= 1'b1;
        end

        if (head_xlate_ok && !double_store_pending_q && entries_next[head_next].entry.valid &&
                (entries_next[head_next].entry.ctrl.exec_class == EXEC_AMO) &&
                (entries_next[head_next].entry.ctrl.amo_op == AMO_SC) &&
                entries_next[head_next].entry.src1_ready &&
                entries_next[head_next].entry.src2_ready &&
                !entries_next[head_next].issued_load) begin
            load_writeback.valid = 1'b1;
            load_writeback.active_id = entries_next[head_next].entry.active_id;
            load_writeback.prd = entries_next[head_next].entry.prd;
            load_writeback.has_dest = entries_next[head_next].entry.has_dest;
            load_writeback.branch_mask = entries_next[head_next].entry.branch_mask;
            load_writeback.data = (reservation_valid_q &&
                (reservation_addr_q == entries_next[head_next].addr)) ? 32'b0 : 32'b1;
            if (load_writeback.data == 32'b0) begin
                entries_next[head_next].issued_load = 1'b1;
                reservation_valid_next = 1'b0;
            end else begin
                entries_next[head_next] = '0;
                head_next = head_next + 1'b1;
                count_next -= 1'b1;
            end
        end

        if (head_xlate_ok && !double_store_pending_q && entries_next[head_next].entry.valid &&
                entries_next[head_next].entry.ctrl.memRead &&
                entries_next[head_next].entry.src1_ready && !entries_next[head_next].issued_load) begin
            data_load_en = 1'b1;
            data_addr = entries_next[head_next].addr[31:2];
            entries_next[head_next].issued_load = 1'b1;
        end

        // Pure-store completion / cross-word probe. A single-word store marks
        // itself complete as soon as its (low) translation resolves. A two-beat
        // store (cross-word misaligned, or FP double) must additionally verify
        // the high word's translation BEFORE it is allowed to commit, so that a
        // page fault on the second word is reported as the store's exception and
        // no partial write is ever performed (store atomicity).
        if (head_xlate_ok && !double_store_pending_q && entries_next[head_next].entry.valid &&
                entries_next[head_next].entry.ctrl.memWrite &&
                entries_next[head_next].entry.src1_ready &&
                entries_next[head_next].entry.src2_ready &&
                !entries_next[head_next].entry.ctrl.memRead &&
                !entries_next[head_next].issued_load) begin
            if (!needs_two_beats(entries_next[head_next].entry.ctrl,
                    entries_next[head_next].addr)) begin
                load_writeback.valid = 1'b1;
                load_writeback.active_id = entries_next[head_next].entry.active_id;
                load_writeback.branch_mask = entries_next[head_next].entry.branch_mask;
                load_writeback.has_dest = 1'b0;
                entries_next[head_next].issued_load = 1'b1;
            end else if (!store_probe_hi_q) begin
                // Low word translated OK; probe the high word next cycle.
                store_probe_hi_next = 1'b1;
            end else begin
                // High word translated OK; capture its PA and complete.
                entries_next[head_next].store_hi_pa = xlate_pa;
                load_writeback.valid = 1'b1;
                load_writeback.active_id = entries_next[head_next].entry.active_id;
                load_writeback.branch_mask = entries_next[head_next].entry.branch_mask;
                load_writeback.has_dest = 1'b0;
                entries_next[head_next].issued_load = 1'b1;
                store_probe_hi_next = 1'b0;
            end
        end else if (!double_store_pending_q && entries_next[head_next].entry.valid &&
                entries_next[head_next].entry.ctrl.memWrite &&
                entries_next[head_next].entry.src1_ready &&
                entries_next[head_next].entry.src2_ready &&
                !entries_next[head_next].entry.ctrl.memRead &&
                !entries_next[head_next].issued_load &&
                store_probe_hi_q) begin
            // High-word translation still walking: hold the probe phase.
            store_probe_hi_next = 1'b1;
        end

        if (!double_store_pending_q && entries_next[head_next].entry.valid &&
                entries_next[head_next].entry.ctrl.memRead &&
                entries_next[head_next].issued_load && data_load_valid &&
                !entries_next[head_next].load_complete &&
                (data_load_addr == entries_next[head_next].addr[31:2])) begin
            load_writeback.valid = 1'b1;
            load_writeback.active_id = entries_next[head_next].entry.active_id;
            load_writeback.prd = entries_next[head_next].entry.prd;
            load_writeback.has_dest = entries_next[head_next].entry.has_dest;
            load_writeback.branch_mask = entries_next[head_next].entry.branch_mask;
            if (entries_next[head_next].entry.ctrl.exec_class == EXEC_AMO) begin
                load_writeback.data = data_load;
                if (entries_next[head_next].entry.ctrl.amo_op == AMO_LR) begin
                    reservation_valid_next = 1'b1;
                    reservation_addr_next = entries_next[head_next].addr;
                    entries_next[head_next] = '0;
                    head_next = head_next + 1'b1;
                    count_next -= 1'b1;
                end else begin
                    entries_next[head_next].store_data = amo_result(
                        entries_next[head_next].entry.ctrl.amo_op,
                        data_load, entries_next[head_next].store_data);
                    entries_next[head_next].store_mask = 4'b1111;
                    entries_next[head_next].load_complete = 1'b1;
                end
            end else begin
                if (needs_two_beats(entries_next[head_next].entry.ctrl,
                        entries_next[head_next].addr) &&
                        !entries_next[head_next].double_low_valid) begin
                    // First beat of a two-beat load (cross-word misaligned, or
                    // the low word of an FP double): stash it, advance one word,
                    // and re-issue. Adding 4 preserves the byte offset used for
                    // the final extraction.
                    load_writeback = '0;
                    entries_next[head_next].load_low_word = data_load;
                    entries_next[head_next].double_low_valid = 1'b1;
                    entries_next[head_next].issued_load = 1'b0;
                    entries_next[head_next].addr =
                        entries_next[head_next].addr + 32'd4;
                end else begin
                    if (entries_next[head_next].double_low_valid) begin
                        // Final beat: combine {high, low} and extract the
                        // requested bytes at the original byte offset.
                        load_writeback.data = format_load_wide(
                            {data_load, entries_next[head_next].load_low_word},
                            entries_next[head_next].addr[1:0],
                            entries_next[head_next].entry.ctrl.ldst_mode);
                    end else begin
                        load_writeback.data = format_load(data_load,
                            entries_next[head_next].addr[1:0],
                            entries_next[head_next].entry.ctrl.ldst_mode);
                    end
                    if (entries_next[head_next].entry.ctrl.exec_class == EXEC_FP) begin
                    load_writeback.fp_write =
                        entries_next[head_next].entry.ctrl.fp_writes_fpr;
                    load_writeback.fp_rd = entries_next[head_next].entry.fp_rd;
                    load_writeback.fp_data = entries_next[head_next].entry.ctrl.fp_double ?
                        {data_load, entries_next[head_next].load_low_word} :
                        {32'hffff_ffff, data_load};
                    load_writeback.has_dest = 1'b0;
                    end
                    entries_next[head_next] = '0;
                    head_next = head_next + 1'b1;
                    count_next -= 1'b1;
                end
            end
        end

        if (head_xlate_ok && !double_store_pending_q && commit_store && entries_next[head_next].entry.valid &&
                entries_next[head_next].entry.ctrl.memWrite &&
                (entries_next[head_next].entry.active_id == commit_store_id)) begin
            // First (low) beat: written through the normal head-VA translation.
            data_addr = entries_next[head_next].addr[31:2];
            data_store = entries_next[head_next].store_data;
            data_store_mask = entries_next[head_next].store_mask;
            // Second (high) beat of a two-beat store: queue a fire-and-forget
            // write at the high word's captured physical address. The high page
            // was already proven fault-free during the probe above, so this
            // write cannot fault. store_second_beat tells the core port to use
            // this PA directly (the entry will have retired by then).
            if (needs_two_beats(entries_next[head_next].entry.ctrl,
                    entries_next[head_next].addr)) begin
                double_store_pending_next = 1'b1;
                double_store_addr_next = entries_next[head_next].store_hi_pa[31:2];
                double_store_data_next = entries_next[head_next].store_data_hi;
                double_store_mask_next = entries_next[head_next].store_mask_hi;
            end
            if (reservation_valid_next &&
                    (reservation_addr_next == entries_next[head_next].addr)) begin
                reservation_valid_next = 1'b0;
            end
            entries_next[head_next] = '0;
            head_next = head_next + 1'b1;
            count_next -= 1'b1;
        end

        if (!full) begin
            for (int lane = 0; lane < OOO_WIDTH; lane += 1) begin
                if (insert_valid[lane]) begin
                    entries_next[tail_next].entry = insert_entry[lane];
                    entries_next[tail_next].entry.valid = 1'b1;
                    entries_next[tail_next].addr_ready = insert_entry[lane].src1_ready;
                    entries_next[tail_next].data_ready = insert_entry[lane].src2_ready;
                    entries_next[tail_next].issued_load = 1'b0;
                    entries_next[tail_next].load_complete = 1'b0;
                    entries_next[tail_next].double_low_valid = 1'b0;
                    entries_next[tail_next].load_low_word = '0;
                    entries_next[tail_next].store_hi_pa = '0;
                    entries_next[tail_next].store_raw =
                        (insert_entry[lane].ctrl.exec_class == EXEC_FP) ?
                            '0 : insert_rs2_data[lane];
                    if (insert_entry[lane].src1_ready) begin
                        entries_next[tail_next].addr = insert_rs1_data[lane] +
                            ((insert_entry[lane].ctrl.exec_class == EXEC_AMO) ?
                             32'b0 : insert_entry[lane].imm);
                    end else begin
                        entries_next[tail_next].addr = '0;
                    end
                    if (insert_entry[lane].src2_ready ||
                            insert_entry[lane].ctrl.fp_uses_rs2) begin
                        if (insert_entry[lane].ctrl.exec_class == EXEC_FP) begin
                            // FP store: each word is full-width (FSW = low word
                            // only; FSD = both words). No byte-level splitting.
                            entries_next[tail_next].store_data =
                                insert_entry[lane].fp_src2_data[31:0];
                            entries_next[tail_next].store_mask = 4'b1111;
                            entries_next[tail_next].store_data_hi =
                                insert_entry[lane].fp_src2_data[63:32];
                            entries_next[tail_next].store_mask_hi =
                                insert_entry[lane].ctrl.fp_double ? 4'b1111 : 4'b0000;
                            entries_next[tail_next].store_data_upper =
                                insert_entry[lane].fp_src2_data[63:32];
                        end else begin
                            format_store_split(insert_entry[lane].ctrl.ldst_mode,
                                entries_next[tail_next].addr[1:0],
                                insert_rs2_data[lane],
                                entries_next[tail_next].store_data,
                                entries_next[tail_next].store_mask,
                                entries_next[tail_next].store_data_hi,
                                entries_next[tail_next].store_mask_hi);
                            entries_next[tail_next].store_data_upper = '0;
                        end
                    end else begin
                        entries_next[tail_next].store_data = '0;
                        entries_next[tail_next].store_data_upper = '0;
                        entries_next[tail_next].store_mask = '0;
                        entries_next[tail_next].store_data_hi = '0;
                        entries_next[tail_next].store_mask_hi = '0;
                    end
                    tail_next = tail_next + 1'b1;
                    count_next += 1'b1;
                end
            end
        end

        // Precise-trap flush: every queued op is younger than the trapping
        // instruction, so discard them all. Suppress any memory/writeback
        // effects driven above this cycle. The LR reservation is committed
        // architectural state and is intentionally preserved.
        if (flush) begin
            for (int i = 0; i < MEM_Q_SIZE; i += 1) begin
                entries_next[i] = '0;
            end
            head_next = '0;
            tail_next = '0;
            count_next = '0;
            data_load_en = 1'b0;
            load_writeback = '0;
            store_probe_hi_next = 1'b0;
            // A pending second store beat belongs to an already-committed store
            // (its first word is in memory and the high page was proven
            // fault-free), so it must still be written this cycle even though
            // the trap squashes everything younger. The write was already driven
            // at the top of this block; leave those outputs intact and just let
            // the one-shot pending flag clear.
            if (!double_store_pending_q) begin
                data_addr = '0;
                data_store = '0;
                data_store_mask = '0;
                store_second_beat = 1'b0;
            end
            double_store_pending_next = 1'b0;
            double_store_addr_next = '0;
            double_store_data_next = '0;
            double_store_mask_next = '0;
        end
    end

    // Access size in bytes implied by the load/store mode.
    function automatic logic [2:0] mem_size(input ldst_mode_t mode);
        unique case (mode)
            LDST_W:          mem_size = 3'd4;
            LDST_H, LDST_HU: mem_size = 3'd2;
            LDST_B, LDST_BU: mem_size = 3'd1;
            default:         mem_size = 3'd4;
        endcase
    endfunction

    // True when an access at byte offset byte_sel spills past the end of its
    // containing 32-bit word and therefore must be split into two word beats.
    function automatic logic mem_crosses(input ldst_mode_t mode,
            input logic [1:0] byte_sel);
        mem_crosses = (({1'b0, byte_sel} + mem_size(mode)) > 4'd4);
    endfunction

    // A memory op needs a second word beat when it is an FP double (FSD/FLD) or
    // an integer access that crosses a word boundary. AMO/LR/SC never split
    // (a misaligned atomic raises an access fault instead).
    function automatic logic needs_two_beats(input ctrl_signals_t ctrl,
            input logic [31:0] addr);
        needs_two_beats =
            ((ctrl.exec_class == EXEC_FP) && ctrl.fp_double) ||
            ((ctrl.exec_class != EXEC_AMO) &&
             mem_crosses(ctrl.ldst_mode, addr[1:0]));
    endfunction

    // Position a store value and byte-enable mask across (up to) two words at an
    // arbitrary byte offset. The high word's mask is zero for accesses that fit
    // within a single word (including within-word misaligned ones).
    task automatic format_store_split(input ldst_mode_t mode,
            input logic [1:0] byte_sel,
            input logic [31:0] value,
            output logic [31:0] store_value_lo,
            output logic [3:0]  store_mask_lo,
            output logic [31:0] store_value_hi,
            output logic [3:0]  store_mask_hi);
        logic [63:0] shifted;
        logic [7:0]  mask8;
        logic [3:0]  full_mask;
        unique case (mode)
            LDST_W:          full_mask = 4'b1111;
            LDST_H, LDST_HU: full_mask = 4'b0011;
            LDST_B, LDST_BU: full_mask = 4'b0001;
            default:         full_mask = 4'b0000;
        endcase
        shifted = ({32'b0, value}) << ({4'b0, byte_sel} * 6'd8);
        mask8   = ({4'b0, full_mask}) << byte_sel;
        store_value_lo = shifted[31:0];
        store_value_hi = shifted[63:32];
        store_mask_lo  = mask8[3:0];
        store_mask_hi  = mask8[7:4];
    endtask

    // Extract and sign/zero-extend a sub-word value contained entirely within a
    // single fetched word at byte offset byte_sel.
    function automatic logic [31:0] format_load(input logic [31:0] raw_word,
            input logic [1:0] byte_sel, input ldst_mode_t mode);
        logic [31:0] sh;
        sh = raw_word >> ({3'b0, byte_sel} * 6'd8);
        unique case (mode)
            LDST_W:  format_load = raw_word;
            LDST_H:  format_load = {{16{sh[15]}}, sh[15:0]};
            LDST_HU: format_load = {16'b0, sh[15:0]};
            LDST_B:  format_load = {{24{sh[7]}}, sh[7:0]};
            LDST_BU: format_load = {24'b0, sh[7:0]};
            default: format_load = 32'b0;
        endcase
    endfunction

    // Extract a value that straddles a word boundary from the 64-bit pair
    // {high_word, low_word} at byte offset byte_sel.
    function automatic logic [31:0] format_load_wide(input logic [63:0] pair,
            input logic [1:0] byte_sel, input ldst_mode_t mode);
        logic [63:0] sh;
        sh = pair >> ({3'b0, byte_sel} * 6'd8);
        unique case (mode)
            LDST_W:  format_load_wide = sh[31:0];
            LDST_H:  format_load_wide = {{16{sh[15]}}, sh[15:0]};
            LDST_HU: format_load_wide = {16'b0, sh[15:0]};
            LDST_B:  format_load_wide = {{24{sh[7]}}, sh[7:0]};
            LDST_BU: format_load_wide = {24'b0, sh[7:0]};
            default: format_load_wide = 32'b0;
        endcase
    endfunction

    function automatic logic [31:0] amo_result(input amo_op_t op,
            input logic [31:0] old_value, input logic [31:0] operand);
        unique case (op)
            AMO_SWAP: amo_result = operand;
            AMO_ADD:  amo_result = old_value + operand;
            AMO_XOR:  amo_result = old_value ^ operand;
            AMO_AND:  amo_result = old_value & operand;
            AMO_OR:   amo_result = old_value | operand;
            AMO_MIN:  amo_result = (signed'(old_value) < signed'(operand)) ?
                old_value : operand;
            AMO_MAX:  amo_result = (signed'(old_value) > signed'(operand)) ?
                old_value : operand;
            AMO_MINU: amo_result = (old_value < operand) ? old_value : operand;
            AMO_MAXU: amo_result = (old_value > operand) ? old_value : operand;
            default:  amo_result = old_value;
        endcase
    endfunction








    always_ff @(posedge clk or negedge rst_l) begin
        if (!rst_l) begin
            count_q <= '0;
            head_q <= '0;
            tail_q <= '0;
            reservation_valid_q <= 1'b0;
            reservation_addr_q <= '0;
            double_store_pending_q <= 1'b0;
            double_store_addr_q <= '0;
            double_store_data_q <= '0;
            double_store_mask_q <= '0;
            store_probe_hi_q <= 1'b0;
            for (int i = 0; i < MEM_Q_SIZE; i += 1) begin
                entries_q[i] <= '0;
            end
        end else begin
            count_q <= count_next;
            head_q <= head_next;
            tail_q <= tail_next;
            reservation_valid_q <= reservation_valid_next;
            reservation_addr_q <= reservation_addr_next;
            double_store_pending_q <= double_store_pending_next;
            double_store_addr_q <= double_store_addr_next;
            double_store_data_q <= double_store_data_next;
            double_store_mask_q <= double_store_mask_next;
            store_probe_hi_q <= store_probe_hi_next;
            entries_q <= entries_next;
        end
    end

endmodule: load_store_queue
