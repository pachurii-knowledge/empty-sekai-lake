/**
 * niigo_ccd_m1.vh  --  M1 dual-core CCD bring-up message layer
 *
 * The M1 milestone (plans/multicore-ccd.md §12: "2 cores, 1 directory, no mesh —
 * direct CMI link") brings up the MOESI protocol in RTL with a SIMPLIFIED physical
 * layer: each coherence message carries the WHOLE 512 b line on one full-line bus
 * (`ccd_msg_t`), instead of the canonical 128 b multi-flit serialisation (§13.1/§W.2).
 * Flit serialisation + the wheel routers are an M3 refinement; M1 validates the
 * PROTOCOL (the directory FSM + the L1D agent), which the formal models (formal/*.m)
 * already proved. The message reuses the canonical NIIGO_CMI enums verbatim.
 *
 * Also defines `cmi_owner_next_e` — the 3-valued FwdGetS-downgrade outcome the v3
 * formal model pinned (R-a / plan §13.9d): O = owner keeps ownership, S = owner stays
 * a clean sharer (E->S), I = owner is leaving (mid-eviction) -> directory clears its
 * sharer bit (no phantom sharer). (TODO: fold `onext` into the canonical cmi_head_t in
 * niigo_cmi.vh, replacing the 1-bit is_owner, when the flit layer lands at M3.)
 */

`ifndef NIIGO_CCD_M1_VH_
`define NIIGO_CCD_M1_VH_

`include "niigo_cmi.vh"

package NIIGO_CCD_M1;

    import NIIGO_Mem::LINE_BITS;             // 512
    import RISCV_UArch::MEMORY_ADDR_WIDTH;   // line word-addr width (30 RV32 / 61 RV64)
    import NIIGO_CMI::CORE_ID_W;             // 2
    import NIIGO_CMI::ACK_CNT_W;             // 4
    import NIIGO_CMI::cmi_op_e;
    import NIIGO_CMI::cmi_state_e;

    // R-a: the owner's post-FwdGetS outcome (carried owner -> dir -> requester echo).
    typedef enum logic [1:0] {
        ON_NA = 2'd0,   // not applicable (non-FwdGetS message)
        ON_O  = 2'd1,   // owner keeps dirty data + ownership (M->O, or O stays O)
        ON_S  = 2'd2,   // owner relinquishes ownership, stays a clean sharer (E->S)
        ON_I  = 2'd3    // owner is leaving (mid-eviction) -> dir clears its sharer bit
    } cmi_owner_next_e;

    // The M1 full-line coherence message (one beat carries the whole line).
    typedef struct packed {
        cmi_op_e                       op;        // GETS/GETM/UPGRADE/PUT*/FWD*/INV/DATA/INVACK/UNBLOCK/WBACK
        logic [CORE_ID_W-1:0]          src;       // originator core (the owner/sharer on a response)
        logic                          is_icache; // D2 discriminator (M1: L1D only -> 0; L1I is v4c)
        cmi_state_e                    gst;       // MOESI grant state on a DATA response
        cmi_owner_next_e               onext;     // owner's next-state on a FwdGetS data forward (R-a)
        logic [ACK_CNT_W-1:0]          acks;      // ack_count stamped on a DATA grant (D6)
        logic [CORE_ID_W-1:0]          req;       // the requester to forward/ack to (on FWD/INV)
        logic [LINE_BITS-1:0]          line;      // full 512 b line payload (DATA / WB-DATA / PutM/PutO)
        logic [MEMORY_ADDR_WIDTH-1:0]  laddr;     // physical LINE word address (PIPT)
    } ccd_msg_t;

    // One direction of an M1 core<->directory link: valid + message (+ ready for flow control).
    typedef struct packed {
        logic      valid;
        ccd_msg_t  msg;
    } ccd_chan_t;

    // ---- M3 flit transport: the ccd_msg control (everything but the 512 b line) packed
    //      ABOVE the router-visible cmi_rhdr_t in a 128 b HEAD flit. The line, when present,
    //      rides 4 body flits (§W.2). cmi_msg_tx/cmi_msg_rx (de)serialise this over the wheel. ----
    typedef struct packed {
        cmi_op_e                       op;
        logic [CORE_ID_W-1:0]          src;
        logic                          is_icache;
        cmi_state_e                    gst;
        cmi_owner_next_e               onext;
        logic [ACK_CNT_W-1:0]          acks;
        logic [CORE_ID_W-1:0]          req;
        logic [MEMORY_ADDR_WIDTH-1:0]  laddr;
    } ccd_ctrl_t;                                  // RV32: 48b ; RV64: 79b  (+8b rhdr <= 128)
    localparam int CCD_CTRL_W = $bits(ccd_ctrl_t);

    // ops whose ccd_msg carries a 512 b line (head + 4 body flits): DATA forward, WB, dirty Put.
    function automatic logic ccd_has_line(input cmi_op_e op);
        ccd_has_line = (op==NIIGO_CMI::OP_DATA)  || (op==NIIGO_CMI::OP_WB_DATA) ||
                       (op==NIIGO_CMI::OP_PUTM)  || (op==NIIGO_CMI::OP_PUTO);
    endfunction

    // ---- the L1D agent's core-side request interface (a simplified LSQ port) ----
    typedef enum logic [2:0] {
        COP_LOAD, COP_STORE, COP_LR, COP_SC, COP_AMO
    } l1_core_op_e;

    typedef enum logic [2:0] {            // AMO sub-op (the RMW the cache applies atomically)
        AMO_ADD, AMO_SWAP, AMO_OR, AMO_AND, AMO_XOR
    } l1_amo_op_e;

endpackage : NIIGO_CCD_M1

`endif /* NIIGO_CCD_M1_VH_ */
