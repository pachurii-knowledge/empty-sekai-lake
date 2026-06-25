/**
 * niigo_cmi.vh  —  Coherence Mesh Interface (CMI) package
 *
 * THE single canonical CMI package for the 4-core MOESI CCD (plans/multicore-ccd.md
 * §13: the human-ratified resolution of the §11 blockers). Author this first; it is
 * `include`d by both the wheel routers (cmi_core_router / cmi_hub_xbar) and the L1/L2
 * coherence agents. NMI (niigo_mem.vh) is demoted to the L2<->MC leg only; coherence
 * rides CMI, NOT NMI (NMI_PROBE stays reserved/unused).
 *
 * Ratified decisions encoded here (plans/multicore-ccd.md §13):
 *   D1  CMI_FLIT_W = 128, a 512 b line = 4 body flits (multi-flit, narrow link).
 *   D2  L1I is directory-tracked: separate i_sharers/d_sharers presence vectors.
 *   D3  MOESI state encoding: I=0 S=1 E=2 O=3 M=4  (cmi_state_e).
 *   D4  CMI_TXN_W = 5; per-agent bound L1D = 1 demand + 1 snoop (txn id is node-local).
 *   D5  routing key = 3-bit flat node id (0..3 = cores, 4 = HUB).
 *   D6  ack_count[3:0] on the C2 Data grant; 3-hop invalidation, acks direct-to-requester.
 *   D7  reset = directory all DIR_I, vectors clear, credits full (CMI_DIR_META_RESET = '0).
 *   §13.2  flit-packing = sideband-ctrl: kind/vc ride a parallel sideband (cmi_sideband_t),
 *          the 128 data wires are PURE data, so 4x128 = 512 bit-exact (no payload steal).
 *   §13.9d C3 Writeback on its OWN VC4 (NUM_VC=5, vc field 3 bits; VC4 is spoke-only/leaf).
 *
 * Addressing: `laddr` is the same PHYSICAL line-aligned WORD-address space as NMI `waddr`
 * (niigo_mem.vh) — VIPT is internal to each L1, so the coherence wire stays word-addressed.
 */

`ifndef NIIGO_CMI_VH_
`define NIIGO_CMI_VH_

`include "riscv_isa.vh"
`include "riscv_uarch.vh"
`include "niigo_mem.vh"

package NIIGO_CMI;

    import RISCV_UArch::MEMORY_ADDR_WIDTH;
    import NIIGO_Mem::LINE_BITS;        // 512  (the coherent line == one 64 B / 512 b NMI line)

    // ================================================================
    // §13.1  Globals
    // ================================================================
    localparam int NUM_CORES  = 4;
    localparam int CORE_ID_W  = $clog2(NUM_CORES);          // 2
    localparam int NUM_NODES  = NUM_CORES + 1;              // 5 (4 cores + 1 hub) — D5
    localparam int NODE_ID_W  = $clog2(NUM_NODES);          // 3
    localparam logic [NODE_ID_W-1:0] CMI_HUB_ID = NODE_ID_W'(NUM_CORES); // 4 — D5

    localparam int CMI_FLIT_W = 128;                        // D1: physical link width
    localparam int LINE_FLITS = LINE_BITS / CMI_FLIT_W;     // 4 (512/128) — D1
    localparam int LINE_FLIT_W = $clog2(LINE_FLITS);        // 2 (beat index width)

    localparam int CMI_TXN_W  = 5;                          // D4: 32 txn ids / agent (MSHR tag)
    localparam int NUM_VC     = 5;                           // §13.9d: VC0..VC4 (C3 WB on its own VC4)
    localparam int VC_ID_W    = $clog2(NUM_VC);             // 3
    localparam int VC_DEPTH   = 2;                           // flits / VC / input port (credit window)

    // ack_count (D6): max INVs on a GetM = (L1D + L1I sharers of all cores) - requester
    //   = 2*NUM_CORES - 1 = 7 for 4 cores -> needs 3 bits; the head carries 4 bits (1-bit slack).
    localparam int ACK_CNT_W  = 4;

    // ================================================================
    // §13.1 / §M  MOESI line state (D3: I=0 S=1 E=2 O=3 M=4)
    //   Used IDENTICALLY in the L1D state_q, the CMI `gstate` grant field, and the
    //   directory's owner-state view. (Corrects the old §M M/O swap.)
    // ================================================================
    typedef enum logic [2:0] {
        CMI_I = 3'd0,   // invalid
        CMI_S = 3'd1,   // clean shared
        CMI_E = 3'd2,   // clean exclusive (sole clean copy)
        CMI_O = 3'd3,   // owned (dirty-shared) — the reason this is MOESI
        CMI_M = 3'd4    // modified exclusive (dirty, sole owner)
    } cmi_state_e;

    // Derived bits (keep the existing L1D flush/scan paths working — §M / §9.1).
    function automatic logic cmi_is_dirty(input cmi_state_e s); // WB-on-evict lines
        cmi_is_dirty = (s == CMI_M) || (s == CMI_O);
    endfunction
    function automatic logic cmi_writable(input cmi_state_e s);
        cmi_writable = (s == CMI_M) || (s == CMI_E);
    endfunction
    function automatic logic cmi_valid_st(input cmi_state_e s);
        cmi_valid_st = (s != CMI_I);
    endfunction

    // Directory state (coarser, 2-bit — distinct from the 3-bit line state above). L2-2.
    //   EM = exactly one core's L1D holds it E *or* M (dir can't tell — the silent-upgrade right).
    typedef enum logic [1:0] {
        DIR_I  = 2'd0,
        DIR_S  = 2'd1,
        DIR_EM = 2'd2,
        DIR_O  = 2'd3
    } dir_state_e;

    // ================================================================
    // §5 / §W.2  Message classes, VCs, ops
    // ================================================================
    // Flit kind (sideband). 512 b line = 1 HEAD + 4 BODY; a control packet = 1 HEADTAIL.
    typedef enum logic [1:0] {
        FLIT_HEAD     = 2'd0,   // header flit, body follows
        FLIT_BODY     = 2'd1,   // interior data flit
        FLIT_TAIL     = 2'd2,   // last data flit
        FLIT_HEADTAIL = 2'd3    // single-flit packet (no data)
    } flit_kind_e;

    // The 5 coherence message classes (§5). Class id != VC id (see cmi_vc): C3 -> VC4.
    typedef enum logic [2:0] {
        C0_REQ  = 3'd0,   // L1 -> dir       (VC0)  request
        C1_FWD  = 3'd1,   // dir -> L1       (VC1)  forwarded snoop / invalidate
        C2_DATA = 3'd2,   // L1<->L1, L2->L1 (VC2)  response data (always-sink; datelined on the ring)
        C3_WB   = 3'd3,   // L1 -> L2/MC     (VC4)  writeback data  (§13.9d: own VC)
        C4_ACK  = 3'd4    // L1 -> dir       (VC3)  completion ack / unblock
    } cmi_class_e;

    // Coherence ops (§W.2 names win; §9.2 semantics). 16 ops -> 4-bit. The class a given
    // op belongs to is fixed (cmi_op_class); `is_icache` on the head distinguishes the
    // L1I vs L1D requester for D2 directory tracking (GETS/PUTS/INV_ACK).
    typedef enum logic [3:0] {
        // C0 request (L1 -> dir)
        OP_GETS     = 4'd0,   // read-shared:   I -> S/E              (L1D load miss; L1I ifill if is_icache)
        OP_GETM     = 4'd1,   // read-for-own:  I/S/O -> M            (store miss / SC / AMO)
        OP_UPGRADE  = 4'd2,   // S->M or O->M, no data needed
        OP_PUTM     = 4'd3,   // evict M: writeback dirty + relinquish (emits a C3 WB_DATA)
        OP_PUTO     = 4'd4,   // evict O: writeback dirty + relinquish ownership (emits C3 WB_DATA)
        OP_PUTS     = 4'd5,   // evict S: clean drop (noisy v1, §9.7)
        OP_PUTE     = 4'd6,   // evict E: clean drop (noisy v1, §9.7)
        // C1 forwarded-snoop (dir -> L1)
        OP_FWD_GETS = 4'd7,   // owner: supply data to requester, downgrade (M->O / E->S / O stays O)
        OP_FWD_GETM = 4'd8,   // owner: supply data to requester, then self -> I
        OP_INV      = 4'd9,   // sharer: invalidate, send INV_ACK direct to requester
        OP_DOWNGRADE= 4'd10,  // M/E -> S without invalidate (rare; clean-before-refill case)
        // C2 response-data (L1<->L1, L2->L1)
        OP_DATA     = 4'd11,  // 512 b line + gstate grant + is_owner + ack_count (D6)
        // C3 writeback-data (L1 -> L2/MC)
        OP_WB_DATA  = 4'd12,  // the dirty line payload for PUTM/PUTO
        // C4 completion-ack (L1 -> dir, and INV_ACK L1 -> requester)
        OP_ACK      = 4'd13,  // generic ack (e.g. WB_ACK from dir is the dir->L1 variant)
        OP_INV_ACK  = 4'd14,  // sharer acknowledges INV (carries the GetM's txn_id; -> requester)
        OP_UNBLOCK  = 4'd15   // requester tells dir "txn closed, I am in final state"
    } cmi_op_e;

    // class -> VC (§13.9d): C3 Writeback gets its OWN VC4; C4 Ack stays VC3.
    function automatic logic [VC_ID_W-1:0] cmi_vc(input cmi_class_e c);
        unique case (c)
            C0_REQ : cmi_vc = 3'd0;
            C1_FWD : cmi_vc = 3'd1;
            C2_DATA: cmi_vc = 3'd2;
            C4_ACK : cmi_vc = 3'd3;
            C3_WB  : cmi_vc = 3'd4;
            default: cmi_vc = 3'd0;
        endcase
    endfunction

    // op -> class (for assertions / routing decode).
    function automatic cmi_class_e cmi_op_class(input cmi_op_e op);
        unique case (op)
            OP_GETS, OP_GETM, OP_UPGRADE,
            OP_PUTM, OP_PUTO, OP_PUTS, OP_PUTE       : cmi_op_class = C0_REQ;
            OP_FWD_GETS, OP_FWD_GETM,
            OP_INV, OP_DOWNGRADE                      : cmi_op_class = C1_FWD;
            OP_DATA                                   : cmi_op_class = C2_DATA;
            OP_WB_DATA                                : cmi_op_class = C3_WB;
            OP_ACK, OP_INV_ACK, OP_UNBLOCK            : cmi_op_class = C4_ACK;
            default                                   : cmi_op_class = C0_REQ;
        endcase
    endfunction

    // Does this op carry a 512 b data line (HEAD + 4 BODY) vs a single HEADTAIL?
    function automatic logic cmi_op_has_body(input cmi_op_e op);
        cmi_op_has_body = (op == OP_DATA) || (op == OP_WB_DATA);
    endfunction

    // Reserved ring data VC: the C2 data VC is the one datelined on the ring (split into
    // two sub-VCs at the dateline link); VC4 (C3 WB) is spoke-only and never rides the ring.
    localparam logic [VC_ID_W-1:0] CMI_DATELINE_VC = 3'd2;

    // ================================================================
    // §13.2 / §W.2  Flit format — sideband-ctrl scheme
    //   Physical link/dir = { valid, cmi_sideband_t ctrl, 128b data }.
    //   On HEAD/HEADTAIL the 128b data = the packed cmi_head_t; on BODY/TAIL it is a pure
    //   128b data chunk. kind/vc travel on the sideband (NOT in the 128b payload), so a
    //   512 b line = 4x128 bit-exact.
    // ================================================================
    typedef struct packed {
        cmi_class_e                    mclass;     // 3  message class
        cmi_op_e                       op;         // 4  coherence op
        logic [NODE_ID_W-1:0]          dst;        // 3  routing target node (D5: core 0..3 or HUB=4)
        logic [CORE_ID_W-1:0]          src_core;   // 2  originator core (= req_core on C1/C2/C4)
        logic                          is_icache;  // 1  D2: 1 = L1I-side req (GETS/PUTS/INV_ACK), 0 = L1D
        logic [CMI_TXN_W-1:0]          txn_id;     // 5  requester MSHR tag (echoed end-to-end)  D4
        cmi_state_e                    gstate;     // 3  MOESI grant on C2 Data (D3)
        logic                          is_owner;   // 1  C2: data confers ownership (=0 on read-share)
        logic [ACK_CNT_W-1:0]          ack_count;  // 4  D6: #INV_ACKs requester collects (on C2 Data)
        logic [MEMORY_ADDR_WIDTH-1:0]  laddr;      // physical LINE word addr (PIPT; == NMI waddr space)
    } cmi_head_t;

    localparam int CMI_HEAD_W = $bits(cmi_head_t);   // RV32: 56b ; RV64: 87b  (both <= 128)

    typedef logic [CMI_FLIT_W-1:0] cmi_body_t;       // a pure 128 b data chunk (sideband carries kind/vc)

    // The parallel control sideband (~5 wires/port/dir).
    typedef struct packed {
        flit_kind_e          kind;   // 2
        logic [VC_ID_W-1:0]  vc;     // 3
    } cmi_sideband_t;

    // One physical direction of a CMI link (router_out -> router_in).
    typedef struct packed {
        logic                  valid;   // a flit is present this cycle
        cmi_sideband_t         ctrl;    // kind + vc (the sideband)
        logic [CMI_FLIT_W-1:0] data;    // 128b: packed head (HEAD/HEADTAIL) or pure data (BODY/TAIL)
    } cmi_link_t;

    // The reverse (credit) wire: one pulse per drained input-buffer slot, per VC.
    typedef struct packed {
        logic [NUM_VC-1:0]     credit;
    } cmi_credit_t;

    // Head <-> 128b-data packing (zero-extend into the physical flit word).
    function automatic logic [CMI_FLIT_W-1:0] cmi_head_pack(input cmi_head_t h);
        cmi_head_pack = {{(CMI_FLIT_W-CMI_HEAD_W){1'b0}}, h};
    endfunction
    function automatic cmi_head_t cmi_head_unpack(input logic [CMI_FLIT_W-1:0] d);
        cmi_head_unpack = d[CMI_HEAD_W-1:0];
    endfunction

    // Line <-> body-flit (de)serialization. Beat order: flit 0 = line[127:0], ... (low first).
    function automatic cmi_body_t cmi_line_flit(input logic [LINE_BITS-1:0] line,
                                                input logic [LINE_FLIT_W-1:0] beat);
        cmi_line_flit = line[beat*CMI_FLIT_W +: CMI_FLIT_W];
    endfunction

    // ================================================================
    // §13.3  Directory coherence metadata (D2: split L1I/L1D tracking)
    //   The full L2 directory entry (niigo_l2_pkg.vh) is { L2 tag, cmi_dir_meta_t,
    //   data_present, l2_dirty, plru }. The coherence-protocol part lives here (shared).
    // ================================================================
    typedef logic [NUM_CORES-1:0] cmi_sharer_vec_t;   // 1 bit / core

    typedef struct packed {
        dir_state_e            dstate;     // I / S / EM / O
        cmi_sharer_vec_t       d_sharers;  // L1D presence vector (D2)
        cmi_sharer_vec_t       i_sharers;  // L1I presence vector (D2: directory-tracked, no broadcast)
        logic [CORE_ID_W-1:0]  owner;      // L1D owner core (L1I never owns); valid only in EM/O
    } cmi_dir_meta_t;

    // D7 reset: directory comes up all-invalid (DIR_I=0, vectors '0, owner '0).
    localparam cmi_dir_meta_t CMI_DIR_META_RESET = '0;

    // D6 ack-count: # of INV_ACKs a GetM/Upgrade requester must collect =
    //   popcount(d_sharers \ req) + popcount(i_sharers \ req). The requester's own
    //   copies are never invalidated, so its bit is cleared in both vectors first.
    function automatic logic [ACK_CNT_W-1:0]
            cmi_ack_count(input cmi_dir_meta_t m, input logic [CORE_ID_W-1:0] req);
        cmi_sharer_vec_t d_inv, i_inv;
        d_inv = m.d_sharers;
        i_inv = m.i_sharers;
        d_inv[req] = 1'b0;
        i_inv[req] = 1'b0;
        cmi_ack_count = ACK_CNT_W'($countones(d_inv) + $countones(i_inv));
    endfunction

    // ================================================================
    // §W.1  Wheel ring topology helpers (ring order: C0 - C1 - C3 - C2 - C0)
    //   E = forward direction (+1 ring position), W = backward (-1). HUB (node 4) is not
    //   on the ring. The ring carries C2 cache-to-cache data only; the spoke is the escape.
    // ================================================================
    // core id -> ring position along the cycle C0(0)-C1(1)-C3(2)-C2(3)
    function automatic logic [1:0] cmi_ring_pos(input logic [CORE_ID_W-1:0] core);
        unique case (core)
            2'd0: cmi_ring_pos = 2'd0;   // C0
            2'd1: cmi_ring_pos = 2'd1;   // C1
            2'd3: cmi_ring_pos = 2'd2;   // C3
            2'd2: cmi_ring_pos = 2'd3;   // C2
            default: cmi_ring_pos = 2'd0;
        endcase
    endfunction
    // ring position -> core id (inverse)
    function automatic logic [CORE_ID_W-1:0] cmi_core_at_pos(input logic [1:0] pos);
        unique case (pos)
            2'd0: cmi_core_at_pos = 2'd0;   // C0
            2'd1: cmi_core_at_pos = 2'd1;   // C1
            2'd2: cmi_core_at_pos = 2'd3;   // C3
            2'd3: cmi_core_at_pos = 2'd2;   // C2
            default: cmi_core_at_pos = 2'd0;
        endcase
    endfunction
    // E-ring (forward) neighbour of a core
    function automatic logic [CORE_ID_W-1:0] cmi_ring_e(input logic [CORE_ID_W-1:0] core);
        cmi_ring_e = cmi_core_at_pos(cmi_ring_pos(core) + 2'd1);
    endfunction
    // W-ring (backward) neighbour of a core
    function automatic logic [CORE_ID_W-1:0] cmi_ring_w(input logic [CORE_ID_W-1:0] core);
        cmi_ring_w = cmi_core_at_pos(cmi_ring_pos(core) - 2'd1);
    endfunction
    // shortest ring direction src->dst: 0 = E (forward), 1 = W (backward). Opposite (dist 2) -> E.
    function automatic logic cmi_ring_dir(input logic [CORE_ID_W-1:0] src,
                                          input logic [CORE_ID_W-1:0] dst);
        logic [1:0] fwd;
        fwd = cmi_ring_pos(dst) - cmi_ring_pos(src);   // forward hop count, mod 4
        cmi_ring_dir = (fwd <= 2'd2) ? 1'b0 : 1'b1;
    endfunction

    // node-id helpers (D5: cores are nodes 0..3, HUB = 4)
    function automatic logic cmi_is_core(input logic [NODE_ID_W-1:0] n);
        cmi_is_core = (n < NODE_ID_W'(NUM_CORES));
    endfunction
    function automatic logic [NODE_ID_W-1:0] cmi_core_node(input logic [CORE_ID_W-1:0] c);
        cmi_core_node = {{(NODE_ID_W-CORE_ID_W){1'b0}}, c};
    endfunction

endpackage : NIIGO_CMI

`endif /* NIIGO_CMI_VH_ */
