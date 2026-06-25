-- ============================================================================
-- moesi_ccd.m  --  CMurphi formal model of the niigo-lake 4-core MOESI protocol
--
-- Models plans/multicore-ccd.md §9 (L1D/directory transition tables) + §13.9
-- (NINE data-source priority, D6 ack-to-requester, noisy E/S eviction).
--
-- Scope (v4a):  v1's serialised L1D+directory+NINE protocol *** PLUS A PER-CORE L1I
--   INSTRUCTION CACHE ***, to validate decision D2: the directory tracks L1I and L1D
--   presence in SEPARATE vectors (i_sharers / d_sharers), invalidates BOTH on a GetM,
--   and ack-counts over both (§13.3/§13.4). The L1I holds only {I, S} (+ a transient
--   IS_D ifetch); it issues GetS(is_icache), is granted S only (never E/M/O), takes Inv
--   -> I + Inv-Ack, and clean-evicts via a fire-and-forget PutS(is_icache). The I/D
--   discriminator rides a Message.is_icache bit (the §13.1 head bit), set by a SendI()
--   wrapper for L1I-tagged traffic so the 50 existing Send() call sites are untouched.
--   Self-CMODX: an L1D GetM/Upgrade Invs that core's OWN L1I too (coherent-I-cache,
--   §9.9 — the array stays current; front-end fence.i is not modelled).
--   Built on the SERIALISED base (no grant-and-go) so the L1I needs no deferral.
--   Deferred to v4b: LR/SC + AMO. To v5: 3-core (RAM), multi-address, per-VC net model.
--
-- Checks:  SWMR (incl. L1I), single-owner, data-value correctness (L1D + L1I copies),
--   memory correctness (DIR_I, both vectors empty), directory-owner consistency, and
--   (Murphi built-in) deadlock freedom.
--
-- RESULT (CMurphi 5.4.9):  No error found, deadlock-free —
--   NumCores=2 : 805,861 states / 2,566,362 rules (~7 s, ~6 GB). 2 cores fully exercises
--   the L1I/D2 logic: an L1I caches code (IC_S), the directory tracks i_sharers, and a GetM
--   invalidates BOTH vectors incl. the requester's own L1I (self-CMODX). Coverage probes
--   (an L1I never reaches IC_S; the directory never tracks an i_sharer) both FAIL -> exercised.
--   3-core not run: the L1I doubles the per-core agent count -> exceeds available RAM
--   (consistent with the v3 ceiling). NumCores=2 is the committed default.
-- ============================================================================

const
  NumCores: 2;          -- L1I doubles the agents; 2 cores keeps it tractable
  NetMax:   18;          -- per-node message-buffer bound

type
  Core:  scalarset(NumCores);
  Home:  enum { HomeType };
  Node:  union { Home, Core };

  Value: 0..1;           -- data values (2 enough to expose corruption)

  -- v4a: ack_count spans BOTH vectors -> up to (NumCores-1 other L1D) + (NumCores L1I).
  CountType: 0..2*NumCores;            -- sharer counts / ack_count
  AckType:   -2*NumCores..2*NumCores;  -- remaining InvAcks (signed: an ack may precede the count)

  VCType: 0..2;          -- 0 = Req (C0), 1 = Fwd (C1), 2 = Resp (C2/C3/C4)

  GrantState: enum { Gnt_S, Gnt_E, Gnt_M };

  MsgOp: enum {
    GetS, GetM, Upgrade, PutM, PutO, PutS, PutE,   -- C0 request  (core -> home)
    FwdGetS, FwdGetM, Inv,                          -- C1 snoop     (home -> core)
    Data,                                           -- C2 data      (home/owner -> core)
    InvAck, Unblock, WBAck                          -- C4 acks      (-> requester / home / core)
  };

  CacheState: enum {
    C_I, C_S, C_E, C_O, C_M,            -- stable MOESI (L1D)
    IS_D, IM_AD, IM_A, SM_AD, OM_A,     -- acquire transients
    MI_A, OI_A, EI_A, SI_A, II_A        -- evict transients
  };

  -- v4a: the L1I instruction cache — read-only. {IC_I, IC_S} + ifetch transient IC_IS_D +
  -- evict transient IC_SI_A (waits WBAck so the L1I cannot re-fetch before its PutS is
  -- processed — else a stale GetM-Inv to the not-yet-cleared i_sharer races the re-fetch).
  ICacheState: enum { IC_I, IC_S, IC_IS_D, IC_SI_A };

  DirState: enum {
    DIR_I, DIR_S, DIR_EM, DIR_O,        -- stable
    DIR_S_D, DIR_M_D                    -- transient (awaiting Unblock from requester)
  };

  Message: record
    op:     MsgOp;
    src:    Node;
    dst:    Node;
    vc:     VCType;
    val:    Value;          -- Data / PutM / PutO payload
    gst:    GrantState;     -- grant on a Data response
    ostays: boolean;        -- (R-a) owner keeps ownership after a FwdGetS forward
    acks:   CountType;      -- ack_count stamped on a Data/grant (D6)
    req:    Node;           -- on Fwd/Inv: the requester to forward/ack to
    is_icache: boolean;     -- v4a/D2: this message targets/originates the L1I (else the L1D)
  end;

  ProcRec: record           -- the L1D
    state: CacheState;
    val:   Value;
    acks:  AckType;         -- remaining InvAcks (signed: an ack may precede the count)
  end;

  IProcRec: record          -- the L1I (v4a): read-only {IC_I, IC_S, IC_IS_D}
    state: ICacheState;
    val:   Value;           -- the code value held in IC_S (for the data-value invariant)
  end;

  HomeRec: record
    state:   DirState;
    owner:   Node;                   -- valid in EM/O (an L1D Core); owner is also in d_sharers
    d_sharers: array[Core] of boolean; -- L1D presence vector
    i_sharers: array[Core] of boolean; -- L1I presence vector (D2/§13.3)
    val:     Value;                  -- L2/memory backing copy (the always-present source)
    pendReq: Node;                   -- requester being served in S_D/M_D
    pendOwner: Node;                 -- the owner forwarded-to (S_D); undefined for an L2-sourced grant
    pendEgrant: boolean;             -- a fresh L2 GetS grant was Exclusive (-> EM on Unblock)
    pendIcache: boolean;             -- v4a: the DIR_S_D requester is an L1I (-> add to i_sharers)
  end;

var
  Procs:    array[Core] of ProcRec;   -- L1D per core
  IProcs:   array[Core] of IProcRec;  -- L1I per core (v4a)
  HomeNode: HomeRec;
  Net:      array[Node] of multiset[NetMax] of Message;
  LastData: Value;                   -- auxiliary: globally last-written value

-- ============================================================================
-- Helpers
-- ============================================================================

procedure SendG(op: MsgOp; dst: Node; src: Node; vc: VCType;
                val: Value; gst: GrantState; ostays: boolean;
                acks: CountType; req: Node; icache: boolean);
var m: Message;
begin
  assert (MultiSetCount(i: Net[dst], true) < NetMax) "network buffer overflow";
  m.op := op; m.src := src; m.dst := dst; m.vc := vc;
  m.val := val; m.gst := gst; m.ostays := ostays; m.acks := acks; m.req := req;
  m.is_icache := icache;
  MultiSetAdd(m, Net[dst]);
end;

-- L1D / directory traffic (is_icache = false) — the 50 existing call sites use this verbatim.
procedure Send(op: MsgOp; dst: Node; src: Node; vc: VCType;
               val: Value; gst: GrantState; ostays: boolean;
               acks: CountType; req: Node);
begin
  SendG(op, dst, src, vc, val, gst, ostays, acks, req, false);
end;

-- L1I-tagged traffic (is_icache = true): an L1I's own GetS/PutS/InvAck, and the directory's
-- Inv to / Data grant for an L1I.
procedure SendI(op: MsgOp; dst: Node; src: Node; vc: VCType;
                val: Value; gst: GrantState; ostays: boolean;
                acks: CountType; req: Node);
begin
  SendG(op, dst, src, vc, val, gst, ostays, acks, req, true);
end;

function OtherSharers(r: Core): CountType;   -- |d_sharers \ {r}|  (L1D-only count)
var n: CountType;
begin
  n := 0;
  for c: Core do
    if HomeNode.d_sharers[c] & (c != r) then n := n + 1; endif;
  endfor;
  return n;
end;

-- D2/§13.4 ack_count: invalidate every OTHER L1D sharer (\ {r}) AND every L1I sharer
-- (including r's own L1I — a store Invs the requester's own stale code copy, self-CMODX).
function OtherSharersBoth(r: Core): CountType;
var n: CountType;
begin
  n := 0;
  for c: Core do
    if HomeNode.d_sharers[c] & (c != r) then n := n + 1; endif;
    if HomeNode.i_sharers[c]               then n := n + 1; endif;
  endfor;
  return n;
end;

function AllSharersEmpty(): boolean;   -- no L1D and no L1I copies anywhere
begin
  for c: Core do
    if HomeNode.d_sharers[c] | HomeNode.i_sharers[c] then return false; endif;
  endfor;
  return true;
end;

procedure ClearSharers();   -- wipe BOTH vectors (used on every exclusive / DIR_I transition)
begin
  for c: Core do HomeNode.d_sharers[c] := false; HomeNode.i_sharers[c] := false; endfor;
end;

function IsOwnerState(s: CacheState): boolean;
begin return (s = C_M) | (s = C_O) | (s = C_E); end;

function IsValidStable(s: CacheState): boolean;
begin return (s = C_M) | (s = C_O) | (s = C_E) | (s = C_S); end;

function DirStable(): boolean;
begin
  return (HomeNode.state = DIR_I) | (HomeNode.state = DIR_S)
       | (HomeNode.state = DIR_EM) | (HomeNode.state = DIR_O);
end;

-- ============================================================================
-- Core-initiated events (core MSHR free = a stable cache state; directory stable)
-- ============================================================================

ruleset c: Core do

  rule "load-miss I -> GetS"
    (Procs[c].state = C_I) & DirStable()
  ==> begin
    Procs[c].state := IS_D; Procs[c].acks := 0;
    Send(GetS, HomeType, c, 0, 0, Gnt_S, false, 0, c);
  end;

  rule "store-miss I -> GetM"
    (Procs[c].state = C_I) & DirStable()
  ==> begin
    Procs[c].state := IM_AD; Procs[c].acks := 0;
    Send(GetM, HomeType, c, 0, 0, Gnt_S, false, 0, c);
  end;

  rule "store-hit S -> Upgrade (SM_AD)"
    (Procs[c].state = C_S) & DirStable()
  ==> begin
    Procs[c].state := SM_AD; Procs[c].acks := 0;
    Send(Upgrade, HomeType, c, 0, 0, Gnt_S, false, 0, c);
  end;

  rule "store-hit O -> Upgrade (OM_A)"
    (Procs[c].state = C_O) & DirStable()
  ==> begin
    Procs[c].state := OM_A; Procs[c].acks := 0;
    Send(Upgrade, HomeType, c, 0, 0, Gnt_S, false, 0, c);
  end;

  rule "store-hit E -> M (silent)"
    (Procs[c].state = C_E)
  ==> begin
    Procs[c].state := C_M; Procs[c].val := 1 - Procs[c].val; LastData := Procs[c].val;
  end;

  rule "store-hit M (write)"
    (Procs[c].state = C_M)
  ==> begin
    Procs[c].val := 1 - Procs[c].val; LastData := Procs[c].val;
  end;

  rule "evict M -> PutM (MI_A)"
    (Procs[c].state = C_M) & DirStable()
  ==> begin
    Procs[c].state := MI_A;
    Send(PutM, HomeType, c, 0, Procs[c].val, Gnt_S, false, 0, c);
  end;

  rule "evict O -> PutO (OI_A)"
    (Procs[c].state = C_O) & DirStable()
  ==> begin
    Procs[c].state := OI_A;
    Send(PutO, HomeType, c, 0, Procs[c].val, Gnt_S, false, 0, c);
  end;

  rule "evict E -> PutE (EI_A, noisy)"
    (Procs[c].state = C_E) & DirStable()
  ==> begin
    Procs[c].state := EI_A;
    Send(PutE, HomeType, c, 0, 0, Gnt_S, false, 0, c);
  end;

  rule "evict S -> PutS (SI_A, noisy)"
    (Procs[c].state = C_S) & DirStable()
  ==> begin
    Procs[c].state := SI_A;
    Send(PutS, HomeType, c, 0, 0, Gnt_S, false, 0, c);
  end;

endruleset;

-- ============================================================================
-- L1I (instruction cache) core-initiated events (v4a). Read-only: {IC_I, IC_S}.
-- ============================================================================

ruleset c: Core do

  rule "ifetch-miss IC_I -> GetS(icache)"
    (IProcs[c].state = IC_I) & DirStable()
  ==> begin
    IProcs[c].state := IC_IS_D;
    SendI(GetS, HomeType, c, 0, 0, Gnt_S, false, 0, c);   -- is_icache=true; dir grants S only
  end;

  rule "L1I clean evict IC_S -> PutS(icache) -> IC_SI_A"
    (IProcs[c].state = IC_S) & DirStable()
  ==> begin
    IProcs[c].state := IC_SI_A;                          -- wait WBAck (cannot re-fetch meanwhile)
    SendI(PutS, HomeType, c, 0, 0, Gnt_S, false, 0, c);
  end;

endruleset;

-- ============================================================================
-- Directory: handle one request (vc=0) at a time, only when stable (§9.5)
-- ============================================================================

ruleset r: Core do
choose midx: Net[HomeType] do
rule "dir handle request"
  DirStable()
  & (Net[HomeType][midx].vc = 0)
  & (Net[HomeType][midx].src = r)
==>
var nshare: CountType;
begin
  alias m: Net[HomeType][midx] do
  alias H: HomeNode do
  switch m.op

  case GetS:
    -- v1: every grant goes through a wait-for-Unblock transient (full per-line
    -- serialisation), so no forward can target a requester mid-acquire.
    H.pendReq := r; H.pendEgrant := false; undefine H.pendOwner;
    H.pendIcache := m.is_icache;                    -- v4a: remember the requester's cache for finalize
    switch H.state
      case DIR_I:
        if m.is_icache then                         -- an I-cache fetch NEVER gets E
          SendI(Data, r, HomeType, 2, H.val, Gnt_S, false, 0, r);
        else
          H.pendEgrant := true;                     -- exclusive-on-miss: grant E (L1D only)
          Send(Data, r, HomeType, 2, H.val, Gnt_E, false, 0, r);
        endif;
      case DIR_S:
        if m.is_icache then SendI(Data, r, HomeType, 2, H.val, Gnt_S, false, 0, r);
        else                Send (Data, r, HomeType, 2, H.val, Gnt_S, false, 0, r); endif;
      case DIR_EM:
        H.pendOwner := H.owner;                     -- FwdGetS carries is_icache so the owner
        SendG(FwdGetS, H.owner, HomeType, 1, 0, Gnt_S, false, 0, r, m.is_icache); -- tags its Data
      case DIR_O:
        H.pendOwner := H.owner;
        SendG(FwdGetS, H.owner, HomeType, 1, 0, Gnt_S, false, 0, r, m.is_icache);
    endswitch;
    H.state := DIR_S_D;
    MultiSetRemove(midx, Net[HomeType]);

  case GetM:
    -- (GetM is always an L1D op.) Invalidate every OTHER L1D sharer AND every L1I sharer
    -- (incl. the requester's own L1I — self-CMODX, D2). ack_count = popcount over both.
    H.pendReq := r; undefine H.pendOwner;
    switch H.state
      case DIR_I:
        Send(Data, r, HomeType, 2, H.val, Gnt_M, false, 0, r);
      case DIR_S:
        nshare := OtherSharersBoth(r);
        for c: Core do
          if H.d_sharers[c] & (c != r) then Send (Inv, c, HomeType, 1, 0, Gnt_S, false, 0, r); endif;
          if H.i_sharers[c]               then SendI(Inv, c, HomeType, 1, 0, Gnt_S, false, 0, r); endif;
        endfor;
        Send(Data, r, HomeType, 2, H.val, Gnt_M, false, nshare, r);
      case DIR_EM:   -- exclusive: no other sharers in either vector (SWMR), ack=0
        H.pendOwner := H.owner;
        Send(FwdGetM, H.owner, HomeType, 1, 0, Gnt_M, false, 0, r);
      case DIR_O:
        H.pendOwner := H.owner;
        nshare := 0;
        for c: Core do
          if H.d_sharers[c] & (c != r) & (c != H.owner) then
            Send(Inv, c, HomeType, 1, 0, Gnt_S, false, 0, r); nshare := nshare + 1;
          endif;
          if H.i_sharers[c] then
            SendI(Inv, c, HomeType, 1, 0, Gnt_S, false, 0, r); nshare := nshare + 1;
          endif;
        endfor;
        Send(FwdGetM, H.owner, HomeType, 1, 0, Gnt_M, false, nshare, r);
    endswitch;
    H.state := DIR_M_D;
    MultiSetRemove(midx, Net[HomeType]);

  case Upgrade:
    H.pendReq := r; undefine H.pendOwner;
    -- (R-b) if r still holds a copy -> no-data grant (invalidate every other sharer,
    -- including the O-owner since r already has the clean value); else it lost its copy
    -- (demoted SM_AD->IM_AD) -> serve it data exactly like a GetM.
    if ((H.state = DIR_S) | (H.state = DIR_O)) & H.d_sharers[r] then
      nshare := OtherSharersBoth(r);
      for c: Core do
        if H.d_sharers[c] & (c != r) then Send (Inv, c, HomeType, 1, 0, Gnt_S, false, 0, r); endif;
        if H.i_sharers[c]               then SendI(Inv, c, HomeType, 1, 0, Gnt_S, false, 0, r); endif;
      endfor;
      Send(Data, r, HomeType, 2, H.val, Gnt_M, false, nshare, r);   -- ack_count; r keeps own data
      H.state := DIR_M_D;
    else
      switch H.state
        case DIR_I:
          Send(Data, r, HomeType, 2, H.val, Gnt_M, false, 0, r);
        case DIR_S:
          nshare := OtherSharersBoth(r);
          for c: Core do
            if H.d_sharers[c] & (c != r) then Send (Inv, c, HomeType, 1, 0, Gnt_S, false, 0, r); endif;
            if H.i_sharers[c]               then SendI(Inv, c, HomeType, 1, 0, Gnt_S, false, 0, r); endif;
          endfor;
          Send(Data, r, HomeType, 2, H.val, Gnt_M, false, nshare, r);
        case DIR_EM:
          H.pendOwner := H.owner;
          Send(FwdGetM, H.owner, HomeType, 1, 0, Gnt_M, false, 0, r);
        case DIR_O:
          H.pendOwner := H.owner;
          nshare := 0;
          for c: Core do
            if H.d_sharers[c] & (c != r) & (c != H.owner) then
              Send(Inv, c, HomeType, 1, 0, Gnt_S, false, 0, r); nshare := nshare + 1;
            endif;
            if H.i_sharers[c] then
              SendI(Inv, c, HomeType, 1, 0, Gnt_S, false, 0, r); nshare := nshare + 1;
            endif;
          endfor;
          Send(FwdGetM, H.owner, HomeType, 1, 0, Gnt_M, false, nshare, r);
      endswitch;
      H.state := DIR_M_D;
    endif;
    MultiSetRemove(midx, Net[HomeType]);

  case PutM:
    if (H.state = DIR_EM) & (H.owner = r) then
      H.val := m.val; H.state := DIR_I; undefine H.owner; ClearSharers();
    else
      H.d_sharers[r] := false;          -- stale/late evict (owner changed by a race): just clear r
    endif;
    Send(WBAck, r, HomeType, 2, 0, Gnt_S, false, 0, r);
    MultiSetRemove(midx, Net[HomeType]);

  case PutO:
    if (H.state = DIR_O) & (H.owner = r) then
      H.val := m.val; H.d_sharers[r] := false; undefine H.owner;
      if AllSharersEmpty() then H.state := DIR_I; else H.state := DIR_S; endif;
    else
      H.d_sharers[r] := false;
    endif;
    Send(WBAck, r, HomeType, 2, 0, Gnt_S, false, 0, r);
    MultiSetRemove(midx, Net[HomeType]);

  case PutE:
    if (H.state = DIR_EM) & (H.owner = r) then
      H.state := DIR_I; undefine H.owner; ClearSharers();
    else
      H.d_sharers[r] := false;
    endif;
    Send(WBAck, r, HomeType, 2, 0, Gnt_S, false, 0, r);
    MultiSetRemove(midx, Net[HomeType]);

  case PutS:
    -- v4a: clear the right vector; WBAck the evictor (its L1I IC_SI_A / L1D SI_A waits).
    if m.is_icache then H.i_sharers[r] := false; else H.d_sharers[r] := false; endif;
    if (H.state = DIR_S) & AllSharersEmpty() then H.state := DIR_I; endif;
    if m.is_icache then SendI(WBAck, r, HomeType, 2, 0, Gnt_S, false, 0, r);
    else                Send (WBAck, r, HomeType, 2, 0, Gnt_S, false, 0, r); endif;
    MultiSetRemove(midx, Net[HomeType]);

  endswitch;
  endalias; endalias;
end;
endchoose;
endruleset;

-- ============================================================================
-- Directory: finalize a transaction on Unblock from the requester (any dir state)
-- ============================================================================

choose midx: Net[HomeType] do
rule "dir handle Unblock"
  (Net[HomeType][midx].op = Unblock)
==>
begin
  alias m: Net[HomeType][midx] do
  alias H: HomeNode do
  if H.state = DIR_S_D then
    -- add the requester to the correct presence vector (D2): i_sharers if it was an L1I fetch.
    if H.pendIcache then H.i_sharers[m.src] := true; else H.d_sharers[m.src] := true; endif;
    if !isundefined(H.pendOwner) then
      -- cache-to-cache read: owner W (always an L1D) forwarded the data to R (= m.src)
      if m.ostays then
        H.d_sharers[H.pendOwner] := true;        -- W kept dirty data + ownership (M->O)
        H.owner := H.pendOwner; H.state := DIR_O;
      else
        -- W relinquished (E->S keeps its bit; an evicting owner's Put clears it later).
        -- Refresh L2 from the forwarded value (no dirty owner -> dir/L2 must hold current).
        H.val := m.val;
        undefine H.owner; H.state := DIR_S;
      endif;
    elsif H.pendEgrant then                        -- fresh exclusive grant (L1D only; icache never E)
      ClearSharers(); H.d_sharers[m.src] := true;
      H.owner := m.src; H.state := DIR_EM;
    else
      H.state := DIR_S;                            -- fresh shared grant from L2 (vector set above)
    endif;
  elsif H.state = DIR_M_D then
    ClearSharers();
    H.owner := m.src; H.d_sharers[m.src] := true; H.state := DIR_EM;
  endif;
  endalias; endalias;
  MultiSetRemove(midx, Net[HomeType]);
end;
endchoose;

-- ============================================================================
-- Core: handle a forwarded snoop (vc=1) — owner / sharer side
-- ============================================================================

ruleset c: Core do
choose midx: Net[c] do
rule "core handle Fwd/Inv"
  (Net[c][midx].vc = 1)
==>
begin
  alias m: Net[c][midx] do
  -- Routing: an Inv tagged is_icache targets this core's L1I. A FwdGetS/FwdGetM ALWAYS
  -- targets the L1D owner — its is_icache flag means "the REQUESTER is an L1I" (so the
  -- owner echoes it onto the forwarded Data), NOT "deliver to the L1I".
  if (m.op = Inv) & m.is_icache then
    alias IP: IProcs[c] do
    -- the InvAck goes to the L1D requester (is_icache=false).
    switch IP.state
      case IC_S:    Send(InvAck, m.req, c, 2, 0, Gnt_S, false, 0, c); IP.state := IC_I;
      case IC_I:    Send(InvAck, m.req, c, 2, 0, Gnt_S, false, 0, c);                  -- race: ack anyway
      case IC_SI_A: Send(InvAck, m.req, c, 2, 0, Gnt_S, false, 0, c);                  -- evicting: ack, stay (WBAck -> IC_I)
      case IC_IS_D: error "L1I Inv during IC_IS_D (cannot happen: serialised dir is busy)";
    endswitch;
    endalias;
  else
  alias P: Procs[c] do
  switch m.op

  case FwdGetS:
    -- the owner only forwards data (with the owner_stays bit) + downgrades its own
    -- state; the REQUESTER sends the single Unblock (echoing owner_stays + the value).
    switch P.state
      case C_E:    -- clean exclusive: supply clean data, relinquish (E->S, owner gone)
        SendG(Data, m.req, c, 2, P.val, Gnt_S, false, 0, m.req, m.is_icache);
        P.state := C_S;
      case C_M:    -- keep dirty data + ownership (M->O)
        SendG(Data, m.req, c, 2, P.val, Gnt_S, true, 0, m.req, m.is_icache);
        P.state := C_O;
      case C_O:    -- stay O
        SendG(Data, m.req, c, 2, P.val, Gnt_S, true, 0, m.req, m.is_icache);
      case OM_A:   -- owner mid-Upgrade: supply data, stay O-ish (its later Upgrade Inv's the new reader)
        SendG(Data, m.req, c, 2, P.val, Gnt_S, true, 0, m.req, m.is_icache);
      case MI_A:   -- WB-vs-Fwd race (B5): supply data, relinquish; PutM still writes L2
        SendG(Data, m.req, c, 2, P.val, Gnt_S, false, 0, m.req, m.is_icache);
      case OI_A:
        SendG(Data, m.req, c, 2, P.val, Gnt_S, false, 0, m.req, m.is_icache);
      case EI_A:   -- B11 fix: still supply the clean data, then -> II_A
        SendG(Data, m.req, c, 2, P.val, Gnt_S, false, 0, m.req, m.is_icache);
        P.state := II_A;
      else
        error "FwdGetS to a non-owner state";
    endswitch;

  case FwdGetM:
    -- propagate the ack_count the dir stamped on the FwdGetM onto the Data the owner
    -- forwards, so the requester waits for the OTHER d_sharers' Inv-Acks before reaching M.
    switch P.state
      case C_E:  SendG(Data, m.req, c, 2, P.val, Gnt_M, false, m.acks, m.req, m.is_icache); P.state := C_I;
      case C_M:  SendG(Data, m.req, c, 2, P.val, Gnt_M, false, m.acks, m.req, m.is_icache); P.state := C_I;
      case C_O:  SendG(Data, m.req, c, 2, P.val, Gnt_M, false, m.acks, m.req, m.is_icache); P.state := C_I;
      case OM_A: SendG(Data, m.req, c, 2, P.val, Gnt_M, false, m.acks, m.req, m.is_icache); P.state := IM_AD;
                 -- owner mid-Upgrade loses to a peer writer: relinquish + re-acquire (->IM_AD)
      case MI_A: SendG(Data, m.req, c, 2, P.val, Gnt_M, false, m.acks, m.req, m.is_icache); P.state := II_A;
      case OI_A: SendG(Data, m.req, c, 2, P.val, Gnt_M, false, m.acks, m.req, m.is_icache); P.state := II_A;
      case EI_A: SendG(Data, m.req, c, 2, P.val, Gnt_M, false, m.acks, m.req, m.is_icache); P.state := II_A;
      else
        error "FwdGetM to a non-owner state";
    endswitch;

  case Inv:
    switch P.state
      case C_S:   Send(InvAck, m.req, c, 2, 0, Gnt_S, false, 0, c); P.state := C_I;
      case C_O:   Send(InvAck, m.req, c, 2, 0, Gnt_S, false, 0, c); P.state := C_I;
                  -- O-owner invalidated by a clean-sharer Upgrade: requester holds the
                  -- same clean value, so discarding this (dirty) copy is value-safe.
      case C_I:   Send(InvAck, m.req, c, 2, 0, Gnt_S, false, 0, c);          -- race; ack anyway
      case SM_AD: Send(InvAck, m.req, c, 2, 0, Gnt_S, false, 0, c); P.state := IM_AD; -- demote
      case OM_A:  Send(InvAck, m.req, c, 2, 0, Gnt_S, false, 0, c); P.state := IM_AD; -- demote
      case SI_A:  Send(InvAck, m.req, c, 2, 0, Gnt_S, false, 0, c); P.state := II_A;
      case MI_A:  Send(InvAck, m.req, c, 2, 0, Gnt_S, false, 0, c); P.state := II_A;
                  -- phantom sharer: an owner that forwarded its data (ostays=0) and is
                  -- mid-PutM is still listed as a sharer until its PutM lands; ack the Inv.
      case OI_A:  Send(InvAck, m.req, c, 2, 0, Gnt_S, false, 0, c); P.state := II_A;
      case EI_A:  Send(InvAck, m.req, c, 2, 0, Gnt_S, false, 0, c); P.state := II_A;
      case II_A:  Send(InvAck, m.req, c, 2, 0, Gnt_S, false, 0, c);  -- phantom sharer already ->I-bound; ack
      else
        error "Inv to an unexpected state";
    endswitch;

  endswitch;
  endalias;        -- P (L1D)
  endif;           -- is_icache
  endalias;        -- m
  MultiSetRemove(midx, Net[c]);
end;
endchoose;
endruleset;

-- ============================================================================
-- Core: handle a response to itself (vc=2) — Data / InvAck / WBAck
-- ============================================================================

ruleset c: Core do
choose midx: Net[c] do
rule "core handle response"
  (Net[c][midx].vc = 2)
==>
begin
  alias m: Net[c][midx] do
  -- An is_icache Data (grant) or WBAck (evict-complete) is for the L1I. An InvAck the L1I
  -- SENT also carries is_icache but targets the requester's L1D — so route by op too.
  if ((m.op = Data) | (m.op = WBAck)) & m.is_icache then
    alias IP: IProcs[c] do
    if m.op = Data then                                -- a Data grant (always gst=S): IC_IS_D -> IC_S
      assert (m.gst = Gnt_S) "L1I granted a non-S state";
      IP.val := m.val; IP.state := IC_S;
      Send(Unblock, HomeType, c, 2, IP.val, Gnt_S, m.ostays, 0, c);  -- echo ostays (cache-to-cache)
    else                                               -- WBAck: IC_SI_A -> IC_I (evict complete)
      IP.state := IC_I;
    endif;
    endalias;
  else
  alias P: Procs[c] do
  switch m.op

  case Data:
    switch P.state
      case IS_D:
        P.val := m.val;
        if m.gst = Gnt_E then P.state := C_E; else P.state := C_S; endif;
        -- echo owner_stays + the value, so the dir can refresh L2 if the owner relinquished
        Send(Unblock, HomeType, c, 2, P.val, m.gst, m.ostays, 0, c);
      case IM_AD:
        P.val := m.val; P.acks := P.acks + m.acks;
        if P.acks = 0 then
          P.state := C_M; P.val := 1 - P.val; LastData := P.val;
          Send(Unblock, HomeType, c, 2, 0, Gnt_M, false, 0, c);
        else
          P.state := IM_A;
        endif;
      case SM_AD:       -- no-data grant: keep own val, just collect acks
        P.acks := P.acks + m.acks;
        if P.acks = 0 then
          P.state := C_M; P.val := 1 - P.val; LastData := P.val;
          Send(Unblock, HomeType, c, 2, 0, Gnt_M, false, 0, c);
        else
          P.state := IM_A;
        endif;
      case OM_A:
        P.acks := P.acks + m.acks;
        if P.acks = 0 then
          P.state := C_M; P.val := 1 - P.val; LastData := P.val;
          Send(Unblock, HomeType, c, 2, 0, Gnt_M, false, 0, c);
        else
          P.state := IM_A;
        endif;
      else
        error "Data in unexpected state";
    endswitch;

  case InvAck:
    P.acks := P.acks - 1;
    if (P.state = IM_A) & (P.acks = 0) then
      P.state := C_M; P.val := 1 - P.val; LastData := P.val;
      Send(Unblock, HomeType, c, 2, 0, Gnt_M, false, 0, c);
    endif;

  case WBAck:
    switch P.state
      case MI_A: P.state := C_I;
      case OI_A: P.state := C_I;
      case EI_A: P.state := C_I;
      case SI_A: P.state := C_I;
      case II_A: P.state := C_I;
      else
        error "WBAck in unexpected state";
    endswitch;

  else
    error "unexpected response op";
  endswitch;
  endalias;        -- P (L1D)
  endif;           -- is_icache
  endalias;        -- m
  MultiSetRemove(midx, Net[c]);
end;
endchoose;
endruleset;

-- ============================================================================
-- Startstate
-- ============================================================================

startstate
begin
  for c: Core do
    Procs[c].state := C_I; Procs[c].val := 0; Procs[c].acks := 0;
    IProcs[c].state := IC_I; IProcs[c].val := 0;   -- L1I starts invalid
  endfor;
  HomeNode.state := DIR_I;
  undefine HomeNode.owner;
  undefine HomeNode.pendReq;
  undefine HomeNode.pendOwner;
  HomeNode.pendEgrant := false;
  HomeNode.pendIcache := false;
  ClearSharers();                                   -- clears BOTH d_sharers and i_sharers
  HomeNode.val := 0;
  LastData := 0;
  undefine Net;
end;

-- ============================================================================
-- Invariants
-- ============================================================================

invariant "single owner: <=1 core in M/O/E"
  forall c1: Core do forall c2: Core do
    (c1 != c2 & IsOwnerState(Procs[c1].state) & IsOwnerState(Procs[c2].state)) -> false
  endforall endforall;

invariant "SWMR: an exclusive M/E excludes any other valid sharer"
  forall c1: Core do forall c2: Core do
    (c1 != c2 & (Procs[c1].state = C_M | Procs[c1].state = C_E)
              & IsValidStable(Procs[c2].state)) -> false
  endforall endforall;

invariant "data: every valid L1D copy holds the current value"
  forall c: Core do
    IsValidStable(Procs[c].state) -> (Procs[c].val = LastData)
  endforall;

-- v4a / D2: the L1I is part of coherence — an S code copy must also be current.
invariant "data: every valid L1I copy holds the current value"
  forall c: Core do
    (IProcs[c].state = IC_S) -> (IProcs[c].val = LastData)
  endforall;

-- v4a / D2: SWMR must exclude L1I sharers too — an exclusive M/E L1D tolerates NO L1I copy
-- (anywhere, incl. the writer's own core — the self-CMODX Inv).
invariant "SWMR also excludes L1I sharers"
  forall c1: Core do forall c2: Core do
    ((Procs[c1].state = C_M | Procs[c1].state = C_E) & (IProcs[c2].state = IC_S)) -> false
  endforall endforall;

invariant "memory correct when the line is uncached (DIR_I)"
  (HomeNode.state = DIR_I) -> (HomeNode.val = LastData);

invariant "directory owner is a core in EM/O"
  (HomeNode.state = DIR_EM | HomeNode.state = DIR_O)
    -> (!isundefined(HomeNode.owner) & ismember(HomeNode.owner, Core));
