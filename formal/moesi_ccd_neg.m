-- ============================================================================
-- moesi_ccd_neg.m  --  NEGATIVE CONTROL of the MOESI model
-- (identical to moesi_ccd.m except the Inv-Ack accumulation is removed, so a GetM
--  requester reaches M without waiting for the other sharers to invalidate. The
--  SWMR invariant MUST fail — this proves the invariants in moesi_ccd.m are not
--  vacuous. Expected: "SWMR ... failed" at ~887 states.)
--
-- (original header below)
-- moesi_ccd.m  --  CMurphi formal model of the niigo-lake 4-core MOESI protocol
--
-- Models plans/multicore-ccd.md §9 (L1D/directory transition tables) + §13.9
-- (NINE data-source priority, D6 ack-to-requester, noisy E/S eviction).
--
-- Scope (v1):  N L1D caches + 1 directory + NINE L2/memory, ONE address,
--   data-value tracking.  The directory serialises per line: a 2nd request to a
--   busy line waits (§9.5).  L1I, LR/SC, AMO and speculative loads are deferred
--   to v2 (orthogonal; would balloon the state space without new MOESI races).
--   Because the directory serialises, a requester in an acquire transient
--   (IS_D/IM_AD/SM_AD) never receives a conflicting snoop, so those deferral
--   rows of §9.3 are not exercised in v1 (they need concurrent same-line txns).
--
-- Two race resolutions this model PINS (feedback to §9.5/§9.6 — they were
-- under-specified for formal modelling):
--   (R-a) FwdGetS data carries an `owner_stays` bit: M/O owner keeps ownership
--         (->O), an E owner or an evicting owner relinquishes (->S/->I). The
--         requester echoes it in Unblock so the directory resolves EM/O -> O vs S
--         WITHOUT having to distinguish E from M (which it structurally cannot).
--   (R-b) A GetM/Upgrade requester that lost its S copy to a prior txn's Inv is
--         detected at the directory by "requester is not a current sharer" and
--         is served data (treated as a GetM), matching the §9.3 SM_AD->IM_AD
--         demotion. No extra message needed.
--
-- Checks:  SWMR coherence, single-owner, data-value correctness, memory
--   correctness (DIR_I), directory-owner consistency, and (Murphi built-in)
--   deadlock freedom of the acyclic-VC message dependence.
--
-- RESULT (CMurphi 5.4.9):  No error found, deadlock-free —
--   NumCores=3 : 66,328 states / 190,772 rules (~0.6 s)
--   NumCores=4 : 872,587 states / 3,082,542 rules (~12 s)
--   Negative control (moesi_ccd_neg.m, ack-wait removed): SWMR violation caught
--   at 887 states — confirms the invariants are not vacuous.
-- ============================================================================

const
  NumCores: 3;           -- 3 fully exercises the protocol (owner + 2 sharers); 4 also verified
  NetMax:   18;          -- per-node message-buffer bound

type
  Core:  scalarset(NumCores);
  Home:  enum { HomeType };
  Node:  union { Home, Core };

  Value: 0..1;           -- data values (2 enough to expose corruption)

  CountType: 0..NumCores;          -- sharer counts / ack_count (named so codegen types match)
  AckType:   -NumCores..NumCores;  -- remaining InvAcks (signed: an ack may precede the count)

  VCType: 0..2;          -- 0 = Req (C0), 1 = Fwd (C1), 2 = Resp (C2/C3/C4)

  GrantState: enum { Gnt_S, Gnt_E, Gnt_M };

  MsgOp: enum {
    GetS, GetM, Upgrade, PutM, PutO, PutS, PutE,   -- C0 request  (core -> home)
    FwdGetS, FwdGetM, Inv,                          -- C1 snoop     (home -> core)
    Data,                                           -- C2 data      (home/owner -> core)
    InvAck, Unblock, WBAck                          -- C4 acks      (-> requester / home / core)
  };

  CacheState: enum {
    C_I, C_S, C_E, C_O, C_M,            -- stable MOESI
    IS_D, IM_AD, IM_A, SM_AD, OM_A,     -- acquire transients
    MI_A, OI_A, EI_A, SI_A, II_A        -- evict transients
  };

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
  end;

  ProcRec: record
    state: CacheState;
    val:   Value;
    acks:  AckType;         -- remaining InvAcks (signed: an ack may precede the count)
  end;

  HomeRec: record
    state:   DirState;
    owner:   Node;                   -- valid in EM/O (a Core); owner is also in `sharers`
    sharers: array[Core] of boolean; -- d_sharers (L1D); v1 has no L1I
    val:     Value;                  -- L2/memory backing copy (the always-present source)
    pendReq: Node;                   -- requester being served in S_D/M_D
    pendOwner: Node;                 -- the owner forwarded-to (S_D); undefined for an L2-sourced grant
    pendEgrant: boolean;             -- a fresh L2 GetS grant was Exclusive (-> EM on Unblock)
  end;

var
  Procs:    array[Core] of ProcRec;
  HomeNode: HomeRec;
  Net:      array[Node] of multiset[NetMax] of Message;
  LastData: Value;                   -- auxiliary: globally last-written value

-- ============================================================================
-- Helpers
-- ============================================================================

procedure Send(op: MsgOp; dst: Node; src: Node; vc: VCType;
               val: Value; gst: GrantState; ostays: boolean;
               acks: CountType; req: Node);
var m: Message;
begin
  assert (MultiSetCount(i: Net[dst], true) < NetMax) "network buffer overflow";
  m.op := op; m.src := src; m.dst := dst; m.vc := vc;
  m.val := val; m.gst := gst; m.ostays := ostays; m.acks := acks; m.req := req;
  MultiSetAdd(m, Net[dst]);
end;

function OtherSharers(r: Core): CountType;   -- |sharers \ {r}| (the ack_count, D6)
var n: CountType;
begin
  n := 0;
  for c: Core do
    if HomeNode.sharers[c] & (c != r) then n := n + 1; endif;
  endfor;
  return n;
end;

function SharerCount(): CountType;
var n: CountType;
begin
  n := 0;
  for c: Core do
    if HomeNode.sharers[c] then n := n + 1; endif;
  endfor;
  return n;
end;

procedure ClearSharers();
begin
  for c: Core do HomeNode.sharers[c] := false; endfor;
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
    switch H.state
      case DIR_I:
        H.pendEgrant := true;                       -- exclusive-on-miss: grant E
        Send(Data, r, HomeType, 2, H.val, Gnt_E, false, 0, r);
      case DIR_S:
        Send(Data, r, HomeType, 2, H.val, Gnt_S, false, 0, r);
      case DIR_EM:
        H.pendOwner := H.owner;
        Send(FwdGetS, H.owner, HomeType, 1, 0, Gnt_S, false, 0, r);
      case DIR_O:
        H.pendOwner := H.owner;
        Send(FwdGetS, H.owner, HomeType, 1, 0, Gnt_S, false, 0, r);
    endswitch;
    H.state := DIR_S_D;
    MultiSetRemove(midx, Net[HomeType]);

  case GetM:
    H.pendReq := r; undefine H.pendOwner;
    switch H.state
      case DIR_I:
        Send(Data, r, HomeType, 2, H.val, Gnt_M, false, 0, r);
      case DIR_S:
        nshare := OtherSharers(r);
        for c: Core do
          if H.sharers[c] & (c != r) then
            Send(Inv, c, HomeType, 1, 0, Gnt_S, false, 0, r);
          endif;
        endfor;
        Send(Data, r, HomeType, 2, H.val, Gnt_M, false, nshare, r);
      case DIR_EM:
        H.pendOwner := H.owner;
        Send(FwdGetM, H.owner, HomeType, 1, 0, Gnt_M, false, 0, r);
      case DIR_O:
        H.pendOwner := H.owner;
        nshare := 0;
        for c: Core do
          if H.sharers[c] & (c != r) & (c != H.owner) then
            Send(Inv, c, HomeType, 1, 0, Gnt_S, false, 0, r);
            nshare := nshare + 1;
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
    if ((H.state = DIR_S) | (H.state = DIR_O)) & H.sharers[r] then
      nshare := OtherSharers(r);
      for c: Core do
        if H.sharers[c] & (c != r) then
          Send(Inv, c, HomeType, 1, 0, Gnt_S, false, 0, r);
        endif;
      endfor;
      Send(Data, r, HomeType, 2, H.val, Gnt_M, false, nshare, r);   -- ack_count; r keeps own data
      H.state := DIR_M_D;
    else
      switch H.state
        case DIR_I:
          Send(Data, r, HomeType, 2, H.val, Gnt_M, false, 0, r);
        case DIR_S:
          nshare := OtherSharers(r);
          for c: Core do
            if H.sharers[c] & (c != r) then
              Send(Inv, c, HomeType, 1, 0, Gnt_S, false, 0, r);
            endif;
          endfor;
          Send(Data, r, HomeType, 2, H.val, Gnt_M, false, nshare, r);
        case DIR_EM:
          H.pendOwner := H.owner;
          Send(FwdGetM, H.owner, HomeType, 1, 0, Gnt_M, false, 0, r);
        case DIR_O:
          H.pendOwner := H.owner;
          nshare := 0;
          for c: Core do
            if H.sharers[c] & (c != r) & (c != H.owner) then
              Send(Inv, c, HomeType, 1, 0, Gnt_S, false, 0, r);
              nshare := nshare + 1;
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
      H.sharers[r] := false;          -- stale/late evict (owner changed by a race): just clear r
    endif;
    Send(WBAck, r, HomeType, 2, 0, Gnt_S, false, 0, r);
    MultiSetRemove(midx, Net[HomeType]);

  case PutO:
    if (H.state = DIR_O) & (H.owner = r) then
      H.val := m.val; H.sharers[r] := false; undefine H.owner;
      if SharerCount() = 0 then H.state := DIR_I; else H.state := DIR_S; endif;
    else
      H.sharers[r] := false;
    endif;
    Send(WBAck, r, HomeType, 2, 0, Gnt_S, false, 0, r);
    MultiSetRemove(midx, Net[HomeType]);

  case PutE:
    if (H.state = DIR_EM) & (H.owner = r) then
      H.state := DIR_I; undefine H.owner; ClearSharers();
    else
      H.sharers[r] := false;
    endif;
    Send(WBAck, r, HomeType, 2, 0, Gnt_S, false, 0, r);
    MultiSetRemove(midx, Net[HomeType]);

  case PutS:
    H.sharers[r] := false;
    if (H.state = DIR_S) & (SharerCount() = 0) then H.state := DIR_I; endif;
    Send(WBAck, r, HomeType, 2, 0, Gnt_S, false, 0, r);
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
    if !isundefined(H.pendOwner) then
      -- cache-to-cache read: owner W forwarded the data to R (= m.src)
      H.sharers[m.src] := true;
      if m.ostays then
        H.sharers[H.pendOwner] := true;        -- W kept dirty data + ownership (M->O)
        H.owner := H.pendOwner; H.state := DIR_O;
      else
        -- W relinquished (E->S keeps its bit; an evicting owner's Put clears it later).
        -- Refresh L2 from the forwarded value: when there is no dirty owner, the dir/L2
        -- must hold the current value (the evicting owner's WB may not have landed yet).
        H.val := m.val;
        undefine H.owner; H.state := DIR_S;
      endif;
    elsif H.pendEgrant then
      ClearSharers(); H.sharers[m.src] := true;  -- fresh exclusive grant from L2
      H.owner := m.src; H.state := DIR_EM;
    else
      H.sharers[m.src] := true; H.state := DIR_S; -- fresh shared grant from L2
    endif;
  elsif H.state = DIR_M_D then
    ClearSharers();
    H.owner := m.src; H.sharers[m.src] := true; H.state := DIR_EM;
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
  alias P: Procs[c] do
  switch m.op

  case FwdGetS:
    -- the owner only forwards data (with the owner_stays bit) + downgrades its own
    -- state; the REQUESTER sends the single Unblock (echoing owner_stays + the value).
    switch P.state
      case C_E:    -- clean exclusive: supply clean data, relinquish (E->S, owner gone)
        Send(Data, m.req, c, 2, P.val, Gnt_S, false, 0, m.req);
        P.state := C_S;
      case C_M:    -- keep dirty data + ownership (M->O)
        Send(Data, m.req, c, 2, P.val, Gnt_S, true, 0, m.req);
        P.state := C_O;
      case C_O:    -- stay O
        Send(Data, m.req, c, 2, P.val, Gnt_S, true, 0, m.req);
      case OM_A:   -- owner mid-Upgrade: supply data, stay O-ish (its later Upgrade Inv's the new reader)
        Send(Data, m.req, c, 2, P.val, Gnt_S, true, 0, m.req);
      case MI_A:   -- WB-vs-Fwd race (B5): supply data, relinquish; PutM still writes L2
        Send(Data, m.req, c, 2, P.val, Gnt_S, false, 0, m.req);
      case OI_A:
        Send(Data, m.req, c, 2, P.val, Gnt_S, false, 0, m.req);
      case EI_A:   -- B11 fix: still supply the clean data, then -> II_A
        Send(Data, m.req, c, 2, P.val, Gnt_S, false, 0, m.req);
        P.state := II_A;
      else
        error "FwdGetS to a non-owner state";
    endswitch;

  case FwdGetM:
    -- propagate the ack_count the dir stamped on the FwdGetM onto the Data the owner
    -- forwards, so the requester waits for the OTHER sharers' Inv-Acks before reaching M.
    switch P.state
      case C_E:  Send(Data, m.req, c, 2, P.val, Gnt_M, false, m.acks, m.req); P.state := C_I;
      case C_M:  Send(Data, m.req, c, 2, P.val, Gnt_M, false, m.acks, m.req); P.state := C_I;
      case C_O:  Send(Data, m.req, c, 2, P.val, Gnt_M, false, m.acks, m.req); P.state := C_I;
      case OM_A: Send(Data, m.req, c, 2, P.val, Gnt_M, false, m.acks, m.req); P.state := IM_AD;
                 -- owner mid-Upgrade loses to a peer writer: relinquish + re-acquire (->IM_AD)
      case MI_A: Send(Data, m.req, c, 2, P.val, Gnt_M, false, m.acks, m.req); P.state := II_A;
      case OI_A: Send(Data, m.req, c, 2, P.val, Gnt_M, false, m.acks, m.req); P.state := II_A;
      case EI_A: Send(Data, m.req, c, 2, P.val, Gnt_M, false, m.acks, m.req); P.state := II_A;
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
  endalias; endalias;
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
        P.val := m.val; P.acks := P.acks + 0; -- NEG CONTROL: ignore ack_count (was + m.acks)
        if P.acks = 0 then
          P.state := C_M; P.val := 1 - P.val; LastData := P.val;
          Send(Unblock, HomeType, c, 2, 0, Gnt_M, false, 0, c);
        else
          P.state := IM_A;
        endif;
      case SM_AD:       -- no-data grant: keep own val, just collect acks
        P.acks := P.acks + 0; -- NEG CONTROL: ignore ack_count (was + m.acks)
        if P.acks = 0 then
          P.state := C_M; P.val := 1 - P.val; LastData := P.val;
          Send(Unblock, HomeType, c, 2, 0, Gnt_M, false, 0, c);
        else
          P.state := IM_A;
        endif;
      case OM_A:
        P.acks := P.acks + 0; -- NEG CONTROL: ignore ack_count (was + m.acks)
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
  endalias; endalias;
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
  endfor;
  HomeNode.state := DIR_I;
  undefine HomeNode.owner;
  undefine HomeNode.pendReq;
  undefine HomeNode.pendOwner;
  HomeNode.pendEgrant := false;
  ClearSharers();
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

invariant "data: every valid copy holds the current value"
  forall c: Core do
    IsValidStable(Procs[c].state) -> (Procs[c].val = LastData)
  endforall;

invariant "memory correct when the line is uncached (DIR_I)"
  (HomeNode.state = DIR_I) -> (HomeNode.val = LastData);

invariant "directory owner is a core in EM/O"
  (HomeNode.state = DIR_EM | HomeNode.state = DIR_O)
    -> (!isundefined(HomeNode.owner) & ismember(HomeNode.owner, Core));
