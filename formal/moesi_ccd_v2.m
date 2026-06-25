-- ============================================================================
-- moesi_ccd.m  --  CMurphi formal model of the niigo-lake 4-core MOESI protocol
--
-- Models plans/multicore-ccd.md §9 (L1D/directory transition tables) + §13.9
-- (NINE data-source priority, D6 ack-to-requester, noisy E/S eviction).
--
-- Scope (v2):  N L1D caches + 1 directory + NINE L2/memory, ONE address,
--   data-value tracking.  *** v2 adds the ACQUIRE-SIDE DEFERRED-SNOOP MATRIX ***:
--   the directory is GRANT-AND-GO for L2-sourced grants (DIR_I/DIR_S reads, writes,
--   sharer-Upgrades) — it returns to a stable state immediately, so the NEXT
--   transaction can target a requester that is still mid-acquire. That requester
--   (in IS_D/IM_AD/IM_A/SM_AD/OM_A) DEFERS the snoop in its MSHR and services it on
--   reaching its stable state (the §9.3 "defer" rows). Cache-to-cache forwarding
--   from a stable owner still uses S_D/M_D + Unblock. The ack-before-write rule (a
--   writer reaches M only after every Inv-Ack) keeps data sound across the window.
--   Deferred to v3: L1I agent (D2 split tracking), LR/SC, AMO, multi-address.
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
--   NumCores=3 : 83,049 states / 165,277 rules (~0.5 s)
--   NumCores=4 : 4,707,141 states / 10,065,162 rules (~40 s, ~14 GB)
--   Coverage probe (assert "no snoop ever deferred") FAILS at 41 states — i.e. the
--   acquire-side deferral path is genuinely exercised, not vacuously absent.
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
    -- one deferred snoop (the MSHR's sticky deferred-snoop slot, §9.0/§9.3): a snoop
    -- that arrived while this core was mid-acquire, to be serviced on reaching stable.
    defValid: boolean;
    defOp:    MsgOp;        -- FwdGetS | FwdGetM | Inv
    defReq:   Node;         -- the requester to forward/ack to
    defAcks:  CountType;    -- ack_count carried on a deferred FwdGetM
    fwd:      boolean;      -- this txn's data came from a peer (cache-to-cache) -> must Unblock the dir
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

-- Record a snoop that arrived while core c was mid-acquire (one slot; assert no overwrite).
procedure RecordDeferred(c: Core; op: MsgOp; req: Node; acks: CountType);
begin
  assert (!Procs[c].defValid) "two snoops deferred at one requester (v2 bound violated)";
  Procs[c].defValid := true;
  Procs[c].defOp    := op;
  Procs[c].defReq   := req;
  Procs[c].defAcks  := acks;
end;

-- Service the (single) deferred snoop after core c has reached a stable state.
procedure ServeDeferred(c: Core);
begin
  if Procs[c].defValid then
    alias P: Procs[c] do
    switch P.defOp
      case FwdGetS:
        if P.state = C_M then
          Send(Data, P.defReq, c, 2, P.val, Gnt_S, true, 0, P.defReq);  P.state := C_O;
        elsif P.state = C_O then
          Send(Data, P.defReq, c, 2, P.val, Gnt_S, true, 0, P.defReq);  -- stay O
        elsif P.state = C_E then
          Send(Data, P.defReq, c, 2, P.val, Gnt_S, false, 0, P.defReq); P.state := C_S;
        else
          error "deferred FwdGetS serviced in a non-owner state";
        endif;
      case FwdGetM:
        Send(Data, P.defReq, c, 2, P.val, Gnt_M, false, P.defAcks, P.defReq);
        P.state := C_I;
      case Inv:
        Send(InvAck, P.defReq, c, 2, 0, Gnt_S, false, 0, c);
        P.state := C_I;
      else
        error "bad deferred op";
    endswitch;
    P.defValid := false;
    endalias;
  endif;
end;

-- ============================================================================
-- Core-initiated events (core MSHR free = a stable cache state; directory stable)
-- ============================================================================

ruleset c: Core do

  rule "load-miss I -> GetS"
    (Procs[c].state = C_I) & DirStable()
  ==> begin
    Procs[c].state := IS_D; Procs[c].acks := 0; Procs[c].fwd := false;
    Send(GetS, HomeType, c, 0, 0, Gnt_S, false, 0, c);
  end;

  rule "store-miss I -> GetM"
    (Procs[c].state = C_I) & DirStable()
  ==> begin
    Procs[c].state := IM_AD; Procs[c].acks := 0; Procs[c].fwd := false;
    Send(GetM, HomeType, c, 0, 0, Gnt_S, false, 0, c);
  end;

  rule "store-hit S -> Upgrade (SM_AD)"
    (Procs[c].state = C_S) & DirStable()
  ==> begin
    Procs[c].state := SM_AD; Procs[c].acks := 0; Procs[c].fwd := true;  -- Upgrade waits-for-Unblock
    Send(Upgrade, HomeType, c, 0, 0, Gnt_S, false, 0, c);
  end;

  rule "store-hit O -> Upgrade (OM_A)"
    (Procs[c].state = C_O) & DirStable()
  ==> begin
    Procs[c].state := OM_A; Procs[c].acks := 0; Procs[c].fwd := true;  -- Upgrade waits-for-Unblock
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

  -- v2 scope: EVICTION DISABLED. v2 isolates the acquire-side deferred-snoop matrix
  -- (concurrent same-line acquires). Eviction was verified serialised in v1; combining
  -- eviction with grant-and-go re-acquire produces a stale-Inv (an Inv to a phantom
  -- sharer that arrives after that core re-acquired the same line) which the single-
  -- address model cannot disambiguate from a current Inv — that needs a per-line
  -- generation / txn-id and is the v3 task. (See README "v3".)

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
    switch H.state
      case DIR_I:    -- L2-sourced exclusive grant, GRANT-AND-GO (dir stable immediately)
        ClearSharers(); H.sharers[r] := true; H.owner := r; H.state := DIR_EM;
        Send(Data, r, HomeType, 2, H.val, Gnt_E, false, 0, r);
      case DIR_S:    -- L2-sourced shared grant, GRANT-AND-GO
        H.sharers[r] := true;                       -- dir stays DIR_S
        Send(Data, r, HomeType, 2, H.val, Gnt_S, false, 0, r);
      case DIR_EM:   -- cache-to-cache: forward to the owner, wait for Unblock
        H.pendReq := r; H.pendOwner := H.owner; H.state := DIR_S_D;
        Send(FwdGetS, H.owner, HomeType, 1, 0, Gnt_S, false, 0, r);
      case DIR_O:
        H.pendReq := r; H.pendOwner := H.owner; H.state := DIR_S_D;
        Send(FwdGetS, H.owner, HomeType, 1, 0, Gnt_S, false, 0, r);
    endswitch;
    MultiSetRemove(midx, Net[HomeType]);

  case GetM:
    switch H.state
      case DIR_I:    -- GRANT-AND-GO: r becomes the exclusive owner immediately
        ClearSharers(); H.sharers[r] := true; H.owner := r; H.state := DIR_EM;
        Send(Data, r, HomeType, 2, H.val, Gnt_M, false, 0, r);
      case DIR_S:    -- GRANT-AND-GO: Inv the clean sharers (they ack r async), r -> owner
        nshare := OtherSharers(r);
        for c: Core do
          if H.sharers[c] & (c != r) then
            Send(Inv, c, HomeType, 1, 0, Gnt_S, false, 0, r);
          endif;
        endfor;
        Send(Data, r, HomeType, 2, H.val, Gnt_M, false, nshare, r);
        ClearSharers(); H.sharers[r] := true; H.owner := r; H.state := DIR_EM;
      case DIR_EM:   -- cache-to-cache RFO: forward to owner, wait for Unblock
        H.pendReq := r; H.pendOwner := H.owner; H.state := DIR_M_D;
        Send(FwdGetM, H.owner, HomeType, 1, 0, Gnt_M, false, 0, r);
      case DIR_O:
        H.pendReq := r; H.pendOwner := H.owner;
        nshare := 0;
        for c: Core do
          if H.sharers[c] & (c != r) & (c != H.owner) then
            Send(Inv, c, HomeType, 1, 0, Gnt_S, false, 0, r);
            nshare := nshare + 1;
          endif;
        endfor;
        Send(FwdGetM, H.owner, HomeType, 1, 0, Gnt_M, false, nshare, r);
        H.state := DIR_M_D;
    endswitch;
    MultiSetRemove(midx, Net[HomeType]);

  case Upgrade:
    -- Upgrades stay WAIT-FOR-UNBLOCK (dir busy M_D until the requester Unblocks), so a
    -- forward never targets an Upgrade requester mid-acquire -> no OM_A/SM_AD deferral.
    H.pendReq := r; undefine H.pendOwner;
    -- (R-b) r still a sharer -> no-data grant (Inv others incl an O-owner since r holds the
    -- clean value); else r lost its copy (SM_AD->IM_AD demotion) -> serve data like a GetM.
    if ((H.state = DIR_S) | (H.state = DIR_O)) & H.sharers[r] then
      nshare := OtherSharers(r);
      for c: Core do
        if H.sharers[c] & (c != r) then
          Send(Inv, c, HomeType, 1, 0, Gnt_S, false, 0, r);
        endif;
      endfor;
      Send(Data, r, HomeType, 2, H.val, Gnt_M, false, nshare, r);   -- ack_count; r keeps own data
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
    endif;
    H.state := DIR_M_D;
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
      case IS_D:   RecordDeferred(c, FwdGetS, m.req, 0);  -- mid-acquire: defer (§9.3)
      case IM_AD:  RecordDeferred(c, FwdGetS, m.req, 0);
      case IM_A:   RecordDeferred(c, FwdGetS, m.req, 0);
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
      case IS_D:   RecordDeferred(c, FwdGetM, m.req, m.acks);  -- mid-acquire: defer (§9.3)
      case IM_AD:  RecordDeferred(c, FwdGetM, m.req, m.acks);
      case IM_A:   RecordDeferred(c, FwdGetM, m.req, m.acks);
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
      case IS_D:  RecordDeferred(c, Inv, m.req, 0);  -- read granted-S then Inv'd: defer, ack after serving the load
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
    if ismember(m.src, Core) then P.fwd := true; endif;  -- data from a peer = cache-to-cache
    switch P.state
      case IS_D:
        P.val := m.val;
        if m.gst = Gnt_E then P.state := C_E; else P.state := C_S; endif;
        -- only a cache-to-cache grant needs the dir Unblock (the grant-and-go L2 grant does not)
        if P.fwd then Send(Unblock, HomeType, c, 2, P.val, m.gst, m.ostays, 0, c); endif;
        ServeDeferred(c);    -- serve the load, then any snoop that arrived mid-IS_D
      case IM_AD:
        P.val := m.val; P.acks := P.acks + m.acks;
        if P.acks = 0 then
          P.state := C_M; P.val := 1 - P.val; LastData := P.val;
          if P.fwd then Send(Unblock, HomeType, c, 2, 0, Gnt_M, false, 0, c); endif;
          ServeDeferred(c);
        else
          P.state := IM_A;
        endif;
      case SM_AD:       -- no-data grant: keep own val, just collect acks
        P.acks := P.acks + m.acks;
        if P.acks = 0 then
          P.state := C_M; P.val := 1 - P.val; LastData := P.val;
          if P.fwd then Send(Unblock, HomeType, c, 2, 0, Gnt_M, false, 0, c); endif;
          ServeDeferred(c);
        else
          P.state := IM_A;
        endif;
      case OM_A:
        P.acks := P.acks + m.acks;
        if P.acks = 0 then
          P.state := C_M; P.val := 1 - P.val; LastData := P.val;
          if P.fwd then Send(Unblock, HomeType, c, 2, 0, Gnt_M, false, 0, c); endif;
          ServeDeferred(c);
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
      if P.fwd then Send(Unblock, HomeType, c, 2, 0, Gnt_M, false, 0, c); endif;
      ServeDeferred(c);
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
    Procs[c].defValid := false; Procs[c].defOp := Inv; Procs[c].defReq := c; Procs[c].defAcks := 0;
    Procs[c].fwd := false;
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
