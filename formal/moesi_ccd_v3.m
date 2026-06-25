-- ============================================================================
-- moesi_ccd.m  --  CMurphi formal model of the niigo-lake 4-core MOESI protocol
--
-- Models plans/multicore-ccd.md §9 (L1D/directory transition tables) + §13.9
-- (NINE data-source priority, D6 ack-to-requester, noisy E/S eviction).
--
-- Scope (v3):  N L1D caches + 1 directory + NINE L2/memory, ONE address, data-value
--   tracking. v3 = v2's ACQUIRE-SIDE DEFERRED-SNOOP MATRIX (directory grant-and-go for
--   L2-sourced from-I GetS/GetM; requesters in IS_D/IM_AD/IM_A defer snoops and service
--   them on reaching stable) *** COMBINED WITH EVICTION RE-ENABLED ***.
--
--   The v2 stale-Inv corner (an Inv to a phantom evicting sharer arriving after that core
--   re-acquired the same line) is CLOSED at the source by the 3-valued `onext` on the
--   FwdGetS downgrade response: an evicting owner reports ON_I ("leaving"), so the dir
--   CLEARS its sharer bit at finalize — no phantom sharer is ever left, hence no stale Inv,
--   and no per-line generation/txn-id is needed. (That is the v3 finding: the snoop-
--   identity problem dissolves if the downgrade response carries the owner's exact next
--   state, because the directory's sharer set then stays exact.)
--
--   Upgrades stay wait-for-Unblock (no upgrade requester is snooped mid-acquire). The
--   ack-before-write rule keeps data sound across the deferral window.
--   Deferred to v4: L1I agent (D2 split tracking), LR/SC, AMO, multi-address.
--
-- Resolutions this model PINS (feedback to §9.5/§9.6/§9.3, under-specified in prose):
--   (R-a) the FwdGetS downgrade response carries the owner's next state `onext` (O/S/I),
--         echoed by the requester in Unblock, so the dir resolves EM/O without telling E
--         from M AND keeps its sharer set exact (a leaving owner is removed) — no phantom.
--   (R-b) a GetM/Upgrade requester that lost its copy to a prior Inv is detected at the dir
--         as "requester not a sharer" and served data as a GetM (the SM_AD->IM_AD demotion).
--
-- Checks:  SWMR coherence, single-owner, data-value correctness, memory correctness
--   (DIR_I), directory-owner consistency, + (Murphi built-in) deadlock freedom.
--
-- RESULT (CMurphi 5.4.9):
--   NumCores=2 (default): No error found, deadlock-free — 91,212 states / 211,528 rules.
--     2 cores fully exercises the v3-specific logic: BOTH stale-Inv scenarios (phantom-
--     owner and sharer-evict-then-reacquire) are 2-core-reachable. Coverage probes confirm
--     the acquire-side deferral path AND the evict-vs-forward race (II_A) both fire.
--   NumCores=3: exhaustive check exceeds a 19 GB host (state space >14.7M; the BFS frontier
--     or the hash table fills). >14.7M states were explored with NO error before exhaustion
--     — a bigger machine / hash-compaction is needed to close 3 cores exhaustively.
-- ============================================================================

const
  NumCores: 2;           -- default 2: exhaustively checks the v3 logic; 3-core state space
                         --   exceeds ~19 GB (see RESULT). Raise to 3 only on a big-RAM host.
  NetMax:   10;          -- per-node message-buffer bound (actual max depth observed ~5)

type
  Core:  scalarset(NumCores);
  Home:  enum { HomeType };
  Node:  union { Home, Core };

  Value: 0..1;           -- data values (2 enough to expose corruption)

  CountType: 0..NumCores;          -- sharer counts / ack_count (named so codegen types match)
  AckType:   -NumCores..NumCores;  -- remaining InvAcks (signed: an ack may precede the count)

  VCType: 0..2;          -- 0 = Req (C0), 1 = Fwd (C1), 2 = Resp (C2/C3/C4)

  GrantState: enum { Gnt_S, Gnt_E, Gnt_M };

  -- v3: the owner's post-FwdGetS outcome, conveyed owner -> requester -> dir, so the dir
  -- resolves the transaction WITHOUT having to distinguish E from M, AND clears a *leaving*
  -- owner's sharer bit (no phantom sharer -> no stale Inv). Replaces v2's 2-valued `ostays`.
  --   ON_O = owner keeps dirty data + ownership (M->O, or O stays O)
  --   ON_S = owner relinquishes ownership but stays a clean sharer (E->S)
  --   ON_I = owner is leaving entirely (it was mid-eviction MI_A/OI_A/EI_A) -> clear its bit
  --   ON_NA = not applicable (any non-FwdGetS message)
  OwnerNext: enum { ON_NA, ON_O, ON_S, ON_I };

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
    onext:  OwnerNext;      -- (v3) owner's post-FwdGetS outcome (O/S/I); ON_NA elsewhere
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
               val: Value; gst: GrantState; onext: OwnerNext;
               acks: CountType; req: Node);
var m: Message;
begin
  assert (MultiSetCount(i: Net[dst], true) < NetMax) "network buffer overflow";
  m.op := op; m.src := src; m.dst := dst; m.vc := vc;
  m.val := val; m.gst := gst; m.onext := onext; m.acks := acks; m.req := req;
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

-- v3 RTL property: an L1 must DRAIN a pending snoop (vc=1 Fwd/Inv) for a line before it
-- issues a NEW demand for that line. Otherwise a snoop for the *previous* incarnation can
-- race a re-acquire and be mistaken for a current snoop (the stale-Inv problem). Modelled
-- by gating the core's request rules on an empty inbound snoop channel (single address).
function NoPendingSnoop(c: Core): boolean;
begin
  return MultiSetCount(i: Net[c], Net[c][i].vc = 1) = 0;
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
          Send(Data, P.defReq, c, 2, P.val, Gnt_S, ON_O, 0, P.defReq);  P.state := C_O;
        elsif P.state = C_O then
          Send(Data, P.defReq, c, 2, P.val, Gnt_S, ON_O, 0, P.defReq);  -- stay O
        elsif P.state = C_E then
          Send(Data, P.defReq, c, 2, P.val, Gnt_S, ON_S, 0, P.defReq); P.state := C_S;
        else
          error "deferred FwdGetS serviced in a non-owner state";
        endif;
      case FwdGetM:
        Send(Data, P.defReq, c, 2, P.val, Gnt_M, ON_NA, P.defAcks, P.defReq);
        P.state := C_I;
      case Inv:
        Send(InvAck, P.defReq, c, 2, 0, Gnt_S, ON_NA, 0, c);
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
    (Procs[c].state = C_I) & DirStable() & NoPendingSnoop(c)
  ==> begin
    Procs[c].state := IS_D; Procs[c].acks := 0; Procs[c].fwd := false;
    Send(GetS, HomeType, c, 0, 0, Gnt_S, ON_NA, 0, c);
  end;

  rule "store-miss I -> GetM"
    (Procs[c].state = C_I) & DirStable() & NoPendingSnoop(c)
  ==> begin
    Procs[c].state := IM_AD; Procs[c].acks := 0; Procs[c].fwd := false;
    Send(GetM, HomeType, c, 0, 0, Gnt_S, ON_NA, 0, c);
  end;

  rule "store-hit S -> Upgrade (SM_AD)"
    (Procs[c].state = C_S) & DirStable() & NoPendingSnoop(c)
  ==> begin
    Procs[c].state := SM_AD; Procs[c].acks := 0; Procs[c].fwd := true;  -- Upgrade waits-for-Unblock
    Send(Upgrade, HomeType, c, 0, 0, Gnt_S, ON_NA, 0, c);
  end;

  rule "store-hit O -> Upgrade (OM_A)"
    (Procs[c].state = C_O) & DirStable() & NoPendingSnoop(c)
  ==> begin
    Procs[c].state := OM_A; Procs[c].acks := 0; Procs[c].fwd := true;  -- Upgrade waits-for-Unblock
    Send(Upgrade, HomeType, c, 0, 0, Gnt_S, ON_NA, 0, c);
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

  -- v3: EVICTION RE-ENABLED (combined with the v2 grant-and-go deferred-snoop matrix).
  -- The v2 stale-Inv corner is closed by the ON_I owner-next on FwdGetS (an evicting owner
  -- reports "leaving", so the dir clears its sharer bit at finalize -> no phantom sharer).
  rule "evict M -> PutM (MI_A)"
    (Procs[c].state = C_M) & DirStable()
  ==> begin
    Procs[c].state := MI_A;
    Send(PutM, HomeType, c, 0, Procs[c].val, Gnt_S, ON_NA, 0, c);
  end;

  rule "evict O -> PutO (OI_A)"
    (Procs[c].state = C_O) & DirStable()
  ==> begin
    Procs[c].state := OI_A;
    Send(PutO, HomeType, c, 0, Procs[c].val, Gnt_S, ON_NA, 0, c);
  end;

  rule "evict E -> PutE (EI_A, noisy)"
    (Procs[c].state = C_E) & DirStable()
  ==> begin
    Procs[c].state := EI_A;
    Send(PutE, HomeType, c, 0, 0, Gnt_S, ON_NA, 0, c);
  end;

  rule "evict S -> PutS (SI_A, noisy)"
    (Procs[c].state = C_S) & DirStable()
  ==> begin
    Procs[c].state := SI_A;
    Send(PutS, HomeType, c, 0, 0, Gnt_S, ON_NA, 0, c);
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
    switch H.state
      case DIR_I:    -- L2-sourced exclusive grant, GRANT-AND-GO (dir stable immediately)
        ClearSharers(); H.sharers[r] := true; H.owner := r; H.state := DIR_EM;
        Send(Data, r, HomeType, 2, H.val, Gnt_E, ON_NA, 0, r);
      case DIR_S:    -- L2-sourced shared grant, GRANT-AND-GO
        H.sharers[r] := true;                       -- dir stays DIR_S
        Send(Data, r, HomeType, 2, H.val, Gnt_S, ON_NA, 0, r);
      case DIR_EM:   -- cache-to-cache: forward to the owner, wait for Unblock
        H.pendReq := r; H.pendOwner := H.owner; H.state := DIR_S_D;
        Send(FwdGetS, H.owner, HomeType, 1, 0, Gnt_S, ON_NA, 0, r);
      case DIR_O:
        H.pendReq := r; H.pendOwner := H.owner; H.state := DIR_S_D;
        Send(FwdGetS, H.owner, HomeType, 1, 0, Gnt_S, ON_NA, 0, r);
    endswitch;
    MultiSetRemove(midx, Net[HomeType]);

  case GetM:
    switch H.state
      case DIR_I:    -- GRANT-AND-GO: r becomes the exclusive owner immediately
        ClearSharers(); H.sharers[r] := true; H.owner := r; H.state := DIR_EM;
        Send(Data, r, HomeType, 2, H.val, Gnt_M, ON_NA, 0, r);
      case DIR_S:    -- GRANT-AND-GO: Inv the clean sharers (they ack r async), r -> owner
        nshare := OtherSharers(r);
        for c: Core do
          if H.sharers[c] & (c != r) then
            Send(Inv, c, HomeType, 1, 0, Gnt_S, ON_NA, 0, r);
          endif;
        endfor;
        Send(Data, r, HomeType, 2, H.val, Gnt_M, ON_NA, nshare, r);
        ClearSharers(); H.sharers[r] := true; H.owner := r; H.state := DIR_EM;
      case DIR_EM:   -- cache-to-cache RFO: forward to owner, wait for Unblock
        H.pendReq := r; H.pendOwner := H.owner; H.state := DIR_M_D;
        Send(FwdGetM, H.owner, HomeType, 1, 0, Gnt_M, ON_NA, 0, r);
      case DIR_O:
        H.pendReq := r; H.pendOwner := H.owner;
        nshare := 0;
        for c: Core do
          if H.sharers[c] & (c != r) & (c != H.owner) then
            Send(Inv, c, HomeType, 1, 0, Gnt_S, ON_NA, 0, r);
            nshare := nshare + 1;
          endif;
        endfor;
        Send(FwdGetM, H.owner, HomeType, 1, 0, Gnt_M, ON_NA, nshare, r);
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
          Send(Inv, c, HomeType, 1, 0, Gnt_S, ON_NA, 0, r);
        endif;
      endfor;
      Send(Data, r, HomeType, 2, H.val, Gnt_M, ON_NA, nshare, r);   -- ack_count; r keeps own data
    else
      switch H.state
        case DIR_I:
          Send(Data, r, HomeType, 2, H.val, Gnt_M, ON_NA, 0, r);
        case DIR_S:
          nshare := OtherSharers(r);
          for c: Core do
            if H.sharers[c] & (c != r) then
              Send(Inv, c, HomeType, 1, 0, Gnt_S, ON_NA, 0, r);
            endif;
          endfor;
          Send(Data, r, HomeType, 2, H.val, Gnt_M, ON_NA, nshare, r);
        case DIR_EM:
          H.pendOwner := H.owner;
          Send(FwdGetM, H.owner, HomeType, 1, 0, Gnt_M, ON_NA, 0, r);
        case DIR_O:
          H.pendOwner := H.owner;
          nshare := 0;
          for c: Core do
            if H.sharers[c] & (c != r) & (c != H.owner) then
              Send(Inv, c, HomeType, 1, 0, Gnt_S, ON_NA, 0, r);
              nshare := nshare + 1;
            endif;
          endfor;
          Send(FwdGetM, H.owner, HomeType, 1, 0, Gnt_M, ON_NA, nshare, r);
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
    Send(WBAck, r, HomeType, 2, 0, Gnt_S, ON_NA, 0, r);
    MultiSetRemove(midx, Net[HomeType]);

  case PutO:
    if (H.state = DIR_O) & (H.owner = r) then
      H.val := m.val; H.sharers[r] := false; undefine H.owner;
      if SharerCount() = 0 then H.state := DIR_I; else H.state := DIR_S; endif;
    else
      H.sharers[r] := false;
    endif;
    Send(WBAck, r, HomeType, 2, 0, Gnt_S, ON_NA, 0, r);
    MultiSetRemove(midx, Net[HomeType]);

  case PutE:
    if (H.state = DIR_EM) & (H.owner = r) then
      H.state := DIR_I; undefine H.owner; ClearSharers();
    else
      H.sharers[r] := false;
    endif;
    Send(WBAck, r, HomeType, 2, 0, Gnt_S, ON_NA, 0, r);
    MultiSetRemove(midx, Net[HomeType]);

  case PutS:
    H.sharers[r] := false;
    if (H.state = DIR_S) & (SharerCount() = 0) then H.state := DIR_I; endif;
    Send(WBAck, r, HomeType, 2, 0, Gnt_S, ON_NA, 0, r);
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
      -- cache-to-cache read: owner W forwarded the data to R (= m.src). Resolve by W's
      -- reported next-state (onext) — v3 eliminates the v2 phantom sharer.
      H.sharers[m.src] := true;
      switch m.onext
        case ON_O:   -- W kept dirty data + ownership (M->O / O stays O)
          H.sharers[H.pendOwner] := true;
          H.owner := H.pendOwner; H.state := DIR_O;
        case ON_S:   -- W relinquished ownership but stays a clean sharer (E->S)
          H.sharers[H.pendOwner] := true;
          H.val := m.val; undefine H.owner; H.state := DIR_S;
        case ON_I:   -- W is LEAVING (it was mid-eviction): clear its bit -> no phantom sharer
          H.sharers[H.pendOwner] := false;
          H.val := m.val; undefine H.owner; H.state := DIR_S;
        else
          error "FwdGetS Unblock without a valid owner-next (onext)";
      endswitch;
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
    -- the owner forwards data + its post-state (onext) + downgrades; the REQUESTER sends the
    -- single Unblock echoing onext + the value. v3: an EVICTING owner reports ON_I (leaving),
    -- so the dir clears its sharer bit at finalize -> NO phantom sharer -> no stale Inv.
    switch P.state
      case C_E:    -- clean exclusive: supply clean data, relinquish but stay a clean sharer
        Send(Data, m.req, c, 2, P.val, Gnt_S, ON_S, 0, m.req);
        P.state := C_S;
      case C_M:    -- keep dirty data + ownership (M->O)
        Send(Data, m.req, c, 2, P.val, Gnt_S, ON_O, 0, m.req);
        P.state := C_O;
      case C_O:    -- stay O
        Send(Data, m.req, c, 2, P.val, Gnt_S, ON_O, 0, m.req);
      case OM_A:   -- owner mid-Upgrade: supply data, stay O-ish (its later Upgrade Inv's the new reader)
        Send(Data, m.req, c, 2, P.val, Gnt_S, ON_O, 0, m.req);
      case MI_A:   -- WB-vs-Fwd race (B5): supply data, LEAVE (ON_I); PutM still writes L2
        Send(Data, m.req, c, 2, P.val, Gnt_S, ON_I, 0, m.req);
      case OI_A:
        Send(Data, m.req, c, 2, P.val, Gnt_S, ON_I, 0, m.req);
      case EI_A:   -- B11 fix: still supply the clean data, then -> II_A (ON_I: leaving)
        Send(Data, m.req, c, 2, P.val, Gnt_S, ON_I, 0, m.req);
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
      case C_E:  Send(Data, m.req, c, 2, P.val, Gnt_M, ON_NA, m.acks, m.req); P.state := C_I;
      case C_M:  Send(Data, m.req, c, 2, P.val, Gnt_M, ON_NA, m.acks, m.req); P.state := C_I;
      case C_O:  Send(Data, m.req, c, 2, P.val, Gnt_M, ON_NA, m.acks, m.req); P.state := C_I;
      case OM_A: Send(Data, m.req, c, 2, P.val, Gnt_M, ON_NA, m.acks, m.req); P.state := IM_AD;
                 -- owner mid-Upgrade loses to a peer writer: relinquish + re-acquire (->IM_AD)
      case MI_A: Send(Data, m.req, c, 2, P.val, Gnt_M, ON_NA, m.acks, m.req); P.state := II_A;
      case OI_A: Send(Data, m.req, c, 2, P.val, Gnt_M, ON_NA, m.acks, m.req); P.state := II_A;
      case EI_A: Send(Data, m.req, c, 2, P.val, Gnt_M, ON_NA, m.acks, m.req); P.state := II_A;
      case IS_D:   RecordDeferred(c, FwdGetM, m.req, m.acks);  -- mid-acquire: defer (§9.3)
      case IM_AD:  RecordDeferred(c, FwdGetM, m.req, m.acks);
      case IM_A:   RecordDeferred(c, FwdGetM, m.req, m.acks);
      else
        error "FwdGetM to a non-owner state";
    endswitch;

  case Inv:
    switch P.state
      case C_S:   Send(InvAck, m.req, c, 2, 0, Gnt_S, ON_NA, 0, c); P.state := C_I;
      case C_O:   Send(InvAck, m.req, c, 2, 0, Gnt_S, ON_NA, 0, c); P.state := C_I;
                  -- O-owner invalidated by a clean-sharer Upgrade: requester holds the
                  -- same clean value, so discarding this (dirty) copy is value-safe.
      case C_I:   Send(InvAck, m.req, c, 2, 0, Gnt_S, ON_NA, 0, c);          -- race; ack anyway
      case SM_AD: Send(InvAck, m.req, c, 2, 0, Gnt_S, ON_NA, 0, c); P.state := IM_AD; -- demote
      case OM_A:  Send(InvAck, m.req, c, 2, 0, Gnt_S, ON_NA, 0, c); P.state := IM_AD; -- demote
      case SI_A:  Send(InvAck, m.req, c, 2, 0, Gnt_S, ON_NA, 0, c); P.state := II_A;
      case MI_A:  Send(InvAck, m.req, c, 2, 0, Gnt_S, ON_NA, 0, c); P.state := II_A;
                  -- phantom sharer: an owner that forwarded its data (ostays=0) and is
                  -- mid-PutM is still listed as a sharer until its PutM lands; ack the Inv.
      case OI_A:  Send(InvAck, m.req, c, 2, 0, Gnt_S, ON_NA, 0, c); P.state := II_A;
      case EI_A:  Send(InvAck, m.req, c, 2, 0, Gnt_S, ON_NA, 0, c); P.state := II_A;
      case II_A:  Send(InvAck, m.req, c, 2, 0, Gnt_S, ON_NA, 0, c);  -- phantom sharer already ->I-bound; ack
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
        if P.fwd then Send(Unblock, HomeType, c, 2, P.val, m.gst, m.onext, 0, c); endif;
        ServeDeferred(c);    -- serve the load, then any snoop that arrived mid-IS_D
      case IM_AD:
        P.val := m.val; P.acks := P.acks + m.acks;
        if P.acks = 0 then
          P.state := C_M; P.val := 1 - P.val; LastData := P.val;
          if P.fwd then Send(Unblock, HomeType, c, 2, 0, Gnt_M, ON_NA, 0, c); endif;
          ServeDeferred(c);
        else
          P.state := IM_A;
        endif;
      case SM_AD:       -- no-data grant: keep own val, just collect acks
        P.acks := P.acks + m.acks;
        if P.acks = 0 then
          P.state := C_M; P.val := 1 - P.val; LastData := P.val;
          if P.fwd then Send(Unblock, HomeType, c, 2, 0, Gnt_M, ON_NA, 0, c); endif;
          ServeDeferred(c);
        else
          P.state := IM_A;
        endif;
      case OM_A:
        P.acks := P.acks + m.acks;
        if P.acks = 0 then
          P.state := C_M; P.val := 1 - P.val; LastData := P.val;
          if P.fwd then Send(Unblock, HomeType, c, 2, 0, Gnt_M, ON_NA, 0, c); endif;
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
      if P.fwd then Send(Unblock, HomeType, c, 2, 0, Gnt_M, ON_NA, 0, c); endif;
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
