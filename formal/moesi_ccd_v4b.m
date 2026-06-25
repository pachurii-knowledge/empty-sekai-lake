-- ============================================================================
-- moesi_ccd.m  --  CMurphi formal model of the niigo-lake 4-core MOESI protocol
--
-- Models plans/multicore-ccd.md §9 (L1D/directory transition tables) + §13.9
-- (NINE data-source priority, D6 ack-to-requester, noisy E/S eviction).
--
-- Scope (v4b):  v1's serialised L1D+directory+NINE protocol *** PLUS LR/SC + AMO ***,
--   validating the ratified atomics contract (§9.10 LR/SC reservation coherence-kill;
--   §13.9c AMO holds M under an amo_lock, write-at-commit, bounded by a snoop-triggered
--   squash-and-replay). LR reserves a readable line; a remote WRITE (FwdGetM/Inv) kills
--   every other core's reservation; SC succeeds iff its reservation survived. An AMO
--   acquires M, RMW-reads (lock), commits the write later, and ANY remote snoop while it
--   holds/acquires M squashes it -> replay (no stale commit).
--
--   Atomicity is checked WITHOUT an unbounded epoch counter: LR/AMO snapshot the value
--   they read (rsv_val / amo_rd_val, both bounded Values), and SC-success / AMO-commit
--   ASSERT the value is unchanged — which holds because the op holds the line and any
--   intervening write would have killed the reservation / squashed the AMO. This keeps the
--   state space bounded (a free-running WriteEpoch would have exploded it).
--
--   Built on the SERIALISED base. Deferred to v4c: L1I + atomics together; multi-address.
--
-- Checks:  SWMR, single-owner, data-value, memory (DIR_I), directory-owner; PLUS
--   SC-atomicity + AMO-atomicity (inline asserts), reservation soundness, exclusive-owner-
--   excludes-other-reservations, amo_locked=>M, at-most-one-amo_locked; + deadlock freedom.
--
-- RESULT (CMurphi 5.4.9):  No error found, deadlock-free —
--   NumCores=2 : 121,194 states / 336,654 rules (~0.6 s). Coverage probes confirm LR
--   reservations, AMO locks, AND the §13.9c AMO squash-replay all fire. A negative control
--   (remove the reservation coherence-kill) fails "exclusive owner excludes other reservations"
--   -> the kill is load-bearing and the atomicity checks have teeth. NumCores=2 default.
-- ============================================================================

const
  NumCores: 2;          -- atomics fields add per-core state; 2 cores keeps it tractable
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

  -- v4b: per-core atomic activity. `atom` overlays the acquire transients (what the in-flight
  -- GetM/Upgrade is FOR) and the AMO lock window:
  --   AT_NONE       idle / a plain load or store
  --   AT_SC         an SC's Upgrade/GetM is in flight (resolve at M against the reservation)
  --   AT_AMO_ACQ    an AMO is acquiring M
  --   AT_AMO_LOCKED an AMO holds M, RMW-read done, write pending (the §13.9c lock)
  --   AT_AMO_REPLAY an AMO was squashed by a remote snoop and must re-acquire (§13.9c)
  AtomPhase: enum { AT_NONE, AT_SC, AT_AMO_ACQ, AT_AMO_LOCKED, AT_AMO_REPLAY };

  ProcRec: record
    state: CacheState;
    val:   Value;
    acks:  AckType;         -- remaining InvAcks (signed: an ack may precede the count)
    -- LR/SC reservation (single granule). rsv_val = the value read at LR (atomicity snapshot).
    rsv_valid: boolean;
    rsv_val:   Value;
    atom:      AtomPhase;   -- the in-flight atomic op (see above)
    amo_rd_val: Value;      -- value the AMO read at the lock point (atomicity snapshot)
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

function IsStable(s: CacheState): boolean;   -- any non-transient L1D state (incl. I)
begin return (s = C_I) | (s = C_M) | (s = C_O) | (s = C_E) | (s = C_S); end;

function DirStable(): boolean;
begin
  return (HomeNode.state = DIR_I) | (HomeNode.state = DIR_S)
       | (HomeNode.state = DIR_EM) | (HomeNode.state = DIR_O);
end;

-- v4b: an acquire (GetM/Upgrade) just reached M. Complete it per the in-flight atomic op
-- and Unblock the directory. A plain store writes; an SC writes iff the reservation survived
-- (atomicity asserted); an AMO takes the lock + RMW-read (the write is a separate commit rule).
procedure ReachM(c: Core);
begin
  alias P: Procs[c] do
  P.state := C_M;
  switch P.atom
    case AT_NONE:                                  -- a plain store
      P.val := 1 - P.val; LastData := P.val; P.rsv_valid := false;
    case AT_SC:                                    -- store-conditional resolves here
      if P.rsv_valid then
        assert (P.val = P.rsv_val) "SC atomicity: value changed under a live reservation";
        P.val := 1 - P.val; LastData := P.val;     -- SC SUCCESS (write)
      endif;                                       -- else SC FAIL (no write); core just holds M
      P.rsv_valid := false; P.atom := AT_NONE;
    case AT_AMO_ACQ:                               -- AMO reached M: take the lock + RMW-read
      P.amo_rd_val := P.val; P.atom := AT_AMO_LOCKED; P.rsv_valid := false;
    case AT_AMO_REPLAY:
      -- the AMO was squashed mid-acquire (its SM_AD was Inv'd -> IM_AD) but its acquire still
      -- completed -> it has fresh M; lock + RMW-read the current value (a correct retry).
      P.amo_rd_val := P.val; P.atom := AT_AMO_LOCKED; P.rsv_valid := false;
    else
      error "ReachM in an unexpected atom phase";
  endswitch;
  Send(Unblock, HomeType, c, 2, 0, Gnt_M, false, 0, c);
  endalias;
end;

-- ============================================================================
-- Core-initiated events (core MSHR free = a stable cache state; directory stable)
-- ============================================================================

ruleset c: Core do

  rule "load-miss I -> GetS"
    (Procs[c].state = C_I) & DirStable() & (Procs[c].atom = AT_NONE)
  ==> begin
    Procs[c].state := IS_D; Procs[c].acks := 0;
    Send(GetS, HomeType, c, 0, 0, Gnt_S, false, 0, c);
  end;

  rule "store-miss I -> GetM"
    (Procs[c].state = C_I) & DirStable() & (Procs[c].atom = AT_NONE)
  ==> begin
    Procs[c].state := IM_AD; Procs[c].acks := 0;
    Send(GetM, HomeType, c, 0, 0, Gnt_S, false, 0, c);
  end;

  rule "store-hit S -> Upgrade (SM_AD)"
    (Procs[c].state = C_S) & DirStable() & (Procs[c].atom = AT_NONE)
  ==> begin
    Procs[c].state := SM_AD; Procs[c].acks := 0;
    Send(Upgrade, HomeType, c, 0, 0, Gnt_S, false, 0, c);
  end;

  rule "store-hit O -> Upgrade (OM_A)"
    (Procs[c].state = C_O) & DirStable() & (Procs[c].atom = AT_NONE)
  ==> begin
    Procs[c].state := OM_A; Procs[c].acks := 0;
    Send(Upgrade, HomeType, c, 0, 0, Gnt_S, false, 0, c);
  end;

  rule "store-hit E -> M (silent)"
    (Procs[c].state = C_E) & (Procs[c].atom = AT_NONE)
  ==> begin
    Procs[c].state := C_M; Procs[c].val := 1 - Procs[c].val; LastData := Procs[c].val;
    Procs[c].rsv_valid := false;                  -- a store (even own) kills the reservation
  end;

  rule "store-hit M (write)"
    (Procs[c].state = C_M) & (Procs[c].atom = AT_NONE)
  ==> begin
    Procs[c].val := 1 - Procs[c].val; LastData := Procs[c].val;
    Procs[c].rsv_valid := false;
  end;

  rule "evict M -> PutM (MI_A)"
    (Procs[c].state = C_M) & DirStable() & (Procs[c].atom = AT_NONE)
  ==> begin
    Procs[c].state := MI_A; Procs[c].rsv_valid := false;  -- evict drops the reservation
    Send(PutM, HomeType, c, 0, Procs[c].val, Gnt_S, false, 0, c);
  end;

  rule "evict O -> PutO (OI_A)"
    (Procs[c].state = C_O) & DirStable() & (Procs[c].atom = AT_NONE)
  ==> begin
    Procs[c].state := OI_A; Procs[c].rsv_valid := false;  -- evict drops the reservation
    Send(PutO, HomeType, c, 0, Procs[c].val, Gnt_S, false, 0, c);
  end;

  rule "evict E -> PutE (EI_A, noisy)"
    (Procs[c].state = C_E) & DirStable() & (Procs[c].atom = AT_NONE)
  ==> begin
    Procs[c].state := EI_A; Procs[c].rsv_valid := false;  -- evict drops the reservation
    Send(PutE, HomeType, c, 0, 0, Gnt_S, false, 0, c);
  end;

  rule "evict S -> PutS (SI_A, noisy)"
    (Procs[c].state = C_S) & DirStable() & (Procs[c].atom = AT_NONE)
  ==> begin
    Procs[c].state := SI_A; Procs[c].rsv_valid := false;  -- evict drops the reservation
    Send(PutS, HomeType, c, 0, 0, Gnt_S, false, 0, c);
  end;

  -- ---------------- LR / SC (v4b) ----------------
  -- LR: take a reservation on a line the core already holds readable (load-then-LR covers a miss).
  rule "LR (reserve a readable line)"
    IsValidStable(Procs[c].state) & (Procs[c].atom = AT_NONE)
  ==> begin
    Procs[c].rsv_valid := true; Procs[c].rsv_val := Procs[c].val;
  end;

  -- SC success on a writable line (M/E) with a live reservation: assert atomicity, write.
  rule "SC success (writable)"
    ((Procs[c].state = C_M) | (Procs[c].state = C_E)) & Procs[c].rsv_valid & (Procs[c].atom = AT_NONE)
  ==> begin
    assert (Procs[c].val = Procs[c].rsv_val) "SC atomicity: value changed under a live reservation";
    Procs[c].state := C_M;                        -- E -> M on the SC write
    Procs[c].val := 1 - Procs[c].val; LastData := Procs[c].val;
    Procs[c].rsv_valid := false;
  end;

  -- SC needs ownership (S/O) with a live reservation: upgrade; resolve at M (atom=AT_SC).
  rule "SC upgrade (S/O) -> AT_SC"
    ((Procs[c].state = C_S) | (Procs[c].state = C_O)) & Procs[c].rsv_valid
       & DirStable() & (Procs[c].atom = AT_NONE)
  ==> begin
    Procs[c].atom := AT_SC; Procs[c].acks := 0;
    if Procs[c].state = C_S then Procs[c].state := SM_AD; else Procs[c].state := OM_A; endif;
    Send(Upgrade, HomeType, c, 0, 0, Gnt_S, false, 0, c);
  end;
  -- (an SC with no live reservation simply fails — a no-op, not modelled as a rule.)

  -- ---------------- AMO (v4b, §13.9c) ----------------
  -- AMO on a writable line (M/E): take the lock + RMW-read now (commit is a separate rule).
  rule "AMO lock (writable) -> AT_AMO_LOCKED"
    ((Procs[c].state = C_M) | (Procs[c].state = C_E)) & (Procs[c].atom = AT_NONE)
  ==> begin
    Procs[c].state := C_M;
    Procs[c].atom := AT_AMO_LOCKED; Procs[c].amo_rd_val := Procs[c].val;
    Procs[c].rsv_valid := false;
  end;

  -- AMO needs M (I/S/O): acquire with atom=AT_AMO_ACQ; ReachM takes the lock.
  rule "AMO acquire (I/S/O) -> AT_AMO_ACQ"
    ((Procs[c].state = C_I) | (Procs[c].state = C_S) | (Procs[c].state = C_O))
       & DirStable() & (Procs[c].atom = AT_NONE)
  ==> begin
    Procs[c].atom := AT_AMO_ACQ; Procs[c].acks := 0;
    switch Procs[c].state
      case C_I: Procs[c].state := IM_AD; Send(GetM,    HomeType, c, 0, 0, Gnt_S, false, 0, c);
      case C_S: Procs[c].state := SM_AD; Send(Upgrade, HomeType, c, 0, 0, Gnt_S, false, 0, c);
      case C_O: Procs[c].state := OM_A;  Send(Upgrade, HomeType, c, 0, 0, Gnt_S, false, 0, c);
    endswitch;
  end;

  -- AMO commit-write: holds M under the lock, no squash pending -> write the RMW result.
  rule "AMO commit (write-at-commit)"
    (Procs[c].state = C_M) & (Procs[c].atom = AT_AMO_LOCKED)
  ==> begin
    assert (Procs[c].val = Procs[c].amo_rd_val) "AMO atomicity: value changed during the RMW lock";
    Procs[c].val := 1 - Procs[c].val; LastData := Procs[c].val;
    Procs[c].atom := AT_NONE;
  end;

  -- AMO replay: a remote snoop squashed the AMO (atom=AT_AMO_REPLAY) -> re-issue from scratch.
  rule "AMO replay (re-acquire)"
    (Procs[c].atom = AT_AMO_REPLAY) & IsStable(Procs[c].state) & DirStable()
  ==> begin
    Procs[c].atom := AT_NONE;     -- the AMO-lock / AMO-acquire rules above will re-fire
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
  -- v4b: a remote WRITE (FwdGetM/Inv) kills this core's LR reservation (§9.10); a remote
  -- READ (FwdGetS) does not (the copy survives as O). ANY snoop squashes an in-flight AMO
  -- (§13.9c) — it loses exclusive M, so it must replay (no stale commit).
  if (m.op = FwdGetM) | (m.op = Inv) then P.rsv_valid := false; endif;
  if (P.atom = AT_AMO_ACQ) | (P.atom = AT_AMO_LOCKED) then P.atom := AT_AMO_REPLAY; endif;
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
        P.val := m.val; P.acks := P.acks + m.acks;
        if P.acks = 0 then
          ReachM(c);
        else
          P.state := IM_A;
        endif;
      case SM_AD:       -- no-data grant: keep own val, just collect acks
        P.acks := P.acks + m.acks;
        if P.acks = 0 then
          ReachM(c);
        else
          P.state := IM_A;
        endif;
      case OM_A:
        P.acks := P.acks + m.acks;
        if P.acks = 0 then
          ReachM(c);
        else
          P.state := IM_A;
        endif;
      else
        error "Data in unexpected state";
    endswitch;

  case InvAck:
    P.acks := P.acks - 1;
    if (P.state = IM_A) & (P.acks = 0) then
      ReachM(c);
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
    Procs[c].rsv_valid := false; Procs[c].rsv_val := 0;
    Procs[c].atom := AT_NONE;    Procs[c].amo_rd_val := 0;
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

-- ---------------- v4b atomics invariants ----------------
-- (SC/AMO atomicity are also asserted inline at SC-success / AMO-commit / ReachM.)

-- reservation soundness: a live reservation implies the line is still held with the reserved
-- value unchanged — i.e. no write (own or remote) has occurred since the LR (the kill works).
-- (A live reservation may coexist with an SC's in-flight Upgrade (SM_AD/OM_A), so we do NOT
-- require a stable state — only that the reserved value still equals the current value, i.e.
-- no write (own or remote) survived since the LR. A missing kill makes rsv_val != LastData.)
invariant "reservation soundness (no write survived under a live reservation)"
  forall c: Core do
    Procs[c].rsv_valid -> (Procs[c].rsv_val = LastData)
  endforall;

-- an exclusive M/E writer excludes any OTHER core's live reservation (it invalidated them).
invariant "exclusive owner excludes other reservations"
  forall c1: Core do forall c2: Core do
    (c1 != c2 & (Procs[c1].state = C_M | Procs[c1].state = C_E) & Procs[c2].rsv_valid) -> false
  endforall endforall;

-- the AMO lock holds M, and at most one core is AMO-locked at a time (it is exclusive).
invariant "amo_locked implies M ownership"
  forall c: Core do (Procs[c].atom = AT_AMO_LOCKED) -> (Procs[c].state = C_M) endforall;

invariant "at most one amo_locked"
  forall c1: Core do forall c2: Core do
    (c1 != c2 & (Procs[c1].atom = AT_AMO_LOCKED) & (Procs[c2].atom = AT_AMO_LOCKED)) -> false
  endforall endforall;
