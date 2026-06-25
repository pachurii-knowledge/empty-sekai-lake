# formal/ — MOESI protocol model checking (CMurphi)

Formal verification of the niigo-lake 4-core **MOESI directory protocol** from
`plans/multicore-ccd.md` §9 + §13, as called for by §V.2 / §8 M2 ("formal-first:
model-check the protocol before any L1D/directory RTL").

The model checker is **CMurphi 5.4.9**, vendored at `references/cmurphi`.

## Files

| file | what |
|---|---|
| `moesi_ccd.m` | **v1** — MOESI directory protocol, directory serialised per line (the spec, executable) |
| `moesi_ccd_v2.m` | **v2** — adds the **acquire-side deferred-snoop matrix** (directory grant-and-go; requesters defer snoops mid-acquire). Eviction disabled. |
| `moesi_ccd_v3.m` | **v3** — v2's deferred-snoop matrix **combined with eviction** (the hard case). Closes the stale-`Inv` via `ownerNext` + a snoop-drain rule. Default 2 cores (3-core exceeds ~19 GB). |
| `moesi_ccd_v4a.m` | **v4a** — v1's serialised base **+ a per-core L1I instruction cache**, validating **D2** (directory tracks `i_sharers`/`d_sharers` separately). Default 2 cores. |
| `moesi_ccd_v4b.m` | **v4b** — v1's serialised base **+ LR/SC + AMO**, validating the atomics contract (§9.10 reservation coherence-kill; §13.9c AMO `amo_lock` snoop-squash-replay). Default 2 cores. |
| `moesi_ccd_neg.m` | a **negative control** (on v1): the Inv-Ack wait removed — SWMR *must* fail, proving the invariants aren't vacuous |
| `run.sh` | compile (`mu` → `g++`) and model-check a `.m` |

## One-time setup — build the `mu` compiler

`flex`/`byacc` are **not** required (the generated parser ships pre-built), but the
old codegen has an `-O2` strict-aliasing bug that segfaults, so build with `-O0`:

```sh
cd references/cmurphi/src
touch lex.yy.c y.tab.c y.tab.h        # mark the pre-generated parser up-to-date
make CFLAGS="-O0 -fno-strict-aliasing -fpermissive -w"
```

## Run

```sh
./formal/run.sh moesi_ccd        # v1: expect "No error found."
./formal/run.sh moesi_ccd_v2     # v2: expect "No error found." (deferred-snoop matrix)
./formal/run.sh moesi_ccd_v3     # v3: expect "No error found." (eviction + deferral combined)
./formal/run.sh moesi_ccd_v4a    # v4a: expect "No error found." (L1I + D2 split tracking)
./formal/run.sh moesi_ccd_v4b    # v4b: expect "No error found." (LR/SC + AMO)
./formal/run.sh moesi_ccd_neg    # negative control: expect SWMR "... failed"
./formal/run.sh moesi_ccd -tv    # -tv etc. pass through (violating trace if any)
```

## What is modelled (v1)

N **L1D** caches + 1 **directory** + a **NINE** L2/memory, **one address**, with
data-value tracking. Full MOESI stable + transient states (`IS_D IM_AD IM_A SM_AD
OM_A MI_A OI_A EI_A SI_A II_A`), directory-orchestrated **cache-to-cache** forwarding,
**ack-to-requester** invalidation counting (D6), and **noisy E/S eviction** (§9.7).

The directory **serialises per line** (a 2nd request to a busy line waits, §9.5):
busy from request until the requester's `Unblock`. This is a sound abstraction for
the safety properties below; it does **not** exercise the §9.3 *acquire-side
deferred-snoop* rows (a requester receiving a conflicting snoop mid-`IS_D`/`IM_AD`),
which need concurrent same-line transactions — see "v2" below. Eviction-vs-forward
races (WB-vs-Fwd / B5, PUTE-vs-Fwd / B11) **are** exercised, because eviction is a
separate action that races an in-flight forward.

### Invariants checked
- **SWMR** — an exclusive `M`/`E` excludes any other valid copy.
- **single owner** — at most one core in `M`/`O`/`E`.
- **data correctness** — every valid copy holds the last-written value.
- **memory correctness** — L2/memory holds the current value when the line is
  uncached (`DIR_I`).
- **directory-owner consistency** — `EM`/`O` ⇒ a defined core owner.
- **deadlock freedom** — CMurphi's built-in (no reachable dead state).

### Result (CMurphi 5.4.9)
| model / config | states | rules | verdict |
|---|---:|---:|---|
| `moesi_ccd` 3 cores (default) | 66,328 | 190,772 | **No error**, deadlock-free (~0.6 s) |
| `moesi_ccd` 4 cores | 872,587 | 3,082,542 | **No error**, deadlock-free (~12 s) |
| `moesi_ccd_v2` 3 cores | 83,049 | 165,277 | **No error**, deadlock-free (~0.5 s) |
| `moesi_ccd_v2` 4 cores | 4,707,141 | 10,065,162 | **No error**, deadlock-free (~40 s, ~14 GB) |
| `moesi_ccd_v3` 2 cores (default) | 91,212 | 211,528 | **No error**, deadlock-free (~1 s) |
| `moesi_ccd_v3` 3 cores | >14.7M explored | — | no error in prefix; **exceeds ~19 GB** to close exhaustively |
| `moesi_ccd_v4a` 2 cores (default) | 805,861 | 2,566,362 | **No error**, deadlock-free (~7 s, ~6 GB) |
| `moesi_ccd_v4b` 2 cores (default) | 121,194 | 336,654 | **No error**, deadlock-free (~1 s) |
| `moesi_ccd_neg` | 887 | — | **SWMR failed** (as intended) |

## v2 — acquire-side deferred-snoop matrix (`moesi_ccd_v2.m`)

v2 relaxes the directory to **grant-and-go** for L2-sourced grants (a from-`I` `GetS`/`GetM`
returns the directory to a stable state immediately), so a *second* transaction can target a
requester that is **still mid-acquire**. That requester (in `IS_D`/`IM_AD`/`IM_A`) **defers**
the snoop in a one-slot MSHR field and services it on reaching its stable state — the §9.3
"defer" rows. Cache-to-cache forwarding from a *stable* owner still uses `S_D`/`M_D` + `Unblock`;
`Upgrade`s stay wait-for-`Unblock` (so an upgrade requester is never snooped mid-acquire — that
avoids an `OM_A`/`SM_AD` forward-vs-respond ambiguity). The **ack-before-write** rule (a writer
reaches `M` only after every Inv-Ack) keeps the data invariant sound across the deferral window.

A **coverage probe** (a temporary invariant "no snoop is ever deferred") **fails at 41 states**,
confirming the deferral path is genuinely exercised, not vacuously absent.

**Eviction is disabled in v2** (verified serialised in v1). Combining eviction with grant-and-go
re-acquire produced a **stale-Inv** finding that v3 resolves.

## v3 — eviction + deferral combined (`moesi_ccd_v3.m`)

v3 = v2's deferred-snoop matrix **with eviction re-enabled** — the genuinely hard case. Two
mechanisms close the stale-`Inv` corner at the source (no per-line generation counter needed):

- **3-valued `ownerNext` on the `FwdGetS` downgrade response** (replaces v2's 2-valued `ostays`):
  an owner reports `O` (keeps ownership), `S` (E→S, stays a clean sharer), or **`I` (leaving —
  it was mid-eviction)**. The directory **clears a leaving owner's sharer bit at finalize**, so a
  *phantom evicting sharer* is never left behind → that source of stale `Inv`s is eliminated.
- **Snoop-drain rule (the v3 RTL requirement):** an L1 must service (ack) a pending snoop for a
  line **before** issuing a new demand for it. Otherwise a snoop for the *previous* incarnation
  (a sharer that was Inv'd, then evicted and re-acquired) races the new acquire and is mistaken
  for a current snoop. Modelled by gating the core's request rules on an empty inbound snoop
  channel — a natural property of an L1 whose snoop FSM drains before the demand FSM re-misses.

**Result:** No error / deadlock-free at **2 cores (91,212 states)**. Both stale-`Inv` scenarios
(phantom-owner and sharer-evict-then-reacquire) are 2-core-reachable, and two coverage probes
confirm the **deferral path** (probe A) and the **evict-vs-forward race** (probe B, `II_A` reached)
both fire. **3 cores** explores >14.7M states with no error but **exceeds a 19 GB host** to close
exhaustively (the BFS frontier or the hash table fills) — a bigger machine / hash-compaction is
needed there. (Lowering `NetMax` shrinks per-state size and helps; it is set to 10.)

## Two race resolutions this model PINS (feedback to §9.5/§9.6)

Formalising the protocol surfaced two cells the prose under-specified; the model
adopts (and verifies) precise resolutions — these should be folded into the RTL spec:

- **(R-a) the `FwdGetS` downgrade response carries the owner's next state.** The directory
  cannot distinguish `E` from `M` at an owner (the whole point of the `EM` state), so it can't
  decide `EM/O → O` vs `→ S` on a cache-to-cache read by itself. Resolution: the owner stamps its
  outcome on the forwarded data — v2 used a 2-valued `owner_stays`; **v3 uses a 3-valued `ownerNext`
  (`O` keeps ownership / `S` E→S stays a clean sharer / `I` leaving mid-eviction)**. The requester
  echoes it (and the value) in `Unblock`; the directory resolves the final state, **refreshes L2**
  when no dirty owner remains, and **clears a leaving (`I`) owner's sharer bit** so no phantom
  sharer is left (the v3 fix for the eviction-race stale-`Inv`).
- **(R-b) lost-copy `Upgrade` = `GetM`.** An `Upgrade` requester that lost its `S` copy
  to a prior transaction's `Inv` (the §9.3 `SM_AD→IM_AD` demotion) is detected at the
  directory as "requester is not a current sharer" and is served **data** (treated as a
  `GetM`). No extra message needed.

## A third v1 finding — phantom evicting sharers
An **evicting** owner that forwards data (`MI_A`/`OI_A`/`EI_A` on a `FwdGetS`) stays a
**phantom directory sharer** until its `Put*` lands, so the L1D snoop FSM must accept an
`Inv`/`Fwd` while in those evict transients (and in `II_A`) — the RTL snoop FSM cannot
assume an evicting line is already gone.

## v4a — L1I instruction cache + D2 split tracking (`moesi_ccd_v4a.m`)

v4a adds a **per-core L1I** to v1's serialised base, validating **D2**: the directory keeps
**separate `i_sharers` / `d_sharers`** presence vectors. The L1I holds only `{IC_I, IC_S}` (+ an
ifetch transient `IC_IS_D` and an evict transient `IC_SI_A`); it issues `GetS(is_icache)`, is
granted **S only** (never E/M/O), takes `Inv → I + Inv-Ack`, and clean-evicts via a `PutS(is_icache)`
that **waits for `WBAck`** (so it cannot re-fetch before its eviction is processed — else a stale
`GetM`-`Inv` to the not-yet-cleared `i_sharer` races the re-fetch, the same stale-snoop class as v3).
A `GetM`/`Upgrade` invalidates **both** vectors — including the requester's **own** L1I
(self-CMODX, coherent-I-cache §9.9) — and the ack-count is the popcount over both. The I/D
discriminator rides a `Message.is_icache` bit; a `SendI()` wrapper tags L1I traffic so the existing
`Send()` call sites are untouched.

Two routing subtleties the model pins (RTL notes): `is_icache` on a **`FwdGetS`** means "the
*requester* is an L1I" (the owner echoes it onto the forwarded Data) — it still targets the **L1D
owner**; and an **`InvAck`** an L1I sends targets the **requester's L1D** (which collects it). So the
core-side router keys on **op + is_icache**, not `is_icache` alone.

**Result:** No error / deadlock-free at **2 cores (805,861 states)**. Coverage probes (an L1I never
reaches `IC_S`; the directory never tracks an `i_sharer`) both **fail** → the L1I/D2 path is exercised.
3 cores is not run — the L1I doubles the per-core agent count, exceeding available RAM.

## v4b — LR/SC + AMO (`moesi_ccd_v4b.m`)

v4b adds the **atomics** to v1's serialised base, validating the ratified contract:
- **LR** reserves a readable line (snapshotting the value read). A **remote write** (`FwdGetM`/`Inv`)
  kills every *other* core's reservation (§9.10 coherence-kill); a remote *read* (`FwdGetS`) does
  not. An own store/evict also drops the reservation. **SC** succeeds iff its reservation survived.
- **AMO** (§13.9c) acquires M, **RMW-reads** (the `amo_lock`), and **writes at commit**. **Any** remote
  snoop while it holds/acquires M **squashes-and-replays** it (it loses exclusive M → no stale commit).

**Atomicity without an epoch counter (the state-economy lever):** LR and AMO snapshot the value they
read (`rsv_val` / `amo_rd_val`, bounded `Value`s), and SC-success / AMO-commit **assert** the value is
unchanged — which holds because the op holds the line and any intervening write would have killed the
reservation / squashed the AMO. A free-running `WriteEpoch` would have exploded the state space; this
keeps it at 121K states.

Invariants: SC-atomicity + AMO-atomicity (inline asserts), reservation soundness (`rsv_valid ⇒ rsv_val
= LastData`), an exclusive M/E owner excludes every other reservation, `amo_locked ⇒ M`, at-most-one
`amo_locked`. **Result:** No error / deadlock-free at **2 cores (121,194 states)**. Coverage probes
confirm LR reservations, AMO locks, and the squash-replay all fire; a **negative control** that removes
the reservation-kill fails "exclusive owner excludes other reservations" → the kill is load-bearing.

## v4c / later (not yet modelled)
- **L1I + atomics together** (v4a ∪ v4b), and **3-core** runs of v3/v4a/v4b — need a bigger-RAM host
  or CMurphi hash-compaction than this ~19 GB machine.
- **multi-address**, the **per-VC network-deadlock** model, and an explicit **NINE-L2-evicted**
  data-source negative control (B12).
