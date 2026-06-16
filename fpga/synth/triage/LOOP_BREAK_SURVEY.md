# FB2b — false-comb-loop break survey (pipeline-split plan)

**Date:** 2026-06-16. **Tree:** `9b42578` (decomposition committed `5aa3871`, B-pipe reverted).
**Goal:** break all 26 Verilator UNOPTFLAT / 23 Vivado LUTLP-1 false combinational loops for a clean,
routable F2 netlist + trustworthy STA.

## HEADLINE — correcting my earlier call

The loops do **NOT** need the IPC-costly `writeback→wakeup` register. **All 26 are FALSE whole-block
aliases**, breakable **value-identically (no registers, zero IPC)** by decomposing a handful of
monolithic `always_comb` blocks — the same technique as the dispatch-block decomposition (`5aa3871`),
applied *inside* the IQ / LSQ / writeback-bus / B1 / B2 instead of at the top level.

Earlier I saw the binding loop's example path thread `spec_wake → … → writeback` and concluded it was
the classic single-cycle wakeup→select loop (which needs a register). **That was wrong.** The
issue→execute S2 register (`99f3f12`) *already* broke the real wakeup→select loop: the ALU reads its
operands from the **registered** `alu_issue_entry_q`, and `spec_wake` is broadcast from that registered
S2 entry. So `branch_writeback ← registered`, `resolve ← registered`, `stack_abort_mask ← registered`
— **every signal in the SCC is a combinational function of registered state** (`entries_q`,
`alu_issue_entry_q`, `meta_q`, `head_q`, …) **+ registered module inputs** (`decode_lanes`,
`data_load`). There is **no real combinational cycle** (consistent with the design settling in sim and
passing exhaustive verification). The 26 loops are tool artifacts of whole-signal/array dependency
granularity over big `always_comb` blocks.

## Why the earlier value-identical attempts didn't drop the count

The dispatch decomposition (33→26) split B1–B7 but left the **module-internal** aliases intact, and the
two top-level sub-breaks (mem-operand split, phys_rs←map_prs) didn't touch the modules where the cycle
actually closes. The SCC reroutes through any un-split aliasing block, so the count holds until **every**
aliasing block on a cycle is split.

## The false aliases on the dominant cycle (and the split that removes each)

Verilator's example path (threads nearly all 26 loops):
```
dispatch_issue_entries → int_issue_entry → phys_rs1 → phys_rs1_data
  → load_writeback → branch_writeback → lane_control_predicted → dispatch_issue_entries
```
Edge-by-edge:

| edge | module / block | real? | why | break |
|---|---|---|---|---|
| `dispatch_issue_entries → int_issue_entry` | int_issue_queue (62-221) | **FALSE** | the block reads `insert_entry` (insert, line 196) and writes `issue_entry` (select, line 143); select runs **before** insert, so `issue_entry` doesn't depend on `insert_entry` | split squash+wakeup / select(→issue_entry) / insert / count into separate blocks |
| `int_issue_entry → phys_rs1 → phys_rs1_data` | phys_rd + regfile | real (forward) | MUL/DIV/FP read regfile at select; this is dispatch→execute, not a cycle | n/a |
| `phys_rs1_data → load_writeback` | load_store_queue (191+) | **FALSE** | the block reads insert operands (`mem_insert_rs*_data`) and writes `load_writeback`, but `load_writeback` is formed from the **registered** `headq = entries_q[head_next]` + `data_load` | split `load_writeback` formation out of the insert/entries_next block |
| `load_writeback → branch_writeback` | ooo_writeback_bus (41) | **FALSE** | `branch_writeback` is assigned inside the arbitration loop that reads **all** sources incl. `load_writeback` (line 99), but only ALU sources ever set `branch_valid` | compute `branch_writeback` from `alu0/alu1_writeback` only, in a separate block |
| `branch_writeback → lane_control_predicted` | riscv_core_ooo B2 (1599) | **FALSE** | B2 reads `branch_writeback` (GHR speculative-history update) and writes `lane_control_predicted`; the latter doesn't depend on the former | split the GHR/branch-history update out of B2 |
| `lane_control_predicted → dispatch_issue_entries` | riscv_core_ooo B3 | real (forward) | `lane_control_predicted` is a field of the rename packet | n/a |

Secondary cycles (same false-alias nature):
- `dispatch_valid → lane_valid` (B1 reads `dispatch_valid` for `dispatch_count`, writes `lane_valid` from `decode_lanes` only) → **split B1**: lane-decode (decode_lanes) vs dispatch-count (dispatch_valid).
- `wakeup_valid/writeback_prd → busy_src_ready → dispatch_issue_entries` → opens once the IQ insert→issue alias is cut (busy feeds the rename `src_ready` field; the cycle closes through the IQ alias).
- commit cycle `stack_reset_mask → active_commit_valid → retire_valid → commit_take_trap → dispatch_valid → lane_valid` → opens via the B1 split (+ the commit chain is register-broken through `active_list.entries_q`).
- `ptw_pte_pmp_fault` (single-signal self-loop) → separate, small; inspect `ptw.sv`/`pmp_checker` wiring (likely a whole-block alias in the PTW or the PMP fault mux). Low priority (not on the binding path).

## Plan (value-identical, no IPC) — in leverage order

1. **int_issue_queue.sv** — HIGHEST leverage. Split the 160-line `always_comb` into: (a) squash+wakeup
   → `entries_wake`; (b) select → `issue_entry` + post-select entries; (c) insert → `entries_next`;
   (d) `count_next`. `issue_entry` then reads only (a)/(b), not `insert_entry`. This cuts the
   `dispatch_issue_entries → int_issue_entry` edge that is on **every** loop → expected to drop the
   count sharply by itself.
2. **load_store_queue.sv** — split `load_writeback` formation (registered `headq`/`entries_q` + `data_load`)
   from the insert/entries_next operand handling.
3. **ooo_writeback_bus.sv** — `branch_writeback` from ALU sources only, separate block.
4. **riscv_core_ooo.sv B2** — split GHR/branch-history update from lane-predict.
5. **riscv_core_ooo.sv B1** — split lane-decode from dispatch-count.
6. **ptw_pte_pmp_fault** — inspect + split if a whole-block alias; else leave (off the binding path).

Re-build after each (UNOPTFLAT count is the cheap progress metric); full-matrix verify (rv32 247 /
priv 28 / rv64 289 / xv6-$) at the end since each step is value-identical (bit-for-bit). When the count
reaches 0 (or only `ptw` remains), re-synth: the netlist becomes a clean DAG (LUTLP-1 gone), STA is
trustworthy, and the real worst path is finally measurable.

## Expected outcome + honest caveats

- **Routability/F2:** clean DAG, no LUTLP-1 CRITICAL — the real win, and it makes a robust F2 build
  possible without `ALLOW_COMBINATORIAL_LOOPS`.
- **WNS:** the false loops carry **no logic depth**, so breaking them does **not** lower the real WNS;
  it makes the *true* worst path visible (fresh-place showed ~−12 ns LSQ→IQ-count). 125 MHz still then
  needs the real logic-depth work (B-pipe and/or scheduler pipelining) — but now those are *measurable
  and meaningful* instead of masked by the phantom loop. **Zero IPC is spent on the loop break itself.**
- **Risk:** if any edge I classified FALSE is actually real, that loop won't break (it reroutes) —
  caught immediately by the count not dropping after its split. Fallback for a genuinely real edge is a
  targeted register (IPC). Confidence is high (the registered-derived argument is rigorous and the
  design provably settles), but items 5/6 (B1 commit cycle, ptw) are the least-verified.
- **Effort:** ~4-6 module decompositions, each the same low-risk pure-code-motion pattern as `5aa3871`.

## UPDATE 2 (2026-06-17): 26->6 done; route experiment; LSQ split fully designed

**Done & verified (26->6, zero IPC):** d7d642c IQ+wb-bus, 99bfc8a LSQ(block1/2)+B1+B2c, d4629f9
branch_stack, abeec37 active_list, f2f99a4 branch_stack restore_valid flush-gate drop. rv32 247 /
priv 28 / rv64 289 / xv6 boot-$.

**"Attack the route" PROVED the loops must be broken first:** fresh synth + default place_design +
phys_opt_design + ALLOW_COMBINATORIAL_LOOPS gave WNS -19.65/-19.53 -- WORSE than Quick's -12.27. The
aggressive placer chases the false-loop phantom paths (high-fanout, unplaceable) and degrades WNS. So
the -12 was a lucky loop-cut; the route is unattackable while the loops live. AND the loops INFLATE
WNS -> a clean DAG may reveal a real WNS << -12. So the LSQ split is UNAVOIDABLE for any real number.

### LSQ wakeup<->load_writeback split -- FULL DESIGN (the remaining WNS-relevant loop)

The alias: load_store_queue block 1 reads wakeup (squash/wakeup writes entries_premerge.src_ready/addr)
AND writes load_writeback (head-ops). load_writeback is registered-derived (headq=entries_q[head_next]
+ data_load + reservation_q) -- BUT head_next itself aliases wakeup (the head-skip reads the post-
wakeup entries_premerge[k].valid). So a clean break needs a 4-block decomposition:

- **1a** (squash + wakeup + store-format): entries_q -> entries_wake + extract prem_valid[k] (1-bit).
  Reads wakeup. The ONLY wakeup reader.
- **1b** (head-skip): prem_valid -> head_next + count_after_skip. Reads prem_valid (NOT entries_wake)
  -> head_next no longer aliases wakeup.
- **1c** (head-ops): reads headq=entries_q[head_next from 1b] + xlate + reservation_q + data_load +
  mem_inflight_q (ALL registered) -> load_writeback + head_done + head_retire + head_delta + reservation
  _mid + load_data_en + load_data_addr + mem_inflight_next + store_probe_hi_next + double_store_next.
  NO entries_wake / wakeup read -> load_writeback is clean.
- **1d** (merge + store-commit): entries_premerge = entries_wake; apply head_delta to [head_next];
  store-commit (reads entries_q[head_next] + commit_store); data-port MUX (double_store < load_data <
  store_commit); reservation_next = reservation_mid then store-commit clears; count. Writes entries_
  premerge (NOT load_writeback) -> reads wakeup (entries_wake) but does not alias load_writeback.
- **block 2** (insert, unchanged): entries_next = entries_premerge + insert.

**~30 premerge signals threaded 1c->1d:** head_delta = retire flag + per-field {val,we} for issued_
load, store_lo_pa, store_hi_pa, store_data, store_mask, load_complete, load_low_word, double_low_valid,
addr (~9 fields x2). Plus reservation_mid, load_data_en/addr, mem_inflight_next, store_probe_hi_next,
double_store_{addr,data,mask}_next, head_next, count. SUBTLETIES: (a) AMO store_data is computed from
load_writeback.data in the current code -> 1d must recompute amo_loaded = format_load(data_load,...)
locally (1c's load_writeback.data is cross-block). (b) two-retire-per-cycle: load-complete retires
head N (head_next++), store-commit then operates on N+1 -> 1d's store-commit reads the post-head-delta
head_next. (c) the data_* memory port is single -> 1d muxes the load-issue drive (1c) under the store-
commit/double-store drive.

**RISK: a missed field/case = silent LSQ memory corruption that passes ACT but fails under stress.**
This is the single most intricate split in the campaign. Execute with FRESH context + verify rv32 247
/priv 28/rv64 289/xv6 + AGENT_DEBUG usertests stress. Then ptw (PtwPMP reads live ptw_mem_addr ->
ptw_pte_pmp_fault -> PTW: split ptw.sv so mem_addr is computed in a block not reading pte_pmp_fault,
OR register the PMP address +1 PTW-walk cycle). Then a clean DAG -> trustworthy STA -> measure the
real WNS (may be << -12) -> route/fanout/pipeline as needed.

## UPDATE 3 (2026-06-17): LSQ + ptw DONE -- UNOPTFLAT 6 -> 1

**LSQ wakeup<->load_writeback (d863562):** done as a 3-block split (HD / W / M), SIMPLER than the
4-block design above. HD (head-skip + headq + per-op head blocks) reads entries_q + abort_mask
(inline squash-valid) + registered state only -> load_writeback + head_delta (sparse per-field
write-enables). W (squash + reset + wakeup + store-rederive) is the sole wakeup reader -> entries_wake.
M layers head_delta onto entries_wake + store-commit + data-port mux. The AMO store_data is kept in HD
(local to where load_writeback.data lives), so NO amo recompute in M was needed. head_delta's per-field
we_* (not whole-entry replace) makes M value-identical regardless of same-cycle wakeup on the head
entry. Result 6 -> 3 (the split also severed extra cycles routing through the LSQ).

**ptw_pte_pmp_fault / ptw_mem_req / ptw_mem_ack (ptw.sv):** done by splitting the PTW next-state
always_comb into 3 blocks: (1) mem_addr/mem_wdata -- read NEITHER pte_pmp_fault NOR mem_ack;
(2) mem_req/mem_we -- read pte_pmp_fault (A/D-write suppression) but NOT mem_ack; (3) next-state --
reads mem_ack/pte_pmp_fault, drives only state. Every memory output is a pure fn of registered state;
isolating mem_addr from pte_pmp_fault severs the PMP loop, isolating the request strobes from mem_ack
severs the request<->ack loop. mem_is_write (PtwPMP's other input) was already a registered continuous
assign. Result 3 -> 1.

**REMAINING (1): retire_valid (riscv_core_ooo.sv:394) -- the in-order commit cycle.** The active_list
ROB commit chain (stack_reset_mask -> active_commit_valid -> retire_valid -> commit_take_trap ->
branch flush). This is the last false loop; it is NOT on the LSQ/IQ critical path. Next: re-synth at 1
loop to read the REAL WNS (the route experiment proved the loops inflate WNS; with the DAG nearly clean
the binding path should finally be a true logic/route path, not a phantom).

## UPDATE 4 (2026-06-17): retire_valid broken -> UNOPTFLAT 26->0; TRUE WNS is REAL

**Last loop broken (47baf97):** ActiveList commit block read commit_taken (pop) AND wrote
commit_valid (present) -- the present runs before the pop, so the alias is false. Split into
C1-present (writes commit_valid, reads NO commit_taken) + C2-pop (reads commit_taken -> final
entries/head/count). Value-identical. UNOPTFLAT 1->0: DAG now fully combinational-loop-free.

**DEFINITIVE WNS (0 loops, trustworthy STA): -12.77 ns Quick-place (~48.5 MHz).** The worst path is
IDENTICAL across 26/6/1/0 loops: ALU writeback_reg[branch_mask] -> IntIssueQueue count_q, 75 levels,
77% route. **With 0 loops STA has nothing to unroll, so this path is REAL** -- the branch-recovery
broadcast (a resolved branch's abort_mask fans out to squash IQ/LSQ/ActiveList/BranchStack, then IQ
occupancy recomputes). The none-flatten 8-module thread I read as an "unrolled loop" was the GENUINE
broadcast cone touching all speculative structures.

**HYPOTHESIS REFUTED:** "false loops inflate WNS; clean DAG reveals a better number" was WRONG. The
loops corrupted STA ATTRIBUTION (the -42<->-12 phantom oscillation, endpoint mislabeling) but NOT the
binding WNS value, which was always this real ~-12.5 ns path. **What the campaign DID deliver (real,
and a prerequisite): UNOPTFLAT 26->0 = robust loop-free F2 netlist (no LUTLP-1) + trustworthy stable
STA so every future cut is measurable. Zero IPC, full matrix green.** Default place+phys_opt = -17
(WORSE), confirming logic-depth/fanout wall, not routing.

**TO 125 MHz (now a clean, measurable target):** attack the branch_mask -> abort_mask broadcast ->
squash -> IQ-count cone (75 lvl, 77% route). Levers: (a) reduce abort_mask compute depth; (b) split
the IQ occupancy count from the full per-entry squash; (c) replicate the high-fanout abort_mask driver
per consumer; (d) pipeline branch recovery (register abort_mask, +1 misprediction-recovery cycle, IPC).
(a)-(c) are the lower-risk value-identical-ish cuts to try first.

## UPDATE 5 (2026-06-17): value-identical timing opt EXHAUSTED -- the recovery cone is pipeline-bound

With the DAG clean (0 loops), attempted a value-identical depth cut on the -12.77 worst path
(ALU writeback branch_mask -> IntIssueQueue count_q, 74 lvl, 77% route): **IQ occupancy computed
INCREMENTALLY (count_q - squashed - issued + inserted) instead of popcount(entries_next.valid).**
Rigorously value-identical (squashed = entries_q.valid & abort-match; issued = popcount(issue_valid);
disjoint since a squashed entry reads valid=0 in entries_wake so can't be selected; inserted =
popcount(insert_valid) when !full). Verified rv32 247 / priv 28 / rv64 289 / xv6.

**RESULT: WNS -12.77 -> -12.75 = NEUTRAL.** The cut took count_q OFF the path (endpoint moved to
IntIssueQueue entries_q[imm]), but WNS held because the binding depth is the SHARED serial chain
abort_mask -> squash (entries_wake) -> select (priority picks, clears entries_sel) -> insert (ins_free
over the POST-select free mask, fills entries_next). count was just one tail branching off the END;
the insert tail is co-deep. **No value-identical change shortens this serial chain:** de-serializing
insert (fill PRE-select free slots so it doesn't wait for select) is NOT value-identical -- when an
issued slot is lower than a free slot, lowest_idx picks a different slot -> different issue priority
(architecturally correct, but not bit-identical, and a scheduling change can expose latent OoO bugs).

**CONCLUSION: value-identical timing optimization is EXHAUSTED at -12.75 ns (~48.5 MHz, full IPC).**
The recovery cone (a resolved branch's abort_mask broadcast -> per-structure squash -> IQ select +
compaction, all single-cycle) is ~74 levels / 77% route and is fundamentally PIPELINE-bound. The
count cut is kept as a building block (count off the path; a prerequisite for any later insert
reschedule to show benefit), but it does not move WNS alone. **To 125 MHz, two user-decided levers:**
(1) PIPELINE branch recovery -- register abort_mask so squash/select/insert span 2 cycles + a kill-bit
to block wrong-path issue (+1 misprediction-recovery cycle; IPC on mispredicts); (2) RESCHEDULE --
de-serialize insert-from-select (no IPC, architecturally correct, but not bit-identical -> re-verify
+ usertests stress). Both reshape the OoO surface -> supervised.
