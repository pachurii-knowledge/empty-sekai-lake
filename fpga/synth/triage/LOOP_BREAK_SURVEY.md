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
