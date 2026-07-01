# ASAP7-synth branch — implementation progress & results

Companion to `ASAP7_STATIC_ANALYSIS.md` (the static-analysis snapshot). This tracks what was
actually implemented on the `asap7-synth` branch, the measured impact, and the open work.

**Verification oracle:** Verilator functional tests — the ASAP7 flow only *elaborates* clean; timing is
*measured* (per-block `char.sh`), not gated. Each RTL change keeps green: **RV64G ACT 289/289** +
**RV64 Sv39 priv 14 PASS / 15 SELFCHECK / 0 fail** (the SELFCHECK split is the known-benign Sail
trap-budget artifact). Timing measured with `char.sh <par> <period_ns> <module>` (placed-parasitic,
period-independent Fmax; ASAP7 Liberty `time_unit=1ps`).

## Landed changes

| Commit | Change | Verified | Measured impact |
|--------|--------|----------|-----------------|
| `ec0db90` | **RAS 128 → 32 entries** (`riscv_core_ooo.sv`) | ACT 289/289 + priv 14/15 | Pure prediction hint (mispredict recovered at JALR resolve → architecturally inert). 4× cut on the `ras_stack` flop array (~6 Kbit at RV64) + 128:1 → 32:1 top-of-stack read mux. |
| `88b25e3` | **CVFPU FMA `PipeRegs` 2 → 3** (`niigo_fp_unit.sv`) | ACT 289/289 + priv 14/15 | **Whole-core binding floor 41.7 → 69.9 MHz (+67%)** — see sweep below. +1 FMA latency (negligible IPC: FP serialized + rare). |

### FMA PipeRegs sweep (niigo_fp_unit, char.sh @5 ns, placed parasitics)

| ADDMUL PipeRegs | Fmax | note |
|---|--:|---|
| 2 (was) | 41.7 MHz | the binding floor |
| **3** | **69.9 MHz** | **chosen — the knee** |
| 4 | 46.4 MHz | worse |
| 6 | 32.9 MHz | much worse |

Non-monotonic: CVFPU `DISTRIBUTED` cuts the *fixed* FMA datapath at internal boundaries, and 3 aligns
best with the mantissa-mul / align-add / normalize-round segments; 4+ split balanced logic unevenly and
add register area/congestion. The wrapper is fully handshake-based and pipeline-depth-agnostic, so this
is functionally transparent (CVFPU inserts the registers, the wrapper waits on `out_valid`).

## Tried & reverted (measure-before-commit)

- **ooo_mul_unit 64×64 `*` → registered 4×(32×32) partial products.** Bit-exact (identity verified over
  200k random RV32/RV64 + edges) and ACT 289/289, **but standalone Fmax REGRESSED 103.6 → 89.2 MHz** at
  the same 5 ns target. Root cause: a naive partial-product split forces serial *binary* CPAs at the
  register boundary + a 3-add recombine, which loses to the tool's monolithic Wallace-tree `*`. Reverted.
  **Lesson: pipelining a multiply for ASIC needs a carry-save (redundant sum/carry) register boundary,
  not binary sub-products** — otherwise you just move and worsen the bottleneck. (And the mul is not the
  binding block anyway — the FMA is — so there was no whole-core upside.)

## Whole-core Fmax floor: current state

After the FMA cut, the real per-block floor is:

| Block | Fmax | |
|---|--:|---|
| **niigo_fp_unit (FMA)** | **69.9 MHz** | new binding floor |
| branch_stack | 74.3 MHz | next, close behind |
| free_list | 102.4 MHz | |
| ooo_mul_unit | 103.6 MHz | |

The whole-core number itself is still unmeasurable (integrated-syn ABC scale wall at ~9 M ANDs). To move
the floor past ~70 MHz, **both** the FMA and branch_stack must rise (they bracket it).

## Memory-cell SRAM mapping — feasibility & the tooling blocker

The #1 *area* problem is that integrated-syn has no SRAM-macro step, so every inferred memory flop-maps
(L1 data ~262 Kbit, ITTAGE ~306 Kbit, etc.). A single-core L1-controller analysis established the
per-array mappability to the available **single-port (1RW)** ASAP7 macros:

| Array | Verdict |
|---|---|
| **L1I data** | **MAPPABLE-AS-IS to 1RW** — refill-only writes (full line), read/write states disjoint (`S_SERVE`/`S_REPLAY` read vs `S_INSTALL` write). No byte-write. |
| **L1D data** | **MAPPABLE-AS-IS to 1RW for R/W exclusivity**, but writes are **byte-masked** (store-hit, `l1_dcache.sv:296-299`) → needs **byte-lane banking** (8-bit sub-arrays) since 1RW macros are full-word-write. |
| **L1D tag** | **MAPPABLE-AS-IS to 1RW** — 2nd read port tied off (`l1_dcache.sv:92`); probe reuses the main port from `S_IDLE`; full-word write. |
| **L1I tag** | **NEEDS DUPLICATE-TAGS** — the C4 snoop read (`ren2`) runs concurrently with fetch reads every committed D-store, which a single 1RW macro cannot serve. Map as two 1RW copies (fetch copy clean; snoop copy needs 1-cycle arbitration vs `S_INSTALL` write). |

**RTL risk is zero** (the default inference stays bit-identical for Verilator; a macro path is
`ifdef NIIGO_SRAM_MACRO`-gated, synth-only). **The blocker is macro geometry/tooling, not RTL:**
- The shipped `asap7/asap7_sram_0p0` set is **1024-deep, ≥16-bit-wide, single-port, no byte-write** — it
  does **not** fit the L1 geometry (64 deep × 512 wide, byte-writable). LIB/LEF exist for several
  `srambank_*` geometries but the depths/widths are wrong and none is byte-writable or 8-bit-wide.
- The **FakeRAM2.0 generator is not installed**, so the needed geometries (e.g. 64×8 byte-lanes,
  64-deep full-line banks) cannot be generated here.

**Path to unblock (future work), in priority order:**
1. Obtain/build **FakeRAM2.0**; generate the L1 geometries (byte-lane 64×8 for L1D data; full-line banks
   for L1I data; narrow tag banks). Add their LIB/LEF to the flow, `NIIGO_SRAM_MACRO`-gate a banked macro
   path in `l1_data_array.sv`/`l1_tag_array.sv`, keep the default inference bit-identical.
2. Alternatively, **blackbox the array modules** in a `timing_flow.tcl` variant (empty blackbox + the
   nearest `srambank_*` LIB for a placeholder) purely to *measure the L1 controller logic Fmax* — this
   also unblocks the `l1_dcache`/`l1_icache` standalone synth that currently TIMEOUTs at 2400 s because
   of the flop-mapped array mass.
3. Handle **L1I tag** via the duplicate-tags structure (two 1RW copies, snoop-copy arbitration).

## Open levers (need a direction decision)

- **Push the FMA past 69.9 MHz** — requires a dedicated pipelined mantissa multiplier inside the vendored
  `fpnew_fma` (CVFPU internals). Higher risk (bit-exactness of vendored FP), the real remaining timing win.
- **Memory-cell SRAM mapping** — blocked on FakeRAM2.0 (see above); RTL is ready to receive it.
- **branch_stack two-stage split** (74.3 MHz) — measurable, but only helps the floor once the FMA also rises.
