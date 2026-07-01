# niigo-lake → OpenROAD + ASAP7

Open-source RTL→GDSII flow for `niigo_soc` on the ASAP7 7 nm predictive PDK, using OpenROAD's
integrated synthesis (yosys-slang → ABC). **Read [`SCOPING.md`](SCOPING.md) first** — it is the
authoritative analysis of what changes are needed and why.

## Status
- ✅ ASAP7 PDK organized; TT-corner Liberty extracted to `lib_tt/` (from `.7z`).
- ⏳ OpenROAD building from source → `/home/mizuki/Desktop/workspace/OpenROAD/build/bin/openroad`.
- 🧪 Flow scripts staged here — **grounded but not yet run end-to-end** (pending the binary).
- ❌ Still needed for full P&R: the ORFS `flow/platforms/asap7/` glue (setRC / tracks / pdn /
  dont_use / KLayout layermap). See SCOPING.md §6.

## Files
| File | Purpose |
|---|---|
| `SCOPING.md` | The scoping report: verdict, RTL changes, memory mapping, flow, phased plan, risks. |
| `lib_tt/` | ASAP7 TT-corner standard-cell Liberty (20 `.lib`; RVT subset is enough for bring-up). |
| `gen_filelist.sh` | Emits `output/openroad/design.f` (slang command file) for `sv_elaborate`. |
| `flow_asap7.tcl` | Tier-1 all-standard-cell flow: elaborate→synthesize→floorplan→place→CTS→route. |
| `constraints.sdc` | Clock + AXI IO + reset false-path + max_fanout skeleton. |
| `dont_use.tcl` | ASAP7 cells the mapper must avoid (fill/decap/tap/tie/latch/ICG/scan). |
| `run.sh` | Driver: `./run.sh synth` (self-contained) … `./run.sh route` (needs the platform). |

## Quick start (once `openroad` is built)
```sh
# Synthesis sanity (self-contained: confirms the SV frontend clears + maps to ASAP7 cells):
./run.sh synth
# Full P&R (after lifting the ORFS asap7 platform RC/tracks/pdn values into flow_asap7.tcl):
./run.sh route
```

## Key facts (verified)
- **Use the 1× ASAP7 collateral everywhere** (tech LEF read FIRST). Mixing 1×/4× = 16× area error.
- `synthesize` always flattens + renames macro instances to `uNNNN`; use external Yosys if you
  need named/hierarchical SRAM macros.
- The `asap7_sram_0p0` macros reference `SITE coreSite`, which is **undefined in the PDK** —
  define/remap it before any SRAM-macro floorplan.
