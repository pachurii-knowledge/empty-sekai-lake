# ASAP7 single-port SRAM macros (memory-cell mapping)

FakeRAM2.0-generated single-port (1RW) SRAM macros used by the `NIIGO_SRAM_MACRO`
memory-cell mapping of the L1 arrays (see `src/mem/l1_data_array.sv`). Each is a
black-box for synthesis/P&R — real timing from the `.lib`, footprint from the `.lef`.
The Verilator/functional build does **not** use these (it keeps the inferred array).

## Generating

FakeRAM2.0 (pure-Python, no deps) generates `.lib`/`.lef`/`.v` for any (width, depth):

```sh
git clone https://github.com/maliberty/FakeRAM2.0.git   # cloned to ../../../../FakeRAM2.0
python3 FakeRAM2.0/run.py fpga/openroad/sram/niigo_sram.cfg --output_dir fpga/openroad/sram
```

`niigo_sram.cfg` carries the ASAP7 7nm process params (copied from the ORFS asap7
`fakeram.cfg`) plus the geometry list.

## Macros

| Macro | Geometry | Use |
|-------|----------|-----|
| `niigo_sram_64x8` | 64 words × 8 bits, 1RW | L1 **data** byte-lane. Per way, 64 lanes (`LINE_BITS/8`); the L1 store byte-mask maps to per-lane `we_in`. 4 ways × 64 lanes = 256 macros / data cache (131072 bits). |
| `niigo_sram_64x52` | 64 words × 52 bits, 1RW | L1 **tag** (RV64 `L1_TAG_BITS=52`), full-word write. 1 macro / way = 4 / cache. Used only for the **L1D** tag (`SNOOP_PORT=0`); the L1I tag keeps inferred dup-tags (its C4 snoop read can collide with a fetch read — a 1RW macro would need snoop arbitration, deferred). |

The `.bb.v` files are hand-written `(* blackbox *)` stubs (ports only) for
`sv_elaborate`; the FakeRAM `.v` is a behavioral model (unused here — it has a
non-overwriting OR-write and is only for gate sim).

## Measuring

`fpga/openroad/timing_flow.tcl` reads SRAM collateral when `SRAM_LIB`/`SRAM_LEF`
are set (colon-separated). Example (synth-only, shows the flop mass replaced):

```sh
TOP=l1_dcache FILELIST=output/openroad/design_sram.f STOP_AFTER=synth \
  SRAM_LIB=fpga/openroad/sram/niigo_sram_64x8/niigo_sram_64x8.lib \
  SRAM_LEF=fpga/openroad/sram/niigo_sram_64x8/niigo_sram_64x8.lef \
  openroad -exit fpga/openroad/timing_flow.tcl
```

`design_sram.f` = `design.f` + `+define+NIIGO_SRAM_MACRO` + the `.bb.v` stub
(regenerate: `awk '1;/\+define\+FPGA_BUILD/{print "+define+NIIGO_SRAM_MACRO"}' design.f`,
then add the stub path after `or_defines.svh`).

**Result (`l1_dcache`, synth-only):** goes from TIMEOUT(2400s) with inferred flop
arrays to synthesizing in seconds. With **data + tag** mapped (pass both libs/lefs
colon-separated): data = 256 `niigo_sram_64x8` (4403 µm²), tag = 4 `niigo_sram_64x52`
(447 µm²); the ~13.5k tag/data flops are gone, leaving only ~1942 control flops
(valid/dirty/PLRU/FSM — correctly not SRAM). **Total design area 7802 µm²**
(data-only was 13994 µm²; inferred did not synthesize at all). `l1_icache` likewise
synthesizes; its L1I tag stays inferred dup-tags (snoop arbitration deferred).
