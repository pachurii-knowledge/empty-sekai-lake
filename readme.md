# empty-sekai-lake

`empty-sekai-lake` or `niigo-lake` is a SystemVerilog RISC-V processor project with
both simple in-order cores and a 4-wide out-of-order core. The datapath width is a build-time switch threaded through every module from `RISCV_ISA::XLEN`: the base build is **RV32G** (Sv32), and `-DRV64` (`RV64=1`) selects a **RV64G** datapath with Sv39 virtual memory. The reference configuration of the out-of-order core — the one the microarchitecture description below assumes unless noted — is the `PERF=1` build: **RV64GC + Sv39** (RV64G plus the `C` compressed extension), with split L1 instruction/data caches and the full landed performance-lever stack. The base RV32G/RV64G builds and every individual lever remain build-gated (see *Build and Test*). Both widths carry the M/S/U privilege
levels and a small system-on-chip device bus (CLINT, PLIC, NS16550A UART). The current
verification focus is the out-of-order core, built with Verilator and run against the
RISC-V architectural tests at both widths.

This project aims to implement the ISA extensions, privilege levels, and
microarchitectural features required for booting real operating systems, and to scale to
a coherent multicore system with a directory-based MOESI cache hierarchy and a NoC
interconnect. As of this writing the RV64G out-of-order core **boots xv6-riscv (RV64G +
Sv39) to its interactive `$` shell both single-core and as a 2-core SMP system** — two
real out-of-order cores kept coherent by a grant-and-go MOESI directory — see *Bring-up
status* below.

The scalar core itself references my lab submissions for CMU's course 18-447,
Introduction to Computer Architecture (Spring 2025), and part of the peripherals,
memory/cache subsystems, simulation infrastructure, and testbenches are built upon lab
infrastructures of the same course.

`empty-sekai-lake` is named after the lake in `Nightcord at 25:00`'s Sekai.
## Bring-up status

- **RV32G** (base build): the OoO core passes `247/247` of the RV32G architectural
  suite plus the Sv32 privileged/translation tests, with no regression. This remains the
  live regression anchor through the RV64 migration.
- **RV64G** (`RV64=1`): the scalar and OoO cores pass the rv64 architectural suite
  (`289/289` unprivileged: `ui/um/ua/uf/ud`) plus the Sv39 privileged tests
  (signature-verified; see *Build and Test*). The reference `PERF=1` build (RV64GC OoO
  + L1D) additionally passes the wider **RV64GC** architectural suite `333/333`.
- **Operating system**: the `RV64=1 OOO=1` core boots **xv6-riscv** (RV64G + Sv39, no
  `C`) from M-mode reset, through `mret` into the S-mode kernel, Sv39 paging with
  hardware A/D updates (Svadu), the scheduler, `init`, and the shell, to the interactive
  `$` prompt — running e.g. `ls` against an in-RAM disk image. Reaching the shell drives
  enough paging and interrupt pressure to expose out-of-order edge cases the architectural
  tests do not: several were found and fixed in store-commit handshaking, load-return
  matching under paging, and page-walk result attribution.
- **Multicore / SMP**: two and four real `riscv_core_ooo` cores over the `niigo_ccd_memsys`
  coherent memory subsystem boot **xv6-riscv (RV64G + Sv39) SMP** to the interactive `$`
  shell and run `ls` — all harts reaching `$`, with or without the optional shared L2
  (2-core commit `b5e9bc6`); single-core CCD likewise boots (`0b00906`). The
  grant-and-go MOESI directory (`src/mem/niigo_dir_gg.sv`) + non-blocking L1D agents
  (`src/mem/niigo_l1d_gg.sv`) pass 2- and 4-core LR/SC-spinlock, AMO-atomicity, and
  cross-hart IPI litmus tests (`make ccd-smp-test` → mutual-exclusion counter == 6;
  `ccd-smp-amo-test`, `ccd-smp-ipi-test`, and their `-rv64`/4-core variants), plus a
  cross-core self-modifying-code (remote-dirty I-fetch) litmus (`ccd-smc-test`). The MOESI
  protocol is independently model-checked in CMurphi (`formal/moesi_ccd*.m`). The
  coherent CCD is validated in RTL/Verilator simulation (FPGA emulation of a >2-core
  cluster is descoped pending a larger target part).
- **FPGA emulation**: The full core and the full SoC (`niigo_soc` = core + L1 caches + memory subsystem + AXI4-512 master) synthesize in Vivado for the AMD Virtex UltraScale+ HBM VU47P (AWS F2 instances), and are wrapped into the F2 Custom Logic shell (`fpga/rtl/cl_niigo.sv`) with an OCL control plane, a virtual UART console, and a post-mortem debug window for on-card bring-up. Timing closure (tracked as FB2b) currently clears **62.5 MHz at Quick-place** (worst negative slack ≈ −7.7 ns ⇒ 63.7 MHz) after three targeted pipeline cuts: a 2-stage ROB commit, a registered LSQ head-translate, and a 2-stage CVFPU FMA, versus an earlier **~60MHz post-route** measurement; the routed/on-card confirmation is the next build. Target frequency is 125MHz. (FPGA
synth/timing covers the single-core `niigo_soc`; the multicore coherent CCD is validated
in RTL/Verilator simulation : see *Multicore / SMP* above.)

## Future Roadmap

- **Linux boot support on FPGA**: The immediate goal after successful synth and P&R on the VU47P is booting a minimal Linux kernel.
  Required ISA extensions and peripherals are already implemented.
- **IPC Optimization**: Microarchitecturally, the identified bottlenecks are as follows:
  - TAGE-SC-L correctness: The current branch predictor (`src/tage_sc_l_predictor.sv`) is only an approximation of the structure proposed in Seznec14. The goal is to implement the 32Kb variant after complete validation on VU47P at target frequency.
  - VIPT L1D/I caches: Currently both caches are PIPT, requiring a TLB lookup and then PMP permission check before cache access. The caching and VM paging geometries allow alias-free L1$ indexing using virtual addresses instead of physical addresses, thus parallelizing the TLB lookup with L1$ access and saving one cycle off L1 hits. This will become the next critical path after the current LSQ path is broken up.
  - Cache parameter sweep: Self-explanatory.
- **Multicore scale-out**: the 2- and 4-core directory-coherent cluster is **built and boots xv6-SMP to the interactive `$` shell** in RTL/Verilator simulation today — all four harts reach `$` and run `ls`, with or without the optional shared L2 (see *Bring-up status* and *Multicore cache coherence (CCD)* below); the NMI line bus is now the committed transport under a full grant-and-go MOESI directory + CMI coherence layer, with the shared L2 as an optional tier on its memory leg. The remaining work is (a) the deferred directory-robustness hardening (P5: WB-vs-Fwd evict-snoop, snoop-slot/ServeDeferred liveness, snoop-drain stale-Inv, directory-capacity broadcast-Inv), torture/RVWMO-litmus stress, and usertests-under-SMP; and (b) FPGA emulation of the cluster — a multi-core CCD does not fit a single VU47P and AWS F2 has no PCIe P2P, so a larger target part may be needed.
  
## ISA Coverage

XLEN is selected at build time (`-DRV64`) and threaded through every module from
`RISCV_ISA::XLEN`; the privileged and MMU RTL is shared between the two widths and
selects its geometry off `MXLEN == XLEN`.

### RV32G (base build)

- `I`: base RV32I integer instructions
- `M`: integer multiply/divide
- `F` and `D`: single- and double-precision floating point
- `Zaamo` and `Zalrsc`: atomic memory operations and LR/SC
- `Zicsr`: CSR instructions
- `Zifencei`: instruction fence

### RV64G / RV64GC (`RV64=1`, `+RVC=1`)

The RV32G coverage above, widened to 64-bit, plus the RV64-only instructions:

- `I`: 64-bit base integer; the `W` (32-bit, sign-extended) forms
  `ADDW/SUBW/SLLW/SRLW/SRAW` and `ADDIW/SLLIW/SRLIW/SRAIW`; 6-bit shift amounts; and
  the doubleword/word loads and stores `LD`, `SD`, `LWU`.
- `M`: 64-bit multiply (`MULH` family on the full 128-bit product) and divide, plus the
  `W` forms `MULW`, `DIVW/DIVUW`, `REMW/REMUW`.
- `A`: 64-bit (`.d`) atomics, LR/SC, and AMOs alongside the 32-bit (`.w`) forms.
- `F`/`D`: CVFPU built in the `RV64D` configuration, adding the 64-bit integer
  conversions `FCVT.L/LU.{S,D}` and `FCVT.{S,D}.L/LU` and the 64-bit moves
  `FMV.X.D`/`FMV.D.X`.
- `C`: the RVC compressed (16-bit) instruction extension — RV64-only, gated by `RVC=1`
  (`-DRVC`). It is **enabled in the reference `PERF` build**, making that configuration
  **RV64GC**. Compressed parcels are expanded to their 32-bit encodings in a pre-decode
  realigner (2 lanes by default, 4 under `REALIGN4`, which `PERF` also enables).

### Privileged architecture (both widths)

- `M`, `S`, `U`: machine, supervisor, and user privilege modes
- `Svbare`: bare (untranslated) addressing when `satp.MODE = 0`
- `Sv32` (RV32) / `Sv39` (RV64): page-based virtual memory — two-level / 4-byte PTEs at
  RV32, three-level / 8-byte PTEs at RV64
- `Svadu`: optional hardware A/D-bit updating, gated by `menvcfg.ADUE`

### Validation snapshot

```sh
# RV32G (base OoO build): expected 247/247
make verilator-build OOO=1
python3 scripts/run_riscv_suite.py \
  --extensions I,M,F,D,Zaamo,Zalrsc,Zicsr,Zifencei \
  --jobs 8 --timeout 120 --output output/riscv-tests-rv32g

# RV64G (RV64=1 OoO build): expected 289/289 unprivileged
make verilator-build RV64=1 OOO=1

# PERF (reference config): RV64GC OoO + L1D + full lever stack — expected RV64GC ACT 333/333
make verilator-build PERF=1
python3 scripts/run_riscv_suite.py \
  --elf-dir references/riscv-tests/work/niigo-rv64gc/elfs \
  --extensions I,M,F,D,Zaamo,Zalrsc,Zicsr,Zifencei,Zca,Zcd,MisalignZca \
  --jobs 8 --timeout 120 --output output/riscv-tests-rv64gc
```

## Microarchitectural Overview

The out-of-order implementation is centered on `src/riscv_core_ooo.sv`. It fetches and
decodes up to four instructions per cycle, renames architectural integer registers onto
a 128-entry physical register file, tracks in-flight work in a 64-entry active list, and
issues operations to independent functional units. The datapath, register file, and
pipeline structures are all `XLEN`-wide, so the same RTL builds as either a 32- or
64-bit core.

Unless a value is explicitly marked otherwise, this section describes the out-of-order
core in its **reference `PERF=1` configuration** — **RV64GC + Sv39** with split L1
instruction/data caches and the full landed performance-lever stack. Where a perf lever
changes a size or feature, the `PERF` value is given first with the base default (and the
flag that sets it) in parentheses; the base RV32G/RV64G defaults and every individual flag
are documented under *Build and Test*.

The main OoO sizing parameters (base defaults in `src/ooo_types.vh`, shown here at their
`PERF` values with the base default and enabling flag in parentheses):

- Dispatch width: `4`
- Physical integer registers: `128` (base default `64`; `DEEP_WINDOW`)
- Active list entries: `64` (base default `32`; `BIG_ROB`, which requires `DEEP_WINDOW`)
- Integer issue queue entries: `24` (base default `16`; `BIG_IQ`)
- Memory queue entries: `32` (base default `16`; `BIG_LSQ`)
- Branch checkpoints: `4` (unchanged under `PERF`; `BIG_BSTACK` doubles this to `8` but is
  measured-neutral and is not part of the umbrella)
- Issue ports: ALU0, ALU1, ALU2, MUL, DIV, FP (the third ALU port, ALU2, comes from
  `ALU4`, folded into `PERF`; the base build has two ALU ports)
- Writeback sources: ALU0, ALU1, ALU2, load/store, multiply, divide, FP (ALU2 present under
  `PERF`; the base build has no ALU2)

The build also provides a small in-order scalar core (`src/riscv_core_scalar.sv`, the
default-flag core and the first target for each RV64 datapath feature) and a
conservative 4-wide in-order core (`src/riscv_core_4wide.sv`, `SUPERSCALAR=4`). The
`src/riscv_core.sv` wrapper instantiates one of the three based on the build flags.

### Frontend

The frontend tracks a 16-byte aligned fetch stream and presents four instructions
to `ooo_fetch_decode`. Under the canonical RV64GC (`PERF=1`) build the RVC compressed
frontend expands 16-bit parcels before decode, with the realigner widened to four lanes
(`REALIGN4`, up from the base two) so the compressed stream can fill all four dispatch
slots. Control-flow prediction is split across a fetch-directed target buffer plus
direction and indirect-target predictors:

- A fetch-directed branch target buffer (`BTB`, part of the `PERF=1` umbrella) steers
  fetch toward predicted-taken targets ahead of decode.
- `tage_sc_l_predictor.sv` handles conditional direct branches.
- `ittage_predictor.sv` handles indirect targets.
- A return-address stack in `riscv_core_ooo.sv` predicts returns.
- `branch_stack.sv` checkpoints rename/free-list state for speculative branches.
- `ooo_branch_recovery.sv` creates redirect and abort masks on mispredicts.

Unpredicted control transfers, serializing instructions, terminal instructions, and
branch recovery can suppress dispatch until the architectural state is safe to continue.

### Rename, Dispatch, and Recovery

Decoded lanes flow through `ooo_dispatch_control.sv`, which enforces the 4-wide dispatch
rules, stops at serializing instructions, limits multiple memory/control instructions
where required, and checks resource pressure. Rename state is maintained by:

- `rename_map_table.sv` for architectural-to-physical mappings
- `free_list.sv` for physical register allocation and release
- `busy_table.sv` for operand readiness and wakeup tracking
- `active_list.sv` as the reorder buffer for precise commit

Branch snapshots include active-list tail, free-list pointers/count, map table state, and
RAS depth. A mispredict restores those snapshots and uses branch masks to squash younger
work.

### Issue and Execution

Integer and FP operations are inserted into `int_issue_queue.sv` with a functional-unit
class. The core has three ALU issue slots (ALU0/ALU1/ALU2) in the canonical `PERF=1`
configuration — the base build has two, and the third is enabled by `-DALU4` (folded into
the `PERF=1` umbrella) via a parameterized N-way ALU pick, which relieves the two-port
throughput ceiling on integer-ILP-heavy code (CSR ops stay confined to ALU0/ALU1) — plus
dedicated multiply, divide, and floating-point paths:

- `ooo_alu_pipe.sv` handles simple integer ALU, branch, CSR, the `W`-form ALU ops, and
  simple FP move/classify style results.
- `ooo_mul_unit.sv` handles `M`-extension multiply operations (incl. `MULW`).
- `ooo_div_unit.sv` handles `M`-extension divide/remainder operations (incl. the `W`
  forms).
- `niigo_fp_unit.sv` handles IEEE floating-point operations through CVFPU.
- `load_store_queue.sv` handles loads, stores, atomics, FP loads/stores, doubleword and
  misaligned split beats, and LR/SC reservation state (in coherent multicore builds the
  reservation is snoop-killed by remote stores and the `SC` is resolved at commit by the
  L1D agent — see *Multicore cache coherence (CCD)*).

Results arbitrate through `ooo_writeback_bus.sv`, which selects up to four writeback
packets per cycle (`OOO_WIDTH`) from the functional-unit sources — the three ALU pipes,
load/store, multiply, divide, and FP under `PERF=1` — and forwards wakeups to dependent
instructions.

### Floating Point and CVFPU

Floating point is implemented by a wrapper around the CVFPU/FPnew implementation vendored
under `src/cvfpu/`. The wrapper in `src/niigo_fp_unit.sv` translates the local `fp_op_t`
control encoding into CVFPU operations, formats, rounding modes, integer formats, and
status flags.

The instantiated CVFPU configuration is selected by XLEN — `RV32D` for the 32-bit build
and `RV64D` for the 64-bit build — so both single- and double-precision operations are
available, and the RV64 build additionally exposes the 64-bit integer conversions and
moves. The wrapper includes small request/result buffers to decouple the core
issue/writeback handshakes from CVFPU's valid-ready interface.

For timing closure on FPGA the CVFPU FMA (ADDMUL) datapath is configured with two
distributed pipeline registers, splitting the multiply-add across two stages (one extra
cycle of FMA latency); FP is infrequent in the integer-heavy OS workloads, so the cost is
negligible. In the canonical `PERF=1` configuration FP dispatch is de-serialized
(`-DFP_OOO`): a single-producer architectural-FPR scoreboard — one in-flight writer per FP
register, enforced by a WAW dispatch-stall with the producer's speculative branch mask
aged on branch recovery, plus an fflags-read drain interlock — replaces the machine-wide
dispatch quiesce the base build raises on every FP op. (In the base build FP issue is
serialized to a single in-flight operation so that an in-flight FP op's speculative branch
mask is aged correctly on branch recovery.)

The project also vendors small compatibility cells under `src/common_cells/`:

- `registers.svh`
- `lzc.sv`
- `rr_arb_tree.sv`

These provide the subset of `common_cells` functionality needed by the local CVFPU
integration.

### Memory and Commit

The load/store queue tracks memory operations independently from the integer issue queue
and produces load writebacks through the common writeback bus. Stores are made
architecturally visible only after the commit unit authorizes the active-list entry,
preserving precise state. The queue works in virtual addresses; the head access is
translated by the MMU (see below) and the resolved physical word address is applied at
the memory port; the head access's translation (DTLB + data-PMP) result is registered
and consumed on the following cycle to shorten the FPGA critical path. In the canonical
`PERF=1` configuration this registered head-translate stage is bypassed (`-DXLATE_BYPASS`)
for a DTLB-hit plain load, which then issues the same cycle it presents; this is the
single biggest L1D performance lever but re-opens the LSQ-head→DTLB→DataPMP→issue path (an
Fmax cost), so an FPGA/ASIC build drops it while keeping the registered stage.

Misaligned loads and stores are supported without trapping (`MISALIGNED_LDST`). The queue
classifies each access from its size and byte offset and, when an access crosses a
machine-word boundary, splits it into two word-granular beats — the simulated memory is
word addressed (4-byte words at RV32, 8-byte words at RV64) and writes a single word at a
time:

- Cross-word loads read the low word, then the high word, and recombine and
  sign/zero-extend the requested bytes.
- Cross-word stores first probe the high word's translation before writing either word,
  so a page fault on the second page is reported as the store's exception and no partial
  write occurs (store atomicity, including the page-crossing case).
- Misaligned atomics (`AMO`/`LR`/`SC`) are never split; an unaligned address raises an
  access fault with `mtval` set to the virtual address (`MISALIGNED_AMO = false`,
  `LRSC_MISALIGNED_BEHAVIOR = always raise access fault`).

`ooo_commit_unit.sv` retires completed active-list entries in order, frees old physical
registers, commits stores, updates architectural debug state, and reports precise halts
or exceptions. Floating-point retire updates the architectural FP register array and
accumulates `fflags` through the CSR file. A synchronous exception, interrupt,
`mret`/`sret`, or `sfence.vma` taken at commit flushes the pipeline and redirects the
frontend, keeping architectural state precise.

### Caches and the memory subsystem

The OoO core speaks three handshaked, latency-agnostic ports into the memory subsystem
(`src/mem/niigo_memsys.sv`): an instruction-fetch port (one 16-byte block at a time), a
word-granular data load/store port, and a page-table-walk request/ack port. In the
reference `PERF` configuration these ports sit behind a real split L1I + write-back L1 data
cache — the canonical memory boundary detailed below — with the PTW routed through the L1D
so page tables stay coherent. The build flags select what backs the ports: `L1D=1` is
`PERF`'s setting (write-back L1D + PTW-through-L1D); `L1=1` adds only the L1 instruction
cache; `L2=1` interposes a shared write-back L2 on the memory-side backend, behind the L1s
(see below); `AXI=1` routes the cache miss/writeback traffic through an NMI→AXI4-512 bridge
onto a simulated AXI slave. With no cache flags — the base-default OoO build used by the
correctness baselines — the ports instead wire straight through to the word-addressed
`main_memory` model (legacy fixed-latency timing), a passthrough variant. Devices
(CLINT/PLIC/UART) always bypass the caches.

The L1I and L1D share one geometry (`src/mem/niigo_mem.vh`, `l1_icache.sv`/`l1_dcache.sv`,
`l1_data_array.sv`/`l1_tag_array.sv`):

| Parameter | Value |
|---|---|
| Capacity | 16 KiB each (L1I, L1D) |
| Organisation | 4-way set-associative, 64 sets, 64 B (512-bit) lines |
| Indexing | VIPT, alias-free; physically tagged |
| Replacement | tree-PLRU (3 state bits per 4-way set, `l1_plru.sv`) |
| Tag/data arrays | synchronous-read (infer BRAM on FPGA) |
| L1I refill | read-only; refill on miss, snoop-invalidated for I/D coherence |
| L1D write policy | write-back, write-allocate, per-line dirty |

The L1 way size is exactly one Sv32/Sv39 base page (64 sets × 64 B = 4096 B), so the cache
index lies entirely within the page offset and is identical in the virtual and physical
address. The caches are therefore virtually indexed but synonym-free, with the physical
tag still resolving aliases. This `L1_VIPT_ALIAS_FREE` property is `initial assert`-checked
in the cache RTL, so the geometry cannot silently grow past a page and break VIPT.

An optional **shared L2** (`src/mem/niigo_l2.sv`; enabled by `L2=1` / `-DL2_CACHE`, default
off) can be interposed on the memory-side backend, below the L1s. It is a transparent,
write-back / write-allocate, **NINE** (non-inclusive/non-exclusive) line cache — 512 KiB,
8-way, 64 B PIPT lines, 8-way tree-PLRU (`l2_plru.sv`) — reusing the same synchronous-read
tag/data arrays as the L1s (`l1_tag_array.sv`/`l1_data_array.sv`, at 1024 sets × 8 ways). It
is an NMI slave upward and an NMI master downward, so it drops onto the *shared* memory
boundary of the subsystem: in the single-core cache builds, between the L1I/L1D-or-directory
arbiter and `main_memory`; in the multicore CCD path, on the directory's single NMI master
(below). Placing it on the merged request stream — rather than one L1's leg — keeps it
coherent with the L1I refill path, so a store written back into the L2 is still seen by a
later I-fetch (an L2 on the directory leg alone would be bypassed by the separate L1I refill
and break `fence.i`). The L2 is *value-transparent*: it changes only memory-leg latency, so
architectural results are identical with it on or off, and every build is bit-identical when
it is off. It is validated on both paths — the single-core CCD suite (`247/247`, including
`fence.i`) and the **4-core xv6-SMP boot to `$` + `ls`** both pass with the L2 on, and
`make ccd-l2-test` unit-checks the cache directly (fill/hit, dirty-victim writeback
round-trip, write-allocate, write-read coherence).

With `L2=1` unset there is no L2: L1 misses and writebacks travel line-granular over the
internal line bus (NMI) directly to `main_memory`, or — under `AXI=1` — through the
NMI→AXI4-512 bridge.

Instruction/data coherence for self-modifying code is maintained with *and* without
`fence.i`. `fence.i` writes back the L1D and then invalidates the L1I. Independently, a
committed data store snoop-invalidates any stale L1I copy of the written line (via a second
read port on the L1I tag array), and an L1I refill is held until the L1D first writes back
any dirty copy of that line (clean-before-refill). This keeps cold code and
patch-then-call-later sequences (e.g. xv6 `exec`) coherent with no explicit fence; only
tight in-place patching of an *already-fetched* line still needs `fence.i`. The load/store
queue allows a single outstanding load and tracks it by an in-flight tag, so cache
responses need no address matching at any latency.

### Multicore cache coherence (CCD)

Multiple OoO cores share memory through a directory-based MOESI coherence fabric
(`src/mem/`, design tracked in `plans/multicore-ccd.md`). It is built under `CCD_AGENT`
and gated so that single-core / `COHERENT=0` builds are bit-identical to the
pre-coherence core (the CCD modules are otherwise parsed but not elaborated). The protocol
carries the five stable MOESI line states — Invalid, Shared, Exclusive, Owned
(dirty-shared, the reason it is MOESI rather than MESI), and Modified
(`CMI_I=0 … CMI_M=4`, `src/mem/niigo_cmi.vh`) — and is machine-checked in CMurphi.

- **L1D agent** (`niigo_l1d_gg.sv`): each core's L1D is a non-blocking, direct-mapped
  (64 sets × 64 B lines in the configured builds) MOESI agent with one outstanding demand
  transaction (an MSHR), acquire/evict transients, deferred snoops (the acquire-side
  deferred-snoop matrix), and an ack-to-requester down-counter for write atomicity.
- **Directory** (`niigo_dir_gg.sv`): the per-line coherence/serialization point. It is a
  set-associative metadata directory (256 sets × 4 ways in the configured builds — far
  larger than the L1s to avoid directory-capacity evictions) that tracks each line's MOESI
  state and sharer vector but holds **no data of its own** — line data is sourced
  cache-to-cache (owner forwarding), or, on a memory-sourced grant, from the optional shared
  L2 / memory over the NMI bus (the L2 sits *below* the coherence point on the directory's
  memory leg, so coherence is unchanged whether or not it is enabled). It is *grant-and-go* — a memory-sourced
  grant commits the new stable state and returns ready in the same step (no transient);
  only cache-to-cache forwards and Upgrades go transient and wait for the requester's
  UNBLOCK.
- **SMP memsys** (`niigo_ccd_memsys.sv`): packages N cores (each a real `l1_icache` + a
  registered launch adapter that muxes cacheable dmem + PTW onto the agent's one `c_req`
  and bypasses device MMIO + the L1D agent) onto one directory, with the directory's single
  NMI master as the data-side backend (`nmi_mem_adapter` → `main_memory`, or the AXI4-512
  bridge under `AXI=1`; the optional shared L2 interposes on this NMI master when `L2=1`,
  and since every L1I refill here is agent-served this leg carries all memory traffic).
- **Cross-core I/D coherence, no `fence.i`** — the *remote-dirty I-fetch*: an L1I refill
  first probes the local L1D agent, and on a miss injects a `COP_LOAD` so the directory's
  GetS pulls the line cold-from-memory **or dirty-from-the-owner** before serving the
  I-fetch. This is what lets one hart `exec` a program and another hart fetch its freshly
  written text coherently (needed for xv6 SMP). A committed store / remote write also
  snoop-invalidates any stale L1I copy.
- **Multi-hart atomics** are agent-authoritative: at commit an `SC` issues a `COP_SC` so
  the directory does the reservation check-and-store atomically at the serialization point,
  and `AMO`s acquire M / replay on a snoop — so exactly one contender wins; a `snoop_kill`
  port coherence-invalidates LR/SC reservations on remote writes.
- **Fabric-agnostic**: the same directory/agents run over a behavioural combinational
  interconnect (`niigo_ccd_gg_direct.sv`, parameter `NACTIVE`) for protocol validation, and
  over a real **wheel NoC** (`cmi_router.sv`/`cmi_wheel.sv`/`niigo_ccd_gg_wheel.sv`) for
  transport validation. The wheel is four radix-4 core routers wired in a ring — each router
  has two ring links, a local port to its core, and a spoke to a central radix-5 hub router
  whose fifth port is the internal leg to the directory/memory controller. Links are 128-bit
  flits (a 512-bit line = four body flits, multi-flit SerDes) with credit-based flow control
  over five logical virtual channels (request, forward/invalidate, response-data, ack, and
  writeback on its own VC) plus a ring dateline sub-VC for deadlock freedom.
- **Shared SoC devices**: under `NIIGO_EXT_DEVICES` the CLINT/PLIC/UART become one shared
  top-level instance fed by every core (per-hart `mhartid` via the `HART_ID` parameter); a
  cross-hart IPI is an uncached `msip` store to the shared CLINT.

The MOESI protocol is machine-checked in CMurphi (`formal/moesi_ccd*.m`); two real cores
boot xv6-SMP to the `$` shell (see *Bring-up status*).

## Privileged Architecture and Virtual Memory

The OoO core implements the M/S/U privilege model with trap delegation, interrupts, and
paging (Sv32 at RV32, Sv39 at RV64). The same privileged RTL is shared with the scalar
core.

### Components

- `priv_csr_file.sv`: the machine + supervisor + user CSR file. It holds the
  architectural privilege-mode register and M/S CSR state (`mstatus`, `mtvec`/`stvec`,
  `mepc`/`sepc`, `mcause`/`scause`, `mtval`/`stval`, `mie`/`mip`, `medeleg`/`mideleg`,
  `satp`, `mscratch`/`sscratch`, the counters/`*counteren`, `menvcfg`/`senvcfg`, the PMP
  configuration/address CSRs, and WARL stubs for the performance-counter CSRs). At RV64,
  `MXLEN = 64`: the `mcause` interrupt bit moves to bit 63, `misa.MXL = 2`, `mstatus`
  carries `SXL`/`UXL`, `menvcfg` is a single 64-bit CSR (with `ADUE` at its native bit
  61), and `satp` uses the Sv39 layout (`MODE[63:60]`, `ASID`, 44-bit `PPN`). Reads are
  combinational with privilege/legality checks; writes and trap/return transitions are
  synchronous, with trap/return taking priority over a same-cycle CSR write.
- `trap_controller.sv`: combinational trap decision. Given the current mode and CSR state
  it selects whether a trap is taken, the cause, the delegated target privilege (via
  `medeleg`/`mideleg`), and the trap-vector PC. Interrupts are prioritized over
  synchronous exceptions.
- `ptw.sv`: the hardware page-table walker, shared between Sv32 and Sv39 and gated by
  `MXLEN`. It performs the two- or three-level walk (4- or 8-byte PTEs), checks leaf-PTE
  permissions against the access type and effective privilege (honouring `mstatus.SUM`
  and `mstatus.MXR`), and, when `menvcfg.ADUE` (Svadu) is set, performs hardware A/D-bit
  updates by writing the PTE back; when it is clear, a leaf that needs an A/D update
  raises the corresponding page fault (Svade). The walker's result is attributed to the
  exact access that launched it — matched on `(VPN, privilege, satp)` — so a speculative
  out-of-order fetch or load cannot consume a walk that another access started.
- `mmu_tlb.sv`: a 16-entry fully-associative TLB, instantiated separately as the ITLB and
  the DTLB (16 entries each), with round-robin replacement. Each entry caches a VPN→PPN
  mapping tagged by ASID and permission/level bits. Lookups are combinational; fills and
  flushes are synchronous. Superpages (Sv32: 4 MiB; Sv39: 1 GiB / 2 MiB) and the global bit
  are supported; `sfence.vma` is modeled as a full flush (no hardware TLB-shootdown — the
  CCD subsystem treats cross-hart `sfence.vma` as a software shootdown).
- `pmp_checker.sv`: a 16-entry Physical Memory Protection checker (TOR/NA4/NAPOT,
  R/W/X/L, lowest-match-wins). It is instantiated on both the scalar core and the OoO
  core. On the OoO core it guards three paths: implicit PTE accesses during a walk
  (`PtwPMP`), the resolved data physical address (`DataPMP`), and the fetch path
  (`FetchPMP`) — and because a 16-byte fetch block can span PMP region boundaries, each
  of the up-to-four fetched words is checked independently. A translation page fault takes
  priority over a PMP access fault.

### System-on-chip devices

The OoO core hosts a small memory-mapped device bus in the low device hole (main RAM is
at `0x8000_0000`). The devices snoop the data store port, and their loads are muxed into
the load/store writeback path:

- `clint.sv`: a SiFive-style core-local interruptor at `0x0200_0000` (`msip`,
  `mtimecmp`, `mtime`), driving the machine timer (`mtip`) and software (`msip`)
  interrupts. Parametrized by `NUM_HARTS` (per-hart `mtimecmp`/`msip`) and `NPORT`
  (independent packed load ports, one per hart); `NUM_HARTS=1`/`NPORT=1` is the
  single-core layout.
- `plic.sv`: a platform-level interrupt controller at `0x0C00_0000` with `NCTX` contexts
  (default 2: one M-external + one S-external per hart), driving `mip.MEIP`/`mip.SEIP`,
  plus `NPORT` packed per-hart load ports. The UART interrupt is wired to PLIC source 10
  (the conventional NS16550 line).

For SMP builds (`-DNIIGO_EXT_DEVICES`) the CLINT/PLIC/UART are a single shared top-level
instance fed by every core, with each core carrying its own `mhartid` via the `HART_ID`
parameter; cross-hart IPIs are uncached `msip` stores to the shared CLINT.
- `uart.sv`: an NS16550A UART at `0x0D00_0000` with the `reg-shift = 2` layout (one
  register per 32-bit word). It implements the divisor latch (DLL/DLM), FCR/IIR FIFO and
  interrupt-cause decode, the scratch register, MCR loopback, a real RX path (driven from
  the simulator via `+uart_in=<string>`), and the THRE/RX interrupt — enough for an
  8250-class kernel driver to probe and use it as a console.

### Expected behaviors

- Three privilege modes with the standard transitions: traps raise privilege (subject to
  delegation), `mret`/`sret` lower it and restore `*PIE`/`*PP`, and `mstatus.MPRV`
  redirects data-access translation/privilege to `MPP` when set.
- Address translation selected by `satp.MODE`: bare (`Svbare`) passes physical = virtual;
  `Sv32`/`Sv39` walk the page table with TLB caching, superpage support, and (under
  Svadu) A/D updates. Permission/page faults produce the architectural cause
  (instruction/load/store page fault) with `mtval` set to the faulting virtual address.
- Synchronous exceptions are precise (`PRECISE_SYNCHRONOUS_EXCEPTIONS`):
  instruction-address-misaligned, illegal instruction (including the reserved
  `0x00000000` encoding), breakpoint (`ebreak`), environment calls from U/S/M, load/store
  access and page faults, and the misaligned-atomic access fault described above.
  `mtval`/`stval` carries the faulting VA where the configuration requires it.
- Interrupts (machine/supervisor timer, software, and external) are delivered at commit
  when enabled and not delegated away, squashing the interrupted instruction so it is
  re-fetched after the handler returns. The external interrupt has a live source: the
  UART, routed through the PLIC.
- Trap-virtualization guards are enforced: `mstatus.TVM` traps `satp` access and
  `sfence.vma`, and `mstatus.TSR` traps `sret`; both fault as illegal instructions when
  violated from S-mode, and the relevant operations are illegal from U-mode.

## Repository Layout

- `src/`: processor RTL (cores, privileged/MMU/PMP, SoC devices), testbench support RTL,
  local CVFPU dependency copy
- `src/mem/`: the memory subsystem — L1I/L1D caches, the NMI line bus + AXI4-512 bridge,
  and the CMI/MOESI coherence fabric (grant-and-go directory, L1D agents, `niigo_ccd_memsys`,
  wheel NoC)
- `src/cvfpu/`: vendored CVFPU/FPnew RTL and required vendor divider/sqrt sources
- `src/common_cells/`: small compatibility modules/macros used by CVFPU
- `formal/`: CMurphi protocol models (the MOESI coherence model-check, `moesi_ccd*.m`)
- `scripts/`: ACT build/run helpers, memory-image and DTB tooling, OS-image loaders
- `act-config/`: version-controlled niigo ACT DUT config (RV32 and RV64), synced into the
  test checkout
- `references/`: upstream/reference material and generated RISC-V test artifacts
**To be changed:** dependencies located in `references/` will be moved outside later
- `tests/`: local project tests (incl. the `tb_ccd_*.sv` coherence/SMP harnesses)
- `output/`: generated build, simulation, and test output

## Build and Test

Build the OoO Verilator simulator. The default is RV32G; add `RV64=1` for the RV64G
datapath:

```sh
make verilator-build PERF=1           # canonical OoO perf build = RV64GC + L1D + landed perf levers (the reference microarchitecture config)
make verilator-build OOO=1            # RV32G OoO (base build / regression anchor)
make verilator-build RV64=1 OOO=1     # RV64G + Sv39 OoO
make verilator-build SUPERSCALAR=4    # conservative 4-wide in-order
make verilator-build                  # scalar in-order (default-flag core)
```

`OOO=1` → `-DOOO_4WIDE`, `SUPERSCALAR=4` → `-DSUPERSCALAR_4WIDE`, `RV64=1` → `-DRV64`,
`AGENT_DEBUG=1` → `-DAGENT_DEBUG`. The memory-subsystem flags (OoO only): `L1=1` →
`-DL1_CACHES` (L1I), `L1D=1` → `-DL1_CACHES -DL1D_CACHE` (write-back L1D + PTW-through-L1D),
`AXI=1` → `-DAXI_MEMSYS` (NMI→AXI4-512 bridge; requires the caches), `CCD=1` →
`-DCCD_AGENT -DL1_CACHES` (single-core grant-and-go MOESI L1D agent; mutually exclusive
with `L1D=1`). The executable lands at `output/simulation/verilator_obj/Vtop`. `PERF=1` is
the umbrella reference build — it expands to `-DRV64 -DOOO_4WIDE -DRVC -DL1_CACHES
-DL1D_CACHE -DREALIGN4 -DDEEP_WINDOW -DBIG_IQ -DBIG_ROB -DBIG_LSQ -DBTB -DXLATE_BYPASS
-DFP_OOO -DALU4` (the landed perf levers of `plans/ooo-perf.md`), the configuration the
microarchitecture sections above describe; each lever also composes individually for A/B
isolation. It is a functional sim build — `XLATE_BYPASS` carries an Fmax cost, so an
FPGA/ASIC build drops it.

Run one ACT ELF:

```sh
NIIGO_TEST_TIMEOUT=60 scripts/run_riscv_test.sh \
  references/riscv-tests/work/niigo-rv32g/elfs/rv32i/D/D-fadd.d-00.elf \
  output/riscv-tests-one
```

Run the RV32G ACT set (expected `247/247`):

```sh
python3 scripts/run_riscv_suite.py \
  --extensions I,M,F,D,Zaamo,Zalrsc,Zicsr,Zifencei \
  --jobs 8 --timeout 120 --output output/riscv-tests-rv32g
```

Run the privileged tests. These are self-checking ELFs under `elfs/priv/` that start
above the reset vector, so they need the bootstrap trampoline (`NIIGO_BOOTSTRAP=1`) and a
longer timeout:

```sh
for elf in references/riscv-tests/work/niigo-rv32g/elfs/priv/*/*.elf; do
  NIIGO_BOOTSTRAP=1 NIIGO_TEST_TIMEOUT=240 \
    scripts/run_riscv_test.sh "$elf" "output/riscv-tests-priv/$(basename "$elf" .elf)"
done
```

The RV32 tree passes the privileged suite (`ExceptionsS`, `ExceptionsU`, `S`, `U`, `Sv`,
and the `sv32_*` translation tests) alongside `247/247` of RV32G. The RV64 tree
(`niigo-rv64g` config, built with `RV64=1`) passes `289/289` of the unprivileged suite;
its Sv39 privileged group reports the architectural signatures correctly — a subset of
the fault-path tests finish in the framework's `SELFCHECK` state because of the
reference's trap budget rather than a wrong result. See `OVERNIGHT_BUGLOG.md` for the
end-to-end xv6 (RV64G) bring-up and the out-of-order bugs uncovered and fixed along the
way.

### Multicore / SMP tests

The coherence and SMP harnesses are standalone `make` targets, each building its own
`Vtop` from a `tests/tb_ccd_*.sv` top:

- `make ccd-m1-test`, `ccd-gg-test`, `ccd-gg-wheel-test`, `ccd-wheel-coh-test` — MOESI
  coherence programs (S1–S6) over the direct interconnect and the wheel NoC.
- `make ccd-stage3-test` — one real OoO core + a behavioural peer; validates the LR/SC
  reservation coherence-kill.
- `make ccd-smp-test` — two real `riscv_core_ooo` over `niigo_ccd_gg_direct(NACTIVE=2)`
  running an LR/SC spinlock; pass = **counter == 6** (mutual exclusion). `ccd-smp-amo-test`
  (AMO atomicity), `ccd-smp-ipi-test` (cross-hart IPI), each with `-rv64` and 4-core
  (`ccd-smp4-test`/`-amo4-test`/`-ipi4-test`) variants.
- `make ccd-smc-test` (and `-rv64`/`ccd-smc4-test`) — cross-core self-modifying-code
  remote-dirty I-fetch; pass = RESULT == 0x222.
- `make ccd-memsys-test` (and `-rv64`) — the reusable `niigo_ccd_memsys` module.
- `make clint-plic-smp-test` — multi-hart CLINT(`NUM_HARTS=4`)/PLIC(`NCTX=8`) directed test.
- `make ccd-xv6-build` (NCORE=2) / `ccd-xv6-1-build` (NCORE=1) — build the xv6-SMP boot
  harness (`tests/tb_ccd_xv6.sv`); then run from a staged xv6 image dir, e.g.
  `cd output/xv6m2 && <Mdir>/Vtop +no_ecall_halt +uart_in=$'ls\n'` (boots to `$`).

## CVFPU Acknowledgement

This repository uses the CVFPU/FPnew floating-point implementation, vendored locally in
`src/cvfpu/`, for hardware-efficient IEEE single- and double-precision execution. The
`niigo_fp_unit.sv` wrapper adapts CVFPU to the niigo-lake OoO issue and writeback
protocol while preserving CVFPU as the floating-point datapath.
