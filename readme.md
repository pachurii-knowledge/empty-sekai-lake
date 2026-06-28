# empty-sekai-lake

`empty-sekai-lake` or `niigo-lake` is a SystemVerilog RISC-V processor project with
both simple in-order cores and a 4-wide out-of-order core. The datapath width is a
build-time switch: the default build is **RV32G**, and `-DRV64` (`RV64=1`) selects a
**RV64G** datapath with Sv39 virtual memory. Both widths carry the M/S/U privilege
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

- **RV32G** (default build): the OoO core passes `247/247` of the RV32G architectural
  suite plus the Sv32 privileged/translation tests, with no regression. This remains the
  live regression anchor through the RV64 migration.
- **RV64G** (`RV64=1`): the scalar and OoO cores pass the rv64 architectural suite
  (`289/289` unprivileged: `ui/um/ua/uf/ud`) plus the Sv39 privileged tests
  (signature-verified; see *Build and Test*).
- **Operating system**: the `RV64=1 OOO=1` core boots **xv6-riscv** (RV64G + Sv39, no
  `C`) from M-mode reset, through `mret` into the S-mode kernel, Sv39 paging with
  hardware A/D updates (Svadu), the scheduler, `init`, and the shell, to the interactive
  `$` prompt — running e.g. `ls` against an in-RAM disk image. Reaching the shell drives
  enough paging and interrupt pressure to expose out-of-order edge cases the architectural
  tests do not: several were found and fixed in store-commit handshaking, load-return
  matching under paging, and page-walk result attribution.
- **Multicore / SMP**: two real `riscv_core_ooo` cores over the `niigo_ccd_memsys`
  coherent memory subsystem boot **xv6-riscv (RV64G + Sv39) SMP** to the interactive `$`
  shell and run `ls` (commit `b5e9bc6`); single-core CCD likewise boots (`0b00906`). The
  grant-and-go MOESI directory (`src/mem/niigo_dir_gg.sv`) + non-blocking L1D agents
  (`src/mem/niigo_l1d_gg.sv`) pass 2- and 4-core LR/SC-spinlock, AMO-atomicity, and
  cross-hart IPI litmus tests (`make ccd-smp-test` → mutual-exclusion counter == 6;
  `ccd-smp-amo-test`, `ccd-smp-ipi-test`, and their `-rv64`/4-core variants), plus a
  cross-core self-modifying-code (remote-dirty I-fetch) litmus (`ccd-smc-test`). The MOESI
  protocol is independently model-checked in CMurphi (`formal/moesi_ccd*.m`). The
  coherent CCD is validated in RTL/Verilator simulation (FPGA emulation of a >2-core
  cluster is descoped pending a larger target part).
- **FPGA emulation**: The full core and the full SoC (`niigo_soc` = core + L1 caches + memory subsystem + AXI4-512 master) synthesize in Vivado for the AMD Virtex UltraScale+ HBM VU47P (AWS F2 instances), and are wrapped into the F2 Custom Logic shell (`fpga/rtl/cl_niigo.sv`) with an OCL control plane, a virtual UART console, and a post-mortem debug window for on-card bring-up. Timing closure (tracked as FB2b) currently clears **62.5 MHz at Quick-place** (worst negative slack ≈ −7.7 ns ⇒ 63.7 MHz) after three targeted pipeline cuts — a 2-stage ROB commit, a registered LSQ head-translate, and a 2-stage CVFPU FMA — versus an earlier **~60MHz post-route** measurement; the routed/on-card confirmation is the next build. Target frequency is 125MHz. (FPGA
synth/timing covers the single-core `niigo_soc`; the multicore coherent CCD is validated
in RTL/Verilator simulation — see *Multicore / SMP* above.)

## Future Roadmap

- **Linux boot support on FPGA**: The immediate goal after successful synth and P&R on the VU47P is booting a minimal Linux kernel.
  Required ISA extensions and peripherals are already implemented.
- **IPC Optimization**: Microarchitecturally, the identified bottlenecks are as follows:
  - TAGE-SC-L correctness: The current branch predictor (`src/tage_sc_l_predictor.sv`) is only an approximation of the structure proposed in Seznec14. The goal is to implement the 32Kb variant after complete validation on VU47P at target frequency.
  - VIPT L1D/I caches: Currently both caches are PIPT, requiring a TLB lookup and then PMP permission check before cache access. The caching and VM paging geometries allow alias-free L1$ indexing using virtual addresses instead of physical addresses, thus parallelizing the TLB lookup with L1$ access and saving one cycle off L1 hits. This will become the next critical path after the current LSQ path is broken up.
  - Cache parameter sweep: Self-explanatory.
- **Multicore scale-out**: the 2-/4-core directory-coherent cluster is **built and boots xv6-SMP** in RTL/Verilator simulation today (see *Bring-up status* and *Multicore cache coherence (CCD)* below); the NMI line bus is now the committed transport under a full grant-and-go MOESI directory + CMI coherence layer. The remaining work is (a) the deferred directory-robustness hardening (P5: WB-vs-Fwd evict-snoop, snoop-slot/ServeDeferred liveness, snoop-drain stale-Inv, directory-capacity broadcast-Inv), torture/RVWMO-litmus stress, and >2-core boot; and (b) FPGA emulation of the cluster — a >2-core CCD does not fit a single VU47P and AWS F2 has no PCIe P2P, so a larger target part may be needed.
  
## ISA Coverage

XLEN is selected at build time (`-DRV64`) and threaded through every module from
`RISCV_ISA::XLEN`; the privileged and MMU RTL is shared between the two widths and
selects its geometry off `MXLEN == XLEN`.

### RV32G (default)

- `I`: base RV32I integer instructions
- `M`: integer multiply/divide
- `F` and `D`: single- and double-precision floating point
- `Zaamo` and `Zalrsc`: atomic memory operations and LR/SC
- `Zicsr`: CSR instructions
- `Zifencei`: instruction fence

### RV64G (`RV64=1`)

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

### Privileged architecture (both widths)

- `M`, `S`, `U`: machine, supervisor, and user privilege modes
- `Svbare`: bare (untranslated) addressing when `satp.MODE = 0`
- `Sv32` (RV32) / `Sv39` (RV64): page-based virtual memory — two-level / 4-byte PTEs at
  RV32, three-level / 8-byte PTEs at RV64
- `Svadu`: optional hardware A/D-bit updating, gated by `menvcfg.ADUE`

### Validation snapshot

```sh
# RV32G (default OoO build): expected 247/247
make verilator-build OOO=1
python3 scripts/run_riscv_suite.py \
  --extensions I,M,F,D,Zaamo,Zalrsc,Zicsr,Zifencei \
  --jobs 8 --timeout 120 --output output/riscv-tests-rv32g

# RV64G (RV64=1 OoO build): expected 289/289 unprivileged
make verilator-build RV64=1 OOO=1
```

## Microarchitectural Overview

The out-of-order implementation is centered on `src/riscv_core_ooo.sv`. It fetches and
decodes up to four instructions per cycle, renames architectural integer registers onto
a 64-entry physical register file, tracks in-flight work in a 32-entry active list, and
issues operations to independent functional units. The datapath, register file, and
pipeline structures are all `XLEN`-wide, so the same RTL builds as either a 32- or
64-bit core.

The main OoO sizing parameters live in `src/ooo_types.vh`:

- Dispatch width: `4`
- Physical integer registers: `64`
- Active list entries: `32`
- Integer issue queue entries: `16`
- Memory queue entries: `16`
- Branch checkpoints: `4`
- Issue ports: ALU0, ALU1, MUL, DIV, FP
- Writeback sources: ALU0, ALU1, load/store, multiply, divide, FP

The build also provides a small in-order scalar core (`src/riscv_core_scalar.sv`, the
default-flag core and the first target for each RV64 datapath feature) and a
conservative 4-wide in-order core (`src/riscv_core_4wide.sv`, `SUPERSCALAR=4`). The
`src/riscv_core.sv` wrapper instantiates one of the three based on the build flags.

### Frontend

The frontend tracks a 16-byte aligned fetch stream and presents four 32-bit instructions
to `ooo_fetch_decode`. Control-flow prediction is split across direct and indirect
predictors:

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
class. The core has two ALU issue slots, plus dedicated multiply, divide, and
floating-point paths:

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

Results arbitrate through `ooo_writeback_bus.sv`, which can accept up to four writeback
packets per cycle and forwards wakeups to dependent instructions.

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
negligible. FP issue is serialized to a single in-flight operation so that an in-flight FP
op's speculative branch mask is aged correctly on branch recovery.

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
and consumed on the following cycle to shorten the FPGA critical path.

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

### Multicore cache coherence (CCD)

Multiple OoO cores share memory through a directory-based MOESI coherence fabric
(`src/mem/`, design tracked in `plans/multicore-ccd.md`). It is built under `CCD_AGENT`
and gated so that single-core / `COHERENT=0` builds are bit-identical to the
pre-coherence core (the CCD modules are otherwise parsed but not elaborated).

- **L1D agent** (`niigo_l1d_gg.sv`): each core's L1D is a non-blocking MOESI agent with
  one outstanding demand transaction (an MSHR), acquire/evict transients, deferred snoops
  (the acquire-side deferred-snoop matrix), and an ack-to-requester down-counter for write
  atomicity.
- **Directory** (`niigo_dir_gg.sv`): the per-line coherence/serialization point. It is
  *grant-and-go* — an L2/memory-sourced grant commits the new stable state and returns
  ready in the same step (no transient); only cache-to-cache forwards and Upgrades go
  transient and wait for the requester's UNBLOCK.
- **SMP memsys** (`niigo_ccd_memsys.sv`): packages N cores (each a real `l1_icache` + a
  registered launch adapter that muxes cacheable dmem + PTW onto the agent's one `c_req`
  and bypasses device MMIO + the L1D agent) onto one directory, with the directory's single
  NMI master as the data-side backend (`nmi_mem_adapter` → `main_memory`, or the AXI4-512
  bridge under `AXI=1`).
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
  over a real **wheel NoC** (4 core routers in a ring + a radix-5 hub, 128b multi-flit
  SerDes: `cmi_router.sv`/`cmi_wheel.sv`/`niigo_ccd_gg_wheel.sv`) for transport validation.
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
  DTLB. Lookups are combinational; fills and flushes are synchronous. Superpages (Sv32:
  4 MiB; Sv39: 1 GiB / 2 MiB) and the global bit are supported; `sfence.vma` is modeled
  as a full flush.
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
- `plans/`: design/bring-up plans (e.g. `rv64-linux.md`, `multicore-ccd.md`)
- `references/`: upstream/reference material and generated RISC-V test artifacts
- `tests/`: local project tests (incl. the `tb_ccd_*.sv` coherence/SMP harnesses)
- `output/`: generated build, simulation, and test output

## Build and Test

Build the OoO Verilator simulator. The default is RV32G; add `RV64=1` for the RV64G
datapath:

```sh
make verilator-build OOO=1            # RV32G OoO (default focus / regression anchor)
make verilator-build RV64=1 OOO=1     # RV64G + Sv39 OoO
make verilator-build SUPERSCALAR=4    # conservative 4-wide in-order
make verilator-build                  # scalar in-order (default-flag core)
```

`OOO=1` → `-DOOO_4WIDE`, `SUPERSCALAR=4` → `-DSUPERSCALAR_4WIDE`, `RV64=1` → `-DRV64`,
`AGENT_DEBUG=1` → `-DAGENT_DEBUG`. The memory-subsystem flags (OoO only): `L1=1` →
`-DL1_CACHES` (L1I), `L1D=1` → `-DL1_CACHES -DL1D_CACHE` (write-back L1D + PTW-through-L1D),
`AXI=1` → `-DAXI_MEMSYS` (NMI→AXI4-512 bridge; requires the caches), `CCD=1` →
`-DCCD_AGENT -DL1_CACHES` (single-core grant-and-go MOESI L1D agent; mutually exclusive
with `L1D=1`). The executable lands at `output/simulation/verilator_obj/Vtop`.

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
