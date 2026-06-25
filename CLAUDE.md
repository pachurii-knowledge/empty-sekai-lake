# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

`niigo-lake` (a.k.a. `empty-sekai-lake`) is a SystemVerilog RISC-V processor with simple in-order cores and a 4-wide out-of-order core, plus M/S/U privilege and paging. The datapath width is a build switch: the default is **RV32G + Sv32**, and `RV64=1` (`-DRV64`) selects **RV64G + Sv39**; XLEN threads through every module from `RISCV_ISA::XLEN`. There is a small SoC device bus (CLINT/PLIC/NS16550A UART), and the RV64G OoO core boots **xv6-riscv to its interactive `$` shell**. It is verified with Verilator against RISC-V architectural tests at both widths. `readme.md` is the authoritative design document — read it for the full microarchitectural and privileged-ISA description; `plans/rv64-linux.md` tracks the RV64/OS work, `plans/fpga-memsys.md` tracks the L1-cache/AXI4/AWS-F2 memory-subsystem work (N1/N2 handshaked boundary + fuzz; C1 L1I, C2 write-back L1D, C3 cache HPM counters, X1 NMI→AXI4-512 bridge, C4 I/D coherence for self-modifying code all done and gated; FB2 done — the full core and the full SoC `niigo_soc` (core + caches + memsys + AXI4-512 master, `src/niigo_soc.sv`, built with `-DFPGA_BUILD`) synthesize in Vivado 2025.2 via `fpga/synth/run_{core,soc}.sh`; FB2b timing closure has the SoC clearing **62.5 MHz at Quick-place** (WNS −7.699 ns = 63.7 MHz; target 125 MHz) via three targeted pipeline cuts — 2-stage ROB commit, a registered LSQ head-translate, and a 2-stage CVFPU FMA (`f4e08bd`/`0c71630`/`db07835`); FB1 done — the F2 `cl_niigo` CL wrapper + OCL post-mortem debug plane under `fpga/rtl/` (`1a4a498`), RTL+runtime sim-verified, on-card deferred), and `OVERNIGHT_BUGLOG.md` logs the xv6 bring-up (with the FB2b timing-closure iterations in `OVERNIGHT_TIMING.md`). This file covers what you need to *build, test, and not break*.

## Build

The simulator must be built before running any test. The build flags select which core the `riscv_core` wrapper instantiates (see `src/riscv_core.sv` + `src/riscv_uarch.vh`):

```sh
make verilator-build OOO=1          # 4-wide out-of-order core, RV32G (current verification focus)
make verilator-build RV64=1 OOO=1   # 4-wide out-of-order core, RV64G + Sv39 (boots xv6)
make verilator-build SUPERSCALAR=4  # conservative 4-wide in-order core
make verilator-build                # scalar in-order core (default)
make verilator-build OOO=1 L1=1     # OoO + L1I cache (fpga-memsys C1)
make verilator-build OOO=1 L1D=1    # OoO + L1I + write-back L1D (fpga-memsys C2/C3)
make verilator-build RV64=1 OOO=1 L1D=1 AXI=1  # full FPGA-equivalent path (caches + AXI4-512); boots xv6
make verilator-build OOO=1 AGENT_DEBUG=1  # add +AGENT_DEBUG debug tracing
make verilator-clean
```

`OOO=1` → `-DOOO_4WIDE`, `SUPERSCALAR=4` → `-DSUPERSCALAR_4WIDE`, `RV64=1` → `-DRV64` (composes with `OOO`/`SUPERSCALAR`; default unset = RV32G), `AGENT_DEBUG=1` → `-DAGENT_DEBUG`. The memory-subsystem flags (OoO only): `L1=1` → `-DL1_CACHES` (L1I), `L1D=1` → `-DL1_CACHES -DL1D_CACHE` (adds the write-back L1D + PTW-through-L1D), `AXI=1` → `-DAXI_MEMSYS` (routes the cache miss/writeback traffic through the AXI4-512 bridge + a sim AXI slave; requires the caches). The default OoO build is still the zero-cache passthrough (the suites run against all of {none, L1, L1D, L1D+AXI}). The Verilator executable lands at `output/simulation/verilator_obj/Vtop`. Requires Verilator 5.x and a `riscv64-unknown-elf-*` toolchain on PATH.

## Test

The RISC-V architectural tests (ACT ELFs) live under `references/riscv-tests/work/niigo-rv32g/elfs/` and, for the 64-bit build, `references/riscv-tests/work/niigo-rv64g/elfs/` (gitignored, built via `scripts/build_riscv_tests.py`). `run_riscv_suite.py` defaults to the rv32g tree; point `--elf-dir` at the rv64g tree for the RV64 build.

```sh
# Full RV32G suite — expected: 247/247 passed
python3 scripts/run_riscv_suite.py \
  --extensions I,M,F,D,Zaamo,Zalrsc,Zicsr,Zifencei \
  --jobs 8 --timeout 120 --output output/riscv-tests-rv32g

# Full RV64G suite (RV64=1 build) — expected: 289/289 passed
python3 scripts/run_riscv_suite.py \
  --elf-dir references/riscv-tests/work/niigo-rv64g/elfs \
  --extensions I,M,F,D,Zaamo,Zalrsc,Zicsr,Zifencei \
  --jobs 8 --timeout 120 --output output/riscv-tests-rv64g

# One ELF
NIIGO_TEST_TIMEOUT=60 scripts/run_riscv_test.sh \
  references/riscv-tests/work/niigo-rv32g/elfs/rv32i/D/D-fadd.d-00.elf output/one

# Privileged ELFs (Sv32 / Sv39 under priv/) start above the reset vector, so they
# need the bootstrap trampoline (NIIGO_BOOTSTRAP=1) and a longer timeout:
NIIGO_BOOTSTRAP=1 NIIGO_TEST_TIMEOUT=240 scripts/run_riscv_test.sh \
  references/riscv-tests/work/niigo-rv32g/elfs/priv/<grp>/<test>.elf output/priv
```

The rv64 Sv39 privileged group reports the architectural signatures correctly, but a subset of the fault-path tests finish in the framework's `SELFCHECK` state because of the Sail reference's trap budget rather than a wrong result (baseline 12 PASS + 15 SELFCHECK; treat that split as green). xv6 (RV64G + Sv39) is the end-to-end OS check on the `RV64=1 OOO=1` build — see `OVERNIGHT_BUGLOG.md` for the boot procedure and the out-of-order bugs it surfaced.

`run_riscv_test.sh` loads the ELF into memory images via `scripts/load_elf_mem.py`, runs `Vtop`, and reports `RVCP-SUMMARY: TEST PASSED/FAILED`. A pass is `a0 == 10` (ECALL halt) at the self-check; `a0 == 11` is a self-check failure. Env vars: `NIIGO_TEST_TIMEOUT`, `NIIGO_BOOTSTRAP`, `NIIGO_CVFPU_IMPL`.

`scripts/priv_diag.sh ELF` decodes the ACT framework's `failure_*` scratch region (failing instruction/address/expected vs. actual) on a privileged self-check failure — the first tool to reach for when a priv test fails.

`scripts/run_riscvdv.sh [TEST] [ITERATIONS]` is the riscv-dv random-instruction flow: it generates programs with the pyflow generator, compiles each (no-compressed), takes a golden trace from Sail and a DUT trace from `Vtop` (built with `AGENT_DEBUG=1`), and diffs them with riscv-dv's `instr_trace_compare.py` (`TARGET=rv32imafd|rv64imafd`, default rv32imafd; helpers `niigo_log_to_trace_csv.py`/`sail_trace_to_csv.py`, trimmed testlist `riscvdv_testlist_trimmed.yaml`). It found two OoO deadlocks fixed in `64daa82`/`80b6f86` (mul/div and FP in-flight `branch_mask` not aged by `reset_mask`); reach for it to stress branch-mask/long-latency-op interactions the ACT suites don't cover.

A handful of standalone `.S` smoke/regression tests live in `tests/` (e.g. `sv32_data.S`, `priv_irq.S`); the regdump-based flow is `make verilator-verify TEST=tests/rv32g_smoke.S` (compared against the sibling `.reg` via `compare_regdump.py`).

After any RTL change, the baseline to keep green is **RV32G 247/247 in the default build, plus RV32G 247/247 and the Sv32 privileged suite on the OoO build** (`OOO=1`; `scripts/run_priv_suites.sh` — 28/28 PASS across `ExceptionsS/U`, `ExceptionsZaamo/Zalrsc`, `ExceptionsSvZaamo/Zalrsc`, `S`, `U`, `Sv`, `Svadu`, `SvPMP`, `SvaduPMP`, `ZicntrS/U` as of `7089b36`). Known pre-existing, NOT regressions: on the **scalar** build, priv `ExceptionsS-00`/`ExceptionsU-00` hang (0-byte sim log; reproduced on pristine `f81ed4e` — they pass on the OoO build), and `make verilator-verify TEST=tests/rv32g_smoke.S` fails on x19/x22 on every build (the `la`-to-`.data` layout skew; the committed `.reg` predates this environment's toolchain). If the change touches the shared datapath/priv/MMU, also keep the RV64 build green: **rv64 ACT 289/289 unprivileged + the Sv39 privileged group** (`RV64=1`), and—for the OoO core—the xv6 boot-to-`$`.

## Hard constraints

- **Treat `DO NOT MODIFY THIS FILE!` (CMU 18-447 lab infrastructure) files as off-limits by default, and never change their behavioral *contract*:** `main_memory.sv`, `sram_simulation.sv`, `cache.sv`, `cache_new.sv`, `testbench.sv`, `register_file.sv`, and the `.vh` headers `memory_segments.vh`, `riscv_isa.vh`, `riscv_abi.vh`, `riscv_uarch.vh`, `riscv_register_names.vh`. The default way to change one is to copy it into `src/` under a new module name and instantiate that. **Established exception — precedent, do not "fix":** a few of these `src/*` copies have already been edited *in place* — `main_memory.sv` (sparse flat DRAM + image loader), `testbench.sv` (XLEN-wide ports), `riscv_isa.vh` (the `-DRV64` XLEN switch), and `riscv_uarch.vh` (RV64 widths) — each minimal, reversible, and with the pristine lab original preserved under `447ref/` (`447ref/447include/`, `447ref/447src/`). They still carry the banner; that's expected. Follow this pattern for any further XLEN-gated header edit (minimal, `-DRV64`-gated, original kept in `447ref/`); do not casually rewrite the memory/cache models.
- **`main_memory` is word-addressed and writes one machine word at a time** (4 bytes at RV32, 8 bytes at RV64 — `WORD_BYTES = XLEN_BYTES`). This is why misaligned/cross-word accesses are split into word-granular beats inside the LSQ rather than at the memory model — see the misaligned load/store handling in `src/load_store_queue.sv`. Do not work around this by changing the memory model.
- **Editing the niigo ACT DUT config:** always edit the live copy in the checkout — `references/riscv-tests/config/cores/niigo-lake/niigo-rv32g/` (and the parallel `niigo-rv64g/` for the 64-bit suite) — since that is what `build_riscv_tests.py` reads (so no sync is needed to build/test). It lives in a gitignored, nested git repo and can't be committed from here. Before committing, run `scripts/sync_act_config.sh` to copy the edits back into the version-controlled `act-config/`, and commit that. (`scripts/sync_act_config.sh --to-checkout` restores the tracked config into a fresh checkout.) Do not hand-edit `act-config/` directly.

## Architecture map

Sizing/config knobs are in `src/ooo_types.vh` (dispatch width 4, 64 phys regs, 32-entry active list, 16-entry IQ/MemQ, 4 branch checkpoints; `XLEN` re-exported from `RISCV_ISA::XLEN`) and `src/riscv_uarch.vh`. The datapath, regfile, and pipeline structs are all `XLEN`-wide, so the same RTL builds as a 32- or 64-bit core; RV64 adds the `W`-form ALU/mul/div ops, `LD/SD/LWU`, 6-bit shamts, and RV64D conversions/moves, all `ifdef RV64`-gated.

OoO core (`src/riscv_core_ooo.sv` is the top of the datapath):
- **Frontend / prediction:** `ooo_fetch_decode.sv`, `riscv_decode.sv`; `tage_sc_l_predictor.sv` (conditional), `ittage_predictor.sv` (indirect), RAS in the core, `branch_stack.sv` (speculative checkpoints), `ooo_branch_recovery.sv` (redirect/abort masks).
- **Rename/dispatch:** `ooo_dispatch_control.sv`, `rename_map_table.sv`, `free_list.sv`, `busy_table.sv`, `active_list.sv` (ROB), `phys_reg_file.sv`.
- **Issue/execute:** `int_issue_queue.sv` → `ooo_alu_pipe.sv` (×2, ALU/branch/CSR), `ooo_mul_unit.sv`, `ooo_div_unit.sv`, `niigo_fp_unit.sv` (CVFPU wrapper; the FMA is split across 2 stages for FB2b timing, and FP issue is serialized). Results arbitrate on `ooo_writeback_bus.sv`. The mul/div and FP units age their in-flight `branch_mask` by `reset_mask` on each branch resolve (else a mispredict frees a checkpoint a long-latency op still references → deadlock; `64daa82`/`80b6f86`).
- **Commit:** `ooo_commit_unit.sv` retires in order, frees physregs, commits stores, and takes precise traps/interrupts/`mret`/`sret`/`sfence.vma` with a full-pipeline flush + frontend redirect.
- **Memory boundary + caches:** the OoO core speaks three handshaked, latency-agnostic ports (ifetch per 16-byte block, word-granular dmem load/store, PTW req/ack) into `src/mem/niigo_memsys.sv`. Default OoO build = passthrough onto `main_memory` (legacy 2/8-cycle timing). With `L1=1`/`L1D=1`/`AXI=1` (`plans/fpga-memsys.md`, C1-X1), niigo_memsys instantiates a split L1I + write-back L1D (16 KiB, 4-way, 64 B PIPT lines, sync-read tag/data arrays that infer BRAM, tree-PLRU) behind a line-granular internal bus (NMI) and an optional NMI→AXI4-512 bridge. fence.i writes back the L1D then invalidates the L1I (held by the core); the PTW flows through the L1D for page-table coherence; devices (CLINT/PLIC/UART) bypass the cache. The LSQ allows one outstanding load and tracks it (`mem_inflight`/`mem_inflight_kill`), so responses need no address matching at any latency; the in-order cores keep the legacy fixed-latency Lab-4 interface (`ifdef OOO_4WIDE` forks in `riscv_core.sv`/`testbench.sv`). **I/D coherence (phase C4)** is maintained without fence.i: a committed D-store snoop-invalidates any stale L1I copy of the line (per-line snoop on the tag array's 2nd read port — `l1_tag_array` `ren2`), and an L1I refill is held while the L1D writes back any dirty copy of that line first (clean-before-refill over the reserved `NMI_PROBE` seam — the L1D `probe_*` port + the niigo_memsys arbiter gate). Tight in-place SMC of an *already-fetched* line still needs fence.i (its re-fetch can be issued speculatively before the older store commits — inherent to an OoO core); C4 covers cold-code and patch-then-call-later (e.g. xv6 `exec`).

Privileged/MMU (shared with the scalar core): `priv_csr_file.sv` (M/S/U CSRs; `MXLEN=XLEN`, Sv32/Sv39 `satp`), `trap_controller.sv` (combinational trap decision + delegation), `ptw.sv` (Sv32 *and* Sv39 walker, gated by `MXLEN`; HW A/D updates under `menvcfg.ADUE`/Svadu), `mmu_tlb.sv` (16-entry ITLB/DTLB; Sv32 4 MiB / Sv39 1 GiB+2 MiB superpages), `pmp_checker.sv` (on both cores; on the OoO core it guards PTE walks `PtwPMP`, the resolved data PA `DataPMP`, and each fetched word `FetchPMP` in `riscv_core_ooo.sv`). PTW results are attributed to the requesting access by `(VPN, priv, satp)`, so a speculative fetch/load can't consume another access's walk. Loads/stores translate at the LSQ memory port (the head translate — DTLB + DataPMP result — is registered into a second stage for FB2b timing); fetch translates through the ITLB.

SoC device bus (memory-mapped in the low device hole; RAM at `0x8000_0000`): `clint.sv` (`0x0200_0000`, mtime/mtimecmp/msip → mtip/msip), `plic.sv` (`0x0C00_0000`, M/S-external contexts, drives `mip.MEIP`/`SEIP`; UART on source 10), `uart.sv` (NS16550A at `0x0D00_0000`, reg-shift=2, RX fed by `+uart_in=<str>`). The devices snoop the data store port; their loads mux into the LSQ writeback.

Floating point: `niigo_fp_unit.sv` wraps the vendored CVFPU/FPnew under `src/cvfpu/` (RV32D config, or RV64D when `-DRV64`) with small request/result buffers; `src/common_cells/` holds the minimal vendor support cells. CVFPU sources are listed explicitly in `src/verilator.mk` (not glob-collected).

## Layout

`src/` RTL + testbench support + vendored CVFPU · `scripts/` ACT build/run + memory-image tooling + riscv-dv glue (`run_riscvdv.sh`) · `tests/` local `.S` tests · `act-config/` version-controlled niigo ACT DUT config (synced into the checkout via `scripts/sync_act_config.sh`) · `references/` reference RTL/PDFs + generated ACT artifacts (gitignored; its `riscv-tests` is a nested git repo) · `output/` build & sim artifacts (gitignored) · `447ref/`, `plans/`, `.cursor/` gitignored.
