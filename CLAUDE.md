# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

`niigo-lake` (a.k.a. `empty-sekai-lake`) is a SystemVerilog RISC-V processor with simple in-order cores and a 4-wide out-of-order core, plus M/S/U privilege and paging. The datapath width is a build switch: the default is **RV32G + Sv32**, and `RV64=1` (`-DRV64`) selects **RV64G + Sv39**; XLEN threads through every module from `RISCV_ISA::XLEN`. There is a small SoC device bus (CLINT/PLIC/NS16550A UART), and the RV64G OoO core boots **xv6-riscv to its interactive `$` shell**. It is verified with Verilator against RISC-V architectural tests at both widths. `readme.md` is the authoritative design document — read it for the full microarchitectural and privileged-ISA description; `plans/rv64-linux.md` tracks the RV64/OS work and `OVERNIGHT_BUGLOG.md` logs the xv6 bring-up. This file covers what you need to *build, test, and not break*.

## Build

The simulator must be built before running any test. The build flags select which core the `riscv_core` wrapper instantiates (see `src/riscv_core.sv` + `src/riscv_uarch.vh`):

```sh
make verilator-build OOO=1          # 4-wide out-of-order core, RV32G (current verification focus)
make verilator-build RV64=1 OOO=1   # 4-wide out-of-order core, RV64G + Sv39 (boots xv6)
make verilator-build SUPERSCALAR=4  # conservative 4-wide in-order core
make verilator-build                # scalar in-order core (default)
make verilator-build OOO=1 AGENT_DEBUG=1  # add +AGENT_DEBUG debug tracing
make verilator-clean
```

`OOO=1` → `-DOOO_4WIDE`, `SUPERSCALAR=4` → `-DSUPERSCALAR_4WIDE`, `RV64=1` → `-DRV64` (composes with `OOO`/`SUPERSCALAR`; default unset = RV32G), `AGENT_DEBUG=1` → `-DAGENT_DEBUG`. The Verilator executable lands at `output/simulation/verilator_obj/Vtop`. Requires Verilator 5.x and a `riscv64-unknown-elf-*` toolchain on PATH.

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

A handful of standalone `.S` smoke/regression tests live in `tests/` (e.g. `sv32_data.S`, `priv_irq.S`); the regdump-based flow is `make verilator-verify TEST=tests/rv32g_smoke.S` (compared against the sibling `.reg` via `compare_regdump.py`).

After any RTL change, the baseline to keep green is **RV32G 247/247 plus the Sv32 privileged suite** (`ExceptionsS/U`, `S`, `U`, `Sv`, `sv32_*`) in the default build. If the change touches the shared datapath/priv/MMU, also keep the RV64 build green: **rv64 ACT 289/289 unprivileged + the Sv39 privileged group** (`RV64=1`), and—for the OoO core—the xv6 boot-to-`$`.

## Hard constraints

- **Treat `DO NOT MODIFY THIS FILE!` (CMU 18-447 lab infrastructure) files as off-limits by default, and never change their behavioral *contract*:** `main_memory.sv`, `sram_simulation.sv`, `cache.sv`, `cache_new.sv`, `testbench.sv`, `register_file.sv`, and the `.vh` headers `memory_segments.vh`, `riscv_isa.vh`, `riscv_abi.vh`, `riscv_uarch.vh`, `riscv_register_names.vh`. The default way to change one is to copy it into `src/` under a new module name and instantiate that. **Established exception — precedent, do not "fix":** a few of these `src/*` copies have already been edited *in place* — `main_memory.sv` (sparse flat DRAM + image loader), `testbench.sv` (XLEN-wide ports), `riscv_isa.vh` (the `-DRV64` XLEN switch), and `riscv_uarch.vh` (RV64 widths) — each minimal, reversible, and with the pristine lab original preserved under `447ref/` (`447ref/447include/`, `447ref/447src/`). They still carry the banner; that's expected. Follow this pattern for any further XLEN-gated header edit (minimal, `-DRV64`-gated, original kept in `447ref/`); do not casually rewrite the memory/cache models.
- **`main_memory` is word-addressed and writes one machine word at a time** (4 bytes at RV32, 8 bytes at RV64 — `WORD_BYTES = XLEN_BYTES`). This is why misaligned/cross-word accesses are split into word-granular beats inside the LSQ rather than at the memory model — see the misaligned load/store handling in `src/load_store_queue.sv`. Do not work around this by changing the memory model.
- **Editing the niigo ACT DUT config:** always edit the live copy in the checkout — `references/riscv-tests/config/cores/niigo-lake/niigo-rv32g/` (and the parallel `niigo-rv64g/` for the 64-bit suite) — since that is what `build_riscv_tests.py` reads (so no sync is needed to build/test). It lives in a gitignored, nested git repo and can't be committed from here. Before committing, run `scripts/sync_act_config.sh` to copy the edits back into the version-controlled `act-config/`, and commit that. (`scripts/sync_act_config.sh --to-checkout` restores the tracked config into a fresh checkout.) Do not hand-edit `act-config/` directly.

## Architecture map

Sizing/config knobs are in `src/ooo_types.vh` (dispatch width 4, 64 phys regs, 32-entry active list, 16-entry IQ/MemQ, 4 branch checkpoints; `XLEN` re-exported from `RISCV_ISA::XLEN`) and `src/riscv_uarch.vh`. The datapath, regfile, and pipeline structs are all `XLEN`-wide, so the same RTL builds as a 32- or 64-bit core; RV64 adds the `W`-form ALU/mul/div ops, `LD/SD/LWU`, 6-bit shamts, and RV64D conversions/moves, all `ifdef RV64`-gated.

OoO core (`src/riscv_core_ooo.sv` is the top of the datapath):
- **Frontend / prediction:** `ooo_fetch_decode.sv`, `riscv_decode.sv`; `tage_sc_l_predictor.sv` (conditional), `ittage_predictor.sv` (indirect), RAS in the core, `branch_stack.sv` (speculative checkpoints), `ooo_branch_recovery.sv` (redirect/abort masks).
- **Rename/dispatch:** `ooo_dispatch_control.sv`, `rename_map_table.sv`, `free_list.sv`, `busy_table.sv`, `active_list.sv` (ROB), `phys_reg_file.sv`.
- **Issue/execute:** `int_issue_queue.sv` → `ooo_alu_pipe.sv` (×2, ALU/branch/CSR), `ooo_mul_unit.sv`, `ooo_div_unit.sv`, `niigo_fp_unit.sv` (CVFPU wrapper), `load_store_queue.sv`. Results arbitrate on `ooo_writeback_bus.sv`.
- **Commit:** `ooo_commit_unit.sv` retires in order, frees physregs, commits stores, and takes precise traps/interrupts/`mret`/`sret`/`sfence.vma` with a full-pipeline flush + frontend redirect.

Privileged/MMU (shared with the scalar core): `priv_csr_file.sv` (M/S/U CSRs; `MXLEN=XLEN`, Sv32/Sv39 `satp`), `trap_controller.sv` (combinational trap decision + delegation), `ptw.sv` (Sv32 *and* Sv39 walker, gated by `MXLEN`; HW A/D updates under `menvcfg.ADUE`/Svadu), `mmu_tlb.sv` (16-entry ITLB/DTLB; Sv32 4 MiB / Sv39 1 GiB+2 MiB superpages), `pmp_checker.sv` (on both cores; on the OoO core it guards PTE walks `PtwPMP`, the resolved data PA `DataPMP`, and each fetched word `FetchPMP` in `riscv_core_ooo.sv`). PTW results are attributed to the requesting access by `(VPN, priv, satp)`, so a speculative fetch/load can't consume another access's walk. Loads/stores translate at the LSQ memory port; fetch translates through the ITLB.

SoC device bus (memory-mapped in the low device hole; RAM at `0x8000_0000`): `clint.sv` (`0x0200_0000`, mtime/mtimecmp/msip → mtip/msip), `plic.sv` (`0x0C00_0000`, M/S-external contexts, drives `mip.MEIP`/`SEIP`; UART on source 10), `uart.sv` (NS16550A at `0x0D00_0000`, reg-shift=2, RX fed by `+uart_in=<str>`). The devices snoop the data store port; their loads mux into the LSQ writeback.

Floating point: `niigo_fp_unit.sv` wraps the vendored CVFPU/FPnew under `src/cvfpu/` (RV32D config, or RV64D when `-DRV64`) with small request/result buffers; `src/common_cells/` holds the minimal vendor support cells. CVFPU sources are listed explicitly in `src/verilator.mk` (not glob-collected).

## Layout

`src/` RTL + testbench support + vendored CVFPU · `scripts/` ACT build/run + memory-image tooling · `tests/` local `.S` tests · `act-config/` version-controlled niigo ACT DUT config (synced into the checkout via `scripts/sync_act_config.sh`) · `references/` reference RTL/PDFs + generated ACT artifacts (gitignored; its `riscv-tests` is a nested git repo) · `output/` build & sim artifacts (gitignored) · `447ref/`, `plans/`, `.cursor/` gitignored.
