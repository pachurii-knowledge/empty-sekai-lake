# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

`niigo-lake` (a.k.a. `empty-sekai-lake`) is a SystemVerilog RV32G processor with simple in-order cores and a 4-wide out-of-order core, plus M/S/U privilege and Sv32 virtual memory. It is verified with Verilator against RISC-V architectural tests. `readme.md` is the authoritative design document — read it for the full microarchitectural and privileged-ISA description. This file covers what you need to *build, test, and not break*.

## Build

The simulator must be built before running any test. The build flags select which core the `riscv_core` wrapper instantiates (see `src/riscv_core.sv` + `src/riscv_uarch.vh`):

```sh
make verilator-build OOO=1          # 4-wide out-of-order core (current verification focus)
make verilator-build SUPERSCALAR=4  # conservative 4-wide in-order core
make verilator-build                # scalar in-order core (default)
make verilator-build OOO=1 AGENT_DEBUG=1  # add +AGENT_DEBUG debug tracing
make verilator-clean
```

`OOO=1` → `-DOOO_4WIDE`, `SUPERSCALAR=4` → `-DSUPERSCALAR_4WIDE`, `AGENT_DEBUG=1` → `-DAGENT_DEBUG`. The Verilator executable lands at `output/simulation/verilator_obj/Vtop`. Requires Verilator 5.x and a `riscv64-unknown-elf-*` toolchain on PATH.

## Test

The RISC-V architectural tests (ACT ELFs) live under `references/riscv-tests/work/niigo-rv32g/elfs/` (gitignored, built via `scripts/build_riscv_tests.py`).

```sh
# Full RV32G suite — expected: 247/247 passed
python3 scripts/run_riscv_suite.py \
  --extensions I,M,F,D,Zaamo,Zalrsc,Zicsr,Zifencei \
  --jobs 8 --timeout 120 --output output/riscv-tests-rv32g

# One ELF
NIIGO_TEST_TIMEOUT=60 scripts/run_riscv_test.sh \
  references/riscv-tests/work/niigo-rv32g/elfs/rv32i/D/D-fadd.d-00.elf output/one

# Privileged / Sv32 ELFs start above the reset vector, so they need the
# bootstrap trampoline (NIIGO_BOOTSTRAP=1) and a longer timeout:
NIIGO_BOOTSTRAP=1 NIIGO_TEST_TIMEOUT=240 scripts/run_riscv_test.sh \
  references/riscv-tests/work/niigo-rv32g/elfs/priv/<grp>/<test>.elf output/priv
```

`run_riscv_test.sh` loads the ELF into memory images via `scripts/load_elf_mem.py`, runs `Vtop`, and reports `RVCP-SUMMARY: TEST PASSED/FAILED`. A pass is `a0 == 10` (ECALL halt) at the self-check; `a0 == 11` is a self-check failure. Env vars: `NIIGO_TEST_TIMEOUT`, `NIIGO_BOOTSTRAP`, `NIIGO_CVFPU_IMPL`.

`scripts/priv_diag.sh ELF` decodes the ACT framework's `failure_*` scratch region (failing instruction/address/expected vs. actual) on a privileged self-check failure — the first tool to reach for when a priv test fails.

A handful of standalone `.S` smoke/regression tests live in `tests/` (e.g. `sv32_data.S`, `priv_irq.S`); the regdump-based flow is `make verilator-verify TEST=tests/rv32g_smoke.S` (compared against the sibling `.reg` via `compare_regdump.py`).

After any RTL change, the baseline to keep green is **RV32G 247/247 plus the privileged suite** (`ExceptionsS/U`, `S`, `U`, `Sv`, `sv32_*`).

## Hard constraints

- **Never edit `DO NOT MODIFY THIS FILE!` files.** These are CMU 18-447 lab infrastructure: `main_memory.sv`, `sram_simulation.sv`, `cache.sv`, `cache_new.sv`, `testbench.sv`, `register_file.sv`, and the `.vh` headers `memory_segments.vh`, `riscv_isa.vh`, `riscv_abi.vh`, `riscv_uarch.vh`, `riscv_register_names.vh`. To change one, copy it into `src/` under a new module name and instantiate that instead.
- **`main_memory` is word-addressed and writes one 32-bit word at a time.** This is why misaligned/cross-word accesses are split into word-granular beats inside the LSQ rather than at the memory model — see the misaligned load/store handling in `src/load_store_queue.sv`. Do not work around this by changing the memory model.
- **Editing the niigo ACT DUT config:** always edit the live copy in the checkout — `references/riscv-tests/config/cores/niigo-lake/niigo-rv32g/` — since that is what `build_riscv_tests.py` reads (so no sync is needed to build/test). It lives in a gitignored, nested git repo and can't be committed from here. Before committing, run `scripts/sync_act_config.sh` to copy the edits back into the version-controlled `act-config/`, and commit that. (`scripts/sync_act_config.sh --to-checkout` restores the tracked config into a fresh checkout.) Do not hand-edit `act-config/` directly.

## Architecture map

Sizing/config knobs are in `src/ooo_types.vh` (dispatch width 4, 64 phys regs, 32-entry active list, 16-entry IQ/MemQ, 4 branch checkpoints) and `src/riscv_uarch.vh`.

OoO core (`src/riscv_core_ooo.sv` is the top of the datapath):
- **Frontend / prediction:** `ooo_fetch_decode.sv`, `riscv_decode.sv`; `tage_sc_l_predictor.sv` (conditional), `ittage_predictor.sv` (indirect), RAS in the core, `branch_stack.sv` (speculative checkpoints), `ooo_branch_recovery.sv` (redirect/abort masks).
- **Rename/dispatch:** `ooo_dispatch_control.sv`, `rename_map_table.sv`, `free_list.sv`, `busy_table.sv`, `active_list.sv` (ROB), `phys_reg_file.sv`.
- **Issue/execute:** `int_issue_queue.sv` → `ooo_alu_pipe.sv` (×2, ALU/branch/CSR), `ooo_mul_unit.sv`, `ooo_div_unit.sv`, `niigo_fp_unit.sv` (CVFPU wrapper), `load_store_queue.sv`. Results arbitrate on `ooo_writeback_bus.sv`.
- **Commit:** `ooo_commit_unit.sv` retires in order, frees physregs, commits stores, and takes precise traps/interrupts/`mret`/`sret`/`sfence.vma` with a full-pipeline flush + frontend redirect.

Privileged/MMU (shared with the scalar prototype core): `priv_csr_file.sv` (M/S/U CSRs), `trap_controller.sv` (combinational trap decision + delegation), `ptw.sv` (Sv32 walker, A/D updates), `mmu_tlb.sv` (16-entry ITLB/DTLB), `clint.sv` (mtime/mtimecmp/msip), `plic.sv` (M/S-external interrupt controller, base `0x0C00_0000`, drives `mip.MEIP`/`SEIP`), `pmp_checker.sv` (instantiated on both the scalar core and the OoO fetch+data paths — `DataPMP`/`FetchPMP` in `riscv_core_ooo.sv`). Loads/stores translate at the LSQ memory port; fetch translates through the ITLB.

Floating point: `niigo_fp_unit.sv` wraps the vendored CVFPU/FPnew under `src/cvfpu/` (RV32D config) with small request/result buffers; `src/common_cells/` holds the minimal vendor support cells. CVFPU sources are listed explicitly in `src/verilator.mk` (not glob-collected).

## Layout

`src/` RTL + testbench support + vendored CVFPU · `scripts/` ACT build/run + memory-image tooling · `tests/` local `.S` tests · `act-config/` version-controlled niigo ACT DUT config (synced into the checkout via `scripts/sync_act_config.sh`) · `references/` reference RTL/PDFs + generated ACT artifacts (gitignored; its `riscv-tests` is a nested git repo) · `output/` build & sim artifacts (gitignored) · `447ref/`, `plans/`, `.cursor/` gitignored.
