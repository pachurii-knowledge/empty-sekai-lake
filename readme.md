# empty-sekai-lake

`empty-sekai-lake` or `niigo-lake` is a SystemVerilog RV32G processor project with both simple in-order cores and a 4-wide out-of-order core. The current verification focus is the OoO RV32G path built with Verilator and run against RISC-V architectural tests. 

This project aims to implement all ISA extensions, privilege levels, and microarchitectural features required for booting Linux, as well as scaling to a coherent multicore system with shared cache hierarchy and interconnect.

`empty-sekai-lake` is named after the lake in `Nightcord at 25:00`'s Sekai.
## ISA Coverage

The OoO core targets RV32G-style coverage as represented by the local ACT tree:

- `I`: base RV32I integer instructions
- `M`: integer multiply/divide
- `F` and `D`: single- and double-precision floating point
- `Zaamo` and `Zalrsc`: atomic memory operations and LR/SC
- `Zicsr`: CSR instructions
- `Zifencei`: instruction fence

Recent validation:

```sh
make verilator-build OOO=1
python3 scripts/run_riscv_suite.py \
  --extensions I,M,F,D,Zaamo,Zalrsc,Zicsr,Zifencei \
  --jobs 8 \
  --timeout 120 \
  --output output/riscv-tests-rv32g-local-cvfpu
```

Expected result for the current tree: `247/247 passed`.

## Microarchitectural Overview

The out-of-order implementation is centered on `src/riscv_core_ooo.sv`. It fetches and decodes up to four instructions per cycle, renames architectural integer registers onto a 64-entry physical register file, tracks in-flight work in a 32-entry active list, and issues operations to independent functional units.

The main OoO sizing parameters live in `src/ooo_types.vh`:

- Dispatch width: `4`
- Physical integer registers: `64`
- Active list entries: `32`
- Integer issue queue entries: `16`
- Memory queue entries: `16`
- Branch checkpoints: `4`
- Issue ports: ALU0, ALU1, MUL, DIV, FP
- Writeback sources: ALU0, ALU1, load/store, multiply, divide, FP

### Frontend

The frontend tracks a 16-byte aligned fetch stream and presents four 32-bit instructions to `ooo_fetch_decode`. Control-flow prediction is split across direct and indirect predictors:

- `tage_sc_l_predictor.sv` handles conditional direct branches.
- `ittage_predictor.sv` handles indirect targets.
- A return-address stack in `riscv_core_ooo.sv` predicts returns.
- `branch_stack.sv` checkpoints rename/free-list state for speculative branches.
- `ooo_branch_recovery.sv` creates redirect and abort masks on mispredicts.

Unpredicted control transfers, serializing instructions, terminal instructions, and branch recovery can suppress dispatch until the architectural state is safe to continue.

### Rename, Dispatch, and Recovery

Decoded lanes flow through `ooo_dispatch_control.sv`, which enforces the 4-wide dispatch rules, stops at serializing instructions, limits multiple memory/control instructions where required, and checks resource pressure. Rename state is maintained by:

- `rename_map_table.sv` for architectural-to-physical mappings
- `free_list.sv` for physical register allocation and release
- `busy_table.sv` for operand readiness and wakeup tracking
- `active_list.sv` as the reorder buffer for precise commit

Branch snapshots include active-list tail, free-list pointers/count, map table state, and RAS depth. A mispredict restores those snapshots and uses branch masks to squash younger work.

### Issue and Execution

Integer and FP operations are inserted into `int_issue_queue.sv` with a functional-unit class. The core has two ALU issue slots, plus dedicated multiply, divide, and floating-point paths:

- `ooo_alu_pipe.sv` handles simple integer ALU, branch, CSR, and simple FP move/classify style results.
- `ooo_mul_unit.sv` handles RV32M multiply operations.
- `ooo_div_unit.sv` handles RV32M divide/remainder operations.
- `niigo_fp_unit.sv` handles IEEE floating-point operations through CVFPU.
- `load_store_queue.sv` handles loads, stores, atomics, FP loads/stores, doubleword split beats, and LR/SC reservation state.

Results arbitrate through `ooo_writeback_bus.sv`, which can accept up to four writeback packets per cycle and forwards wakeups to dependent instructions.

### Floating Point and CVFPU

Floating point is implemented by a wrapper around the CVFPU/FPnew implementation vendored under `src/cvfpu/`. The wrapper in `src/niigo_fp_unit.sv` translates the local `fp_op_t` control encoding into CVFPU operations, formats, rounding modes, integer formats, and status flags.

The instantiated CVFPU configuration uses the RV32D feature set, so both single-precision and double-precision operations are available. The wrapper also includes small request/result buffers to decouple the core issue/writeback handshakes from CVFPU's valid-ready interface.

The project also vendors small compatibility cells under `src/common_cells/`:

- `registers.svh`
- `lzc.sv`
- `rr_arb_tree.sv`

These provide the subset of `common_cells` functionality needed by the local CVFPU integration.

### Memory and Commit

The load/store queue tracks memory operations independently from the integer issue queue and produces load writebacks through the common writeback bus. Stores are made architecturally visible only after the commit unit authorizes the active-list entry, preserving precise state.

`ooo_commit_unit.sv` retires completed active-list entries in order, frees old physical registers, commits stores, updates architectural debug state, and reports precise halts or exceptions. Floating-point retire updates the architectural FP register array and accumulates `fflags` through `rv32g_csr_file.sv`.

## Repository Layout

- `src/`: processor RTL, testbench support RTL, local CVFPU dependency copy
- `src/cvfpu/`: vendored CVFPU/FPnew RTL and required vendor divider/sqrt sources
- `src/common_cells/`: small compatibility modules/macros used by CVFPU
- `scripts/`: ACT build/run helpers and memory image tooling
- `references/`: upstream/reference material and generated RISC-V test artifacts
- `tests/`: local project tests
- `output/`: generated build, simulation, and test output

## Build and Test

Build the OoO Verilator simulator:

```sh
make verilator-build OOO=1
```

Run one ACT ELF:

```sh
NIIGO_TEST_TIMEOUT=60 scripts/run_riscv_test.sh \
  references/riscv-tests/work/niigo-rv32g/elfs/rv32i/D/D-fadd.d-00.elf \
  output/riscv-tests-one
```

Run the RV32G ACT set:

```sh
python3 scripts/run_riscv_suite.py \
  --extensions I,M,F,D,Zaamo,Zalrsc,Zicsr,Zifencei \
  --jobs 8 \
  --timeout 120 \
  --output output/riscv-tests-rv32g
```

## CVFPU Acknowledgement

This repository uses the CVFPU/FPnew floating-point implementation, vendored locally in `src/cvfpu/`, for hardware-efficient IEEE single- and double-precision execution. The `niigo_fp_unit.sv` wrapper adapts CVFPU to the niigo-lake OoO issue and writeback protocol while preserving CVFPU as the floating-point datapath.
