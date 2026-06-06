# empty-sekai-lake

`empty-sekai-lake` or `niigo-lake` is a SystemVerilog RV32G processor project with both simple in-order cores and a 4-wide out-of-order core. The current verification focus is the OoO RV32G path built with Verilator and run against RISC-V architectural tests, now extended with the M/S/U privilege levels and Sv32 virtual memory.

This project aims to implement all ISA extensions, privilege levels, and microarchitectural features required for booting Linux, as well as scaling to a coherent multicore system with shared cache hierarchy and interconnect.

The scalar core itself references upon my lab submissions for CMU's course 18-447 Introduction to Computer Architecture (Spring 2025), and part of the peripherals, memory/cache subsystems, simulation infrastructure, and testbenches are built upon lab infrastructures of the same course.

`empty-sekai-lake` is named after the lake in `Nightcord at 25:00`'s Sekai.
## ISA Coverage

The OoO core targets RV32G-style coverage as represented by the local ACT tree:

- `I`: base RV32I integer instructions
- `M`: integer multiply/divide
- `F` and `D`: single- and double-precision floating point
- `Zaamo` and `Zalrsc`: atomic memory operations and LR/SC
- `Zicsr`: CSR instructions
- `Zifencei`: instruction fence

The privileged architecture is implemented on top of this base:

- `M`, `S`, `U`: machine, supervisor, and user privilege modes (`Sm`, `S`, `U`)
- `Svbare`: bare (untranslated) addressing when `satp.MODE = 0`
- `Sv32`: two-level page-based virtual memory when `satp.MODE = 1`

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
- `load_store_queue.sv` handles loads, stores, atomics, FP loads/stores, doubleword and misaligned split beats, and LR/SC reservation state.

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

The load/store queue tracks memory operations independently from the integer issue queue and produces load writebacks through the common writeback bus. Stores are made architecturally visible only after the commit unit authorizes the active-list entry, preserving precise state. The queue works in virtual addresses; the head access is translated by the MMU (see below) and the resolved physical word address is applied at the memory port.

Misaligned loads and stores are supported without trapping (`MISALIGNED_LDST`). The queue classifies each access from its size and byte offset and, when an access crosses a 32-bit word boundary, splits it into two word-granular beats (the simulated memory is word addressed and writes a single word at a time):

- Cross-word loads read the low word, then the high word, and recombine and sign/zero-extend the requested bytes.
- Cross-word stores first probe the high word's translation before writing either word, so a page fault on the second page is reported as the store's exception and no partial write occurs (store atomicity, including the page-crossing case).
- Misaligned atomics (`AMO`/`LR`/`SC`) are never split; an unaligned address raises an access fault with `mtval` set to the virtual address (`MISALIGNED_AMO = false`, `LRSC_MISALIGNED_BEHAVIOR = always raise access fault`).

`ooo_commit_unit.sv` retires completed active-list entries in order, frees old physical registers, commits stores, updates architectural debug state, and reports precise halts or exceptions. Floating-point retire updates the architectural FP register array and accumulates `fflags` through `rv32g_csr_file.sv`. A synchronous exception, interrupt, `mret`/`sret`, or `sfence.vma` taken at commit flushes the pipeline and redirects the frontend, keeping architectural state precise.

## Privileged Architecture and Virtual Memory

The OoO core implements the M/S/U privilege model with trap delegation, interrupts, and Sv32 paging. The same privileged RTL is shared with the scalar prototype core.

### Components

- `priv_csr_file.sv`: the machine + supervisor + user CSR file. It holds the architectural privilege-mode register and M/S CSR state (`mstatus`, `mtvec`/`stvec`, `mepc`/`sepc`, `mcause`/`scause`, `mtval`/`stval`, `mie`/`mip`, `medeleg`/`mideleg`, `satp`, `mscratch`/`sscratch`, the counters/`*counteren`, `menvcfg`/`senvcfg`, and WARL stubs for the performance-counter CSRs). Reads are combinational with privilege/legality checks; writes and trap/return transitions are synchronous, with trap/return taking priority over a same-cycle CSR write.
- `trap_controller.sv`: combinational trap decision. Given the current mode and CSR state it selects whether a trap is taken, the cause, the delegated target privilege (via `medeleg`/`mideleg`), and the trap-vector PC. Interrupts are prioritized over synchronous exceptions.
- `ptw.sv`: Sv32 hardware page-table walker. It performs the two-level walk, checks leaf-PTE permissions against the access type and effective privilege (honouring `mstatus.SUM` and `mstatus.MXR`), and performs hardware A/D-bit updates by writing the PTE back.
- `mmu_tlb.sv`: a 16-entry fully-associative TLB, instantiated separately as the ITLB and DTLB. Lookups are combinational; fills and flushes are synchronous. Superpages (4 MiB leaves at level 1) and the global bit are supported; `sfence.vma` is modeled as a full flush.
- `clint.sv`: a minimal SiFive-style core-local interruptor (`msip`, `mtimecmp`, `mtime`) that snoops the data store port and drives the machine timer (`mtip`) and software (`msip`) interrupts.
- `pmp_checker.sv`: a 16-entry Physical Memory Protection checker (TOR/NA4/NAPOT, R/W/X/L, lowest-match-wins). It is currently wired into the scalar prototype core only; PMP is not yet integrated into the OoO data/fetch paths.

### Expected behaviors

- Three privilege modes with the standard transitions: traps raise privilege (subject to delegation), `mret`/`sret` lower it and restore `*PIE`/`*PP`, and `mstatus.MPRV` redirects data-access translation/privilege to `MPP` when set.
- Address translation selected by `satp.MODE`: bare (`Svbare`) passes physical = virtual; `Sv32` walks the two-level table with TLB caching, superpage support, and A/D updates. Permission/page faults produce the architectural cause (instruction/load/store page fault) with `mtval` set to the faulting virtual address.
- Synchronous exceptions are precise (`PRECISE_SYNCHRONOUS_EXCEPTIONS`): instruction-address-misaligned, illegal instruction (including the reserved `0x00000000` encoding), breakpoint (`ebreak`), environment calls from U/S/M, load/store access and page faults, and the misaligned-atomic access fault described above. `mtval`/`stval` carries the faulting VA where the configuration requires it.
- Interrupts (machine/supervisor timer, software, and external) are delivered at commit when enabled and not delegated away, squashing the interrupted instruction so it is re-fetched after the handler returns.
- Trap-virtualization guards are enforced: `mstatus.TVM` traps `satp` access and `sfence.vma`, and `mstatus.TSR` traps `sret`; both fault as illegal instructions when violated from S-mode, and the relevant operations are illegal from U-mode.

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

Run the privileged / Sv32 tests. These are self-checking ELFs under `elfs/priv/` that start above the reset vector, so they need the bootstrap trampoline (`NIIGO_BOOTSTRAP=1`):

```sh
for elf in references/riscv-tests/work/niigo-rv32g/elfs/priv/*/*.elf; do
  NIIGO_BOOTSTRAP=1 NIIGO_TEST_TIMEOUT=240 \
    scripts/run_riscv_test.sh "$elf" "output/riscv-tests-priv/$(basename "$elf" .elf)"
done
```

The current tree passes the privileged suite (`ExceptionsS`, `ExceptionsU`, `S`, `U`, `Sv`, and the `sv32_*` translation tests) alongside `247/247` of RV32G, with no regression.

## CVFPU Acknowledgement

This repository uses the CVFPU/FPnew floating-point implementation, vendored locally in `src/cvfpu/`, for hardware-efficient IEEE single- and double-precision execution. The `niigo_fp_unit.sv` wrapper adapts CVFPU to the niigo-lake OoO issue and writeback protocol while preserving CVFPU as the floating-point datapath.
