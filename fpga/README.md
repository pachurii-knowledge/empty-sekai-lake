# niigo on AWS F2 — FB1 CL integration + debug interface

This tree wraps the `niigo_soc` (4-wide OoO RV64G core + L1I/L1D + AXI4-512
master) into the AWS F2 Custom Logic shell so it can boot xv6 (and, later, Linux)
on a real FPGA, with a host-side console and a debug window for bring-up.

```
  host (niigo_host)                          FPGA (cl_niigo)
  ─────────────────         OCL AXI-Lite     ──────────────────────────────────
   peek/poke CSRs   ───────────────────────▶  ocl_csr ── CTRL/STATUS/counters
   vUART console    ◀──────────────────────▶          ├─ TX/RX byte FIFOs ─┐
   debug dump       ◀───────────────────────          └─ debug block ◀─ dbg_probe
   preload image    ── DMA_PCIS 512b ──┐                                    │
                                       ▼                                    │
                          reset-gated AXI mux ──▶ sh_ddr (DDR4)             │
                                       ▲                                    │
                       niigo_soc m_axi ┘   (device space CLINT/PLIC/UART    │
                                            stays inside the core)  ────────┘
```

## Components

| File | Role |
|---|---|
| `rtl/cl_niigo.sv` | F2 CL top: instantiates `niigo_soc`, the OCL plane, vUART FIFOs, the reset-gated AXI mux, and `sh_ddr`; ties off unused shell interfaces. |
| `rtl/ocl_csr.sv` | AXI4-Lite slave (AppPF BAR0): control (go/soft-reset), status, counters, vUART FIFO ports, and the debug observability block. |
| `rtl/uart_host_fifo.sv` | Synchronous FWFT byte FIFO used for the vUART TX (core→host) and RX (host→core) buffers. |
| `rtl/axi512_mux.sv` | 2:1 AXI4-512 mux: `niigo_soc` master vs. DMA_PCIS preload, selected by core-reset. |
| `runtime/niigo_host.c` | Host program: attach OCL, preload DRAM over DMA_PCIS, release reset, run the vUART console, dump the debug block. |
| `runtime/niigo_ocl.h` | The OCL register map (mirrors `ocl_csr.sv` — keep in sync). |
| `synth/` | OOC synthesis + timing-triage scripts for `niigo_soc` (FB2/FB2b). |

The core itself is reused unchanged except for **`FPGA_BUILD`-gated taps**: the
NS16550 UART's console (`uart.sv` `$write`/`+uart_in`) is rerouted to the vUART
FIFO ports, and a zero-functional-change `debug_probe` struct is brought out of
`riscv_core_ooo` → `niigo_soc`. With `FPGA_BUILD` undefined (every Verilator
build) none of this exists, so the ACT/xv6 verification is byte-for-byte
unaffected.

## Bring-up model

The host preloads card DRAM **while the core is held in reset** (`CTRL.go=0`),
then releases it. This is why a simple reset-gated AXI mux is sufficient instead
of a full crossbar: the PCIS preload path and the core's master never drive DDR
at the same time (the plan's DMA-coherence non-goal — "host loads memory only
while the core is in reset"). Device space (CLINT/PLIC/UART) never leaves the
core, so only cacheable traffic appears on AXI/DDR.

```sh
# on the F2 instance, after the AFI is loaded with fpga-load-local-image:
source $AWS_FPGA_REPO_DIR/sdk_setup.sh
make -C fpga/runtime
sudo ./fpga/runtime/niigo_host --kernel kernel.bin --fs fs.img --debug
#   --kernel : raw memory image at 0x8000_0000   --fs : disk blob at 0x9000_0000
#   console: Ctrl-]  quits,  Ctrl-\  dumps the debug block
```

(`kernel.bin` is the flat memory image — the same bytes the sim stages via
`load_elf_mem.py`; for xv6 also stage `fs.img` at `DISKBASE` as the sim does.)

## Debug interface (for Linux/xv6 bring-up)

A post-mortem observability window, all over the OCL plane — no JTAG, no sim
rebuild. Driven by a commit-stage probe (`debug_probe_t`), so it reflects exactly
what retired. Read it any time with `--debug` or Ctrl-\ in the console; on a hang
the counters stop and the state is naturally frozen.

- **Liveness** — `CYCLE` / `INSTRET` free-running counters. Not advancing ⇒ hung.
- **Committed-PC ring** — the last 16 retired PCs (most-recent first). "Where did
  it die?" — the tail of the trace at a hang or panic loop.
- **Trap log** — count + last `{cause, epc, tval, is_int}`. "Did it take an
  unexpected fault?" (e.g. a load page fault during early Linux paging).
- **Shadow architectural regfile** — the committed values of x0–x31 (a0/sp/…),
  for inspecting state at the failure point.
- **L1 HPM counters** — I-miss / D-miss / D-writeback.
- **Memory peek** — free: read card DRAM directly over the same DMA_PCIS path
  used for preload (quiesce the core first, e.g. via `CTRL.soft_reset`).

Register map: `runtime/niigo_ocl.h` (mirrors `rtl/ocl_csr.sv`).

## Building the AFI (HDK)

`cl_niigo` instantiates shell library blocks (`sh_ddr`, `lib_pipe`,
`xpm_cdc_async_rst`), so it builds inside the aws-fpga HDK `build_cl` flow, not
this repo's generic OOC synth. The CL source list is:

- `fpga/rtl/{cl_niigo,ocl_csr,uart_host_fifo,axi512_mux}.sv`
- all `src/*.sv` + `src/mem/*.sv` **except** the sim/lab-only files excluded by
  `fpga/synth/run_soc.sh` (testbench, main_memory, sram_simulation, cache*,
  register_file, riscv_core*, nmi_mem_adapter, axi_mem_shim, axi_chk) plus the
  CVFPU list and `src/common_cells/{lzc,rr_arb_tree}.sv`
- include dirs: `src`, `src/mem`, `src/common_cells`, `src/cvfpu/{src,vendor,...}`
- defines: `RV64 OOO_4WIDE L1_CACHES L1D_CACHE AXI_MEMSYS FPGA_BUILD SYNTHESIS
  LAB_18447="4b"`
- top: `cl_niigo`, DDR present (`EN_DDR=1`); target a conservative shell clock
  recipe first (≈62–125 MHz class — see `plans/branch-recovery-pipeline.md` for
  the timing-closure work).

Set the AFI id registers in `cl_id_defines.vh` (`CL_SH_ID0/1`) before the build.
Cross-check the HBM-APB / PCIe-transceiver tie-offs in `cl_niigo.sv` against the
HDK `CL_TEMPLATE` for the f2 shell version in use.

## Verification status

- **Shared-RTL taps inert in sim** — `make verilator-build RV64=1 OOO=1 L1D=1
  AXI=1` clean; rv64 ACT **289/289** unchanged (the probe/vUART are `FPGA_BUILD`
  only). ✓
- **FPGA path elaborates** — `verilator --lint-only --top-module niigo_soc
  -DFPGA_BUILD …` clean (no errors). ✓
- **OCL plane + vUART + debug block functional** — `tests/fpga/tb_ocl_csr.sv`
  passes (AXI-Lite r/w, CTRL/STATUS, vUART TX/RX FIFO paths, cycle/instret +
  clear, PC ring, shadow regfile incl. x0=0, trap log). ✓
  ```sh
  verilator --binary --sv --timing -Wno-fatal --top-module tb_ocl_csr \
    -Isrc -Isrc/mem -DRV64 \
    fpga/rtl/uart_host_fifo.sv fpga/rtl/ocl_csr.sv tests/fpga/tb_ocl_csr.sv \
    --Mdir /tmp/tb_ocl && /tmp/tb_ocl/Vtb_ocl_csr
  ```
- **`niigo_soc` OOC synth** (with the new ports) — `fpga/synth/run_soc.sh`
  (needs Vivado).
- **Pending hardware** — `cl_niigo` HDK simulation (shell BFM hello-store/load +
  boot-start smoke) and on-card xv6 boot-to-`$` over vUART are FB2-HDK/FB3, on an
  F2 instance.
