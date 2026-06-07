# niigo ACT (architectural-test) DUT config

Version-controlled source of truth for the niigo-lake RISC-V arch-test (ACT)
DUT configuration. These files describe the DUT to the arch-test framework and
the Sail reference model (extensions, CSR/feature parameters, interrupt-injection
macros, memory layout).

`references/riscv-tests/` is an external checkout that is **gitignored** (and is
its own git repo, so its files can't be tracked from this repo). The ACT build
reads its config from
`references/riscv-tests/config/cores/niigo-lake/niigo-rv32g/`, so that checkout
copy is the live one and this directory is the version-controlled mirror.

**Workflow:** edit the config **in the checkout** (no sync needed to build/test),
then before committing copy the edits back here:

```sh
scripts/sync_act_config.sh              # checkout -> act-config (run before committing)
scripts/sync_act_config.sh --to-checkout  # act-config -> checkout (restore a fresh checkout)
```

Do not hand-edit `act-config/` directly — edits there are overwritten by the next
`sync_act_config.sh`. Use `--to-checkout` to seed/restore the config after a fresh
`references/` checkout.

## Files (`niigo-lake/niigo-rv32g/`)

- `niigo-rv32g.yaml` — DUT description: `implemented_extensions` (gates which ACT
  suites are generated) and CSR/feature parameters (`NUM_PMP_ENTRIES`,
  `MCOUNTINHIBIT_IMPLEMENTED`, …). Editing extensions here changes which suites
  build.
- `rvmodel_macros.h` — `RVMODEL_*` hooks: HALT_PASS/FAIL, and the interrupt
  injection macros (`SET/CLR_MEXT/SEXT/MSW/SSW`) that poke the PLIC
  (`0x0C00_0000`) and CLINT (`0x0200_0000`). NOTE: `CLR_SSW_INT` is intentionally
  empty — the arch-test framework's `clr_Ssw` handler already clears SSIP
  mode-correctly (`csrc mip` in M-mode, `csrc sip` in S-mode); emitting `csrc mip`
  here executes it from the S-mode handler, which is illegal (mip is M-only).
- `sail.json` — Sail reference-model config (CLINT, `simple_interrupt_generator`,
  extensions, `max_time_to_wait`, …).
- `test_config.yaml`, `riscv_arch_test.h`, `link.ld`, `run_cmd.txt` — framework
  glue (test selection, linker script, run command).

See `plans/priv-isa-linux-completion.md` and the `act-suite-validation` memory
for the build/run flow and the Interrupts S/U out-of-scope finding.
