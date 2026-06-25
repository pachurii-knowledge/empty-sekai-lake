#!/usr/bin/env bash
#
# run_riscvdv.sh -- drive the niigo OoO RTL against the riscv-dv random
# instruction generator, comparing its commit trace against Sail.
#
# riscv-dv's own run.py only does gen -> ISS -> ISS-compare; it never touches a
# DUT. This script bolts the RTL leg on:
#
#   1. generate random programs with the pyflow generator (no commercial sim)
#   2. compile each to an ELF (no-compressed march matching the niigo target)
#   3. golden trace from the ISS (ISS=sail|spike) -> riscv-dv CSV
#   4. DUT    trace from Vtop  (AGENT_DEBUG retire lines)   -> riscv-dv CSV
#   5. diff the two with riscv-dv's instr_trace_compare.py
#
# The golden model is selectable with ISS: "sail" (default, the config-based
# sail_riscv_sim) or "spike" (the riscv-isa-sim build under references/). Both
# legs feed the SAME instr_trace_compare.py via convention-matched converters
# (sail_trace_to_csv.py / spike_trace_to_csv.py), so the verdict logic is shared.
#
# Prereqs:
#   - Vtop built with AGENT_DEBUG for the matching XLEN, e.g.
#       make verilator-clean && make verilator-build OOO=1 AGENT_DEBUG=1            # rv32imafd
#       make verilator-clean && make verilator-build RV64=1 OOO=1 AGENT_DEBUG=1     # rv64imafd
#   - a Python env with pyvsc/bitstring (the pyflow generator); see RISCVDV_PY.
#
# Usage:
#   scripts/run_riscvdv.sh [TEST] [ITERATIONS]
# Env overrides:
#   TARGET=rv32imafd|rv64imafd   (default rv32imafd)
#   ISS=sail|spike               (golden model; default sail)
#   RISCVDV_PY=<python>          (default: a venv that imports vsc)
#   OUT=<dir>                    (default: output/riscvdv)
#   NIIGO_TEST_TIMEOUT=<secs>    (default 120)
#   SPIKE_TIMEOUT=<secs>         (per-ELF spike wall-clock cap; default 120)
#   SPIKE_INSN_LIMIT=<n>         (spike --instructions cap; default 2000000)
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
RVDV="$ROOT/references/riscv-dv"
SAIL="$ROOT/references/sail-riscv-Linux-x86_64/bin/sail_riscv_sim"
SPIKE="${SPIKE:-$ROOT/references/riscv-isa-sim/build/spike}"
GCC="${RISCV_GCC:-riscv64-unknown-elf-gcc}"
NM="${RISCV_NM:-riscv64-unknown-elf-nm}"

TEST="${1:-riscv_arithmetic_basic_test}"
ITER="${2:-1}"
TARGET="${TARGET:-rv32imafd}"
ISS="${ISS:-sail}"
OUT="${OUT:-$ROOT/output/riscvdv}"
NIIGO_TEST_TIMEOUT="${NIIGO_TEST_TIMEOUT:-120}"
export NIIGO_TEST_TIMEOUT

case "$ISS" in
  sail|spike) ;;
  *) echo "error: ISS must be sail or spike (got '$ISS')" >&2; exit 2 ;;
esac

case "$TARGET" in
  rv32imafd) XLEN=32; MARCH=rv32imafd_zicsr_zifencei; MABI=ilp32; SAILCFG_DIR=niigo-rv32g
             SPIKE_ISA=rv32imafd_zicsr_zifencei_zicntr_zihpm ;;
  rv64imafd) XLEN=64; MARCH=rv64imafd_zicsr_zifencei; MABI=lp64;  SAILCFG_DIR=niigo-rv64g
             SPIKE_ISA=rv64imafd_zicsr_zifencei_zicntr_zihpm ;;
  *) echo "error: TARGET must be rv32imafd or rv64imafd (got '$TARGET')" >&2; exit 2 ;;
esac
SAILCFG="$ROOT/references/riscv-tests/config/cores/niigo-lake/$SAILCFG_DIR/sail.json"

# --- locate a python that can run the pyflow generator (needs vsc) ----------
pick_py() {
  local cands=("${RISCVDV_PY:-}" "/home/mizuki/venv/bin/python" python3 python)
  for p in "${cands[@]}"; do
    [ -z "$p" ] && continue
    if command -v "$p" >/dev/null 2>&1 && "$p" -c "import vsc, bitstring" >/dev/null 2>&1; then
      command -v "$p"; return 0
    fi
  done
  return 1
}
PY="$(pick_py || true)"
if [ -z "$PY" ]; then
  echo "error: no python with pyvsc+bitstring found. Set RISCVDV_PY=/path/to/venv/bin/python" >&2
  exit 2
fi
PYDIR="$(dirname "$PY")"

# VTOP: override the simulator binary (default = the in-tree build). Set this to
# a private immutable copy when fanning out parallel runs so a stray `make` in
# one worker can't clobber the shared binary. Exported so run_riscv_test.sh sees it.
VTOP="${VTOP:-$ROOT/output/simulation/verilator_obj/Vtop}"
export VTOP
[ -x "$VTOP" ] || { echo "error: $VTOP missing -- build with AGENT_DEBUG first" >&2; exit 2; }

mkdir -p "$OUT"
if [ "$ISS" = "spike" ]; then GOLDEN_BIN="$SPIKE"; else GOLDEN_BIN="$SAIL"; fi
echo "== riscv-dv: target=$TARGET test=$TEST iters=$ITER iss=$ISS python=$PY"
echo "   out=$OUT  $ISS=$([ -x "$GOLDEN_BIN" ] && echo present || echo MISSING)"

# --- 1. generate (or directed ASM mode) -------------------------------------
# Directed mode: ASM=<space-separated list of .S files> bypasses the random
# generator and runs the pre-written program(s) straight through the
# compile/golden/DUT/diff legs below. Used for hand-written corner-case torture
# tests that the random generator cannot reliably hit (precise store-load
# aliasing, exact Sv39 superpage layouts + A/D transitions, sfence/TLB
# coherence, self-modifying code, RAS depth, exception precedence). The
# directed .S must be linkable with riscv-dv's scripts/link.ld (ENTRY=_start at
# 0x80000000, a .tohost section) and terminate via `ecall` (the trace
# converters' stop point) with a trap handler that writes `gp` to `tohost`
# (HTIF exit, so all three sims halt). Default (ASM unset) = the random path.
if [ -n "${ASM:-}" ]; then
  # shellcheck disable=SC2206
  ASMS=($ASM)
  [ "${#ASMS[@]}" -gt 0 ] || { echo "error: ASM set but empty" >&2; exit 2; }
  for a in "${ASMS[@]}"; do
    [ -f "$a" ] || { echo "error: ASM file not found: $a" >&2; exit 2; }
  done
  echo "== directed mode: ${#ASMS[@]} asm file(s)"
else
# The pyflow test entry does a *relative* `sys.path.append("pygen/")`, so the
# generator child only resolves its package when cwd is the riscv-dv root. Run
# there (the -o output dir is absolute) and also export the absolute pygen path.
GENDIR="$OUT/gen"
rm -rf "$GENDIR"
# pyflow (pure-Python constraint solving) is slow for complex tests; bound it so
# an over-long gen fails cleanly instead of hanging. riscv_arithmetic_basic_test
# is seconds; riscv_rand_instr_test can take many minutes -- raise GEN_TIMEOUT.
GEN_TIMEOUT="${GEN_TIMEOUT:-300}"
# TESTLIST: optional custom testlist (e.g. a trimmed one) passed to run.py.
# When set, run.py uses ONLY that file, so it must contain $TEST. Resolve to an
# absolute path since the gen step runs with cwd=riscv-dv root.
TESTLIST_ARG=()
if [ -n "${TESTLIST:-}" ]; then
  case "$TESTLIST" in /*) ;; *) TESTLIST="$ROOT/$TESTLIST" ;; esac
  [ -f "$TESTLIST" ] || { echo "error: TESTLIST not found: $TESTLIST" >&2; exit 2; }
  TESTLIST_ARG=(--testlist "$TESTLIST")
fi
# SEED: pin the generator seed so a fan-out of single-iteration calls (each a
# distinct SEED) produces distinct, reproducible programs that parallelize across
# cores -- instead of one slow ITER=N call generating N programs sequentially.
SEED_ARG=()
[ -n "${SEED:-}" ] && SEED_ARG=(--seed "$SEED")
( cd "$RVDV" && PATH="$PYDIR:$PATH" PYTHONPATH="$RVDV/pygen${PYTHONPATH:+:$PYTHONPATH}" \
    timeout "$GEN_TIMEOUT" "$PY" "$RVDV/run.py" \
    --target "$TARGET" --simulator pyflow --steps gen \
    "${TESTLIST_ARG[@]}" "${SEED_ARG[@]}" \
    --test "$TEST" -i "$ITER" -o "$GENDIR" >/dev/null ) || {
  echo "error: generation failed or exceeded GEN_TIMEOUT=${GEN_TIMEOUT}s for '$TEST'" \
       "(pyflow is slow; raise GEN_TIMEOUT or pick a lighter test)" >&2
  exit 1
}

mapfile -t ASMS < <(find "$GENDIR/asm_test" -name "${TEST}_*.S" | sort)
[ "${#ASMS[@]}" -gt 0 ] || { echo "error: generation produced no .S files" >&2; exit 1; }
fi

pass=0; fail=0
for asm in "${ASMS[@]}"; do
  name="$(basename "$asm" .S)"
  wd="$OUT/$name"; mkdir -p "$wd"
  elf="$wd/test.elf"

  # --- 2. compile -----------------------------------------------------------
  "$GCC" -march="$MARCH" -mabi="$MABI" -static -mcmodel=medany \
    -fvisibility=hidden -nostdlib -nostartfiles \
    -I "$RVDV/user_extension" -T "$RVDV/scripts/link.ld" \
    "$asm" -o "$elf"

  # entry / tohost are needed by BOTH the golden leg (spike bootrom skip) and the
  # DUT leg (niigo trampoline skip + HTIF terminate), so resolve them once here.
  entry="$(riscv64-unknown-elf-readelf -h "$elf" 2>/dev/null | awk '/Entry point/{print $NF}')"
  entry="${entry#0x}"
  tohost="$("$NM" "$elf" 2>/dev/null | awk '$3=="tohost"{print $1}')"

  # --- 3. golden trace from the ISS (sail | spike) -------------------------
  # Both converters emit the SAME riscv-dv CSV convention as
  # niigo_log_to_trace_csv.py (one row per committed instr, integer-GPR writes
  # only, stop at-but-excluding the first ecall) so the verdict logic below is
  # shared. Spike runs a 0x1000..0x1010 reset trampoline before the ELF entry;
  # --start-pc drops it to align the window with Sail/niigo (which start at entry).
  if [ "$ISS" = "spike" ]; then
    if [ ! -x "$SPIKE" ]; then echo "  [$name] SKIP: Spike not present at $SPIKE" >&2; continue; fi
    # SPIKE_EXTRA: extra spike flags for directed tests whose corner needs a
    # matching reference config (e.g. --misaligned so spike handles a misaligned
    # access the niigo LSQ splits instead of trapping it).
    timeout "${SPIKE_TIMEOUT:-120}" "$SPIKE" --isa="$SPIKE_ISA" ${SPIKE_EXTRA:-} \
      -l --log-commits --log="$wd/golden.trace" \
      --instructions="${SPIKE_INSN_LIMIT:-2000000}" "$elf" \
      >"$wd/golden.stdout" 2>&1 || true
    golden_msg="$("$PY" "$ROOT/scripts/spike_trace_to_csv.py" \
      --log "$wd/golden.trace" --csv "$wd/golden.csv" --xlen "$XLEN" \
      ${entry:+--start-pc "$entry"} 2>&1)"
  else
    if [ ! -x "$SAIL" ]; then echo "  [$name] SKIP: Sail not present at $SAIL" >&2; continue; fi
    "$SAIL" --config "$SAILCFG" --trace-instr --trace-gpr \
      --trace-output "$wd/golden.trace" --inst-limit 2000000 "$elf" \
      >"$wd/golden.stdout" 2>&1 || true
    golden_msg="$("$PY" "$ROOT/scripts/sail_trace_to_csv.py" \
      --log "$wd/golden.trace" --csv "$wd/golden.csv" --xlen "$XLEN" 2>&1)"
  fi
  echo "$golden_msg" >&2

  # --- 4. DUT trace from Vtop ----------------------------------------------
  # niigo resets at 0x00400000; NIIGO_BOOTSTRAP=1 installs the trampoline that
  # jumps to the program entry (0x80000000). tohost lets the testbench's HTIF
  # monitor end the run cleanly (the program finishes via
  # test_done: ecall -> sw gp,tohost) instead of spinning.
  if [ -n "$tohost" ]; then
    export NIIGO_VTOP_ARGS="+tohost=$tohost"
  else
    unset NIIGO_VTOP_ARGS || true
  fi
  # run_riscv_test.sh reports FAILED for non-ACT programs (no a0==10 self-check);
  # that's expected -- we only need its sim.log, so ignore the exit status.
  NIIGO_BOOTSTRAP=1 "$ROOT/scripts/run_riscv_test.sh" "$elf" "$wd/dut" >/dev/null 2>&1 || true
  niigo_msg="$("$PY" "$ROOT/scripts/niigo_log_to_trace_csv.py" \
    --log "$wd/dut/sim.log" --csv "$wd/niigo.csv" --xlen "$XLEN" \
    ${entry:+--start-pc "$entry"} 2>&1)"
  echo "$niigo_msg" >&2

  # --- 5. verdict -----------------------------------------------------------
  # Termination/length gating BEFORE the GPR compare: instr_trace_compare's
  # in-order pass is blind to one trace ending early, so a hung or diverged DUT
  # that matches on its (shorter) prefix would otherwise read as PASS. Both
  # sides must reach the program's terminating ecall and produce equal-length
  # traces; only then is the GPR-by-GPR compare meaningful.
  golden_stop="$(printf '%s\n' "$golden_msg" | sed -n 's/.*stop_reason=\([a-z]*\).*/\1/p' | tail -1)"
  niigo_stop="$(printf '%s\n' "$niigo_msg" | sed -n 's/.*stop_reason=\([a-z]*\).*/\1/p' | tail -1)"
  niigo_lastpc="$(printf '%s\n' "$niigo_msg" | sed -n 's/.*last_pc=\([0-9a-fx]*\).*/\1/p' | tail -1)"
  n_golden="$(printf  '%s\n' "$golden_msg" | sed -n 's/.*wrote \([0-9]*\) .*/\1/p' | tail -1)"
  n_niigo="$(printf '%s\n' "$niigo_msg" | sed -n 's/.*wrote \([0-9]*\) .*/\1/p' | tail -1)"

  : >"$wd/compare.log"
  PYTHONPATH="$RVDV/scripts" "$PY" "$RVDV/scripts/instr_trace_compare.py" \
    --csv_file_1 "$wd/golden.csv" --csv_file_2 "$wd/niigo.csv" \
    --csv_name_1 "$ISS" --csv_name_2 niigo --log "$wd/compare.log" >/dev/null 2>&1 || true

  if [ "$niigo_stop" != "ecall" ] && [ "$golden_stop" = "ecall" ]; then
    echo "  [$name] FAIL  -- DUT never reached test_done (hang/incomplete near pc=$niigo_lastpc;" \
         "niigo=$n_niigo vs $ISS=$n_golden instrs) -- see $wd/dut/sim.log"
    fail=$((fail + 1))
  elif [ -n "$n_golden" ] && [ "$n_niigo" != "$n_golden" ]; then
    echo "  [$name] FAIL  -- trace-length divergence (niigo=$n_niigo vs $ISS=$n_golden instrs);" \
         "first diff in $wd/compare.log"
    fail=$((fail + 1))
  elif grep -q "\[PASSED\]" "$wd/compare.log"; then
    echo "  [$name] PASS  -- $(grep "\[PASSED\]" "$wd/compare.log" | head -1) ($n_niigo instrs)"
    pass=$((pass + 1))
  else
    verdict="$(grep -E "\[FAILED\]|Mismatch" "$wd/compare.log" | head -1)"
    echo "  [$name] FAIL  -- ${verdict:-no verdict; see $wd/compare.log}"
    fail=$((fail + 1))
  fi
done

echo "== riscv-dv summary: $pass passed, $fail failed (of $((pass + fail)))"
[ "$fail" -eq 0 ]
