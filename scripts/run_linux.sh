#!/usr/bin/env bash
#
# Run a long (Linux-class) simulation with a console watch.
#
# Unlike the directed/ACT flow -- which ends when the program ecalls a0=10/11 --
# a booting OS never does that (and the ECALL_ARG_HALT halt is disabled here via
# +no_ecall_halt so the kernel's SBI ecalls don't spuriously stop the sim). The
# run therefore ends one of three ways:
#   * the console watch string appears in the UART log   -> success (exit 0)
#   * the simulator finishes on its own (HTIF tohost)    -> success (exit 0)
#   * the timeout elapses with no match                  -> failure (exit 1)
#
# Per-instruction tracing is left off for throughput. The image is expected to
# already be staged in the run dir (mem.*.bin, optionally mem.image.manifest /
# DTB); pass --kernel/--dtb/--initrd to stage a flat image first.
#
# Usage:
#   scripts/run_linux.sh --dir DIR --watch 'STRING' [--timeout SEC]
#                        [--kernel F@ADDR] [--dtb F@ADDR] [--initrd F@ADDR]
#                        [--no-halt-gate] [--vtop PATH]
set -uo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
DIR=""
WATCH=""
TIMEOUT="${NIIGO_TEST_TIMEOUT:-1800}"
VTOP="$ROOT/output/simulation/verilator_obj/Vtop"
HALT_GATE="+no_ecall_halt"
KERNEL="" DTB="" INITRD=""

while [[ $# -gt 0 ]]; do
    case "$1" in
        --dir)         DIR="$2"; shift 2 ;;
        --watch)       WATCH="$2"; shift 2 ;;
        --timeout)     TIMEOUT="$2"; shift 2 ;;
        --kernel)      KERNEL="$2"; shift 2 ;;
        --dtb)         DTB="$2"; shift 2 ;;
        --initrd)      INITRD="$2"; shift 2 ;;
        --no-halt-gate) HALT_GATE=""; shift ;;
        --vtop)        VTOP="$2"; shift 2 ;;
        *) echo "unknown arg: $1" >&2; exit 2 ;;
    esac
done

[[ -n "$DIR" ]]   || { echo "error: --dir is required" >&2; exit 2; }
[[ -x "$VTOP" ]]  || { echo "error: Vtop not found at $VTOP (build first)" >&2; exit 2; }
mkdir -p "$DIR"

# Optionally stage a flat Linux image (kernel/dtb/initrd) into the run dir.
if [[ -n "$KERNEL$DTB$INITRD" ]]; then
    args=()
    [[ -n "$KERNEL" ]] && args+=(--kernel "$KERNEL")
    [[ -n "$DTB"    ]] && args+=(--dtb "$DTB")
    [[ -n "$INITRD" ]] && args+=(--initrd "$INITRD")
    python3 "$ROOT/scripts/load_linux_image.py" real -o "$DIR" "${args[@]}" || exit $?
fi

LOG="$DIR/console.log"
: > "$LOG"
echo "run_linux: Vtop $HALT_GATE in $DIR (timeout ${TIMEOUT}s, watch='${WATCH}')"

( cd "$DIR" && exec timeout "$TIMEOUT" "$VTOP" $HALT_GATE ) > "$LOG" 2>&1 &
PID=$!

found=1
while kill -0 "$PID" 2>/dev/null; do
    if [[ -n "$WATCH" ]] && grep -qaF "$WATCH" "$LOG"; then
        found=0
        kill "$PID" 2>/dev/null
        wait "$PID" 2>/dev/null
        break
    fi
    sleep 1
done

# The sim may have finished on its own (tohost) before/just as we checked.
if [[ $found -ne 0 && -n "$WATCH" ]] && grep -qaF "$WATCH" "$LOG"; then
    found=0
fi

if [[ $found -eq 0 ]]; then
    echo "run_linux: MATCH '$WATCH' -> success"
    exit 0
fi
echo "run_linux: no match (sim ended or timed out); tail of console:"
tail -3 "$LOG" >&2
exit 1
