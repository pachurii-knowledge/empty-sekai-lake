#!/usr/bin/env bash
# run.sh — drive the OpenROAD ASAP7 flow for niigo_soc.
#   ./run.sh [synth|floorplan|place|cts|route]   (default: synth, the self-contained half)
# Env: OPENROAD=<path to openroad binary> CLK_PERIOD_NS=<ns> BUILD_DIR=<dir>
set -euo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
STOP="${1:-synth}"
OR="${OPENROAD:-/home/mizuki/Desktop/workspace/OpenROAD/build/bin/openroad}"

[ -x "$OR" ] || { echo "openroad not built yet at: $OR
  (set OPENROAD=... or build it: cd /home/mizuki/Desktop/workspace/OpenROAD && ./etc/Build.sh -no-gui -no-tests)"; exit 1; }

bash "$HERE/gen_filelist.sh"
echo ">>> openroad flow_asap7.tcl (STOP_AFTER=$STOP)"
STOP_AFTER="$STOP" "$OR" -exit "$HERE/flow_asap7.tcl"
