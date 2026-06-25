#!/bin/bash
# formal/run.sh -- compile and model-check a CMurphi .m model with the in-tree CMurphi.
#
#   ./formal/run.sh moesi_ccd        # verify the MOESI protocol (expect: No error found)
#   ./formal/run.sh moesi_ccd_neg    # negative control (expect: SWMR ... failed)
#   ./formal/run.sh moesi_ccd -tv    # extra args (e.g. -tv) pass through to the verifier
#
# CMurphi lives at references/cmurphi. Its `mu` compiler must be built once with
#   -O0 -fno-strict-aliasing (an -O2 strict-aliasing bug in the old codegen segfaults):
#     cd references/cmurphi/src && touch lex.yy.c y.tab.c y.tab.h \
#       && make CFLAGS="-O0 -fno-strict-aliasing -fpermissive -w"
# (flex/byacc are NOT needed — the generated parser ships pre-built.)
set -e
HERE="$(cd "$(dirname "$0")" && pwd)"
CM="$HERE/../references/cmurphi"
MU="$CM/src/mu"
INC="$CM/include"
N="$1"; shift || true

[ -x "$MU" ] || { echo "ERROR: build the mu compiler first (see header of this script)"; exit 2; }
cd "$HERE"
rm -f "$N.cpp" "$N"
"$MU" "$N.m"
g++ -O2 -w -o "$N" "$N.cpp" -I"$INC" -lm
exec "./$N" -m8000 "$@"
