#!/usr/bin/env bash
# Object-code snapshot for the vl53l1x-althold-parity equivalence checks ( tasks 1, 5, 6, 8 ).
#
#   snapshot.sh <L0X|L1X|NOLASER> <out-name>
#
# Temporarily enables the laser defines for the chosen configuration in the PRIMUS_X2_v1 target.h,
# clean-builds it with the pluto-build driver, and writes into this folder:
#   <out-name>.dis   objdump -d -r -s of altitudehold.o with debug sections stripped
#   <out-name>.txt   sha256 of that dump and of the .hex, plus the memory bars
# target.h is always restored from a copy, even if the build fails.
#
# Compare two snapshots with:  diff altitudehold-L0X.dis <new>.dis   ( empty = identical )
set -euo pipefail

CFG="${1:?config: L0X | L1X | NOLASER}"
OUT="${2:?output name}"
HERE="$(cd "$(dirname "$0")" && pwd)"
ROOT="$(cd "$HERE/../../../../.." && pwd)"
TARGET=PRIMUS_X2_v1
TH="$ROOT/src/main/target/$TARGET/target.h"
TC="/c/PlutoIDE/tools/ARM GNU ToolChain/bin"
[ -d "$TC" ] && export PATH="$TC:$PATH"

cp "$TH" "$HERE/.target.h.bak"
trap 'cp "$HERE/.target.h.bak" "$TH"; rm -f "$HERE/.target.h.bak"' EXIT

on()  { sed -i "s|^// #define $1 |#define $1 |" "$TH"; grep -q "^#define $1 " "$TH" || { echo "could not enable $1"; exit 2; }; }
off() { sed -i "s|^#define $1 |// #define $1 |" "$TH"; ! grep -q "^#define $1 " "$TH" || { echo "could not disable $1"; exit 2; }; }
# Every configuration sets all three defines explicitly, so a temporary define left on in the working
# tree ( e.g. LASER_TOF_L1x for a bench test ) cannot leak into another configuration's snapshot.
case "$CFG" in
  L0X)     on LASER_TOF;      off LASER_TOF_L1x; on LASER_ALT ;;
  L1X)     off LASER_TOF;     on LASER_TOF_L1x;  on LASER_ALT ;;
  NOLASER) off LASER_TOF;     off LASER_TOF_L1x; off LASER_ALT ;;
  *) echo "unknown config $CFG"; exit 2 ;;
esac

cd "$ROOT"
.claude/skills/pluto-build/driver.sh "$TARGET" > "$HERE/.build-$OUT.log" 2>&1 || { tail -30 "$HERE/.build-$OUT.log"; exit 1; }

OBJ="Build/$TARGET/bin/flight/altitudehold.o"
TMP="$HERE/.alt-$OUT.o"
arm-none-eabi-objcopy --strip-debug --remove-section=.comment "$OBJ" "$TMP"
# drop the header line that carries the object path
arm-none-eabi-objdump -d -r -s "$TMP" | sed '1,/file format/d' > "$HERE/$OUT.dis"
rm -f "$TMP"

HEX="$(ls Build/$TARGET/*.hex | grep -v AltHold2 | head -1)"
{
  echo "config: $CFG   target: $TARGET   HEAD: $(git rev-parse --short HEAD)   other src changes: $(git status --porcelain -- src | grep -v target.h | wc -l)"
  echo "dis sha256: $(sha256sum "$HERE/$OUT.dis" | cut -d' ' -f1)"
  echo "hex: $HEX"
  echo "hex sha256 ( not reproducible: the Makefile bakes __BUILD_DATE__ / __BUILD_TIME__ into the image ): $(sha256sum "$HEX" | cut -d' ' -f1)"
  echo
  grep -E '(Flash|RAM) +\[' "$HERE/.build-$OUT.log" | sort -u
} > "$HERE/$OUT.txt"
rm -f "$HERE/.build-$OUT.log"
cat "$HERE/$OUT.txt"
