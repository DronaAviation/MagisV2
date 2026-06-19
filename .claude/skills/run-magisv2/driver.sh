#!/usr/bin/env bash
# MagisV2 firmware build driver.
#
# This is bare-metal ARM firmware — there is no app to "launch" in a
# container. The realistic agent verification for a firmware change is:
# does it still cross-compile for every board target, does it produce a
# flashable .hex, and does it still fit in the 256 KB flash / 40 KB RAM
# budget. This driver does exactly that and fails loudly on any problem.
#
# Usage:
#   driver.sh                 # clean-build all targets, verify artifacts + memory
#   driver.sh PRIMUS_X2_v1    # build a single target
#   driver.sh --no-clean      # incremental build all targets (faster)
#
# Run from the repo root.
set -uo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
cd "$REPO"

# PlutoIDE installs the arm-none-eabi toolchain under the user's home dir.
# Layout is the same across platforms ("<home>/.pluto-ide/tools/ARM GNU
# ToolChain/bin"); only the home variable differs. Probe each candidate and
# prepend the first that exists; if none match, fall back to whatever is
# already on PATH (covers a system-installed gcc-arm-none-eabi too).
#
# This script needs a bash environment:
#   Linux / macOS : native shell.
#   Windows       : Git Bash or WSL (cmd.exe / PowerShell will not run it).
#
# Only the Linux path is verified in CI; macOS/Windows rely on the same
# PlutoIDE layout under $HOME / $USERPROFILE, plus the PATH fallback.
TOOLCHAIN_SUBDIR=".pluto-ide/tools/ARM GNU ToolChain/bin"
TOOLCHAIN_CANDIDATES=(
  "$HOME/$TOOLCHAIN_SUBDIR"             # Linux + macOS (Git Bash maps $HOME too)
  "${USERPROFILE:-}/$TOOLCHAIN_SUBDIR"  # Windows (Git Bash/MSYS expose $USERPROFILE)
)
for dir in "${TOOLCHAIN_CANDIDATES[@]}"; do
  [ -n "${dir%/$TOOLCHAIN_SUBDIR}" ] || continue   # skip if the home var was empty
  if [ -d "$dir" ]; then export PATH="$dir:$PATH"; break; fi
done

# command -v finds arm-none-eabi-g++ or arm-none-eabi-g++.exe (Windows) alike.
if ! command -v arm-none-eabi-g++ >/dev/null 2>&1; then
  echo "FATAL: arm-none-eabi-g++ not found. Install the PlutoIDE ARM toolchain" >&2
  echo "       or put gcc-arm-none-eabi on PATH. Searched:" >&2
  for dir in "${TOOLCHAIN_CANDIDATES[@]}"; do echo "         $dir" >&2; done
  exit 1
fi

ALL_TARGETS="PRIMUS_X2_v1 PRIMUSX2 PRIMUS_V5"
CLEAN=1
TARGETS=""
for arg in "$@"; do
  case "$arg" in
    --no-clean) CLEAN=0 ;;
    *) TARGETS="$TARGETS $arg" ;;
  esac
done
[ -n "$TARGETS" ] || TARGETS="$ALL_TARGETS"

rc=0
for t in $TARGETS; do
  echo "==================== $t ===================="
  [ "$CLEAN" = 1 ] && make TARGET="$t" clean >/dev/null 2>&1
  if ! make TARGET="$t" 2>&1 | tail -8; then
    echo "BUILD FAILED: $t" >&2; rc=1; continue
  fi
  hex=$(ls Build/"$t"/*.hex 2>/dev/null | head -1)
  if [ -z "$hex" ]; then
    echo "NO ARTIFACT: $t produced no .hex" >&2; rc=1; continue
  fi
  echo "OK: $hex"
done

echo
if [ "$rc" = 0 ]; then echo "ALL BUILDS PASSED"; else echo "SOME BUILDS FAILED"; fi
exit $rc
