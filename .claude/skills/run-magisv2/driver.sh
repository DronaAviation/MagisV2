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
# Prepend the first existing dir from the args to PATH; return 0 if one was
# added, 1 if none existed. A drive-letter path ("C:/...") cannot go on PATH
# verbatim — the colon is the PATH separator and would corrupt it — so convert
# "C:/..." -> "/c/..." for PATH after testing existence with the original form.
prepend_first_existing() {
  for dir in "$@"; do
    case "$dir" in /|*//*|"") continue ;; esac      # skip empties from unset home vars
    if [ -d "$dir" ]; then
      case "$dir" in
        [A-Za-z]:/*) drive="$(printf '%s' "${dir%%:*}" | tr 'A-Z' 'a-z')"
                     dir="/$drive/${dir#*:/}" ;;
      esac
      export PATH="$dir:$PATH"; return 0
    fi
  done
  return 1
}

# Windows: the PlutoIDE installer puts the toolchain at a fixed root ("C:\PlutoIDE"),
# NOT under the user profile, and does NOT persist it to the system PATH — the
# extension only sets PATH inside its own spawned terminal, which is gone once
# that terminal closes. So probe the standard install root too, so a fresh Git
# Bash shell still finds the compiler. ${SYSTEMDRIVE} is e.g. "C:"; the /c/...
# form is a belt-and-suspenders fallback.
TOOLCHAIN_SUBDIR=".pluto-ide/tools/ARM GNU ToolChain/bin"
TOOLCHAIN_CANDIDATES=(
  "$HOME/$TOOLCHAIN_SUBDIR"             # Linux + macOS (Git Bash maps $HOME too)
  "${USERPROFILE:-}/$TOOLCHAIN_SUBDIR"  # Windows under user profile (rare)
  "${SYSTEMDRIVE:-C:}/PlutoIDE/tools/ARM GNU ToolChain/bin"  # Windows install root
  "/c/PlutoIDE/tools/ARM GNU ToolChain/bin"
)
prepend_first_existing "${TOOLCHAIN_CANDIDATES[@]}"

# `make` is a system tool on Linux/macOS, but on Windows PlutoIDE ships it under
# its CYGWIN bundle (same install root). Only add that if make isn't already
# resolvable, so we never shadow a system make on Unix.
MAKE_CANDIDATES=(
  "${SYSTEMDRIVE:-C:}/PlutoIDE/tools/CYGWIN 64/bin"
  "/c/PlutoIDE/tools/CYGWIN 64/bin"
)
command -v make >/dev/null 2>&1 || prepend_first_existing "${MAKE_CANDIDATES[@]}"

# command -v finds arm-none-eabi-g++ or arm-none-eabi-g++.exe (Windows) alike.
if ! command -v arm-none-eabi-g++ >/dev/null 2>&1; then
  echo "FATAL: arm-none-eabi-g++ not found. Install the PlutoIDE ARM toolchain" >&2
  echo "       or put gcc-arm-none-eabi on PATH. Searched:" >&2
  for dir in "${TOOLCHAIN_CANDIDATES[@]}"; do echo "         $dir" >&2; done
  exit 1
fi

if ! command -v make >/dev/null 2>&1; then
  echo "FATAL: 'make' not found. On Windows it ships with PlutoIDE under" >&2
  echo "       'CYGWIN 64/bin'; install PlutoIDE or put make on PATH. Searched:" >&2
  for dir in "${MAKE_CANDIDATES[@]}"; do echo "         $dir" >&2; done
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
