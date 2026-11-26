#!/usr/bin/env bash
# Host simulation of setDefault_PIDProfile ( ) ( topic set-default-pid-profile, API test T06 ).
#
# Builds the REAL src/main/API-Src/FC-Config-PID.cpp, src/main/flight/pid_defaults.cpp and
# src/main/common/maths.cpp for the PC, with tools/sim/stub shadowing the hardware headers that
# FC-Config-PID.cpp includes. The structure under test ( pidProfile_t, pidIndex_e, pid_profile_e )
# comes from the real headers. Not the src/test GoogleTest suite, and never part of the firmware build.
#
# Usage ( from the repo root ):
#   tools/sim/run.sh                       # build and run; exit 0 = PASS
#   PID_API_SRC=<file> tools/sim/run.sh    # test another FC-Config-PID.cpp ( e.g. an old revision )
#   CXX=g++ tools/sim/run.sh               # force a compiler
#
# Sanitizers: clang++ with -fsanitize=address,undefined ( catches a read or write past an array,
# which is the T06 bug ). If that cannot build here, it falls back to g++ without sanitizers and
# says so. On Windows, ASan prints a harmless "interception_win: unhandled instruction" line.

set -u
ROOT="$( cd "$( dirname "${BASH_SOURCE[0]}" )/../.." && pwd )"
cd "$ROOT" || exit 2

for d in /c/DevTools/LLVM/bin /c/DevTools/MinGW/bin; do
  [ -d "$d" ] && PATH="$PATH:$d"
done

PID_API_SRC="${PID_API_SRC:-src/main/API-Src/FC-Config-PID.cpp}"
OUT="$( mktemp -d )"
EXE="$OUT/test_set_default_pid.exe"
trap 'rm -rf "$OUT"' EXIT

SRCS=( tools/sim/test_set_default_pid.cpp "$PID_API_SRC" src/main/flight/pid_defaults.cpp src/main/common/maths.cpp )
INCS=( -Itools/sim/stub -Isrc/main )
FLAGS=( -std=gnu++17 -g -O1 -Wall -Wextra -Wno-unused-parameter )

build ( ) {    # $1 = log name, $2 = compiler, rest = extra flags
  local log="$OUT/build.$1.log"; shift
  local cxx="$1"; shift
  command -v "$cxx" > /dev/null 2>&1 || return 1
  "$cxx" "${FLAGS[@]}" "$@" "${INCS[@]}" "${SRCS[@]}" -o "$EXE" > "$log" 2>&1 || return 1
  BUILD_LOG="$log"
  return 0
}

BUILD_LOG=""
# -fno-sanitize-recover=all: a UBSan report must abort, not print and carry on,
# or a UBSan-only bug would still reach the PASS line with exit 0.
if [ -n "${CXX:-}" ]; then
  build req "$CXX" || { cat "$OUT/build.req.log"; exit 2; }
  echo "compiler: $CXX ( no sanitizers requested )"
elif build clang clang++ -fsanitize=address,undefined -fno-sanitize-recover=all; then
  echo "compiler: clang++ -fsanitize=address,undefined -fno-sanitize-recover=all"
elif build gcc g++; then
  echo "compiler: g++ ( sanitizers unavailable here: bounds are checked by the marker tests only )"
  echo "  clang++ could not build; its first error was:"
  grep -m1 -E "error|fatal" "$OUT/build.clang.log" | sed 's/^/  /'
else
  cat "$OUT"/build.*.log 2> /dev/null
  echo "no host compiler could build the simulation"
  exit 2
fi
# Show -Wall -Wextra output, minus the MSVC linker's "Creating library" chatter.
if [ -n "$BUILD_LOG" ] && grep -qv "Creating library" "$BUILD_LOG" 2> /dev/null; then
  grep -v "Creating library" "$BUILD_LOG" | grep -q . && { echo "build warnings:"; grep -v "Creating library" "$BUILD_LOG" | sed 's/^/  /'; }
fi
echo "under test: $PID_API_SRC"

"$EXE" 2> "$OUT/run.err"
rc=$?
grep -v "interception_win" "$OUT/run.err"
# A sanitizer that reported without aborting must still fail the run.
if grep -qE "runtime error|AddressSanitizer|UndefinedBehaviorSanitizer" "$OUT/run.err"; then
  echo "sanitizer reported a fault"
  [ "$rc" -eq 0 ] && rc=1
fi
exit $rc
