# Set default PID profile - Testing

[README](README.md) · [TASKS](TASKS.md) · [SCOUT](SCOUT.md) · [INVESTIGATION](INVESTIGATION.md) · [CHANGES](CHANGES.md) · [TESTING](TESTING.md)

## Method: host simulation ( task 3, 14 Nov 2026 )

The user chose a simulated test over a bench or flight log for this topic. Nothing in the tree calls
`setDefault_PIDProfile ( )`, so no flown path changes; the evidence is whether the function itself writes the right
slots and nothing else.

**What is compiled.** [tools/sim/run.sh](../../../../tools/sim/run.sh) builds, for the PC:

- the **real** [API-Src/FC-Config-PID.cpp](../../../../src/main/API-Src/FC-Config-PID.cpp) ( or any other revision,
  through `PID_API_SRC=` ),
- the **real** [flight/pid_defaults.cpp](../../../../src/main/flight/pid_defaults.cpp) and `common/maths.cpp`,
- the harness [tools/sim/test_set_default_pid.cpp](../../../../tools/sim/test_set_default_pid.cpp).

`pidProfile_t`, `pidIndex_e`, `pid_profile_e`, `PID` and `common/axis.h` come from the real headers.
[tools/sim/stub/](../../../../tools/sim/stub/) shadows only the hardware headers `FC-Config-PID.cpp` includes directly
( 22 files, most of them empty ). Four carry content: `profile_t` reduced to its `pidProfile` member ( the only member
the PID API touches ), `extern currentProfile`, `extern userLoopFrequency`, and the head of the real `rc_alias_e`
( `ROLL = 0, PITCH, YAW` ), which `get_PIDProfile` / `set_PIDProfile` index with. `tools/*` is in `.graphifyignore`,
and the firmware Makefile includes only `src/main`, so the stubs never reach the firmware or the graph.

**Sanitizers.** clang++ 18.1.8 ( `x86_64-pc-windows-msvc` ) with `-fsanitize=address,undefined
-fno-sanitize-recover=all`. The no-recover flag matters: without it UBSan prints its report and the program carries
on, so a fault only UBSan sees would still reach the `PASS` line with exit 0. The runner also fails the run if the
stderr holds a sanitizer report at all. On this machine UBSan alone and every g++ sanitizer fail to link; ASan prints a
harmless `interception_win: unhandled instruction` line, which the runner filters. Fallback without sanitizers: MinGW
g++ 14.3.

**Checks ( 103 ).**

| Test | What it asserts |
|---|---|
| Restores only the API slots, profile marked `0xAA` | After `memset` of the whole `profile_t` to `0xAA`: `PID_ROLL` / `PID_PITCH` 40 / 10 / 30, `PID_YAW` 150 / 70 / 50, `PID_ALT` 128 / 0 / 30, `PID_USER` 60 / 30 / 5, both in `P8` / `I8` / `D8` and through `get_PIDProfile ( )` ( the T06 read-back ); `P8` / `I8` / `D8` of `PIDPOS`, `PIDPOSR`, `PIDNAVR`, `PIDLEVEL`, `PIDMAG`, `PIDVEL` still `0xAA`; and, with the five API slots put back, the whole profile byte-identical to before ( controller, float gains, cut frequencies untouched ). |
| Same, marked `0x55` | Rules out a value that only matches the marker by chance. |
| API slots equal `pidProfileDefaultGains ( )` | The restored values are the firmware defaults, the same source `resetPidProfile ( )` uses. |
| `set_PIDProfile ( )` then `setDefault_PIDProfile ( )` twice | T06 as a user runs it: all five profiles tuned to 1 / 2 / 3, restored, idempotent. |

## Results

### Fixed code ( working tree )

```
$ tools/sim/run.sh
compiler: clang++ -fsanitize=address,undefined
under test: src/main/API-Src/FC-Config-PID.cpp
test: restores the five API slots only, profile marked [0xAA]
test: restores the five API slots only, profile marked [0x55]
test: API slots equal pidProfileDefaultGains ( )
test: set_PIDProfile ( ) then setDefault_PIDProfile ( )
PASS: 103 checks, 0 failed
```

`CXX=g++ tools/sim/run.sh`: `PASS: 103 checks, 0 failed`. No sanitizer report. **PASS.**

### Control: the old code ( HEAD `6d1b5cd` ) fails

This shows the harness detects the bug rather than passing whatever it is given.

`PID_API_SRC=<git show HEAD:src/main/API-Src/FC-Config-PID.cpp> tools/sim/run.sh` ( clang, sanitizers ): aborts at the
first call, exit 1.

```
under test: .../FC-Config-PID.HEAD.cpp
.../FC-Config-PID.HEAD.cpp:160:44: runtime error: index 5 out of bounds for type 'const int[5]'
SUMMARY: UndefinedBehaviorSanitizer: undefined-behavior .../FC-Config-PID.HEAD.cpp:160:44
sanitizer reported a fault
```

Line 160 is `currentProfile->pidProfile.P8 [ i ] = defaultP8 [ i ]`. Without `-fno-sanitize-recover=all` the run
continues past this point and ASan then reports the same access as a `global-buffer-overflow`, `READ of size 4`.

The same with `CXX=g++` ( no sanitizers, so it runs to the end ): **`FAIL: 103 checks, 58 failed`**, exit 1. Extract:

```
  FAIL [0xAA] get_PIDProfile ( PID_ALT )  got 100 / 0 / 30, want 128 / 0 / 30
  FAIL [0xAA] get_PIDProfile ( PID_USER ) got 2 / 150 / 70, want 60 / 30 / 5
  FAIL [0xAA] PIDLEVEL P8 untouched       got   0, want 170
  FAIL [0xAA] PIDLEVEL I8 untouched       got   0, want 170
  FAIL [0xAA] PIDLEVEL D8 untouched       got   0, want 170
  FAIL [0xAA] PIDMAG I8 untouched         got  40, want 170
  FAIL [0xAA] PIDVEL P8 untouched         got   1, want 170
  FAIL [0xAA] PIDVEL I8 untouched         got  40, want 170
  FAIL [0xAA] rest of profile unchanged   got   0, want   1
```

All six hidden slots are overwritten, and `PIDLEVEL` is zeroed, which on the craft is the angle loop's gain.
`PID_ALT` reads 100, as in T06. `PID_USER` reads 2 / 150 / 70 here against 0 / 0 / 40 on the STM32. The
values past the tables depend on how each compiler lays out memory, which is what undefined behaviour means.

## How to reproduce

```bash
tools/sim/run.sh                                         # fixed code, expect PASS, exit 0
git show HEAD~N:src/main/API-Src/FC-Config-PID.cpp > /tmp/old.cpp    # any pre-fix revision
PID_API_SRC=/tmp/old.cpp tools/sim/run.sh                # expect the UBSan abort, exit 1
CXX=g++ PID_API_SRC=/tmp/old.cpp tools/sim/run.sh        # expect FAIL: 58 failed, exit 1
```
