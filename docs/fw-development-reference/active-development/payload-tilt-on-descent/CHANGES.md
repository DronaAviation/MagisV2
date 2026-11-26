# Payload Tilt On Descent - Changes

[README](README.md) · [TASKS](TASKS.md) · [SCOUT](SCOUT.md) · [INVESTIGATION](INVESTIGATION.md) · [CHANGES](CHANGES.md) · [TESTING](TESTING.md) · [PIPELINE_UPDATE](PIPELINE_UPDATE.md)

Task 1 added temporary diagnostics; task 12 removed them. Task 9 is the fix, and the only change left in the tree.

## Task 1: temporary diagnostic log ( 27 Oct 2026 )

| File | Change | Why |
|---|---|---|
| [pid.cpp](../../../../src/main/flight/pid.cpp) | `pidGetGyroITerm ( axis )` after `pidResetErrorGyro ( )`: returns `errorGyroI [ axis ] >> 13`, 0 for an axis outside 0-2. Read-only. | `errorGyroI` is file-static and `axisPID_I` is only written under `BLACKBOX`; the log needs the rate I-term `pidRewrite` adds to the mix. |
| [pid.h](../../../../src/main/flight/pid.h) | Declaration inside the existing `extern "C"` guard. | Linkage for the user file. |
| [PlutoPilot.cpp](../../../../PlutoPilot.cpp) | `extern "C"` declarations of `motor []`, `pidGetGyroITerm` and `millis`; a `START` marker set in `onLoopStart ( )`; one ~121-byte log line in `plutoLoop ( )`. | Fields and units: [TESTING.md](TESTING.md). |

No control-path change in task 1: that build flew exactly as HEAD.

## Task 9: freeze the rate I-term during an airborne stick-down ( 27 Oct 2026 )

| File | Change | Why |
|---|---|---|
| [pid.cpp:69-72](../../../../src/main/flight/pid.cpp#L69-L72) | `static bool gyroITermFrozen`, main loop only. | The freeze state. |
| [pid.cpp:108-111](../../../../src/main/flight/pid.cpp#L108-L111) | `pidSetGyroITermFreeze ( bool )`. | Setter used by `mw.cpp`. |
| [pid.cpp:841-844](../../../../src/main/flight/pid.cpp#L841-L844) | `pidRewrite` skips the `errorGyroI` accumulation while frozen; the clamp and `ITerm = errorGyroI >> 13` still run. | The held trim keeps acting on the motors and cannot wind up. |
| [pid.h:110](../../../../src/main/flight/pid.h#L110) | Declaration inside the existing `extern "C"` guard. | |
| [mw.cpp:897-909](../../../../src/main/mw.cpp#L897-L909) | `freezeGyroITerm = THROTTLE_LOW && ARMED && throttleRaisedSinceArm && !isThrottleStickArmed && controller is pidRewrite`; sets the freeze; the reset runs only when the throttle is low and not frozen. `throttleStatus` unchanged. | The fix: airborne, a stick-down no longer zeroes the payload trim. |
| [mw.cpp:573](../../../../src/main/mw.cpp#L573) | `pidSetGyroITermFreeze ( false )` in `mwDisarm ( )`. | No freeze survives a disarm. |
| mw.cpp banner | Last Modified 27 Oct 2026, HISTORY row. | |

Build ( PRIMUS_X2_v1, `PROJECT=AltX` ): gate passed, no new warnings in `src/`; flash 122.0 KB ( +0.1 KB ), RAM 20.5 KB.

## Task 12: diagnostics and worktrees removed ( 27 Oct 2026 )

| What | How |
|---|---|
| `PlutoPilot.cpp` | restored to HEAD ( `git checkout` ); the task 1 lines were additions only |
| `pid.cpp` / `pid.h` | `pidGetGyroITerm ( )` and its declaration removed; the task 9 fix untouched |
| Worktrees `../MagisV2-4657cbf`, `../MagisV2-v3.0.0` | `git worktree remove --force` ( held only the ported diagnostics and build output ) |

Gate on the fix-only tree ( PRIMUS_X2_v1, `PROJECT=AltX` ): no new `src/` warnings; flash 121.1 KB, RAM 20.5 KB;
`pidSetGyroITermFreeze` linked, `pidGetGyroITerm` absent.

## File index

| File | Tasks | State |
|---|---|---|
| `src/main/flight/pid.cpp` | 9 ( 1 removed in 12 ) | fix |
| `src/main/flight/pid.h` | 9 ( 1 removed in 12 ) | fix |
| `src/main/mw.cpp` | 9 | fix |
| `PlutoPilot.cpp` | 1, removed in 12 | unchanged from HEAD |
