# Crash Detect False Trigger - Changes

[README](README.md) · [TASKS](TASKS.md) · [SCOUT](SCOUT.md) · [INVESTIGATION](INVESTIGATION.md) · [CHANGES](CHANGES.md) · [TESTING](TESTING.md)

## Task 1 - Impact limits 15% less sensitive ( 8 Oct 2026 )

[failsafe.cpp:366-368](../../../../src/main/flight/failsafe.cpp#L366) `failsafeOnCrash ( )`: the two lateral
accel limits are raised by 15%. The 70° angle limit, the ANGLE_MODE gate and the flip bypass are unchanged.

| Term | Was | Now | In g ( acc_1G 4096 ) |
|---|---|---|---|
| \|`accSmooth` X/Y\| | 1250 | 1438 | 0.305 → 0.351 g |
| \|`accADC` X/Y\| | 1750 | 2013 | 0.427 → 0.491 g |

Rollback: put 1250 / 1750 back.

## Task 12 - A further 20% less sensitive ( 8 Oct 2026 )

[failsafe.cpp:366-368](../../../../src/main/flight/failsafe.cpp#L366): both limits multiplied by 1.20 on top of task 1.

| Term | Original | Task 1 | Now | In g |
|---|---|---|---|---|
| \|`accSmooth` X/Y\| | 1250 | 1438 | 1726 | 0.305 → 0.421 g |
| \|`accADC` X/Y\| | 1750 | 2013 | 2416 | 0.427 → 0.590 g |

Rollback: 1438 / 2013 ( task 1 ) or 1250 / 1750 ( original ).

## User edit - limits 2500 / 3000, disarm off ( 9 Oct 2026 )

The user changed `failsafe.cpp` by hand:
- limits `accSmooth` 2500 ( 0.610 g ), `accADC` 3000 ( 0.732 g );
- `set_FSI ( Crash )` and `mwDisarm ( )` commented out, so crash detection never disarms in the current
  tree, to collect data. **Restore both before any flight that relies on crash disarm, and before commit.**

## Task 2 - Temporary crash-trip diagnostics ( 9 Oct 2026 )

To be removed in task 6.

- [failsafe.cpp](../../../../src/main/flight/failsafe.cpp#L362):
  - `extern "C"` globals `crashDbgPeakAccSmooth [ 2 ]` and `crashDbgPeakAccADC [ 2 ]` hold the largest |X|, |Y|,
    updated every control loop. `crashDbgTripTerm` holds the term bitmask: 1 angle, 2 `accSmooth`, 4 `accADC`.
    `crashDbgTripCount` counts events, once per contiguous event, through `crashDbgInEvent`.
  - The trip condition is split into `angleTrip`, `smoothTrip` and `rawTrip`; the logic is unchanged.
  - The limits comment is corrected to 2500 / 3000.
- [PlutoPilot.cpp](../../../../PlutoPilot.cpp): `plutoLoop` prints `aS:x,y aA:x,y R: P: T: N: Arm:`
  ( ~105 B per tick on the wire, ≤ 111 worst case ) and then zeroes the peaks. Developer Mode only.

## Task 4 - Test configuration: disarm restored, limits 5000 / 8000 ( 12 Oct 2026 )

[failsafe.cpp](../../../../src/main/flight/failsafe.cpp#L380):
- `set_FSI ( Crash )` and `mwDisarm ( )` are uncommented again;
- limits are now `accSmooth` 5000 ( 1.22 g ) and `accADC` 8000 ( 1.95 g ), a candidate placed between log-1's
  bumps ( ≤ 1.32 g raw ) and its crash ( 5.9 g on the first line ). The final values come from task 5.

The `plutoLoop` limits comment is updated. Gate: no new `src/` warnings, flash 101.7 KB, RAM 14.8 KB.

## Task 5 - Final limits 4000 / 7000 ( 13 Oct 2026 )

[failsafe.cpp:381-384](../../../../src/main/flight/failsafe.cpp#L381): `accSmooth` 5000 → **4000** ( 0.977 g ),
`accADC` 8000 → **7000** ( 1.71 g ), as the user chose; the limits comment now says "final". The `plutoLoop`
comment is updated. Gate: no new `src/` warnings, flash 101.7 KB, RAM 14.8 KB.

Rollback: 5000 / 8000 ( flown in test 2 ).

## Task 6 - Diagnostics removed ( 13 Oct 2026 )

- [failsafe.cpp](../../../../src/main/flight/failsafe.cpp#L362): the `crashDbg*` globals, the peak-hold loop, the
  event flag, the term bitmask and the count are gone. What remains against HEAD is the limits **4000 / 7000** and
  the three named terms `angleTrip` / `smoothTrip` / `rawTrip`, kept by the user's choice; the stale `>5000` trailing comment from HEAD was cut. The logic is the
  original single `if`.
- [PlutoPilot.cpp](../../../../PlutoPilot.cpp): restored to HEAD ( no diff ).

Gate: no new `src/` warnings, flash 100.9 KB ( as before the diagnostics ), RAM 14.8 KB.

## Task 8 prep - Reverted to 5000 / 8000 ( 13 Oct 2026 )

The user set the limits back to **5000 / 8000** by hand in
[failsafe.cpp:367-368](../../../../src/main/flight/failsafe.cpp#L367); the comment above them was then
corrected to match ( 1.22 g / 1.95 g ). Gate: no new `src/` warnings, flash 100.9 KB, RAM 14.8 KB.
The final diff against HEAD is limits 5000 / 8000, three named terms and the two-line comment.

## File index

| File | Tasks |
|---|---|
| [src/main/flight/failsafe.cpp](../../../../src/main/flight/failsafe.cpp) | 1, 12, user edit, 2, 4, 5, 6, revert |
| [PlutoPilot.cpp](../../../../PlutoPilot.cpp) | 2, 4, 5 ( all removed in 6 ) |
