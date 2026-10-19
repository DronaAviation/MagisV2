# VL53L1X XRanging API - Investigation

[README](README.md) · [TASKS](TASKS.md) · [SCOUT](SCOUT.md) · [DESIGN](DESIGN.md) · [CHANGES](CHANGES.md) · [TESTING](TESTING.md)

## Task 18: `alt` jumps at the laser return ( log-2, log-3 )

**Symptom.** In log-2 `alt` went 29 → 125 cm in one tick when the laser came back ( `src` 0 → 1, t ≈ 73.4 s ), after a
baro leg in which `alt` read ~95 cm below `D`. log-3 showed the same pattern: 10 → 94 at 26.1 s, 107 → 17 at 72 s, and
after landing −159 → +24.

**Cause ( confirmed ).** These are **frame shifts**, which the design intends.

- `altShiftFrame<T>` ( `altitudehold.cpp`, before `checkReadingFor` ) adds the same delta to `_position_base_z`,
  `_position_z`, the delay buffer, `altHoldFilter.X` / `EstAlt`, `altTarget`, `altGoal`, `AltHold` and the post-flip goal.
- Every controller error is a difference of these, so none changes and the motors see no step.
- The logged `alt` is `EstAlt`, so it shows the shift.

**Sequence in both logs.**

1. A hand ( the user's, in both flights ) under the drone gives a surface step: `src:2`, held on the baro.
2. After 2.5 s the frame is re-based onto the hand, so `alt` becomes the height above the hand.
3. On removal the laser sees the floor again. That is a step, so the baro takes over ( `src:0` ), and above 160 cm it
   is the normal handover. The baro leg runs in the hand frame, which is the ~95 cm "baro low".
4. When the laser returns ( below 140 cm, through the VL53L1X return guard ), the frame shifts back to the floor.

**Evidence.**

- The user saw no movement at the returns in log-3.
- log-2's step at 52 s was the user's hand.
- The source of `altShiftFrame` matches the description above.

**Not settled here.** The log-3 crash at 186 s came after hand tests with the hand ~6 cm under the drone. The real drop
was 80 cm in 0.6 s, with large `r` / `p`. The log has no throttle / setpoint field, so the cause is open. A clean landing
( log-4 ) is part of task 10.
