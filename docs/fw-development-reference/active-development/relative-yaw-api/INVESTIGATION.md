# Relative Yaw API - Investigation

[README](README.md) · [TASKS](TASKS.md) · [SCOUT](SCOUT.md) · [INVESTIGATION](INVESTIGATION.md)

## Problem

The user added `DesiredAngle_setRelative ( angle_e ANGLE, int32_t angle )` ( `FC-Config-Setpoints.cpp:110-131`,
uncommitted ). It turns the craft by `angle` degrees from its current heading. `setDesiredAngle ( AG_YAW, x )`
commands an absolute compass bearing instead.

In the user's words, the setter "only works if I give manual yaw stick input from the controller". In the failing
case, Developer Mode is on and the call is made, but the craft holds its heading. Only after the pilot nudges yaw and
lets go does it turn. The requirement: as soon as dev mode is on, the call works with no stick input, like
`setDesiredAngle`.

## Evidence

- A flight observation only: no log and no video.
- The user is **not sure** whether heading hold ( MAG or HEADFREE ) was on in the failing test.
- The call site can be in `onLoopStart`, `plutoLoop` or `onLoopFinish`; the user's code decides the timing with a
  timer or counter. The code itself was not shared.

## How the yaw setpoint reaches the motors

1. **The setter** writes `magHold = userHeading = target` and sets `isUserHeadingSet = true`.
   - `setDesiredAngle`: target = `angle`.
   - `DesiredAngle_setRelative`: target = ( `userHeading` if one is latched, otherwise `heading` ) + `angle`,
     wrapped to 0..359.
2. **`userCode ( )`** ( `mw.cpp:1115` ) runs `plutoLoop ( )` once per `userLoopFrequency` ( 100 ms by default ). On
   every other loop it re-applies `magHold = userHeading` ( `mw.cpp:1143-1145` ).
3. **`updateMagHold ( )`** ( `mw.cpp:695-724` ) runs after `userCode` in the same loop. It applies heading hold only
   when `|rcData[YAW] - 1500| < 300` **and** the craft is in MAG or HEADFREE mode. When that holds, it computes
   `rcCommand[YAW] = constrain ( heading - magHold, ±30 ) × P8[PIDMAG]`, clamped to ±400. Otherwise it sets
   `magHold = heading` and `isMagHoldChanged = true`. When the stick next comes back to centre, the latch clears
   only once the yaw gyro reads `|gyroADC[YAW]/4| < 70`.
4. **Leaving dev mode** clears `isUserHeadingSet` ( `mw.cpp:1170` ) but leaves `magHold` at the last user target.

From this path, the two setters behave the same for a single call. Nothing in it needs a stick input for one and not
the other.

## Hypotheses ( ranked )

| # | Cause | For | Against | Test that settles it |
|---|---|---|---|---|
| H1 | **Heading hold is off.** AUX1 is not in 900-1700 ( neither MAG nor HEADFREE ), so the else-branch sets `magHold = heading` on every loop and the target is wiped. | The user is not sure it was on. It explains "no turn" fully. A stick "turn" is just the pilot's own yaw. | If `setDesiredAngle` really worked in the same setup, H1 should break it too. | Log `FlightMode_Check ( MAGHOLD )` / `( HEADFREE )`. Fly `setDesiredAngle` as a control in the same flight. |
| H2 | **The call pattern adds +x on every tick.** An unguarded call in `plutoLoop` moves the target 90° every 100 ms, so the ±30° clamp dithers and there is no clean turn. | `setDesiredAngle` is normally called every loop, and the habit carries over. | The user says the timing is guarded by a timer or counter, and the symptom is "no turn", not jitter. | Log the target and the call count. Use a one-shot guarded call. |
| H3 | **The `isMagHoldChanged` latch is stuck.** No heading command is produced until the yaw gyro reads under ~17°/s ( the threshold assumes 16.4 LSB/dps, not checked for this IMU ). | It would give "nothing happens" with heading hold on. | It would affect both setters the same way, and a stick nudge sets the latch rather than clearing it. | Log `rcCommand[YAW]` with the stick centred after the call. If it is 0 while heading ≠ target, H3 holds. |
| H4 | **The base is re-sampled on a dev-mode re-entry** ( a Wi-Fi gap over 200 ms runs `onLoopFinish` then `onLoopStart` ). | It is a real path. | It has no link to the stick. | Log `isUserHeadingSet` across RC gaps. |

## Result of the diagnostic flight ( task 1, 15 Oct 2026 )

**H1 is proven** ( [TESTING.md](TESTING.md) §1, log-1 / log-2 ).

1. On the user's normal setup ( `Rx_ESP`, app defaults ), AUX1 puts the craft in neither MAG nor HEADFREE mode:
   `Md:0` on all 690 records.
2. `updateMagHold ( )` therefore takes its else-branch on every loop ( `mw.cpp:719-723` ) and overwrites
   `magHold = heading`.
3. A user target survives only until the end of the loop in which it was written: `Mh` 226 on the call tick, 136 on
   the next.
4. Both setters fail the same way. `setDesiredAngle ( AG_YAW, H0 )` was flown as the control and also had no effect.
   The planning assumption that `setDesiredAngle` works without a stick input does **not** hold in this setup.
5. "It only works with stick input" was the pilot's own yaw ( turning only while the stick was past ±300 ).

With `FlightMode_Set ( MAGHOLD )` called every `plutoLoop` ( log-2 ), every relative and absolute call turned with
no stick input: 90° in about 1.5 s, ±1° hold, exact queueing ( 147 → 237 → 147 → 237 → 147 ).

H2 and H3 are ruled out for one-shot calls. H4 ( a flicker re-base ) was not seen and stays a design concern.

**What this means for the fix.** `DesiredAngle_setRelative` itself is correct. What is missing is that user yaw
needs heading hold on, and nothing in the API turns it on or says so. Task 2 decides which:

- **A:** the yaw setters turn MAGHOLD on by themselves while in dev mode.
- **B:** the API requires the user to call `FlightMode_Set ( MAGHOLD )`.

## Defects found in the diff ( fixed whatever the cause )

- **The base after a pilot yaw is stale.** After the pilot yaws, `userHeading` still holds the old target. The next
  relative call adds to it, and when the stick is released `mw.cpp:1145` pulls the craft back to the old target.
  Decided: the next call is measured from the nose at stick release, with no snap-back, for relative targets only.
- **The header text is wrong.** It says the stick "discards the setpoint", but the code restores the setpoint after
  release.
- **Dev-mode exit** leaves `magHold` at the last user target, so a half-done turn finishes after dev mode is off.
  Decided: hold the heading at the exit, for both setters.

## Q&A that shaped the plan ( 15 Oct 2026 )

| Question | Answer |
|---|---|
| Mode | `fix` |
| Target | `PRIMUS_X2_v1` |
| Done when | Repeated rotations work: each +90 turns a further 90°, with no drift or wrap error at 0/359. |
| Out of scope | `setDesiredAngle` semantics, yaw PID or rate tuning, blending of pilot yaw |
| Call pattern | Any hook; the user's code times it with a timer or counter. |
| A repeat call before the turn has finished | Adds to the target ( exact queueing ). |
| Dev mode off / on again | Drop the target and hold the heading. The next call after re-entry is measured from the nose. |
| Evidence | A flight observation only |
| Heading hold on in the test? | Not sure / not set |
| Symptom | No turn until a stick nudge |
| Base after a pilot yaw | The nose after the pilot; no snap-back |
| Scope of the dev-exit rule | Both setters |
