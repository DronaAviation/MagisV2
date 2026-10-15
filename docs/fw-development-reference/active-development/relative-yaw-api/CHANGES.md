# Relative Yaw API - Changes

[README](README.md) · [TASKS](TASKS.md) · [SCOUT](SCOUT.md) · [INVESTIGATION](INVESTIGATION.md) · [CHANGES](CHANGES.md) · [TESTING](TESTING.md)

## Task 2: heading hold is engaged automatically for a user yaw target ( 15 Oct 2026 )

**Why.** log-1 showed that, with AUX1 in neither MAG nor HEADFREE, `updateMagHold ( )` overwrites `magHold` with the
measured heading on every loop, so every user yaw target is lost one loop after it is set. This applied to both
setters. log-2 showed that the turns are clean once MAG mode is on. The user's choice: engage heading hold
automatically for both setters ( decisions log ).

| File | Change |
|---|---|
| [mw.cpp:1155-1165](../../../../src/main/mw.cpp#L1155-L1165) | In `userCode ( )`, inside the `runUserCode` branch after the user-loop timer block, under `#ifdef MAG`: `if ( isUserHeadingSet && ! FLIGHT_MODE ( HEADFREE_MODE ) ) ENABLE_FLIGHT_MODE ( MAG_MODE );`. `processRx ( )` re-derives `MAG_MODE` from AUX1 on every loop before `userCode ( )`, and nothing in between reads it, so MAG mode stays on exactly while user code holds a target. It runs on every loop, including the one that calls `plutoLoop ( )`, so it covers the call loop before `updateMagHold ( )` runs. A pilot-selected HEADFREE already holds heading, and forcing MAG would switch off its stick mapping, so it is left alone. |
| [mw.cpp:14](../../../../src/main/mw.cpp#L14) | Banner `Last Modified` set to 15 Oct 2026. |
| [FC-Config.h](../../../../src/main/API/FC-Config.h) | The `setDesiredAngle` and `DesiredAngle_setRelative` comments say heading hold is engaged automatically while Developer Mode is on and a yaw target is held. The relative setter's old "only takes effect in MAGHOLD or HEADFREE mode" is removed. The stick sentence is corrected in task 3. |

**Side effects** ( every reader of `MAG_MODE` was checked ):

- The app's back-flip ( `command.cpp:254`, accepted only in `MAG_MODE` ) is now accepted while user code holds a yaw
  target.
- The MSP status reports `BOXMAG` to the app during that time ( `serial_msp.cpp:856` ).
- `ledstrip.c` and `smartport.c` are not used on these builds.

**Fixes from the task-2 review** ( `pluto-reviewer`, no BLOCKING findings ):

| File | Change | Why |
|---|---|---|
| [mw.cpp:701-707](../../../../src/main/mw.cpp#L701-L707) | The `isMagHoldChanged` re-arm in `updateMagHold ( )` now sets `magHold = isUserHeadingSet ? userHeading : heading` ( it was always `heading` ). | With AUX1 off the latch is set on every loop. On the call loop, MAG mode comes on, the latch fires and the new target is overwritten. `userCode ( )` restores it only on loops that do not call `plutoLoop ( )`, so at a fast user loop rate ( e.g. 3.5 ms ) a single call was lost while `userHeading` kept it. The same re-arm after a pilot yaw now goes straight back to the user target, where before it was one loop later. Task 3 hooks the relative re-base in here. |
| [mw.cpp:1159-1168](../../../../src/main/mw.cpp#L1159-L1168) | Added `sensors ( SENSOR_MAG )` to the condition. The comment now says that `processRx ( )` runs on each RX update ( at least 50 Hz ), not on every loop. | Without a magnetometer, forcing MAG mode would hold nothing and would only open the flip gate and report BOXMAG. |
| [FC-Config-Setpoints.cpp:124-126](../../../../src/main/API-Src/FC-Config-Setpoints.cpp#L124-L126) | `( base + angle % 360 ) % 360` | `base + angle` overflowed for `angle` near `INT32_MAX`. |

**Review notes carried to later tasks**

- The flip gate now opens while a user target is held. This is checked in task 6.
- BOXMAG and `FlightMode_Check ( MAGHOLD )` read true while a target is held. This goes in the task-7 wiki.
- A Wi-Fi flicker now stops a turn when AUX1 is off. This is task 4's scope.

**Build.** PRIMUS_X2_v1: flash 107.5 KB, RAM 17.6 KB. Gate: no new warnings in `src/`.

## Task 3: a relative target is re-based after a pilot yaw, and the header contract ( 15 Oct 2026 )

**Why.** This is the decision from planning. After the pilot yaws and lets go, a relative target is measured again
from the nose, so the craft does not swing back to the old target. An absolute target ( `setDesiredAngle` ) keeps
returning to its bearing. Pilot-yaw blending is unchanged.

| File | Change |
|---|---|
| [API-Utils.cpp](../../../../src/main/API-Src/API-Utils.cpp) / [API-Utils.h](../../../../src/main/API/API-Utils.h) | Two new C-linkage flags. `isUserHeadingRelative` records which setter wrote `userHeading`. `userHeadingStale` is set when yaw was taken off heading hold after the target was set. |
| [FC-Config-Setpoints.cpp:103-104](../../../../src/main/API-Src/FC-Config-Setpoints.cpp#L103-L104) | `setDesiredAngle ( AG_YAW )` sets `isUserHeadingRelative = false` and clears `userHeadingStale`. |
| [FC-Config-Setpoints.cpp:126, 136-137](../../../../src/main/API-Src/FC-Config-Setpoints.cpp#L126) | `DesiredAngle_setRelative` measures from `heading` when the latched target is stale, so a call made while the stick is held is measured from the nose at that call. It then sets `isUserHeadingRelative = true` and clears `userHeadingStale`. |
| [mw.cpp:697-728](../../../../src/main/mw.cpp#L697-L728) | `updateMagHold ( )` sets `userHeadingStale` on the **rising edge** of the yaw stick passing ±300 ( static `yawStickWasOut` ). At the `isMagHoldChanged` re-arm ( stick centred, yaw gyro under ~17°/s ), it re-bases a relative target ( `userHeading = heading` ) if the target is stale, clears the flag, then re-arms on the user target. |
| [FC-Config.h](../../../../src/main/API/FC-Config.h) | The stick behaviour is now documented for both setters: the absolute one returns to its bearing; the relative one re-bases to the nose; a relative call made during a stick hold is measured from the nose at that call. The old wrong claim "the stick discards the setpoint" is removed. |

**Design notes**

- **The re-base is keyed on a stick edge, not on the re-arm alone.** The `isMagHoldChanged` re-arm also fires when
  MAG mode switches on at the first call ( task 2 ), and that must not erase the first turn.
- **It is an edge, not a level.** With a level flag, a relative call made during a stick hold would be overwritten
  at release.
- **`rcData`, not `rcDataPilot`.** This matches the existing heading-hold gate. For yaw the two are the same signal,
  because user code never writes `rcData[YAW]`: `RcCommand_Set` yaw goes to `rcCommand` only, and only failsafe
  centres it ( review finding ).
- **Armed only.** The stick edge counts only while armed, so a stick gesture on the ground ( e.g. stick arming ) does
  not re-base a turn that user code set before takeoff ( review note ).

**Code trace**

| Scenario | Result |
|---|---|
| Relative target → pilot yaw → release | The edge sets the flag. At the re-arm, `userHeading = heading` and `magHold = userHeading`, so the craft holds the new nose. |
| Absolute target → pilot yaw → release | The flag is cleared and not re-based ( absolute ). `magHold = userHeading`, so the craft returns to the bearing ( unchanged ). |
| First relative call with AUX1 off | No stick edge, so the flag is false and the re-arm keeps the target. The turn is flown ( as in log-3 ). |
| Relative call during a stick hold | It is measured from `heading` and clears the flag. With no further edge, the target stands after release. |

**Build.** PRIMUS_X2_v1: flash 107.6 KB, RAM 17.6 KB. Gate: no new warnings in `src/`.

**Fixes from the task-3 review** ( `pluto-reviewer`, no BLOCKING findings; all 8 traced scenarios OK ):

| File | Change |
|---|---|
| [mw.cpp:699-711](../../../../src/main/mw.cpp#L699-L711) | The comment no longer claims that a user `RcCommand_Set` yaw override sets the flag. The edge is gated on `ARMING_FLAG ( ARMED )`. |
| [FC-Config.h](../../../../src/main/API/FC-Config.h) | "measured from the nose" now also covers a call made while the craft is still turning after the release ( the flag clears at the re-arm, under ~17°/s ). The header says user yaw overrides are not flown while a yaw target is held, because heading hold owns yaw. |

**For task 4.** When `isUserHeadingSet` is made to survive a Wi-Fi flicker, `userHeadingStale` and
`isUserHeadingRelative` must survive with it. A flicker holds `rcData`, so it creates no stick edge.

**Build after the fixes.** Flash 107.6 KB, RAM 17.6 KB. Gate: no new warnings in `src/`.

## Task 4: a real dev-mode switch-off holds the heading, and a Wi-Fi flicker keeps the target ( 15 Oct 2026 )

**Why.** These are the decisions from planning and from the task-2 review.

- At a real Developer Mode switch-off ( both setters ), the craft holds where it points instead of finishing a
  half-done turn.
- On Wi-Fi, an RC frame more than 200 ms late ( about every 3 s in an earlier topic's log ) runs the same
  `onLoopFinish ( )` / `onLoopStart ( )` path. Before this task it cleared `isUserHeadingSet`. With AUX1 off, task 2
  then also dropped heading hold, so a turn stopped partway and the queued relative turns lost their base.

**How a switch-off is told from a flicker.** AUX channels **hold their last value** through a signal loss
( `rx.cpp:526-528` hold for `MAX_INVALID_PULS_TIME`, then `getRxfailValue ( )`, where AUTO falls through to HOLD for
AUX ). So the dev-mode AUX range goes false only on a real switch-off, never on a flicker.

| File | Change |
|---|---|
| [mw.cpp:1145-1150](../../../../src/main/mw.cpp#L1145-L1150) | `devSwitchOn` ( AUX2 within the dev-mode range ) is computed once. `runUserCode` is still `devSwitchOn && signal`. |
| [mw.cpp:1200-1210](../../../../src/main/mw.cpp#L1200-L1210) | `isUserHeadingSet = false` is removed from the `callonPilotFinish` block, which is also the flicker path. `onLoopFinish ( )` / `onLoopStart ( )` still re-run on a flicker ( existing behaviour ). |
| [mw.cpp:1209-1219](../../../../src/main/mw.cpp#L1209-L1219) | New: `if ( ! devSwitchOn && isUserHeadingSet )` clears `isUserHeadingSet`, `isUserHeadingRelative` and `userHeadingStale`, and sets `magHold = heading`. It is level-triggered, but in effect one-shot, because it clears its own condition. |
| [mw.cpp:1221-1234](../../../../src/main/mw.cpp#L1221-L1234) | The MAG-mode forcing block ( task 2 ) moves out of the `runUserCode` branch to the end of `userCode ( )`, so heading hold keeps flying the target through a flicker. During a flicker the user-loop restore ( `magHold = userHeading` ) does not run, but nothing overwrites `magHold` either: heading hold stays on, the stick stays centred, and the latch stays clear. |

**Build.** PRIMUS_X2_v1: flash 107.7 KB, RAM 17.6 KB. Gate: no new warnings in `src/`.

**Fixes from the task-4 review** ( `pluto-reviewer`, no BLOCKING findings ):

| File | Change | Why |
|---|---|---|
| [mw.cpp:1209-1224](../../../../src/main/mw.cpp#L1209-L1224) | The same clear also runs on the **armed → disarmed edge** ( static `wasArmed` ). | Without it, a target survived a landing, crash or ARM-switch disarm. With dev mode still on, a craft moved by hand would yaw back to the old target on the next takeoff, at up to `rcCommand` 400. Before task 4 a Wi-Fi flicker cleared it by accident. The edge is used, not the disarmed level, so a target set before arming ( `onLoopStart ( )` on the ground ) survives to takeoff. |

**Review notes carried to task 7 ( wiki )**

- **A long RC loss ( failsafe LAND ).** AUX2 is held, so the user target stays and heading hold keeps flying it while
  the craft descends. It may finish a queued turn. This is yaw only, and it is the same as with the pilot's AUX1 MAG
  switch. Accepted. A `FAILSAFE_MODE` gate would not help, because RX-loss lands through the `LAND` command.
- **A yaw setter called inside `onLoopFinish ( )` at a real switch-off** is wiped by the clear in the same call.
- **Verified by the reviewer.** AUX holds through a loss on MSP ( a missing channel reads 0, which is invalid, so it
  is held ), PPM and CRSF. The `config.cpp:479` default for every AUX channel is `RX_FAILSAFE_MODE_HOLD`. User code
  cannot write AUX channels. Nothing relied on the old clear at `onLoopFinish ( )`.

**Build after the fix.** Flash 107.7 KB, RAM 17.6 KB. Gate: no new warnings in `src/`.


## Task 10: `setDesiredAngle ( AG_YAW )` wraps its value into 0..359 ( 15 Oct 2026 )

**Why.** This was found while writing the task-7 wiki, and confirmed by simulating the heading-hold error. The
`AG_YAW` case stored `angle` unwrapped, and `updateMagHold ( )` folds the error by 360 only once. So a bearing of
**720 or more, or −361 or less, never settles and the craft yaws without stopping**. Task 2 made this reachable without
the app's heading-hold switch. The old header documented yaw in deci-degrees, so `900` meant 90.0° and produced a
spin. The user chose to fix it rather than document it.

| File | Change |
|---|---|
| [FC-Config-Setpoints.cpp:98-115](../../../../src/main/API-Src/FC-Config-Setpoints.cpp#L98-L115) | The `AG_YAW` case is now a braced block. It computes `bearing = angle % 360`, adds 360 if negative, and stores `( int16_t ) bearing` into `magHold` and `userHeading`. Values 0..359 are unchanged. |

**Trace** ( the heading-hold error simulated over every heading 0..359 ):

| Input | Before | After |
|---|---|---|
| 900 | spins | 180, settles |
| 450 | settles at 90 | 90, settles |
| −90 | settles at 270 | 270, settles |
| 720 | spins | 0, settles |
| −400 | spins | 320, settles |
| −361 | spins | 359, settles |
| 359 | settles at 359 | 359, settles |
| INT32_MAX | spins | 127, settles |
| INT32_MIN | spins | 232, settles |

**Build.** PRIMUS_X2_v1: gate clean, no new warnings. Two baseline warnings are fixed: the old implicit
`int32_t` → `int16_t` assignments, now explicit casts on the wrapped value. Flash 108.0 KB with the test
`PlutoPilot.cpp`, RAM 17.6 KB.
