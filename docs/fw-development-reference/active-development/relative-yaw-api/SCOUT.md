# Relative Yaw API - Scout brief

[README](README.md) · [TASKS](TASKS.md) · [SCOUT](SCOUT.md) · [INVESTIGATION](INVESTIGATION.md)

`pluto-scout` brief from planning ( 15 Oct 2026 ), kept verbatim.

---

## Scout brief: DesiredAngle_setRelative ( AG_YAW, x ) turns only after a manual yaw-stick input; it should act as soon as Developer Mode is on, like setDesiredAngle
Graph: stale (built c495d6aa, HEAD edfbd615; the graph has no node for the new function)

**Subsystems:** User API, dev-mode gate, heading hold. Pipeline docs: `docs/fw-development-reference/fw-architecture-pipeline/subsystems/User_Space_API.md`, `Core_PID_Controller.md`.

**Key code** (all paths under `F:\MagisV2\src\main\`)
- `API-Src/FC-Config-Setpoints.cpp:110-131` `DesiredAngle_setRelative`: new and uncommitted.
- `API-Src/FC-Config-Setpoints.cpp:85-108` `setDesiredAngle`: the yaw case writes `magHold`, `userHeading` and `isUserHeadingSet=true`.
- `mw.cpp:1115-1176` `userCode`: on non-plutoLoop iterations, `magHold = userHeading` at 1143-1145. `isUserHeadingSet=false` only on leaving dev mode (1170).
- `mw.cpp:695-724` `updateMagHold`: the only consumer. It runs after `userCode` (loop order 1314 → 1320 → 1344).
- `mw.cpp:941-950` `processRx`: `magHold=heading` while disarmed, and on entering BOXMAG.
- `API-Src/RxConfig.cpp:118-129` Rx_ESP: MAG is AUX1 900-1300, HEADFREE is AUX1 1300-1700, dev mode is AUX2 1450-1550.
- `API-Src/API-Utils.cpp:100,101,124`: `userHeading`, `userLoopFrequency=100000` µs, `isUserHeadingSet`.
- `io/serial_msp.cpp:1409` `MSP_SET_HEAD` also writes `magHold`.

**Mode:** fix (diagnosis comes first, because the cause is not proven)

**Current behaviour**
- After the target is worked out, both setters write the same three globals. They are read by the same `updateMagHold` and pass the same gates:
  - stick: `|rcData[YAW]-1500| < 300` (mw.cpp:699)
  - mode: MAG or HEADFREE (mw.cpp:699)
  - output: error clamped to ±30°, ×`P8[PIDMAG]`=40 (config.cpp:210), clamped to ±400 on `rcCommand[YAW]`
- Nothing in the firmware depends on yaw-stick input for one path and not the other. For a single call, relative and absolute behave the same.
- The `PlutoPilot.cpp` in the repo is the empty template, so the user's test code is not available.

**Analysis.** Hypotheses, ranked:
1. **Called every plutoLoop, as is usual with setDesiredAngle (most likely).**
   - Mechanism: each call adds again, because base = `userHeading` once it is latched (line 122). At the default 100 ms tick the target moves 90° per tick, about 900°/s.
   - Result: the error wraps between ±180 and the ±400 command dithers, so the craft does not make a clean turn. Holding the stick past ±300 bypasses heading hold (else-branch, 719-723), so "it works with the stick" is simply the pilot's own yaw.
   - Test: call it once behind a static guard, and log `heading`/`magHold` through Monitor_Print.
2. **Neither MAG nor HEADFREE is active.**
   - Mechanism: `magHold=heading` every loop, so no turn happens.
   - Doubt: setDesiredAngle would fail the same way, so this fits only if the earlier setDesiredAngle test had heading-hold on.
   - Test: print `FlightMode_Check ( MAGHOLD/HEADFREE )`, or call `FlightMode_Set ( MAGHOLD )` every plutoLoop.
3. **One-shot call in `onLoopStart`, with the base re-sampled.**
   - A Wi-Fi gap over 200 ms (User_Space_API.md:26-30) runs onLoopFinish, then onLoopStart again. That clears `isUserHeadingSet` and adds another +90 from the nose.
   - With dev mode on before arming, the turn happens at takeoff.
   - This has no stick link.
4. **`isMagHoldChanged` latch (mw.cpp:696-704).**
   - While the latch is set, no heading command is produced. It clears only when the stick is centred and `|gyroADC[YAW]/4| < 70`. It applies to both setters.
   - The ~17°/s threshold assumes 16.4 LSB/dps and is suspected, not checked.

**Bugs found in the diff**
- **Base after a pilot yaw.** After the pilot yaws, `isUserHeadingSet` stays true. The next relative call is measured from the old target, not from the nose.
- **Stick does not discard the setpoint.** Once the stick is released, line 1145 restores the old `userHeading` and the craft yaws back. The new header comment says the stick "discards the setpoint"; the code does not do that.

**Minimal fix touches**
- Fix the call contract: either document "call once per turn", or make the function edge-safe.
- Choose one base: always `heading`, or `userHeading`.
- Optionally re-sync `userHeading=heading` in the `updateMagHold` else-branch. That changes setDesiredAngle behaviour too.

**Constraints / invariants that apply**
- The 0-359 wrap and the explicit casts are clean: no double, and the casts are -Wconversion-safe.
- Yaw is in degrees, roll and pitch in decidegrees. The new header documents this correctly.
- It is a new public API (pluto-rules API stability). That means a doc for the API, `User_Space_API.md`, and an `API_Version` bump from 1.5.0 to 1.6.0 (Makefile:32).
- The flip needs MAG_MODE (command.cpp:254). The failsafe and LAND paths own only the throttle, so yaw stays with the user.
- rcData/rcDataPilot is not involved: no RC override is used.

**Resource conflicts:** none.

**Already answered by the code**
- Units are degrees, the result is 0-359, and negative values wrap correctly.
- AG_ROLL and AG_PITCH are ignored without any warning.
- Heading hold needs MAG or HEADFREE mode and a centred stick.
- Dev-mode exit clears `isUserHeadingSet`.
- Full yaw authority is reached at an error of 10° or more.

**Questions for the user**
1. Paste your `plutoLoop` / `onLoopStart`: where is the call made, how often, and is it guarded?
2. Should one call be one turn, or should a repeated call re-target from the current nose?
3. Is heading-hold (AUX1) on in the app, or do you call `FlightMode_Set ( MAGHOLD )`?
4. Do you switch dev mode on before or after takeoff?
5. After the pilot yaws, should the setpoint be dropped, or should the craft return to it?
6. What does "only works after stick input" look like: no rotation at all, jitter, or a turn once the stick is released?
7. Can you send a log of `heading`, `magHold` and `FlightMode_Check ( MAGHOLD )`?

**Doc drift**
- No pipeline doc describes heading hold or user yaw.
- The new header's "stick discards the setpoint" claim disagrees with `mw.cpp:1143-1145`.

**Prior work**
- `active-development/yaw-api-test/` and `relative-yaw-api/` are empty stubs.
- Related topics: `user-loop-timer-wrap`, `flip-althold-regression`.
- Last commit on these files: 7fd105a.

**Suggested skills:** pluto-rules, pluto-build (`--gate PRIMUS_X2_v1`), pluto-flighttest, pluto-log-analyst, pluto-commit.

**Pipeline docs to update at the end**
- `User_Space_API.md`: yaw setpoint semantics, including the relative call and how the stick interacts with it.
- `Core_PID_Controller.md`: the heading-hold path through `updateMagHold`.
- There is no FC-Config doc under `docs/API/`, so one may need to be created.
- `CHANGELOG.md` and the API_Version bump.
