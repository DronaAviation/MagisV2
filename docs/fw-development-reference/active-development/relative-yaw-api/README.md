# Relative Yaw API

[README](README.md) · [TASKS](TASKS.md) · [SCOUT](SCOUT.md) · [INVESTIGATION](INVESTIGATION.md) · [CHANGES](CHANGES.md) · [TESTING](TESTING.md)

| | |
|---|---|
| **Status** | **Closed - pending commit** ( 15 Oct 2026, FW 3.13.0, API 1.6.0 ) |
| **Mode** | `fix` |
| **Branch** | `BugFix-June26` |
| **Target** | `PRIMUS_X2_v1`, receiver `Rx_ESP` |
| **Origin** | New, uncommitted `DesiredAngle_setRelative ( angle_e, int32_t )` in `FC-Config-Setpoints.cpp` / `FC-Config.h` |

**Problem.** `DesiredAngle_setRelative ( AG_YAW, x )` should turn the craft by `x` degrees from its own heading
( `setDesiredAngle ( AG_YAW, x )` turns it to a compass bearing ). In flight, with Developer Mode on, the craft does
not turn until the pilot nudges the yaw stick and lets it go. It should turn as soon as dev mode is on and the call
is made, as `setDesiredAngle` does.

**What the code says.** For a single call, both setters write the same three globals ( `magHold`, `userHeading`,
`isUserHeadingSet` ). The same `updateMagHold ( )` consumes them ( `mw.cpp:695-724` ), so the cause is not yet
proven. Heading hold runs only in MAG or HEADFREE mode ( AUX1 900-1700 on `Rx_ESP` ) with the yaw stick centred. The
user did not know whether heading hold was on during the failing test. Ranked causes: [INVESTIGATION.md](INVESTIGATION.md).

**Agreed behaviour.**

- Repeated calls add to the target: 4 × +90 makes 360, with a clean wrap at 0/359.
- After a pilot yaw, the next relative call is measured from the nose once the craft has stopped turning after the release, and the craft does not snap
  back. `setDesiredAngle` keeps its return-to-bearing behaviour.
- When dev mode goes off ( both setters ), `magHold = heading`: the craft holds where it points and does not finish a
  half-done turn.

**Gotchas.**

- The root `PlutoPilot.cpp` is back to the template. The task-6 validation sequence is kept in
  [test/PlutoPilot_task6_validation.cpp.txt](test/PlutoPilot_task6_validation.cpp.txt): copy it over `PlutoPilot.cpp`
  to fly the untested cases ( TESTING.md §3 ), and restore the template before committing.
- New public API: wiki `docs/API/YAW_HEADING_API_WIKI.md`. `API_Version` is 1.6.0 and FW is 3.13.0.

**Open items.**

1. ~~Diagnostic flight ( task 1 )~~. Done: **heading hold was off** ( `Md:0` in log-1 ). Both setters were wiped
   every loop. With MAGHOLD forced ( log-2 ), every call turned with no stick input.
2. ~~Task 2~~. Done: while dev mode is on and a user yaw target is held ( either setter ), the firmware engages MAG
   mode itself, unless the pilot chose HEADFREE. log-3 shows all four turns with no stick input and AUX1 off.
3. ~~Task 4~~. Done: the user yaw target is dropped ( and the heading held ) only on a real dev-mode switch-off or a
   disarm. A Wi-Fi flicker keeps the target and heading hold.
4. Task 6 ( partial, accepted ): log-4 proves cases 1-3, the absolute turn and the disarm clear. **Not flight-tested
   yet:** the hand-yaw re-base and return, dev off mid-turn, a turn through a flicker, no yaw at takeoff after a
   disarm, the app flip, and HEADFREE ( TESTING.md §3 ).
5. ~~Task 10~~. Done: `setDesiredAngle ( AG_YAW, x )` wraps `x` into 0..359. Values of 720 or more, or −361 or less,
   used to spin the drone.
6. Follow-up ( not this topic ): `MSP_SET_HEAD` stores the app heading unwrapped, with the same spin risk. Plan it with
   `/pluto-grill fix`.
