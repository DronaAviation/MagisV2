# Take-off API ground hover

[README](README.md) · [TASKS](TASKS.md) · [SCOUT](SCOUT.md) · [INVESTIGATION](INVESTIGATION.md) · [TESTING](TESTING.md)

| | |
|---|---|
| **Status** | **Closed - pending commit** ( 16 Oct 2026, with the fix topic [takeoff-land-command-fix](../takeoff-land-command-fix/README.md) ) |
| **Mode** | `check` |
| **Branch** | `BugFix-June26` |
| **Target** | `PRIMUS_X2_v1` |
| **Origin** | User report: `Command_Arm ( )` then `Command_TakeOff ( 100 )` in `onLoopStart ( )` |

**Problem.** With ALT HOLD on and the stick centred, `Command_Arm ( )` followed by `Command_TakeOff ( 100 )` spins the
motors up. The drone lifts a few cm and skims the floor. It never climbs to 100 cm.

**What the code says.** `Command_TakeOff` only queues the command. `takeOff ( )` sets the climb goal only when it does
the arming itself. If the drone is already armed, it marks the take-off finished and sets no height
( `command.cpp:97-103` ). `Command_Arm` arms the drone straight away, so the take-off finishes without a goal, and
altitude hold holds the floor at hover thrust. Confirmed in flight ( [TESTING.md](TESTING.md) §2 ).

**Findings.** The cause above is confirmed. The investigation also found four more defects ( D1-D4 ). The serious one
is D1: a `Command_TakeOff` / `Command_Flip` during a landing leaves the drone landing with the motors running and the
stick locked out. Workarounds that need no firmware change, the ranked causes and the recommended fix:
[INVESTIGATION.md](INVESTIGATION.md) §Findings.

**Gotchas.**

- The test versions A, B and C are in [test/](test/) ( [TESTING.md](TESTING.md) ). The root `PlutoPilot.cpp` is the template.
- `Command_Land ( )` with its default speed starts at the touchdown-confirm throttle ( 1200 ) and could disarm the drone
  in the air. The test versions use `Command_Land ( 5 )` ( task 4 ).

**Open items.**

1. ~~A/B flights~~. Done: **H1 confirmed**. With `Command_Arm` first the height is never set and the target stays at 0 cm ( log-1 ); `Command_TakeOff` alone climbs to about 90 cm ( log-2 ).
2. ~~Workaround check~~. Done: `Command_Arm ( )` then `DesiredPosition_setRelative ( Z, 100 )` in the same tick climbs and holds the same as `Command_TakeOff` alone ( log-3 ).
3. Choose a fix ( options in INVESTIGATION.md, task 4 done ). Recommended: `Command_TakeOff` also sets the goal when the drone is already armed on the ground ( API 1.7.0 ), plus the landing-confirm fix and the docs.
4. ~~Task 7~~. Done, **confirmed from the code**: `Command_TakeOff` / `Command_Flip` during a user or RC-loss landing stops the landing logic but keeps the throttle pinned. Nothing disarms after touchdown and the stick is locked out; only the ARM switch stops the motors. Battery auto-land is safe.
