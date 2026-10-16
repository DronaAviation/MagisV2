# Take-off / land command fix

[README](README.md) · [TASKS](TASKS.md) · [SCOUT](SCOUT.md) · [INVESTIGATION](INVESTIGATION.md) · [CHANGES](CHANGES.md) · [TESTING](TESTING.md) · [PIPELINE_UPDATE](PIPELINE_UPDATE.md)

| | |
|---|---|
| **Status** | **Closed - pending commit** ( 16 Oct 2026, FW 3.14.0, API 1.7.0 ) |
| **Mode** | `fix` |
| **Branch** | `BugFix-June26` |
| **Target** | `PRIMUS_X2_v1` ( all targets at commit ) |
| **Origin** | [takeoff-api-ground-hover](../takeoff-api-ground-hover/README.md) ( `check`, findings 16 Oct 2026 ) |

**Problem.** `Command_Arm ( )` followed by `Command_TakeOff ( 100 )` skims the floor instead of climbing. The armed
branch of `takeOff ( )` sets no height ( `command.cpp:97-103` ). This was proven in flight by log-1 vs log-2 of the
check topic. The check also found three hazards in the same command code:

- D1: `Command_TakeOff` / `Command_Flip` during a landing leave the motors running after touchdown, with the stick
  locked out.
- D2: the default `Command_Land ( )` can confirm touchdown at the start of the landing.
- D3: after an ARM-switch arm with the stick low, the ground idle wipes any take-off goal.

**Agreed fix.**

1. D1: `Command_TakeOff` and `Command_Flip` are ignored silently while a landing is in progress.
2. The armed branch of `takeOff ( )` sets the goal once per arm, within 3 s of arming or at ground idle. It releases
   the idle first ( this fixes D3 ). In the air it does nothing, as today.
3. D2: `land ( )` confirms touchdown only after the ramp has run for at least 1 s.
4. D4: the `FC-Control.h` comments, a new `docs/API/FC_CONTROL_API_WIKI.md`, and `User_Space_API.md`. API 1.6.0 → 1.7.0.

**Gotchas.**

- The root `PlutoPilot.cpp` is the template. All validation cases are in [test/PlutoPilot_fix_validation.cpp.txt](test/PlutoPilot_fix_validation.cpp.txt)
  ( `TEST_CASE` 1-7, [TESTING.md](TESTING.md) ).
- User code workarounds until this ships: [check topic findings](../takeoff-api-ground-hover/INVESTIGATION.md).

**Open items.**

1. ~~Code and flights~~. Done: tasks 1-3 and 11 are built and reviewed, and all six validation flights pass ( TESTING.md §2 ).
2. ~~Docs, review, commit~~. Done: pipeline docs promoted, CHANGELOG ( v4.0.0 ), FW 3.14.0 / API 1.7.0, staged for the user's commit.
