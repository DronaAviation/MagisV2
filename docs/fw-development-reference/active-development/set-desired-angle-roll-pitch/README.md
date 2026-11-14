# Set desired angle roll / pitch

[README](README.md) · [TASKS](TASKS.md) · [SCOUT](SCOUT.md) · [INVESTIGATION](INVESTIGATION.md) · [CHANGES](CHANGES.md) · [TESTING](TESTING.md) · [PIPELINE_UPDATE](PIPELINE_UPDATE.md) · [TEST_CODE](TEST_CODE.md)

| | |
|---|---|
| **Status** | **Closed - pending commit** ( 14 Nov 2026, FW 3.15.8, API 1.8.6 ) |
| **Mode** | `fix` |
| **Branch** | `BugFix-June26` |
| **Target** | `PRIMUS_X2_v1` ( all targets built at commit ) |
| **Last updated** | 14 Nov 2026 |
| **Pipeline docs** | [subsystems/User_Space_API.md](../../fw-architecture-pipeline/subsystems/User_Space_API.md), [subsystems/Core_PID_Controller.md](../../fw-architecture-pipeline/subsystems/Core_PID_Controller.md) ( applied from `PIPELINE_UPDATE.md` at commit, with line-reference corrections in five more pipeline docs ) |

**Problem.** API test T11 ( B4 ): `setDesiredAngle ( AG_ROLL, 100 )` read back once, was then overwritten, and the drone
never tilted. `AG_YAW` is OK. The roll / pitch branch in
[FC-Config-Setpoints.cpp](../../../../src/main/API-Src/FC-Config-Setpoints.cpp) ( :92 / :96 ) writes the IMU's measured
attitude `inclination` instead of a setpoint. The next `imuUpdate` overwrites it, and `pidRewrite` builds its ANGLE target
only from `2 * rcCommand`, so the user value never reaches the controller. `AG_PITCH` has the same bug. A value above
700 also trips the crash-angle disarm in the same loop. Details: [INVESTIGATION.md](INVESTIGATION.md).

**Planned fix ( option B ).** Store the user angle in the unused `userDesiredAngle[]` with a held flag, and add it to
the ANGLE-mode target in `pidRewrite` before the existing `max_angle_inclination` clamp. It holds until replaced ( like
`AG_YAW` ) and applies in ANGLE mode once the craft is airborne and while Developer Mode is live ( a 600 ms grace
window covers Wi-Fi flickers ). It is not applied during a flip or a landing, and it is cleared at
dev-off and on the disarm edge ( the yaw pattern, not a Wi-Fi flicker ). No API signature change.

## Open items

- **Removed before release:** the automatic tilt test sequence ( task 1 ) is no longer in `PlutoPilot.cpp`; the
  commit restores the stock file. The code is kept in [TEST_CODE.md](TEST_CODE.md) to re-run T11.
- Task 1 done ( 14 Nov 2026 ): B4 reproduced in [logs/log-1.txt](logs/log-1.txt). Each step-change tick reads back the
  command, then the angle returns to the hover band with no response to +/-5 deg on either axis; heading unchanged.
  Evidence: [TESTING.md](TESTING.md).
- Task 2 done ( 14 Nov 2026 ): `setDesiredAngle` roll / pitch now store into the existing unused `userDesiredAngle [ ]`
  with `isUserDesiredAngle [ ]`, so the attitude estimate is no longer corrupted and the crash-angle trip hazard is
  gone. Nothing flies the value yet ( task 3 ). Gate green, reviewer 0 blocking. Details: [CHANGES.md](CHANGES.md).
- Task 3 done ( 14 Nov 2026 ): `pidRewrite` now adds `userAngleTarget ( )` to its ANGLE target before the
  `max_angle_inclination` clamp, gated on ANGLE mode, no flip and no landing. Gate green ( Flash 122.5 KB, +0.2 KB ),
  reviewer 0 blocking.
- Task 4 done ( 14 Nov 2026 ): the tilt is cleared at a real dev switch-off and on the disarm edge, next to the yaw
  clear. The review added two gates: a 600 ms Developer Mode grace window ( a normal Wi-Fi flicker keeps the tilt, a
  real stall suppresses it ) and an airborne test ( throttle raised and 20 cm above the arm reference ). Gate green
  ( Flash 122.6 KB ), reviewer 0 blocking. The firmware is now flyable.
- Task 5 done ( 14 Nov 2026 ): `FC-Config.h` doxygen and a "Roll and pitch tilt" section in
  [YAW_HEADING_API_WIKI.md](../../../API/YAW_HEADING_API_WIKI.md), each statement checked against a source line
  ( [CHANGES.md](CHANGES.md) ).
- Task 6 done ( 14 Nov 2026 ): clean gate green; the fix costs +256 B flash and +20 B RAM against `ebf4e46`
  ( exact, HEAD built in a temporary worktree ).
- Task 7 done ( 14 Nov 2026 ): **the fix works in flight** ( [logs/log-2.txt](logs/log-2.txt) ). Roll +, pitch + and
  pitch - reached the commanded 5 deg within 0.2-0.3 s and held from a single call; levelling, Wi-Fi resumes, dev-off
  and re-arm all passed. The roll - step hit an obstacle in the small room and is not validated. Details:
  [TESTING.md](TESTING.md).
- Task 8 done ( 14 Nov 2026 ): [PIPELINE_UPDATE.md](PIPELINE_UPDATE.md) stages the `User_Space_API.md` and
  `Core_PID_Controller.md` changes and 45 shifted line references in 7 pipeline docs; CLAUDE.md has the new invariant.
- Task 9 done ( 14 Nov 2026 ): topic review, 0 blocking. The staged pipeline line tables were rebuilt by symbol lookup
  after the review found the mapped-forward numbers were stale; three doc nits fixed; one nit rejected after checking
  the code ( a flicker does not clear the tilt ). Gate re-run green.
- Task 10 done ( 14 Nov 2026 ): graph refreshed; all three targets built ( FW 3.15.8 / API 1.8.6 ), pipeline docs
  and CHANGELOG updated, change staged. The user makes the commit.
- Open: roll -5 deg is not flight-validated ( log-2 collision ); re-fly the sequence in open space when convenient.
- Follow-ups, not in this topic: `getDesiredAngle` returns `2 * rcCommand`, not the flown target, and 0 for yaw;
  `setDesiredRate` writes the persistent rate profile; the `config.cpp:420` comment says 50 deg for 200 ( 20 deg ).
