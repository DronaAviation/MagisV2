# Set desired rate API

[README](README.md) · [TASKS](TASKS.md) · [SCOUT](SCOUT.md) · [INVESTIGATION](INVESTIGATION.md) · [CHANGES](CHANGES.md) · [TESTING](TESTING.md) · [TEST_CODE](TEST_CODE.md)

| | |
|---|---|
| **Status** | **Closed - pending commit** ( 15 Nov 2026, FW 3.15.9, API 1.8.7 ) |
| **Mode** | `fix` |
| **Branch** | `BugFix-June26` |
| **Target** | `PRIMUS_X2_v1` ( all targets built at commit ) |
| **Last updated** | 15 Nov 2026 |
| **Pipeline docs** | [subsystems/User_Space_API.md](../../fw-architecture-pipeline/subsystems/User_Space_API.md), [subsystems/Core_PID_Controller.md](../../fw-architecture-pipeline/subsystems/Core_PID_Controller.md) ( via `PIPELINE_UPDATE.md` at commit ) |

**Problem.** API test T16 ( B30 ): `setDesiredRate ( AG_ROLL, n )` never sticks. `flip ( false )` runs on every
disarmed loop ( `mw.cpp:1404-1412` ) and writes roll / pitch rate = 75 ( `acrobats.cpp:343-349` ); every flip exit
also leaves 75 / 75. Separately, `getDesiredRate` returns the PID's rate setpoint `desiredRate [ ]`, not the set
value, so a read-back never matches on any axis. The setter also narrows `int32_t` into `uint8_t` with no clamp.
Details: [INVESTIGATION.md](INVESTIGATION.md).

**Planned fix.** A RAM-only user rate per axis in `FC-Config-Setpoints.cpp`, clamped ( 0-100 roll / pitch,
0-255 yaw ). The flip code restores roll / pitch to the user rate when one is set, else to 75 as today; the flip's own
75 / 110 while flipping are unchanged. `getDesiredRate` returns the set ( or effective ) rate. Held until reboot or
the next call; never written to EEPROM by the API. A roll / pitch call during a flip is stored but not applied ( task 10 ). New
`docs/API/FC_CONFIG_API_WIKI.md`, API 1.8.6 -> 1.8.7 at commit.

## Open items

- **Remove before release:** `PlutoPilot.cpp` ( repo root ) holds the T16 test code ( task 1 ). Task 9 restores the
  stock file by hand before the commit; `pluto-commit` has no step that does it. The code is kept in
  [TEST_CODE.md](TEST_CODE.md).
- Task 1 done ( 15 Nov 2026 ): test code built; pre-fix baseline skipped ( cause proven in code ). Procedure in
  [TESTING.md](TESTING.md).
- Task 2 ( 15 Nov 2026 ): `setDesiredRate` clamps and holds the rate in `userDesiredRate [ ]`, `getDesiredRate` returns
  it, `userRollPitchRate ( )` added for task 3. Gate green ( flash 122.6 KB ), 3 setter warnings fixed, reviewer 0
  blocking. Details: [CHANGES.md](CHANGES.md).
- Task 3 done ( 15 Nov 2026 ): the flip's normal exits and the disarmed `flip ( false )` restore `userRollPitchRate ( )`
  instead of a hard 75; the flip's own 75 / 110 are unchanged. Gate green, reviewer 0 blocking. **The T16 fix is now
  effective**.
- Task 4 done ( 15 Nov 2026 ): `FC-Config.h` doxygen rewritten for both functions. API 1.8.6 -> 1.8.7 moved to the
  commit task, with the FW bump.
- Task 5 done ( 15 Nov 2026 ): gate green on PRIMUS_X2_v1, flash 122.6 KB, RAM 20.5 KB.
- Task 6 done ( 15 Nov 2026 ): **bench log-2 PASS** ( props off ): no 75 on any of 790 lines, getter = live rate,
  clamp 100 / 0 / 255, value held across arm and disarm. Control run skipped. **Not flight-validated:** the RATE-mode
  stick response and the flip restore ( code reading and reviewer only ). Details: [TESTING.md](TESTING.md).
- Task 7 done ( 15 Nov 2026 ): new [FC_CONFIG_API_WIKI.md](../../../API/FC_CONFIG_API_WIKI.md) and
  [PIPELINE_UPDATE.md](PIPELINE_UPDATE.md). The `spec-compliance` run found two real documentation errors, now fixed:
  a setter call **during** a flip overrides the flip's spin rate ( can disarm in the air; guarded by task 10 ), and
  under a non-default PID controller the rates scale that controller's P / I / D gains in every mode.
- Task 10 done ( 15 Nov 2026 ): a roll / pitch call during a flip is held and applied at the flip's exit, so it can no
  longer change the flip's spin rate ( that path could disarm in mid-air ). Yaw applies at once. Gate green
  ( flash 122.7 KB ), reviewer 0 blocking.
- Task 8 done ( 15 Nov 2026 ): whole-topic review, **0 blocking**. Two documentation errors fixed ( the save / reload
  behaviour, and a flip-timeout sentence in `FC-Config.h` that contradicted the wiki ) plus two nits. Details:
  [CHANGES.md](CHANGES.md).
- Task 9 done ( 15 Nov 2026 ): stock `PlutoPilot.cpp` restored ( the test code is in [TEST_CODE.md](TEST_CODE.md) ),
  graph refreshed, all three targets built clean, FW 3.15.8 -> 3.15.9 and API 1.8.6 -> 1.8.7, `PIPELINE_UPDATE.md`
  applied to `User_Space_API.md`, `Core_PID_Controller.md` and `Power_BMS_Pipeline.md`, CHANGELOG updated, staged.
  **Closed - pending commit**: the user commits.
- Separate finding ( not this topic ): four abnormal flip timeouts end the flip without disarming and leave the flip
  axis at 110 with ANGLE mode off until disarm. Pre-existing, and **unreachable today**: only `BACK_FLIP` is ever
  commanded ( `flipDirection` is only written 0 ). Logged in TASKS.md decisions.
- Out of scope: flip rates and timing, `getDesiredAngle`, EEPROM persistence, other FC-Config APIs, the app's
  MSP_SET_RC_TUNING roll / pitch ( also reset to 75 today; unchanged ).
