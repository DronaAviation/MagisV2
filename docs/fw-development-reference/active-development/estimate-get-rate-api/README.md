# Estimate_Get Rate API

[README](README.md) · [TASKS](TASKS.md) · [SCOUT](SCOUT.md) · [INVESTIGATION](INVESTIGATION.md)

| | |
|---|---|
| **Status** | **Closed - pending commit** ( 5 Nov 2026, FW 3.15.4, API 1.8.2 ) |
| **Mode** | `fix` |
| **Branch** | `BugFix-June26` |
| **Target** | `PRIMUS_X2_v1` ( `PRIMUS_V5` must stay correct ) |
| **Last updated** | 5 Nov 2026 |
| **Pipeline docs** | [User_Space_API.md](../../fw-architecture-pipeline/subsystems/User_Space_API.md) ( the `Estimate_Get` bullet, via `PIPELINE_UPDATE.md` ); [FC_DATA_API_WIKI.md](../../../API/FC_DATA_API_WIKI.md) |

**Problem.** API test T03 ( B8 ): `Estimate_Get ( Rate, X | Y | Z )` always returns `75`, the stick-rate setting, not
the measured rotation rate. Cause ( proven by code ): the `Rate` case in
[FC-Data-Estimate.cpp](../../../../src/main/API-Src/FC-Data-Estimate.cpp) lines 77-86 returns
`currentControlRateProfile->rates []`, a stored profile value, which the flip code sets to 75. Position Z, Velocity Z and
Angle are correct. Details: [INVESTIGATION.md](INVESTIGATION.md).

**Fix.** Return the measured gyro rate in deg/s x10, rounded `gyroADC [ axis ] / 1.64f`, the same scale as
`Sensor_Get ( Gyroscope )`. The flown DCM attitude integrates `gyroADC` at 16.4 counts per deg/s, which fixes the unit.
Correct the `FC-Data.h` doc ( `int16_t` return, units per case ). API 1.8.2, FW 3.15.4.

## Open items

- Task 1 done ( 4 Nov 2026 ): Rate now `gyroADC / 1.64f` rounded ( deg/s x10 ), header corrected; gate green on `PRIMUS_X2_v1`; reviewer found nothing blocking. Task 2 done: flash / RAM identical to HEAD ( the getter is uncalled and stripped ).
- Task 3 done ( 5 Nov 2026 ): bench log-1 PASS. Rest within +/-5 ( 0.5 deg/s ); one hand turn integrates to -353.8 ( yaw ), +386.7 ( roll ), +356.9 ( pitch ) deg. Roll right and pitch nose down read positive; **yaw right reads negative, opposite to the heading** ( raw gyro sign, D2 ): documented in task 4. `PlutoPilot.cpp` restored to HEAD, clean hex rebuilt. Details: [TESTING.md](TESTING.md).
- Task 4 done ( 5 Nov 2026 ): [PIPELINE_UPDATE.md](PIPELINE_UPDATE.md) staged; the FC-Data wiki gains the `Estimate_Get` sections ( units, signs, examples, "Changes in 1.8.2" ) and confirms the gyro unit as deg/s x10; header docs carry the signs.
- Task 5 done ( 5 Nov 2026 ): topic review found nothing blocking; one wiki example and three doc nits fixed.
- Task 6 done ( 5 Nov 2026 ): graph refreshed; all three targets built clean ( no new warnings ); FW 3.15.4 / API 1.8.2; `User_Space_API.md` and CHANGELOG updated.
- Seen in the log, out of scope: with the app on Wi-Fi the user code restarted 26 times in 122 s and 3.5 % of the log records were lost ( known behaviour ).
- Defaults taken unattended ( see INVESTIGATION.md Q&A ): deg/s x10, raw, bench rotation test, API 1.8.2.
- Out of scope, its own topic: the stale `gyro->scale = 1 / 131` and contradicting comments in `accgyro_icm20948.cpp`
  ( only read by code that is not active on these targets ).
- Position / Velocity X and Y are 0 without UWB: documented, unchanged.

## Next action

None: staged with `.git/MAGISV2_COMMIT_MSG.txt`; the user commits, and the next `pluto-commit` / `pluto-grill` run fills in the hash.
