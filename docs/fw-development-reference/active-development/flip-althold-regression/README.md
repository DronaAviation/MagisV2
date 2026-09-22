# Flip / Altitude Hold Regression

[README](README.md) · [TASKS](TASKS.md) · [INVESTIGATION](INVESTIGATION.md) · [CHANGES](CHANGES.md) · [TESTING](TESTING.md) · [PIPELINE_UPDATE](PIPELINE_UPDATE.md)

| | |
|---|---|
| **Status** | **Closed** in `c569c0f` ( FW 3.8.1, 21 Sep 2026 ). Pipeline doc, CHANGELOG and CLAUDE.md promoted |
| **Branch** | `BugFix-June26`, base `b91e6a7` |
| **Target** | `PRIMUS_X2_v1` |
| **Last updated** | 21 Sep 2026 |
| **Pipeline doc** | [Altitude_Hold_Estimator.md](../../fw-architecture-pipeline/subsystems/Altitude_Hold_Estimator.md): staged in `PIPELINE_UPDATE.md` (task 7) |

**Problem.** Since `2a8d59a` (altitude setpoint shaping), the app back-flip no longer
rotates the drone. The app keeps BOXBARO on through AUX3, so the flip's
`rcData[THROTTLE] = 2000` is now treated as a shaped stick input capped at 40 cm/s. The
flip's ASCEND phase needs 100 cm/s, so it times out after 2.2 s and never starts the
rotation. Details: [INVESTIGATION.md](INVESTIGATION.md).

**Planned change.** While `flipState >= 1`, altitude hold uses the pre-`2a8d59a` raw rate
path (up to 120 cm/s, no cap or slew), and on flip exit it reseeds the shaped controller
cleanly. Pilot sticks outside a flip stay shaped.

## Open items

- Flyaway after the flip (`log-2.txt`): fixed in task 4, stable in `log-3.txt`.
- Return to the pre-flip height: validated in `log-5.txt` (task 6).
- Post-flip dip: the integrator part is fixed in task 11. The remaining 16-42 cm `EA` dip is
  an estimator effect, accepted and deferred.
- Accepted: post-flip `Vz` bias, `EA` 6-14 cm above `AH` for 3-8 s.
- Known drift, out of scope: `Command_Flip()` ignores its direction argument.
- Temporary diagnostics (the flip log in `PlutoPilot.cpp`, `altholdDebug6/7` writes,
  `DEV_MODE_ALWAYS_ON` in `mw.cpp`): **removed** in task 9 (21 Sep 2026), and the graph
  re-run afterwards.

## Next action

Root cause confirmed by `log-1.txt` (TESTING.md test 1). The flip bypass is in (task 3).
All code tasks are done and flight-validated, and `PIPELINE_UPDATE.md` is staged (task 7).
The graph is refreshed (8), the diagnostics are removed and the final review is done (9).
Next: `commit-magisv2` stages the change and drafts the message; the user commits.
The flight logs were deleted; their results are in TESTING.md.
