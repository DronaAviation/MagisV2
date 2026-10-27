# Payload Tilt On Descent

[README](README.md) · [TASKS](TASKS.md) · [SCOUT](SCOUT.md) · [INVESTIGATION](INVESTIGATION.md) · [CHANGES](CHANGES.md) · [TESTING](TESTING.md) · [PIPELINE_UPDATE](PIPELINE_UPDATE.md)

| | |
|---|---|
| **Status** | **Closed - pending commit** ( 27 Oct 2026, FW 3.15.2, API 1.8.0 ) |
| **Branch** | `BugFix-June26` |
| **Target** | `PRIMUS_X2_v1` |
| **Last updated** | 27 Oct 2026 |
| **Pipeline docs** | Promoted at commit from [PIPELINE_UPDATE.md](PIPELINE_UPDATE.md): [Core_PID_Controller.md](../../fw-architecture-pipeline/subsystems/Core_PID_Controller.md) ( `pidRewrite`, the stick-low reset / freeze section ), [RC_Input_Pipeline.md](../../fw-architecture-pipeline/subsystems/RC_Input_Pipeline.md) ( THROTTLE_LOW users ), [Mixer_Pipeline.md](../../fw-architecture-pipeline/subsystems/Mixer_Pipeline.md) ( minthrottle 1100 ), [Altitude_Hold_Estimator.md](../../fw-architecture-pipeline/subsystems/Altitude_Hold_Estimator.md), [FLIGHT_INVARIANTS.md](../../dev-guide/FLIGHT_INVARIANTS.md) and a CLAUDE.md bullet; CHANGELOG, FW 3.15.2 |

**Problem.** With a small payload ( under 10 g ) fixed on one arm, the drone leans toward that arm and
drifts that way whenever the app throttle stick is pulled fully down ( ALT_HOLD + ANGLE ). The lean
follows the payload side ( front, back, left, right ). A partial stick-down is level, the app Land
button is level, and the craft levels again as soon as the stick is centred. The user bisected it
visually: the build at `4657cbf` / `d2e900c` ( 18 Sep 2026 ) is level on the same test, the build at
`2a8d59a` ( altitude setpoint shaping ) leans. Logged in log-1..5.

**Cause ( confirmed in five logs ).** With the raw throttle below `mincheck` ( 1100, the bottom 10 % of
the stick ) `mw.cpp:895-897` zeroes the rate I-term on every RX frame, armed or not. Under `pidRewrite` in
ANGLE mode that I-term is the only thing holding a CG offset, so the craft leans 2-7 deg toward a payload
( 2-3 deg toward its own trim with none ) until the stick comes back above 1100. It happened in all 34 full
stick-downs and in none of the partial ones. **Not a regression:** `4657cbf` and release `v3.0.0` do the same;
their 100 cm/s drop hid it, and `2a8d59a` exposed it by slowing the descent to 30 cm/s. Details:
[INVESTIGATION.md](INVESTIGATION.md).

## Open items

- **Diagnostics removed** ( task 12 ): `PlutoPilot.cpp` is back to HEAD; only the fix remains in `mw.cpp`, `pid.cpp` and
  `pid.h`. Both old-build worktrees are removed.
- **log-1 ( front payload ) confirms the mechanism on HEAD**: each full stick-down puts `Th` below 1100,
  the roll / pitch I-term drops to 0 within 100 ms and the craft leans 2-7 deg toward the payload; partial
  stick-downs stay level ( [TESTING.md](TESTING.md) ). **log-2 ( right arm )** shows the same on roll: a
  4.5-6 deg roll toward the payload, and a partial stick-down to `Th` 1104 stays level. **log-3 ( no payload )**
  still leans 2-3 deg on a full stick-down from the airframe's own roll trim: the lean scales with the I-term the
  reset removes.
- **Control flight on `4657cbf` ( log-4 )**: the old build zeroes the I-term and leans just the same; the
  100 cm/s drop hid it ( the user saw it only barely ). `2a8d59a` exposed the lean by slowing the descent;
  the cause is older.
- **Release `v3.0.0` control flight ( task 7, log-5 )**: the same reset and lean. The behaviour has shipped
  since at least `v3.0.0` ( 10 Feb 2026 ).
- The app sends `Th` 1003-1008 at full stick-down ( log-1 ), so the reset band is reached.

## Fix ( implemented and flown 27 Oct 2026 )

Freeze the `pidRewrite` rate I-term instead of resetting it while the throttle is low and the craft is airborne ( `ARMED && throttleRaisedSinceArm && !isThrottleStickArmed` ); reset everywhere else as today. Design and case table: [INVESTIGATION.md](INVESTIGATION.md). Tasks 8-15 in [TASKS.md](TASKS.md).

**Workaround until it ships:** descend with the stick just above the bottom ( above 1100 ); the shaped descent is nearly at its full 30 cm/s there.

**Status of the fix:** implemented in task 9 ( `mw.cpp`, `pid.cpp`, `pid.h`; [CHANGES.md](CHANGES.md) ), gate clean, reviewed and **flight-validated** ( log-6: the I-term is held through a 6.5 s full stick-down and the craft stays within 0.3 deg of its partial-stick attitude; log-7: take-off, floor hold at full stick, lift-off and Land as before; the residual did not show on a level floor ). Flip covered by review only ( not loggable from the app ).

## Next action

None: the change is staged with a drafted message ( `.git/MAGISV2_COMMIT_MSG.txt` ); the user commits. The commit hash is filled in here the next time `pluto-commit` or `pluto-grill` runs.

Open follow-ups, not part of this topic: THROTTLE mode after an ARM-switch arm still resets the I-term in flight ( see INVESTIGATION.md *Fix* ); `tools/graph_labels.py` is broken by `tools/warnings.py` ( `.claude/TOOLING_BACKLOG.md` item 13 ).
