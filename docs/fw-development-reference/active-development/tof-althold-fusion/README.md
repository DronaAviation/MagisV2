# ToF Altitude Hold Fusion

[README](README.md) · [TASKS](TASKS.md) · [INVESTIGATION](INVESTIGATION.md) · [CHANGES](CHANGES.md) · [TESTING](TESTING.md) · [PIPELINE_UPDATE](PIPELINE_UPDATE.md)

| | |
|---|---|
| **Status** | **Closed - pending commit** ( FW 3.9.0, 22 Sep 2026 ). Pipeline doc, CHANGELOG, CLAUDE.md and skills promoted |
| **Branch** | `BugFix-June26`, base `f326096` |
| **Target** | `PRIMUS_X2_v1` with `LASER_TOF` + `LASER_ALT` ( VL53L0X ) for the flights; committed with the defines off in `target.h` ( pilot's choice ) |
| **Last updated** | 22 Sep 2026 |
| **Pipeline doc** | [Altitude_Hold_Estimator.md](../../fw-architecture-pipeline/subsystems/Altitude_Hold_Estimator.md): staged in [PIPELINE_UPDATE.md](PIPELINE_UPDATE.md) ( task 9 ) |

**Problem.** With the VL53L0X feeding the altitude estimator (`LASER_ALT`), altitude hold
bobs slowly ( 10-30 cm, 1-2 s period ) over a flat floor and has never been stable. A hand
placed under the craft makes it climb hard; removing the hand makes it drop hard. The laser is
a hard source switch at 200 cm with no hysteresis, no blending and no step rejection, and each
source change also flips the estimator time constant ( 1.5 s laser / 2 s baro ).
Details: [INVESTIGATION.md](INVESTIGATION.md).

**Root cause ( tasks 2, 12 ).** The laser is clean ( 1.2 cm noise ). The bob comes from the estimated vertical velocity being 0.3-0.4× the real speed in the hover, which leaves the velocity loop under-damped. The loss comes from the 40-count ( ~9.6 cm/s² ) accelerometer Z deadband, which removes most of the small accelerations of a hover.

**Planned change.** One estimator time constant for both sources; a laser/baro handover with
hysteresis ( baro above 160 cm, laser below 140 cm ) that freezes the baro offset at the
handover so `EstAlt` never jumps; and a step detector ( >30 cm within 0.5 s ) that holds the
estimator on baro for 2.5 s before re-basing to the laser and flying the setpoint to the new
clearance on the normal goal profile, symmetric for objects appearing and disappearing.

## Open items

- VL53L1X ( `LASER_TOF_L1x` ): builds, gets the deadband / tau fix, but keeps the old 350 cm hard switch with none of the handover or object logic, and never checks out-of-range. Planned topic: [vl53l1x-althold-parity](../vl53l1x-althold-parity/README.md).

- Handover validated in flight ( `log-5` ). Object hold-off validated for brisk moves, slow slides and moving edges ( `log-7`, task 14 ).
- Accepted: no climb cap, so an object held up under the craft re-bases it repeatedly ( `log-7` climbed ~137 cm ).

- Known limit: after climbing over an object, removing it at a height that makes the laser read ≥ 160 cm hands over to the baro and the craft stays up instead of stepping down.

- App disconnects while logging: the task 12 log line overran the MSP TX ring; trimmed to ~115 bytes. The ~250-byte ceiling in CLAUDE.md is too high with the app connected ( task 9 ).
- Found, out of scope: `Monitor_Print` doubles print 0 for every digit after the first decimal ( `debugPrint` ASCII bug ); separate topic.

- Baseline analysed ( `log-1.txt` ): the bob is a lightly damped loop limit cycle, 30 cm p-p at 4.1 s; estimated `Vz` is only 0.3-0.4× the real speed.
- Root cause found ( `log-2.txt` ): the 40-count accelerometer Z deadband. Fixed in task 3; confirmed in `log-3.txt`: the limit cycle is gone ( sd 6.0 → 2.6 cm ).
- Hover target met in `log-3-temp.txt` ( ±3 cm for 30 s ); the slower settle after a fresh power-up is accepted ( task 13 ). Watched: the bias term winds up at touchdown.
- Handover done ( task 5 ): 160 / 140 cm, bench-verified in `log-4.txt`; in-flight continuity checked in task 8.
- Temporary diagnostics in `PlutoPilot.cpp`: **removed** in task 11 ( file restored to HEAD ). `target.h` restored: PRIMUS_X2_v1 ships with the laser defines off ( pilot's choice ).
- 8 laser-code warnings surfaced by enabling the laser; cleaned in task 3.
- Floor surface for the VL53L0X at 1 m assumed matt and light enough; verify from the baseline.

## Next action

Staged with a drafted message ( `.git/MAGISV2_COMMIT_MSG.txt` ); the user commits. The flight logs stay out of the commit.
