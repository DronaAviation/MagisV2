# Payload Tilt On Descent - Investigation

[README](README.md) · [TASKS](TASKS.md) · [SCOUT](SCOUT.md) · [INVESTIGATION](INVESTIGATION.md) · [CHANGES](CHANGES.md) · [TESTING](TESTING.md) · [PIPELINE_UPDATE](PIPELINE_UPDATE.md)

Why the topic exists and what the evidence says. Readable without the code. The hypotheses table is
the planning-time version; the verdicts are in *Findings*.

## Problem

With a small payload ( under 10 g ) fixed on one arm, the drone leans toward that arm and drifts that
way whenever the app throttle stick is pulled fully down in ALT_HOLD + ANGLE. The lean follows the
payload side ( front: pitch forward; back: pitch back; right: roll right; left: roll left ). Observed
visually only, on `PRIMUS_X2_v1`, at planning time; logged in log-1..5 since.

What the user has already established ( 27 Oct 2026 ):

| Observation | Value |
|---|---|
| When | only on a fast / full stick-down pull |
| Partial stick-down ( about one third ) | level |
| App Land button | level |
| After the stick is centred | levels again at once |
| Shape | steady lean of a few degrees, with drift toward the payload |
| Payload | under 10 g, on an arm / frame edge, one side at a time |
| Last good build | `4657cbf` / `d2e900c` ( 18 Sep 2026 ), same full-stick test, level |
| First bad build | `2a8d59a` ( altitude setpoint shaping; only `flight/altitudehold.cpp` changed ) |

`d2e900c` itself changes two README lines, so the bisect sits between `4657cbf` ( baro drift
compensation, datum frozen on arm ) and `2a8d59a` ( the stick now shapes the altitude setpoint: full
stick-down is a 30 cm/s descent with a ramp, instead of 100 cm/s applied at once ).

## What the code does on a stick-down ( HEAD `a81c180` )

1. The app writes the raw throttle into `rcData[THROTTLE]`. In ALT_HOLD the stick only moves the
   altitude setpoint: inside 1460-1540 the craft holds, below 1460 the descent rate rises linearly to
   30 cm/s at 1000 ( `altitudehold.cpp:428`, `:598` ). The velocity loop takes about 112 counts off the
   hover throttle for a 30 cm/s descent, so the motors stay well above minthrottle ( 1100 ).
2. Every loop `mw.cpp:892-897` computes `calculateThrottleStatus ( )` on the raw `rcData[THROTTLE]`
   ( `rc_controls.cpp:131-139`: THROTTLE_LOW below `mincheck` 1100 ) and, if low, calls
   `pidResetErrorAngle ( )` and `pidResetErrorGyro ( )`. There is no armed, airborne or ALT_HOLD check.
3. The default controller is `pidRewrite` ( `pidController = 1` ). In ANGLE mode its outer loop is
   P only, so the rate I-term `errorGyroI` is the only integrator that can hold a constant moment such
   as an off-centre payload. With it zeroed every loop, the craft is held by the P terms alone:
   about 0.78 counts of motor difference per decidegree of lean. A payload that needs about 40 counts
   of difference leans the craft about 5 degrees toward itself, and the craft drifts that way.
4. When the stick leaves the bottom band the I-term is allowed to rebuild ( `I8` 10, `GYRO_I_MAX` 256 )
   and the craft levels.
5. The firmware landing ( Land button, `LAND` ) keeps its throttle at or above `LAND_THROTTLE_MIN`
   1150, so it never enters the reset band.

## Hypotheses ( ranked )

| # | Hypothesis | Needs | For | Against | Test |
|---|---|---|---|---|---|
| 1 | The THROTTLE_LOW I-term reset ( `mw.cpp:895-897` ) was always there; `2a8d59a` slowed the full-stick descent from 100 to 30 cm/s, so the stick is now held below 1100 for 3 s or more instead of about 1 s and the P-only lean has time to show. | `2a8d59a` bad ( matches the bisect ) | Only full stick-down leans ( the reset band is the bottom 10 % ); partial is level; Land button is level ( throttle floor 1150 ); levels at once when centred; only integrator in ANGLE mode. | The app must send below 1100 at full stick ( not yet seen in a log ); the user reports the old build level on the same test. | Task 2 log: raw throttle below 1100 while the I-term sits at 0 and the lean grows toward the payload; partial stick-down keeps the I-term and stays level. |
| 2 | The old build leaned too, but its 1 s drop at up to 100 cm/s hid it. | either | Same reset code at `4657cbf`. | User says the old build stayed level on the same test. | Task 3 control flight on a rebuilt `4657cbf` with the same log. |
| 3 | Light-side motors clip at minthrottle 1100 during the descent, so the mixer loses differential authority. | either | A classic payload-lean mechanism. | Descent commands are smaller now ( about -112 counts vs -300 ); the mixer only redistributes at the high end, and motors sit near hover minus 100. | Task 4: minimum of each motor during the full stick-down; killed if none reaches 1100. |
| 4 | The chuck-arm path ( `rc_controls.cpp:183-187` ) resets the I-terms in free fall. | either | Also an I-term reset. | Needs net acceleration under about 0.33 g; a 30 cm/s descent is nowhere near that. | Task 4: accel magnitude in the log never under 0.33 g. |

## What would count as an explanation

A log on HEAD in which, during a full stick-down with the payload, the raw throttle is below 1100, the
roll or pitch I-term is 0, and the lean grows toward the payload side; and in which a partial stick-down
with the same payload keeps a non-zero I-term and a level attitude. Plus the control log on `4657cbf`
saying whether the same thing happened in the short drop.

## Candidate fix ( planning-time note; the design chosen is under *Fix* below )

Gate the reset at `mw.cpp:895-897` so it only runs when the craft is not flying: not armed, or ground
idle, or `!throttleRaisedSinceArm`. The stick-arm path ( ARM box + THROTTLE_LOW, `rc_controls.cpp:169-180` )
and the chuck-arm path must keep working, and a landing must still not go through the stick limits.
( The planning-time worry about `Command_Land ( speed > 205 )` was wrong: `land ( )` writes its throttle after
`processRx`, so the reset never sees it; see *Other observations*. )

## Doc drift seen on the way

- `Mixer_Pipeline.md:22` says minthrottle is about 1070; the code is 1100 ( `config.cpp:278` ).
- `Core_PID_Controller.md` does not mention the THROTTLE_LOW I-term reset or that controller 1 is
  `pidRewrite`.
- `FLIGHT_INVARIANTS.md` has nothing on in-flight I-term resets.

## Findings ( task 8, 27 Oct 2026 )

Evidence: five PlutoMonitor logs with the task 1 diagnostic line, all with a payload under 10 g except log-3.
Per-log tables and readings: [TESTING.md](TESTING.md).

| Log | Build | Payload | Full stick-downs | Partial stick-downs |
|---|---|---|---|---|
| log-1 | HEAD `a81c180` ( FW 3.15.1 ) | front arm | 5 | 2 |
| log-2 | HEAD | right arm | 4 | 2 |
| log-3 | HEAD | none | 10 | 4 |
| log-4 | `4657cbf` ( FW 3.7.0 ) | front arm | 11 airborne | brief transitions only |
| log-5 | release `v3.0.0` ( FW 3.0.0 ) | front arm | 4 | 2 |

### Cause ( confirmed )

**The THROTTLE_LOW I-term reset at `mw.cpp:895-897` zeroes the rate I-term while the craft is flying.**

1. With the app connected, the throttle stick pulled fully down sends a raw `rcData[THROTTLE]` of 1003-1008
   ( all logs ). Below `mincheck` 1100, `calculateThrottleStatus ( )` returns THROTTLE_LOW
   ( `rc_controls.cpp:131-139` ).
2. On every RX frame the block at `mw.cpp:895-897` then calls `pidResetErrorAngle ( )` and
   `pidResetErrorGyro ( )`, with no armed, airborne or flight-mode check.
3. The default controller is `pidRewrite` ( `pidController` 1 ). In ANGLE mode its angle loop is P only, so
   the rate I-term `errorGyroI` is the only integrator that can hold a constant moment. In hover it carries the
   whole payload: pitch I-term about -72 counts with the front payload ( front motors about 70 counts above the
   rear ), roll about -57 with the right-arm payload. The bare airframe also needs about +35 counts of roll.
4. In every one of the 34 full stick-downs, the I-term on both axes dropped to 0 or -1 within one 100 ms
   record and stayed there while `Th` was below 1100.
5. The craft then leans until the P terms alone produce the moment: 2-7 deg toward the payload ( peaks to
   10 deg ), or 2-3 deg toward its own trim with no payload, and drifts that way.
6. When the stick goes back above 1100, the I-term rebuilds in about 1-1.5 s and the craft levels.

**The boundary is exactly `mincheck`.** Stick-downs whose lowest `Th` was 1103-1104 kept the I-term and stayed
level; 1096-1099 zeroed it. Every partial stick-down ( `Th` 1104-1459, ten in log-1, log-2, log-3 and log-5 )
stayed level with the I-term at its hover value, so descending does not cause the lean; only the reset band does.

**The lean scales with the I-term removed.**

| I-term held in hover | Lean on a full stick-down | Log |
|---|---|---|
| about 35 counts ( no payload ) | 2-3 deg | log-3 |
| about 57 counts ( right payload ) | 4.5-6 deg | log-2 |
| about 70-75 counts ( front payload ) | 2-7 deg averages, peaks 7-10 deg | log-1, log-4, log-5 |

A payload makes an effect larger that every airframe already has; it does not create it.

### History ( confirmed )

**Not a regression of this branch.** `4657cbf` ( log-4 ) and release `v3.0.0` ( log-5 ) zero the I-term and lean
the same way, or more. Their full stick commanded `( rcData - 1500 ) / 4`, clamped to 100 cm/s with no ramp, so
the craft dropped more than 1 m in 1-1.5 s and four of eleven stick-downs in log-4 ended on the floor. The user
watched for the lean on that build and saw it "barely, it dropped too fast to notice". `2a8d59a` ( altitude
setpoint shaping ) slowed the full-stick descent to a ramped 30 cm/s, so the craft now hangs in the air for 3 s
or more, leaning and drifting, in plain sight. The visual bisect found the commit that exposed the lean; the
reset itself is older than `v3.0.0` ( 10 Feb 2026 ).

### Hypotheses: verdicts

| # | Hypothesis | Verdict | Evidence |
|---|---|---|---|
| 1 | THROTTLE_LOW I-term reset, exposed by the slower descent of `2a8d59a` | **Confirmed** | 34 of 34 full stick-downs zero the I-term and lean; every partial stick-down keeps it and stays level; boundary exactly 1100 |
| 2 | The old build leaned too, hidden by the fast drop | **Confirmed** | log-4 ( `4657cbf` ) and log-5 ( `v3.0.0` ): same reset, same or larger lean; user saw it only barely |
| 3 | Light-side motors clip at minthrottle 1100 | **Killed** | lowest motor in any airborne stick-down 1453 ( HEAD ), 1264 ( `4657cbf` ); the 1100 values are floor contacts |
| 4 | Chuck-arm reset in free fall | **Killed** | the I-term zeroing coincides exactly with `Th` < 1100 in every case, and never happens above it |

### Other observations

- **The swings of 10-18 deg after centring the stick were pilot input** ( log-1: user's pitch correction;
  log-2: the motors drive into the roll, which in ANGLE mode only a stick command does ).
- **The app's firmware LAND is not affected**: `land ( )` writes `landThrottle` into `rcData` after `processRx`,
  so the reset never sees it ( scout follow-up ). The Land button was level in every log.
- **Side finding ( out of scope )**: while disarmed with the app stick centred, the rate I-terms wind up to the
  256 clamp; the app take-off clears them at arming ( `command.cpp:176-178` ), so take-off is not affected.

### Doc drift found

- `Mixer_Pipeline.md:22` says minthrottle is about 1070; the code is 1100 ( `config.cpp:278` ).
- `Core_PID_Controller.md` does not mention the THROTTLE_LOW I-term reset, nor that controller 1 is
  `pidRewrite` with a P-only angle loop.
- `FLIGHT_INVARIANTS.md` has nothing on in-flight I-term resets.

### Recommendation

Freeze, rather than reset, the `pidRewrite` rate I-term while the throttle is low and the craft is airborne
( design and case table below; user-confirmed ). **Workaround until it ships:** descend with the stick just
above the bottom of its travel ( above 1100 ): the I-term is kept, and the shaped descent there is already
close to its full 30 cm/s.

## Fix: causal chain and design ( 27 Oct 2026 )

**Chain.** App throttle stick fully down → `rcData[THROTTLE]` 1003-1008 → `calculateThrottleStatus ( )`
returns THROTTLE_LOW ( `rc_controls.cpp:131-139` ) → `pidResetErrorGyro ( )` at `mw.cpp:895-897` on every RX
frame → `errorGyroI` 0 → `ITerm` 0 in `pidRewrite` → the payload moment is held by the P terms alone → lean of
2-7 deg toward the payload, and drift, for as long as the stick stays below 1100.

**Chosen design ( user, 27 Oct 2026 ): freeze, not reset, while airborne.** While the throttle is low, the
craft is armed, the throttle has been raised since arming and it is not in stick-armed ground idle
( `ARMED && throttleRaisedSinceArm && !isThrottleStickArmed` ), `mw.cpp` sets a freeze flag instead of calling
the reset, and `pidRewrite` skips the rate I-term accumulation on all three axes. In every other case the
reset runs exactly as today.

| Case | Today | With the fix |
|---|---|---|
| Full app stick-down in flight ( ALT_HOLD or THROTTLE mode, app take-off arm ) | I-term zeroed, lean | I-term frozen at the hover trim, level |
| Stick-down touchdown, still armed, stick low | I-term zeroed | I-term frozen: no wind-up after the stick goes low ( see residual below ) |
| Disarmed | reset | reset |
| Radio stick-arm, ground idle ( `isThrottleStickArmed` ) | reset | reset |
| THROTTLE mode after an ARM-switch arm with the throttle low ( radio, or the app's ARM button with its stick at the bottom ) | reset | reset ( `isThrottleStickArmed` is cleared only by altitude hold, a take-off goal or a disarm ) |
| Arming paths ( app take-off, `Command_Arm`, chuck, stick-arm ) | reset | reset ( outside this block, or before `throttleRaisedSinceArm` ) |
| Firmware LAND, flip, user `RcCommand_Set` | not reached ( written after `processRx` ) | unchanged |
| PID controllers other than `pidRewrite` | reset | reset ( freeze only for `pidRewrite` ) |

`calculateThrottleStatus ( )` and `throttleStatus` stay unchanged. The baro datum gate and the meaning of
`throttleRaisedSinceArm` stay unchanged.

**Known residual ( reviewer, tasks 9 and 14 ).** A wind-up that happens *before* the stick goes low is kept, not
cleared ( a second way in: after `Command_Arm` with the app stick centred, `throttleRaisedSinceArm` is set at once on
the floor, so a stick-down there freezes rather than clears ): in ALT_HOLD, a touchdown on a partial stick ( 1100-1450 ) with the craft sitting tilted lets
`errorGyroI` wind toward the 256 clamp in about 5-7 s ( `pid_at_min_throttle` keeps the PID acting at idle );
a full stick-down then freezes that value instead of zeroing it, and a lift-off without a disarm starts
tilted. The same wind-up already reached the lift-off today whenever the pilot raised the stick from a
partial position; the fix only removes the accidental escape through the bottom of the stick. Tested in
log-7 ( task 11 ); if it shows, the next step is a reset on a detected ground contact ( there is no touchdown
detector for a stick-down landing today ).

Precision note: at `mw.cpp:894` `rcData` equals `rcDataPilot` except in `skipRxSamples` frames ( after a
suspend / resume ) and the 600 ms invalid-pulse hold ( `rx.cpp:528` ); in those frames the freeze is chosen
instead of the reset, which is harmless.

**Rejected:** skipping the reset in flight without a freeze ( the I-term winds up against the floor after a
stick-down touchdown: about 5-7 s to the 256 clamp at a 3 deg tilt, then a tilted lift-off ); gating on
BARO_MODE ( a stick-arm in ALT_HOLD would wind up at ground idle ).
