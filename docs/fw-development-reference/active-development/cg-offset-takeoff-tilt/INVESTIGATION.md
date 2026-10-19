# Off-centre payload lean - Investigation

[README](README.md) · [TASKS](TASKS.md) · [SCOUT](SCOUT.md) · [INVESTIGATION](INVESTIGATION.md)

## 1. Problem ( user report, 19 Oct 2026 )

> With the current build I flew the drone, it was good and stable. Now I attached a 9 g servo towards the front.
> On the take-off command the drone tilted forward a little and went forward; on throttle-up it was normal; on
> throttle-down it tilted forward again and went forward; on the land command it was proper. It happens in every
> direction: servo on the right rolls right on take-off and throttle-down, same for left and back. No stick is
> given; I have to give the opposite stick so it does not move.

Conditions ( interview ):

| | |
|---|---|
| Target | PRIMUS_X2_v1, FW 3.15.1 ( `a81c180` ) |
| Payload | 9 g servo, mounted only, not wired ( dead weight ) |
| Take-off | app take-off button ( `MSP_SET_COMMAND` TAKE_OFF ) |
| Mode | ALT_HOLD ( BARO ) on in every phase |
| Throttle-down | stick fully at the bottom |
| PIDs | firmware defaults + accelerometer / compass calibration |
| Known good | `f60c9ad` ( 20 Aug 2026 ) with the same payload: no lean, accelerometer recalibrated after flashing |

## 2. How the controller holds an off-centre mass

`pidRewrite` ( `pidController = 1`, [pid.cpp:735-896](../../../../src/main/flight/pid.cpp#L735) ) runs ANGLE mode
as a **P-only angle loop** ( `desiredRate = errorAngle * P8[PIDLEVEL] >> 4`, `pid.cpp:803` ) feeding a **rate PI
loop**. `errorAngleI` is not used. In steady hover the rate integrator `errorGyroI` ( [pid.cpp:831-836](../../../../src/main/flight/pid.cpp#L831) ,
±`GYRO_I_MAX` 256 ) supplies the torque that the off-centre mass needs, and the angle error goes to zero. With
`errorGyroI` at 0 the torque has to come from the P terms, which only produce it with a standing lean toward the
heavy side. A lean of θ accelerates the level-holding craft sideways at about g·tanθ ( ~0.5 m/s² at 3° ).

## 3. Where the integrator is zeroed

All callers are identical in `f60c9ad` and HEAD:

| Where | When | In flight? |
|---|---|---|
| [mw.cpp:894-897](../../../../src/main/mw.cpp#L894) | every RX frame with `rcData[THROTTLE] < mincheck` 1100 ( [rc_controls.cpp:131-139](../../../../src/main/io/rc_controls.cpp#L131) ) | **yes**: ALT_HOLD keeps flying with the stick low |
| [command.cpp:176-177](../../../../src/main/command/command.cpp#L176) | `takeOff ( )` before `mwArm` | no ( arm ) |
| [FC-Control-Command.cpp:188-189](../../../../src/main/API-Src/FC-Control-Command.cpp#L188) | `Command_Arm` | no ( arm ) |
| [rc_controls.cpp:185-186](../../../../src/main/io/rc_controls.cpp#L185) | chuck-arm | no ( arm ) |

`Command_Land` does not reset: `landThrottle` ( floor 1150 ) is written into `rcData` after `processRx`
( `mw.cpp:389-390`, `:1409-1410` ), and the stick stays centred.

## 4. Why `f60c9ad` showed no lean ( regression analysis )

`pid.cpp`, `pid.h`, `mixer.cpp`, `rc_controls.cpp` and `rx.cpp` are unchanged since `f60c9ad`. `config.cpp`
changed only `P8[PIDALT]` 100 → 128, `failsafe_delay` and the EEPROM version; `imu.cpp` only the altitude
estimator's Z deadband. The `rcData[THROTTLE]` writes in `annexCode` are the same. What changed is how long the
craft stays in the zero-integrator phases:

| Phase | `f60c9ad` | HEAD |
|---|---|---|
| Full-stick descent rate | `( rcData − 1500 ) / 4`, clamped to **−100 cm/s** at once | **−30 cm/s** ( `ALT_MAX_DESCENT_CMS`, [altitudehold.cpp:428](../../../../src/main/flight/altitudehold.cpp#L428) ), ramped |
| Baro datum while the stick is low | `mwArm ( )` runs every stick-low frame and re-zeroed the datum in flight ( `f60c9ad mw.cpp:582` ): the estimator saw 0 cm and drove down, a fast drop | guarded by `throttleRaisedSinceArm` ( [mw.cpp:604-611](../../../../src/main/mw.cpp#L604), `4657cbf` ) |
| Take-off | `AltHold` stepped by 120 cm: throttle ~1800 from the first tick, the craft leaves the floor at once | goal profile 0 → 60 cm/s at 100 cm/s² ( `2a8d59a` ): near-hover lift-off, ~2.7 s to 120 cm |
| Time with no trim ( est. ) | ~1 s, ~0.4 m drift | ~4 s, several metres |

Drift grows with the square of the time, so ~4× the exposure gives ~16× the drift. Both newer behaviours are wanted
( the datum re-zero was a bug that moved the datum by up to 255 cm ), so the fix is the reset, not a revert.

Ruled out:

- **Motor low-side clipping** ( `mixer.cpp:818` ): HEAD commands about 1400 on a full-stick descent and 1500+ on
  take-off; the old build was closer to `minthrottle`, not HEAD.
- **Integrator limit**: a saturated limit would also lean in steady hover.
- **Trim re-applied**: `accelerometerTrims` are static config.
- **Loop order / ground idle**: the same; `altHoldGroundIdle` cannot engage in flight.
- **Settings**: HEAD was on defaults + calibration; `f60c9ad` was recalibrated after flashing.

## 5. Hypotheses and the tests that settle them

| # | Hypothesis | Confirm / kill |
|---|---|---|
| H1 | Full-stick lean = stick-low reset + slow descent | Task 1: partial-stick descent ( raw 1100-1460 ) is straight, full-stick leans. If the partial descent also leans, H1 is dead and the plan becomes a bisect `4657cbf` → `2a8d59a` → later commits. **Confirmed 19 Oct 2026** ( [TESTING.md §1](TESTING.md) ): the lean appears only at or just above full stick down, the `mincheck` band |
| H2 | Take-off lean = arm reset + slow lift-off | Task 3: the I-term log is 0 at lift-off and the lean decays as it builds ( time constant ~0.5 s ). **Confirmed 19 Oct 2026** ( [TESTING.md §3](TESTING.md) ): `Ip` drops to 0 at every arm ( even after a hover at −105 ), the lean ( +4.8° / +8.7° ) starts on the floor as the motors spin up and fades as `Ip` reaches `Ih` ≈ −103 in ~1.7 s. H1 measured: ~10° standing nose-down lean with `Ip` held at 0 in the full-stick descent |

## 6. Fix options considered ( take-off )

| Option | Effect | Risk |
|---|---|---|
| **B** hover snapshot seed ( chosen ) | repeat flights start trimmed | stale after a payload change without power-off ( user: never happens ) |
| **C** faster wind-up at take-off ( chosen, with B ) | first flight after power-on levels faster | higher I gain may overshoot; winds faster on the floor if started too early |
| D raw carry-over | - | the integrator winds against the floor ( ±256 in ~9 s at a 2° rest tilt ); rejected |
| Raise I8 with `set_PIDProfile` | user-side only | not a firmware fix; rejected |
| Angle trim | - | hovers tilted once the integrator works; rejected |
