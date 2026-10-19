# Off-centre payload lean - Scout briefs

[README](README.md) · [TASKS](TASKS.md) · [SCOUT](SCOUT.md) · [INVESTIGATION](INVESTIGATION.md)

Three `pluto-scout` briefs from planning ( 19 Oct 2026 ), verbatim. Graph stale ( built `5e268237`, HEAD `a81c180`;
changes since then are laser code only ). Line numbers are HEAD `a81c180` unless an `f60c9ad` path is given.

---

## Brief 1: off-centre mass ( 9 g servo ) tilts and drifts the craft toward that side during take-off and stick throttle-down, not during throttle-up, hover or Command_Land

Subsystems: Core PID, RC input, Altitude hold, Commands. Pipeline docs: `fw-architecture-pipeline/subsystems/Core_PID_Controller.md`, `Altitude_Hold_Estimator.md`, `RC_Input_Pipeline.md`.

Key code:
- `src/main/mw.cpp:892-898` `processRx`: when `throttleStatus == THROTTLE_LOW`, it calls `pidResetErrorAngle/Gyro`. Nothing checks whether the craft is armed or airborne.
- `src/main/io/rc_controls.cpp:131-139` `calculateThrottleStatus`: LOW means `rcData[THROTTLE] < mincheck`. `mincheck` is 1100 (`config.cpp:472`).
- `src/main/flight/pid.cpp:735-896` `pidRewrite`, the active controller (`pidController = 1` at `config.cpp:176`, which is enum `PID_CONTROLLER_REWRITE`, `pid.h:47`).
- `src/main/flight/pid.cpp:831-836`: the rate I-term `errorGyroI`, limited to ±`GYRO_I_MAX` 256 (`pid.h:24`).
- `src/main/command/command.cpp:174-178` `takeOff()`: resets the PID, then `mwArm`.
- `src/main/API-Src/FC-Control-Command.cpp:184-190` `Command_Arm`: resets the PID, then arms.
- `src/main/command/command.cpp:231-280` `land()`: ramps `landThrottle` down with a floor of `LAND_THROTTLE_MIN` 1150.
- `src/main/flight/altitudehold.cpp:572-605`: in ALT_HOLD the stick sets the climb rate. Full down (deflection -500) gives -30 cm/s (`ALT_MAX_DESCENT_CMS`, line 428).

Mode: fix (check-style ranking included)

Current behaviour: In `pidRewrite`, ANGLE mode is a P-only angle loop (`desiredRate = errorAngle*P8[PIDLEVEL]>>4`, `pid.cpp:803`) feeding a rate PI loop. `errorAngleI` is never used in this controller. So the only state that can hold the steady torque from an off-centre mass is `errorGyroI`. That integrator is zeroed:
- on every arm;
- on every RX update while the raw stick `rcData[THROTTLE]` is below 1100, even while flying.

Because of `pid_at_min_throttle = 1` (`config.cpp:369`) and MOTOR_STOP being off, the motors keep flying in ALT_HOLD when the stick is low (`mixer.cpp:818-824`). The craft does not fall, so the reset goes unnoticed.

Analysis:
- **H1, throttle-down. Mechanism proven; the stick value is unconfirmed.** Chain: in ALT_HOLD the stick goes to the bottom 10 % (raw value below 1100). `calculateThrottleStatus` returns LOW (`rc_controls.cpp:135`), and `mw.cpp:895-898` zeroes `errorGyroI` on every frame. The CG torque is then carried only by the P-term, so the craft holds a tilt toward the heavy side and drifts. The descent itself works because alt-hold reads the stick as a rate. To kill or confirm: descend once with the stick at about 3/4 down and once fully down. If only full down tilts, H1 is confirmed. A log of raw throttle plus the I-term would prove it.
- **H2, take-off. Proven by design.** The I-term is 0 at lift-off. Every arm path zeroes it:
  - `takeOff()` at `command.cpp:176-177`;
  - `Command_Arm` at `FC-Control-Command.cpp:188-189`;
  - ARM-box arming, which requires THROTTLE_LOW (`rc_controls.cpp:169`) and so is reset through `mw.cpp:895`;
  - chuck-arm at `rc_controls.cpp:185-186`.

  After lift-off the I-term has to build again (I8 roll/pitch is 10, `config.cpp:179/182`, with `PIDweight` 100 because `dynThrPID = 0`, `config.cpp:352`). My estimate from `pid.cpp:803,819,831` is a time constant of about 0.5 s, so the craft tilts for about 1-1.5 s. Suspected size: P-only tilt of roughly X/0.78 decidegrees for a needed torque of X in `axisPID` units. Test: log the I-term and the angle from arm to 3 s after lift-off.
- **H3, I-term saturation. Unlikely.** If the 256 limit were too small for this mass, the tilt would also show in steady hover, and it does not. My rough estimate for 9 g at 6 cm is X ≈ 50-100.
- **H4, motor low-side clipping at `minthrottle` 1100 (`mixer.cpp:818`) during descent. Unlikely**, because hover throttle is far above 1100. A log of motor outputs would kill it.
- **H5, trim re-applied. Ruled out.** `accelerometerTrims` are static config (`config.cpp:532`, stick combo `rc_controls.cpp:310-324`), and no reset path touches them.

Why `Command_Land` behaves: `land()` never resets the I-term. `landThrottle` only goes into `rcData` after `processRx` (`mw.cpp:389-390, 1409-1410`), and its floor of 1150 is above 1100 anyway. The reset check sees the centred stick, so the I-term learned in hover is kept.

Candidate fixes:
1. **Skip the reset at `mw.cpp:895` when flying in alt hold.** Condition: `ARMED && FLIGHT_MODE(BARO_MODE) && !altHoldGroundIdle && !isLanding`. This needs a getter, because `altHoldGroundIdle` is a static at `altitudehold.cpp:454`. Files: `mw.cpp`, `altitudehold.{h,cpp}`. Throttle mode still resets on a low stick, and so does the ground idle. It also keeps the I-term through a Wi-Fi failsafe, where `rxfail` throttle is low (`rx.cpp:531,558`). Safety: no change on the ground. Rollback is one condition.
2. **Freeze instead of zero** (stop accumulating while the stick is low). Touches `pid.cpp` and `mw.cpp`. Lower windup risk than option 1, but more code.
3. **For the take-off:** carry the hover I-term from the last flight into the next arm, clamped and decaying (`pid.cpp`, plus the arm paths), or raise roll/pitch I8 with the existing `set_PIDProfile` API (`FC-Config.h:71`) with no firmware change. A stale seed after a payload change is a risk.
4. **Angle trim:** wrong once the I-term works (it hovers tilted by the trim). Not recommended.

Constraints / invariants that apply:
- Pilot throttle comes from `rcDataPilot`. The reset reads `rcData`, which equals `rcDataPilot` at `processRx` time (`rx.cpp:564`), but `rx.cpp:528` holds the old `rcData` on an invalid pulse (CLAUDE.md, rcData vs rcDataPilot).
- Landing must not go through the stick limits. Flips write `rcData = 2000` (`acrobats.cpp:121`), which is not low.
- Ground windup: while the ARM-box ground idle holds the motors at 1000 and the stick is between 1100 and 1500 (`altitudehold.cpp:608-610`), the integrator already runs on the ground. Any fix must keep the reset on the ground.
- `throttleRaisedSinceArm` (`mw.cpp:403-404`) becomes true right after a centred-stick arm, so it does not mean "airborne". Do not gate on it.

Resource conflicts: none.

Already answered by the code:
- The controller is pidRewrite, not luxfloat.
- `errorAngleI` is unused.
- `max_angle_inclination` is 200 (`config.cpp:458`).
- `alt_hold_deadband` is 40 (`config.cpp:364`).
- `minthrottle` is 1100 (`config.cpp:278`).
- No `BLACKBOX` on either target (`target.h:154/155`).
- `GPS` is not defined, so `pid.cpp:791` is the compiled branch.
- Common to PRIMUS_X2_v1 and PRIMUS_V5 (both define `BARO`, `MAG`, `ENABLE_ACROBAT`).

Questions for the user:
1. On throttle-down, was the stick pushed fully down? Does a partial descent also tilt?
2. Arming flow: app ARM then Take-off (stick low at arm), or Take-off alone?
3. Was ALT_HOLD (BARO) on for every phase, or did they also fly in throttle mode?
4. Were PIDs or the controller ever changed from the app (EEPROM may differ from the defaults)?
5. Fix scope: throttle-down only, or the take-off lag too (seed or raise I)?
6. Target being flown: PRIMUS_X2_v1 or PRIMUS_V5?

Doc drift:
- `Core_PID_Controller.md:18` names `pidController()` and `applyGyroData()`. Neither exists; the code uses the `pid_controller` pointer (`pid.cpp:81`) and `pidRewrite`.
- `Core_PID_Controller.md:27` cites `pidProfile->pid[axis].I` as the windup bound. The real bound is `GYRO_I_MAX` (`pid.cpp:835`).
- The doc does not mention the throttle-low or arm resets at all.

Prior work: `takeoff-land-command-fix` / `takeoff-api-ground-hover` (`5e26823`) changed the goal-setting and land guards, not the resets. The `mw.cpp:895` reset dates from `9592044` (Cleanflight origin). There is no regression, which fits "stable before the servo".

Suggested skills: pluto-rules, pluto-build, pluto-flighttest, pluto-reviewer agent, pluto-log-analyst.

Log fields: `PlutoPilot.cpp` is empty, so there is no log today. Usable now:
- `RcData_Get(RC_THROTTLE)` (`RC-Interface.h:63`; this returns `rcData`, after user overrides)
- `Estimate_Get` angle (`FC-Data.h:121`)

Must be added: a getter for `errorGyroI[ROLL/PITCH] >> 13` (or copy it into `debug[]`, `debug.h:23`), plus a counter of reset events.

Pipeline docs to update at the end:
- `Core_PID_Controller.md`: the real controller, the I-term bound and where the I-term is reset.
- `Altitude_Hold_Estimator.md`: the stick-low behaviour in ALT_HOLD.
- `RC_Input_Pipeline.md`: who uses THROTTLE_LOW.
- `FLIGHT_INVARIANTS.md`: a new I-term invariant, if the fix lands.

---

## Brief 2 ( follow-up ): which take-off fix to choose, and the conditions for the throttle-down fix ( PRIMUS_X2_v1, ALT_HOLD on )

**A. Take-off timeline**
- **Path:** the app sends `MSP_SET_COMMAND` → `current_command = TAKE_OFF` (`serial_msp.cpp:1568-1574`) → `executeCommand` (`mw.cpp:1509`) → `takeOff()`.
- **Arming:** if disarmed, the ARM box and a centred stick are needed. It then resets the PID (`command.cpp:176-177`), calls `mwArm`, and `setTakeOffGoal` raises `AltHold` by `takeOffHeight` 120 cm (`command.cpp:120-130`, `mw.cpp:573`).
- **Climb throttle:** `rcCommand[THROTTLE] = initialThrottleHold + altHoldThrottleAdjustment` (`altitudehold.cpp:605`).
  - `initialThrottleHold` is 1500 when BARO mode is entered (`:648`). After that, `offloadHoverTrim` moves it toward the learned hover throttle (`:515-545`).
  - The velocity P-term is `120*err/32` (`:728`). The goal profile ramps up at 100 cm/s² to 60 cm/s (`:429,433`).
- **Ground phase:** the throttle starts near hover at once, with no idle stage. Its length is not measured, but it is short.
- **Integrator:** it is 0 at lift-off. No reset fires during the climb: the stick is centred (above 1100), and `landThrottle` is never involved.
- **Airborne signal:** none exists that means "has left the ground".
  - `isTookOff` and `isTakeOffHeightSet` mean the take-off was commanded.
  - `throttleRaisedSinceArm` means the stick is above `mincheck`.
  - The local `airborne` at `altitudehold.cpp:1158` means armed, not in ground idle and not landing.
  - `Altitude_Hold_Estimator.md:519` states "no general landed detector".
  - A precedent exists: height below 20 cm counts as on the ground (`ALT_FLIP_RETURN_MIN_CM`, `:480,695`).

**B. Keeping the hover trim across flights (snapshot)**
- **Where to take it:** the settled-hover test already in `offloadHoverTrim`: armed, not in ground idle, `setVelocity==0`, `altRate==0`, no goal active, `|VelocityZ|<10` (`altitudehold.cpp:519-523`). Add `flipState==0` and `EstAlt ≥ 20 cm`.
  - Landing excludes itself, because `setVelocity` is negative from `land()` onward (`:588-590`, `isLanding` set at `command.cpp:260`). So no touchdown windup reaches the snapshot.
- **Where to store it:** a low-pass value of `errorGyroI[ROLL/PITCH]>>13`, kept as `static` in `pid.cpp` (8 B RAM, not EEPROM).
- **Where to apply it:** make `pidResetErrorGyro` (`pid.cpp:92`) load the seed into roll/pitch instead of 0. Yaw stays 0. Every arm and ground-idle path then gets the seed without separate edits.
- **Clamp:** ±128, half of `GYRO_I_MAX`.
- **Disarm cases:**
  - Crash (`failsafe.cpp:380-381`): clear the seed.
  - Failsafe landing (`:306`) and ARM switch off mid-air (`rc_controls.cpp:195-197`): keep the snapshot, which was taken in hover.
- **Yaw:** no concern. A CG offset adds no yaw torque, and heading hold owns yaw.

**C. Integrating faster at take-off**
- **Rate:** each loop adds `((RateError*cycleTime)>>11)*I8*PIDweight/100` (`pid.cpp:831`). That makes ITerm grow at 0.596×RateError per second.
  - With the angle loop at ×2.5 (`:803`) and the P-term at 0.3125×RateError (`:819`), the estimated time constant is about 0.5 s.
- **Change:** a k× factor on roll/pitch at `:831` for N s after lift-off is about 5 lines.
- **Reuse:** none is clean.
  - `PIDweight` (uint8, `mw.cpp:377`) scales P, I and D together and only reduces them.
  - `dynI8` (`mw.cpp:369`) is not used by `pidRewrite`.
  - `acrobats.cpp` has no gain handling.
- **No-firmware alternative:** raise I8 in `plutoInit` with `set_PIDProfile` (`FC-Config-PID.cpp:125-139`). It applies to the whole flight.

**D. Carrying the integrator over raw (no snapshot): dangerous**
- `pid_controller` runs every loop even when disarmed (`mw.cpp:1476`, not gated).
- A craft at rest 2° off its trim winds 0.596×50 ≈ 30 units/s and reaches the 256 clamp in about 8.6 s.
- Touchdown at a 5° tilt adds about 75 units/s, so 22-75 units over the 0.3-1 s settle (`command.cpp:234-235`).
- 256 is several times the torque the servo needs.

**E. Throttle-down condition**
- `altHoldGroundIdle` (`altitudehold.cpp:454`, static) is set while `isThrottleStickArmed` (ARM-box arming with the stick low, `rc_controls.cpp:175`) and `rcData ≤ 1500`; the motors are held at 1000 (`:608-610`). `setTakeOffGoal` releases it (`:1472-1475`). It needs a getter.
- `isLanding` is visible in `mw.cpp` (`command.h:44`).
- **Recommended condition:** skip the reset at `mw.cpp:895` when `ARMED && FLIGHT_MODE(BARO_MODE) && !altHoldGroundIdleGet()`. That covers this fix.
  - Gap: after `Command_Arm` with the stick low, the integrator winds on the floor. The seed clamp and the ±256 clamp bound it.
- **Lift-off latch:** set when `EstAlt` is 20 cm or more above its value at arm, cleared in `mwDisarm` (`mw.cpp:546`). It is better for gating the B snapshot than for E.

**F. Diagnostic log**
- `debug[]` is taken: `accgyro_mpu.cpp:143` and `altitudehold.cpp:871-873`.
- Add `int16_t pidGetGyroITerm(uint8_t axis)` and a `uint16_t` reset counter to `flight/pid.h`. That is an internal header that already has `extern "C"` (`:20-21`), so the public API does not change.
- Read them from `PlutoPilot.cpp` with `extern "C"`, as `takeoff-land-command-fix/test/PlutoPilot_fix_validation.cpp.txt:37-46` does.
- Short tags in that file's style (`" Ir:"`, `" Ip:"`, `" Rs:"`, `" Tp:"` from `rcDataPilot`), 7-8 fields per line, well under 130 B.

**Recommendation: B (reset to the hover snapshot) together with E.** It removes the lean on repeat flights, leaves the tuning alone, is bounded by the clamp, and follows `initialThrottleHold`, which already carries the hover throttle across flights in a power cycle. Its gap is the first take-off after power-up. If the log shows that is still unacceptable, add C (k=2, about 1.5 s).

Risks:
- **B:** a stale seed after the payload is removed without a power cycle gives the same-sized lean the other way, and it decays in about 0.5 s.
- **C:** a higher I-gain can overshoot or oscillate in the rate loop, and winds faster on the floor.
- **D:** carries floor windup of up to 256.
- **E:** an armed craft on the floor with the stick low in BARO mode winds the integrator.

Questions for the user:
1. Is the first take-off after power-up acceptable with a small lean (B alone), or must it be covered too (B+C)?
2. Is the servo ever removed or moved without power-cycling the drone?

---

## Brief 3 ( follow-up 2 ): lean that appears in f60c9ad..HEAD but not in f60c9ad

**Short answer.** The integrator-zero periods are the same in both revisions. What changed is how long the craft spends in them. Two commits made the take-off and the full-stick descent slow, gentle phases that last several seconds instead of about one second:
- **2a8d59a**: the setpoint shaping.
- **4657cbf**: removed the in-flight baro datum re-zero, which used to turn a full-stick descent into a fast drop.

Hover, throttle-up and `Command_Land` keep the integrator, or still move fast, which matches what the user saw. I found no HEAD-only code that writes `axisPID`, `PIDweight`, or the motors.

**Ranked mechanisms**

**1. Full-stick descent got about 3× slower, and its fast-drop side effect is gone. Likely; numbers from code.**

| | f60c9ad | HEAD |
|---|---|---|
| Stick 1000 commands | `setVelocity = (1000-1500)/4 = -125`, clamped to **-100 cm/s** at once (`altitudehold.cpp:231-233`) | **-30 cm/s** (`ALT_MAX_DESCENT_CMS`, `altitudehold.cpp:428`), ramped at 100 cm/s² and slewed at 150 cm/s² (`:597-600, :785, :816`) |
| Baro datum while stick is below 1100 | `rc_controls.cpp:169-180` calls `mwArm()` on every RX frame, and while armed it ran `baroResetGroundLevel()` without a guard (`git show f60c9ad:src/main/mw.cpp`, mwArm ARMED branch, about line 582). The baro then read about 0 every frame, so the estimator saw no descent and kept pushing down: the craft fell fast. | Guarded by `throttleRaisedSinceArm` (`mw.cpp:604-611`) |

- In both revisions the gyro integrator is held at 0 for the whole stick-down period (`mw.cpp:895-898`).
- A P-only lean of θ accelerates the craft sideways at g·tanθ: about 0.5 m/s² at 3°. Over the old ~1.2 s descent that is about 0.4 m of drift. Over HEAD's ~4 s from 1.2 m it is about 4 m (drag ignored). That explains "goes forward on throttle-down".
- `Command_Land` did not change: about -50 to -87 cm/s via `isLanding` (`althold-setpoint-shaping/README.md:39`), and no reset because its floor is 1150. So it stays clean in both.

**2. Take-off changed from a hard throttle burst to a gentle goal profile. Likely, but less certain than #1.**
- **Old:** `setRelativeAltitude` stepped `AltHold = EstAlt + 120` (`f60c9ad altitudehold.cpp:720-722`).
  - Position P with its 5 cm deadband gives 100·115/128 ≈ 90 cm/s (`:315-319`).
  - Velocity P is 120·90/32 = 337, clamped to 300 (`:326`), so the throttle is about 1800 from the first tick.
  - The craft leaves the floor almost at once while the integrator builds (time constant about 0.5 s), and higher thrust also gives more control authority per count.
- **HEAD:** the goal profile ramps 0→60 cm/s at 100 cm/s² (`:429, :433, :773-818`).
  - The throttle starts at about `initialThrottleHold` (1500 on BARO entry, `:648`) plus a small P-term.
  - So the lift-off is slow, near hover thrust, close to the floor, and starts with the integrator at 0. Climbing 120 cm takes about 2.7 s, against the old burst followed by a crawl.
  - The ground-phase length is not measured; a log would show it.

**3. Motor low-side clipping (`mixer.cpp:818`). Ruled out as the cause; it points the wrong way.**
- Old full-stick: the first error is -100, so P is -375, clamped to -300, giving about 1200. The light side then sits at about 1100-1150 and clips in the **old** build.
- HEAD: P is 3.75×(-30) ≈ -112, giving about 1400, so no clipping.
- Take-off: old ≈ 1800 ± X is fine; HEAD ≈ 1500+ is fine.

**4. Ordering, arming, ground idle. No effect.**
- The loop order is the same. The only addition is the `isLanding` pin after `userCode` (`mw.cpp:1407-1411`), which does not affect pilot descents.
- App take-off: old `command.cpp:110-112` and HEAD `:176-178` both reset the PID and call `mwArm`. HEAD also needs a centred stick (5e26823), which the app already sends.
- `altHoldGroundIdle` (`:608-610`, same logic at old `:248-249`) needs `isThrottleStickArmed`. That is set only by ARM-box arming while disarmed (`rc_controls.cpp:174-176`) and cleared when the stick goes above 1500 or by `setTakeOffGoal`. It cannot fire in flight.

**5. Settings reset between the two flights. Low.**
- The EEPROM version is 106 at f60c9ad, 107 from **4657cbf**, and 108 from **84c326d**. Each crossing (including flashing back to f60c9ad) resets all settings and the accelerometer calibration.
- A wrong level reference would also drift in hover, and the user saw none there.

**Commits per mechanism**
- Descent rate, take-off goal profile, leash: **2a8d59a** (introduces `ALT_MAX_DESCENT_CMS`, `ALT_CMD_MAX_CLIMB_CMS`, `ALT_TARGET_LEASH_CM`).
- Datum re-zero guard, `offloadHoverTrim`, `P8[PIDALT]` 128, EEPROM 107: **4657cbf**.
- Ground idle: 2a8d59a, c569c0f, 484226c, 7cf2444, 5e26823.
- `altHoldStickCentred` and the take-off goal when already armed: 5e26823.

**One flight on HEAD can settle it without bisecting**
- **Descent:** descend with the stick about 4/5 down (raw 1150-1200, above `mincheck`, so no reset; about 20 cm/s), then with the stick fully down.
  - Lean only on full stick: reset plus the slow descent is confirmed.
  - Lean also on the partial stick: something else, so bisect.
- **Take-off:** log the gyro I-term roll/pitch, raw throttle and `EstAlt` from arm (getter from follow-up 1, F).

**Bisect order if needed (fewest flights)**
1. Flash **4657cbf**, re-calibrate the accelerometer (EEPROM 107 resets the settings), and fly the app take-off plus a full-stick descent.
   - Lean: the cause is 4657cbf (datum guard or hover-trim offload). Stop.
2. If clean, flash **2a8d59a** (EEPROM 107, no reset).
   - Lean: the cause is 2a8d59a. Stop.
3. If still clean, binary search c569c0f → 386599b → 84c326d (EEPROM 108, re-calibrate) → 5e26823. That is at most 2-3 more flights.

**Effect on the fix**
- Nothing to revert: the fast drop was a bug (it re-zeroed the datum by up to +255 cm, per the 4657cbf notes), and the slow profiles are wanted.
- The fix from follow-up 1 still applies:
  - Skip the `mw.cpp:895` reset while armed and flying in BARO mode.
  - Start roll/pitch from a hover snapshot instead of 0.
- Speeding the profiles back up would only shorten the exposure, not remove it.
- If the user objects to the first-flight lean, a faster take-off start (a throttle kick or a higher climb rate below about 20 cm) would be the other way to cover it.

Questions for the user:
1. Did the f60c9ad flight use the same app take-off and a full-stick descent from a similar height?
2. On f60c9ad, did the full-stick descent come down fast or hit the floor hard?
3. Was the accelerometer re-calibrated after each flash?
