# Active Development

Working documentation for firmware changes that are **in progress**: what is
being changed, why, how, and the evidence that it works. One folder per topic.

This is the counterpart to [`../fw-architecture-pipeline/`](../fw-architecture-pipeline/),
which describes the firmware **as committed**. Keeping the two apart means the
pipeline docs never describe behaviour that is still being tested, and the
reasoning behind a change is kept even after it ships.

## Rules

1. **Start a topic folder** here when a piece of work needs more than a commit
   message to explain: an investigation, a multi-step fix, anything tested on
   hardware.
2. **Keep it current while the work is live.** Record changes, measurements and
   dead ends as they happen, not at the end.
3. **Do not touch `fw-architecture-pipeline/` during the work.** If a pipeline
   doc will need changing, write the new version into the topic's
   `PIPELINE_UPDATE.md` instead.
4. **At commit, once the change is confirmed correct:**
   - copy `PIPELINE_UPDATE.md` over the pipeline doc it targets;
   - update `CHANGELOG.md`, and any `docs/API/` wiki if public API changed;
   - set the topic's status to **Closed** here and in its README, with the commit hash.
5. **Closed topics stay.** They are the record of why the code is the way it is.
   Do not delete them; mark anything superseded.

## Topic folder layout

| File | Contents |
|---|---|
| `README.md` | Status, branch, target, one-paragraph summary, gotchas, open items, next actions. Start here. |
| `TASKS.md` | Resume block, numbered task list ( serial number, title, description, depends on, safety impact, done when, rollback, status ) and dated decisions log. Written by `/pluto-grill` after the planning interview; worked one task per turn with `/pluto-task`. |
| `SCOUT.md` | The `pluto-scout` brief from planning, kept so no later session repeats the reconnaissance. |
| `DESIGN.md` | `feature` topics only: the design options, the one chosen and why. |
| `INVESTIGATION.md` | **Why**: the problem, evidence, root causes, decisions. Readable without the code. |
| `CHANGES.md` | **What and how**: each code change with file and line links, and a file index. |
| `TESTING.md` | **Evidence**: test method, measurements, results, how to reproduce. |
| `PIPELINE_UPDATE.md` | Staged text for the pipeline doc(s), applied at commit. Only if a pipeline doc changes. |

Links to source from a topic folder use `../../../../src/main/...`.

## Topics

| Topic | Status | Branch | Summary |
|---|---|---|---|
| [takeoff-land-command-fix](takeoff-land-command-fix/README.md) | **Closed - pending commit** ( FW 3.14.0, API 1.7.0, 16 Oct 2026 ) | `BugFix-June26` | Fix of takeoff-api-ground-hover: `Command_TakeOff` sets the goal when already armed ( 3 s window or ground idle ); take-off and flip are refused during a landing ( D1 ); touchdown is confirmed only after 1 s of ramp ( D2 ); FC-Control wiki, API 1.7.0. |
| [takeoff-api-ground-hover](takeoff-api-ground-hover/README.md) | **Closed - pending commit** ( 16 Oct 2026, `check` ), with takeoff-land-command-fix | `BugFix-June26` | `Command_Arm ( )` then `Command_TakeOff ( 100 )` skims the floor. Flight-proven cause: the armed branch of `takeOff ( )` sets no height ( `command.cpp:97-103` ). Also found D1 ( take-off / flip during a landing locks the stick with the motors running ), D2, D3 and D4. |
| [relative-yaw-api](relative-yaw-api/README.md) | **Closed** in `794002d` ( FW 3.13.0, API 1.6.0, 15 Oct 2026 ) | `BugFix-June26` | `DesiredAngle_setRelative ( AG_YAW, turn )` ( new ): a turn relative to the nose. A user yaw target did nothing unless the app's heading-hold switch was on ( `updateMagHold` wiped it, log-1 ). Heading hold is now engaged automatically while user code holds a target. The target survives a Wi-Fi flicker and is dropped on a dev switch-off ( the heading is held ) or a disarm. A pilot yaw re-bases a relative target. `setDesiredAngle` yaw wraps into 0..359 ( 900 used to spin the drone ). Flight-tested in part ( cases 1-3, the disarm clear ); the hand-yaw, mid-turn switch-off, flicker and HEADFREE cases are untested. Follow-up: `MSP_SET_HEAD` wrap. |
| [tof-single-pipeline](tof-single-pipeline/README.md) | **Closed** in `edfbd61` ( FW 3.12.0, API 1.5.0, 15 Oct 2026 ) | `BugFix-June26` | Fix for laser-tof-init-conflict: one VL53L0X pipeline behind `XRanging`. `XRanging.initAlt ( gpio )` / `initAlt ( )` replaces `LASER_TOF` + `LASER_ALT` ( boot-latched ); one bring-up at main.cpp:600; per-laser state; paced, cached side-laser reads; `DOWN` id and `getStatus ( )`. Down-laser XSHUT on GPIO_5 ( FRONT's LED is no longer on that pin ). |
| [laser-tof-init-conflict](laser-tof-init-conflict/README.md) | **Closed** in `edfbd61` ( `check`, 14 Oct 2026; fixed by `tof-single-pipeline`, committed with it ) | `BugFix-June26` | With `LASER_TOF` + `LASER_ALT` and `initObjectAvoidance ( ... )` / `XRanging.init ( )`, no laser works ( down + 4 side VL53L0X ). The two inits are separate: the down-laser has no XSHUT and stays at 0x29 while XRanging re-addresses chips through 0x29, and all lasers share one `Global_Status` latch. Analysis of both pipelines and 2-3 single-pipeline designs. |
| [oled-flight-wobble](oled-flight-wobble/README.md) | **Closed** in `c495d6a` ( FW 3.11.3, API 1.4.1, 14 Oct 2026 ) | `BugFix-June26` | With an OLED and live telemetry the drone twitched at 5 Hz: ~630 blocking I2C transactions every 200 ms on the polled bus the IMU shares. Fix ( written and flown in a separate copy, merged here ): a 1 KB RAM framebuffer and at most 16 changed bytes per loop; `Oled_Update` is deferred; the OLED switches off if the panel stops answering. RAM +2 KB. Confirmation hover passed on the merged build. |
| [user-loop-timer-wrap](user-loop-timer-wrap/README.md) | **Planned** ( 14 Oct 2026 ) | - | `plutoLoop ( )` stalls if Developer Mode first comes on after ~35.8 min of uptime ( `userLoopTime` starts at 0, signed 32-bit compare ). |
| [rx-signal-loss-tolerance](rx-signal-loss-tolerance/README.md) | **Closed** in `84c326d` ( FW 3.11.2, 14 Oct 2026 ) | `BugFix-June26` | Wi-Fi RC stalls of 1-2 s triggered `Signal_loss` and `LAND` because `failsafe_delay` was never applied ( `LAND` ~1.0 s after the last frame ). Now applied: default 15, cap 22, so `LAND` comes ~2.5 s after the last frame. `EEPROM_CONF_VERSION` 108 resets all settings ( recalibrate the accelerometer and compass ). Bench and flights passed ( user's report ). |
| [crash-detect-false-trigger](crash-detect-false-trigger/README.md) | **Closed** in `386599b` ( FW 3.11.1, 13 Oct 2026 ) | `BugFix-June26` | Crash failsafe disarmed on light bumps and landings: one 3.5 ms sample of lateral accel over 0.305 / 0.427 g, about 10× tighter since `49a2fdb`. Logs show survivable bumps and contacts that flip the craft both peak at 1-1.3 g raw; the flips are caught by the 70° rule. Final limits **5000 / 8000 LSB ( 1.22 / 1.95 g )**: no false trip in log-2..4 or in the validation flights, and a hard crash is 3× over. |
| [crash-stuck-detection](crash-stuck-detection/README.md) | **Planned** ( 13 Oct 2026 ) | - | Crash cases no accel limit or 70° rule can see: pinned upright against a wall, ceiling hit or upright drop ( Z not checked ), caught in a curtain or hand. Needs another signal ( motor saturation, attitude-error integral, Z term clear of `LAND` ). |
| [battery-soc-fix](battery-soc-fix/README.md) | **Closed** in `cbd1c72` ( FW 3.11.0, API 1.4.0, 3 Oct 2026; partial commit `369ea71` ) | `BugFix-June26` | Pluto Fuel Gauge: coulomb counting from a LiPo-curve plug-in estimate, per-flight pack resistance, compensated-voltage floor; low battery at 3.745 V/cell or 15%, critical at 3.60 V or 5%, latched; no wrap at empty; auto-gain removed. Low-Battery Auto-Land at a confirmed critical, arming refused until the pack is changed; the app is told low battery while armed. `Bms_Get` SoC / Warning_Level / Resistance. Validated on 4 packs ( log-3..8 ): warning with 13-20% really left, critical ~5-8%, 4 auto-landings. |
| [battery-capacity-estimate](battery-capacity-estimate/README.md) | **Closed** ( 26 Sep 2026, superseded by battery-soc-fix, committed with it ) | `BugFix-June26` | `check`: ~300 mAh shown on an empty 600 mAh pack. Cause: a stacked second R020 ( 10 mOhm read as 20 ) on the test board, plus the plug-in estimate. With one R020 ( production ) the count matched the charger within 1%. Found: empty-pack SoC jump to 54-58%, warning ~15 s before empty, auto-gain −5%. |
| [vl53l1x-extended-reach](vl53l1x-extended-reach/README.md) | **Planned** ( 24 Sep 2026 ) | - | VL53L1X handover at 270 / 240 cm. Not possible by changing only the band: above ~2 m every sample is `St` 2 ( signal fail ). Likely lever: the preset's 1.5 MCPS minimum signal rate; bench and reach flights in a hall with a ceiling of at least 3.5 m. |
| [vl53l1x-error-latch-recovery](vl53l1x-error-latch-recovery/README.md) | **Planned** ( 24 Sep 2026 ) | - | A VL53L1X I2C error latches for good ( laser out, baro hold until a power cycle ). Guarded, non-blocking re-init; never seen in any log so far. |
| [vl53l1x-althold-parity](vl53l1x-althold-parity/README.md) | **Closed** ( FW 3.10.0, 24 Sep 2026, `7cf2444` ) | `BugFix-June26` | The VL53L1X ( `LASER_TOF_L1x` ) runs the flight-validated VL53L0X laser fusion ( 160 / 140 handover, frame shift, object hold-off ) through a shared, accessor-based body ( L0X object code byte-identical ). Driver: one sample per measurement, 45 / 50 ms Medium, 0.79 ms poll, 15 mm minimum. New L1x return guard against objects seen from above ( fan blades ). Flight-validated on PRIMUS_X2_v1 ( log-10..13 ). Next topic: 270 / 240 band. |
| [tof-althold-fusion](tof-althold-fusion/README.md) | **Closed** in `484226c` ( FW 3.9.0, 22 Sep 2026 ) | `BugFix-June26` | VL53L0X altitude hold bobs and reacts hard to objects underneath: hard 200 cm source switch, no hysteresis, no step rejection, tau flips per source. Fixed: accelerometer Z deadband ( the bob ), one tau, 160/140 cm handover with frozen offset and frame shift, 2.5 s object hold-off with a window test; flight-validated on PRIMUS_X2_v1. |
| [flip-althold-regression](flip-althold-regression/README.md) | **Closed** in `c569c0f` ( FW 3.8.1, 21 Sep 2026 ) | `BugFix-June26` | App back-flip stopped rotating after `2a8d59a`: ALT_HOLD stays on through AUX3 and the shaped 40 cm/s stick cap stops ASCEND reaching its 100 cm/s threshold. Fix: the flip bypasses setpoint shaping, with a clean handover on exit. |
| [althold-setpoint-shaping](althold-setpoint-shaping/README.md) | **Closed** in `2a8d59a` ( FW 3.8.0, 18 Sep 2026 ) | `BugFix-June26` | Throttle stick moves the altitude setpoint ( ArduPilot / DJI style ) instead of switching to raw velocity control; commanded altitudes flown on a trapezoidal goal profile; landing keeps its own descent rate; controller held in reset while armed at idle. |
| [altitude-hold](altitude-hold/README.md) | **Closed** in `4657cbf` ( FW 3.7.0, 18 Sep 2026 ) | `BugFix-June26` | Barometer altitude hold sank in flight. Four firmware bugs fixed, throttle and temperature compensation added, validated on one PRIMUS_V5 with a laser. |
