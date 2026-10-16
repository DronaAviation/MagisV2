# Take-off / land command fix - Changes

[README](README.md) · [TASKS](TASKS.md) · [SCOUT](SCOUT.md) · [INVESTIGATION](INVESTIGATION.md) · [CHANGES](CHANGES.md)

## Task 1: D1, take-off / flip refused during a landing

| File | Change | Why |
|---|---|---|
| [FC-Control-Command.cpp](../../../../src/main/API-Src/FC-Control-Command.cpp) | New `static bool landingInProgress ( )`: `isLanding \|\| ( current_command == LAND && command_status != FINISHED )` | One test for "a landing owns `current_command`". The second term covers the cycle before `land ( )` first sets `isLanding`. The status test mirrors `land ( )`'s own early return ( `command.cpp:178` ), because `current_command` stays LAND after a landing ( `updateCommandStatus ( )` has no callers ) |
| same | `Command_TakeOff` and `Command_Flip` return at once while `landingInProgress ( )` | D1: replacing LAND stopped `land ( )` but left `isLanding` set, with a pinned throttle and no disarm after touchdown |
| same | Banner `Last Modified` 16 Oct 2026 | style |

The review confirmed that every LAND writer sets RUNNING ( `Command_Land`, `failsafe.cpp:249-250`, `mw.cpp:1273-1274`,
`serial_msp.cpp:1570-1571` ). Every disarm goes through `mwDisarm ( )`, which sets FINISHED and clears `isLanding`
( `mw.cpp:550-553` ), so a take-off is not locked out after a landing or a kill-switch disarm.

## Task 2: take-off goal on an already-armed drone

| File | Change | Why |
|---|---|---|
| [command.cpp](../../../../src/main/command/command.cpp) | New `TAKEOFF_ARMED_WINDOW_MS` 3000 and `static bool takeOffFromFloor ( )`: `( millis ( ) - arm_time ) < 3000 \|\| isThrottleStickArmed` | The user's definition of "still on the floor" for a drone armed before the take-off ran. The unsigned subtraction is wrap-safe; `arm_time` is set on the arming edge ( `mw.cpp:622` ) and zeroed at disarm |
| same | New `static void setTakeOffGoal ( )`: returns unless `altHoldStickCentred ( )`; releases the ground idle; laser `setDesiredPosition` or baro `DesiredPosition_setRelative`; sets `isTakeOffHeightSet` | One goal path for both branches. A stick rate cancels a goal at once ( `altitudehold.cpp:741-743` ), so the once-per-arm flag is spent only on a goal that will be flown ( review finding 1 ) |
| same | Armed branch of `takeOff ( )`: `if ( ! isTakeOffHeightSet && takeOffFromFloor ( ) ) setTakeOffGoal ( );` before finishing | **The reported fault:** `Command_Arm` + `Command_TakeOff` set no height |
| same | Disarmed branch: goal only `if ( ARMING_FLAG ( ARMED ) && ! isTakeOffHeightSet )` after `mwArm ( )` | Existing defect: a refused `mwArm ( )` ( calibration, battery ) still set the goal and the flag. The goal was wiped while disarmed, and the later arm got no goal |
| [altitudehold.cpp](../../../../src/main/flight/altitudehold.cpp) / [.h](../../../../src/main/flight/altitudehold.h) | New `altHoldReleaseGroundIdle ( )`: clears `isThrottleStickArmed` and the static `altHoldGroundIdle` | The controller runs at the top of the loop ( `mw.cpp:1331` ), before `applyAltHold ( )` recomputes the idle. Clearing only `isThrottleStickArmed` would leave one cycle whose reset ( `:688` ) wipes the goal |
| same | New `altHoldStickCentred ( )`: `ABS ( rcData [ THROTTLE ] - 1500 ) <= alt_hold_deadband` | The same test as the stick-rate branch of `applyMultirotorAltHold ( )` |
| altitudehold.cpp | Banner `Last Modified` 16 Oct 2026 | style |

## Task 3: D2, touchdown confirmed only after 1 s of ramp

| File | Change | Why |
|---|---|---|
| [command.cpp](../../../../src/main/command/command.cpp) | New `LAND_CONFIRM_MIN_MS` 1000; in `land ( )`, `conclusive = landThrottle <= LAND_CONFIRM_THROTTLE && elapsed >= LAND_CONFIRM_MIN_MS` | D2: `Command_Land ( 105 )` starts at 1200 = `LAND_CONFIRM_THROTTLE`, so 300 ms of "arrest" before the descent built up disarmed the drone in the air. The earliest settle disarm is now 1.3 s; the descent passes −8 cm/s at about 0.4 s ( log-2 ). The impact check and the 30 s timeout are not gated |
| same | Landing header comment: the "throttle is far above LAND_CONFIRM_THROTTLE at entry" assumption now names the `Command_Land` exception | The comment stated the very assumption D2 broke |

Effect by `landSpeed`: 65 or more ( start at 1240 or below ) changes; below 65, and the failsafe and battery landings
( 1300 ), reach 1200 after 1 s anyway, so they are unchanged.

## Task 11: a queued take-off cannot arm the drone later

| File | Change | Why |
|---|---|---|
| [command.cpp](../../../../src/main/command/command.cpp) | New `endTakeOff ( )`, used for every end of a take-off: NONE, FINISHED, clears `takeOffFromUser` and the refusal timer | One place to finish a take-off, so nothing carries over to the next one |
| same | Disarmed branch of `takeOff ( )`: with the ARM switch on and `mwArm ( )` refusing, start a timer; after `TAKEOFF_REFUSED_DROP_MS` ( 2000 ) call `endTakeOff ( )`. ARM off resets the timer: the take-off waits | Decision 1. After a landing `OK_TO_ARM` stays off until ARM goes off, and the old retry armed the drone at the next ARM-on. It applies to the app's take-off too |
| same | `dropUserTakeOff ( )`: drops a queued TAKE_OFF that `Command_TakeOff` queued ( `takeOffFromUser` ) | Decision 2 |
| same | `executeCommand ( )` clears the refusal timer while no take-off is queued | Ends that skip `endTakeOff ( )` ( disarm, an app command, a flip ) could leave it running, and the next refused take-off would drop at once ( review N2 ) |
| [command.h](../../../../src/main/command/command.h) | `extern bool takeOffFromUser;`, `void dropUserTakeOff ( void );` | |
| [FC-Control-Command.cpp](../../../../src/main/API-Src/FC-Control-Command.cpp) | `Command_TakeOff` sets `takeOffFromUser = true`; HISTORY row | Marks a user-code take-off |
| [serial_msp.cpp](../../../../src/main/io/serial_msp.cpp) | `MSP_SET_COMMAND` sets `takeOffFromUser = false`; banner date and HISTORY row | The app's take-off is not dropped with Dev Mode |
| [mw.cpp](../../../../src/main/mw.cpp) | `userCode ( )`: `if ( ! devSwitchOn ) dropUserTakeOff ( );` after the disarm edge; banner date and HISTORY row | A real Dev Mode switch-off. `devSwitchOn` reads the AUX value, which holds through a Wi-Fi flicker and an RX loss ( rxfail HOLD ) |

Behaviour, from the review:

- **Correction ( task 7 doc check ):** with the battery OK, `BMS_Update ( )` re-enables `OK_TO_ARM` every 21 ms
  ( `battery.cpp:794-795` ), so arming is **not** refused after a landing. A take-off re-sent with ARM on re-arms and
  climbs at once. The 2 s drop fires only on a real refusal ( calibrating, low battery ). The user chose to document
  this rather than block it ( wiki: "After a landing the drone can take off again at once" ).
- A take-off queued with ARM off waits with no limit while Dev Mode is on. It arms and climbs at the next ARM-on.
  This is case B's path, accepted under decision 2 ( N1 ).
- A calibration refusal does not succeed on a retry: calibration clears `OK_TO_ARM`, and only ARM off brings it
  back. So in practice the 2 s rule drops such a take-off. That is safe: the old code could not arm there either.

## Task 7: API docs and version

| File | Change | Why |
|---|---|---|
| [FC-Control.h](../../../../src/main/API/FC-Control.h) | Comments for `Command_TakeOff`, `Command_Land`, `Command_Flip` and `Command_Arm` describe the 1.7.0 rules. Height in cm, 100-250 ( was "height in units" ). Banner date and HISTORY row | D4: the header did not say the take-off arms the drone, or when it is ignored |
| [docs/API/FC_CONTROL_API_WIKI.md](../../../../docs/API/FC_CONTROL_API_WIKI.md) | New wiki: Quick Start ( one flight per ARM switch-on, from `plutoLoop ( )` ), take-off rules, drop table, landing rates, examples, pitfalls, changes in 1.7.0 | There was no FC-Control wiki |
| [Makefile](../../../../Makefile) | `API_Version` 1.6.0 → 1.7.0 | Public API behaviour changed ( tasks 1, 2, 3, 11 ). `FW_Version` is bumped by `pluto-commit` |

## Task 12: `Command_Land` ignored during a flip

| File | Change | Why |
|---|---|---|
| [FC-Control-Command.cpp](../../../../src/main/API-Src/FC-Control-Command.cpp) | `Command_Land` returns while `flipState != 0` ( `#ifdef ENABLE_ACROBAT` ); includes `flight/acrobats.h`; HISTORY row | B_FLIP marks itself FINISHED when the flip starts, so a landing could take the throttle over mid-flip, and `land ( )`'s impact check has no flip exemption. The same test as the battery auto-land ( `mw.cpp:1274-1277` ) |
| [FC-Control.h](../../../../src/main/API/FC-Control.h) | `Command_Land` comment: ignored while a flip is in progress | doc sync |

## Task 13: `Command_TakeOff` arms only with the stick centred

| File | Change | Why |
|---|---|---|
| [command.cpp](../../../../src/main/command/command.cpp) | Disarmed branch of `takeOff ( )`: arms only `if ( IS_RC_MODE_ACTIVE ( BOXARM ) && altHoldStickCentred ( ) )`; otherwise it waits | With the stick off-centre the drone armed with no goal and followed the stick ( compliance check ) |
| same | New `static void takeOffRefusedTick ( )`, the 2 s refusal timer, called on a refused arm **and** whenever ARM is on with `OK_TO_ARM` off, whatever the stick | Review finding: otherwise an off-centre stick during a refusal kept the take-off alive past the 2 s drop. ( Its "after a landing" premise was wrong with the battery OK, see task 11 ) |

The wait applies to ARM off, and to ARM on + off-centre + `OK_TO_ARM` set. In throttle mode ( ALT HOLD off ) the take-off
now arms only at mid stick ( 1460-1540 ); before, it armed anywhere from 1100 to 2000. That is not a safety regression
( review ).

## Task 15: flip watchdog

| File | Change | Why |
|---|---|---|
| [acrobats.cpp](../../../../src/main/flight/acrobats.cpp) | `FLIP_WATCHDOG_MS` 8000 and `FLIP_UPRIGHT_MAX_DECI` 700. At the top of `flip ( true )`, a flip running longer than 8 s since `flipStartTime` ends: `flipState` 0, rates 75. Within 70° on both axes it hands back ( BARO box and ANGLE mode on, as a finished flip does ); otherwise it disarms, as the other stuck-flip timeouts do | SLOWDOWNANDEXIT has no exit for some attitudes, so a flip could stay active until disarm. That blocked `Command_Land` ( task 12 ), the battery auto-land and crash detection. 70° is the crash detector's limit ( `failsafe.cpp:368` ): beyond it, the detector would disarm the drone in the same loop |

From the review:

- `flipStartTime` is set on the same line that starts every flip.
- Measured flips take about 2.5 s, so the margin is more than 5 s.
- A watchdog end gets the same altitude-hold flip-exit handover as a normal end.
- An inverted drone ( roll above 90° ) is never handed back.

## Task 8: pipeline docs ( staged ) and comments

| File | Change | Why |
|---|---|---|
| [PIPELINE_UPDATE.md](PIPELINE_UPDATE.md) | Staged edits: `User_Space_API.md` ( data structures, functions, landing and take-off bullets, link-drop note, flowchart node `dropUserTakeOff ( )` ), `Power_BMS_Pipeline.md` ( landing confirm, cancel rules, references ), `Altitude_Hold_Estimator.md` ( idle release ); CLAUDE.md and `FLIGHT_INVARIANTS.md` lines; drift list | Applied at commit by `pluto-commit` |
| [mw.cpp](../../../../src/main/mw.cpp) | `batteryCriticalAutoLand ( )` header comment: user take-off / flip are refused first, and the re-assert is a backstop; a flip ends within 8 s | The comment described the pre-task-1 behaviour ( task 1 review ) |
| [altitudehold.cpp](../../../../src/main/flight/altitudehold.cpp) | Landing descent comment: `Command_Land` starts at 1305 − speed, clamped to −100 cm/s | It described only the 1300 start ( task 3 review ) |

## Task 9: topic review fixes

| File | Change | Why |
|---|---|---|
| [command.cpp](../../../../src/main/command/command.cpp) | `executeCommand ( )` also clears the refusal timer when `command_status == FINISHED` | `mwDisarm ( )` sets FINISHED but leaves TAKE_OFF, so a disarm with a take-off queued kept the timer, and the next refused take-off was dropped at once |
| [altitudehold.cpp](../../../../src/main/flight/altitudehold.cpp) | HISTORY row | rule 18 |
| [FC-Control.h](../../../../src/main/API/FC-Control.h), [wiki](../../../../docs/API/FC_CONTROL_API_WIKI.md) | "low battery" → "critical battery"; drop table: 2 s at any stick while calibrating | `isBatteryLow` is never set, and BATTERY_WARNING does not refuse arming |

Build ( 16 Oct 2026, template `PlutoPilot.cpp` ): PRIMUS_X2_v1 clean gate, no new `src/` warnings, text 108276, data 1280,
bss 16760, so flash is **+456 B** against 3.13.0. PRIMUS_V5 and PRIMUSX2 both build ( 106.9 KB / 106.8 KB flash ).

## File index

| File | Tasks |
|---|---|
| `src/main/API-Src/FC-Control-Command.cpp` | 1, 11, 12 |
| `src/main/command/command.cpp` | 2, 3, 11, 13 |
| `src/main/command/command.h` | 11 |
| `src/main/io/serial_msp.cpp` | 11 |
| `src/main/mw.cpp` | 11, 8 ( comment ) |
| `src/main/flight/altitudehold.cpp`, `.h` | 2, 8 ( comment ) |
| `src/main/API/FC-Control.h` | 7, 12 |
| `docs/API/FC_CONTROL_API_WIKI.md` | 7 |
| `Makefile` | 7 |
| `src/main/flight/acrobats.cpp` | 15 |
