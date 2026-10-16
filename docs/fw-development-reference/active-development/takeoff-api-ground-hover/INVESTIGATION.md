# Take-off API ground hover - Investigation

[README](README.md) · [TASKS](TASKS.md) · [SCOUT](SCOUT.md) · [INVESTIGATION](INVESTIGATION.md)

## Problem

In `PlutoPilot.cpp`, `onLoopStart ( )` called `Command_Arm ( )` and then `Command_TakeOff ( 100 )` once, in the
same tick, when Developer Mode came on. The motors spun up, and the drone lifted a few cm and skimmed the floor. It
never climbed to the asked-for 100 cm.

## Flight conditions ( from the user )

| | |
|---|---|
| Target | `PRIMUS_X2_v1` |
| Hook | `onLoopStart ( )`, both calls once, `Command_Arm` first |
| ALT HOLD ( app ) | on |
| Throttle stick | centre |
| `Command_TakeOff` alone | flown 16 Oct 2026 ( log-2 ): climbs to about 90 cm |
| Log | A/B flight, 16 Oct 2026: [log-1.txt](log-1.txt) ( Arm + TakeOff ), [log-2.txt](log-2.txt) ( TakeOff alone ), [TESTING.md](TESTING.md) §2 |

## Flight evidence ( task 2 )

H1 is confirmed in flight. In log-1 ( Arm + TakeOff ) `isTakeOffHeightSet` stayed 0 and the altitude target stayed at
0 cm for the whole 12 s. The estimate stayed at the floor ( −19 to +5 cm ) with the motors at about 1555 µs. In log-2
( TakeOff alone ) the height was set, the target ramped to 100 cm in 2.5 s, and the drone held 84-94 cm. H2 was ruled
out: the throttle stick read 1500 throughout. H3 was ruled out: ALT HOLD was on throughout.

## How take-off works ( code trace )

1. `Command_TakeOff ( h )` ( [FC-Control-Command.cpp:124-135](../../../../src/main/API-Src/FC-Control-Command.cpp#L124-L135) )
   only stores `takeOffHeight = constrain ( h, 100, 250 )` cm and sets `current_command = TAKE_OFF`,
   `command_status = RUNNING`. Nothing flies yet.
2. `executeCommand ( )` at the end of the loop ( `mw.cpp:1491` ) runs `takeOff ( )`
   ( [command.cpp:93-127](../../../../src/main/command/command.cpp#L93-L127) ), which has two branches:
   - **Not armed, ARM switch on:** `mwArm ( )`, then the climb goal: `setDesiredPosition ( Z, h )` on the laser or
     `DesiredPosition_setRelative ( Z, h )` on the baro ( lines 107-124 ).
   - **Already armed:** `current_command = NONE`, `command_status = FINISHED`, `isTookOff = true`. **No height is
     set** ( lines 97-103 ).
3. `Command_Arm ( )` ( [FC-Control-Command.cpp:157-167](../../../../src/main/API-Src/FC-Control-Command.cpp#L157-L167) )
   calls `mwArm ( )` straight away, inside the user hook, before `executeCommand ( )` runs.

So `Command_Arm` followed by `Command_TakeOff` always reaches the "already armed" branch. Altitude hold keeps the
setpoint it took at the arm instant ( `resetAltSetpoint`, `altitudehold.cpp:435` ), which is the floor.
`Command_Arm` also does not set `isThrottleStickArmed`, so the ground idle ( `altitudehold.cpp:581` ) is skipped:
the controller starts from `initialThrottleHold` ( 1500, about hover ) and holds 0 cm. That gives the few-cm skim.

## Hypotheses ( ranked )

| # | Hypothesis | For | Against | Test |
|---|---|---|---|---|
| H1 | Arming first makes the take-off do nothing: no climb goal, and no ground idle | Code trace above; it explains both the hover thrust and the 0 cm hold | Not yet seen in a log | Flight A ( Arm + TakeOff ): desired Z equals ground Z. Flight B ( TakeOff alone ): desired Z is about +100 cm and the craft climbs in about 2.7 s |
| H2 | Throttle stick off centre: a velocity request cancels the goal ( `altitudehold.cpp:545,741`, deadband 40 ) | A low stick asks for -30 cm/s | User had the stick centred; H1 sets no goal to cancel anyway | Log `RcData_Get ( RC_THROTTLE )` |
| H3 | BARO_MODE off, so no height is flown | - | ALT HOLD was on in the app | Log `FlightMode_Check ( ATLTITUDEHOLD )` |
| H4 | `Command_Arm` re-zeroes the baro datum through `mwArm` → `baroResetGroundLevel` | `mw.cpp:597` | Calls run once in `onLoopStart`, not every loop | None needed unless H1 fails |

## Q&A that shaped the plan

- Mode: `check` ( the fix comes after the cause is confirmed, through `/pluto-grill fix takeoff-api-ground-hover` ).
- Symptom: lifted a few cm and skimmed, never climbed.
- Evidence: A/B flight with a log ( Arm + TakeOff, then TakeOff alone ).
- Recommendation: compare both outcomes and let the user choose. Option (a): `Command_TakeOff` sets the climb goal
  even when the drone is already armed ( behaviour change, API bump ). Option (b): docs only ( `Command_TakeOff` arms by
  itself, and Arm + TakeOff is not supported ).

## `Command_TakeOff` during a landing ( task 7 )

**Confirmed from the code. Not seen in flight: logs 1-3 had no Dev Mode restart.**

### Mechanism

1. `Command_TakeOff` replaces any command that is not TAKE_OFF, LAND included ( `FC-Control-Command.cpp:126-129` ).
   `Command_Flip` does the same ( `:151-153` ). `executeCommand ( )` then runs `takeOff ( )`, which sees ARMED and sets
   NONE / FINISHED ( `command.cpp:97-103` ). `land ( )` is never called again.
2. Only `finishLanding ( )` ( `command.cpp:170` ) or a disarm ( `mw.cpp:553` ) clears `isLanding`. Neither runs, so
   `isLanding` stays true.
3. While `isLanding` is true, `rcData [ THROTTLE ]` is pinned to `landThrottle` twice per loop ( `mw.cpp:385-386`,
   `:1391-1393` ). `landThrottle` is written only by the ramp inside `land ( )` ( `command.cpp:201` ), by
   `Command_Land` and at disarm. So it freezes at its last ramp value.
4. Result: the drone keeps descending at the frozen rate. In ALT HOLD that is `setVelocity = deflection / 4`
   ( `altitudehold.cpp:561-563` ), e.g. 1260 → −60 cm/s. The touchdown detection, the impact check and the 30 s timeout
   all live inside `land ( )` ( `command.cpp:203-225` ), so **nothing disarms the drone after touchdown**. The motors
   keep running on the floor, and **the pilot's throttle stick is ignored**, because `rcData` stays pinned.
5. The way out is the ARM switch: `disarm_kill_switch` = 1 ( `config.cpp:493` ) disarms at any throttle
   ( `rc_controls.cpp:195-197` ). Dev Mode off does not help: it stops user code, not the landing state.
6. A later `Command_Land` is accepted, because the status is FINISHED. But `setLandTimer` is still false, so `land ( )`
   resumes with the old `landStartTime`. If more than 30 s have passed, `LAND_TIMEOUT_MS` fires on the first cycle and
   `finishLanding ( )` disarms the drone **wherever it is, possibly in the air** ( `command.cpp:203-206` ).

### Which landings are exposed

| Landing | Exposed? | Why |
|---|---|---|
| User `Command_Land` | yes | nothing re-asserts it |
| RC-loss failsafe | yes, once the link returns | `failsafe.cpp:245-250` re-asserts LAND only while there is no RX data. User code only runs with RX data, so the overwrite happens as soon as the link is back and `onLoopStart ( )` re-runs |
| Battery critical | no | LAND is put back every loop before `executeCommand ( )` ( `mw.cpp:1250-1275` ) |

### Which user code triggers it

- `Command_TakeOff` ( or `Command_Flip` ) in `onLoopStart ( )`, the user's pattern. `onLoopStart ( )` re-runs on every
  RC frame gap over 200 ms ( about every 3 s on `Rx_ESP`, pluto-flighttest §1 ) and when the link returns after a
  failsafe.
- `Command_TakeOff` in `plutoLoop ( )` on every tick. It overwrites any landing on the next tick, so a user
  `Command_Land` can never complete. This corrects the scout brief, which called calling it every loop "harmless".

### Fix direction

Make `Command_TakeOff` and `Command_Flip` refuse while a landing is in progress ( `isLanding`, or
`current_command == LAND` ). That is a few bytes in `FC-Control-Command.cpp`, needed whichever take-off option is
chosen. Option (a)'s on-ground guard runs inside `takeOff ( )`, after the overwrite has already happened, so it does
not cover this.

## Fix options ( task 4 )

No code is changed in this topic. These are the choices for `/pluto-grill fix takeoff-api-ground-hover`.

### Facts the options rest on

- `takeOff ( )` sets the goal only in its arming branch ( `command.cpp:107-124` ). Its armed branch only sets
  FINISHED, NONE and `isTookOff` ( `:97-103` ). `isTakeOffHeightSet` is cleared at every disarm ( `mw.cpp:559` ), so it
  already means "a take-off goal was set during this arm".
- A goal written to `AltHold` is picked up by the next controller cycle ( `altitudehold.cpp:737-740` ), provided the
  controller is not held in reset. Flight-proven by log-2 ( from `takeOff ( )` ) and log-3 ( from user code ).
- **The ground idle wipes a goal.** After an ARM-switch arm with the stick low, `isThrottleStickArmed` keeps
  `altHoldGroundIdle` on while `rcData [ THROTTLE ] <= 1500` ( `altitudehold.cpp:581-583` ). The controller then calls
  `resetAltSetpoint ( )` on every cycle ( `:688-689` ). The app's centred stick is 1500, so the idle stays on.
  `Command_TakeOff` on a drone armed this way would have the same symptom as the user's report: no climb. The motors
  stay at idle rather than hover. This is from reading the code; it was not flown.
- The battery auto-land cannot be overridden by user code: user commands are replaced by LAND before
  `executeCommand ( )` ( `mw.cpp:1250-1255` ). The failsafe landing starts at 1300 ( `landThrottle` default,
  `mw.cpp:562` ).
- In the air, the baro branch's `DesiredPosition_setRelative ( Z, h )` means "climb a further h from here". The laser
  branch's `setDesiredPosition ( Z, h )` means "go to h above the floor" ( `command.cpp:117-121` ).

### Option (a): `Command_TakeOff` also works when the drone is already armed

In the armed branch of `takeOff ( )`: if no take-off goal has been set during this arm ( `! isTakeOffHeightSet` ) **and
the drone is still on the ground**, release the ground idle ( `isThrottleStickArmed = false` ), set the goal through the
same laser / baro branches as the arming path, and set `isTakeOffHeightSet`. Then finish as today. Factor the goal
code into one helper so both branches share it.

- **The on-ground guard is needed.** Without it, a pilot who flies up and then switches Dev Mode on, with user code
  calling `Command_TakeOff` in `onLoopStart ( )`, gets a further +100 cm climb on the baro. That can reach the ceiling
  indoors. Suggested guard: `getEstAltitude ( )` within 30 cm of the floor, and not landing. In the air the call stays a
  no-op, as it is today. The 30 cm value needs the user's sign-off.
- **Files:** `command/command.cpp` ( `takeOff ( )`, about 15 lines ). The helper could reuse `altLaserFusion ( )` as
  now. `API/FC-Control.h` documentation.
- **Cost:** about 60-100 B of flash, no RAM, no loop time ( it runs once per take-off ).
- **Safety:** touches arming and take-off only. It clears the ground idle only when user code asks for a take-off. The
  battery and failsafe landings are unaffected ( see above ). Releasing the idle starts the motors from about 1500 ( hover )
  instead of 1000. That is the same lift-off as `Command_Arm` + setRelative ( log-3, no hop ).
- **API:** a behaviour change to `Command_TakeOff`. Bump `API_Version` 1.6.0 → 1.7.0 and `FW_Version`, and document it
  in a new `docs/API/` wiki section for FC-Control ( none exists ).
- **Risk:** low. The code path is the one log-2 already flew; only its trigger widens. Validation: fly A again ( must
  now climb ), then B ( unchanged ), then an ARM-switch arm with the stick low followed by `Command_TakeOff` ( must
  climb, not idle ).

### Option (b): documentation only

Document the current behaviour: `Command_TakeOff` arms the drone itself and must be called while disarmed, with the
ARM switch on. If the drone is already armed, by `Command_Arm` or the ARM switch, the call does nothing. Then use
`DesiredPosition_setRelative ( Z, h )` ( flight-proven, log-3 ), and only with the stick above 1500 if the ARM switch
armed it with the stick low.

- **Files:** `API/FC-Control.h:92-100`. The comment says "height in units": it should say cm, constrained to 100-250,
  arms by itself, and is a no-op when armed. Also a new `docs/API` FC-Control wiki and `User_Space_API.md` ( it also
  names `Control_Command_t`, `Command.set` and `Estimate.get`, which do not exist, at `:15,20-21` ).
- **Cost:** none in firmware. No version bump ( behaviour unchanged ).
- **Risk:** the trap stays. `Command_Arm` followed by `Command_TakeOff` reads naturally, and the failure is silent:
  the drone hovers at the floor with no status or error.

### Bundled finding: the `Command_Land ( )` default ( found in the task 1 review )

`Command_Land ( uint8_t landSpeed = 105 )` sets `landThrottle = 1305 - landSpeed` ( `FC-Control-Command.cpp:145` ).
Any speed of 105 or more starts the landing at or below `LAND_CONFIRM_THROTTLE` 1200 ( `command.cpp:155` ). Then
`conclusive` holds from the first cycle, and a descent slower than 8 cm/s for 300 ms ( `command.cpp:217-225` ) disarms
the drone. That can happen while it is still in the air, in the first fraction of a second before the descent builds.
It is not proven to fire. The flown tests used speed 5 ( 1300 ) and landed cleanly ( logs 1-3 ).

Fix candidates, smallest first:

1. In `land ( )`, count `conclusive` only once the ramp has run for at least `LAND_SETTLE_MS` plus the time to reach a
   descent. Or require `landThrottle` to have dropped by some amount from `landThrottleStart`. Internal only, no API
   change.
2. Clamp `landThrottleStart` to at least 1250 in `land ( )`. This changes how fast `Command_Land ( 105+ )` starts down.
3. Change the header default to 5 ( 1300, the same as the failsafe ). This is an API behaviour change, and callers
   passing a large speed are still exposed.

Recommended: candidate 1. It keeps the meaning of `landSpeed` and closes the case for every caller. It needs one
landing flight with the default `Command_Land ( )`.

### Also noted

`updateCommandStatus ( )` has no callers, so `current_command` stays LAND ( 2 ) after a landing until the next command
( log-1 to log-3: `C:20` ). Nothing reads it in a way that matters today. Include it only if the fix touches that
code.

### Recommendation

**Option (a) with the on-ground guard, plus land fix candidate 1, plus option (b)'s documentation, which is needed
either way.** Option (a) removes a silent trap at about 100 B of flash, through a path already flown. The guard keeps
mid-air behaviour as it is today. **Plus the task 7 guard:** `Command_TakeOff` and `Command_Flip` must refuse while a
landing is in progress. This is the most safety-relevant item of the three, because the drone touches down with
the motors running and the stick locked out.

## Findings ( task 5, 16 Oct 2026 )

### The cause of the report: confirmed in flight

**`Command_TakeOff` does nothing when the drone is already armed.** `Command_TakeOff ( h )` only queues the command.
`takeOff ( )` sets the climb goal only in the branch where it arms the drone itself ( `command.cpp:107-124` ). When
`Command_Arm ( )` has armed the drone first, in the same `onLoopStart ( )` tick, `takeOff ( )` takes its armed branch
( `command.cpp:97-103` ). That branch marks the take-off finished and sets no height. Altitude hold keeps its target
at the floor, and the drone skims at below-hover thrust.

| Evidence | Value |
|---|---|
| log-1 ( Arm + TakeOff ) | `isTakeOffHeightSet` 0 and target `Zd` 0 cm for all 12 s. `Ze` −19 to +5 cm, motors about 1555 µs |
| log-2 ( TakeOff alone ) | height set, `Zd` 0 → 100 cm in 2.5 s, held 84-94 cm, motors about 1610 µs |
| log-3 ( Arm + setRelative ) | `Zd` 1 → 101 cm in 2.5 s, held 80-97 cm, the same climb as log-2 |

### Ranked hypotheses

| # | Hypothesis | Verdict | Evidence |
|---|---|---|---|
| H1 | Arming first makes the take-off do nothing | **confirmed** | log-1 vs log-2 differ only in `Command_Arm`. The height flag and the target show the difference |
| H2 | Throttle stick off centre cancels the goal | ruled out | `Tp` 1500 on every tick in all three logs |
| H3 | BARO_MODE off | ruled out | the ALT HOLD flag was 1 on every tick |
| H4 | `Command_Arm` re-zeroes the baro datum | not a factor | the user's calls run once; API arming never reaches `baroResetGroundLevel` ( first-arm branch, `mw.cpp:608-646` ) |
| - | Missing ground idle | not a factor | no API arming path sets it; log-2 and log-3 climbed without it |

### Further defects found along the way

| # | Defect | Severity | Status |
|---|---|---|---|
| D1 | `Command_TakeOff` / `Command_Flip` during a user or RC-loss landing stop `land ( )` but leave `isLanding` set. The throttle stays pinned, nothing disarms after touchdown and the stick is locked out; only the ARM-switch kill stops the motors. A later `Command_Land` after 30 s disarms at once, possibly in the air | **high** ( lands with the motors running; a possible disarm in the air ) | confirmed from the code ( task 7 ) |
| D2 | `Command_Land ( )` default 105 starts at `LAND_CONFIRM_THROTTLE` 1200. The touchdown check can pass at the start of the landing and disarm the drone in the air | medium | from the code, not proven to fire ( task 4 ) |
| D3 | After an ARM-switch arm with the stick at or below 1500, the ground idle wipes any goal, so `Command_TakeOff` or setRelative would not climb | medium | from the code, not flown ( task 4 ) |
| D4 | Doc drift: `FC-Control.h:98` says "height in units" and does not say that the call arms the drone. `User_Space_API.md:15,20-21` names APIs that do not exist. There is no FC-Control wiki | low | from the code |
| D5 | `updateCommandStatus ( )` has no callers, so `current_command` stays LAND after a landing | cosmetic | from the code |

### What users can do today ( no firmware change )

- Call `Command_TakeOff ( h )` on its own: it arms the drone ( ARM switch on, Dev Mode on after it ). Or call
  `Command_Arm ( )` and then `DesiredPosition_setRelative ( Z, h )` in the same tick ( log-3 ).
- Guard the take-off so a Wi-Fi restart cannot re-send it during a landing:
  `if ( ! FlightStatus_Check ( FS_ARMED ) ) Command_TakeOff ( h );`. Never call it on every `plutoLoop ( )` tick.
- Use `Command_Land ( 5 )` rather than the default until D2 is fixed.

### Recommendation

Plan the fix with `/pluto-grill fix takeoff-api-ground-hover`, in priority order:

1. **D1:** `Command_TakeOff` / `Command_Flip` refuse while a landing is in progress ( a few bytes ).
2. **The reported fault:** option (a). `takeOff ( )` also sets the goal when the drone is already armed, only on the
   ground ( within about 30 cm of the floor, value for sign-off ) and only once per arm. It also releases the ground
   idle, which fixes D3. About 100 B; API 1.6.0 → 1.7.0.
3. **D2:** count the touchdown check only after the ramp has run ( internal to `land ( )` ).
4. **D4:** the `FC-Control.h` comments, a new `docs/API` FC-Control wiki, and `User_Space_API.md`.

Validation flights: A again ( must climb ), B ( unchanged ), an ARM-switch arm with the stick low followed by
`Command_TakeOff` ( must climb ), a `Command_TakeOff` re-sent during a `Command_Land` ( landing must complete ), and a
default `Command_Land ( )` landing.
