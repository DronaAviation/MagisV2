# MagisV2 Yaw Heading API Wiki

Point the drone's nose from user code: at a compass bearing with `setDesiredAngle ( AG_YAW, … )`, or by turning
relative to where it points with `DesiredAngle_setRelative ( AG_YAW, … )`. Both are declared in `API/FC-Config.h` and
available from `PlutoPilot.h`. `setDesiredAngle` also tilts the drone on roll and pitch: see "Roll and pitch tilt".

API version **2.0.0** ( FW 4.0.0; developed as 1.8.6, a development build released in 2.0.0 ). See "Changes in 1.8.6" and "Changes in 1.6.0".

## Quick Start

```cpp
#include "PlutoPilot.h"

static uint32_t lastTurn = 0;

void plutoLoop ( void ) {
  // Turn 90 degrees clockwise every 5 seconds ( the first turn comes at once ):
  // four turns make a full circle
  if ( millis ( ) - lastTurn >= 5000 ) {
    DesiredAngle_setRelative ( AG_YAW, 90 );
    lastTurn = millis ( );
  }
}
```

The Developer Mode switch must be on. **No app switch is needed:** the firmware engages heading hold itself while your code
holds a yaw target.

---

## Functions

| Function | `angle` for `AG_YAW` | What it does |
|---|---|---|
| `setDesiredAngle ( AG_YAW, angle )` | Compass bearing, whole degrees, 0..359 ( other values are wrapped: 450 is 90, −90 is 270 ) | Turns to that bearing and holds it. Each call replaces the target. |
| `DesiredAngle_setRelative ( AG_YAW, angle )` | Turn in whole degrees; + is clockwise ( right ), − is anticlockwise | Turns by `angle` from the current target, or from where the nose points if no target is held. Calls add up. |

`setDesiredAngle` also takes `AG_ROLL` and `AG_PITCH`, in **deci-degrees** ( 100 = 10° ): it tilts the drone and holds
the tilt ( see "Roll and pitch tilt" ). `DesiredAngle_setRelative` acts on `AG_YAW` only; `AG_ROLL` and `AG_PITCH` are
ignored.

The current heading is `Estimate_Get ( Angle, AG_YAW )`, in degrees, 0..359.

---

## How a turn is flown

- **Heading hold flies the target.** The firmware compares the heading with the target and commands yaw towards it,
  taking the short way: from 10° to 350° it turns −20°, and a relative +270 flies −90. Full yaw speed is used down to
  about 10° from the target, then it slows and stops. A 90° turn takes about 1.5-2 s and ends within about 1°.
- **Heading hold is engaged for you.** While Developer Mode is on and your code holds a yaw target, the firmware turns
  on MAG ( heading hold ) mode, whatever the app's heading-hold switch says.
  - If the pilot has selected **HEADFREE**, that mode is kept: it holds heading too, and keeps its own stick mapping.
  - While a target is held, heading hold is on whenever the firmware flies it. `FlightMode_Check ( MAGHOLD )` read
    from `plutoLoop`, and the app's heading-hold indicator, can still flicker to off: they are read before the
    firmware switches it back on in each loop. Do not use them to tell whether your target is being flown.
  - The app's **flip** button works while a target is held, because flips need MAG mode.
  - Without a magnetometer there is no heading hold, and the yaw functions have no effect.
- **Relative turns add up exactly.** Each relative call adds to the target, not to the heading of the moment, so four
  calls of +90 come back to exactly the starting bearing, even if a call is made before the previous turn has
  finished.

---

## The pilot's yaw stick

The pilot can always take yaw back.

- **While the stick is pushed** more than 300 from centre ( 60 % of its travel to one side ), the drone yaws as the
  pilot commands and heading hold steps aside.
- **When the stick is released**, heading hold takes over again once the drone has almost stopped turning ( under
  about 17°/s ). What happens then depends on the setter:

| Last target came from | After the pilot lets go |
|---|---|
| `setDesiredAngle` | The drone **turns back to the bearing**. |
| `DesiredAngle_setRelative` | The drone **stays where the pilot left it**. Once it has stopped turning, the target moves to the nose, and the next relative call is measured from there. |

- **A relative call made while the pilot holds the stick** ( or while the drone is still turning from it ) is measured
  from the nose at that moment, and that target then stands. If the pilot keeps yawing after the call, the drone
  turns back to the call's target when the stick is released. A timer that fires during a long stick hold, as in the
  Quick Start, can do this.
- **A stick held from before arming** into the flight does not count as a pilot yaw: on release, a relative target
  is kept and the drone turns back to it.
- **`RcCommand_Set` yaw and a yaw target do not mix.** Heading hold replaces the yaw command once it is engaged. But
  after a stick release, or when heading hold first switches on, it waits until the drone turns slower than about
  17°/s, and during that wait a `RcCommand_Set` yaw override is flown. An override that keeps the drone turning faster
  than that stops heading hold from ever taking over. Use one or the other.

---

## When the target ends

| Event | What happens |
|---|---|
| Developer Mode switched **off** | The target is dropped. The drone stops turning and holds the heading it points at; it does not finish a half-done turn. The next relative call after dev mode comes back on is measured from the nose. |
| **Disarm** ( landing, crash, ARM switch ) | The target is dropped, so a drone moved by hand does not yaw back on the next takeoff. A target set **while disarmed** ( e.g. in `onLoopStart` on the ground ) is kept until takeoff, and the drone turns to it as it lifts. |
| A Wi-Fi **hiccup** ( an RC frame more than 200 ms late ) | The target is **kept**. `onLoopFinish` / `onLoopStart` still run again, but a turn in progress continues. This relies on the dev-mode channel's failsafe setting being HOLD ( the default ): with `rxfail` set to a fixed value outside the dev range, a stall longer than 0.6 s counts as switching dev mode off. |
| A long **RC loss** ( failsafe landing ) | The target is kept, and heading hold keeps flying it while the drone lands. This affects yaw only. |

A yaw call made inside `onLoopFinish` when dev mode is switched off is dropped with the target.

---

## Roll and pitch tilt

`setDesiredAngle ( AG_ROLL, angle )` and `setDesiredAngle ( AG_PITCH, angle )` tilt the drone by `angle` deci-degrees
( 50 = 5° ) and hold the tilt until a new call replaces it. The drone tilts until `Estimate_Get ( Angle, AG_ROLL )` or
`Estimate_Get ( Angle, AG_PITCH )` reads that value. `setDesiredAngle ( AG_ROLL, 0 )` levels it again.

**Signs**, the same as `Estimate_Get ( Angle, … )`
( [FC_DATA_API_WIKI.md, "Angles"](FC_DATA_API_WIKI.md#angles-estimate_get--angle-angle-) ): **`AG_ROLL` positive puts
the right side down**, so the drone moves right, and **`AG_PITCH` positive puts the nose down**, so it moves forward.
Negative values are the other way. The tilt is measured from level, around the drone's own nose, so it does not follow
the pilot's HEADFREE heading.

One call is enough: there is no need to call it every loop, but calling it every loop with the same value does no harm.
A held tilt keeps the drone accelerating that way, so always level it again: see the example "Lean right for 1 s, then
level". Do not set a tilt in `onLoopStart`: it re-runs after every Wi-Fi hiccup and would tilt the drone again after
your code had levelled it. Nothing needs undoing in `onLoopFinish`: switching Developer Mode off drops the tilt.

- **The pilot's stick adds to it.** The tilt and the stick are added, and the sum keeps the pilot's angle limit
  ( `max_angle_inclination`, 20° by default ). With a 15° tilt held, the pilot can add only 5° more the same way.
- **When it is flown.** All of these must hold; otherwise the tilt is **kept but not flown**, and the drone flies the
  sticks alone:

| Condition | Why |
|---|---|
| ANGLE mode is on | ACRO and HORIZON fly rates, not angles. The call does not switch to ANGLE mode. |
| The drone is airborne: the throttle has been raised since arming **and** it is at least 20 cm above where it armed | A tilt on the ground would pivot the take-off on a leg. A tilt set before take-off starts once the drone is off the ground. |
| Developer Mode is running, or stopped for no more than 0.6 s | A normal Wi-Fi hiccup keeps the tilt. In a longer stall your code is not running and the pilot's sticks have centred, so the drone levels instead of flying away; the tilt comes back with the link. |
| No flip is running | A flip owns roll and pitch. The tilt comes back when the flip ends. |
| No landing is running ( `Command_Land`, the app, RC loss or low battery ) | A tilt would drift the descent. The landing ends in a disarm, which drops the tilt. |
| In ALT_HOLD with the throttle stick fully down: the drone has been seen moving down and has not touched down | With the stick fully down the drone may already be standing on a table or the floor, where a tilt cannot be flown and would make the next take-off kick. In the air the tilt pauses only until the descent shows, and comes back when the stick comes up. |

- **When it ends.** The tilt is dropped when Developer Mode is switched **off** ( the drone levels ) and when the drone
  **disarms**, so the next flight never starts on an old tilt. A tilt set while disarmed is kept until take-off.
- **Default PID controller only.** The tilt is flown by the default controller ( `pidRewrite` ); the others, chosen only
  through the stored configuration, ignore it.
- **`getDesiredAngle ( AG_ROLL / AG_PITCH )`** returns the stick plus the tilt being flown, **before** the 20° limit,
  so it can read more than the drone actually flies. `getDesiredAngle ( AG_YAW )` returns 0.

---

## Examples

### Square pattern with a 3 s pause after each turn

```cpp
// Not reset in onLoopStart: it re-runs after every Wi-Fi hiccup, which would
// start the square again and add extra turns.
static uint32_t lastTurn = 0;
static uint8_t  turns    = 0;

void plutoLoop ( void ) {
  if ( turns < 4 && millis ( ) - lastTurn >= 3000 ) {
    DesiredAngle_setRelative ( AG_YAW, 90 );
    lastTurn = millis ( );
    turns++;
  }
}
```

### Face north, then east

```cpp
static bool facedNorth = false;    // not reset in onLoopStart ( see above )

void plutoLoop ( void ) {
  int16_t hd = Estimate_Get ( Angle, AG_YAW );

  if ( ! facedNorth ) {
    setDesiredAngle ( AG_YAW, 0 );            // north
    if ( hd <= 2 || hd >= 358 ) facedNorth = true;
  } else {
    setDesiredAngle ( AG_YAW, 90 );           // then east
  }
}
```

`setDesiredAngle` can be called on every loop with the same bearing: it only replaces the target.

### Lean right for 1 s, then level

```cpp
static uint32_t leanStart = 0;    // not reset in onLoopStart ( see above )
static uint8_t  leanStep  = 0;

void plutoLoop ( void ) {
  if ( leanStep == 0 ) {
    setDesiredAngle ( AG_ROLL, 50 );    // 5° right side down, held
    leanStart = millis ( );
    leanStep  = 1;
  } else if ( leanStep == 1 && millis ( ) - leanStart >= 1000 ) {
    setDesiredAngle ( AG_ROLL, 0 );     // level again
    leanStep = 2;
  }
}
```

The tilt is held between the two calls: the firmware keeps flying it while `plutoLoop` does nothing.

---

## Pitfalls

- **Do not call `DesiredAngle_setRelative` unconditionally in `plutoLoop`.** Each call adds another turn: at the
  default 100 ms loop, +90 on every loop asks for 900°/s and the drone never settles. Guard it with a timer, a counter
  or an event, as in the examples.
- **Yaw is in degrees, roll and pitch in deci-degrees.** Both yaw functions wrap any value into 0..359, so
  `setDesiredAngle ( AG_YAW, 900 )` points at 180° ( 900 mod 360 ), not 90°, and `-90` is 270°.
- **`onLoopStart` re-runs after a Wi-Fi hiccup.** Counters and timers reset there start again, and a turn set there is
  made again. Keep sequence state in `static` variables that `onLoopStart` does not reset, as in the examples.
- **HEADFREE changes the sticks, not the yaw API.** With the pilot's HEADFREE on, roll and pitch stay relative to the
  stored headfree heading while your code turns the nose. A `setDesiredAngle` roll / pitch tilt is not rotated: it
  stays relative to the drone's own nose.
- **A tilt keeps the drone moving.** A held tilt accelerates the drone in that direction for as long as it is held.
  Hold it briefly, and level with `setDesiredAngle ( AXIS, 0 )`; the firmware does not limit how far it travels.
- **No tilt on the ground.** A tilt set before take-off waits until the drone is 20 cm up. If it never seems to start,
  check that the drone is in ANGLE mode and actually off the ground.

---

## Changes in 1.8.6

- **Fixed:** `setDesiredAngle ( AG_ROLL / AG_PITCH, angle )` did not tilt the drone ( API test T11, B4 ). It wrote the
  drone's measured attitude instead of a target, so `Estimate_Get ( Angle, … )` read the value back once and the next
  sensor update overwrote it. A value above 700 could also trip the crash detector and disarm the drone in the air.
  The tilt is now flown and held until replaced; see "Roll and pitch tilt".
- **Changed:** `getDesiredAngle ( AG_ROLL / AG_PITCH )` now includes the user tilt being flown. It is documented as
  the request before the angle limit, and `getDesiredAngle ( AG_YAW )` as 0.

## Changes in 1.6.0

- **New:** `DesiredAngle_setRelative ( angle_e, int32_t )`.
- **Fixed:** a yaw target did nothing unless the app's heading-hold switch was on. Heading hold is now engaged
  automatically while user code holds a yaw target.
- **Changed:** a Wi-Fi hiccup no longer drops the yaw target, and switching Developer Mode off holds the current
  heading instead of finishing the last turn. A disarm drops the target.
- **Changed:** the `AG_YAW` unit of `setDesiredAngle` is documented as whole degrees ( 0..359 ); it was listed as
  deci-degrees.
- **Fixed:** `setDesiredAngle ( AG_YAW, angle )` wraps `angle` into 0..359. Before, a value of 720 or more ( e.g. 900,
  written as deci-degrees ) or −361 or less made the drone spin without stopping.
