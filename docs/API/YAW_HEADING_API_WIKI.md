# MagisV2 Yaw Heading API Wiki

Point the drone's nose from user code: at a compass bearing with `setDesiredAngle ( AG_YAW, … )`, or by turning
relative to where it points with `DesiredAngle_setRelative ( AG_YAW, … )`. Both are declared in `API/FC-Config.h` and
available from `PlutoPilot.h`.

API version **1.6.0**. See "Changes in 1.6.0" for what differs from 1.5.x.

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

`setDesiredAngle` also takes `AG_ROLL` and `AG_PITCH`, in **deci-degrees** ( 100 = 10° ).
`DesiredAngle_setRelative` acts on `AG_YAW` only; `AG_ROLL` and `AG_PITCH` are ignored.

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
  stored headfree heading while your code turns the nose.

---

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
