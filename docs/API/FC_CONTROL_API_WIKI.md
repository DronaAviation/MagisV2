# MagisV2 Flight Command API Wiki

Arm, take off, land and flip from user code with `Command_Arm ( )`, `Command_TakeOff ( )`, `Command_Land ( )` and
`Command_Flip ( )`. They are declared in `API/FC-Control.h` and available from `PlutoPilot.h`.

API version **1.7.0**. See "Changes in 1.7.0" for what differs from 1.6.x.

## Quick Start

```cpp
#include "PlutoPilot.h"

// One flight per ARM switch-on. The flags live in plutoLoop ( ), not onLoopStart ( ), because
// onLoopStart ( ) runs again after every Wi-Fi hiccup.
static bool     takeOffSent = false;
static bool     landSent    = false;
static uint32_t armedAt     = 0;

void plutoRxConfig ( void ) {
  Receiver_Mode ( Rx_ESP );
}

void plutoInit ( void ) {
}

void onLoopStart ( void ) {
}

void onLoopFinish ( void ) {
}

void plutoLoop ( void ) {
  if ( ! App_isArmSwitchOn ( ) ) {    // ARM switch off: ready for the next flight
    takeOffSent = landSent = false;
    armedAt     = 0;
    return;
  }
  if ( ! takeOffSent ) {
    takeOffSent = true;
    Command_TakeOff ( 100 );    // arms the drone and climbs 100 cm
  }
  if ( FlightStatus_Check ( FS_ARMED ) ) {
    if ( armedAt == 0 ) {
      armedAt = millis ( );
    }
    if ( ! landSent && millis ( ) - armedAt > 10000 ) {    // hover 10 s, then land
      Command_Land ( 5 );
      landSent = true;
    }
  }
}
```

Before the drone will fly this:

1. Put it on the floor and turn **ALT HOLD** on in the app. Leave the throttle stick **centred**.
2. Turn **Developer Mode** on.
3. Turn the **ARM** switch on: the drone arms and climbs. For the next flight, switch ARM off and on again.

---

## Functions

| Function | What it does |
|---|---|
| `Command_TakeOff ( height )` | Climbs to `height` cm ( 100-250, default 150 ) and holds it. Arms the drone first if needed. |
| `Command_Land ( landSpeed )` | Descends, confirms touchdown and disarms. `landSpeed` sets the starting descent rate ( default 105 ). |
| `Command_Arm ( )` | Arms the drone at once, if the ARM switch is on and arming is allowed. Returns `true` when armed. |
| `Command_DisArm ( )` | Disarms the drone at once, in the air too: the motors stop. Returns `true` when disarmed. |
| `Command_Flip ( BACK_FLIP )` | Starts a back flip, in MAG ( heading hold ) mode only. A flip ends after at most 8 s. |

The commands are **queued**: each call records the request, and the firmware carries it out at the end of the
loop. Only one command runs at a time. A `Command_Flip` replaces a take-off that is still waiting for the ARM switch,
even when the flip itself then does not run. A second `Command_TakeOff` while one is queued is ignored: the first
height stays.

---

## Taking off

- **From the floor, disarmed.** `Command_TakeOff` arms the drone once the ARM switch is on **and the throttle is
  centred**, then climbs. Until then it waits: switch off, or stick off centre. Exception: with the stick **at the
  bottom** ( below 1100 ) the ARM switch itself arms the drone to ground idle, and the next bullet applies.
- **After `Command_Arm ( )`.** It still climbs, if it comes within 3 s of arming ( for example, in the same
  `onLoopStart ( )` ).
- **After the ARM switch armed the drone with the stick low.** The motors run at ground idle. `Command_TakeOff`
  leaves the idle and climbs, at any time while the drone idles on the floor, **once the stick is at 1460-1500**.
  With the stick still at the bottom the call is used up with no height.
- **In the air** ( more than 3 s after arming, not at idle ): it sets no height. Use
  `DesiredPosition_setRelative ( Z, cm )` to climb from where the drone is.
- **Once per arm.** A second `Command_TakeOff` in the same flight is ignored. After a disarm it works again.
- **The throttle must be centred** when the take-off sets the height: within the alt-hold deadband of 1500
  ( `alt_hold_deadband`, default 40, so 1460-1540 ). The firmware reads the throttle channel that altitude hold flies,
  so a throttle override from `RcCommand_Set` counts too. On a drone that is already armed, an off-centre stick sets
  no height and uses the take-off up. Call it again with the stick centred, within 3 s of arming or at ground idle.
- **The height.** On the barometer it is measured from the current height. With the down laser
  ( `XRanging.initAlt ( )` ) it is measured from the floor. A 100 cm climb takes about 2-2.5 s, and the drone settles
  within about 20 cm of the height.

### When a take-off is dropped

A queued take-off that cannot arm the drone does not wait for ever:

| Situation | What happens |
|---|---|
| ARM switch on, but arming is refused ( calibrating, critical battery ) | Dropped after 2 s ( while calibrating, whatever the stick ). Call it again once arming is allowed |
| ARM switch on, arming allowed, stick off centre | Waits. It arms and climbs once the stick is centred |
| ARM switch off, Developer Mode on | Waits. It arms as soon as the switch goes on ( stick centred ) |
| ARM switch off, then Developer Mode switched off | Dropped. It never arms the drone without your code running |
| A landing is in progress | Ignored |

The app's take-off button follows the same rules, except that switching Developer Mode off does not drop it. It
climbs to the last height a `Command_TakeOff` stored, or 120 cm after a disarm or at power-up.

**After a landing the drone can take off again at once.** With the battery OK, arming is allowed again right
after the landing's disarm, even with the ARM switch still on. A `Command_TakeOff` ( or `Command_Arm ( )` ) sent
then, for example by an `onLoopStart ( )` that re-runs after a Wi-Fi hiccup, re-arms the drone and climbs. Only
your code can prevent this: send the take-off once per ARM switch-on, as in the Quick Start.

---

## Landing

- `Command_Land` is accepted only while the drone is armed, no other command is running and **no flip is in
  progress**. A landing sent during a flip is ignored: send it again after the flip.
- The descent throttle starts at **1305 − `landSpeed`** and is lowered steadily to 1150 ( a `landSpeed` above 155
  starts below 1150 and stays there ). In ALT HOLD the descent rate is ( 1500 − throttle ) / 4 cm/s, at most
  100 cm/s:

  | `landSpeed` | Start | Descent at the start |
  |---|---|---|
  | 5 | 1300 | about 50 cm/s ( the same as the failsafe landing ) |
  | 105 ( default ) | 1200 | about 75 cm/s |

- **Touchdown** is confirmed when the descent has stopped at a low throttle for 300 ms, and never earlier than 1 s
  after the landing started. A firm impact on the floor disarms the drone at once. After 30 s a landing disarms
  the drone wherever it is ( backstop ).
- **Nothing in user code except `Command_DisArm ( )` can interrupt a landing.** `Command_TakeOff` and `Command_Flip`
  are ignored until the drone has disarmed. The same applies to the RC-loss failsafe and the low-battery landings.

---

## Examples

### Arm first, then take off

```cpp
static bool flown = false;    // once per ARM switch-on: see "After a landing" above

void plutoLoop ( void ) {
  if ( ! App_isArmSwitchOn ( ) ) {
    flown = false;
    return;
  }
  if ( ! flown && Command_Arm ( ) ) {
    flown = true;
    Command_TakeOff ( 120 );    // within 3 s of arming: climbs 120 cm
  }
}
```

### Climb further once airborne

```cpp
// In the air Command_TakeOff sets no height: move the altitude target instead
DesiredPosition_setRelative ( Z, 50 );    // 50 cm higher
```

---

## Pitfalls

- **`onLoopStart ( )` runs again after every Wi-Fi hiccup** ( an RC frame more than 200 ms late, about every few
  seconds on Wi-Fi ). Code that calls `Command_TakeOff` there calls it again. The firmware ignores the repeat during
  the flight ( once per arm ) and during a landing. After the landing, though, a repeat with the ARM switch **off**
  waits, and takes off again at the next ARM-on. With the ARM switch still **on** and the battery OK, it re-arms and
  climbs at once. **Send the take-off ( and `Command_Arm ( )` ) once per ARM switch-on, from `plutoLoop ( )`**, as in
  the Quick Start and the Examples.
- **Do not call `Command_TakeOff` on every `plutoLoop ( )` tick.** It is harmless in flight, but after a landing it
  takes off again.
- **`Command_DisArm ( )` stops the motors in the air.** Land with `Command_Land` instead.
- **`Command_Flip` needs MAG mode** ( heading hold ). Without it the flip is ignored.
- **`Command_Land` during a flip is ignored.** Wait for the flip to end ( about 2.5 s ), then send it. A flip that
  does not finish is ended after 8 s: back to level flight if the drone is within 70°, otherwise it is disarmed.
- **Throttle mode** ( ALT HOLD off ): take-off and landing still run, but without altitude hold the take-off has no
  height to fly to. Fly these commands with ALT HOLD on.

---

## Changes in 1.7.0

- **Fixed:** `Command_Arm ( )` followed by `Command_TakeOff ( )` skimmed the floor: no height was set when the drone
  was already armed. It now climbs if it comes within 3 s of arming, or while the ARM-switch ground idle runs.
- **Fixed:** `Command_TakeOff` or `Command_Flip` during a landing stopped the landing logic but kept the throttle
  pinned. The drone touched down with the motors running and the stick locked out. Both are now ignored during a
  landing.
- **Fixed:** the default `Command_Land ( )` could confirm touchdown at the start of the landing and disarm in the air.
  Touchdown now needs at least 1 s of landing.
- **Fixed:** a take-off queued while arming was refused stayed queued, and armed the drone at the next ARM-on, even
  with Developer Mode off. It is now dropped after 2 s of refused arming, or at a Developer Mode switch-off.
- **Changed:** `Command_TakeOff` sets the height once per arm, and only with the throttle centred. From disarmed,
  it arms only with the throttle centred, and waits otherwise.
- **Changed:** `Command_Land` is ignored while a flip is in progress. Before, it took the throttle over in mid-flip.
- **Fixed:** a flip that got stuck in its slow-down phase stayed active until disarm, blocking landings and crash
  detection. It now ends after 8 s: in level flight if the drone is within 70°, otherwise it is disarmed.
