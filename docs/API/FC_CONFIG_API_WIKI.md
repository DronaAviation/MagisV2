# MagisV2 Stick Rate API Wiki

Change how fast the drone turns at full stick with `setDesiredRate ( )`, and read the setting back with
`getDesiredRate ( )`. Both are declared in `API/FC-Config.h` and available from `PlutoPilot.h`. This page covers only
these two functions.

API version **2.0.0** ( FW 4.0.0; developed as 1.8.7, a development build released in 2.0.0 ). See "Changes in 1.8.7".

## Quick Start

```cpp
#include "PlutoPilot.h"

void plutoRxConfig ( void ) {
  Receiver_Mode ( Rx_ESP );
}

void plutoInit ( void ) {
}

// Runs once each time Developer Mode is switched on.
void onLoopStart ( void ) {
  setDesiredRate ( AG_YAW, 20 );    // yaw stick about twice as fast as the default ( 0 )
}

void plutoLoop ( void ) {
}

// Runs once when Developer Mode is switched off. The setting does not end on its own, so put it back.
void onLoopFinish ( void ) {
  setDesiredRate ( AG_YAW, 0 );
}
```

Switch Developer Mode on and turn with the yaw stick: the drone turns about twice as fast at the same stick.

---

## Functions

| Function | What it does |
|---|---|
| `setDesiredRate ( axis, rate )` | Sets the stick rate of `AG_ROLL`, `AG_PITCH` or `AG_YAW`. Acts at once and holds until the next call or a reboot. |
| `getDesiredRate ( axis )` | Returns the rate setting of that axis: the value set, or the default when none was set. |

## The rate value

`rate` is the Cleanflight rate setting. The bigger it is, the faster the drone turns at the same stick. It does not
change how the drone flies with the sticks centred.

| Axis | Range | Default ( no call ) | Out of range |
|---|---|---|---|
| `AG_ROLL` | 0-100 | 75 | clamped: 150 becomes 100, -5 becomes 0 |
| `AG_PITCH` | 0-100 | 75 | clamped |
| `AG_YAW` | 0-255 | the stored setting, 0 unless the app changed it | clamped: 400 becomes 255 |

Any other axis value is ignored by `setDesiredRate ( )`, and `getDesiredRate ( )` returns 0 for it.

## Where it acts

With the default PID controller:

| Axis | Flight modes | Full-stick turn rate |
|---|---|---|
| `AG_YAW` | every mode, ANGLE included | grows with `rate + 20`: 20 doubles the default 0, 60 is four times it |
| `AG_ROLL`, `AG_PITCH` | only outside ANGLE mode, e.g. after `FlightMode_Set ( RATE )` | grows with `rate + 20`: 30 spins at about half the default 75 |

**With the default PID controller, roll and pitch rates have no effect in ANGLE mode.** ANGLE mode turns the stick
into a tilt angle, not a turn rate. To change how far the drone leans, use `setDesiredAngle ( )` instead.

One exception: if the PID controller has been changed from the default through the app or the CLI, the roll, pitch and
yaw rates also scale that controller's own P, I and D gains as the stick moves, in **every** mode, ANGLE included. A
high rate then weakens the stabilisation at full stick. A stock drone uses the default controller, where this does not
happen.

## How long it holds

- It acts at once and **holds until the next call or a reboot**: through arming, disarming, Developer Mode on and
  off, and Wi-Fi hiccups.
- **It is not dropped when Developer Mode is switched off.** Unlike `setDesiredAngle ( )`, the pilot keeps your rate
  afterwards. To use a rate only while your code runs, set it back in `onLoopFinish ( )` ( see the examples ).
- `getDesiredRate ( )` returns what you set, after the clamp. Before any call it returns the rate in use.

## Flips

A flip sets its own rates when it starts, and roll and pitch go back to your value when it ends.

**A roll or pitch call made while a flip is running waits for the flip to end.** The value is kept, and
`getDesiredRate ( )` returns it at once, but the flip goes on flying its own rates and your value is applied when the
flip ends. A yaw call applies at once: the flip never uses yaw. A flip ends within 8 s of `Command_Flip ( )`.

A flip that does not finish in time either hands your value back straight away or disarms the drone, and your value
is applied then. Once a flip has ended, a new call applies at once as usual.

## Saving

`setDesiredRate ( )` does not save anything. It changes the drone's live settings, so:

- if the settings are saved while your rate is set ( a settings save or a profile change from the app, a stick trim,
  or an accelerometer or compass calibration ), your rate is saved too and **comes back after a reboot**;
- if the app then loads other settings ( a profile change or a settings reset ), yaw takes the loaded value, while
  roll and pitch are put back to your value on the next disarmed loop. Either way `getDesiredRate ( )` keeps
  returning your value until you call `setDesiredRate ( )` again.

---

## Examples

### Read the settings once ( `onLoopStart` )

```cpp
void onLoopStart ( void ) {
  Monitor_Print ( "roll:", ( int ) getDesiredRate ( AG_ROLL ) );
  Monitor_Print ( " pitch:", ( int ) getDesiredRate ( AG_PITCH ) );
  Monitor_Println ( " yaw:", ( int ) getDesiredRate ( AG_YAW ) );
}
```

With no `setDesiredRate ( )` call this prints `roll:75 pitch:75 yaw:0` on a drone with the default settings.

### Use a rate only while Developer Mode is on ( `plutoInit`, `onLoopStart`, `onLoopFinish` )

```cpp
static int32_t stockYaw = 0;

void plutoInit ( void ) {
  stockYaw = getDesiredRate ( AG_YAW );    // before any setDesiredRate ( ): the stored setting
}

void onLoopStart ( void ) {
  setDesiredRate ( AG_YAW, 40 );           // three times the default yaw stick
}

void onLoopFinish ( void ) {
  setDesiredRate ( AG_YAW, stockYaw );     // the pilot gets the stored setting back
}
```

`onLoopStart ( )` and `onLoopFinish ( )` also run on a Wi-Fi hiccup, so the rate may switch back and forth for a
moment. That is harmless: setting the same value again changes nothing.

### Slower roll and pitch in RATE mode ( `onLoopStart` )

```cpp
void onLoopStart ( void ) {
  setDesiredRate ( AG_ROLL, 30 );     // about half the default spin rate
  setDesiredRate ( AG_PITCH, 30 );
}
```

This only changes how the drone flies in RATE mode. In ANGLE mode it has no effect.

## Pitfalls

- **Roll / pitch "do nothing".** They act only outside ANGLE mode. In ANGLE mode, the default, use
  `setDesiredAngle ( )` to make the drone lean.
- **The rate stays after Developer Mode is off.** Set it back in `onLoopFinish ( )` if the pilot should not keep it.
- **A high yaw rate makes the yaw stick very fast** in every mode: 255 is about 14 times the default 0. Start small.
- **A roll / pitch change during a flip waits for the flip to end** ( see "Flips" ). `getDesiredRate ( )` already
  returns the new value, but the drone does not use it until then.
- **`getDesiredRate ( )` is the setting, not the drone's rotation.** For the measured rotation rate use
  `Estimate_Get ( Rate, axis )`.
- **Saved settings.** If the app saves the settings while your rate is set, the rate survives a reboot ( see
  "Saving" ).

---

## Changes in 1.8.7

- **Fixed:** `setDesiredRate ( AG_ROLL | AG_PITCH, ... )` was overwritten with 75 on every loop while the drone was
  disarmed, and again at the end of every flip, so the value never stuck ( API test T16 ). It now holds until the next
  call or a reboot; a flip goes back to your value when it ends.
- **Fixed:** `getDesiredRate ( )` returned the controller's internal turn-rate request, not the setting, so it never
  matched what was set. It now returns the setting.
- **Fixed:** a roll or pitch call made while a flip was running replaced the flip's spin rate. A low value could
  stall the flip, and the firmware then disarmed the drone in mid-air. The value is now kept and applied when the flip
  ends.
- **Changed:** values outside the range are clamped ( 0-100 roll and pitch, 0-255 yaw ). Before, they wrapped: yaw
  400 became 144.
- Unchanged: with no `setDesiredRate ( )` call, roll and pitch are 75 and yaw is the stored setting, as before. The
  flip flies with the same rates as before.
