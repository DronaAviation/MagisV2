# MagisV2 Failsafe API Wiki

Switch the firmware's built-in protections on and off from user code with `Failsafe_enable ( )` and
`Failsafe_disable ( )`. Both are declared in `API/FC-Config.h` and available from `PlutoPilot.h`. This page covers only
these two functions and the `failsafe_e` values they take.

API version **2.0.0** ( FW 4.0.0; developed as 1.8.8, a development build released in 2.0.0 ). See "Changes in 1.8.8".

## Quick Start

```cpp
#include "PlutoPilot.h"

void plutoRxConfig ( void ) {
  Receiver_Mode ( Rx_ESP );
}

void plutoInit ( void ) {
}

// Runs once each time Developer Mode is switched on, and again after a link drop.
void onLoopStart ( void ) {
  Failsafe_disable ( CRASH );    // no crash disarm while the drone is tied to a test stand
}

void plutoLoop ( void ) {
}

// Runs once when Developer Mode is switched off, and on a link drop. The setting does not end on its own, so put it back.
void onLoopFinish ( void ) {
  Failsafe_enable ( CRASH );    // crash disarm back on; the battery protection is not touched
}
```

While Developer Mode is on, a hard bump or a tilt past 70 deg no longer disarms the drone. The low-battery
protection stays on the whole time.

---

## Functions

| Function | What it does |
|---|---|
| `Failsafe_enable ( failsafe )` | Switches the named failsafe on; `ALL` switches all three on. The others keep their state. |
| `Failsafe_disable ( failsafe )` | Switches the named failsafe off; `ALL` switches all three off. The others keep their state. |

There is no function that reads the state back. Keep your own flag if your code needs to know it.

## The failsafes: `failsafe_e`

| Value | What it protects | When it is off |
|---|---|---|
| `LOW_BATTERY` | **Nothing.** It has no effect on the drone ( see Pitfalls ). It is kept so existing code still compiles. | No change. |
| `INFLIGHT_LOW_BATTERY` | All the battery levels, on the ground and in flight. Low battery: `FS_LOW_BATTERY` and the low-battery beeper. Critical: `FS_INFLIGHT_LOW_BATTERY`, the critical beeper, **the automatic landing**, and arming refused until the battery is changed. | No level is ever raised: no battery status, no beeper, no landing, and arming is allowed on a flat battery. |
| `CRASH` | The crash disarm: while armed in ANGLE mode, and not during a flip, a tilt past 70 deg or a hard impact disarms the drone and sets `FS_CRASHED`. | The drone is never disarmed for a crash. The motors keep running after an impact. |
| `ALL` | All three of the above. | |

The battery levels, the landing and the crash detector are described in the
[BMS API Wiki](BMS_API_WIKI.md) and the [Flight Command API Wiki](FC_CONTROL_API_WIKI.md).

## How the switches behave

- **Each call changes only the failsafe you name.** `Failsafe_enable ( CRASH )` switches the crash disarm on and leaves
  the battery protection as it was. `ALL` changes all three.
- **All three are on at power-up.** Code that never calls these functions has every protection.
- **A setting holds until the next call or a reboot.** Disarming, landing and switching Developer Mode off do not
  reset it. It is not saved: after a reboot all three are on again.
- **The crash switch acts at once.** The crash detector checks it on every loop.
- **The battery switch only stops a level from being raised.** Switching `INFLIGHT_LOW_BATTERY` off once the low
  battery or critical level has been reached does not lower it: the beeper keeps going and the critical landing still
  happens. Switched on again, a level is raised at the next battery update ( every 21 ms ) if the battery is already
  low.

## Examples

### Make sure every protection is on ( `plutoInit` )

```cpp
void plutoInit ( void ) {
  Failsafe_enable ( ALL );    // runs once at power-up; all three are already on, so this only states the intent
}
```

### Crash disarm off only while Developer Mode runs ( `onLoopStart`, `onLoopFinish` )

```cpp
void onLoopStart ( void ) {
  Failsafe_disable ( CRASH );
}

void onLoopFinish ( void ) {
  Failsafe_enable ( CRASH );    // the battery protection keeps its state
}
```

### Switch one failsafe back on without touching the others ( `onLoopFinish` )

```cpp
void onLoopStart ( void ) {
  Failsafe_disable ( ALL );                   // bench test: no protection at all
}

void onLoopFinish ( void ) {
  Failsafe_enable ( INFLIGHT_LOW_BATTERY );   // battery protection back on
  Failsafe_enable ( CRASH );                  // crash disarm back on
}
```

## Pitfalls

- **`LOW_BATTERY` does nothing.** Nothing in the firmware reads it any more: its last use, a ground warning, was
  removed when the battery management was rewritten. To switch the battery protection off or on, use
  `INFLIGHT_LOW_BATTERY`, which covers every battery level, on the ground too.
- **`Failsafe_disable ( INFLIGHT_LOW_BATTERY )` also removes the automatic landing.** With it off, the drone flies until
  the battery can no longer hold it in the air, and nothing stops you arming on a flat battery.
- **The setting stays after Developer Mode is off.** Put it back in `onLoopFinish ( )` if the pilot should not keep it.
- **The crash detector acts only in ANGLE mode and not during a flip.** Switching `CRASH` on does not add crash
  detection in other modes.
- **Firmware older than 4.0.0 ( API 2.0.0; development build 1.8.8 ):** `Failsafe_enable ( X )` also switched the other two failsafes off ( see below ). On
  such firmware call `Failsafe_enable ( ALL )` and then `Failsafe_disable ( )` for the ones you do not want.

---

## Changes in 1.8.8

- **Fixed:** `Failsafe_enable ( X )` switched the two failsafes you did not name off ( API test T07 ). For example
  `Failsafe_enable ( CRASH )` also switched off `INFLIGHT_LOW_BATTERY`, so the drone gave no low-battery warning, did not
  land at critical, and could be armed on a flat battery. `Failsafe_enable ( LOW_BATTERY )` or
  `( INFLIGHT_LOW_BATTERY )` switched the crash disarm off. Now only the named failsafe changes, as the header always
  said.
- Unchanged: `Failsafe_enable ( ALL )`, `Failsafe_disable ( )`, and the power-up default ( all on ). Code that never
  calls `Failsafe_enable` behaves exactly as before.
- Documented: `LOW_BATTERY` has no effect.

## File Architecture

| File | Contents |
|------|----------|
| `src/main/API/FC-Config.h` | `failsafe_e`; `Failsafe_enable`, `Failsafe_disable` |
| `src/main/API-Src/FC-Config-Setpoints.cpp` | Their implementations |
| `src/main/API-Src/API-Utils.cpp` | The switches `fsLowBattery`, `fsInFlightLowBattery`, `fsCrash` ( all `true` at power-up ) |
| `src/main/sensors/battery.cpp` | `updateBatteryState ( )`: the step out of `BATTERY_OK` needs `fsInFlightLowBattery` |
| `src/main/flight/failsafe.cpp` | `failsafeOnCrash ( )`: acts only with `fsCrash` on |
