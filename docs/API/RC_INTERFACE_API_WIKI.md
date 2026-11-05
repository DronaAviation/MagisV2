# MagisV2 RC Interface API Wiki

Reading the pilot's stick commands and overriding them from user code, from `API/RC-Interface.h`:

- `RcCommand_Get` ( two overloads ): the four primary commands ( roll, pitch, yaw, throttle ) the flight controller
  computed from the sticks.
- `RcData_Get` ( two overloads ): the raw channel values, 1000-2000, including the AUX channels.
- `RcCommand_Set` ( two overloads ): override roll, pitch, yaw or throttle.

API version **1.8.3**. See "Changes in 1.8.3".

## Quick Start

```cpp
#include "PlutoPilot.h"

void plutoLoop ( void ) {
    int16_t *rc = RcCommand_Get ( );    // roll, pitch, yaw, throttle
    Monitor_Print ( "R:", ( int ) rc [ RC_ROLL ] );
    Monitor_Print ( " P:", ( int ) rc [ RC_PITCH ] );
    Monitor_Print ( " Y:", ( int ) rc [ RC_YAW ] );
    Monitor_Println ( " T:", ( int ) rc [ RC_THROTTLE ] );
}
```

---

## Channels: `rc_channel_e`

| Name | Index | Channel |
|------|-------|---------|
| `RC_ROLL` | 0 | roll |
| `RC_PITCH` | 1 | pitch |
| `RC_YAW` | 2 | yaw |
| `RC_THROTTLE` | 3 | throttle |
| `RC_AUX1` .. `RC_AUX4` | 4-7 | AUX switches ( arm, flight modes, Developer Mode ) |
| `RC_USER1` .. `RC_USER3` | 8-10 | user channels |

## All four commands: `RcCommand_Get ( )`

```cpp
int16_t *RcCommand_Get ( void );
```

Returns a pointer to four values, indexed by `RC_ROLL`, `RC_PITCH`, `RC_YAW`, `RC_THROTTLE`:

| Element | Value | Range ( default settings ) |
|---------|-------|----------------------------|
| `[ RC_ROLL ]`, `[ RC_PITCH ]` | 1500 + `rcCommand` | 1500 centred, about 1050 / 1950 at full stick; about 860-2140 in HEADFREE |
| `[ RC_YAW ]` | 1500 + `rcCommand` | 1500 centred, 1000 / 2000 at full stick |
| `[ RC_THROTTLE ]` | `rcCommand [ THROTTLE ]` | 1100-2000: `rcData [ THROTTLE ]` through the throttle curve ( with the default curve, that value, never below 1100 ). Normally the throttle stick; see below |

- **Same values as `RcCommand_Get ( channel )`** for roll, pitch and yaw. On the bench the two matched on every one of
  563 log records.
- **Signs.** Roll and pitch keep the sign of the stick channel: a roll or pitch channel above 1500 ( roll right,
  pitch forward on a standard transmitter ) reads **above** 1500. **Yaw is inverted:** a yaw channel above 1500 ( yaw
  right ) reads **below** 1500, because the default `yaw_control_direction` 1 negates the yaw command ( `mw.cpp:364` ). This is the same convention
  `RcCommand_Set` uses, so `RcCommand_Set ( RC_YAW, 1600 )` asks for the same turn as yaw stick left.
- **Stick curve.** Roll and pitch go through the rate and expo curve ( `rcRate8` 90, `rcExpo8` 65 ), so half stick reads
  only about 1500 +/- 120. Yaw has no expo by default ( `rcYawExpo8` 0 ).
- **One shared array.** The values are copied into one static array on every call. Read or copy what you need before
  the next call; do not write to it. The pointer is never NULL.
- **Roll, pitch, yaw: the pilot's sticks, not your override.** They are recomputed from the sticks every loop, and user
  code runs before your own `RcCommand_Set` overrides are mixed in. They show an override only for a value you set
  earlier in the same `plutoLoop` call.
- **Throttle: whatever `rcData [ THROTTLE ]` held.** Normally the stick, but during a landing it is the landing
  throttle, above the altitude limit it is at most 1500, in RC failsafe 1200, and a throttle override from
  `RcCommand_Set` is written into `rcData`, so it can show up here on the next loops.
- **Not the altitude-hold throttle.** Altitude hold writes its throttle after user code has run, so in ALT_HOLD the
  throttle element is not the hover throttle.

## One command: `RcCommand_Get ( channel )`

```cpp
int16_t RcCommand_Get ( rc_channel_e CHANNEL );
```

Returns `1500 + rcCommand [ CHANNEL ]` for `RC_ROLL`, `RC_PITCH` and `RC_YAW` ( ranges and signs as above ), and 0 for
any channel after `RC_THROTTLE`. **For `RC_THROTTLE` it is wrong** ( see Known gaps ): use
`RcCommand_Get ( ) [ RC_THROTTLE ]`.

## Raw channels: `RcData_Get ( channel )`

```cpp
int16_t RcData_Get ( rc_channel_e CHANNEL );
```

Returns `rcData [ CHANNEL ]`, the received channel value, 1000-2000 ( 1500 centred for the sticks ), before any curve.
The throttle can differ from the pilot's stick: a throttle override from `RcCommand_Set`, a landing and the altitude
limit write into `rcData [ THROTTLE ]`.

## Override: `RcCommand_Set`

```cpp
void RcCommand_Set ( rc_channel_e CHANNEL, int16_t rcValue );
void RcCommand_Set ( int16_t *rcValueArray );    // four values: roll, pitch, yaw, throttle
```

- Values are clamped to 1000-2000, 1500 centred for roll, pitch and yaw. Channels after `RC_THROTTLE` are ignored.
- **The override is not sticky:** it is dropped unless it is set again within `max ( 250 ms, 2 x` the user loop
  period `)`, so call it every `plutoLoop` for as long as you want it, and stop calling it to hand the channel back.
- **The pilot keeps authority:** the override is cross-faded against the pilot's sticks by how far they are deflected,
  reaching the pilot alone at 200 counts off centre ( `USER_RC_STICK_TRAVEL`, a little under half stick ).
- The full rules are in the `RC-Interface.h` comments and `FLIGHT_INVARIANTS.md`.

## Known gaps ( not yet fixed, do not rely on them )

- **`RcData_Get ( )` ( no argument ) returns NULL.** Like `RcCommand_Get ( )` before 1.8.3, it returns the address of a
  local array, which the compiler turns into NULL: reading it gives unrelated numbers. Use `RcData_Get ( channel )`.
- **`RcCommand_Get ( RC_THROTTLE )` reads 2600-3500.** It adds 1500 to a throttle that is already 1100-2000. Use
  `RcCommand_Get ( ) [ RC_THROTTLE ]`, or subtract 1500.

## Examples

### Read the four commands when Developer Mode starts ( `onLoopStart` )

`onLoopStart` also runs again after a short RC-link stall ( Wi-Fi ), so this can print more than once.

```cpp
void onLoopStart ( void ) {
    int16_t *rc = RcCommand_Get ( );
    Monitor_Print ( "Start roll:", ( int ) rc [ RC_ROLL ] );
    Monitor_Println ( " thr:", ( int ) rc [ RC_THROTTLE ] );
}
```

### Copy the commands to keep them across calls ( `plutoLoop` )

```cpp
static int16_t lastRc [ 4 ];

void plutoLoop ( void ) {
    int16_t *rc = RcCommand_Get ( );
    for ( int i = 0; i < 4; i++ ) {
        lastRc [ i ] = rc [ i ];    // the array behind rc is refilled on the next call
    }
}
```

### Detect a yaw stick to the right ( `plutoLoop` )

```cpp
void plutoLoop ( void ) {
    int16_t *rc = RcCommand_Get ( );
    bool yawRight = rc [ RC_YAW ] < 1400;    // yaw right reads below 1500
    Monitor_Println ( "Yaw right:", ( int ) yawRight );
}
```

## Changes in 1.8.3

- `int16_t *RcCommand_Get ( void )` links: the source defined it as `RcCommand_get`, so calling it from user code was
  an undefined reference ( API test B12 ).
- It returns real values. Before, it returned the address of a local array, which the compiler replaced with NULL, so
  the array read unrelated numbers whatever the sticks did. It now returns a static array: roll, pitch and yaw as
  1500 + `rcCommand` ( the same as `RcCommand_Get ( channel )` ), throttle as `rcCommand [ THROTTLE ]`.
- The header comments of the two `RcCommand_Get` overloads, which described each other, are corrected.

## File Architecture

| File | Contents |
|------|----------|
| `src/main/API/RC-Interface.h` | `rc_channel_e`; `RcData_Get`, `RcCommand_Get`, `RcCommand_Set`; also `App_getAppHeading`, `App_isArmSwitchOn` ( not covered here ) |
| `src/main/API-Src/RC-Interface.cpp` | Their implementations; `rcCommandSnapshot` |
| `src/main/mw.cpp` | `annexCode ( )`: sticks to `rcCommand` ( curves, signs, HEADFREE ); `userCode ( )`: `plutoLoop`, then `resetUserRCflag ( )` and `applyUserRcOverride ( )` |
| `src/main/io/rc_curves.cpp` | `lookupPitchRollRC`, `lookupYawRC`, `lookupThrottleRC` |
| `src/main/io/rc_controls.cpp` | `rcCommand [ 4 ]` |
| `src/main/rx/rx.cpp` | `rcData [ ]` |
