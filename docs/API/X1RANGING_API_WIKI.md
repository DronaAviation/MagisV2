# MagisV2 X1Ranging API Wiki

Laser ranging for user code on a drone whose lasers are **VL53L1X**: the downward altitude laser and up to five side
lasers, their ranges and health, and object avoidance. Everything is in `API/X1Ranging.h` through the `X1Ranging`
object. It works like [XRanging](XRANGING_API_WIKI.md) ( VL53L0X ): the same laser names, the same status values, the
same pins and the same object avoidance.

API version **1.8.0** ( firmware 3.15.0, vl53l1x-xranging-api ).

## Choosing the laser family

There is no build switch: the same firmware drives either kind of laser. **Your `plutoInit ( )` chooses** by the API
it asks for lasers. A call counts when it enables a laser: `init`, `initObjectAvoidance` with a side laser, or an
accepted `initAlt`. Reading calls ( `getRange`, `getStatus`, `isTriggered` ) and `enableOA` / `disableOA` do not count.

| `plutoInit ( )` enables lasers through | The firmware starts | Altitude hold with `initAlt` |
|---|---|---|
| only `X1Ranging` | the VL53L1X lasers | VL53L1X laser fusion |
| only `XRanging` | the VL53L0X lasers | VL53L0X laser fusion |
| neither | no laser | barometer |
| **both** | **no laser: both are refused** | barometer |

The choice is made once, right after `plutoInit ( )`. When both APIs are used, every call's request is dropped at
start-up, even an `initAlt` that returned `true`:

- `getRange` returns −1 and `getStatus` `LASER_NOT_ENABLED` on both;
- `isTriggered` returns `false`, and object avoidance never runs;
- SPI2 and the flash stay on.

VL53L0X and VL53L1X cannot be mixed on one drone, because both start at I2C address 0x29.

`LASER_TOF_L1x` and `LASER_ALT` no longer exist: defining either in `target.h` is a build error.

## Quick Start

```cpp
#include "PlutoPilot.h"

void plutoInit ( void ) {
    X1Ranging.initAlt ( GPIO_5 );                                      // down-laser + altitude hold, XSHUT on GPIO_5
    X1Ranging.initObjectAvoidance ( 50, LEFT, RIGHT, FRONT, BACK );   // side lasers, avoid within 50 cm
}

void onLoopStart ( void ) {
    X1Ranging.enableOA ( );       // avoidance only while Developer Mode is on
}

void plutoLoop ( void ) {
    Monitor_Print ( "Down:", X1Ranging.getRange ( DOWN ) );           // mm
    Monitor_Println ( " Front:", X1Ranging.getRange ( FRONT ) );      // mm
}

void onLoopFinish ( void ) {
    X1Ranging.disableOA ( );
}
```

---

## The lasers

| `laser_e` | XSHUT pin | Status LED | I2C address after start-up |
|---|---|---|---|
| `LEFT` | `GPIO_7` | `GPIO_6` | 0x2A + n |
| `RIGHT` | `GPIO_9` | `GPIO_10` | 0x2A + n |
| `FRONT` | `GPIO_16` | `GPIO_5` | 0x2A + n |
| `BACK` | `GPIO_15` | `GPIO_8` | 0x2A + n |
| `EXTERNAL` | `GPIO_14` | - | 0x2A + n |
| `DOWN` | the pin given to `initAlt ( )`, or none | - | 0x30 |

`n` counts the side lasers enabled before it, in the table's order. The firmware holds every enabled laser in reset
and brings them up one at a time, `DOWN` first.

Every VL53L1X ranges in **Medium** mode with a 45 ms timing budget and **a new sample every 50 ms**. A valid range is
15 mm up to 4.5 m; indoors Medium mode usually reaches about 2.9 m, and less on dark or glossy surfaces.

## Functions

All `init…` calls only **record** what you want. The lasers are started once, after `plutoInit ( )` returns, and the
choice is fixed for the rest of the run. Call them from `plutoInit ( )`.

| Function | What it does |
|---|---|
| `bool initAlt ( peripheral_gpio_pin_e xshutGpio )` | Uses the down-laser for altitude hold ( the laser and the laser / baro fusion together ). `xshutGpio` is the pin wired to the down-laser's XSHUT. Returns `false` if refused, see below. |
| `bool initAlt ( void )` | The same, for a down-laser with **no** XSHUT wire. The firmware moves it from 0x29 to 0x30 before the side lasers start. |
| `void init ( void )` | Enables `LEFT`, `RIGHT`, `FRONT`, `BACK`, each with a 20 cm trigger threshold. |
| `void init ( laser_e laser, int16_t threshold = 20 )` | Enables one side laser with a trigger threshold in **cm**. `DOWN` is ignored here: use `initAlt`. |
| `void initObjectAvoidance ( uint16_t avoidDist, laser_e l1, … l4 )` | Enables up to four side lasers for object avoidance, which reacts within `avoidDist` **cm**. Unused arguments default to `DOWN`, which is ignored. **Avoidance acts on `LEFT`, `RIGHT`, `FRONT` and `BACK` only:** `EXTERNAL` is started but never avoided. |
| `void enableOA ( void )` / `void disableOA ( void )` | Turns object avoidance on or off. `disableOA` also switches the side lasers' LEDs off. |
| `int16_t getRange ( laser_e laser )` | The laser's latest range in **mm**. `-100` while it has no valid sample ( out of range, stale, failed ); `-1` if that laser was not enabled. |
| `bool isTriggered ( laser_e laser )` | `true` if a side laser's range is below its trigger threshold, and it drives that laser's LED. While the laser has no valid sample it returns `false` and leaves the LED as it was. |
| `laser_status_e getStatus ( laser_e laser )` | The laser's health, see below. |

`getRange`, `isTriggered` and object avoidance return **cached** values and never wait on the I2C bus. The firmware
reads the lasers itself:

- **DOWN:** checked every 10 ms; a new sample is read when one is ready ( one every 50 ms ).
- **Side lasers:** one laser per 10 ms, in turn, never in the same loop as the DOWN read. With four or five fitted, each
  is updated about every 50 ms on average ( the reads are paced, not guaranteed ).

## Laser health: `getStatus ( )`

| `laser_status_e` | Meaning |
|---|---|
| `LASER_OK` | Ranging; the last sample is valid. A side laser also reads OK for up to 160 ms after start-up, before its first sample, while `getRange` returns -100. |
| `LASER_OUT_OF_RANGE` | Ranging, but the last sample had no valid range ( nothing in reach, under 15 mm, or no new sample for 160 ms ). Normal for a side laser facing open space. `DOWN` reads this from start-up until its first sample. |
| `LASER_NOT_ENABLED` | Not requested, refused ( including when both APIs are used ), or not started yet. |
| `LASER_BOOT_FAILED` | Did not answer or failed its start-up, including another chip already answering at 0x29. Not retried. |
| `LASER_I2C_ERROR` | Stopped answering after a good start. **One bus error takes that laser out until the next restart.** The other lasers keep working. |

## Altitude hold with the down-laser

- **Without `initAlt`, the firmware holds altitude on the barometer alone, exactly as a build without a laser.**
- With `initAlt`, altitude hold uses the down-laser below **190 cm** and the barometer above it ( the VL53L0X: 160 cm ). The handover moves the
  whole altitude frame, so the craft does not jump. A sudden change of the surface underneath ( a table, an object ) is
  held on the barometer for 2.5 s before the frame is re-based. Details:
  `fw-architecture-pipeline/subsystems/Altitude_Hold_Estimator.md`.
- **Going back from the barometer to the laser** happens below 170 cm after a climb, and below 190 cm after a laser
  dropout, once 3 good samples in a row have arrived. In the air, the VL53L1X has one extra check that the VL53L0X does not:
  - a laser sample counts for the return only if it is within **50 cm** of the barometer estimate;
  - if the two disagree by a steady amount ( within 25 cm ) for **2.5 s**, the laser's reading is taken as a new floor
    and the return goes ahead.

  This keeps ceiling-fan blades and passing objects from pulling the height.
- If the down-laser fails to start after an accepted `initAlt`, altitude hold still uses the laser tuning and flies
  on the barometer as for a laser dropout. `getStatus ( DOWN )` tells you.
- Only `DOWN` ever reaches altitude hold. A hand or a wall in front of a side laser does not move the height.

## When `initAlt` is refused

`initAlt` returns `false` and changes nothing if:

- it is called after `plutoInit ( )`;
- the pin is a side laser's XSHUT ( `GPIO_7`, `9`, `14`, `15`, `16` ), enabled or not;
- the pin is an RGB-LED data slot or a UART2 pin ( `GPIO_1`, `3`, `6`, `10`, `11`, `12`, `13`, `16`, `17`, `18` ): those
  drivers can take the pin over after start-up, and a timer or UART line on an XSHUT resets the laser. That leaves
  `GPIO_2`, `4`, `5` and `8`, subject to the next rule;
- the pin's ADC or PWM alias is in use ( e.g. `GPIO_5` = `ADC_2` / `PWM_4`, `GPIO_2` = `ADC_1` ).

A refused call keeps any earlier accepted request. The pin is checked again after `plutoInit ( )`: an ADC or PWM
enabled **after** `initAlt`, or an `XRanging` call in `plutoInit ( )` that enables a laser ( `init`,
`initObjectAvoidance` with a side laser, an accepted `initAlt` ), also cancels it. `getStatus ( DOWN )` then reads
`LASER_NOT_ENABLED`.

## Pins: what X1Ranging takes over

- **The down-laser's XSHUT pin is reserved after start-up.** `Peripheral_Init` / `Peripheral_Write` on it, and on its
  ADC / PWM alias, are ignored, because a LOW would reset the altitude laser in flight.
- **The down-laser XSHUT replaces an LED:** with `initAlt ( GPIO_5 )` `FRONT` has no LED, and with `initAlt ( GPIO_8 )`
  `BACK` has none.
- **The side lasers' XSHUT pins are not reserved.** Do not drive `GPIO_7`, `9`, `14`, `15` or `16` from user code while
  that side laser is enabled: a LOW resets it, and it then reads out of range or fails.
- **SPI2 is switched off whenever X1Ranging is used** ( at least one laser still requested after start-up ).
  `GPIO_5`-`GPIO_8` are SPI2's pins. After start-up every user SPI call silently does nothing ( `SPI_Read` returns 0 ), and
  the on-board flash reports size 0. SPI calls made inside `plutoInit ( )` still work.
- `GPIO_8` ( `BACK`'s LED ) is also the flash chip-select: a board-level overlap, as for XRanging.

## Wiring rules

- **Enable every laser that is fitted.** A side laser that is fitted but not enabled is never held in reset, so it
  stays awake at 0x29. The firmware then refuses to start the other lasers rather than mix two chips up
  ( `LASER_BOOT_FAILED` ), and that includes the down-laser.
- Without an XSHUT wire, the down-laser keeps its 0x30 address over a reflash or reset that does not cut power. The
  firmware looks for it there first.
- **Without an XSHUT wire ( `initAlt ( )` ), a fitted but disabled side laser cannot be detected**: the down-laser sits
  at 0x29 itself. Enable every fitted laser.

## Object avoidance

The same code as XRanging's object avoidance:

- It runs every 100 ms while enabled, on `LEFT`, `RIGHT`, `FRONT` and `BACK`.
- A side laser closer than `avoidDist` pushes the craft away for 0.5 s, then brakes for 0.5 s. While the obstacle stays
  in range, the push and brake repeat.
- **While OA is enabled, the pilot's roll / pitch stick is scaled to 30 %** ( 20 % while braking ).
- A side laser that is not working counts as "no obstacle" at once. One that reads out of range keeps its last distance
  for two more 100 ms ticks, then counts as "no obstacle". So a short dropout at a real obstacle keeps the push, and a
  removed obstacle is released within about 0.3 s.
- While OA is enabled it rewrites the side lasers' LEDs every 100 ms, so it overrides what `isTriggered` set.
- With ~50 ms between samples per side laser, a fast approach is seen later than with a VL53L0X ( ~40 ms ). Approach
  slowly.

## Examples

### Altitude hold with a wired down-laser ( `plutoInit` )

```cpp
void plutoInit ( void ) {
    if ( ! X1Ranging.initAlt ( GPIO_5 ) ) {
        // refused: altitude hold stays on the barometer
    }
}
```

### Health line ( `plutoLoop` )

```cpp
void plutoLoop ( void ) {
    Monitor_Print ( "D:", ( int ) X1Ranging.getStatus ( DOWN ) );
    Monitor_Println ( " F:", ( int ) X1Ranging.getStatus ( FRONT ) );
}
```

### A side laser as a switch ( `plutoLoop` )

```cpp
void plutoInit ( void ) {
    X1Ranging.init ( FRONT, 30 );    // trigger below 30 cm
}

void plutoLoop ( void ) {
    if ( X1Ranging.isTriggered ( FRONT ) ) {
        // something within 30 cm in front
    }
}
```

### Moving from XRanging

On a drone whose lasers were changed from VL53L0X to VL53L1X, rename `XRanging` to `X1Ranging` in `PlutoPilot.cpp`.
The laser names, pins, thresholds and return values stay the same. Do not leave any `XRanging` call behind: using both
APIs refuses both.

## Changes in 1.8.0

| Item | 1.7.x | 1.8.0 |
|---|---|---|
| VL53L1X | one down-laser only, fixed at 0x29, chosen at build time with `LASER_TOF_L1x` + `LASER_ALT`; no user API, no side lasers | `X1Ranging`, chosen at run time by calling it: down-laser by `initAlt`, up to five side lasers, `getStatus`, object avoidance |
| `LASER_TOF_L1x` / `LASER_ALT` in `target.h` | build switches | build errors |
| Using `XRanging` and the VL53L1X together | allowed ( VL53L0X side lasers beside the VL53L1X down-laser ) | `XRanging` + `X1Ranging` in one `plutoInit`: both refused |
| Object avoidance, out of range ( both APIs ) | kept the last distance until a valid sample came | released after 3 out-of-range ticks ( ~0.3 s ); the first sample after a release is used as-is |

## File Architecture

| File | Role |
|---|---|
| `src/main/API/X1Ranging.h` | Public header: `X1Ranging_P`, using `laser_e` / `laser_status_e` from `XRanging.h`. |
| `src/main/API-Src/X1Ranging.cpp` | Requests, pin checks ( XRanging's table ), the single start-up ( `x1RangingInit ( )` ). |
| `src/main/API-Src/XRanging.cpp` | The laser-family choice made after `plutoInit ( )` ( `rangingFamilyLatch ( )` ). |
| `src/main/drivers/tof_manager_l1x.cpp` | One VL53L1X entry per laser: start-up, polling, cached results, status. |
| `src/main/drivers/tof_dev_area.cpp` | One RAM area for the VL53L0X and VL53L1X device data; only the chosen family uses it. |
| `src/main/API-Src/ObjectAvoidanceCore.h` | The object-avoidance loop shared with XRanging. |
| `src/main/flight/altitudehold.cpp` | The laser / baro fusion, compiled once per laser family; `initAlt` picks the copy at start-up. |
