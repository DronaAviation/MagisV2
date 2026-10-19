# MagisV2 XRanging API Wiki

Laser ranging for user code: the downward altitude laser and up to five side lasers ( VL53L0X ), their ranges and
health, and object avoidance. Everything is in `API/XRanging.h` through the `XRanging` object.

API version **1.8.0** ( firmware 3.15.0 ). See "Changes in 1.8.0" and "Changes in 1.5.0".

**VL53L1X lasers use [X1Ranging](X1RANGING_API_WIKI.md)** instead. The same firmware drives either kind: `plutoInit ( )`
chooses by the API it enables lasers through ( `init`, `initObjectAvoidance` with a side laser, an accepted `initAlt` ).
**If it enables lasers through both XRanging and X1Ranging, both are refused** after `plutoInit ( )`: no laser
starts, `getRange` returns −1, `getStatus` `LASER_NOT_ENABLED`, object avoidance never runs, and altitude hold uses the
barometer ( see the X1Ranging wiki, "Choosing the laser family" ).

## Quick Start

```cpp
#include "PlutoPilot.h"

void plutoInit ( void ) {
    XRanging.initAlt ( GPIO_5 );                                      // down-laser + altitude hold, XSHUT on GPIO_5
    XRanging.initObjectAvoidance ( 50, LEFT, RIGHT, FRONT, BACK );   // side lasers, avoid within 50 cm
    XRanging.enableOA ( );
}

void plutoLoop ( void ) {
    Monitor_Print ( "Down:", XRanging.getRange ( DOWN ) );           // mm
    Monitor_Println ( " Front:", XRanging.getRange ( FRONT ) );      // mm
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

`n` counts the side lasers enabled before it, in the table's order. Every VL53L0X starts at 0x29. The firmware holds
every **enabled** laser in reset and brings them up one at a time, so 0x29 is empty once start-up is done ( a fitted
laser that is not enabled is not held: see "Wiring rules" ).

## Functions

All `init…` calls only **record** what you want. The lasers are started once, after `plutoInit ( )` returns, and the
choice is fixed for the rest of the run. Call them from `plutoInit ( )`.

| Function | What it does |
|---|---|
| `bool initAlt ( peripheral_gpio_pin_e xshutGpio )` | Uses the down-laser for altitude hold ( the laser and the laser / baro fusion together ). `xshutGpio` is the pin wired to the down-laser's XSHUT. Returns `false` if refused, see below. |
| `bool initAlt ( void )` | The same, for a down-laser with **no** XSHUT wire. The firmware moves it from 0x29 to 0x30 before the side lasers start. |
| `void init ( void )` | Enables `LEFT`, `RIGHT`, `FRONT`, `BACK`, each with a 20 cm trigger threshold. |
| `void init ( laser_e laser, int16_t threshold = 20 )` | Enables one side laser with a trigger threshold in **cm**. `DOWN` is ignored here: use `initAlt`. |
| `void initObjectAvoidance ( uint16_t avoidDist, laser_e l1, … l4 )` | Enables up to four side lasers for object avoidance, which reacts within `avoidDist` **cm**. |
| `void enableOA ( void )` / `void disableOA ( void )` | Turns object avoidance on or off. `disableOA` also switches the side lasers' LEDs off. |
| `int16_t getRange ( laser_e laser )` | The laser's latest range in **mm**. `-100` while it has no valid sample ( out of range, failed ); `-1` if that laser was not enabled. |
| `bool isTriggered ( laser_e laser )` | `true` if a side laser's range is below its trigger threshold. It also drives that laser's LED. |
| `laser_status_e getStatus ( laser_e laser )` | The laser's health, see below. |

`getRange`, `isTriggered` and object avoidance return **cached** values and never wait on the I2C bus. The firmware
reads the lasers itself:
- **DOWN:** every 33 ms;
- **side lasers:** one per 10 ms in turn, so each is updated about every 40 ms with four fitted ( measured 23-26 Hz each; the reads are paced, not guaranteed, and can come later when the loop is busy ).

## Laser health: `getStatus ( )`

| `laser_status_e` | Meaning |
|---|---|
| `LASER_OK` | Ranging; the last sample is valid. Also reported just after start-up, before the first sample, while `getRange` still returns -100. |
| `LASER_OUT_OF_RANGE` | Ranging, but the last sample had no valid range ( nothing in reach, or too far ). Normal for a side laser facing open space. |
| `LASER_NOT_ENABLED` | Not requested, refused, or not started yet. |
| `LASER_BOOT_FAILED` | Did not answer or failed its start-up, including another chip already answering at 0x29. Not retried. |
| `LASER_I2C_ERROR` | Stopped answering after a good start. **One bus error takes that laser out until the next power-up.** The other lasers keep working. |

## Altitude hold with the down-laser

- **Without `initAlt`, the firmware holds altitude on the barometer alone, exactly as a build without a laser.**
- With `initAlt`, altitude hold uses the down-laser below 160 cm and the barometer above it ( back below 140 cm ).
  The handover moves the whole altitude frame, so the craft does not jump. Details:
  `fw-architecture-pipeline/subsystems/Altitude_Hold_Estimator.md`.
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
- the pin's ADC or PWM alias is in use ( e.g. `GPIO_5` = `ADC_2` / `PWM_4`, `GPIO_2` = `ADC_1` );

A refused call keeps any earlier accepted request. The pin is checked again after `plutoInit ( )`, so an ADC or PWM
enabled **after** `initAlt`, or an `X1Ranging` call in `plutoInit ( )` that enables a laser ( `init`,
`initObjectAvoidance` with a side laser, an accepted `initAlt` ), also cancels it ( `getStatus ( DOWN )` then reads
`LASER_NOT_ENABLED` ).

## Pins: what XRanging takes over

- **The down-laser's XSHUT pin is reserved after start-up.** `Peripheral_Init` / `Peripheral_Write` on it, and on its
  ADC / PWM alias, are ignored, because a LOW would reset the altitude laser in flight. With `initAlt ( GPIO_5 )`,
  `FRONT` has no LED ( `GPIO_5` is its LED pin ).
- **SPI2 is switched off whenever XRanging is used**: when at least one laser is still requested after start-up ( an accepted `initAlt` that passed its start-up check, or any side laser from `init` / `initObjectAvoidance` ). A refused `initAlt` alone does not switch it off.
  `GPIO_5`-`GPIO_8` are SPI2's pins. After start-up:
  - every user SPI call ( `SPI_Init`, `SPI_Read`, `SPI_Write`, `SPI_Enable`, `SPI_Disable` ) silently does nothing,
    and `SPI_Read` returns 0;
  - the on-board flash reports size 0.

  SPI calls made inside `plutoInit ( )` still work. Pins you set up in `plutoInit ( )` as RGB, PWM, ADC or GPIO keep
  their setup.
- `GPIO_8` ( `BACK`'s LED ) is also the flash chip-select. Driving it LOW selects the flash chip, which can then drive
  `GPIO_6` against `LEFT`'s LED. This is a board-level overlap.

## Wiring rules

- **Enable every laser that is fitted.** A side laser that is fitted but not enabled is never held in reset, so it
  stays awake at 0x29. The firmware then refuses to start the other lasers rather than mix two chips up
  ( `LASER_BOOT_FAILED` ), and that includes the down-laser.
- **A down-laser XSHUT wired to `GPIO_5` needs this firmware with `initAlt ( GPIO_5 )`.** Older firmware drives that
  pin as SPI2 data, which holds the laser in reset.
- Without an XSHUT wire, the down-laser keeps its 0x30 address over a reflash or reset that does not cut power. The
  firmware looks for it there first.
- **Without an XSHUT wire ( `initAlt ( )` ), a fitted but disabled side laser cannot be detected:** the down-laser
  sits at 0x29 itself, so both chips are moved to 0x30 together and their readings mix. With a wire, the firmware
  refuses instead ( `LASER_BOOT_FAILED` ). Enable every fitted laser.

## Object avoidance

- It runs every 100 ms while enabled.
- A side laser closer than `avoidDist` pushes the craft away for 0.5 s, then brakes for 0.5 s.
- **While OA is enabled, the pilot's roll / pitch stick is scaled to 30 %** ( 20 % while braking ). The craft drifts
  more and is harder to correct by hand. This was measured as larger height swings over uneven floors.
- A side laser that is not working counts as "no obstacle" at once. One that reads out of range keeps its last
  distance for two more 100 ms ticks, then counts as "no obstacle": a removed obstacle is released within about
  0.3 s ( since 1.8.0; before, the last near distance was kept until a valid sample came, and the push / brake
  repeated ).

## Examples

### Altitude hold with a wired down-laser ( `plutoInit` )

```cpp
void plutoInit ( void ) {
    if ( ! XRanging.initAlt ( GPIO_5 ) ) {
        // refused: altitude hold stays on the barometer
    }
}
```

### Health line ( `plutoLoop` )

```cpp
void plutoLoop ( void ) {
    Monitor_Print ( "D:", ( int ) XRanging.getStatus ( DOWN ) );
    Monitor_Println ( " F:", ( int ) XRanging.getStatus ( FRONT ) );
}
```

### A side laser as a switch ( `plutoLoop` )

```cpp
void plutoInit ( void ) {
    XRanging.init ( FRONT, 30 );    // trigger below 30 cm
}

void plutoLoop ( void ) {
    if ( XRanging.isTriggered ( FRONT ) ) {
        // something within 30 cm in front
    }
}
```

## Changes in 1.8.0

| Item | 1.7.x | 1.8.0 |
|---|---|---|
| VL53L1X lasers | one down-laser via `LASER_TOF_L1x` + `LASER_ALT`; XRanging could run VL53L0X side lasers beside it | `X1Ranging` ( its own wiki ), chosen at run time; XRanging + X1Ranging in one `plutoInit`: both refused |
| `LASER_TOF_L1x` / `LASER_ALT` in `target.h` | VL53L1X build switches | build errors |
| Object avoidance, out of range | the last distance was kept until a valid sample came | released after 3 out-of-range ticks ( ~0.3 s ); the first sample after a release is used as-is ( no false push from a quartered first reading ) |
| `initObjectAvoidance` unused arguments | default `( laser_e ) -1` | default `DOWN` ( ignored the same way ) |
| Object avoidance code | in `XRanging.cpp` | shared with X1Ranging ( `API-Src/ObjectAvoidanceCore.h` ), the same push / brake |

## Changes in 1.5.0

| Item | 1.4.x | 1.5.0 |
|---|---|---|
| Down-laser | build switch `LASER_TOF` + `LASER_ALT` in `target.h`; could not run with the side lasers ( every laser stopped ) | `initAlt ( pin )` / `initAlt ( )` at run time; runs with all side lasers |
| `laser_e` | `LEFT` … `EXTERNAL` | adds `DOWN` |
| `getStatus ( )`, `laser_status_e` | - | new |
| `getRange`, `isTriggered`, OA | started a blocking read on every call ( ~1 ms each, 4-6 ms per OA pass ) | return the cached value, no bus wait |
| One laser's error | stopped every laser for good | stops only that laser |
| SPI2 / flash | always on | off whenever XRanging is used |
| `LASER_TOF` in `target.h` | supported | build error: use `initAlt` |

## File Architecture

| File | Role |
|---|---|
| `src/main/API/XRanging.h` | Public header: `laser_e`, `laser_status_e`, `XRanging_P`. |
| `src/main/API-Src/XRanging.cpp` | Requests, pin checks, the single start-up ( `xRangingInit ( )` ), the object-avoidance wrapper. |
| `src/main/API-Src/ObjectAvoidanceCore.h` | The object-avoidance loop, shared with X1Ranging. |
| `src/main/drivers/tof_dev_area.cpp` | One RAM area for the VL53L0X and VL53L1X device data; only the chosen family uses it. |
| `src/main/drivers/tof_manager.cpp` | One table entry per laser: start-up, polling, cached results, status. |
| `src/main/drivers/ranging_vl53l0x.cpp` | VL53L0X helpers: configure, address, probe, read. |
| `src/main/drivers/bus_spi.c`, `flash_m25p16.cpp` | SPI2 and flash release when XRanging is used. |
| `src/main/flight/altitudehold.cpp` | The laser / baro fusion, switched on by `initAlt` at start-up. |
