# MagisV2 BMS API Wiki

Battery information for user code: voltage, current, charge used and left, state of charge, the low-battery level and
the pack resistance. Everything comes from `Bms_Get ( option )` in `API/BMS.h`, which returns a `uint16_t`.

API version **1.4.0** ( firmware with battery-soc-fix ). See "Changes in 1.4.0" for what differs from 1.3.x.

## Quick Start

```cpp
#include "PlutoPilot.h"

void plutoLoop ( void ) {
    // Print the charge left and the low-battery level every loop
    Monitor_Print ( "SoC:", Bms_Get ( SoC ) );
    Monitor_Println ( " Level:", Bms_Get ( Warning_Level ) );
}
```

---

## Options

The table describes boards with current sensing ( PRIMUS_X2_v1, PRIMUS_V5 ). For boards without it, see
"Without current sensing" below.

| Option | Unit | What it returns | Valid from |
|--------|------|-----------------|------------|
| `Voltage` | mV | Battery voltage, averaged over ~1 s. Under load it sags by current x pack resistance. | ~0.1 s after power-up |
| `Current` | mA | Battery current, averaged over ~1 s. ~125 mA with the board idle, ~4-5 A in hover. | power-up |
| `mAh_Consumed` | mAh | Charge drawn since power-up. | power-up |
| `mAh_Remain` | mAh | Charge left: the plug-in estimate minus `mAh_Consumed`, lowered by the voltage check near empty. Never rises during a power-up. | ~1 s after power-up |
| `Battery_Capicity` | mAh | The capacity set in the app for the pack in use ( e.g. 600, 800 ). | power-up |
| `Estimated_Capacity` | mAh | Charge in the pack at plug-in, from its resting voltage on the LiPo curve times `Battery_Capicity`. 0 for a pack at or below 3.27 V at rest. | ~1 s after power-up |
| `SoC` | % | State of charge, 0 to 100: `mAh_Remain` / `Battery_Capicity`, rounded. The app shows the same value truncated, so it can read 1 lower. | ~1 s after power-up |
| `Warning_Level` | - | 0 = OK, 1 = low battery, 2 = critical. The same level the app and the beeper use. | power-up |
| `Resistance` | mOhm | Resistance seen by the battery sensor, measured once per power-up: the pack plus its connector, the board wiring and the 20 mOhm shunt, under sustained load, clamped to 80-250. It reads ~25-60 mOhm above a charger's IR figure for the same pack. 0 until measured. | after ~35 s of flight |

`Battery_Capicity` keeps its historical spelling.

### How the low-battery level is decided

- **Low battery** when about 15% of the pack is left, **critical** at about 5%. Either the charge count or the pack
  voltage can trigger it, whichever comes first.
- **The voltage check runs in flight only** ( armed, drawing at least 1.5 A ). It corrects the voltage for the load
  using the pack resistance and compares the result with 3.745 V ( low battery ) and 3.60 V ( critical ) per cell.
  Measured under load in hover that is roughly 3.0-3.1 V and 2.9-2.95 V. On the ground only the charge count can
  raise the level.
- A level must hold for 1.5 s before it is raised, and once raised it **stays until the battery is unplugged**. The
  pack cannot recharge on board, so a confirmed level never goes back down.
- **Before the pack resistance is measured** ( the first ~35 s of flight drawing at least 1.5 A, added up over all
  flights since power-up ), the voltage check only acts once the charge count shows 40% or less left. A level it
  raises that the count does not support is provisional: when you land, or once the resistance is measured, it drops
  back to the level the count supports.
- The level is only raised when the in-flight low-battery failsafe is enabled ( the default ). With it disabled
  ( `Failsafe_disable` ) before any level is raised there is no low-battery level, no beeper and **no auto-land**.

### Low-Battery Auto-Land

When the level reaches **critical while the drone is armed, the firmware lands it**: the throttle is ramped down, the
touchdown is detected and the drone disarms. Your code does not need to call `Command_Land ( )` for this.

- Roll, pitch and yaw stay with the pilot ( or your code ) during the landing; only the throttle is taken over.
- It cannot be cancelled: `Command_TakeOff ( )` and `Command_Flip ( )` are ignored during the battery landing.
- `RcCommand_Set` on the throttle has no effect while any landing runs ( the battery landing, `Command_Land ( )` or a
  signal-loss landing ). Before 1.4.0 it changed the descent of a `Command_Land ( )` in altitude hold.
- `Bms_Get ( Warning_Level )` reads 2 from the moment critical is reached. The app is shown "low battery" until the
  drone has landed, then "critical", and it will not arm again until the battery is changed.
- Critical is reached with about 5% of the pack left. After the landing the drone **does not arm again** until the
  battery is changed.
- In the first ~35 s of flight, before the pack resistance is measured, a critical raised by the voltage alone is
  provisional: the beeper sounds, but the drone does not land and the level can drop back.

### Timing notes

- **Right after power-up** ( ~0.5-1 s ) `SoC`, `mAh_Remain` and `Estimated_Capacity` read 0 while the plug-in voltage
  settles. Wait about 1 s after power-up before using them.
- **Plug the pack in rested.** A pack straight off the charger reads slightly high, one plugged in right after a flight
  reads low; the estimate assumes a rested pack.
- **Set the capacity in the app** to the pack in use. A wrong capacity shifts `mAh_Remain` and `SoC`; the voltage
  check still catches the end of the pack.

### Without current sensing

On boards without current sensing ( the legacy PRIMUSX2, or with current sensing switched off ):

- `Estimated_Capacity` stays 0 on PRIMUSX2; with current sensing only switched off it is still estimated at plug-in.
  With current sensing switched off, `Current`, `mAh_Consumed` and `Resistance` also
  stay 0; on PRIMUSX2 they are measured but play no part in the level.
- `SoC` comes from the voltage on the LiPo curve ( in flight, with 0.65 V added for the typical hover sag ), and
  `mAh_Remain` is that fraction of `Battery_Capicity`. Both follow the voltage: they never rise in flight but can rise
  on the ground as the pack recovers.
- The level comes from the loaded voltage alone, in flight only: low battery at 3.10 V and critical at 3.00 V per cell.

---

## Examples

### Print the pack at the start of the loop ( `onLoopStart` )

```cpp
void onLoopStart ( void ) {
    // One-time read when Developer Mode starts
    Monitor_Print ( "Pack:", Bms_Get ( Estimated_Capacity ) );
    Monitor_Println ( " of ", Bms_Get ( Battery_Capicity ) );
}
```

### Live battery line ( `plutoLoop` )

```cpp
void plutoLoop ( void ) {
    Monitor_Print ( "V:", Bms_Get ( Voltage ) );
    Monitor_Print ( " I:", Bms_Get ( Current ) );
    Monitor_Print ( " Used:", Bms_Get ( mAh_Consumed ) );
    Monitor_Println ( " Left:", Bms_Get ( mAh_Remain ) );
}
```

### React to the low-battery level ( `plutoLoop` )

```cpp
void plutoLoop ( void ) {
    // Level 1: finish up. At level 2 the firmware lands the drone by itself.
    if ( Bms_Get ( Warning_Level ) == 1 ) {
        RGB_SetColorAll ( 255, 80, 0 );    // your own "land soon" signal ( after RGB_Init )
    }
}
```

### Pack resistance from the last flight ( `onLoopStart` )

```cpp
void onLoopStart ( void ) {
    // Read when Developer Mode is entered after a flight. 0 means not measured yet: less than ~35 s
    // of flight at 1.5 A or more since power-up, or no current sensing on this board.
    Monitor_Println ( "R mOhm:", Bms_Get ( Resistance ) );
}
```

---

## Changes in 1.4.0

| Option | 1.3.x | 1.4.0 |
|--------|-------|-------|
| `Voltage` | 0.1 V steps ( e.g. 3700 ) | exact mV ( e.g. 3712 ) |
| `Current` | scaled by an automatic gain that started at 1.0 each power-up and settled at ~0.95 in flight | the measured current, no gain: ~5% higher than before in and after flight, the same before the first flight |
| `mAh_Consumed` | counted from the gain-scaled current in whole-ms steps | counted from the measured current in microsecond steps: ~5% higher than before |
| `Estimated_Capacity` | straight line between 3.0 and 4.2 V | LiPo resting-voltage curve on a ~0.5 s average |
| `mAh_Remain` | could wrap to ~65000 on an empty pack | stops at 0; lowered by the voltage check near empty; ~5% faster fall ( see `mAh_Consumed` ) |
| `SoC`, `Warning_Level`, `Resistance` | not available | new |
| Critical battery | level raised late or not at all; no landing | the firmware lands the drone ( Low-Battery Auto-Land ) |

---

## File Architecture

| File | Role |
|------|------|
| `src/main/API/BMS.h` | Public header: `BMS_Option_e`, `Bms_Get ( )`. |
| `src/main/API-Src/BMS.cpp` | Maps each option to the battery module's values. |
| `src/main/sensors/battery.cpp` | Measurement, charge count, plug-in estimate, resistance, SoC and warnings. |
| `src/main/drivers/ina219.c` | INA219 current / voltage sensor driver. |
