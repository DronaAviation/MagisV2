# Failsafe_enable API - Pipeline update

[README](README.md) · [TASKS](TASKS.md) · [SCOUT](SCOUT.md) · [INVESTIGATION](INVESTIGATION.md) · [CHANGES](CHANGES.md) · [TESTING](TESTING.md) · [PIPELINE_UPDATE](PIPELINE_UPDATE.md)

Staged text for the pipeline docs. Apply at commit ( `pluto-commit` ); do not edit `fw-architecture-pipeline/` before.

## 1. `fw-architecture-pipeline/subsystems/Failsafe_Subsystem.md`

### 1a. Section "Key Data Structures"

**Insert** after the bullet that ends `` `failsafeActivate ( )`,
  which would apply them, is never called. `` ( currently lines 20-21, the last bullet before "## Primary Functions" ):

```markdown
- `fsLowBattery`, `fsInFlightLowBattery`, `fsCrash` ( `API-Src/API-Utils.cpp:122-124`, `bool`, all `true` at
  power-up, not saved ): the user-code switches, written only by `Failsafe_enable ( )` / `Failsafe_disable ( )`
  ( `API-Src/FC-Config-Setpoints.cpp` ). Each call changes only the named flag, `ALL` all three ( API 1.8.8; before,
  `Failsafe_enable` cleared the two it did not name, API test T07 ). Not reset on disarm or at Developer Mode off.
  `fsCrash` gates `failsafeOnCrash ( )` ( below ); `fsInFlightLowBattery` gates the step out of `BATTERY_OK` in
  `updateBatteryState ( )` ( `battery.cpp:798`, see `Power_BMS_Pipeline.md` ); `fsLowBattery` has no reader since the
  battery-management rewrite ( `88f594f` ). All readers and writers run in the main loop.
```

### 1b. Section "Crash Detector ( `failsafeOnCrash ( )` )", first paragraph

**Replace** the first three lines of the paragraph:

```markdown
`failsafeOnCrash ( )` ( [failsafe.cpp:362](../../../../src/main/flight/failsafe.cpp#L362) ) runs once per control loop
( 3.5 ms, called from [mw.cpp:1536](../../../../src/main/mw.cpp#L1536) ). It is active only while **armed, `fsCrash` on
and in ANGLE mode**. `fsCrash` is on by default; user code can switch it with `Failsafe_enable / disable ( CRASH )`. It
```

**with:**

```markdown
`failsafeOnCrash ( )` ( [failsafe.cpp:363](../../../../src/main/flight/failsafe.cpp#L363) ) runs once per control loop
( 3.5 ms, called from [mw.cpp:1536](../../../../src/main/mw.cpp#L1536) ). It is active only while **armed, `fsCrash` on
and in ANGLE mode** ( the check at `failsafe.cpp:365` ). `fsCrash` is on by default; user code can switch it with
`Failsafe_enable / disable ( CRASH )`, and from API 1.8.8 no other failsafe call changes it ( see Key Data Structures ). It
```

The next line ( `disarms on **one sample** in which any of these terms is true:` ) and the rest of the section stay.
No flowchart change: the `Armed and fsCrash and ANGLE mode?` gate at line 118 is unchanged, and no call path moved.

## 2. `fw-architecture-pipeline/subsystems/Power_BMS_Pipeline.md`, step 7 "Levels"

Not in the original task list; added in task 4 because this doc names the gate without the flag or the inert
`LOW_BATTERY` ( decisions log ). The same applies to section 3.

**Replace** the end of step 7 ( currently lines 87-88 ):

```markdown
   the count supports on disarm and when R is measured. The levels are raised only while the in-flight low-battery
   failsafe is enabled ( the default ).
```

**with:**

```markdown
   the count supports on disarm and when R is measured. The levels are raised only while the in-flight low-battery
   failsafe is enabled ( the default ): `fsInFlightLowBattery` guards the step out of `BATTERY_OK` ( `battery.cpp:798` ),
   on the ground and in flight. It is the only way out of `BATTERY_OK`: the re-level of a provisional alarm runs only
   after a guarded step. Clearing it later does not lower a level. User code switches it with
   `Failsafe_enable / disable ( INFLIGHT_LOW_BATTERY )`; `LOW_BATTERY` has no effect ( see `Failsafe_Subsystem.md` ).
```

## 3. `fw-architecture-pipeline/subsystems/User_Space_API.md`, API feature list

**Insert** after the bullet `` - **Battery**: `Bms_Get` has `SoC`, `Warning_Level` and `Resistance` ( API 1.4.0 ); wiki `docs/API/BMS_API_WIKI.md`. ``
( currently line 84 ):

```markdown
- **Failsafe switches**: `Failsafe_enable / disable ( failsafe_e )` ( `FC-Config.h` ) change only the named switch,
  `ALL` all three ( API 1.8.8 ); all on at power-up, held until reboot, not saved. `INFLIGHT_LOW_BATTERY` gates every
  battery level and the critical auto-land, `CRASH` the crash disarm; `LOW_BATTERY` has no effect. Wiki
  `docs/API/FAILSAFE_API_WIKI.md` ( API 1.8.8 ).
```

## 4. Hardware maps

No change: no pin, DMA channel or timer is touched.

## 5. At commit ( for `pluto-commit` )

- `API_Version` 1.8.7 to 1.8.8 and `FW_Version` 3.15.9 to 3.15.10 in the Makefile.
- `docs/API/FAILSAFE_API_WIKI.md` is new in this topic ( API 1.8.8 ); `docs/API/BMS_API_WIKI.md` gained a note at the
  in-flight failsafe bullet. Both are already in the working tree, not staged here.
- `CHANGELOG.md`: `Failsafe_enable ( X )` no longer switches the other failsafes off ( API test T07 ); `LOW_BATTERY`
  documented as inert; new Failsafe API wiki.
- CLAUDE.md, flight invariants, one line: `Failsafe_enable / disable ( X )` changes only X ( `ALL` all three );
  `INFLIGHT_LOW_BATTERY` gates every battery level and the critical landing; `LOW_BATTERY` does nothing.
