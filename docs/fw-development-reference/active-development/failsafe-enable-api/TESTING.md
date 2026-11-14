# Failsafe_enable API - Testing

[README](README.md) · [TASKS](TASKS.md) · [SCOUT](SCOUT.md) · [INVESTIGATION](INVESTIGATION.md) · [CHANGES](CHANGES.md) · [TESTING](TESTING.md)

No hardware test in this topic ( decision of 15 Nov 2026 ). Evidence is ( a ) a host truth-table check that compiles
the real source file, and ( b ) a source trace from each flag to what it gates. The testing team re-runs T07 on hardware.

## ( a ) Host truth-table check ( task 2, 15 Nov 2026 )

**What is compiled.** The **real** [FC-Config-Setpoints.cpp](../../../../src/main/API-Src/FC-Config-Setpoints.cpp)
and the real `src/main/common/maths.cpp` ( for `constrain` ), built for the PC with MinGW g++ 14.3.0,
`-std=gnu++17 -O1 -Wall -Wextra`. Headers resolve in this order:

1. a **scratchpad overlay** with declarations only, typed as in the firmware:
   - `API/API-Utils.h`: the 12 globals the file uses, with the types of `src/main/API/API-Utils.h`.
   - `io/rc_controls.h`: `controlRateConfig_t`, copied verbatim from `io/rc_controls.h:140`, and the two `CONTROL_RATE_CONFIG_*_MAX` defines.
   - `config/config_master.h`: adds `currentControlRateProfile`.
   - `sensors/sensors.h` ( `heading` ), `mw.h` ( `magHold` ), `flight/altitudehold.h` ( three prototypes ).
2. the in-repo [tools/sim/stub/](../../../../tools/sim/stub) tree from `set-default-pid-profile`, unchanged;
3. the real `src/main` headers, including `API/FC-Config.h` ( `failsafe_e` ) and `common/axis.h`.

The file compiled with **no warnings** under `-Wall -Wextra`. The harness defines the globals with the firmware
defaults ( the three flags `true`, `API-Utils.cpp:122-124` ) and stubs the three altitude functions. No repo file was
added or changed. The harness, runner and overlay are in the session scratchpad,
`C:\Users\MechAsh\AppData\Local\Temp\claude\9df01950423a7843\325f15fd-cf23-43fc-aff4-0a755472f794\scratchpad\fs\`,
which a reboot can clear. They are not committed ( decision of 15 Nov 2026 ).

**What is checked.** A model of the documented contract ( FC-Config.h:270-292 ): `enable ( X )` sets X's flag,
`disable ( X )` clears it, `ALL` means all three, and no other flag changes. The model is compared with the real
functions on:

| Set | Count |
|---|---|
| Every start state ( 8 ) x every single call ( 4 enable + 4 disable ) | 64 |
| Every sequence of three calls from every start state ( 8 x 8^3 ) | 4096 |
| The T07 scenarios, by name: reboot defaults then `enable ( CRASH )`; reboot defaults then `enable ( LOW_BATTERY )`; `disable ( ALL )` then enabling each in turn; `disable ( CRASH )` then `enable ( INFLIGHT_LOW_BATTERY )` | 4 |
| **Total** | **4164** |

**How it was run** ( from the repo root; the pre-fix file is `git show HEAD:src/main/API-Src/FC-Config-Setpoints.cpp`,
HEAD `3be9093`, which still has the old body ):

```
$ bash <scratchpad>/fs/run.sh                                   # working tree, task 1 fix
$ bash <scratchpad>/fs/run.sh <scratchpad>/fs/FC-Config-Setpoints.prefix.cpp   # pre-fix
```

**Fixed file: PASS** ( exit 0 ). Flags shown as `LB IF CR` ( LOW_BATTERY, INFLIGHT_LOW_BATTERY, CRASH ):

```
truth table ( LB IF CR after one call ):
  call                               from 1 1 1 from 0 0 0
  enable ( LOW_BATTERY )             1 1 1      1 0 0
  enable ( INFLIGHT_LOW_BATTERY )    1 1 1      0 1 0
  enable ( CRASH )                   1 1 1      0 0 1
  enable ( ALL )                     1 1 1      1 1 1
  disable ( LOW_BATTERY )            0 1 1      0 0 0
  disable ( INFLIGHT_LOW_BATTERY )   1 0 1      0 0 0
  disable ( CRASH )                  1 1 0      0 0 0
  disable ( ALL )                    0 0 0      0 0 0
PASS: 4164 checks, 0 failed
```

**Pre-fix file: FAIL** ( exit 1 ). This is the T07 report reproduced: from the reboot defaults ( `1 1 1` ),
`enable ( CRASH )` leaves `0 0 1`:

```
truth table ( LB IF CR after one call ):
  call                               from 1 1 1 from 0 0 0
  enable ( LOW_BATTERY )             1 0 0      1 0 0
  enable ( INFLIGHT_LOW_BATTERY )    0 1 0      0 1 0
  enable ( CRASH )                   0 0 1      0 0 1
  enable ( ALL )                     1 1 1      1 1 1
  disable ( LOW_BATTERY )            0 1 1      0 0 0
  disable ( INFLIGHT_LOW_BATTERY )   1 0 1      0 0 0
  disable ( CRASH )                  1 1 0      0 0 0
  disable ( ALL )                    0 0 0      0 0 0
  FAIL  from [1 0 0]  enable ( INFLIGHT_LOW_BATTERY )  -> [0 1 0], expected [1 1 0]
  FAIL  from [1 0 0]  enable ( CRASH )  -> [0 0 1], expected [1 0 1]
  ... ( first 12 of 1468 shown )
FAIL: 1468 of 4164 checks failed
```

The `disable` rows are identical in the two builds: `Failsafe_disable` was already correct and was not changed.

**Result: PASS.** The fixed function meets the header contract on every state and every sequence of up to three
calls, and the same check catches the pre-fix body.

## ( b ) Flag-to-consumer trace ( task 2, 15 Nov 2026 )

Every reader of the three flags, and what is lost when a flag is wrongly cleared. All of them run in the main loop
( task 1 review ): `BMS_Update` every 21 ms from `mw.cpp:458-464`, `failsafeOnCrash ( )` at `mw.cpp:1536` in `loop ( )`.

### `fsInFlightLowBattery`: the only way into the low-battery chain

1. `BMS_Update ( )` ( `mw.cpp:464` ) debounces the warning and critical conditions for 1.5 s and calls
   `updateBatteryState ( warnLow, critLow )` ( [battery.cpp:1046](../../../../src/main/sensors/battery.cpp#L1046) ).
2. In `BATTERY_OK` the step out is guarded by
   `batteryEstimateReady && ( warnLow || critLow ) && fsInFlightLowBattery`
   ( [battery.cpp:798](../../../../src/main/sensors/battery.cpp#L798) ). If it passes: `BATTERY_WARNING`,
   `set_FSI ( Low_battery )`, `BEEPER_BAT_LOW`, `BatteryWarningMode = 1` ( :799-802 ); with `critLow` too, straight
   on to `BATTERY_CRITICAL`, `set_FSI ( LowBattery_inFlight )`, `BEEPER_BAT_CRIT_LOW`, `BatteryWarningMode = 2`
   ( :804-809 ).
3. **That guard is the only way out of `BATTERY_OK`.** `batteryState` is written to WARNING or CRITICAL only at
   :799, :805 and :817 ( :817 is WARNING to CRITICAL, already past the guard ), and in `batteryApplyLevel ( )` ( :863 ).
   `batteryApplyLevel ( )` runs only while `alarmProvisional` is set ( :1000-1005 ), and `alarmProvisional` is only
   raised after a state step at :1055-1056, that is, after the guard has already let one through.
4. Downstream, from `BATTERY_CRITICAL`:
   - `batteryCriticalConfirmed ( )` ( :233-235 ) returns true for a non-provisional critical.
   - That starts `batteryCriticalAutoLand ( )` ( `mw.cpp:1306-1310` ), the landing at critical battery.
   - It also makes `mwArm ( )` refuse to arm ( `mw.cpp:625-626` ) until the pack is changed.

**So, with the pre-fix code, `Failsafe_enable ( CRASH )` cleared this flag, and the battery level stayed `BATTERY_OK`
for the whole flight.** The consequences:

- no `Low_battery` / `LowBattery_inFlight` status for the app;
- no low-battery or critical beeper;
- `BatteryWarningMode` stuck at 0;
- no auto-land at critical;
- arming allowed on a flat pack.

**One-way gate.** Clearing the flag after the level has left `BATTERY_OK` changes nothing: WARNING to CRITICAL at
:816-822 has no guard. A T07 re-run therefore shows the bug only if the API is called before the pack runs low, for
instance in `plutoInit ( )`.

### `fsCrash`: the crash disarm

`failsafeOnCrash ( )` ( [failsafe.cpp:363-384](../../../../src/main/flight/failsafe.cpp#L363) ) acts only when
`ARMING_FLAG ( ARMED ) && fsCrash && FLIGHT_MODE ( ANGLE_MODE )` ( :365 ). It trips on a tilt over 70 deg
( `> 700` deci-degrees ), smoothed X/Y accel over 5000 LSB, or raw over 8000 LSB. Unless a flip is running, it then
calls `set_FSI ( Crash )` and `mwDisarm ( )`. With the pre-fix code, `Failsafe_enable ( LOW_BATTERY )` or
`( INFLIGHT_LOW_BATTERY )` cleared `fsCrash`, so a crash left the motors running. This gate is checked every loop, so
it is not one-way: re-enabling `CRASH` restores it at once.

### `fsLowBattery`: no reader

Nothing in `src/` reads `fsLowBattery`; it is written only by the two API functions. Its last reader, the
`annexCode ( )` ground warning, was removed in `88f594f`. `Failsafe_enable / disable ( LOW_BATTERY )` changes no
flight behaviour. With the fix it no longer has the side effect of clearing the other two flags.

## ( c ) Build gate ( task 3, 15 Nov 2026 )

**Gate.** `PROJECT=AltX .claude/skills/pluto-build/driver.sh --gate PRIMUS_X2_v1`, clean build of the working tree:
`no new warnings in src/`, `ALL BUILDS PASSED`, exit 0. Output: `Build/PRIMUS_X2_v1/AltX_PRIMUS_X2_v1_3.15.9.hex`.
The baseline was not touched.

**Against `HEAD`.** `HEAD` ( `3be9093`, the pre-fix body ) was exported with `git archive HEAD` into the session
scratchpad and clean-built the same way. The working tree and `.git` were not changed.

| ELF ( `PRIMUS_X2_v1` ) | text | data | bss | Flash | RAM |
|---|---|---|---|---|---|
| `HEAD` 3be9093 | 123108 | 1316 | 19760 | 121.5 KB / 256 KB ( 47 % ) | 20.5 KB / 40 KB ( 51 % ) |
| Working tree ( task 1 ) | 123108 | 1316 | 19760 | 121.5 KB / 256 KB ( 47 % ) | 20.5 KB / 40 KB ( 51 % ) |
| **Delta** | **0** | **0** | **0** | | |

**Why the delta is 0.** Neither `Failsafe_enable` nor `Failsafe_disable` is in either ELF. Nothing in the firmware or
the default `PlutoPilot.cpp` calls them ( task 1 review ), so `-Wl,-gc-sections` removes them. The fix reaches a
binary only when user code calls `Failsafe_enable ( )`. That is also when the bug did.

**Generated code.** The object file `Build/PRIMUS_X2_v1/bin/API-Src/FC-Config-Setpoints.o` from each build
( `arm-none-eabi-objdump -d` ):

| | `Failsafe_enable` | Stores |
|---|---|---|
| `HEAD` | 60 B | writes literal zeros ( `movs r3, #0` / `movs r0, #0`, then `strb` ): the clears behind T07 |
| Working tree | 56 B | every `strb` stores 1 ( `r3` or `r2` loaded with `#1`, or `r0` on the branch where `r0 == 1` ). Three stores for `ALL` ( 3 ), one for 0 / 1 / 2, none for any other value |
| `Failsafe_disable` | 64 B in both | unchanged, as decided |

With a caller, the fix is 4 B smaller in flash. No RAM change: the flags already existed.

**Result: PASS.**
