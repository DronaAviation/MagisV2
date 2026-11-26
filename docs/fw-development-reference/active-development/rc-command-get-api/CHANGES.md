# RcCommand_Get API - Changes

[README](README.md) · [TASKS](TASKS.md) · [SCOUT](SCOUT.md) · [INVESTIGATION](INVESTIGATION.md) · [CHANGES](CHANGES.md)

What changed and how. Why is in [INVESTIGATION.md](INVESTIGATION.md).

## Task 1 ( 5 Nov 2026 ): a real array from `RcCommand_Get ( )`

### [RC-Interface.cpp](../../../../src/main/API-Src/RC-Interface.cpp)

- The user's rename `RcCommand_get` -> `RcCommand_Get` kept: it is the B12 link fix.
- New file-static `int16_t rcCommandSnapshot [ 4 ]` ( 8 B RAM when linked ). `RcCommand_Get ( )` fills it on each call
  and returns its address: `[ RC_ROLL / RC_PITCH / RC_YAW ] = ( int16_t ) ( 1500 + rcCommand [ ROLL / PITCH / YAW ] )`,
  `[ RC_THROTTLE ] = rcCommand [ THROTTLE ]`. Was a local array, returned by address, which GCC compiled to
  `return NULL`.
- Banner `Last Modified` 5 Nov 2026 ( set by the user ).

### [RC-Interface.h](../../../../src/main/API/RC-Interface.h) ( comments only )

- The doc blocks of the two `RcCommand_Get` overloads were swapped ( each described the other ). Rewritten: the array
  overload gives each element's value and range, the shared-snapshot lifetime and "never NULL"; the channel overload
  gives its range and a `@warning` that `RC_THROTTLE` reads 2600-3500 ( out of scope ).
- Task 1 review fix: the throttle element is documented as the stick through the throttle curve ( 1100-2000 ), not
  the altitude-hold output, because `userCode ( )` runs before `applyAltHold ( )` in each loop; HEADFREE widens roll /
  pitch to about 860-2140.
- Banner `Last Modified` 5 Nov 2026.

### Not changed

`RcData_Get ( )` ( same local-array bug, out of scope ), `RcCommand_Get ( channel )` code, `RcCommand_Set`.

### Object code ( `RC-Interface.o`, PRIMUS_X2_v1 )

| | Before | After |
|---|---|---|
| `RcCommand_Get ( )` | `movs r0, #0; bx lr` ( NULL ) | 3 x `ldrh / addw #1500 / strh`, throttle `ldrh / strh`, `r0` = `rcCommandSnapshot`; 36 B + 8 B literal pool |

## Build ( task 1, `driver.sh --gate PRIMUS_X2_v1` )

| | |
|---|---|
| text / data / bss ( B ) | 122752 / 1316 / 19728, identical to HEAD `1b43fbe`: no firmware code calls the getter, so `-gc-sections` drops it |
| Flash / RAM | 121.1 KB / 256 KB, 20.5 KB / 40 KB |
| Warning gate | no new `src/` warning; the `rcCommandArray` `-Wreturn-local-addr` entry is fixed ( gate: 109 fixed, 108 before ); `rcDataArray` remains ( out of scope ) |

## Build gate ( task 2, 5 Nov 2026, `PROJECT=AltX driver.sh --gate PRIMUS_X2_v1`, after the task 1 review fix )

| | HEAD ( `1b43fbe` ) | This change |
|---|---|---|
| text / data / bss ( B ) | 122752 / 1316 / 19728 | 122752 / 1316 / 19728 |
| Flash | 121.1 KB / 256 KB | 121.1 KB / 256 KB |
| RAM | 20.5 KB / 40 KB | 20.5 KB / 40 KB |
| Warnings in `RC-Interface.cpp` | `rcDataArray`, `rcCommandArray` ( `-Wreturn-local-addr` ) | `rcDataArray` only ( out of scope ) |

No new `src/` warning; the gate counts 109 fixed baseline entries ( 108 carried from earlier commits plus `rcCommandArray` ).
The getter is not in the ELF ( no firmware caller, `-Wl,-gc-sections` ). Cost when user code calls it ( `nm -S` on
`RC-Interface.o` ): `RcCommand_Get ( )` 0x2c = 44 B flash, `rcCommandSnapshot` 8 B RAM ( `.bss` ).

## Task 4 ( 5 Nov 2026 ): docs

- New [RC_INTERFACE_API_WIKI.md](../../../API/RC_INTERFACE_API_WIKI.md) ( 1.8.3 ): channels, both `RcCommand_Get`
  overloads, `RcData_Get ( channel )`, `RcCommand_Set` summary, signs, ranges, when user code reads what, known gaps
  ( `RcData_Get ( )` NULL, channel throttle 2600-3500 ), three examples, "Changes in 1.8.3".
- New [PIPELINE_UPDATE.md](PIPELINE_UPDATE.md): one `User_Space_API.md` bullet; no flowchart change.

### Spec-to-code check ( inline; the multi-agent `spec-compliance` workflow needs the user's opt-in )

| Wiki claim | Source | Holds |
|---|---|---|
| Array = 1500 + `rcCommand` for roll / pitch / yaw, `rcCommand [ THROTTLE ]` for throttle, static, never NULL | `RC-Interface.cpp` `RcCommand_Get ( )`, objdump ( task 1 ) | yes |
| Equal to `RcCommand_Get ( channel )` | `RC-Interface.cpp:116-124`; log-1 563 / 563 | yes |
| Roll / pitch full stick about 1050 / 1950, half stick about +/-120 | `rcRate8` 90, `rcExpo8` 65 ( `config.cpp:348-349` ), `lookupPitchRollRC` ( task 3 review ) | yes |
| Yaw 1000 / 2000, inverted, no expo | `mw.cpp:364` `* -yaw_control_direction`, default 1 ( `config.cpp:459` ), `rcYawExpo8` 0 ( `config.cpp:353` ) | yes |
| Sign follows the channel for roll / pitch | `mw.cpp:381-382` ( negate when `rcData < midrc` ) | yes |
| Throttle 1100-2000, curve of `rcData [ THROTTLE ]` ( stick, landing, altitude cap, failsafe 1200, user override ) | `mw.cpp:408-411` ( `mincheck`, `lookupThrottleRC` ), min / max throttle 1100 / 2000; log-1 `a3 = d3 - 1` | yes |
| HEADFREE about 860-2140 | `mw.cpp:423-430` rotation ( task 1 review ) | yes |
| Pilot command, not the user override or the hold throttle | `mw.cpp` annexCode 1410, userCode 1416 ( plutoLoop, then `applyUserRcOverride` 1212-1213 ), applyAltHold 1455 | yes |
| `RcData_Get ( channel )` = `rcData`, throttle changed by Set / landing / altitude limit | `RC-Interface.cpp:95-98`, `mw.cpp:385-392`, `applyUserRcOverride` | yes |
| Set clamps 1000-2000, ignores channels above throttle | `RC-Interface.cpp` `RcCommand_Set` | yes |
| Override expiry `max ( 250 ms, 2 x` user loop period `)` | `API-Utils.cpp:196-206`, `USER_RC_HOLD_MIN_US` 250000 | yes |
| Pilot alone at 200 counts off centre | `USER_RC_STICK_TRAVEL` 200 ( `API-Utils.h:59` ), `mw.cpp:1117` | yes |
| `RcData_Get ( )` returns NULL; channel throttle 2600-3500 | objdump ( SCOUT.md ); `RC-Interface.cpp:118`, throttle floor 1100 | yes |

## Task 5 ( 5 Nov 2026 ): topic review fixes ( comments and docs only, no rebuild needed )

- Throttle element: documented as the curve of `rcData [ THROTTLE ]`, normally the stick but the landing throttle
  during a landing, at most 1500 above the altitude limit, 1200 in RC failsafe, and it can show the user's own throttle
  override ( `applyUserRcOverride` writes `rcData`, `mw.cpp:1157-1158`; `mw.cpp:385-392`; `rx.cpp:447` ). Header,
  wiki, PIPELINE_UPDATE.md.
- HEADFREE range 860-2140 ( a 450 / 450 rotation peaks at 636 ), header and wiki.
- Wiki: the localisation note dropped ( `isLocalisationOn` is set only under `OPTIC_FLOW`, off on every target );
  the `onLoopStart` example says it can run again after an RC-link stall.
- TASKS.md Summary: throttle 1100-2000.

## File index

| File | Change |
|---|---|
| `src/main/API-Src/RC-Interface.cpp` | `RcCommand_Get` name; `rcCommandSnapshot`; element values |
| `src/main/API/RC-Interface.h` | both `RcCommand_Get` doc blocks rewritten |
