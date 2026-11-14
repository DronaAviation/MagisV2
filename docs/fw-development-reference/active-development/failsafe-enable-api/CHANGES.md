# Failsafe_enable API - Changes

[README](README.md) · [TASKS](TASKS.md) · [SCOUT](SCOUT.md) · [INVESTIGATION](INVESTIGATION.md) · [CHANGES](CHANGES.md)

Every firmware change in this topic, with file, line and reason. Uncommitted until `/pluto-commit`.

## Task 1: correct `Failsafe_enable ( )` ( 15 Nov 2026 )

| File | Line | Change | Why |
|---|---|---|---|
| [API-Src/FC-Config-Setpoints.cpp](../../../../src/main/API-Src/FC-Config-Setpoints.cpp#L213) | 213-236 | The three assignments `fsX = enableAll \|\| ( FAILSAFE == X )` become three `if ( enableAll \|\| ( FAILSAFE == X ) ) { fsX = true; }`. Comments now say "set", note that a flag is never cleared here, cite T07, and note that `fsLowBattery` has no reader. | The assignments wrote `false` to the two flags not named, against the header doc ( FC-Config.h:270-279, "others remain unchanged" ). `Failsafe_enable ( CRASH )` therefore cleared `fsInFlightLowBattery` and with it the low-battery warning, the auto-land at critical and the arm refusal. |

Not changed in task 1, by decision: `Failsafe_disable ( )` ( already correct ), the header `API/FC-Config.h` ( its
`failsafe_e` comments are corrected later, in task 7 ), the flag definitions in `API-Utils.cpp`. No signature change.

Build ( `PROJECT=AltX driver.sh --gate PRIMUS_X2_v1` ): no new `src/` warnings, ALL BUILDS PASSED; flash 121.5 KB / 256 KB
( 47 % ), RAM 20.5 KB / 40 KB ( 51 % ). The gate's "fixed in src/ ( 114 )" list is baseline drift from the earlier
sibling commits on this branch, not from this change.

## Task 7: correct the `failsafe_e` comments ( 15 Nov 2026 )

| File | Line | Change | Why |
|---|---|---|---|
| [API/FC-Config.h](../../../../src/main/API/FC-Config.h#L47) | 47-50 | The four `failsafe_e` value comments rewritten. `LOW_BATTERY`: no effect, use `INFLIGHT_LOW_BATTERY`. `INFLIGHT_LOW_BATTERY`: every battery level, ground and flight, with the auto-land and arm refusal. `CRASH`: armed, ANGLE mode, not in a flip, tilt past 70 deg or hard impact. `ALL`: all three. | The old comments said `LOW_BATTERY` triggered actions ( nothing reads it ), `INFLIGHT_LOW_BATTERY` acted only in flight ( it gates the ground levels too ), and `ALL` could "check" ( there is no check function ). Found in task 4's header check. |
| [API/FC-Config.h](../../../../src/main/API/FC-Config.h#L24) | 24-25 | Two HISTORY rows in the banner. | Banner convention. |

Comment-only: no enumerator, value or signature changed. Gate ( `PROJECT=AltX driver.sh --gate PRIMUS_X2_v1` ): no new
`src/` warnings, ALL BUILDS PASSED; ELF text 123108 / data 1316 / bss 19760, identical to task 3.
