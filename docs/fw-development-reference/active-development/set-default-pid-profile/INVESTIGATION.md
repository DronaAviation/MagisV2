# Set default PID profile - investigation

[README](README.md) · [TASKS](TASKS.md) · [SCOUT](SCOUT.md)

## Problem

API test **T06 ( B1 )**, user's external test sheet, build B1 on `PRIMUS_X2_v1`:

| Read back after `setDefault_PIDProfile ( )` | Expected | Observed |
|---|---|---|
| `PID_ALT` P / I / D | 128 / 0 / 30 | **100** / 0 / 30 |
| `PID_USER` P / I / D | 60 / 30 / 5 | **0 / 0 / 40** |

The user's note with the result: *"Reads past its table; hidden gains corrupted."*

## Evidence

The reads come back through `get_PIDProfile ( )`, whose index map
`{ ROLL, PITCH, YAW, PIDALT, PIDUSER }` is correct, so the wrong numbers are written by
`setDefault_PIDProfile ( )` itself.

[`src/main/API-Src/FC-Config-PID.cpp:147-168`](../../../../src/main/API-Src/FC-Config-PID.cpp#L147) holds three
five-entry tables and a loop over eleven indexes:

```c
static const int defaultP8 [] = { 40, 40, 150, 100, 0 };
static const int defaultI8 [] = { 10, 10,  70,   0, 0 };
static const int defaultD8 [] = { 30, 30,  50,  30, 0 };

for ( int i = ROLL; i <= PIDUSER; ++i ) {       // 0 .. 10
    currentProfile->pidProfile.P8 [ i ] = defaultP8 [ i ];
    ...
}
```

`pidIndex_e` in [`flight/pid.h:30-43`](../../../../src/main/flight/pid.h#L30) runs
`PIDROLL, PIDPITCH, PIDYAW, PIDALT, PIDPOS, PIDPOSR, PIDNAVR, PIDLEVEL, PIDMAG, PIDVEL, PIDUSER`, so `PIDUSER` is 10,
not 4. Two separate defects follow:

1. **Stale ALT value.** Table index 3 is `PIDALT` and holds P8 = 100. The firmware default moved to 128 at
   [`config.cpp:191`](../../../../src/main/config/config.cpp#L191) ( "P8 = 128 is unity: a 10 cm error asks for 10 cm/s" ),
   and the API's private copy was never updated. This alone explains the observed 100.
2. **Read past the tables.** Indexes 4 .. 10 read outside `defaultP8 / I8 / D8`. That is undefined behaviour, so what
   lands in the hidden slots is whatever the compiler's chosen layout and optimisation produce, which is why `PIDUSER`
   came back as 0 / 0 / 40 rather than anything derivable from the tables. The slots written are `PIDPOS`, `PIDPOSR`,
   `PIDNAVR`, `PIDLEVEL`, `PIDMAG`, `PIDVEL` and `PIDUSER`.

The compiler already reports it, and the warnings are baselined:

- `tools/warnings-baseline.json` : `FC-Config-PID.cpp -Waggressive-loop-optimizations iteration 5 invokes undefined behavior` ( 1 )
- `tools/warnings-baseline.json` : `FC-Config-PID.cpp -Wconversion conversion from 'int' to 'uint8_t' may change value` ( 3 )

## Why it matters in flight

The corrupted slots are not cosmetic. They are read live, every loop:

| Slot | Used by |
|---|---|
| `PIDLEVEL` | the angle loop, [`pid.cpp:813`](../../../../src/main/flight/pid.cpp#L813) |
| `PIDVEL` | the inner altitude-hold loop, [`altitudehold.cpp:728-751`](../../../../src/main/flight/altitudehold.cpp#L728) |
| `PIDUSER` | the user PID *and* the UWB velocity loop, [`posControl.cpp:192-194`](../../../../src/main/flight/posControl.cpp#L192) |
| `PIDMAG` | heading hold |

So a user calling a function documented as "restore the defaults" retunes the angle and altitude loops to arbitrary
values. Nothing in the tree calls it today ( no caller in `PlutoPilot.cpp`, API-Src or MSP ), which is why it has not
cost a flight yet. The function writes RAM only, but any later save ( `MSP_EEPROM_WRITE` while disarmed, a trim through
`saveConfigAndNotify ( )`, or the CLI ) would persist the corrupted profile.

## Decisions taken in planning

- **Reset the five API slots only** ( `PIDROLL`, `PIDPITCH`, `PIDYAW`, `PIDALT`, `PIDUSER` ): exactly the slots
  `get_PIDProfile` / `set_PIDProfile` expose. The hidden gains are left as they are, so app or MSP tuning of them
  survives the call. Resetting all eleven was rejected as a behaviour change that the API never promised.
- **One source of truth.** The values are taken from the firmware defaults rather than from a second table in the API,
  so the stale-100 class of bug cannot come back. The `P8` / `I8` / `D8` block moves out of the `static`
  `resetPidProfile ( )` into a shared `pidProfileDefaultGains ( pidProfile_t * )`; `resetPidProfile ( )` calls it.
  Only the gains move: `pidController`, `yaw_p_limit`, the cut frequencies, the float gains, `A_level` / `H_level` and
  the GTUNE limits stay inside `resetPidProfile ( )`, since an API gain reset has no business touching them.
- **Not changed:** no EEPROM write, and no armed guard ( `set_PIDProfile` and `MSP_SET_PID` are unguarded too ). Both
  facts go into the header comment instead.
- **Verification is a host simulation**, not a bench log: a `g++` harness in `tools/sim/` compiles the real code against
  a stub profile, marks all eleven slots, calls the function and asserts that five slots take the firmware defaults and
  six are untouched. The user explicitly declined bench logging for this topic.
- **Out of scope:** `get_PIDProfile` / `set_PIDProfile` ( their index map is correct ), the mixed `ROLL` vs `PIDROLL`
  naming ( cosmetic ), an armed guard, an EEPROM save, and a new `docs/API/` FC-Config wiki.

## Open risk

With `pidController == 2` the roll / pitch / yaw gains come from the float `P_f` / `I_f` / `D_f` set, so the `P8` values
this function restores are ignored on those axes. The default controller is 1, so this does not affect the fix; it is
noted because the header comment should not over-promise.
