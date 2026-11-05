# RcCommand_Get API - Scout brief

[README](README.md) · [TASKS](TASKS.md) · [SCOUT](SCOUT.md) · [INVESTIGATION](INVESTIGATION.md)

The `pluto-scout` agent was not run: the request is one function and the reconnaissance was done inline during
planning ( 5 Nov 2026 ). Facts, each from a source line or a build artefact:

## Files

- `src/main/API/RC-Interface.h:76` declares `int16_t *RcCommand_Get ( void )`; `:87` declares
  `int16_t RcCommand_Get ( rc_channel_e CHANNEL )`. The doc comment above each describes the other overload.
- `src/main/API-Src/RC-Interface.cpp:101-111` ( after the user's rename ): `int16_t *RcCommand_Get ( void )`, local
  `int16_t rcCommandArray [ 4 ]`, copy of `rcCommand [ 0..3 ]`, `return rcCommandArray;`. HEAD names it `RcCommand_get`.
- `RC-Interface.cpp:82-92`: `int16_t *RcData_Get ( void )`, the same pattern over 11 channels of `rcData`.
- `RC-Interface.cpp:114-121`: `RcCommand_Get ( CHANNEL )` returns `1500 + rcCommand [ CHANNEL ]` for
  `CHANNEL <= RC_THROTTLE`, else 0.
- `RC-Interface.cpp:123-172`: `RcCommand_Set` constrains to 1000-2000; roll / pitch / yaw go to `rcCommand` minus
  1500, throttle to `rcData [ THROTTLE ]`; each call asserts the user override ( `userRCassert` ).

## Ranges

- `rc_channel_e` ( `RC-Interface.h:27-41` ): `RC_ROLL`, `RC_PITCH`, `RC_YAW`, `RC_THROTTLE`, then AUX1-4, USER1-3.
- `rcCommand [ ROLL / PITCH / YAW ]`: about -500..500 ( Cleanflight stick deflection after the rate curve ).
- `rcCommand [ THROTTLE ]`: `lookupThrottleRC`, MINTHROTTLE..MAXTHROTTLE ( `mw.cpp:411` ); altitude hold writes its
  own output into it in ALT_HOLD, but only after `userCode ( )` in the same loop ( `mw.cpp` annexCode 1410, userCode
  1416, applyAltHold 1455 ), so user code always sees the stick value ( task 1 review ).

## Build evidence

- `Build/PRIMUS_X2_v1/build.log`: `RC-Interface.cpp:91` and `:110`, `-Wreturn-local-addr`, address of local variable
  returned; both in `tools/warnings-baseline.json:135-136`.
- `arm-none-eabi-objdump -d Build/PRIMUS_X2_v1/bin/API-Src/RC-Interface.o`: `RcData_Get()` and `RcCommand_get()` are
  each `movs r0, #0; bx lr`: GCC replaced the dangling pointer with NULL.

## Users

- No firmware caller of either array getter. No `docs/API/` wiki covers the RC interface ( only `OLED_API_WIKI.md`
  mentions RC ). `User_Space_API.md` has no `RcCommand_Get` line.

## Skills

pluto-rules ( code ), pluto-build ( gate ), pluto-flighttest ( bench log ), pluto-reviewer, pluto-commit.
