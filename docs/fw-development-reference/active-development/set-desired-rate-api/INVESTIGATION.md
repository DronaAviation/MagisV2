# Set desired rate API - Investigation

[README](README.md) · [TASKS](TASKS.md) · [SCOUT](SCOUT.md) · [INVESTIGATION](INVESTIGATION.md)

## Problem

API test T16 ( B30 ), reported by the test team: `setDesiredRate ( angle_e ANGLE, int32_t rate )` from
`API/FC-Config.h` does not keep its value. "Roll rate reset to 75 every loop while disarmed ( flip code ); the set
value never sticks."

## Causal chain ( proven by reading the code, 2026-11-15 )

1. **Setter.** [FC-Config-Setpoints.cpp:154](../../../../src/main/API-Src/FC-Config-Setpoints.cpp) writes
   `currentControlRateProfile->rates [ FD_* ] = rate` with no clamp. `rates` is `uint8_t [ 3 ]`
   ( `io/rc_controls.h:145` ), so any value outside 0-255 is silently narrowed ( 3 baseline `-Wconversion` warnings ).
2. **Disarmed reset.** [mw.cpp:1404-1412](../../../../src/main/mw.cpp) calls `flip ( false )` on every disarmed loop,
   before `userCode ( )`. [acrobats.cpp:343-349](../../../../src/main/flight/acrobats.cpp) then writes
   `rates [ FD_ROLL ] = rates [ FD_PITCH ] = 75`. It is a hard-coded value, not a save and restore, so a roll / pitch
   value set while disarmed lasts at most until the next loop.
3. **Armed reset.** A value set while armed holds until a flip: the watchdog ( `acrobats.cpp:87-88` ), ASCEND
   ( `:116-117` ), HOLD ( `:240-241` ) and HOLDPOS ( `:329-330` ) write 75, and the flip axis flies 110
   ( `:125 / 130 / 135 / 140` ). After the flip, roll / pitch are left at 75. A disarm brings back step 2.
4. **Getter.** [FC-Config-Setpoints.cpp:149](../../../../src/main/API-Src/FC-Config-Setpoints.cpp) `getDesiredRate`
   returns `desiredRate [ ]`, the PID's computed rate setpoint ( `flight/pid.cpp:75`, written at `:793 / :813 / :819` ),
   not the setting. A read-back never matches the setter on any axis, yaw included.
5. **Where the rate acts.** In `pidRewrite` ( `pid.cpp:788-813` ) the roll / pitch rate only matters outside ANGLE
   mode: RATE mode ( `FlightMode_Set ( RATE )` ), HORIZON, or the flip's own non-ANGLE states. Full-stick spin rate is
   `( rate + 20 ) * rcCommand >> 4`. The yaw rate scales the yaw stick in every mode, and the flip never touches it.

The effective roll / pitch rate on every craft today is 75: the config default is 0 ( `config.cpp:318-320` ), and
`flip ( false )` replaces it with 75 from the first disarmed loop.

## Decisions from the interview ( 2026-11-15 )

| Question | Answer |
|---|---|
| Mode | `fix` |
| Target | `PRIMUS_X2_v1` |
| Done when | The user runs the test themselves ( no T16 script is handed over ); the task plan supplies the test code and steps. |
| Scope | Only `setDesiredRate` and its getter. The flip's in-flight rates and timing, `getDesiredAngle`, EEPROM persistence and every other FC-Config API stay as they are ( what a flip leaves behind at its exits does change: that is the fix ). |
| Getter | `getDesiredRate` returns the set rate ( or the effective rate when none was set ). |
| Lifetime | Held in RAM until reboot or the next `setDesiredRate` call, across disarm / arm. After a flip, roll / pitch return to the user value instead of 75. The API does not write EEPROM. |
| Range | Clamp: 0-100 roll / pitch, 0-255 yaw ( the limits MSP_SET_RC_TUNING uses ). |
| Default | With no call, roll / pitch stay at 75: nothing changes for code that does not use the API. |
| Fix shape | Scout option A / C: a RAM user rate per axis in Setpoints; every place the flip code restores roll / pitch to 75 restores the user value when one is set. The flip's in-flight 75 / 110 are unchanged. |
| Wiki | New `docs/API/FC_CONFIG_API_WIKI.md` covering only `setDesiredRate` / `getDesiredRate`. |

## Open points

- Which `75` writes in `acrobats.cpp` are the flip's own flying rates ( kept ) and which are exits / idle restores
  ( changed to the user value ) is settled in task 3, by reading each state. Rule: anything the craft flies with
  while `flipState != 0` keeps 75 / 110; anything that leaves roll / pitch for normal flight uses the user value.
- The app's MSP_SET_RC_TUNING roll / pitch is also reset to 75 by `flip ( false )`. Out of scope; stays as today.
