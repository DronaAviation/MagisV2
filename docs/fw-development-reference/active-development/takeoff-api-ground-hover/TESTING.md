# Take-off API ground hover - Testing

[README](README.md) · [TASKS](TASKS.md) · [SCOUT](SCOUT.md) · [INVESTIGATION](INVESTIGATION.md) · [TESTING](TESTING.md)

## 1. A/B flight ( tasks 1-2 )

### Test versions

| Version | File | `onLoopStart ( )` |
|---|---|---|
| A | [test/PlutoPilot_A_arm_takeoff.cpp.txt](test/PlutoPilot_A_arm_takeoff.cpp.txt) | `Command_Arm ( )`, then `Command_TakeOff ( 100 )`: the user's report |
| B | [test/PlutoPilot_B_takeoff_only.cpp.txt](test/PlutoPilot_B_takeoff_only.cpp.txt) | `Command_TakeOff ( 100 )` only. It arms the drone itself |

Both versions are otherwise identical: `Rx_ESP`, the same log line, and an automatic `Command_Land ( 5 )` 12 s after
the drone arms. Speed 5 ramps down from 1300, like the failsafe landing. The API default of 105 starts at 1200, which
is already `LAND_CONFIRM_THROTTLE`, so the touchdown check could disarm the drone in the air ( task 4 ). Each makes its calls only on the first `onLoopStart ( )` while the drone is disarmed. That first call is
exactly the user's scenario. Later restarts, from a Wi-Fi flicker, skip the calls so that they cannot overwrite the
landing ( task 7 ). To fly a version, copy it over the root `PlutoPilot.cpp` and build PRIMUS_X2_v1. **Restore the
template `PlutoPilot.cpp` before committing** ( `git checkout PlutoPilot.cpp` ).

Build ( 16 Oct 2026, FW 3.13.0 ): both compile, no new `src/` warnings. Flash 107.8 KB ( 42% ), RAM 17.6 KB ( 44% ).

### Log line ( one per user tick, about 119 B; budget 130 )

`t:<ms> n:<tick> F:<flags> C:<cmd> Zd:<cm> Ze:<cm> Vz:<cm/s> Mo:<us> Tp:<us>`

| Field | Meaning |
|---|---|
| `t` | `millis ( )` |
| `n` | tick counter. It steps by 2 across a lost record and by 1 across a stall |
| `F` | 4 flag digits, zero-padded on the left: armed, ALT HOLD, `isThrottleStickArmed` ( ground idle ), `isTakeOffHeightSet`. `1101` = armed, ALT HOLD, no ground idle, height set |
| `C` | `current_command * 10 + command_status`. Command: 0 NONE, 1 TAKE_OFF, 2 LAND. Status: 0 FINISHED, 1 RUNNING, 2 ABORT. A take-off finishes within one main loop of arming, before the first logged tick, so a normal log shows `0` until the landing ( `21` ), and `20` after it ( nothing resets `current_command` ). A `C:11` that stays means the take-off is waiting for the ARM switch, or `mwArm ( )` refused |
| `Zd` | shaped altitude target `AltHold`, cm ( `getDesiredPositions ( Z )`, `altitudehold.cpp:775` ). It is not the goal: the +100 cm goal itself is not logged. The target leads the estimate by at most 50 cm ( leash, `altitudehold.cpp:765` ), and it ramps even with BARO_MODE off |
| `Ze` | estimated altitude, cm ( `Estimate_Get ( Position, Z )` ) |
| `Vz` | estimated Z velocity, cm/s |
| `Mo` | mean of the four motor outputs, µs |
| `Tp` | pilot throttle stick, `rcDataPilot [ RC_THROTTLE ]`, µs |
| `S:<ms>` | on its own line: `onLoopStart ( )` ran. More than one means Dev Mode restarted on a Wi-Fi flicker |

### Procedure ( same place, same battery, A then B )

1. Flash version A. Put the drone on a flat floor in open space. Connect the app and turn ALT HOLD on. Leave the
   throttle stick centred.
2. Start logging in PlutoMonitor. Nothing is printed yet: the log comes from `plutoLoop ( )`, which runs only in
   Developer Mode. The first line is the `S:` marker, and in both versions the drone arms on that same tick, so the
   log has no disarmed lines from before take-off. The `Ze` on the first lines is the floor height. API arming does
   not re-zero the baro reference: `baroResetGroundLevel` runs only when `mwArm ( )` is called while already armed,
   from the ARM-switch path. So read a climb as `Ze` minus the first line's `Ze`.
3. Turn the app's ARM switch on. With the stick centred the switch does not arm the drone by itself: that needs
   the stick low, or a throw ( free fall, `rc_controls.cpp:169-190` ).
4. Then turn Developer Mode on. `onLoopStart ( )` runs. **Always ARM first, then Dev Mode.** In the other order
   `Command_Arm ( )` fails, and version A turns into version B when ARM goes on.
5. Do not touch the sticks. After 12 s armed the test sends `Command_Land ( 5 )`. Take over and land from the app if
   anything looks wrong.
6. Keep Developer Mode on. Wait 5 s after the drone disarms, so the log keeps a few disarmed lines, then stop the log. Save it as `log-1.txt` in this folder.
7. Flash version B and repeat steps 1-6. Save the log as `log-2.txt`.

Note anything the log cannot show: a visible hop, sliding on the floor, how high the drone went by eye.

### Expected result

| | A ( Arm + TakeOff ) | B ( TakeOff alone ) |
|---|---|---|
| H1 confirmed | `F` is `1100` ( height not set ) and `C` is `0` from the first logged tick. `Zd` stays flat at the ground `Ze`. `Mo` near hover, `Ze` within a few cm of the ground | `F` is `1101` ( height set ). `Zd` ramps to about ground + 100 cm in about 3 s and `Ze` follows |
| H1 ruled out | `F` ends with 1 and `Zd` climbs, yet the drone stays low | B also skims the floor: `F` = `1101` with `Zd` held at about `Ze` + 50 ( the leash ) |
| H2 | not visible: A has no goal for the stick to cancel | `Tp` outside 1460-1540 while armed ( `alt_hold_deadband` 40 ), with `Zd` stopping its ramp. Ignore `Tp` once `C` = 21 ( LAND pins `rcData` ) |
| H3 | the second digit of `F` is 0 ( read `F`, not `Zd` ) | same |

**Invalid run:** the third digit of `F` is 1 ( `isThrottleStickArmed`, e.g. `F:1110` ). The log starts at Dev Mode, so
lines from before it cannot show this. Only the ARM-switch path sets that flag; both test versions arm through
`mwArm ( )`, which does not. A 1 means the app sent a low throttle and the switch armed the drone before Dev Mode came
on. The user code then skipped both calls, so `F` also ends in 0 and `C` stays `0`. Repeat the run with the stick
centred.

**Not tested here: the ground idle.** Both versions arm through `mwArm ( )` directly, so `isThrottleStickArmed` stays 0
in A and in B ( the third digit of `F` ). The idle also needs `rcData [ THROTTLE ] <= 1500` ( `altitudehold.cpp:581` ).
The idle part of H1 shows only in task 3, if at all.

## 2. Results ( task 2, 16 Oct 2026, PRIMUS_X2_v1, FW 3.13.0, `Rx_ESP` )

Logs: [log-1.txt](log-1.txt) ( A ), [log-2.txt](log-2.txt) ( B ). Both runs are valid: one `S:` marker each, and the third
digit of `F` is 0 throughout ( no ARM-switch arming ).

| | A: Arm + TakeOff ( log-1 ) | B: TakeOff alone ( log-2 ) |
|---|---|---|
| Records | 189 ( 1 incomplete ). `n` 50 → 58 over 813 ms with no `S:`: 7 records lost on the link, no stall | 284, no gaps |
| `F` while armed | `1100` on every tick: **height never set** | `1101` on every tick: height set |
| `C` | `0` from the first tick, `21` at the landing ( 12.1 s ), `20` after | same |
| `Zd` ( target ) | **0 for the whole 12 s** ( −4 to 0 ) | 1 → 50 → 98 → 100 in 2.5 s ( about 40 cm/s ), then 100 held |
| `Ze` ( estimate ) | −19 to +5 cm, mean about −4: baro noise around the floor, never a climb | 78 cm at 2.5 s, 84-94 cm from 4.5 s to 12 s, max 95 |
| `Mo` ( mean motor ) | 1507-1603, about 1555: below hover, the drone is carried partly by the floor and ground effect | about 1610 in the hover |
| `Tp` | 1500 throughout | 1500 throughout |
| Landing | `C:21` at 12.1 s, disarmed at 12.5 s ( 0.4 s ) | `C:21` at 12.1 s, descent about −70 cm/s, disarmed at 14.4 s ( 2.3 s ) |
| Restarts | none ( one `S:` ) | none |

**Verdict.**

- **H1 confirmed.** With `Command_Arm ( )` first, the take-off sets no height ( `F` last digit 0 ), and the target stays
  at the floor ( `Zd` 0 ). Altitude hold holds 0 cm at about 1555 µs, which is the skim. Without `Command_Arm ( )` the
  same call arms the drone, sets the goal and climbs to about 90 cm.
- **H2 ruled out.** `Tp` was 1500 on every tick, inside the 1460-1540 deadband.
- **H3 ruled out.** The ALT HOLD digit of `F` was 1 on every tick in both flights.
- **H4 not tested, by design.** There were no restarts, and the test versions do not re-call on restarts.

**Side observations ( not this topic's cause ).**

- B settles 6-16 cm below the 100 cm target during the hover. That is the baro altitude-hold error, not the take-off.
- A disarmed 0.4 s after `Command_Land ( 5 )`. The drone was on the floor, so cutting from about 1555 to 1300 most
  likely jolted it past the impact check ( `|accZ| > LAND_IMPACT_ACC`, `command.cpp:209` ). `accZ` is not logged, so
  this is unproven.
- No Dev Mode restart happened in either flight, so task 7 has no flight evidence. It relies on the code.

## 3. Workaround flight ( task 3 )

Version C, [test/PlutoPilot_C_arm_setrelative.cpp.txt](test/PlutoPilot_C_arm_setrelative.cpp.txt), is now in the
root `PlutoPilot.cpp`. Its `onLoopStart ( )` runs `if ( Command_Arm ( ) ) DesiredPosition_setRelative ( Z, 100 );`, on
the first call while disarmed only. Everything else is as in A and B ( the log line, `Command_Land ( 5 )` at 12 s ).
Build: no new warnings, flash 107.8 KB, RAM 17.6 KB. Review: no blocking findings.

Why it should climb: `Command_Arm ( )` arms in the same call, and `setRelativeAltitude` writes
`AltHold = EstAlt + 100` ( `altitudehold.cpp:1462` ). The controller runs at the top of the next loop
( `executePeriodicTasks ( )`, `mw.cpp:1331` ), already armed. It sees `AltHold` differ from the target and makes it
the goal ( `altitudehold.cpp:737-740` ). Nothing resets the setpoint in between. Sketch B arms and writes the goal
the same way, from `executeCommand ( )`.

**Reading log-3 ( it differs from A and B ):**

- **`F:1100` and `C:2` are normal for C.** C never calls `takeOff ( )`, so `isTakeOffHeightSet` stays 0, and
  `command_status` keeps its boot value 2 ( ABORT ). The landing shows `21`, and `20` after the disarm. Judge
  success on `Zd` and `Ze` only.
- **Expected:** `Zd` ramps from the floor `Ze` to about floor + 100 cm in about 2.5 s. `Ze` settles 84-94 cm above
  the floor, with `Mo` about 1610: the same climb as B.
- **Failure:** `Zd` stays at the floor, as in A.

**Run conditions ( as in §1 ):** ALT HOLD on before Dev Mode. Turning it on after arming resets the setpoint
( `altitudehold.cpp:617-620` ) and drops the goal. ARM before Dev Mode: otherwise `Command_Arm` fails once and C
does nothing. Toggle Dev Mode to retry. Save the log as `log-3.txt`.

**Not answered by C: the ground idle.** Like A and B, C arms through `mwArm ( )`, so `isThrottleStickArmed` stays 0
and the lift-off is the same as B's. Only the ARM-switch path ( stick low, `rc_controls.cpp:175` ) sets the idle. No
API arming path does, and B climbed without it, so the missing idle is not part of this fault.

### Result ( 16 Oct 2026, log-3 )

[log-3.txt](log-3.txt): 177 records. `n` 78 → 86 is 7 records lost on the link. One `S:`, so no restarts. The third
digit of `F` is 0, so the run is valid.

| | C: Arm + `DesiredPosition_setRelative` ( log-3 ) | B for comparison ( log-2 ) |
|---|---|---|
| `F` / `C` | `1100` / `2` throughout, normal for C. `21` at the landing, `20` after | `1101` / `0` |
| `Zd` | 1 → 50 → 99 → 101 in 2.5 s, then held at 101 | 1 → 100 in 2.5 s |
| `Ze` | 74 at 2.5 s, 80-97 from 4.5 s to 12 s | 78 at 2.5 s, 84-94 |
| `Mo` hover | about 1635 ( 1593-1662 ) | about 1610 |
| Lift-off | `Ze` dips to −10 at 0.5 s, then climbs at 32-38 cm/s | dips to −5, then climbs at 35-41 cm/s |
| Landing | `C:21` at 12.1 s, disarmed at 14.2 s | disarmed at 14.4 s |

**Verdict: the workaround works.** `Command_Arm ( )` followed by `DesiredPosition_setRelative ( Z, 100 )` in the same
`onLoopStart ( )` tick climbs and holds the same as `Command_TakeOff ( 100 )` alone. The lift-off is the same as B's,
with no hop or jolt in `Vz` or `Mo`. The small dip in `Ze` is baro pressure under the rotors at lift-off. It appears in
B too. Hover `Mo` is about 25 µs higher than in B, most likely because this was the third flight on the same battery.
That is not related to this topic.
