# Set desired rate API - Scout brief

[README](README.md) · [TASKS](TASKS.md) · [SCOUT](SCOUT.md) · [INVESTIGATION](INVESTIGATION.md)

Brief from the `pluto-scout` agent, 2026-11-15, kept verbatim.

## Scout brief: setDesiredRate ( angle_e, int32_t ) value is overwritten (T16 / B30: roll rate goes back to 75 every loop while disarmed)
Graph: stale (built ebf4e46b, HEAD 73022c3). Only the setDesiredAngle commit is missing, so I used grep for the findings below.

Subsystems: User API (setpoints), acrobats/flip, PID (pidRewrite), MSP. Pipeline docs: `subsystems/User_Space_API.md`, `subsystems/Core_PID_Controller.md`.

Key code:
- `src/main/API-Src/FC-Config-Setpoints.cpp:154` `setDesiredRate`: writes `currentControlRateProfile->rates[FD_*] = rate` with no clamp. `rates` is `uint8_t[3]` (`io/rc_controls.h:145`), so the value is silently narrowed. This is 3 of the baseline `-Wconversion` warnings (`tools/warnings-baseline.json:109`).
- `FC-Config-Setpoints.cpp:149` `getDesiredRate`: returns `desiredRate[ANGLE]`, which is the PID's computed rate setpoint (`flight/pid.cpp:75`, written at `:793/:813/:819`). It does not read the setting.
- `src/main/flight/acrobats.cpp:343-349` `flip(false)`: sets `rates[FD_PITCH] = 75; rates[FD_ROLL] = 75;`.
- `src/main/mw.cpp:1404-1412`: `if ARMED { if flipState>=1 flip(true); } else flip(false);`. This runs every disarmed loop, before `userCode()` (`mw.cpp:1436`).
- `acrobats.cpp:87-88, 116-117, 240-241, 329-330`: the flip watchdog, ASCEND and HOLD/HOLDPOS states write 75. `:125/130/135/140` write 110 on the flip axis.
- `src/main/flight/pid.cpp:788-813` (pidRewrite, `pidController = 1`, `config.cpp:176`): uses roll/pitch `rate` only when ANGLE mode is off. Yaw `rate` is always used (`:793`).
- `src/main/io/serial_msp.cpp:1516` MSP_SET_RC_TUNING: clamps to 100 / 255. `:1638-1644` MSP_EEPROM_WRITE: disarmed only.
- `src/main/config/config.cpp:318-320`: default `rates[] = 0`.

Mode: fix

Current behaviour: the setter writes the live, EEPROM-backed rate profile in RAM. While the craft is disarmed, `flip(false)` puts roll and pitch back to 75 on every loop. Armed, the value stays until the next disarm or a flip, which ends with 75/75 again. Yaw is never reset. The getter returns a different quantity, so a read-back never matches the setter on any axis.

Analysis (causal chain):
1. Set roll/pitch while disarmed, e.g. in plutoInit or onLoopStart: the next loop's `flip(false)` (`acrobats.cpp:347-348`) writes 75. **Proven by reading the code.** The reset is unconditional; it is not a save/restore. In practice 75 is the real roll/pitch rate on every craft, because `config.cpp:319` defaults to 0 and nothing else writes it. The same reset defeats the app's MSP_SET_RC_TUNING roll/pitch, and an MSP_EEPROM_WRITE while disarmed stores 75.
2. Set while armed: the value holds until a flip (ASCEND `:116-117`, HOLD `:240`, HOLDPOS `:329`, watchdog `:87`), which leaves 75/75, or until disarm. **Proven.**
3. Read back with `getDesiredRate`: the test sees the PID setpoint `desiredRate[]` (gyro/4 units, roughly 4.1 per deg/s), never the set value, even for yaw. **Proven.** It is unclear whether T16 reads back through this getter or through MSP/app; ask the user.
4. What changes in flight: roll/pitch `rate` only takes effect in RATE mode (`FlightMode_Set(RATE)`, `FC-Control-Command.cpp:88-92`), HORIZON mode, or during the flip's non-ANGLE states. In RATE mode it is the full-stick spin rate, `(rate+20)*rcCommand>>4`. Yaw `rate` scales the yaw stick in every mode. In the default ANGLE mode, roll/pitch rates have no effect at all.

Candidate fixes:
- **A. Separate user rate field (recommended).** Add `userRate[3]` (or "isUserRateSet") in Setpoints.cpp. The flip writes its own values, and `flip(false)` and the flip exits restore the user/base value instead of a hard 75. The setter clamps to 0..100 (roll/pitch) and 0..255 (yaw), which removes 3 baseline warnings. Fix `getDesiredRate` to return the set rate. Files: `FC-Config-Setpoints.cpp`, `acrobats.cpp`, maybe `API-Utils.h`, `FC-Config.h` doc comment. Cost: about 100-200 B flash, 3-4 B RAM, no extra loop cost. The flip still flies 75/110 during its states, so its tuning is unchanged.
- **B. Save/restore inside acrobats.** Snapshot `rates[]` at flip entry (`command.cpp:343`) and restore at every exit. Drop the per-loop disarmed write, or make it one-shot. Files: `acrobats.cpp`, `command.cpp`. Smaller, but it changes the app/EEPROM default from 75 to the stored value (0 by default, so the RATE-mode stick gets much slower). It also has to cover 6+ exit paths (watchdog, `mwDisarm` timeouts `:175/:197`, state ends).
- **C. Minimal.** Keep `flip(false)`, but restore a stored user value in place of 75 when one is set. Fix the getter and add the clamp. Cheapest option; the flip internals stay as they are.

All options change API behaviour, so they need: a wiki (none exists for FC-Config, so create one or add to an existing wiki), and an `API_Version` bump (currently FW 3.15.8 / API 1.8.6).

Constraints / invariants that apply:
- Flip invariants: altitude hold is bypassed, not switched off, and a flip ends within 8 s (`FLIGHT_INVARIANTS.md`). Do not alter the flip's own 75/110 rates without flight evidence (see flip-althold-regression).
- Strict `-Wconversion`: clamp explicitly with `constrain`. Do not regenerate the baseline.
- API behaviour change: update the `docs/API/` wiki and bump `API_Version` (CLAUDE.md, Architecture).
- Decide persistence: writing `masterConfig` from user code is persisted by any app EEPROM save.

Resource conflicts: none.

Already answered by the code:
- Units are the Cleanflight "rate", range 0-100 (roll/pitch) and 0-255 (yaw), stored as `uint8_t`.
- The setter does not write EEPROM itself, but MSP_EEPROM_WRITE / profile change do.
- 75 is the flip's fixed baseline and 110 its flip-axis rate. The flip never touches yaw.
- `rc_controls.cpp:445-459` in-flight adjustments and `serial_cli.cpp:450-452` are other writers.

Questions for the user:
1. How does T16 read the value back: `getDesiredRate`, the app, or MSP_RC_TUNING? Should `getDesiredRate` return the set rate?
2. Should a user rate apply only in Developer Mode (and be dropped at dev-off/disarm, as setDesiredAngle is), or persist like a config setting?
3. Is the user rate allowed to change the flip's own rates (75/110), or must flips keep them?
4. What should the default be when no user rate is set: keep 75 (today's effective value) or the stored config value (0)?
5. Clamp out-of-range input, or ignore it?
6. Is fixing the app's MSP_SET_RC_TUNING roll/pitch (also reset to 75) in scope?

Doc drift: the FC-Config.h:180-194 comments state no units or range, and do not say the getter returns the PID setpoint. `config.cpp:420` says "50 degrees" for 200 (actually 20 deg); this was noted before and is out of scope. The pipeline docs do not mention the 75 reset.

Prior work: `active-development/set-desired-angle-roll-pitch/` (SCOUT.md:88, TASKS.md:224 list this as a follow-up); `estimate-get-rate-api` (1b43fbe) found the same "always 75"; `flip-althold-regression` (c569c0f).

Suggested skills: pluto-rules, pluto-build (--gate PRIMUS_X2_v1), pluto-flighttest (RATE-mode stick plus a flip), pluto-commit; pluto-reviewer agent; cpp-pro.

Pipeline docs to update at the end:
- `User_Space_API.md`: setDesiredRate/getDesiredRate semantics.
- `Core_PID_Controller.md`: where roll/pitch/yaw rates act, and the flip's rate handling.
- New or extended `docs/API/` FC-Config wiki.
