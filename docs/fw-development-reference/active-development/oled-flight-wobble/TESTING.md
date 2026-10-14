# OLED Flight Wobble: testing

[README](README.md) · [TASKS](TASKS.md) · [SCOUT](SCOUT.md) · [INVESTIGATION](INVESTIGATION.md) · [CHANGES](CHANGES.md) · [TESTING](TESTING.md) · [PIPELINE_UPDATE](PIPELINE_UPDATE.md)

## Build ( 10 Oct 2026 )

- `PRIMUS_X2_v1`: builds, warning gate "no new warnings in src/".
- Flash 102.7 KB, RAM 17.8 KB.
- Review by `pluto-reviewer`: no blocking findings. The should-fix items and nits were fixed:
  - leaky failure score;
  - glyph range 32-166;
  - wiki texts;
  - banners.

## Build in MagisV2 ( 14 Oct 2026, task 1 )

- `PRIMUS_X2_v1` at HEAD `84c326d` with the merged code: no new `src/` warnings.
- Flash 101.3 KB ( +428 B over HEAD ), RAM 16.8 KB ( +2048 B ). The extra 1 KB is the shadow buffer, which is now
  always linked; at HEAD the linker dropped it, because the stock `PlutoPilot.cpp` never calls `Oled_display_Update ( )`.

## 1. A/B flight: confirm the cause ( passed, separate copy )

Use the new firmware for both flights. Same drone, OLED fitted and wired, Dev mode **off**, angle mode,
30 s hover, no stick input.

| Flight | `PlutoPilot.cpp` | Expected |
|---|---|---|
| A | `Oled_Init ( )` commented out | stable: the OLED is never addressed |
| B | `Oled_Init ( )` in ( telemetry live ) | stable as well, if the fix works |

How to read the result:

- **A twitches:** the cause is mechanical or electrical, not the I2C blocking. Stop and plan again: fly once
  more with the OLED unplugged and its weight taped in the same place.
- **A stable, B twitches:** the flush step is still too costly. Lower `OLED_FLUSH_MAX_BYTES` ( 16 → 8 ) and
  fly again.
- **Both stable:** fixed.

Result: **passed** ( user's report, 14 Oct 2026; flown in the separate copy ): both flights stable.

## 2. Validation: the fix ( passed, separate copy )

New firmware, `Oled_Init ( )` in, Traffic Hazard Drone `PlutoPilot.cpp`.

1. **Bench, props off, disarmed.**
   - The splash shows for 2 s, then the telemetry screen.
   - P/R/Y and the RC values update about 5 times a second, with no garbled characters.
   - Dev mode on: the warning screen builds up in about 0.25 s, the message rotates every 3 s, and the
     buzzer runs 500/500.
   - Dev mode off: the telemetry returns.
2. **Bench, armed, props off.** Repeat step 1. There must be no leftover pixels in either direction.
3. **Hover, Dev mode off**, 30 s: as steady as without the OLED, with no 5 Hz twitch.
4. **Hover, Dev mode switched on and off** 3 times: no jerk at the switch, and steady while the warning
   runs.

Result: **passed** ( user's report, 14 Oct 2026; flown in the separate copy ). Per-step notes were not recorded.

## 3. Confirmation hover on this repo's merged build ( task 2: passed )

**Why:** the separate copy's tested build did not contain `386599b` ( crash limits ) or `84c326d` ( failsafe
delay, `EEPROM_CONF_VERSION` 108 ). Neither touches the OLED code; this hover confirms that the merged build
behaves the same.

**Build:** `Build/PRIMUS_X2_v1/DEFAULT_PRIMUS_X2_v1_3.11.2.hex` from task 1 ( FW 3.11.2 plus the OLED fix ).
- **User code:** a `PlutoPilot.cpp` that calls `Oled_Init ( )` in `plutoInit`, for example the Traffic Hazard
  one. **Do not commit it**: the repo's stock `PlutoPilot.cpp` stays as it is.
- **First boot:** if the board's last firmware was older than 3.11.2, the first boot resets the settings.
  Calibrate the accelerometer and the compass before flying.

**Method:** observation, with notes in [logs/log-1.txt](logs/log-1.txt).

**Steps** ( OLED fitted and wired, ANGLE mode, low over a soft floor ):
1. **Bench, disarmed:**
   - the splash shows for 2 s, then the telemetry screen;
   - the values update about 5 times a second, with no garbled characters.
2. **Hover, Dev mode off, 30 s, no stick input:** as steady as the drone flies without the OLED, with no 5 Hz
   twitch. The OLED keeps updating.
3. **Hover, Dev mode switched on and off 3 times:** no jerk at the switch, steady while Dev mode runs, and the
   OLED switches between the screens.
4. **Land.** Check that the screen shows no leftover pixels.

**Pass:** steps 1-4 with no twitch and the OLED updating.
**Fail:** a 5 Hz twitch comes back. Check that the right hex was flashed. If it was, lower
`OLED_FLUSH_MAX_BYTES` ( 16 → 8 ), as in §1.

Result: **passed** ( user's report, 14 Oct 2026 ), flown with the Traffic Hazard `PlutoPilot.cpp` on the merged 3.11.2 build. The notes sheet was not filled in.
