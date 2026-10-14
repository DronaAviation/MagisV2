# OLED Flight Wobble

[README](README.md) · [TASKS](TASKS.md) · [SCOUT](SCOUT.md) · [INVESTIGATION](INVESTIGATION.md) · [CHANGES](CHANGES.md) · [TESTING](TESTING.md) · [PIPELINE_UPDATE](PIPELINE_UPDATE.md)

| | |
|---|---|
| **Status** | **Closed - pending commit** ( 14 Oct 2026, FW 3.11.3, API 1.4.1 ) |
| **Mode** | `fix` ( planned with pluto-grill in the separate copy; merge planned 14 Oct with a TASKS.md ) |
| **Branch** | `BugFix-June26` ( written in a separate working copy that was not a git repo ) |
| **Target** | `PRIMUS_X2_v1`, receiver `Rx_ESP` |
| **Origin** | Traffic Hazard Drone user project, in a separate working copy; its `PlutoPilot.cpp` and write-up are not part of this repo |

**Problem.** With an OLED fitted and `Oled_Init ( )` called from `plutoInit`, the drone twitched about 5 times a
second for the whole flight, with Dev mode on or off. Without the OLED it flew stable.

**Cause ( from the code; confirm with the A/B flight ).**

- `Oled_Init ( )` turns on the system OLED telemetry. `OledDisplayData ( )` ran it every 200 ms, armed or not.
  It is called from `userCode ( )` ( `mw.cpp:1130` ), between `imuUpdate ( )` and the PID.
- The old telemetry wrote straight to the panel with **one blocking I2C transaction per font byte**: about
  630 transactions, **45-60 ms every 200 ms**.
- I2C1 is polled and shared with the ICM20948 gyro/acc, the baro and the mag. The PID then ran on a gyro
  sample about 50 ms old with `cycleTime` about 54 ms: an I-term step and a D spike at 5 Hz.
- The stock `PlutoPilot.cpp` never called `Oled_Init ( )`, so the OLED did nothing in flight before.

**Fix ( design A: RAM framebuffer + rate-limited diff flush ).**

- The system telemetry is drawn into a 1 KB RAM framebuffer ( `oledSystemBuffer` ).
- Every write to the panel goes through `oledFlushStep ( )`. Once per gated loop it sends one run of at most
  16 changed bytes ( about 0.6 ms ).
- `Oled_Update ( )` / `Oled_display_Update ( )` only queue the frame, and `Oled_Mode ( )` no longer clears the
  hardware. This is the same in User and System mode, armed or not.
- A leaky failure score switches the OLED off if the panel stops answering.

Telemetry stays live in flight ( the user's choice ).

**Gotchas.**

- `Oled_Update ( )` is now deferred: a full-screen change reaches the panel in about 225 ms. A buffer passed to
  `Oled_display_Update ( )` must stay valid ( `static` ).
- The 2 s start-up splash still writes the panel directly. It runs only at boot, before arming, and ends
  with a hardware clear plus `oledPanelCleared ( )`.
- Never write the panel directly. The only exception is a full hardware clear followed by `oledPanelCleared ( )`
  inside `oled_display.c`: zeroing the shadow after any other direct write makes the diff flush skip bytes.

**Open items.**

- **Before commit:** `PlutoPilot.cpp` holds the Traffic Hazard test code. Leave it out of the commit.

1. A/B flight ( TESTING.md §1 ): confirm that the software cause is the whole story.
2. Validation hover with telemetry live, Dev mode off and on ( TESTING.md §2 ).
3. At commit ( `/pluto-commit` ):
   - `API_Version` bump, because `Oled_Update` behaviour changed ( wiki already updated ).
   - CHANGELOG entry.
   - Correct the CHANGELOG v4.0.0 line that calls the old OLED rendering "non-blocking".
   - Note in `Hardware_Bus_Pipeline.md` that I2C1 is polled and shared with the OLED.
4. Not in scope:
   - The start-up splash ( blocking, boot only ).
   - The `mw.cpp:1134` user-loop timer wrap after 35.8 min ( found in the PlutoPilot review ).
