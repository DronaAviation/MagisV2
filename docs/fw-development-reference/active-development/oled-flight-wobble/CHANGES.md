# OLED Flight Wobble: changes

[README](README.md) · [TASKS](TASKS.md) · [SCOUT](SCOUT.md) · [INVESTIGATION](INVESTIGATION.md) · [CHANGES](CHANGES.md) · [TESTING](TESTING.md) · [PIPELINE_UPDATE](PIPELINE_UPDATE.md)

## Firmware

**[display_ug2864hsweg01.c](../../../../src/main/drivers/display_ug2864hsweg01.c) / `.h`**: new `i2c_OLED_send_run ( page, col, data, len )`.

- Sends the page and column commands as one transaction ( `80 B0+p 80 lo 80 hi`, Co = 1 per command ).
- Then sends the data bytes as a second transaction.

**[oled_display.c](../../../../src/main/io/oled_display.c)**

- New state:
  - `oledSystemBuffer [ 1024 ]`
  - `oledActiveBuffer`
  - `oledDirty`, `oledFlushPage`, `oledFailScore`
  - `OLED_FLUSH_MAX_BYTES` 16
  - `OLED_FAIL_COST` 4 / `OLED_FAIL_LIMIT` 12
- `oledFlushStep ( )`:
  - Each call finds the next byte that differs from the shadow and sends it with any changed bytes up to
    16, in one run.
  - On success it updates the shadow.
  - It goes idle after a full pass that finds no difference.
  - A failure adds 4 to the score and a success takes 1 off; at 12 it sets `OledInitStatus = false`.
- `sysText ( )` draws the 6 px grid text into `oledSystemBuffer`. All `disp* ( )` functions and
  `OledDisplaySystemData ( )` now use it instead of `i2c_OLED_set_xy` / `i2c_OLED_send_string`.
- `OledDisplayData ( )`:
  - Returns while the splash is showing.
  - In System mode it renders every 200 ms, and on a Dev-mode toggle it clears the frame with a `memset`
    instead of a hardware clear.
  - It always runs one `oledFlushStep ( )`.
- `Oled_SystemMode ( )` / `Oled_UserMode ( )`: no hardware access. They blank the system buffer, make it the
  active frame and mark it dirty. System mode forces a render on the next loop.
- `Oled_display_Update ( buf )` only queues: `oledActiveBuffer = buf`, dirty.
- `Oled_Print ( )` draws into the system buffer.
- The end of the splash and `Oled_display_Clear ( )` call `oledPanelCleared ( )` ( zero the shadow ) after
  the hardware clear.

**Docs and comments**

- [oled_display.h](../../../../src/main/io/oled_display.h) and [API/Oled.h](../../../../src/main/API/Oled.h):
  descriptions of `Oled_Clear`, `Oled_Update`, `Oled_Print` and the modes.
- [OLED_API_WIKI.md](../../../API/OLED_API_WIKI.md): new section "Timing and flight safety"; the
  `Oled_Print` and `Oled_display_Update` notes.

## User project ( separate copy only, not merged )

In the separate copy, the Traffic Hazard `PlutoPilot.cpp` gained buzzer off plus `Oled_Mode ( System )` in
`onLoopFinish`, and its write-up ( `project/TRAFFIC_HAZARD_DRONE.md` ) gained an OLED timing section. Neither is
part of MagisV2: the stock `PlutoPilot.cpp` stays as it is.

## Merge fixes ( task 5 review, 14 Oct 2026 )

- [oled_display.c](../../../../src/main/io/oled_display.c) `Oled_display_Clear ( )`: returns at once if `! OledInitStatus`,
  so an absent or switched-off OLED gets no bus traffic at all. Before, user code could still trigger 32 timed-out
  transactions and an I2C1 re-init while disarmed.
- `Oled.h` `Oled_Update` comment: a 21-character line takes ~30 ms, not "a few milliseconds"; User mode only.
- `oled_display.c` HISTORY rows realigned to 80 columns.

## Cost

Merged build in MagisV2, PRIMUS_X2_v1, task 1:
- Flash +0.4 KB ( 101.3 KB / 256 KB ).
- RAM +2 KB ( 16.8 KB / 40 KB ). This is `oledSystemBuffer` plus `oledShadowBuffer`; at HEAD the shadow was dropped by
  the linker in the stock build. In the separate copy the figures were 102.7 / 17.8 KB, with a different `PlutoPilot.cpp`.
- Per loop: one run of 16 bytes or less, about 0.6 ms, only while a frame differs from the panel.
