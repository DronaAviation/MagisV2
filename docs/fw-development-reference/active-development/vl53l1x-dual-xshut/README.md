# VL53L1X Dual XSHUT

[README](README.md) · [TASKS](TASKS.md) · [SCOUT](SCOUT.md) · [DESIGN](DESIGN.md) · [CHANGES](CHANGES.md) · [TESTING](TESTING.md) · [PIPELINE_UPDATE](PIPELINE_UPDATE.md)

| | |
|---|---|
| **Status** | **Closed - pending commit** ( 5 Oct 2026, FW 3.11.0 not bumped ). `Altitude_Hold_Estimator.md`, CHANGELOG and CLAUDE.md promoted; maps edited with the topic |
| **Mode** | feature |
| **Branch** | `feat/vl53l1x-dual-xshut` ( from `BugFix-June26` at `cbd1c72` ) |
| **Target** | `PRIMUS_X2_v1` ( flown ) with `LASER_TOF_L1x` + `LASER_ALT`; `PRIMUS_V5` gets the same pin defines |

**Summary.** Two VL53L1X ( TOF400C ) share I2C1 and both answer at 0x29. The bottom one has its XSHUT on
`GPIO_5` ( PB15 ), the second on `GPIO_6` ( PB14 ). Today nothing drives these pins, so both sensors boot and
corrupt each other's replies without any I2C error: wrong calibration and wrong ranges that the estimator
trusts. This topic holds the second sensor in shutdown from boot and enables only the bottom one, in
`ranging_init_L1 ( )` before the probe, with the pins given by `target.h` defines ( option (a),
[DESIGN.md](DESIGN.md) ).

**Gotchas.**

- PB14 / PB15 are also SPI2 MISO / MOSI of the M25P16 flash, ADC_2 / ADC_3, PWM_4 / PWM_5, RGB_3, the XRanging
  FRONT / LEFT LEDs, `OPTIC_FLOW` and the user SPI API ( `SPI_Init` re-runs `spiInit ( SPI2 )` ). Using any of
  them after laser init can shut the bottom laser off or wake the second one. The flash is guarded: it is
  compiled out on PRIMUS_X2_v1 and an `#error` stops a build that has both ( task 10 ). The rest is documented
  only ( PIN_MAP conflict 8 ).
- The VL53L1 platform wait functions are stubs, so `WaitDeviceBooted` does not wait: the driver adds `delay ( 2 )`
  after releasing the bottom sensor.
- `plutoInit` runs after laser init, so user code cannot do this job.
- The TOF400C pulls SHUT up to ~2.8 V ( 2.7 V measured, task 1 ): the bottom pin is released as an input, never driven HIGH.

**Temporary:** the diagnostic log in `PlutoPilot.cpp` ( task 6 ) must be removed before release: task 9 reverts
the file to HEAD ( `git checkout -- PlutoPilot.cpp` ) before the all-target build.

**Open items.** All tasks done. Graph refresh skipped ( `graphify` not installed ): run `graphify update .` and
`python tools/graph_labels.py` once it is.
Outside this topic, for the user: the `setUserLoopFrequency` unit bug ( public API ) and the `tools/flightlog.py`
`t` field collision ( TESTING.md, task 6 results ).
