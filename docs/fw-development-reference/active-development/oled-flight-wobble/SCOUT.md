# OLED Flight Wobble: scout brief

[README](README.md) · [TASKS](TASKS.md) · [SCOUT](SCOUT.md) · [INVESTIGATION](INVESTIGATION.md) · [CHANGES](CHANGES.md) · [TESTING](TESTING.md) · [PIPELINE_UPDATE](PIPELINE_UPDATE.md)

`pluto-scout` brief from planning ( 10 Oct 2026 ). The line numbers are from before the fix.

## Scout brief: OLED fitted, PRIMUS_X2_v1 shakes constantly for the whole flight, Dev mode on or off

Subsystems: OLED/user API, I2C bus, main loop/PID. Pipeline docs: subsystems/Hardware_Bus_Pipeline.md,
User_Space_API.md, Core_PID_Controller.md. None of them covers the OLED.

Key code:

- src/main/main.cpp:540 `plutoInit()` runs, then :594-595 `if (OledEnable) OledStartUpInit()`. This is the
  only place `OledInitStatus` gets set (oled_display.c:114).
- src/main/mw.cpp:1130 `OledDisplayData()` is called on every `userCode()` call, regardless of Dev mode or arm
  state.
- src/main/io/oled_display.c:331-345 `OledDisplayData`: the only gates are `OledInitStatus`, `oledMode==USER`
  and the 200 ms timer. Nothing checks ARMED.
- src/main/drivers/display_ug2864hsweg01.c:233-249: one `i2cWrite` per font byte, so 6 I2C transactions per
  character.
- src/main/mw.cpp:1285-1395 `loop()` runs `imuUpdate` → `cycleTime` → `userCode()` (:1324) →
  `pid_controller` (:1395) → `writeMotors`.
- src/main/flight/pid.cpp:831 (I-term multiplied by `cycleTime`) and :869-871 (D-term is the raw gyro difference
  with no division by dt). The active controller is pidRewrite (`pidController=1`, config.cpp:172).
- src/main/flight/imu.cpp:349-350 integrates a single gyro sample over the whole `deltaT`.

Current behaviour: before this change, the OLED code did nothing in flight. Calling `Oled_Init()` now turns on
system telemetry while armed, with Dev mode OFF. The ICM20948 gyro/acc, ICP10111 baro and AK09916 mag all sit
on I2C1 at 400 kHz together with the OLED. SPI2 is used only for flash, and no laser is enabled. Everything is
polled and blocking, so the main loop stops until each OLED write has finished.

Blocking cost per path (about 85 µs per `i2cWrite`, a 128-byte page is about 2.9 ms):

- System telemetry with Dev mode OFF: about 630 transactions, **45-60 ms once every 200 ms**, i.e. 13-17
  control cycles of 3500 µs missed, 5 times a second.
- A Dev-mode change triggers `clear_display_quick`: about 24 ms.
- User full repaint: about 25 ms. A message change (page 4 only): about 3 ms every 3 s.
- The `Oled_Update` compare loop takes about 0.1-0.15 ms when nothing changed.
- The splash screen redraws on every loop pass for 2 s at boot (about 40 ms each), but only before arming.

Analysis (ranked):

1. **I2C blocking in the loop.** The PID runs on a gyro sample about 50 ms old, then the next `cycleTime` is
   about 54 ms. That gives an I-term step about 15 times larger than normal, a D spike from 54 ms of gyro
   change, and a jump in IMU attitude. The result is a repeating 5 Hz kick. *For:* only the System-mode path
   runs with Dev mode OFF, and that is exactly the case reported. The user later confirmed "~5 twitches per
   second" and a firm screw mount.
2. **Mechanical.** OLED mass or a loose mount raises the CG and couples vibration into the IMU.
3. **Electrical/bus.** The OLED module adds pull-ups and capacitance to the IMU's I2C bus and charge-pump
   noise on 3.3 V. The I2C error count is in MSP_STATUS (serial_msp.cpp:846).
4. **Buzzer (unlikely).** It is held LOW while Dev mode is OFF.

Fix options:

- **A ( PlutoPilot.cpp only ).** Take the OLED into User mode in `plutoInit`. The system screen is lost.
- **B ( firmware gate ).** Skip `OledDisplayData` while armed. The screen freezes in flight.
- **C ( firmware rework ).** Framebuffer plus a diff flush of at most one bounded run per call, or
  interrupt/DMA I2C. **The user chose C ( framebuffer + chunked flush ), with telemetry kept live in flight.**

Doc drift:

- CHANGELOG.md:36 calls the OLED rendering "non-blocking"; it was not.
- Hardware_Bus_Pipeline.md:18 says "blocking/non-blocking" for `i2cWrite`; it only blocks.
- No pipeline doc describes `OledDisplayData` at mw.cpp:1130 running between the IMU read and the PID.

---

## Merge scout brief ( 14 Oct 2026, MagisV2 HEAD 84c326d )

`pluto-scout` brief for merging the fix from the separate copy into this repo. The decisions it asked for are in
TASKS.md.

**Merge safety: no reverts found.** At HEAD all 5 files were last committed in `fc6879b` ( 2026-06-24 ), with
banners dated 24 Jun 2026, so the separate copy started from the same base. Inverted text ( `Oled_DrawTextColor` )
and the `int16_t` coordinates from `fc6879b` are intact. Every functional hunk matches CHANGES.md. Not described
there:
- (a) about 200 lines of clang-format reflow in `display_ug2864hsweg01.c`, whitespace only. The warning baseline
  ignores line numbers ( `tools/warnings.py:31` ).
- (b) the banners are dated 14 Oct while the HISTORY entry says 10 Oct; only `oled_display.c` got a HISTORY line.

**Completeness:**
- `docs/API/OLED_API_WIKI.md` was **not** brought over: lines 57, 152 and 163 are stale for the deferred update,
  and there is no timing section.
- No Makefile change is needed ( `Makefile:195`, `:278` ), and nothing in `mw.cpp` or `main.cpp`. The only flush
  call site is present: `OledDisplayData ( )` at `mw.cpp:1130`, on every gated loop.
- Every symbol resolves ( `multiWiiFont`, `FONT_WIDTH`, `CHARACTER_WIDTH_TOTAL` in `display_ug2864hsweg01.h` ). The
  font has 135 rows, so the 32-166 clamp is right.
- The Traffic Hazard `PlutoPilot.cpp` stays out; the stock one makes no OLED calls.

**pluto-rules:**
- No ISR touched.
- One flush step is ~7 + 18 bytes at 400 kHz ( ~0.6 ms, ~17% of a 3.5 ms loop ), and only while the frame is
  dirty. The diff scan is up to 1024 compares per dirty loop.
- I2C1 is shared with the ICM20948, ICP10111 and AK09916. `LASER_*` is off on all targets.
- RAM +1024 B ( `oledSystemBuffer` ); measure here.
- There is no target gating: the code runs only when `Oled_Init ( )` sets `OledEnable`.
- With no OLED fitted, init fails on the first NACK ( one timeout at boot ), and nothing runs after that.
- If the panel stops answering, the failure score clears `OledInitStatus` after 3 failures in a row.
- Residual: `Oled_display_Clear` ( `:507` ) does not check `OledInitStatus` ( user code, disarmed only ).

**API:** the signatures are unchanged, but the behaviour of `Oled_Update`, `Oled_display_Update`, `Oled_Mode` and
`Oled_Print` changed ( deferred, static buffer, no hardware clear ). The wiki must be updated and the API version
bumped. The scout suggested 1.5.0; the user chose patch 1.4.1, behaviour only.

**Testing:** the A/B flight and the validation hover were pending in TESTING.md; the user reports both passed in
the separate copy. That build lacks `386599b` and `84c326d` ( no OLED overlap ), hence one confirmation hover here.

**Out of scope, confirmed:**
- The user-loop timer wrap: `mw.cpp:156` `userLoopTime = 0`, `:1134` signed compare, so `plutoLoop` stalls if Dev
  mode first comes on after ~35.8 min.
- The boot splash still writes the panel directly for 2 s ( `mw.cpp:1233` ).

**Doc drift:**
- CHANGELOG.md:11 and :36 call the OLED "non-blocking";
- Hardware_Bus_Pipeline.md:17-18 claims I2C DMA and non-blocking writes ( I2C is polled only );
- OLED_API_WIKI.md:57, 152, 163.
