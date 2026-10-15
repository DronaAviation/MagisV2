# Laser ToF Init Conflict ( down-laser vs XRanging check )

[README](README.md) · [TASKS](TASKS.md) · [SCOUT](SCOUT.md) · [INVESTIGATION](INVESTIGATION.md)

| | |
|---|---|
| **Status** | **Closed** in `edfbd61` ( 14 Oct 2026 ): fixed by [tof-single-pipeline](../tof-single-pipeline/README.md) ( FW 3.12.0 ) and committed with it |
| **Mode** | `check` ( investigation only; the fix follows with `/pluto-grill fix laser-tof-init-conflict` ) |
| **Branch** | `BugFix-June26` |
| **Target** | `PRIMUS_X2_v1` |
| **Origin** | User report: with `LASER_TOF` + `LASER_ALT` defined and `initObjectAvoidance ( ... )` / `XRanging.init ( )` in user code, **no laser works**: neither the down ( altitude ) laser nor any of the 4 side lasers. Each pipeline works on its own. |

**Problem.** Two VL53L0X pipelines share I2C1 and the default address 0x29, but they are not aware of each other.
The altitude laser ( `ranging_init ( )`, [main.cpp](../../../../src/main/main.cpp) :518 ) has no XSHUT line and stays
at 0x29. The XRanging / object-avoidance lasers ( `xRangingInit ( )`, :600 ) are brought up one by one through their
XSHUT pins and moved to 0x2A+. The two inits are separate: there is no central owner of the bus addresses, the
XSHUT pins or the driver state.

**Fitted hardware.** 1 down-laser VL53L0X ( no XSHUT today; a port is planned and its API is to be discussed ) +
4 side VL53L0X ( LEFT / RIGHT / FRONT / BACK ), all on I2C1 with the IMU and baro.

**Cause ( confirmed in the code, [INVESTIGATION.md](INVESTIGATION.md#findings) ).** The first enabled side laser is released
while the down-laser still sits at 0x29. Its init and its address change reach both chips, so the down-laser moves to
0x2A with LEFT. The first down-laser read then times out at the empty 0x29, and the shared `Global_Status` in
[ranging_vl53l0x.cpp](../../../../src/main/drivers/ranging_vl53l0x.cpp) stores −20 for good. Every VL53L0X read
checks that flag, so all five lasers stop. Also confirmed: side lasers awake during the down-laser's setup ( H3 );
side-laser data able to reach altitude hold through shared globals once H1 is fixed ( H2, a flight risk ); blocking
side-laser reads ( H5 ); F1-F5; doc drift D1-D12.

**Chosen direction ( user ).** One ToF pipeline behind `XRanging`: `XRanging.initAlt ( gpio )` / `initAlt ( )` replaces
`#define LASER_TOF` + `LASER_ALT`; one bring-up at main.cpp:600; per-laser state; paced, cached side-laser reads;
`getRange ( DOWN )`. VL53L1X stays a define ( its API is the next topic ).

**Outcome.** Findings complete ( task 6 ). The fix is planned in [tof-single-pipeline](../tof-single-pipeline/README.md) ( 14 Oct 2026 ).

**Out of scope.** Any code change; bench tests ( analysis from code only ); VL53L1X / `LASER_TOF_L1x` ( H4 recorded
in one line only ); estimator fusion ( laser / baro handover ); object-avoidance behaviour ( thresholds, stick
response ); the XRanging pin overlaps with SPI2 flash / SWDIO / WS2811 ( known to the user ).

**Open items.**

- **Direction chosen ( 14 Oct 2026 ):** option B, one ToF pipeline behind XRanging; `XRanging.initAlt ( gpio )` / `initAlt ( )` replaces `#define LASER_TOF` + `LASER_ALT` ( D1-D9 in [INVESTIGATION.md](INVESTIGATION.md#chosen-direction--user-14-oct-2026-) ). XSHUT pin settled in the fix topic: **GPIO_5**, since the FRONT LED is no longer wired to it ( an XSHUT claim beats an LED ).
