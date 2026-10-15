# User Space API (`PlutoPilot.cpp`)

## Overview
The `PlutoPilot` developer sandbox allows custom user C++ code to intercept the main flight loop. Developers can securely override controls, read live sensor variables, and trigger external LEDs or displays—all without breaking the real-time safety and timing of the core PID loops.

## Source Files
- **Sandbox Definition**: `PlutoPilot.cpp`, `PlutoPilot.h` (Root level developer files)
- **API Interfaces**: `src/main/API-Src/*` (e.g., `FC-Control-Command.cpp`, `FC-Data-Sensors.cpp`)
- **API Headers**: `src/main/API/*` (e.g., `FC-Control.h`, `Peripherals.h`)
- **Laser ranging**: `src/main/API/XRanging.h`, `src/main/API-Src/XRanging.cpp`, behind it `src/main/drivers/tof_manager.cpp`

## Key Data Structures
- `rcCommand[4]`: The user API often overwrites this array (Pitch, Roll, Yaw, Throttle) to fly the drone autonomously.
- `Flight_State_e`: Enumeration that the API uses to determine if the drone is Armed, in Failsafe, or Grounded.
- `Control_Command_t`: The wrapper structure the API uses to push new commands (like `TakeOff`, `Land`, or `Flip`) to the flight controller queue.

## Primary Functions
- `plutoInit()`: Called once during boot by `init()` ( main.cpp:534 ), after `SYSTEM_STATE_READY` and before the motor timers and the laser bring-up. Used by the developer to configure GPIOs, external sensors, or set up API timers. `XRanging` calls here only record requests; the lasers start after `plutoInit()` returns.
- `plutoLoop()`: The continuously executing user function. Called directly by the main control loop.
- `Command.set(CommandType)`: Exposed API function used inside `plutoLoop()` to trigger complex maneuvers or state changes natively.
- `Estimate.get(State)`: Exposed API function to pull live `EstAlt`, `EstVel`, or `Pitch/Roll` angles safely.

## Data Flow & Boundaries
- **Time Slice Strictness**: Because `plutoLoop()` executes within the high-frequency control loop, user code *must not block*. The use of `delay()` is explicitly forbidden, as it would cause the drone to instantly crash by halting the PID calculations.
- **Override Priority**: If the Failsafe mechanism activates, the firmware explicitly ignores `rcCommand` overrides originating from `plutoLoop()`. Failsafe takes ultimate precedence over the API.
- **Rate Limiting**: To prevent API logic from running unnecessarily fast (e.g., polling an ultrasonic sensor 1000 times a second), developers use the `Monitor.timer` API to schedule tasks sequentially.
- **Landing owns the throttle**: while any `LAND` runs ( `Command_Land`, RX loss or the critical-battery auto-land ), the landing throttle is re-applied after user code, so a throttle `RcCommand_Set` has no effect; the battery auto-land also replaces `Command_TakeOff` / `Command_Flip`.
- **Battery**: `Bms_Get` has `SoC`, `Warning_Level` and `Resistance` ( API 1.4.0 ); wiki `docs/API/BMS_API_WIKI.md`.
- **Link drop stops user code**: `plutoLoop ( )` runs only while `rxIsReceivingSignal ( )` is true. With `Rx_ESP`
  that is false **200 ms after the last `MSP_SET_RAW_RC` frame**. The firmware then runs `onLoopFinish ( )`,
  clears every user RC override and re-runs `onLoopStart ( )` when frames return. A Wi-Fi stall therefore
  restarts user code long before failsafe acts ( ~2.5 s ).
- **OLED is deferred**: `Oled_Update ( )` / `Oled_display_Update ( )` only queue the frame. The firmware sends the
  changed bytes to the panel at most 16 per loop, so a full screen appears in ~0.25 s. A buffer passed to
  `Oled_display_Update ( )` must stay valid ( `static` or global ). `Oled_Update ( )` works in `User` mode only, and
  `Oled_Mode ( )` starts from a blank frame. Wiki: `docs/API/OLED_API_WIKI.md`.
- **Laser ranging**: `XRanging` requests are made in `plutoInit ( )` and fixed at boot. Reads return cached values.
  Using XRanging switches SPI2 off. See **XRanging** below; wiki `docs/API/XRANGING_API_WIKI.md`.

```mermaid
flowchart TD
    Start([User Code Hook]) --> DevCheck{Is Dev Mode AUX High?}
    
    DevCheck -- No --> FinishCheck{Was it just turned off?}
    FinishCheck -- Yes --> RunFinish["Run onLoopFinish()"]
    FinishCheck -- No --> End
    
    DevCheck -- Yes --> TimeCheck{Has userLoopInterval passed?}
    TimeCheck -- No --> ApplyOld[Apply existing RC overrides]
    
    TimeCheck -- Yes --> StartCheck{Is this the first loop?}
    StartCheck -- Yes --> RunStart["Run onLoopStart()"]
    StartCheck -- No --> RunPluto
    RunStart --> RunPluto["Run plutoLoop()"]
    
    RunPluto --> ReadSensors[Process FC_Data_Get calls]
    ReadSensors --> ProcessLogic[Process User C++ Logic]
    ProcessLogic --> Overrides[Process User rcCommand Overrides]
    
    Overrides --> ApplyOld
    ApplyOld --> End([Return to firmware Loop])
    RunFinish --> End
```

## XRanging ( laser ranging and object avoidance )

Up to six VL53L0X lasers on I2C1: four side lasers, EXTERNAL and the downward altitude laser ( `DOWN` ). Everything
goes through the `XRanging` object ( `API/XRanging.h` ). Full reference, examples and wiring rules:
[XRANGING_API_WIKI.md](../../../API/XRANGING_API_WIKI.md) ( API 1.5.0 ).

| Call | Where | What it does |
|---|---|---|
| `initAlt ( gpio )` / `initAlt ( )` | `plutoInit ( )` only | Requests the down-laser and the altitude-laser fusion ( the estimator then corrects on the laser below 160 cm ). `gpio` is the pin wired to its XSHUT; `initAlt ( )` is for a down-laser with no XSHUT wire. Returns `false` when refused. |
| `init ( )` / `init ( laser, threshold_cm )` | `plutoInit ( )` | Enables the four side lasers ( threshold 20 cm ), or one side laser with its own threshold. |
| `initObjectAvoidance ( dist_cm, l1 … l4 )` | `plutoInit ( )` | Enables up to four side lasers and sets the avoidance distance. OA stays off until `enableOA ( )`. |
| `enableOA ( )` / `disableOA ( )` | any time | Turns object avoidance on or off; `disableOA` also turns the laser LEDs off. |
| `getRange ( laser )` | any time | The cached range in mm. −100 when the laser has no valid sample or is not running, −1 when it was not enabled. `DOWN` is allowed. |
| `isTriggered ( laser )` | any time | A side laser's range is below its threshold; drives that laser's LED. |
| `getStatus ( laser )` | any time | `LASER_OK`, `LASER_OUT_OF_RANGE`, `LASER_BOOT_FAILED`, `LASER_I2C_ERROR` or `LASER_NOT_ENABLED`, per laser, `DOWN` included. |

Rules:
- **Requests are fixed at boot.** `xRangingInit ( )` brings every requested laser up once, after `plutoInit ( )`
  ( see `Main_Initialization.md` ). A laser enabled later is not started. A laser that fails, at boot or in flight, is
  not retried until power-off; the others carry on.
- **Reads are cached.** `UPDATE_LASER_TOF_TASK` polls the chips: DOWN every 33 ms, one side laser per 10 ms in
  round-robin ( each of four side lasers every ~40 ms ). `getRange`, `isTriggered` and `getStatus` read the cached
  entry and make no I2C call, so they are safe in `plutoLoop ( )`.
- **Object avoidance runs from `annexCode ( )`**, every 100 ms while `enableOA ( )` is on, whether or not user
  code runs. It overrides roll and pitch ( `RC_ARRAY`, pilot authority 0 through `userRCassert`; the pilot's stick is
  blended in at 30 % / 20 % ). A laser that is not OK or OUT_OF_RANGE counts as "no obstacle".
- **`initAlt` is refused** in these cases:
  - outside `plutoInit ( )`, i.e. once the bring-up has run;
  - in a VL53L1X ( `LASER_TOF_L1x` ) build;
  - for an invalid pin, or any side laser's XSHUT pin ( GPIO_7, 9, 16, 15, 14 );
  - for an RGB-LED slot or UART2 pin ( GPIO_1, 3, 6, 10, 11, 12, 13, 16, 17, 18 ): those drivers can take the pin after start-up;
  - when the pin's ADC or PWM alias is in use ( GPIO_5 = ADC_2 / PWM_4 ). This is checked again after `plutoInit ( )`;
    a request refused there leaves the firmware on the baro.

  A refused call keeps an earlier accepted request. `init` and `initObjectAvoidance` take side lasers only.
- **The down-laser's XSHUT pin is claimed.** After the bring-up starts, `Peripheral_Init` / `Peripheral_Write` on that
  pin, and `Peripheral_Init` on its ADC or PWM alias, are ignored. No LED write reaches it either: on this board
  GPIO_5 is FRONT's LED and the down XSHUT, so with `initAlt ( GPIO_5 )` FRONT has no LED.
- **Every fitted laser must be enabled.** A fitted side laser that is not enabled stays awake at 0x29. Every release
  after it is then refused ( `LASER_BOOT_FAILED`, step `BUS_BUSY` ), the down-laser included.
- **SPI2 is off whenever XRanging is used.** GPIO_5-8 are SPI2's pins. SPI calls made in `plutoInit ( )` work; from
  the bring-up on, the SPI API does nothing ( `SPI_Read` returns 0 ) and the on-board flash reports size 0.
- **No side laser reaches altitude hold.** Only the DOWN entry feeds the estimator.

```mermaid
flowchart TD
    PI([plutoInit]) --> Req[initAlt / init / initObjectAvoidance<br/>record requests only]
    Req --> XR[xRangingInit at boot<br/>single bring-up]
    XR --> Tab[(tof_manager table<br/>one entry per laser)]

    Task([UPDATE_LASER_TOF_TASK]) --> PD[tofManagerPollDown<br/>DOWN every 33 ms]
    Task --> PS[tofManagerPollSides<br/>one side laser per 10 ms]
    PD --> Tab
    PS --> Tab

    Tab --> Get[getRange / getStatus / isTriggered<br/>cached, no I2C]
    PL([plutoLoop]) --> Get
    AC([annexCode]) --> OA[applyObjectAvoidance<br/>every 100 ms, if enableOA]
    OA --> Get
    OA --> RC[RC_ARRAY roll / pitch<br/>userRCassert]
    Tab -- DOWN entry only --> Alt[altitude hold<br/>tofDown accessors]
```
