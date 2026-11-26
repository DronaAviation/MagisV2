# Flight status enum map - Pipeline update

[README](README.md) · [TASKS](TASKS.md) · [SCOUT](SCOUT.md) · [INVESTIGATION](INVESTIGATION.md) · [CHANGES](CHANGES.md) · [TESTING](TESTING.md) · [PIPELINE_UPDATE](PIPELINE_UPDATE.md)

Staged text for the pipeline docs. Apply at commit ( `pluto-commit` ); do not edit `fw-architecture-pipeline/` before.

## 1. `fw-architecture-pipeline/subsystems/User_Space_API.md`, section "Primary Functions"

**Replace** the bullet that starts `` - `Command_Arm ( )`, `Command_TakeOff ( cm )` `` and ends
`` `docs/API/FC_CONTROL_API_WIKI.md` ( API 1.7.0 ). `` with:

```markdown
- `Command_Arm ( )`, `Command_TakeOff ( cm )`, `Command_Land ( speed )`, `Command_Flip ( BACK_FLIP )`,
  `Command_DisArm ( )` ( `API/FC-Control.h` ): arm and fly the drone from user code. Wiki
  `docs/API/FC_CONTROL_API_WIKI.md` ( API 1.8.4 ).
- `FlightStatus_Get ( )` / `FlightStatus_Check ( status )` ( `API/FC-Control.h` ): the flight status from
  `flightIndicatorFlag`, one bit per internal `FlightStatus_e` ( `config/runtime_config.h` ); the lowest set bit is the
  most important ( `leastSignificantBit`, `common/maths.cpp` ). The internal enum and the public `flightstatus_e` are
  numbered differently at 2-4 ( internal Crash 2, Low_battery 3, LowBattery_inFlight 4; public FS_LOW_BATTERY 2,
  FS_INFLIGHT_LOW_BATTERY 3, FS_CRASHED 4 ), so `FC-Control-Status.cpp` translates by name through
  `flightStatusMap [ ]`, both ways; `MSP_FLIGHT_STATUS` ( `io/serial_msp.cpp` ) maps the same bit by name to
  `AppFlightStatus_e`. Never reorder the internal enum: its bit order is the priority, and `MSP_STATUS` sends the raw
  bits. Crash is cleared about 0.3 s after the crash disarm once level ( `mw.cpp` `annexCode ( )` ). Wiki section
  "Flight status" in `docs/API/FC_CONTROL_API_WIKI.md` ( API 1.8.4 ).
```

No other line of the doc changes. No flowchart change: no call path moved ( the API functions only read
`flightIndicatorFlag` ).

## 2. Hardware maps

No change: no pin, DMA channel or timer is touched.

## 3. At commit ( for `pluto-commit` )

- `CHANGELOG.md` ( Fixed ): `FlightStatus_Get ( )` returned 2 ( `FS_LOW_BATTERY` ) after a crash disarm instead of 4
  ( `FS_CRASHED` ) ( API test T05 / B3 ), and swapped the two battery statuses; `FlightStatus_Check ( )` tested the
  wrong flag for the same three. Both now translate the internal status by name; the app ( MSP ) was already right.
  API 1.8.4, behaviour only. Documentation: new "Flight status" section in `docs/API/FC_CONTROL_API_WIKI.md`;
  `FC-Control.h` comments of both functions rewritten.
- Makefile: `FW_Version` 3.15.6, `API_Version` 1.8.4.
- `tools/warnings-baseline.json`: no change from this topic ( no warning added or fixed ). Never refresh to hide a new
  warning.
