# RcCommand_Get API - Pipeline update

[README](README.md) · [TASKS](TASKS.md) · [SCOUT](SCOUT.md) · [INVESTIGATION](INVESTIGATION.md) · [CHANGES](CHANGES.md) · [TESTING](TESTING.md) · [PIPELINE_UPDATE](PIPELINE_UPDATE.md)

Staged text for the pipeline docs. Apply at commit ( `pluto-commit` ); do not edit `fw-architecture-pipeline/` before.

## 1. `fw-architecture-pipeline/subsystems/User_Space_API.md`, section "Primary Functions"

**Insert** after the bullet that ends `` `docs/API/FC_DATA_API_WIKI.md` ( API 1.8.2 ). `` ( currently line 38, the last bullet
before "## Data Flow & Boundaries" ):

```markdown
- `RcCommand_Get ( )` ( `API/RC-Interface.h` ): a pointer to a file-static `int16_t [ 4 ]` refreshed on each call
  ( `RC-Interface.cpp` `rcCommandSnapshot` ): roll / pitch / yaw as `1500 + rcCommand` ( the same as
  `RcCommand_Get ( channel )` ), throttle as `rcCommand [ THROTTLE ]`, 1100-2000. User code runs after `annexCode ( )`
  and before `applyUserRcOverride ( )` and `applyAltHold ( )`: roll / pitch / yaw are the pilot's command, not its own
  `RcCommand_Set` override; throttle is the curve of `rcData [ THROTTLE ]` ( the stick, or the landing throttle, the
  altitude-limit cap, failsafe 1200 or a throttle override written there ), never the altitude-hold throttle. Yaw is inverted ( `yaw_control_direction` 1: yaw right
  reads below 1500 ). `RcData_Get ( )` ( array ) still returns NULL and `RcCommand_Get ( RC_THROTTLE )` reads 2600-3500
  ( known gaps ). Wiki `docs/API/RC_INTERFACE_API_WIKI.md` ( API 1.8.3 ).
```

No other line of the doc changes. No flowchart change: no call path moved ( the getter only reads `rcCommand` ).

## 2. Hardware maps

No change: no pin, DMA channel or timer is touched.

## 3. At commit ( for `pluto-commit` )

- `CHANGELOG.md` ( Fixed ): `RcCommand_Get ( )` was an undefined reference ( defined as `RcCommand_get` ) and, under the
  name, returned NULL ( a local array's address ); it now links and returns the four live commands. API 1.8.3,
  behaviour only. Documentation: new `docs/API/RC_INTERFACE_API_WIKI.md`; `RC-Interface.h` `RcCommand_Get` comments
  corrected.
- Makefile: `FW_Version` 3.15.5, `API_Version` 1.8.3.
- `tools/warnings-baseline.json`: the `RC-Interface.cpp` `-Wreturn-local-addr` entry for `rcCommandArray` is fixed; the
  gate reports 109 fixed entries in total ( 108 carried from earlier commits ). A refresh drops them; if done, the
  commit message must say so. Never refresh to hide a new warning.
