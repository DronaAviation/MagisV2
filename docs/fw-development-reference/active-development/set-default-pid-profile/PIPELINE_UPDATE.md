# Set default PID profile - Pipeline update

[README](README.md) · [TASKS](TASKS.md) · [SCOUT](SCOUT.md) · [INVESTIGATION](INVESTIGATION.md) · [CHANGES](CHANGES.md) · [TESTING](TESTING.md) · [PIPELINE_UPDATE](PIPELINE_UPDATE.md)

Staged text for the pipeline docs. Apply at commit ( `pluto-commit` ); do not edit `fw-architecture-pipeline/` before.
Every line reference below was checked against the working tree on 14 Nov 2026 ( see the compliance table at the end ).

## 1. `fw-architecture-pipeline/subsystems/Core_PID_Controller.md`

### 1a. Section "Source Files"

**Replace** the three bullets of the section with:

```markdown
- **Controller Logic**: `src/main/flight/pid.cpp`, `src/main/flight/pid.h`
- **Default Gains**: `src/main/flight/pid_defaults.cpp` ( `pidProfileDefaultGains ( )` )
- **Profile Config**: `src/main/config/config_profile.h`
- **User API Hook**: `src/main/API-Src/FC-Config-PID.cpp` ( `get_PIDProfile`, `set_PIDProfile`, `setDefault_PIDProfile` )
```

### 1b. New section, inserted after "Key Data Structures" and before "Primary Functions"

```markdown
## Default gains and the API reset
- `pidProfileDefaultGains ( pidProfile_t * )` ( `pid_defaults.cpp:38`, declared `pid.h:111` ) is the single source of the
  default `P8` / `I8` / `D8` for all eleven `pidIndex_e` slots. `resetPidProfile ( )` ( `config.cpp:175` ) calls it on a
  config reset ( `config.cpp:179` ) and sets the rest of the profile itself: `pidController`, the float gains, the cut
  frequencies, `A_level` / `H_level` / `H_sensitivity` and the GTUNE limits.
- `setDefault_PIDProfile ( )` ( `FC-Config-PID.cpp:155` ) restores only the five slots the PID API exposes,
  `apiSlot [] = { PIDROLL, PIDPITCH, PIDYAW, PIDALT, PIDUSER }` ( `FC-Config-PID.cpp:157` ), from a stack copy filled
  by `pidProfileDefaultGains ( )`. `PIDPOS`, `PIDPOSR`, `PIDNAVR`, `PIDLEVEL`, `PIDMAG` and `PIDVEL` are never
  written. The change is RAM only, and gains are read live every loop, so a call while armed steps the tuning in flight
  ( header comment, `FC-Config.h:95` ). Before FW 3.15.7 the function looped over all eleven slots with five-entry
  tables: it set ALT P8 to a stale 100 and overwrote the hidden gains, `PIDLEVEL` and `PIDVEL` included
  ( `active-development/set-default-pid-profile/` ).
```

### 1c. Section "Primary Functions", the `pidRewrite()` bullet

**Replace** `( \`config.cpp:369\`; forced to 0 only with \`retarded_arm\` )` with
`( \`config.cpp:331\`; forced to 0 only with \`retarded_arm\` )`.

The line moved because the gain block left `config.cpp`; the statement is unchanged.

No flowchart change: no call path in the control loop moved. The two new edges, `resetPidProfile ( )` to
`pidProfileDefaultGains ( )` and `setDefault_PIDProfile ( )` to `pidProfileDefaultGains ( )`, are direct calls
( `config.cpp:179`, `FC-Config-PID.cpp:160` ) and the doc has no diagram that shows either caller.

## 2. `fw-architecture-pipeline/subsystems/Mixer_Pipeline.md`, "Motor Clipping" bullet

**Replace** `(default 1100, \`config.cpp:278\`)` with `(default 1100, \`config.cpp:240\`)`.

Line shift only ( `escAndServoConfig->minthrottle = 1100` ).

## 3. `fw-architecture-pipeline/subsystems/User_Space_API.md`, the RC-loss sentence

**Replace** `the default for AUX, config.cpp:479;` with `the default for AUX, config.cpp:441;`.

Line shift only ( `channelFailsafeConfiguration->mode = ... RX_FAILSAFE_MODE_HOLD` ).

## 4. Not changed, and why

| Doc | Why not |
|---|---|
| `docs/API/` wiki | No FC-Config wiki exists; the decision ( TASKS decisions log ) is the header comment only. |
| `User_Space_API.md`, PID API | The doc does not describe the PID API today; adding it was not asked for. |
| `DMA_MAP.md` / `TIMER_MAP.md` / `PIN_MAP.md` | No pin, DMA channel or timer touched. |
| `dev-guide/`, CLAUDE.md | The gotcha is narrow and now stated in `FC-Config.h`; no new invariant. |
| `.claude/skills/pluto-rules/references/invariants.md` ( `pid.h:89` ) | Still correct: `pid.h` grew below line 89 only ( line 111 ). |
| `config.cpp:176` in `Core_PID_Controller.md` | Still correct: `pidProfile->pidController = 1;` is above the moved block. |

## Compliance check ( by hand )

The `spec-to-code-compliance` skill is not available in this session, so each staged statement was checked against the
source directly on 14 Nov 2026:

| Staged statement | Source |
|---|---|
| `pidProfileDefaultGains ( )` defined at `pid_defaults.cpp:38`, all eleven slots | `src/main/flight/pid_defaults.cpp:38-79`, `P8 [ PIDROLL ]` .. `D8 [ PIDUSER ]` ( line 78 ) |
| declared at `pid.h:111` | `src/main/flight/pid.h:111` |
| `resetPidProfile ( )` at `config.cpp:175`, call at `:179` | `src/main/config/config.cpp:175`, `:179` |
| `resetPidProfile ( )` still sets controller, float gains, cut frequencies, levels, GTUNE | `config.cpp:176` ( controller ), `:181-199` ( `yaw_p_limit` .. `pid5_oldyw` ), `:201` ( `#ifdef GTUNE` ) |
| `setDefault_PIDProfile ( )` at `FC-Config-PID.cpp:155`, `apiSlot` at `:157` | `src/main/API-Src/FC-Config-PID.cpp:155`, `:157` |
| hidden slots never written | the loop runs over `apiSlot` only ( `:162-168` ); host simulation, `TESTING.md` ( 6 slots x 3 gains x 2 markers ) |
| header comment at `FC-Config.h:95` | `src/main/API/FC-Config.h:75-95` ( comment `75-94`, declaration `95` ) |
| `pid_at_min_throttle` now `config.cpp:331` | `mixerConfig->pid_at_min_throttle = 1;` ( HEAD line 369 ) |
| `minthrottle` now `config.cpp:240` | `escAndServoConfig->minthrottle = 1100;` ( HEAD line 278 ) |
| AUX failsafe mode now `config.cpp:441` | `channelFailsafeConfiguration->mode = ...` ( HEAD line 479 ) |
| "Before FW 3.15.7" | the version this fix ships in ( task 8 bump from 3.15.6 ) |
