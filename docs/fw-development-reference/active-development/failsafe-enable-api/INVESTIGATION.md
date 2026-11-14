# Failsafe_enable API - investigation

[README](README.md) · [TASKS](TASKS.md) · [SCOUT](SCOUT.md) · [INVESTIGATION](INVESTIGATION.md)

## Problem

From the testing team, API test **T07 ( B2 )**: *"enabling one failsafe switches the other two off"*. The API under
test is `void Failsafe_enable ( failsafe_e FAILSAFE )` from
[API/FC-Config.h:280](../../../../src/main/API/FC-Config.h#L280), with
`failsafe_e = { LOW_BATTERY 0, INFLIGHT_LOW_BATTERY 1, CRASH 2, ALL 3 }` ( FC-Config.h:46-51 ). No written T07
procedure exists in the repo; the report is the only evidence, and the code confirms it on its own.

## Evidence

The defect is visible in the source, so no log or flight is needed to establish it.

`FC-Config-Setpoints.cpp:213-225` **at `3be9093`** ( the pre-fix body; the fixed function is at
[:213-236](../../../../src/main/API-Src/FC-Config-Setpoints.cpp#L213) ):

```c
void Failsafe_enable ( failsafe_e FAILSAFE ) {
  bool enableAll = ( FAILSAFE == ALL );
  fsLowBattery         = enableAll || ( FAILSAFE == LOW_BATTERY );
  fsInFlightLowBattery = enableAll || ( FAILSAFE == INFLIGHT_LOW_BATTERY );
  fsCrash              = enableAll || ( FAILSAFE == CRASH );
}
```

Each of the three flags is **assigned**, not set. For any argument other than `ALL`, exactly one assignment is `true`
and the other two are `false`, whatever the flags held before.

`Failsafe_disable` ( `3be9093` :227-243, now [:238-253](../../../../src/main/API-Src/FC-Config-Setpoints.cpp#L238), unchanged ) is correct by comparison: `fsX = disableAll || ( FAILSAFE == X ) ? false : fsX`. `||`
binds tighter than `?:`, so it parses as `( a || b ) ? false : fsX` and keeps the flags it was not asked to change.

The flags are plain `bool`s defined in
[API-Utils.cpp:122-124](../../../../src/main/API-Src/API-Utils.cpp#L122), all initialised `true`, declared `extern` at
[API-Utils.h:111-113](../../../../src/main/API/API-Utils.h#L111). Nothing else in the firmware writes them: they are
not reset on disarm or at Developer Mode off, only by a reboot.

## Cause and consequences

**Cause.** Assignment where the documented contract is a set. The header
( [FC-Config.h:270-279](../../../../src/main/API/FC-Config.h#L270) ) states: *"If the FAILSAFE type is ALL, all
failsafes are enabled. Otherwise, only the specified failsafe will be enabled while others remain unchanged."* The
implementation does not match its own specification.

**Consequence 1: the low-battery landing can be switched off by enabling crash detection.**
`fsInFlightLowBattery` gates the `BATTERY_OK` -> `BATTERY_WARNING` transition in `updateBatteryState ( )` at
[battery.cpp:798](../../../../src/main/sensors/battery.cpp#L798). With the flag false the level never leaves OK, so:

- no `Low_battery` / `LowBattery_inFlight` flight-status bit and no beeper,
- `BatteryWarningMode` stays 0 and `batteryCriticalConfirmed ( )` is never true,
- the auto-land at critical battery ( `mw.cpp:1306` ) never starts,
- `mwArm ( )` never refuses to arm on a flat pack ( `mw.cpp:625` ).

A user who writes `Failsafe_enable ( CRASH )` in `plutoInit ( )`, intending to turn crash detection on, silently loses
the behaviour the CLAUDE.md flight invariant guarantees ( at critical battery the firmware lands, and arming is refused
until the pack is changed ).

**Consequence 2: crash detection can be switched off by enabling a battery failsafe.** `fsCrash` gates
`failsafeOnCrash ( )` at [failsafe.cpp:365](../../../../src/main/flight/failsafe.cpp#L365), so after
`Failsafe_enable ( LOW_BATTERY )` or `( INFLIGHT_LOW_BATTERY )` a crash no longer disarms.

**Gate timing, which matters for any test.** The `fsInFlightLowBattery` gate only guards the step out of
`BATTERY_OK`. Clearing the flag once `WARNING` or `CRITICAL` has been reached does not step the level back down, so the
defect is only observable if the flag is cleared *before* the battery falls.

## Secondary defect: LOW_BATTERY is inert

`fsLowBattery` is written by the two API functions and read by nothing. Its only consumer was
`annexCode ( )`'s `else if ( fsLowBattery ) set_FSI ( Low_battery )`, a ground warning at vbat <= 3.4 V, deleted in
commit `88f594f` ( the battery-management rewrite ), which moved the gate onto `fsInFlightLowBattery`. So
`Failsafe_enable ( LOW_BATTERY )` and `Failsafe_disable ( LOW_BATTERY )` have no effect on flight behaviour today, and
`Failsafe_enable ( LOW_BATTERY )`'s only observable effect is the bug above ( it clears the other two ).

Giving `LOW_BATTERY` a real function ( for instance gating the warning level and beeper while `INFLIGHT_LOW_BATTERY`
gates critical and the auto-land ) is a design decision about battery safety, not a bug fix. It is out of scope here
and recorded as a finding; this topic documents the flag as inert and kept for compatibility.

## Q&A that shaped the plan

| Question | Answer |
|---|---|
| Mode | `fix`. The cause is proven by reading the code, so no separate `check` topic. |
| Target | `PRIMUS_X2_v1` ( `selected_target` in `plutoide.ini` ). |
| How it is verified | No hardware test from us. A host truth-table check over every enable / disable sequence, plus a written code-flow trace to both consumers. The test team re-runs T07 themselves. |
| Is the host harness committed | No. It lives in the scratchpad and reuses the in-repo `tools/sim/stub/` tree, so no new repo files. |
| `Failsafe_disable` | Left alone. It is correct, and the smallest diff is preferred. |
| `LOW_BATTERY` | Fixed as part of the enable semantics, and documented as having no effect. Not wired to anything. |
| Versions | API 1.8.7 -> 1.8.8, FW 3.15.9 -> 3.15.10 at commit, as the sibling API fix topics did. |
