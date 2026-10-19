# Off-centre payload lean - Changes

[README](README.md) · [TASKS](TASKS.md) · [SCOUT](SCOUT.md) · [INVESTIGATION](INVESTIGATION.md) · [TESTING](TESTING.md) · [CHANGES](CHANGES.md)

## Task 2: diagnostic getters ( 19 Oct 2026 )

| File | Change | Why |
|---|---|---|
| [pid.h](../../../../src/main/flight/pid.h) | `pidGetGyroITerm ( axis )`, `pidGetGyroResetCount ( )` declared in the internal `extern "C"` block, with the scale caveat | Read the rate integrator and count its resets from the test sketch; no public API change |
| [pid.cpp](../../../../src/main/flight/pid.cpp) | `static uint16_t gyroResetCount`, incremented at the end of `pidResetErrorGyro ( )`; the two getters after it | `errorGyroI >> 13` is axisPID units in `pidRewrite` ( ±`GYRO_I_MAX` ); out-of-range axis returns 0 |
| [PlutoPilot.cpp](../../../../PlutoPilot.cpp) | Replaced by [test/PlutoPilot_lean_diag.cpp.txt](test/PlutoPilot_lean_diag.cpp.txt) | Temporary diagnostic log ( `t F Ir Ip Rs Tp R P Ze`, worst case ~125 B / tick ); the template is restored in task 11 |

Build ( PRIMUS_X2_v1, gate ): no new `src/` warnings. Text 123804 B, data 1316, bss 19736, against the stock `a81c180`
build with the template sketch: 122652 / 1316 / 19728 ( +1152 B flash, almost all of it the test sketch; +8 B RAM ).
Whether the getters stay after the fix is decided in task 9.

Review ( pluto-reviewer ): 0 blocking, 0 should-fix, 4 notes. Applied: the `pid.h` comment now says LuxFloat returns 0.

## Task 4: no stick-low I-term reset in an ALT_HOLD flight ( 19 Oct 2026 )

| File | Change | Why |
|---|---|---|
| [mw.cpp](../../../../src/main/mw.cpp) `processRx ( )` | The stick-low reset becomes `throttleStatus == THROTTLE_LOW && ! altHoldKeepRateITerm ( )`, with a comment; HISTORY row | In ALT_HOLD a low stick is a −30 cm/s descent; zeroing the integrator there stood the craft ~10° toward the payload ( log-1 ) |
| [altitudehold.cpp](../../../../src/main/flight/altitudehold.cpp) | `altHoldKeepRateITerm ( )`: `ARMED && BARO_MODE && ! altHoldGroundIdle && ! altStickTouchdown && EstAlt >= 20 cm` ( `ALT_ITERM_KEEP_MIN_CM` ) | Keeps the reset on the ground: disarmed, throttle mode, ARM-switch ground idle, below 20 cm, after a stick-down touchdown |
| [altitudehold.cpp](../../../../src/main/flight/altitudehold.cpp) | `updateStickTouchdown ( )` and `static bool altStickTouchdown`, called at the end of `applyMultirotorAltHold ( )`. It latches once a fast stick descent ( request ≥ 15 cm/s ) has been seen moving ( `VelocityZ` ≤ −8 ) and then stopped for 300 ms; it is cleared when the request ends, at disarm or at ground idle | The 20 cm floor alone fails after a warm flight ( the baro errs high ) and on a table ( `EstAlt` is relative to the arm point ); the same arrested-descent test as `land ( )` |
| [altitudehold.h](../../../../src/main/flight/altitudehold.h) | `altHoldKeepRateITerm ( )` declared | Called from `mw.cpp` |

Build ( PRIMUS_X2_v1, gate, with the task 2 sketch ): no new `src/` warnings. Text 123804 → 124012 ( **+208 B** ), bss
19736 → 19740 ( +4 B ).

Review ( pluto-reviewer, two rounds ): round 1 had 1 should-fix ( the height floor alone ). With the user's choice it was
fixed by the touchdown latch, and round 2 confirmed it closed: 0 blocking, 0 should-fix. Every false-latch path in the
air was refuted ( start of a descent, goal / leash, flip, `isLanding`, partial then full stick ); a false latch could
only bring back today's reset. Applied: the `alt_hold_fast_change` 1 caveat ( the latch never sets; only the 20 cm
floor remains ) and the HISTORY row. Carried to task 8: log the latch and time a stick-down landing; raise
`ALT_TOUCHDOWN_MIN_REQ_CMS` to ~23 if it is ever seen latching in the air. Out of scope: the failsafe premise was wrong
( failsafe throttle 1200 is not stick-low ), and the pre-existing stale `isThrottleStickArmed` is task 12.
Noted for task 3: read `Rs` as steps between records ( it also counts at ground idle and wraps after ~22 min of stick-low
frames ); the sketch has no `n` tick counter, so a lost record shows only as a ~200 ms gap in `t`.
