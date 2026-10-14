# User-Loop Timer Wrap

| | |
|---|---|
| **Status** | **Planned** ( noted 14 Oct 2026; not started ) |
| **Branch** | not started |
| **Target** | all |
| **Origin** | [oled-flight-wobble](../oled-flight-wobble/README.md) merge scout ( 14 Oct 2026 ) |

**Problem.** `userLoopTime` starts at 0 ( `mw.cpp:156` ), and the user-loop gate at `mw.cpp:1134` tests
`( int32_t ) ( now - userLoopTime ) >= 0`. If Developer Mode first comes on after more than ~35.8 min of uptime,
or stays off that long, the difference reads as negative. `plutoLoop ( )` then does not run until `micros ( )`
wraps, at ~71.6 min.

**Likely fix.** Re-seed `userLoopTime = now` when Developer Mode turns on ( or on every `onLoopStart ( )` ), so
the comparison always works over a short interval. Check every other user of `userLoopTime`.

**Next action:** plan with `/pluto-grill fix user-loop-timer-wrap` when it is picked up.
