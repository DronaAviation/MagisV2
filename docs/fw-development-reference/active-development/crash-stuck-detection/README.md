# Crash Detection: Stuck / Pinned / Vertical Impacts

| | |
|---|---|
| **Status** | **Planned** ( noted 13 Oct 2026; not started ) |
| **Branch** | not started |
| **Target** | `PRIMUS_X2_v1` |
| **Origin** | [crash-detect-false-trigger](../crash-detect-false-trigger/README.md), whose task 5 set the lateral impact limits to 5000 / 8000 LSB ( 1.22 / 1.95 g ) |

**Goal.** Disarm in the crash cases that neither the lateral accel limits nor the 70° rule can see. Today the
pilot's ARM switch is the only protection in these cases.

| Case | Why `failsafeOnCrash ( )` misses it |
|---|---|
| Pinned upright against a wall, motors pushing | At ≤ 20° tilt the accel reads only g·sin θ ≈ 0.34 g once the contact spike is over; the angle stays under 70° |
| Ceiling hit, or held against the ceiling | The force is on Z; only X / Y are checked |
| Upright drop onto the floor ( power sag or fall while level ) | The force is on Z |
| Caught in a curtain or net, or grabbed by a hand | Low acceleration and no tip-over |
| Resting tilted at 20-69° on an edge | Gravity is under 1 g, so it cannot reach 1.22 g, and the angle stays under 70° |
| Any crash outside ANGLE mode | The detector runs only in ANGLE mode ( already true before this work ) |

**Why an accel limit cannot fix it.** In the crash-detect-false-trigger logs, survivable bumps and contacts
that flipped the craft both peaked at 1-1.3 g raw. A stuck craft reads less than either. A different signal
is needed. Candidates, none measured yet:
- motor output near saturation with no matching attitude or velocity change;
- the attitude-error integral wound up for longer than a set time;
- a Z impact term that does not interfere with `LAND`'s own touchdown check ( `LAND_IMPACT_ACC`, `command.cpp:212` ).

**Evidence:** [crash-detect-false-trigger TESTING.md](../crash-detect-false-trigger/TESTING.md), tests 1 and 2.

**Next action:** plan with `/pluto-grill feature crash-stuck-detection` when it is picked up.
