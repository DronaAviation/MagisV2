# Testing

[README](README.md) · [CHANGES](CHANGES.md) · [TESTING](TESTING.md)

Not flown yet. Log `AltHold`, `EstAlt`, `VelocityZ`, `getSetVelocity ( )`,
throttle and `Arm` (see the `flight-test` skill). Keep each tick under about
250 bytes.

## Plan

| # | Test | Pass if |
|---|---|---|
| 1 | Arm on the throttle stick, wait 15 s with the stick down, then give full stick | Takes off as promptly as an immediate take-off; the integrator is about 0 at release |
| 2 | Hover, then full stick up for 3 s and centre | Climb builds up smoothly to about 40 cm/s, no jump; stops within about 10 cm of the centring point without coming back down |
| 3 | Same test with full stick down | About 30 cm/s, stops smoothly |
| 4 | Small stick movements just outside the dead zone | Slow creep, no bump |
| 5 | Take-off command | Reaches about 120 cm in about 3 s, cruising at about 60 cm/s, braking smoothly with no overshoot and no slow crawl at the top |
| 6 | `Command_Land` | Lands and disarms. Note the descent rate and the time to disarm |
| 7 | Ceiling set (`max_altitude`), climb into it | Stops below the ceiling |
| 8 | 2 min hover with the stick centred | Hold as good as before (±8 cm); the trim offload still moves `initialThrottleHold` |

## Results

None yet.
