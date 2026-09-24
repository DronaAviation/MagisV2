# VL53L1X Extended Reach ( 270 / 240 cm handover )

| | |
|---|---|
| **Status** | **Planned** ( noted 24 Sep 2026; not started ) |
| **Branch** | not started |
| **Target** | `PRIMUS_X2_v1` with `LASER_TOF_L1x` + `LASER_ALT` |
| **Origin** | [vl53l1x-althold-parity](../vl53l1x-althold-parity/README.md) ( FW 3.10.0 ), which fixed the VL53L1X at a 160 / 140 cm band |

**Goal ( user ).** Hand over from the laser to the baro at **270 cm up / 240 cm down** instead of 160 / 140,
so laser altitude hold covers heights up to about 2.7 m.

**Why changing only the band values does not work.** Every VL53L1X sample above about 2 m on the flying floor
comes back with status 2 ( signal fail: the return is below the sensor's minimum count-rate limit ), and the
driver treats any non-zero status as out of range. With only the band raised to 270 / 240:

- the laser is still lost at about 1.9-2.0 m, and the drone hands over through the 185 ms **dropout** instead of
  the band;
- at 1.9-2.0 m about half the samples are valid, and after a dropout the return is allowed below the **upper**
  edge ( 270 ). The estimator would hunt: dropout, return on 3 valid samples with a frame shift, dropout again.
  With 160 / 140 the handover happens on fully valid data, before that zone.

**Evidence** ( [vl53l1x-althold-parity TESTING.md](../vl53l1x-althold-parity/TESTING.md) ):

| Measurement | Result |
|---|---|
| Valid fraction by height, Medium 45 / 50 ms ( log-5 ) | 100 % up to 180 cm, 90 % at 180-190, 57 % at 190-200, ~0 above 210 cm |
| Long mode 45 / 50 ms ( log-6, bench ) | worse: 5-17 % `St` 7 ( wrap ) when level, 99 % at a 20° tilt |
| Long mode 95 / 100 ms ( log-7 bench, log-8 flight ) | as clean as Medium below 1.8 m, but **no extra reach**: 0 / 188 valid above 2 m |
| Raw readings flagged `St` 2 above 2 m ( log-10, 12, 13 ) | often plausible ( laser − estimate −35…−5 cm, sd 4-6 cm ), but some windows had sd 17-64 cm with jumps of ±1.5 m to ±6 m |

Both modes and both integration times stop at the same height with the same error. That points at a fixed
software threshold, not at the optics.

**Likely lever.** The ST preset sets the minimum return signal to **1.5 MCPS** ( `tp_lite_med/long_min_count_rate_rtn_mcps`,
192 in 9.7 fixed point, `lib/main/VL53L1X_API/core/src/vl53l1_api_preset_modes.c` ). `VL53L1_DataInit` writes
0.25 MCPS, but `StaticInit` / `SetDistanceMode` overwrite it. The sigma limit works the same way: 90 mm
( 18 mm at init ).

**Planned approach.**

1. Re-apply a lower signal limit after `SetDistanceMode ( )` in `ranging_init_L1 ( )`
   ( `VL53L1_SetLimitCheckValue`, e.g. 0.5 then 0.25 MCPS ), as a target-overridable constant. Check in the ST
   source how the limit reaches the device ( timing config, sent at `StartMeasurement` ).
2. Bench: noise and outlier rate at 1-3 m for each limit value, Long 95 / 100 as well as Medium.
3. Reach flights in a hall with a ceiling of at least 3.5 m, away from ceiling fans. Valid fraction per 10 cm
   band; outliers counted against the baro estimate.
4. Only if the reach is 100 % valid to about 2.9-3.0 m with no outliers, set the band to 270 / 240. If the rate
   drops to 10 Hz ( Long 95 / 100 ), the fusion constants follow `L1X_SAMPLE_PERIOD_MS` automatically
   ( dropout 3P + 35 = 335 ms, step window 5 samples ); check the object hold-off still reacts fast enough.
5. Re-check the return guard ( 50 cm agreement ) and its 25 cm exit at the new heights.

**Ready to use from the last topic.** `L1X_DISTANCE_MODE`, `L1X_TIMING_BUDGET_US` and `L1X_SAMPLE_PERIOD_MS`
are target-overridable, with a `static_assert` that the period is at least the budget + 5 ms.
`vl53l1x-althold-parity/baseline/snapshot.sh` ( if committed ) proves the VL53L0X object code is unchanged.
The flight-log fields used were `mm St S E H V Tl Ar` ( TESTING.md, tests 10-13 ).

Run `/pluto-grill` on this topic to start it.
