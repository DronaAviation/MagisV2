# VL53L1X XRanging API ( X1Ranging )

[README](README.md) · [TASKS](TASKS.md) · [SCOUT](SCOUT.md) · [DESIGN](DESIGN.md) · [CHANGES](CHANGES.md) · [TESTING](TESTING.md)

| | |
|---|---|
| **Status** | **Closed - pending commit** ( 19 Oct 2026, FW 3.15.0, API 1.8.0 ) |
| **Mode** | `feature` |
| **Branch** | `BugFix-June26` |
| **Target** | `PRIMUS_X2_v1` |
| **Origin** | Deferred as "X1Ranging" by [tof-single-pipeline](../tof-single-pipeline/README.md) |

**Summary.** `X1Ranging` is a user API for VL53L1X lasers that mirrors `XRanging` ( VL53L0X ).

- **No build flag.** The laser family is chosen at run time from the API `plutoInit ( )` enables lasers through
  ( `rangingFamilyLatch ( )` ); both APIs used → both refused ( no laser, baro ).
- **Altitude.** One down-laser ( `X1Ranging.initAlt ( GPIO_5 )` ) feeds altitude hold through the VL53L1X copy of the
  fusion: 160 / 140 cm handover, frame shift, 2.5 s hold-off, constants unchanged. The fusion is compiled once per
  sensor; the latch picks the copy, and both copies are instruction-identical to their references.
- **Avoidance.** Four side lasers run the same object avoidance as XRanging through one shared core; out of range
  releases after 0.3 s ( both APIs ).
- **Driver.** `drivers/tof_manager_l1x` brings each laser up on its own XSHUT and address ( DOWN 0x30, sides 0x2A+n ).
  Both managers share one device RAM area ( `drivers/tof_dev_area` ).
- **Removed.** The old single-sensor path ( `ranging_init_L1`, `XVision` ); `LASER_TOF_L1x` and `LASER_ALT` are
  build errors.

FW 3.15.0, API 1.8.0; +14.4 KB flash and +3.0 KB RAM. Flight-validated on PRIMUS_X2_v1 ( log-3, log-5 ). Design:
[DESIGN.md](DESIGN.md). Plan: [TASKS.md](TASKS.md).

**Gotchas.**

- The L0X probe ( `ranging_vl53l0x.cpp:115` ) cannot see a VL53L1X. Both chips boot at 0x29, so one drone runs one
  family; user code that calls both APIs gets both refused ( mixed drones are out of scope ).
- The shared avoidance core touches `XRanging.cpp`; L0X behaviour must stay identical ( task 1 checklist ).
- The test `PlutoPilot.cpp` is a copy in `test/`. Restore the template before committing.
- `#define LASER_TOF_L1x` was uncommented in the X2_v1 `target.h` for log-2 only; task 15 then removed the flag ( `#error` now ).

**Open items.**

1. ~~Tasks 1-7~~ done: baseline, `tof_manager_l1x`, the `X1Ranging` API, wiring and the old `LASER_ALT` / `XVision` path removed, and one fusion switch ( the VL53L0X `altitudehold.o` is identical to HEAD ), and the shared OA core ( the VL53L0X `applyObjectAvoidance` has the same instructions ). Test file ready ( TESTING.md §2 ). ~~Task 14~~ ( OA out-of-range release, debounced, both APIs ) done. Open: tasks 8-13.
2. **Design revision ( 19 Oct, user ): no `LASER_TOF_L1x` flag.** X1Ranging is chosen at run time; both APIs used means both refused; two compiled fusion copies; one shared RAM area. ~~Task 15~~ ( flag removed ) and ~~task 17~~ ( two fusion copies, both instruction-identical to their references ) done; ~~task 16~~ ( shared RAM area, −2.1 KB ) done; the wiki ( task 8 ), the log-2 check ( task 18, by design ) and the build gate ( task 9 ) done; **hardware validated** ( task 10: log-3 + log-5 ). Remaining: pipeline docs ( 11 ), topic review ( 12 ), graph + commit ( 13 ).
3. ~~R1~~: the side update rate ( ~50 ms each ) was enough for the slow approaches flown in log-3 / log-5; the wiki advises slow approaches.
