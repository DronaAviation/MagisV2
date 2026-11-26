# Scout brief: Estimate_Get ( Rate, axis ) returns the stick-rate setting ( 75 ), not a measured rate

( `pluto-scout` brief, 4 Nov 2026, verbatim. Graph was stale ( built a7f398d1, HEAD 7a41a37 ) and not used; everything
comes from reading the source directly. )

Subsystems: User API ( FC-Data ), IMU / DCM, PID -> `fw-architecture-pipeline/subsystems/User_Space_API.md`

Key code:
- `src/main/API-Src/FC-Data-Estimate.cpp:77-86` Rate case: returns `currentControlRateProfile->rates [ FD_ROLL / PITCH / YAW ]` ( uint8 config )
- `FC-Data-Estimate.cpp:88-108` Position / Velocity cases
- `FC-Data-Estimate.cpp:118-139` Angle overload
- `src/main/flight/acrobats.cpp:87-88,116-117,240-241` writes `rates [ ROLL / PITCH ] = 75` ( 110 during a flip, `:125-140` )
- `src/main/flight/imu.cpp:588-595` `matrix_update`: active DCM attitude, hard-coded `scale = 0.0174532f / 16.4f`
- `src/main/drivers/accgyro_icm20948.cpp:68` `gyro->scale = 1.0f / 131.0f`; `:78` `GYRO_CONFIG_1 = 0x1F`
- `src/main/flight/pid.cpp:826` pidRewrite `RateError = desiredRate - gyroADC / 4`
- `src/main/API-Src/FC-Data-Sensors.cpp:52,65` `Sensor_Get ( Gyroscope )` = `gyroADC / 1.64`

Mode: fix

Current behaviour / causal chain ( proven ):
1. **Rate X/Y/Z** returns `rates []`, the stick-to-rate setting, not a measurement. The config reset sets it to 0
   ( `config.cpp:357` ). The flip code writes 75 to roll and pitch, and 110 while a flip runs. The value shown is 75
   because that is what is stored in the tester's profile. Yaw ( Z ) shows its own stored value; the code cannot tell
   whether that was also 75 on the tester's board.
2. **Position Z** = `getEstAltitude ( )` = `EstAlt`, int32 in cm ( `altitudehold.cpp:1510` ). Correct ( T03 confirms 0 at rest ).
3. **Velocity Z** = `VelocityZ`, int32 in cm/s ( `altitudehold.cpp:91,1002` ). Correct.
4. **Position / Velocity X/Y** = `PositionX/Y` ( float, cm ) and `VelocityX/Y` ( int16, cm/s ) from `posEstimate.cpp:299-302`.
   These only update when `localisationType == UWB` ( `mw.cpp:822` ) or `OPTIC_FLOW` is defined, and `OPTIC_FLOW` is
   commented out on both targets ( `target.h:179/180` ). Without one of those they always read 0. Documented behaviour,
   not a bug.
5. **Angle**: roll and pitch in deci-degrees, yaw in whole degrees 0..359 ( `imu.cpp:1185-1195`, DCM path ). Correct.
6. The implicit int32 / float -> int16 conversions already produce 2+2 warnings that are in the baseline
   ( `tools/warnings-baseline.json:115-116` ).

Analysis, measured rate source and unit:
- `gyroADC []` ( int16, `sensors/gyro.h:34` ) is the raw reading with bias removed and board alignment applied.
  `mpuGyroRead` does no shift. Index 0/1/2 = roll/pitch/yaw, which matches `axis_e` X/Y/Z = 0/1/2.
- Both targets define `DCM` ( `target.h:178/179` ), so the attitude flown comes from `matrix_update`, which uses
  **16.4 LSB/dps** ( `imu.cpp:590` ). `gyro.scale` ( 1/131 ) is only used by the non-DCM `imuCalculateEstimatedAttitude`
  ( compiled out ) and by `pidLuxFloat` ( not active: `pidController = 1` = REWRITE, `config.cpp:176` ).
- `0x1F` decodes to FS_SEL = 11 = +/-2000 dps, and `BANK_2 = 0x20` is correct, so the register write lands. Correct
  angles in flight prove the 16.4 LSB/dps scale, and therefore that `scale = 1/131` is stale but harmless.
- Proposed unit: deg/s x10 = `gyroADC / 1.64`, which matches `Sensor_Get ( Gyroscope )`. Alternative: whole deg/s = `gyroADC / 16.4`.
- Bench test to confirm: rotate the board 360 deg in a known time T, or by hand while integrating the logged Rate over
  time. The integral of Rate x10 / 10 dt should come to about 360. An 8x scale error would show as about 45.
- Range: +/-32767 counts gives +/-19980 for deg/s x10, which fits int16. Altitude in cm overflows int16 above 327 m,
  which is not a concern.

Fix options:
- **A ( minimal )**: Rate returns `lrintf ( gyroADC [ axis ] / 1.64f )` ( deg/s x10, raw ). About 30-50 B of flash, no
  RAM, negligible loop time. It duplicates `Sensor_Get ( Gyroscope )` but now with a stated unit.
- **B**: Rate in whole deg/s ( `/ 16.4f` ). Easier to read, but loses resolution below 1 deg/s.
- **C**: A or B, plus a low-pass filtered copy computed in `imu.cpp` ( about 12 B RAM, adds a hook in the loop ). Less
  noise, but adds delay.
- Any option changes behaviour, so per pluto-rules :164 / CLAUDE.md, bump API 1.8.1 -> 1.8.2 and FW 3.15.3 -> 3.15.4.
  Fix the header comment and the wiki in the same change.

Constraints / invariants that apply:
- Keep the public header signature stable ( CLAUDE.md "Two-layer API" ).
- No new warnings: explicit cast plus rounding, float not double ( `1.64f` ).
- Keep `Monitor_Print` under 130 B per tick in the test sketch.

Resource conflicts: none

Already answered by the code:
- No firmware code and no PlutoPilot.cpp example calls `Estimate_Get ( Rate, ... )`. Existing consumers only use Angle
  ( `OLED_API_WIKI.md:126`, `YAW_HEADING_API_WIKI.md:41,129` ) and Position / Velocity Z ( takeoff-* and x1ranging test
  sketches ). None of those cases change.
- Header `FC-Data.h:96` and `:116` say `@return uint16_t` while the function returns int16_t. The axis overload states
  no units at all, and "roll rate" is ambiguous.

Questions for the user:
1. Unit for Rate: deg/s x10 ( same as Sensor_Get ) or whole deg/s?
2. Raw or filtered rate?
3. Should the same change update `Sensor_Get ( Gyroscope )` in the wiki now that the 16.4 scale is proven, or should the
   driver comment / scale be corrected in a separate topic?
4. Is the bench rotation test acceptable as the hardware validation?
5. Should Estimate_Get X/Y keep returning 0 without UWB, or should that just be documented?

Doc drift:
- `User_Space_API.md:27` does not mention Rate. Once fixed, add "Rate gyro deg/s x10".
- `FC-Data.h:96,116` say uint16_t; the functions return int16_t.
- `FC_DATA_API_WIKI.md:61,68-69` describes the 8x scale conflict as unresolved; the code now resolves it ( 16.4 LSB/dps ).

Prior work: baro-temperature-api ( closed 7a41a37, same T03 sheet; it created FC_DATA_API_WIKI.md ). History of these
files: `3b7c633`, `da36cf3`.

Suggested skills: pluto-rules, pluto-build ( `--gate PRIMUS_X2_v1` ), pluto-flighttest, spec-to-code-compliance, pluto-commit.

Pipeline docs to update at the end:
- `User_Space_API.md:27`: add units for every case.
- `docs/API/FC_DATA_API_WIKI.md`: new `Estimate_Get` section, "Changes in 1.8.2", and a correction to the gyro "not verified" note.
- CHANGELOG.
