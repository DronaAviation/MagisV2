# RX Signal-Loss Tolerance - Testing

[README](README.md) · [TASKS](TASKS.md) · [SCOUT](SCOUT.md) · [INVESTIGATION](INVESTIGATION.md) · [CHANGES](CHANGES.md) · [TESTING](TESTING.md) · [PIPELINE_UPDATE](PIPELINE_UPDATE.md) · [CONTROLLER_FINDINGS](CONTROLLER_FINDINGS.md) · [MSP_TIMING](MSP_TIMING.md)

## Test 1 - task 4 plan: bench, settings reset and real-loss timing ( 14 Oct 2026 )

**Build.** `Build/PRIMUS_X2_v1/DEFAULT_PRIMUS_X2_v1_3.11.2.hex`: `failsafe_delay` 15 applied, cap 22,
`EEPROM_CONF_VERSION` 108. Gate passed ( no new warnings, flash 100.9 KB, RAM 14.8 KB ).
**Method: observation only.** There is no drone-side log, and Developer Mode stops 200 ms after the last frame
anyway. Count the time ( a stopwatch, or "one-thousand-one…" ) and write what you see in
[logs/log-1.txt](logs/log-1.txt).

**Expected timeline after the controller goes off** ( props off, armed, on the bench ):

| After | What you should see or hear |
|---|---|
| 0-0.8 s | No change: the last stick input is replayed, then held |
| ~0.8 s | Motor note drops: sticks centred, throttle 1200 |
| **~2.5 s** | **`LAND` starts** ( `Signal_loss` ); motor note changes again |
| shortly after | `LAND` sees it is already on the ground and disarms; motors stop |

Before this change, `LAND` started at about **1.0 s**, so the "motor note drops" step and `LAND` came almost
together. Now there should be a clear pause of about 1.7 s between them.

**Steps:**
1. **Flash** the hex from PlutoIDE ( PRIMUS_X2_v1 ).
2. **Settings reset:** the first boot resets every saved setting ( `EEPROM_CONF_VERSION` 108 ). Calibrate the
   **accelerometer** ( level surface ) and the **compass**. **On Wi-Fi the drone will not arm until the compass
   is calibrated** ( `MAG_ENFORCE`; ELRS skips this check ). Set the battery capacity again if you use a non-default pack. Check ARM,
   ANGLE and ALT_HOLD on their switches.
3. **Readback:** CLI `get failsafe_delay`, or the app's failsafe setting. Expect **15**.
4. **Real-loss timing**, props **off**:
   1. Arm, and raise the throttle a little so the motors spin audibly.
   2. Switch the controller off.
   3. Note the time to the motor-note drop, to `LAND` or `Signal_loss`, and to the motors stopping.
   4. Repeat 3 times.
5. **Short interruption:** not tested on the bench, because there is no controlled 1-2 s gap available. The
   task 5 flights cover it with real Wi-Fi stalls.

**Pass:**
- the settings were reset and the drone arms after both calibrations;
- `failsafe_delay` reads 15;
- in all 3 repeats, `LAND` or the motor stop comes **clearly later than ~1 s, at roughly 2-3 s**.

**Fail:**
- `LAND` still comes at about 1 s. The fix did not take effect: check the hex that was flashed and the readback.
- Nothing happens for well over 4 s. The cap or the timing is wrong.

## Results - tasks 4 and 5 ( 14 Oct 2026 )

**Reported by the user: bench and flights done, all good.** On 3.11.2 the bench real-loss check and the flights
passed; no unwanted `Signal_loss` was reported. The notes sheet [logs/log-1.txt](logs/log-1.txt) was not filled
in, and no controller log was shared. So these are not recorded:
- the settings reset and the accelerometer and compass calibration;
- the `failsafe_delay` readback;
- the observed times to `LAND`;
- the number and length of Wi-Fi stalls ridden through;
- the per-mode real-loss results.
