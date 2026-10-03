# Battery SoC Fix

[README](README.md) · [TASKS](TASKS.md) · [SCOUT](SCOUT.md) · [INVESTIGATION](INVESTIGATION.md)

| | |
|---|---|
| **Status** | **Closed - pending commit** ( 3 Oct 2026, FW 3.11.0, API 1.4.0; partial commit `369ea71` 29 Sep ). Pipeline doc ( Power_BMS_Pipeline ), Failsafe / MSP / User_Space_API notes, FLIGHT_INVARIANTS, CLAUDE.md, CHANGELOG and the BMS API wiki promoted |
| **Mode** | `fix` |
| **Branch** | `BugFix-June26` |
| **Target** | `PRIMUS_X2_v1` ( one R020, 20 mOhm: production ); all targets built at commit |
| **Origin** | [battery-capacity-estimate](../battery-capacity-estimate/README.md) ( `check`, closed 26 Sep 2026 ): its findings, logs and measurements are the evidence for this fix |

**Problem.** The app's remaining-mAh figure and the low-battery warning cannot be trusted. The check topic
found:

- the ~300 mAh shown on an empty pack came mostly from this board's stacked second R020 ( a rework, not
  production );
- with the production shunt the count lands within 1% of the charger, but only because a −5% auto-gain
  cancels a +4% reading;
- the warning fires ~15 s before empty;
- an empty pack reads 54-58% SoC ( `mAhRemain` wrap );
- the starting estimate depends on one floored voltage sample and a straight line.

**Fix design ( user's choices, 26 Sep ).**

- **SoC is count-led.** The mAh count drives it, anchored at plug-in by a LiPo resting-voltage curve applied to
  an averaged sample.
- **A current-compensated voltage floor catches the end.** It fires the warnings and pulls the remaining figure
  to ~0 on a pack below its rating.
- **Remaining saturates at 0** instead of wrapping.
- **The auto-gain is removed.**
- **Warnings keep today's actions, earlier.** Beeper and app flag only: warning with ≥15% really left, critical
  at ~5%.
- **Capacity is the configured rated value** ( 600 / 800 / 1200, set by the user ). The ELRS 800 default stays.

**Done when.** The bench supply sweep shows no SoC jump at ≤3.0 V. Full-to-warning flights on three packs ( an
older 600 pack, a newer 600, an 800 ) each meet all of:

- `D` within ±5% of the charger;
- app remaining at empty ≤5% of capacity;
- the warning comes with ≥15% really left.

**Constraints.** The `MSP_ANALOG` layout stays unchanged. `Bms_Get ( )` may change, with the `docs/API` wiki
and an `API_Version` bump. **Out of scope:** CRSF current units, the app's pre-gain current, auto-land, the
arming block, and the Dev Mode link fix as a product feature.

**Outcome.** Pluto Fuel Gauge and Low-Battery Auto-Land, validated on four packs ( TESTING.md log-3 to log-8 ):
warning with 13-20% really left ( one flight 12.9% ), critical with ~5-8%, four auto-landings, empty ≤ 5% on every
flight; the count reads ~5% above the charger on healthy packs ( accepted ). The temporary test code is removed.
The app-side recommendations ( APP_INTEGRATION.md, not committed ) were sent to the app developer separately.
