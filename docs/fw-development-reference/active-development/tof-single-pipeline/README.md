# ToF Single Pipeline

[README](README.md) · [TASKS](TASKS.md) · [SCOUT](SCOUT.md) · [INVESTIGATION](INVESTIGATION.md)

| | |
|---|---|
| **Status** | **Closed - pending commit** ( 15 Oct 2026, FW 3.12.0, API 1.5.0 ) |
| **Mode** | `fix` ( follows the check topic [laser-tof-init-conflict](../laser-tof-init-conflict/README.md) ) |
| **Branch** | `BugFix-June26` |
| **Target** | `PRIMUS_X2_v1` ( all three targets plus an L1x + `LASER_ALT` check build at the gate and at commit ) |
| **Origin** | With `LASER_TOF` + `LASER_ALT` and `initObjectAvoidance ( ... )` / `XRanging.init ( )`, no laser works ( down + 4 side VL53L0X ) |

**Cause ( from the check topic ).** Two separate VL53L0X bring-ups share I2C1 and 0x29. The first side laser's init
and its address write also reach the down-laser and move it to 0x2A. Its next read times out, and the shared
`Global_Status` latch then stops all five lasers. Once that is fixed, the side-laser data could reach altitude hold
through shared globals ( H2 ).

**Fix ( option B ).** One ToF manager behind `XRanging`:

- `XRanging.initAlt ( gpio )` / `initAlt ( )` replaces `#define LASER_TOF` + `LASER_ALT` and is latched at boot.
- One bring-up runs at main.cpp:595 ( `xRangingInit ( )`, after `plutoInit ( )` ).
- Each laser has its own status and results.
- Side lasers are read round-robin when their data is ready, and the getters return cached ranges.
- `getRange ( DOWN )` reads the down range, and `getStatus ( id )` reports each laser's health.

On this board the down-laser XSHUT is **GPIO_5 ( PB15 )**, which used to be FRONT's LED pin. An XSHUT claim beats an
LED. L1x builds are unchanged.

**Gotchas.**

- Any LOW on an XSHUT resets that VL53L0X and wipes its address. LED writes must never reach a claimed XSHUT pin.
- A down-laser without an XSHUT keeps its moved address across a warm reset, so the bring-up probes 0x30 before 0x29.
- PB13 / PB15 driven as XSHUT take those pins from SPI2, so the flash is unusable. This overlap is known and out of
  scope.

**Validated** ( TESTING.md ):
- **Bench:** all five lasers together ( `Scan:158` ), FRONT does not reach altitude hold ( T4 ), warm reset.
- **Paced reads:** OA 4-6 ms → 30 µs.
- **Flight:** no regression against today's firmware ( same-day A/B ), and the baro-only hover matches the
  baseline.
- **SPI2:** off with XRanging, untouched without it.

**Open items.**

- **Temporary diagnostics:** the root `PlutoPilot.cpp` carries the hover-B test log. It must be restored to the stock file before release ( `pluto-commit` checks ).
- **Scratch worktree** for the A/B reference build ( scratchpad `wt-head` ): remove with `git worktree remove` at commit.
- **Follow-up topics, not this one:**
  - OA's roll / pitch override ( pilot at 30 % ) adds craft motion ( `Df/10` sd 9.0 against 5.5 cm );
  - the M25P16 flash is not recognised at boot on this board ( `Flash:0`, pre-existing );
  - L1x + XRanging collide at 0x29 ( the X1Ranging topic );
  - F1, the I2C driver's missing NACK check.
