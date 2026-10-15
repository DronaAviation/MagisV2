# ToF Single Pipeline - Investigation

[README](README.md) · [TASKS](TASKS.md) · [SCOUT](SCOUT.md) · [INVESTIGATION](INVESTIGATION.md)

The root-cause work is in the check topic [laser-tof-init-conflict](../laser-tof-init-conflict/INVESTIGATION.md).
This file summarises it and records what the fix planning added.

## Problem

With `LASER_TOF` + `LASER_ALT` defined and `initObjectAvoidance ( ... )` or `XRanging.init ( )` in user code, no laser
works. That covers the down ( altitude ) laser and all four side lasers. Each pipeline works on its own.

## Causal chain ( confirmed in code by the check topic )

1. The down-laser is set up at 0x29 by `ranging_init ( )` ( main.cpp:518 ). It has no XSHUT under that code path.
2. `xRangingInit ( )` ( main.cpp:600 ) releases the first enabled side laser at 0x29, so two chips answer.
3. The side laser's init and its address write ( 0x2A ) reach both chips, and the down-laser moves to 0x2A too
   ( XRanging.cpp:109-110 ).
4. The first down-laser read goes to the now empty 0x29. It times out, and the shared `Global_Status` latches −20.
5. Every VL53L0X read checks that one flag, so all five lasers stop. Altitude hold sees a dropout ( baro hold ).

Also confirmed in the check topic:
- **H3:** side lasers are awake during the down-laser's setup.
- **H2:** side-laser data can reach altitude hold through the shared result globals. This is latent today and
  becomes a flight risk once H1 is fixed.
- **H5:** the blocking side-laser reads cost about 4-6 ms every 100 ms.
- **F1-F5:** the further findings listed there.

## Fix ( option B, chosen in the check topic, D1-D9 )

One ToF manager behind `XRanging`:
- `initAlt ( gpio )` / `initAlt ( )` replaces `#define LASER_TOF` + `LASER_ALT`, and is latched at boot.
- One bring-up at main.cpp:600.
- Each laser has its own status and results. Altitude hold reads only DOWN.
- Side lasers are read round-robin when their data is ready, and the getters return cached values.
- `getRange ( DOWN )` reads the down range.

### Bring-up order ( task 5 )

| Step | Action | Rule |
|---|---|---|
| 1 | Nothing runs at main.cpp:516-520; everything starts at :600, after `plutoInit ( )` | R2 |
| 2 | Drive every enabled XSHUT low, DOWN's included; wait 10 ms | R1, R2 |
| 3a | DOWN with a pin ( GPIO_5 on this board ): release alone, init with today's config, move to 0x30 | R1, R6, R8 |
| 3b | DOWN without a pin: probe 0x30 ( warm reset ); else probe 0x29 and move to 0x30 | R1, R8 |
| 4 | Release side lasers one at a time in table order, 0x2A + n; check every status | R1, F2 |
| 5 | Record a status per laser; a failed one is never retried | R3, R4 |

## Added in fix planning ( 14 Oct 2026 )

**GPIO_5 is the down-laser XSHUT on this board.** It was FRONT's status-LED pin ( XRanging.cpp:57 ), and the FRONT
LED is no longer wired to it. That LED pin is written in three places:
- LOW at bring-up ( :93 ),
- toggled 4× in the blink animation ( :123 ),
- LOW every 100 ms from OA when FRONT sees nothing ( :319 ).

On an XSHUT, any LOW puts the VL53L0X in hardware standby and wipes its assigned address, so the altitude laser
would die on the first OA pass. Hence the rule: **an XSHUT claim beats an LED**. With `initAlt ( GPIO_5 )`, FRONT has
no LED, and every LED write skips a claimed XSHUT pin.

**GPIO_5 = PB15**, which is also SPI2_MOSI ( the flash ), ADC_2 and PWM_4 ( [PIN_MAP.md](../../PIN_MAP.md) :46 ):
- `initAlt` refuses the pin when ADC_2 or PWM_4 is enabled.
- Once the bring-up drives PB15, SPI2 flash is unusable. LEFT's XSHUT on PB13 ( SPI2_SCK ) already does the same
  today. This overlap is known and out of scope.

**L1x builds.** The L1x fusion lives inside `#ifdef LASER_ALT` ( altitudehold.cpp:111 / :185 ). To keep those builds
as they are, the latched flag is `LASER_TOF_L1x && LASER_ALT` at compile time. `initAlt` is refused in a build with
L1x compiled in.

**Cost.** The fusion code becomes always compiled, adding about 2-2.5 KB of flash and 0.4 KB of RAM over today's
101.4 KB / 16.8 KB. Because of that, the flag-off build is not byte-identical to today's baro build. Its equivalence
is shown in two ways:
- the reviewer checks every converted gate's flag-off branch;
- a baro-only hover is compared against a baseline log of today's build.

## Q&A that shaped this plan

| Question | Answer |
|---|---|
| Where do the fix tasks live? | A new topic, `tof-single-pipeline`. The check topic is closed and points here. |
| How much of option B? | Both phases: state, bring-up and API first, then paced polling, with a bench check between them. |
| Reproduce T1 / T2 first? | No. The chain is deterministic in the code, so we go straight to the code. |
| Done when? | Bench ( five lasers read, address scan, T4 ), then a hover with `initAlt` + OA on four sides and a wall approach, then a baro-only hover. |
| Down-laser XSHUT pin? | GPIO_5. Only the XSHUT is connected to it; the FRONT LED is not. |
| A laser fails? | It is marked failed and never retried; the others keep working. A failed DOWN is a fusion dropout. |
| Status for user code? | `XRanging.getStatus ( id )`, returning OK / NOT_ENABLED / BOOT_FAILED / I2C_ERROR / OUT_OF_RANGE. |
| L1x builds? | `LASER_ALT` is kept, for L1x only, so L1x behaves exactly as today. |
| How do we show the baro build is unchanged? | A reviewer check gate by gate, plus a baro-only hover compared against a baseline. |
| Rollback? | Today's HEAD built with `LASER_TOF` + `LASER_ALT`, with its hex saved in task 1. |
