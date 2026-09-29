# Mid-flight `Signal_loss` on the Wi-Fi RC link: findings and recommended MagisV2 changes

**Status:** investigation complete on the controller side, no MagisV2 change made.
**Date:** 24 August 2026.
**Written for:** whoever works on MagisV2 next. All paths in sections 4 onward are
relative to the MagisV2 repository root. Sections 1 to 3 are the evidence that
justifies the change; section 4 onward is the change itself.

**Controller firmware under test:** Pluto Controller v3, `0.36.0`, talking MSP over
TCP at 100 Hz to `192.168.4.1:23`. The controller has **not** been modified to work
around this, deliberately, so that the effect of a MagisV2 change can be measured
against the four logs below.

---

## 1. The symptom

A drone flying normally suddenly declares `Signal_loss`, lands itself and disarms.
It happened on every one of four consecutive test flights, between 9 s and 75 s
after take-off. The link is completely healthy before and after.

From the controller's own log, one representative event:

```
82..97 s   tx 100/s  rx 112/s  drop 0/s   maxWrite 1.2-1.7 ms   ← 15 s flawless
97.050     last telemetry reply received
98.000     tx 13/s   rx 8/s    drop 98/s                        ← total collapse
98.481     flight status 5: SIGNAL LOSS (bits 0x0120, armed)
98.481     flight timer stopped at 27 s
100.001    tx 100/s  rx 112/s  drop 0/s                         ← perfect again
```

The outage is a **1.43 s two-way blackout**. `drop` is the controller's count of
RC frames refused by the socket because its send buffer was full.

## 2. What the four flights measured

| Flight | Controller cell | Transmit power | Flight length | Outage |
| --- | --- | --- | --- | --- |
| 1 | 27 → 25 % | dynamic, dropped to 15 dBm mid-flight | 9 s | two events |
| 2 | 23 % | 15 dBm (pinned low by the cell) | 38 s | 2.17 s |
| 3 | 21 → 10 % | full | 75 s | 1.21 s |
| 4 | **91 %** | **full** | 27 s | **1.43 s** |

Observed stall durations: **1.21 s, 1.43 s, 2.17 s.** No correlation with flight
duration, cell charge or transmit power.

## 3. What has been ruled out, and how

Each of these was tested rather than assumed.

1. **Range and signal strength.** RSSI was **−48 to −57 dBm, four bars**, steady
   through every event, before and during. Flight 4 spent 15 seconds at −52 dBm
   immediately before collapsing.
2. **Controller transmit power.** Flights 3 and 4 ran at full power and still
   dropped out. A dynamic power-saving feature on the controller was the leading
   suspect and is now excluded.
3. **Controller battery sag.** Flight 4 ran at 91 %, 4.05 V, rock steady. Flight 3
   ran down to 10 % and lasted the longest of the four.
4. **Controller-side serial logging.** The controller's `LOG_*` enqueues with a
   zero timeout and cannot block its caller; no run ever filled the queue; the log
   line rate was flat at ~33/s and *fell* during each event.
5. **The controller's RC task stalling.** It attempted all 112 sends in the
   blackout second and its worst single write was 206 µs. It kept perfect time
   throughout — the socket refused it, it was not late.
6. **Wi-Fi disassociation.** No disconnect, no reassociation, no reconnect in any
   log. The station stayed associated for the whole of every outage.

### The leading explanation: TCP head-of-line blocking

One RC frame is lost on the air. TCP will not deliver anything queued behind that
segment until it is retransmitted, and the retransmission waits out an RTO of
roughly a second, doubling on repeat. Meanwhile the send buffer fills — about
100 ms at 100 Hz and 22 bytes a frame — and every send after that is refused.
Telemetry shares the same socket, so the replies stop too, which is why **one lost
packet kills both directions at once**. When the retransmission lands, the backlog
flushes and the link is instantly perfect again.

That mechanism is indifferent to signal strength and transmit power, because one
lost frame is all it takes and frames are lost at any RSSI. Which is exactly what
the four flights show. It also explains the 1.2–2.2 s durations clustering around
an RTO and its first backoff.

**This is a hypothesis, not a proof.** It is the only explanation left that fits
all four flights. Section 8 says how to test it.

> **The important consequence for MagisV2:** the RC stream *will* stop for 1–2 s
> at a time on this transport, on a link that is otherwise perfect, and neither
> end can currently prevent it. The question this document is about is whether the
> flight controller should be landing itself when that happens.

---

## 4. How MagisV2 currently decides the link is gone

The controller uses `FEATURE_RX_MSP`. Counting from the last RC frame that arrived:

| Time | What happens | Where |
| --- | --- | --- |
| **200 ms** | `rxSignalReceived` goes false, so every channel reads `PPM_RCVR_TIMEOUT` and is treated as an invalid pulse. | `needRxSignalBefore = currentTime + DELAY_5_HZ` — [`src/main/rx/rx.cpp:369`], cleared by `resetRxSignalReceivedFlagIfNeeded()` at [`rx.cpp:313`]. `DELAY_5_HZ` is 200 000 µs at [`rx.cpp:92`]. |
| 200 → 600 ms | Channels **hold their last received values** and still count as valid, so `failsafeOnValidDataReceived()` keeps being called and keeps refreshing the clock. Failsafe does not yet know anything is wrong. | `sample = rcData[channel]` at [`rx.cpp:528`], bounded by `MAX_INVALID_PULS_TIME` = 600 ms at [`rx.cpp:87`], refreshed on each valid pulse at [`rx.cpp:536`]. |
| **600 ms** | Hold expires. Channels take their rxfail values, `rxFlightChannelsValid` goes false, and `failsafeOnValidDataFailed()` starts being called instead. | `getRxfailValue()` at [`rx.cpp:431`]; the branch at [`rx.cpp:550-556`]. |
| **≈ 800 ms** | The failure has persisted 200 ms past the last refresh, so `rxLinkState` becomes `FAILSAFE_RXLINK_DOWN`. | `PERIOD_RXDATA_FAILURE` = 200 ms at [`src/main/flight/failsafe.h:35`], compared in `failsafeOnValidDataFailed()` at [`failsafe.cpp:211-216`]. |
| immediately | `FAILSAFE_IDLE` + armed + not receiving → `set_FSI(Signal_loss)` and `current_command = LAND`. There is no further grace; the staged `FAILSAFE_RX_LOSS_DETECTED` delay was replaced with a direct land. | [`failsafe.cpp:245-250`] |

**Total tolerance: about 800 ms.**

This matches the logs. In flight 4 the last frame was around t = 97.05 s, so
800 ms puts the decision at ≈ 97.85 s. The controller only *saw* the flag at
98.481 s because it could not receive anything until the link came back.

**Every outage measured — 1.21 s, 1.43 s, 2.17 s — exceeds this budget. None of
those four flights could have survived it.**

Two of the three constants have already been raised once for what looks like the
same reason, which is worth knowing before touching them again:

- `MAX_INVALID_PULS_TIME` carries `// original 300?? failsafe_drona`.
- The MSP branch uses `DELAY_5_HZ` where every other RX provider uses
  `DELAY_10_HZ`, commented `//temp fix drona failsafe_drona`.

---

## 5. A bug: the configured failsafe delay does nothing

`failsafeReset()` computes the intended budget at [`src/main/flight/failsafe.cpp:122`]:

```c
failsafeState.rxDataFailurePeriod =
    PERIOD_RXDATA_FAILURE + failsafeConfig->failsafe_delay * MILLIS_PER_TENTH_SECOND;
```

`failsafe_delay` defaults to **10**, i.e. one second, at
[`src/main/config/config.cpp:544`] (`// 1sec`), and is settable over CLI and MSP:

- [`src/main/io/serial_cli.cpp:456`] — `failsafe_delay`, `VAR_UINT8`, range 0–200.
- [`src/main/io/serial_msp.cpp:1256`] and [`:1734`] — read and written by the app.
- [`src/main/flight/failsafe.h:39`] — *"Guard time for failsafe activation after
  signal lost. 1 step = 0.1sec."*

So the intended stage budget is **200 ms + 1000 ms = 1200 ms**.

**`rxDataFailurePeriod` is written and never read.** It is assigned at
`failsafe.cpp:122`, declared at [`failsafe.h:64`], and that is the whole of its
use — grep the tree and there is no third occurrence. `failsafeOnValidDataFailed()`
compares against the raw `PERIOD_RXDATA_FAILURE` instead:

```c
void failsafeOnValidDataFailed ( void ) {
  failsafeState.validRxDataFailedAt = millis ( );
  if ( ( failsafeState.validRxDataFailedAt - failsafeState.validRxDataReceivedAt ) > PERIOD_RXDATA_FAILURE ) {
    failsafeState.rxLinkState = FAILSAFE_RXLINK_DOWN;
  }
}
```

**The flight controller is therefore 1000 ms less tolerant than its own
configuration asks for.** Fixing that is a bug fix, not a safety retune — it
restores a number Drona already chose and already exposes to the app.

---

## 6. What the drone is actually doing during the window

This matters more than it looks, because it is what makes extending the window
defensible.

The four flight channels default to `RX_FAILSAFE_MODE_AUTO`
([`src/main/config/config.cpp:469`]; AUX channels default to
`RX_FAILSAFE_MODE_HOLD`). In `AUTO`, `getRxfailValue()` at [`rx.cpp:431`] returns:

- **roll, pitch, yaw → `rxConfig->midrc`** — centred, not held.
- **throttle → 1200** (`//rxConfig->rx_min_usec; drona_failsafe`).

So the window has two distinct phases:

| Phase | Sticks | Risk |
| --- | --- | --- |
| 0 – 600 ms | **Held** at the last received values | A drone commanded into a turn keeps turning |
| 600 ms onward | **Centred**, throttle 1200 | Levels off and descends gently |

**Any extension beyond 600 ms is spent with the sticks already neutralised and the
throttle already below hover.** It is not a runaway; it is a level, slowly
descending aircraft waiting to see whether the link comes back. The AUX hold keeps
it armed, which is what lets it recover in the air rather than falling out of it.

That is a very different trade from the one it first appears to be, and it is the
main reason this document recommends extending the window rather than only
patching the controller.

---

## 7. Recommended changes, in order

### 7.1 Fix the dead `failsafe_delay` — do this first

At [`src/main/flight/failsafe.cpp:213`], compare against
`failsafeState.rxDataFailurePeriod` rather than `PERIOD_RXDATA_FAILURE`:

```c
if ( ( failsafeState.validRxDataFailedAt - failsafeState.validRxDataReceivedAt ) > failsafeState.rxDataFailurePeriod ) {
```

- Budget goes from **~800 ms to ~1800 ms**.
- No new constants; it applies the configured 1 s that is already the default and
  already documented in `failsafe.h`.
- **Covers the 1.21 s and 1.43 s stalls. Does not cover the 2.17 s one.**

Things to check while making this change:

- `failsafeReset()` runs where `failsafeConfig` is already valid, and reruns after
  a config change — otherwise `rxDataFailurePeriod` is stale or zero. A zero would
  make the FC *more* trigger-happy than today, so this is worth confirming rather
  than assuming.
- Whether `PERIOD_RXDATA_RECOVERY` on the other side ([`failsafe.cpp:206`]) should
  stay at 200 ms. Recovery should stay fast; probably leave it.

### 7.2 Consider `failsafe_delay = 15`

A config change, not a code change — `failsafe_delay` is already a CLI/MSP
variable with a 0–200 range.

- 15 → total ≈ **2.3 s**, which clears the worst stall observed (2.17 s).
- Per section 6, the extra second is spent with sticks centred and throttle at
  1200.

**Do 7.1 first and fly it.** If 1.8 s turns out to be enough in practice, do not
buy the extra second. Only reach for 15 if another 2 s stall appears.

### 7.3 Do not touch `MAX_INVALID_PULS_TIME` or `DELAY_5_HZ`

Both have already been raised once and both are shared with other RX providers or
with the stick-hold behaviour described in section 6. Lengthening the **hold**
phase is the genuinely risky change, because that is the phase where the drone
keeps flying the last commanded input. `failsafe_delay` extends only the
sticks-centred phase, which is why it is the right knob.

### 7.4 Worth investigating separately: why frames are lost at all

Nothing here explains why the underlying packet loss happens on a −52 dBm link a
few metres away. If the drone's Wi-Fi bridge can be made to accept **UDP** for the
RC stream, head-of-line blocking disappears entirely and none of the above is
needed — a lost frame would simply be a lost frame, which at 100 Hz costs nothing.
That is the real fix if it is reachable. It is in the drone's ESP firmware, not in
MagisV2.

---

## 8. How to test

Fly the same profile that produced the four logs: hover and gentle movement,
30–90 s, controller on a healthy cell, transmit power on **Full** so that variable
is pinned.

The controller carries an **observation-only stall detector** (`include/stall.h`,
action disabled) that logs the line below whenever a stall lasts long enough that
it *would* have intervened. Use it to time the outage from the controller side:

```
W rc  ] send buffer full 512 ms - would drop socket here (stall action disabled)
D rc  ] tx 0/s rx 0/s drop 112/s maxWrite 206 us resets 0
```

Then read the flight-status line:

| Result | Meaning |
| --- | --- |
| Stalls still appear in the `drop` counters, but **no `SIGNAL LOSS`** and the drone keeps flying | **The fix worked.** The link now rides through a stall that used to end the flight. |
| `SIGNAL LOSS` still fires, and the stall was longer than the new budget | Real, and the budget is still short — consider 7.2. |
| `SIGNAL LOSS` fires on a stall *shorter* than the new budget | The change did not take effect. Check `failsafeReset()` is running with a valid `failsafeConfig` (section 7.1). |

Also confirm the ordinary failsafe still works, because that is what is being
traded away: power the controller off mid-hover and check the drone still
neutralises, descends and disarms. Time it — it should now take about a second
longer than before, and no more than that.

---

## 9. What not to do

- **Do not disable failsafe**, or set `failsafe_delay` to its 200 maximum. The
  target is to survive a 1–2 s transport stall, not to stop landing on genuine
  loss of control. Anything past ~2.5 s is buying nothing against the measured
  data and giving up real safety.
- **Do not lengthen `MAX_INVALID_PULS_TIME`** to get the budget — see 7.3.
- **Do not conclude this is a range or antenna problem.** Section 3 excludes it
  with data, and chasing it will cost time.

---

## 10. Cross-references

Controller-side, in the JoyStick2FW repository:

- `include/stall.h` — the stall detector, its reasoning, and the 800 ms budget
  derivation, with the action currently disabled by `stall::ACTION_ENABLED`.
- `CLAUDE.md`, constraints section — the one-paragraph version of this finding.
- `include/rssi.h` — why the controller's four-bar signal indicator reads normally
  straight through these outages, and must not be used as evidence about one.

MagisV2, the six files this touches:

| File | Why |
| --- | --- |
| `src/main/flight/failsafe.cpp` | Lines 122, 206, 211-216, 245-250 — the decision and the bug |
| `src/main/flight/failsafe.h` | Lines 35, 39, 64 — the constants and the unread field |
| `src/main/rx/rx.cpp` | Lines 87, 92, 313, 369, 431, 528, 536, 550-556 — the timing chain |
| `src/main/config/config.cpp` | Lines 469, 544 — the defaults |
| `src/main/io/serial_cli.cpp` | Line 456 — the CLI variable |
| `src/main/io/serial_msp.cpp` | Lines 1256, 1734 — the app's read/write |
