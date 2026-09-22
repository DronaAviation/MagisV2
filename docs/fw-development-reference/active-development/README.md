# Active Development

Working documentation for firmware changes that are **in progress**: what is
being changed, why, how, and the evidence that it works. One folder per topic.

This is the counterpart to [`../fw-architecture-pipeline/`](../fw-architecture-pipeline/),
which describes the firmware **as committed**. Keeping the two apart means the
pipeline docs never describe behaviour that is still being tested, and the
reasoning behind a change is kept even after it ships.

## Rules

1. **Start a topic folder** here when a piece of work needs more than a commit
   message to explain: an investigation, a multi-step fix, anything tested on
   hardware.
2. **Keep it current while the work is live.** Record changes, measurements and
   dead ends as they happen, not at the end.
3. **Do not touch `fw-architecture-pipeline/` during the work.** If a pipeline
   doc will need changing, write the new version into the topic's
   `PIPELINE_UPDATE.md` instead.
4. **At commit, once the change is confirmed correct:**
   - copy `PIPELINE_UPDATE.md` over the pipeline doc it targets;
   - update `CHANGELOG.md`, and any `docs/API/` wiki if public API changed;
   - set the topic's status to **Closed** here and in its README, with the commit hash.
5. **Closed topics stay.** They are the record of why the code is the way it is.
   Do not delete them; mark anything superseded.

## Topic folder layout

| File | Contents |
|---|---|
| `README.md` | Status, branch, target, one-paragraph summary, gotchas, open items, next actions. Start here. |
| `TASKS.md` | Numbered task list ( serial number, title, description, status ) and dated decisions log, written by the `grill-magisv2` skill after the planning interview and worked one task at a time. |
| `INVESTIGATION.md` | **Why**: the problem, evidence, root causes, decisions. Readable without the code. |
| `CHANGES.md` | **What and how**: each code change with file and line links, and a file index. |
| `TESTING.md` | **Evidence**: test method, measurements, results, how to reproduce. |
| `PIPELINE_UPDATE.md` | Staged text for the pipeline doc(s), applied at commit. Only if a pipeline doc changes. |

Links to source from a topic folder use `../../../../src/main/...`.

## Topics

| Topic | Status | Branch | Summary |
|---|---|---|---|
| [vl53l1x-althold-parity](vl53l1x-althold-parity/README.md) | **Planned** ( 22 Sep 2026 ) | - | Bring the VL53L1X ( `LASER_TOF_L1x` ) path to the VL53L0X handover / object logic from `tof-althold-fusion`; its branch still has a 350 cm hard switch and never checks out-of-range ( do not fly it above ~2.5 m with `LASER_ALT` ). Waiting for hardware. |
| [tof-althold-fusion](tof-althold-fusion/README.md) | **Closed - pending commit** ( FW 3.9.0, 22 Sep 2026 ) | `BugFix-June26` | VL53L0X altitude hold bobs and reacts hard to objects underneath: hard 200 cm source switch, no hysteresis, no step rejection, tau flips per source. Fixed: accelerometer Z deadband ( the bob ), one tau, 160/140 cm handover with frozen offset and frame shift, 2.5 s object hold-off with a window test; flight-validated on PRIMUS_X2_v1. |
| [flip-althold-regression](flip-althold-regression/README.md) | **Closed** in `c569c0f` ( FW 3.8.1, 21 Sep 2026 ) | `BugFix-June26` | App back-flip stopped rotating after `2a8d59a`: ALT_HOLD stays on through AUX3 and the shaped 40 cm/s stick cap stops ASCEND reaching its 100 cm/s threshold. Fix: the flip bypasses setpoint shaping, with a clean handover on exit. |
| [althold-setpoint-shaping](althold-setpoint-shaping/README.md) | **Closed** in `2a8d59a` ( FW 3.8.0, 18 Sep 2026 ) | `BugFix-June26` | Throttle stick moves the altitude setpoint ( ArduPilot / DJI style ) instead of switching to raw velocity control; commanded altitudes flown on a trapezoidal goal profile; landing keeps its own descent rate; controller held in reset while armed at idle. |
| [altitude-hold](altitude-hold/README.md) | **Closed** in `4657cbf` ( FW 3.7.0, 18 Sep 2026 ) | `BugFix-June26` | Barometer altitude hold sank in flight. Four firmware bugs fixed, throttle and temperature compensation added, validated on one PRIMUS_V5 with a laser. |
