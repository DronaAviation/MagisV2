---
name: pluto-grill
description: Plan any MagisV2 / Pluto firmware work before code is touched - interview the user in detail, analyse with the pluto-scout agent, and write a topic folder with a numbered TASKS.md that /pluto-task then works one task at a time. Five modes - check (investigate an issue, no code change), fix (resolve a bug), feature (propose design options for something new), improve (refactor or optimise without changing behaviour), docs (write or correct documentation). Use FIRST, before reading or editing firmware, whenever the user describes work in plain words and no active TASKS.md covers it - "check why…", "look into…", "I'm facing…", "X is not working", "fix…", "the drone drifts / sinks / flips", "add / implement / can we make it…", "improve / clean up / optimise…", "document…" - or runs /pluto-grill, says "grill me", "plan this" or "make a task list". Skip for pure questions about the code, trivial one-line edits, flight-log reading on its own, or "no grill" / "just do it". Executing tasks is /pluto-task, not this skill.
argument-hint: "[check|fix|feature|improve|docs] <what you want>  |  fix <topic>"
model: claude-opus-5-5
effort: high
allowed-tools:
  - Read
  - Grep
  - Glob
  - Bash(graphify query *)
  - Bash(graphify explain *)
  - Bash(graphify path *)
  - Bash(git log *)
  - Bash(git rev-parse *)
---

# pluto-grill: interview, analyse, plan

Turn what the user says into a task list both sides agree on **before any
firmware is touched**. On this drone a wrong guess about intent costs a flight:
ask until nothing important is left to guess.

This skill **plans only**. It ends by writing the topic folder and stopping.
Tasks are carried out with `/pluto-task` ( see *After the plan* ).

```
/pluto-grill <what you want>          new topic; mode detected, confirmed in round 1
/pluto-grill check|fix|feature|improve|docs <what>   new topic, mode given
/pluto-grill fix <topic>              turn a finished `check` topic into a fix plan
/pluto-grill-deep <…>                 same skill on Fable 5.1 ( user-invoked only )
```

## 1. When this starts on its own

Start it without being asked when the user describes work in plain words and
no active topic's `TASKS.md` covers it. Say so in one line: *"Planning this first
with pluto-grill ( mode: fix ). Say no grill to skip."*

- **Covered by an active topic** → `/pluto-task add <text>` instead.
- **An existing Planned topic** ( e.g. one waiting for hardware ) → plan it in
  place: its folder gets `TASKS.md` and `SCOUT.md`, its README keeps its notes
  as input, and its status becomes **Active**. Do not create a second folder.
- **Argument is a topic?** If the text after the mode matches an existing
  folder under `active-development/`, it is a topic, not a description.
- **Do not start** for: questions about how the code works, a typo, a single
  constant whose value the user gave, reading a flight log on its own
  ( `pluto-flighttest` ), or "no grill" / "just do it" / "skip planning". If a
  question turns into a request for work, start then.
- Never start `/pluto-grill-deep` yourself. It exists for the user to choose Fable.

### Mode detection

| Mode | The user says | Output of the plan |
|---|---|---|
| `check` | "check / look into / why does / is it normal that…" | Investigation tasks only. No firmware change. Ends with ranked causes and a recommendation. |
| `fix` | "fix / resolve / it's broken / X is not working, make it work" | Reproduce → root cause → fix → regression test → docs. |
| `feature` | "add / implement / new / can we make it…" | 2-3 design options first; the user picks; then tasks for the chosen one. |
| `improve` | "improve / clean up / refactor / optimise / speed up / reduce RAM" | Baseline numbers → change → proof behaviour is unchanged. |
| `docs` | "document / update the docs / wiki / the doc is wrong" | Doc tasks checked against the code. |

If two modes fit ( "check and fix" ), plan `check` first: a fix without a
confirmed cause is a guess. Confirm the mode as the first question of round 1.

## 2. Models, subagents and context

**Models.** This skill runs on Opus 5.5 ( `/pluto-grill-deep` on Fable 5.1 ).
Subagents never use Fable:

| Agent | Model | Used for |
|---|---|---|
| `pluto-scout` | Opus 5.5 | reconnaissance and analysis brief |
| `pluto-reviewer` | Opus 5.5 | every review ( in `/pluto-task` ) |
| `c-pro` / `cpp-pro` | Opus 5.5 | delegated implementation ( in `/pluto-task` ) |
| `pluto-log-analyst` | Sonnet 5 | large flight logs |

**Launching subagents well.** A subagent sees none of this conversation.

- Launch in the **background, in the same message** as the next question round,
  so the user answers while it works.
- Give it everything it needs in the prompt: the request verbatim, the mode, the
  working target, the topic folder path, answers already given, and the exact
  question you want answered.
- Ask for its fixed output layout and word limit ( the scout has one ).
- **Follow-ups go to the same agent** with SendMessage. It keeps what it already
  read; a new agent starts from zero.
- **Ambiguous request:** launch two scouts in parallel, one per reading, each
  told which reading it owns.
- **Mode changed in round 1** ( the scout started with the detected mode ):
  SendMessage the scout the confirmed mode so its analysis matches.

**Context preservation.** The main conversation holds decisions, not file dumps.

- Do not re-read what the scout summarised. Read a source file only to cite a line.
- Graph before grep: `graphify query "<q>" --budget 800`, `graphify explain "X"`,
  `graphify path "A" "B"`.
- Save the scout brief to the topic as `SCOUT.md` the moment the topic exists, so
  no later session repeats the reconnaissance.
- Write each decision to the TASKS.md decisions log as it is made.

## 3. Interview

### Round 1: in the same message as the scout launch

Use `AskUserQuestion`: up to 4 questions per call, 2-4 concrete options each,
recommended first and marked `(Recommended)`. Round 1 does not need the scout.

1. **Mode** ( detected mode as the recommended option ).
2. **Target flown**: `PRIMUS_V5` / `PRIMUS_X2_v1` ( suggest `selected_target` from
   `plutoide.ini`; skip if already known this session ).
3. **Done when**: something observable ( a log field, a behaviour in flight, a number ).
4. **Scope edge**: what is explicitly not part of this.

### Round 2+: the mode's questions, then the shared ones

Ask a round, update your understanding, ask the next. **Never ask what the code
or the scout already answers: state it as a fact and let the user correct it.**
Options come from what the code allows ( constants, modes, pins the scout found ).
Use free text ( "Other" ) for symptoms and numbers. Ask everything the code
cannot answer; the user has asked for a thorough interview.

**`check`: investigate**
- Exact symptom in the user's words; when ( armed? which mode? after how long?
  on which surface or battery level? ).
- How often: every flight, sometimes, once? Reproducible on the bench?
- When it last behaved: commit, FW version, before which change?
- Evidence available: `logs*.txt`, video, which log fields? What to log if none.
- What the user already suspects or ruled out.
- What would count as an explanation ( "a cause with a log that shows it" ).

**`fix`: resolve**
- All of `check`, plus:
- Is the cause known ( from a `check` topic or the scout ), or does the plan
  start by confirming it?
- Acceptable fix: smallest change, or the proper one? Any constraint ( no API
  change, no new constant, must work on both targets )?
- How will we know it is fixed, and that nothing else broke ( regression flights )?
- Rollback: what do we fly if the fix misbehaves?

**`feature`: suggestion first**
- Who uses it: user code ( `PlutoPilot.cpp` API ), the pilot, the app over MSP?
- The problem it solves, in one sentence. Is there something already there that
  does it ( scout finds candidates; offer them as options )?
- Default on or off? Which flight modes? How it interacts with the pilot's sticks
  ( `rcDataPilot`, override cross-fade ) and with failsafe.
- Public API: new function or changed one → `docs/API/` wiki, `API_Version` bump.
- Constraints: flash/RAM, loop time, pins, both targets or one.
- Then **present 2-3 design options** ( see Analysis ) and ask the user to pick.

**`improve`: no behaviour change**
- Goal in numbers: flash, RAM, loop time, warning count, readability of which files?
- What must stay identical ( outputs, timing, log fields, API )?
- How equivalence is shown: same flight log fields within X, same build output
  size budget, bench comparison.
- Files in or out of scope; vendored `lib/` is always out.

**`docs`: documentation**
- Audience: user-code developers ( `docs/API/` wiki ), firmware maintainers
  ( `fw-architecture-pipeline/`, `dev-guide/` ), or hardware maps?
- Which doc is wrong or missing, and what triggered it?
- Must diagrams be redrawn ( every edge must be a real call path )?
- Checked against code with `spec-to-code-compliance`?

**Shared rounds: ask the rows that apply**

| Area | Ask about |
|---|---|
| Flight safety | Affected modes ( ANGLE, ALT_HOLD, landing, failsafe, flip )? RC loss or low battery while it runs? Arm / disarm interaction? **Safety paths are never cut to save scope.** |
| Hardware | New pin, DMA channel, timer, I2C/SPI device, ADC? Conflicts from `PIN_MAP` / `DMA_MAP` / `TIMER_MAP` ( the scout lists them ). |
| Numbers | Rates, limits, loop frequency, units. Unknown value: measure first, or a guess the user signs off? |
| Budget | Flash/RAM headroom, loop-time cost, `Monitor_Print` bytes per tick. |
| Verification | Bench or flight, which log fields, who flies and when. |
| Documentation | Which pipeline docs, maps, API wiki or CLAUDE.md lines change. |
| History | A previous topic on this area to reuse or avoid ( scout checks ). |

Stop when every row that applies has an answer or is logged as `assumption` /
`out-of-scope`. If the user says "enough", stop and log what is open.

## 4. Analysis

Wait for the scout brief. Check it against the answers, then do the mode's analysis:

| Mode | Analysis written before the playback |
|---|---|
| `check` | 2-5 **ranked hypotheses**, each with evidence for / against and the test that would confirm or kill it. |
| `fix` | The **causal chain** symptom → mechanism → code ( file:line ); if not proven, the first task proves it. |
| `feature` | **2-3 design options**, each: how it works, files touched, flash/RAM/loop cost, safety impact, API impact, risk. Recommend one. The user picks before tasks are cut. |
| `improve` | **Baseline**: current numbers to beat ( or "measure in task 1" ) and the equivalence test. |
| `docs` | **Drift list**: each doc statement that disagrees with the code, with file:line. |

Anything the scout could not settle becomes a `risk` or `assumption` in the log.

## 5. Playback

Before writing anything, reply with **at most 15 lines**: mode, goal, done-when,
target, the analysis result ( ranked causes / chosen option / baseline ),
affected subsystems and pipeline docs, key decisions, assumptions, out of scope,
proposed topic name. Ask for confirmation. Correct and repeat until confirmed.

## 6. Write the topic

`docs/fw-development-reference/active-development/<topic>/` ( short kebab-case name ):

- `README.md`: status **Active**, mode, branch, target, summary, open items
  ( same layout as existing topics ).
- `TASKS.md` from [TASKS_TEMPLATE.md](TASKS_TEMPLATE.md), Resume block filled in.
- `SCOUT.md`: the scout brief, verbatim.
- `INVESTIGATION.md` for `check` and `fix`: problem, evidence, hypotheses, the Q&A
  that shaped it.
- `DESIGN.md` for `feature`: the options, the choice and why.

Add the topic row to the table in `active-development/README.md`.
Folder rules: [active-development/README.md](../../../docs/fw-development-reference/active-development/README.md).

## 7. Cut the tasks

- **One task = one reviewable step** that ends with the firmware building, or a
  doc / log finished. Usually 4-12 tasks. Split anything that touches two
  subsystems or needs a flight in between.
- Fill every field of the template: description ( 2-5 sentences: what, why, where,
  which decisions it relies on ), depends on, skills / agent, safety impact,
  done when ( with a number where possible ), rollback for code tasks.
- Order: investigation / measurement → code in dependency order → verification →
  docs → commit.
- **Mode shapes the list:** `check` has no code tasks ( it ends with a findings
  task and a recommendation ); `feature` starts from the chosen option; `improve`
  starts with a baseline task and ends with an equivalence task; `docs` has only
  doc tasks plus a `spec-to-code-compliance` check.

**Routing** ( the task's Skills field ):

| Task touches | Use |
|---|---|
| Any code under `src/main` | `pluto-rules` ( before and while editing ) |
| New driver, peripheral, pin, DMA, timer, ADC | `pluto-driver` |
| Plain-C driver / ISR / register code, self-contained | `c-pro` agent |
| C++ module or API layer, self-contained | `cpp-pro` agent |
| Build, warning gate, flash/RAM | `pluto-build` ( working target, `--gate` ) |
| Test plan, log fields, reading a log | `pluto-flighttest`; `pluto-log-analyst` agent for big logs |
| Code checked against a doc or datasheet | `spec-to-code-compliance` |
| Review of a task's diff or the topic diff | `pluto-reviewer` agent ( always a subagent ) |
| Commit | `pluto-commit` |

**Closing tasks** ( drop one only if it clearly does not apply, and log why ):

1. **Build gate**: `pluto-build --gate` on the working target; no new `src/`
   warnings; flash/RAM fits. ( Not for `check` or `docs`. )
2. **Hardware validation**: bench or flight test from `pluto-flighttest`, results
   in `TESTING.md`. ( `improve`: the equivalence comparison. )
3. **Architecture & pipeline docs**: new version of every affected
   `fw-architecture-pipeline/` doc into the topic's `PIPELINE_UPDATE.md` ( never
   edit the pipeline folder directly ), Mermaid flowcharts where the flow changed
   ( **every edge a real call or data path**, confirmed with `graphify path` or a
   source line ), `DMA_MAP` / `TIMER_MAP` / `PIN_MAP`, the `docs/API/` wiki,
   `dev-guide/` and one-line CLAUDE.md rules for new gotchas.
4. **Topic review**: `pluto-reviewer` on the whole topic diff; BLOCKING findings
   fixed ( and re-gated ) before moving on.
5. **Graph refresh & commit**: after the review fixes, `graphify update .` and
   `python tools/graph_labels.py`, then `/pluto-commit`. ( Refreshing after the
   fixes keeps the committed graph current. )

**`check` topics** have no code, so they close differently: investigation tasks →
a **Findings** task ( ranked causes, evidence, recommendation ) → **Close**:
`/pluto-commit` records the findings and sets the topic Closed ( no firmware
build ), unless the user moves on with `/pluto-grill fix <topic>`.
**`docs` topics** keep 3-5 and drop 1-2.

## 8. After the plan

Show the TASKS.md index and **stop. Do not start task 1 in the same turn.** Tell
the user the next step:

```
/pluto-task next      do the next open task
/pluto-task status    show the index
/pluto-task resume    continue in a fresh session from the Resume block
```

### `check` → `fix`

When a `check` topic's findings task is done and the user wants the fix:
`/pluto-grill fix <topic>`. Read that topic's `SCOUT.md`, `INVESTIGATION.md` and
decisions log instead of scouting again; ask only the `fix` questions they do not
answer; then either append fix tasks to the same topic ( small fix ) or open a new
`fix` topic that links back ( anything larger ). Ask which.

Appending to the same topic: set **Mode** to `fix` in the Summary; mark any
unfinished `check` closing task `dropped (superseded by fix tasks)`; append the
fix tasks and then the full set of fix closing tasks with new serial numbers;
update **Order** and Resume. A new topic: set the `check` topic Closed with a
link to the new one.
