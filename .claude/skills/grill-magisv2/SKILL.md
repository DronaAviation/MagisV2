---
name: grill-magisv2
description: Interview the user before any MagisV2 firmware work - implementing a feature, changing behaviour or workflow, or fixing a bug - then write a numbered TASKS.md for the topic and work it one task at a time. Use this FIRST, before reading or editing code, whenever the user describes a problem or a change in plain words with no active TASKS.md covering it - e.g. "I'm facing this issue", "X is not working", "the drone drifts / sinks / flips", "I want to change / add / implement", "can we make it do", "fix", "bug" - and also when they run /grill-magisv2, say "grill me", "plan this" or "make a task list", or say "next", "status" or "done <n>" for an existing list. Skip it for pure questions about the code, trivial one-line edits, or when the user says "no grill" / "just do it". Uses the magisv2-scout agent and graphify for context, routes each task to the right project skill, and makes the architecture / pipeline doc updates explicit tasks.
---

# Grill, plan, then work one task at a time

The purpose is to get from what the user says to a task list both sides agree
on **before any firmware is touched**. On this drone a wrong guess about intent
costs a flight, so ask questions until nothing important is left to guess.

```
/grill-magisv2 <what you want>   → new topic: scout + grill + TASKS.md
/grill-magisv2 next              → do the next open task in the active topic
/grill-magisv2 status            → show the task index of the active topic
/grill-magisv2 done <n> [note]   → mark task n done
/grill-magisv2 add <text>        → append a task (asks 1-2 questions first)
```

## When this runs without being typed

Start this skill on your own when the user describes a problem or change
("I'm facing …", "X doesn't work", "I want to change …") and no active
topic's `TASKS.md` already covers it. Say so in one line ("Planning this first
with grill-magisv2. Say *no grill* to skip.") and go to *Mode: new topic*.

- **Already covered** by an active topic's TASKS.md → use *Mode: add* (or
  *next*), not a new topic.
- **Do not start it** for questions about how the code works, trivial edits
  (a typo, a single constant the user gave the value for), flight-log analysis on
  its own, or when the user says "no grill", "just do it" or "skip planning".
  If a question turns into a request for work, start it then.

Task lists live in the topic folder, next to the other topic docs:
`docs/fw-development-reference/active-development/<topic>/TASKS.md`
(template: [TASKS_TEMPLATE.md](TASKS_TEMPLATE.md), folder rules:
[active-development/README.md](../../../docs/fw-development-reference/active-development/README.md)).

## Context budget rules (apply in every mode)

The main conversation holds decisions, not file dumps.

- **Reconnaissance goes to the `magisv2-scout` agent.** It reads the graph, the
  pipeline docs and the code, and returns a brief of 400 words or less. Do not
  re-read in the main thread what it already summarised. Ask it again
  (SendMessage to the same agent) for more detail on one point.
- **Graph before grep.** For "where / how does X connect" use
  `graphify query "<q>" --budget 800`, `graphify path "A" "B"`,
  `graphify explain "X"`. Read a source file only for a line you need to cite.
- **Progressive disclosure in TASKS.md.** The index table comes first. When
  resuming, read the index and the one task being worked on, not the whole file
  and not the other topic docs.
- **Write decisions down as they are made** in the `Decisions log` of TASKS.md
  (`YYYY-MM-DD [decision|assumption|out-of-scope|risk] text`). A later session
  then needs the log, not the whole conversation.

## Mode: new topic

### 1. Start the scout, start asking

In the **same message**:

- launch `magisv2-scout` in the background with the user's request verbatim
  plus the working target if known;
- ask **Round 1** (below) with `AskUserQuestion`. Round 1 does not need the scout.

Target: follow CLAUDE.md. If the session's working target is not known yet, it
is one of the Round 1 questions (suggest `selected_target` from `plutoide.ini`).

### 2. Grill in rounds

Use `AskUserQuestion`: up to 4 questions per call, 2-4 concrete options each,
recommended option first and marked `(Recommended)`. Options should come from
what the code allows (constants, modes, pins the scout found), not invented
ones. Use free-text questions ("Other") for symptoms and numbers.

Ask a round, update your understanding, then ask the next round. **Never ask
something the code or the scout brief already answers. State it as a fact
instead and let the user correct it.**

**Round 1: is this the right work? (ask first, in order)**
1. What kind of work: bug fix / new feature / behaviour change / tuning /
   refactor / investigation only?
2. Does this need to exist, or is something already there that does it (an API
   call, a mode, a constant)? Put what the scout found as an option.
3. Which target is flown (PRIMUS_V5 / PRIMUS_X2_v1)?
4. What does "done" look like, in something observable (log field, behaviour in
   flight, number)?

**Round 2+: pick the rounds that apply**

| Area | Ask about |
|---|---|
| Bug | Exact symptom, when it happens (armed? which mode? after how long?), how often, when it last worked (commit / FW version), log available (`logs*.txt`)? |
| Feature / behaviour | Who uses it (user code via `PlutoPilot.cpp`, pilot, MSP / app)? Default on or off? Interaction with the pilot's sticks (`rcDataPilot`, override cross-fade)? |
| Flight mode & safety | Which modes are affected (ANGLE, ALT_HOLD, landing, failsafe)? What happens on RC loss or low battery while it runs? Arming / disarm interaction? **Safety paths are never cut to save scope.** |
| Public API | Does a header under `src/main/API/` change? New function or changed behaviour? API version bump and `docs/API/` wiki expected? |
| Hardware | New pin, DMA channel, timer, I2C/SPI device, ADC? Conflicts from `PIN_MAP` / `DMA_MAP` / `TIMER_MAP` (scout lists them). |
| Numbers | Rates, limits, loop frequency, units. Where no value is known: measure first, or pick a guess the user signs off? |
| Budget | Flash/RAM headroom concerns? Loop-time cost acceptable? |
| Verification | Bench test, flight test, which log fields? Who flies, and when? |
| Scope | What is explicitly **not** part of this? Anything to reuse or avoid from a previous topic? |

Stop grilling when every row that applies has an answer, or is written in the
decisions log as `assumption` or `out-of-scope`. Most work takes 2-4 rounds.
If the user says "enough", stop and log what is still open as assumptions.

### 3. Play it back

Before writing anything, reply with **at most 12 lines**: the goal, done-when,
target, affected subsystems / pipeline docs, key decisions, assumptions, out of
scope. Ask for confirmation. Correct and repeat until the user confirms.

### 4. Write the topic

Pick a short kebab-case topic name (confirm it in the playback). Create
`active-development/<topic>/`:

- `README.md`: status **Active**, branch, target, summary, open items (same
  layout as the existing topics).
- `TASKS.md` from [TASKS_TEMPLATE.md](TASKS_TEMPLATE.md).
- `INVESTIGATION.md` only for a bug or investigation: the problem, evidence and
  the Q&A that shaped it.

Add the topic row to the table in `active-development/README.md`.

### 5. How to cut the tasks

- **One task = one reviewable step** that ends with the firmware building
  (or with a doc / log finished). Aim for roughly 4-12 tasks. Split anything that
  touches two subsystems or needs a flight in between.
- Each task has a **serial number, title and description**. The description is
  2-5 sentences: what changes, why, where (files / functions from the scout),
  and anything from the decisions log it depends on. Add **Skills**, **Done when**
  and **Status** fields as in the template.
- Order: investigation / measurement → code, in dependency order → verification
  → docs → commit.
- **Route each task to its skill or agent** (listed in the task's Skills field):

| Task touches | Use |
|---|---|
| Any code under `src/main` | `magisv2-rules` (before and while editing) |
| A new driver, peripheral, pin, DMA, timer, ADC | `add-driver` |
| Building / warning gate | `run-magisv2` (working target only) |
| A log to read, a test plan, diagnostic log fields | `flight-test`, `flightlog-analyst` agent for big logs |
| Plain C drivers / ISR / DMA | `c-pro` agent; C++ API layer: `cpp-pro` agent; MCU-level design: `embedded-systems` |
| Constants with units (cm/s, Pa, counts) | `dimensional-analysis` |
| Code checked against a pipeline doc or datasheet | `spec-to-code-compliance` |
| A new public API | `sharp-edges` (is it easy to misuse from `PlutoPilot.cpp`?) |
| Review of a task's diff or the finished diff | `magisv2-reviewer` agent (always a subagent, never inline) |
| Commit | `commit-magisv2` |

- **Always end with these tasks** (drop one only if it clearly does not apply,
  and say why in the decisions log):
  1. **Build gate**: `run-magisv2` with `--gate` on the working target. No new
     warnings under `src/`, and flash/RAM fits.
  2. **Hardware validation**: a bench or flight test from `flight-test`, with
     results in `TESTING.md`.
  3. **Architecture & pipeline docs**: write the new version of every affected
     `fw-architecture-pipeline/` doc (overview `Firmware_Pipeline.md` and each
     `subsystems/*.md`) into the topic's `PIPELINE_UPDATE.md`. Do not edit the
     pipeline folder directly. Include updated Mermaid flowcharts where the flow
     changed. **Every edge in a diagram must be a real call or data path**:
     confirm it with `graphify path` or the source line, and do not draw an
     edge because it "should" exist. Also update `DMA_MAP` / `TIMER_MAP` /
     `PIN_MAP` for resource changes, the `docs/API/` wiki for API changes, and
     `CLAUDE.md` for new rules or gotchas.
  4. **Graph refresh**: `graphify update .` then `python tools/graph_labels.py`.
  5. **Review & commit**: `magisv2-reviewer` agent on the whole topic diff, then
     `commit-magisv2`, which
     promotes `PIPELINE_UPDATE.md` and the CHANGELOG.

Show the index table to the user and stop. **Do not start task 1 in the same
turn**. The user starts work with `/grill-magisv2 next`.

## Mode: next

1. Find the active topic (the row in `active-development/README.md` whose status
   is Active. If there are several, ask which one). Read its `TASKS.md` index
   and decisions log only.
2. Take the lowest-numbered task that is `todo` (or `in-progress`). Read only its
   section. Load the skills listed in its Skills field.
3. If the task has an unknown the list did not settle, ask 1-3 questions first
   (a mini-grill) and log the answers.
4. Set it to `in-progress`, do it, and verify it against its **Done when**. If
   the task changed code, launch the `magisv2-reviewer` agent on that task's
   files and fix its BLOCKING findings before closing (list the rest in the
   report). Then set it to `done` with the date and a one-line result. Keep `README.md` open items
   and `CHANGES.md` current as the topic rules require.
5. Report what was done and what the next task is, then **stop**. Do not run into
   the next task unless the user says so.

If new work turns up while doing a task, do not do it silently. Add it as a new
task (next free serial number, never renumber) and mention it in the report.

## Mode: status / done / add

- `status`: print the index table and the next task. Nothing else.
- `done <n> [note]`: mark it done with the date and the note.
- `add <text>`: ask 1-2 questions to scope it, append it with the next serial
  number, and place it before the closing tasks in the **Order** line.
