---
name: pluto-task
description: Carry out a planned MagisV2 / Pluto topic one task at a time from its TASKS.md - next, status, done, add, resume. Use when the user runs /pluto-task, says "next", "next task", "continue", "carry on", "status", "where are we", "done <n>", "add a task", or resumes a topic in a new session. Loads the skills each task names (pluto-rules for code, pluto-driver for hardware, pluto-build for the gate, pluto-flighttest for tests), delegates to c-pro / cpp-pro when a task says so, runs pluto-reviewer on every code task, and keeps the Resume block current. Planning a new topic is /pluto-grill, not this skill.
argument-hint: "next | status | resume | done <n> [note] | add <text> [--topic <name>]"
model: claude-opus-5-5
effort: high
allowed-tools:
  - Read
  - Grep
  - Glob
  - Bash(graphify query *)
  - Bash(graphify explain *)
  - Bash(graphify path *)
  - Bash(graphify update *)
  - Bash(python tools/graph_labels.py)
  - Bash(git status *)
  - Bash(git log *)
  - Bash(.claude/skills/pluto-build/driver.sh *)
  - Bash(python tools/warnings.py check *)
  - Bash(python tools/warnings.py summary *)
  - Bash(python tools/flightlog.py *)
---

# pluto-task: work the plan

Executes the topic `/pluto-grill` wrote, **one task per turn**. Every step reads
as little as possible: the Resume block, the index and the one task.

```
/pluto-task next              do the next open task, then stop
/pluto-task status            index + next task, nothing else
/pluto-task resume            new session: read Resume, report, wait for "next"
/pluto-task done <n> [note]   mark task n done
/pluto-task add <text>        scope it with 1-2 questions, append it
--topic <name>                pick the topic when more than one is Active
```

## Find the topic

The Active row(s) in `docs/fw-development-reference/active-development/README.md`.
One → use it. Several → `--topic`, or ask which. None → tell the user to plan
with `/pluto-grill`.

## Context rules

- Read `TASKS.md` **Resume** and **Index** and the decisions log; then only the
  current task's section. Do not read the other topic docs unless the task
  names them. `SCOUT.md` is there for facts the task needs: read the part you
  need, do not re-scout.
- Graph before grep ( `graphify explain` / `path` ). Read source only where you
  edit or cite.
- Subagents get a self-contained prompt: topic path, target, the task section
  verbatim, the files, and the output you want. Launch in the background when
  you have other work; follow-ups to the same agent via SendMessage.
- **Update the Resume block at the end of every turn** ( current task, next
  step, open questions, blocked on, date ). It is what the next session reads.

## `next`

1. **Pick** the lowest-numbered task that is `in-progress`, else the lowest
   `todo` whose *Depends on* are all done. Tasks that are `blocked` are skipped
   and reported.
2. **Load** the skills in its *Skills / agent* field. Any task touching
   `src/main` loads `pluto-rules` before the first edit, without exception.
3. **Mini-grill** if the task has an unknown the plan did not settle: 1-3
   questions with `AskUserQuestion`; log the answers.
4. **Set `in-progress`** in the index and the task, update Resume.
5. **Do the task** by its kind:

| Kind | How |
|---|---|
| Investigation / measurement | Evidence into `INVESTIGATION.md` / `TESTING.md`. Large logs to `pluto-log-analyst` ( Sonnet 5 ) with the log path and the question. |
| Code | Follow `pluto-rules`. If the task names `c-pro` / `cpp-pro`, delegate with the task section, files and rules; review what comes back, do not paste it blind. Record each change in `CHANGES.md` ( file, line, why ); create it from the topic layout on first use, as with `TESTING.md`. |
| Hardware / driver | `pluto-driver` checklist; maps updated in the same task. |
| Build gate | `.claude/skills/pluto-build/driver.sh --gate <TARGET>`: no new `src/` warnings, flash/RAM fits. Report the numbers. |
| Hardware validation | `pluto-flighttest` writes the test plan and log fields. Create the next numbered `<topic>/logs/log-N.txt`, open it with `code <path>`, and **stop**: the user flies and pastes the log. The next `/pluto-task next` analyses it into `TESTING.md`. |
| Pipeline docs | New text into `PIPELINE_UPDATE.md` ( never the pipeline folder ); diagram edges confirmed with `graphify path` or a source line; maps, API wiki, `dev-guide/`, CLAUDE.md one-liners as the task lists. |
| Topic review | `pluto-reviewer` on the whole topic diff; fix its BLOCKING findings ( each fix goes through `pluto-rules` and the gate again ). |
| Graph refresh & commit | After the review fixes: `graphify update .` then `python tools/graph_labels.py`, then hand over to `/pluto-commit` ( the user commits ). |
| Findings ( `check` ) | Ranked causes with their evidence into `INVESTIGATION.md`, a recommendation, and the README summary. |

6. **Verify** against the task's *Done when*. For any task that changed code,
   launch `pluto-reviewer` ( Opus 5.5 ) on that task's files and fix its BLOCKING
   findings before closing; list the rest in the report.
7. **Close**: `done YYYY-MM-DD` and a one-line *Result*; README open items and
   `CHANGES.md` current; Resume points at the next task.
8. **Report** in a few lines: what was done, numbers, reviewer outcome, next
   task. **Stop.** Do not run into the next task unless the user says so.

New work found mid-task is never done silently: append it as a new task ( next
free number, never renumber ), set its *Depends on*, and mention it.

A task that cannot finish ( waiting for a flight, a board, a decision ) is set to
`blocked (<why>)` with the Resume block saying what unblocks it.

## `status`

Print the index table and the next runnable task. Nothing else.

## `resume`

For a fresh session: read Resume, the index and the decisions log; report in
up to 8 lines where the topic stands and what `next` would do. Wait.

## `done <n> [note]`

Mark it done with the date and the note; update Resume.

## `add <text>`

Ask 1-2 questions to scope it ( and whether it is safety-relevant ), fill every
template field, append with the next serial number, and place it before the
closing tasks in the **Order** line.

## When the topic is finished

After the graph refresh & commit task hands over to `/pluto-commit`, the topic
is closed there ( status, commit hash, promoted pipeline docs, Resume block ).

A `check` topic ends with its findings task. Then offer the two ways out:
`/pluto-grill fix <topic>` ( plan the fix from the findings ) or `/pluto-commit`
to record the findings and close the topic ( docs only, no firmware build ).
Either way the topic stops being Active.
