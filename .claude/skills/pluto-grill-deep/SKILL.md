---
name: pluto-grill-deep
description: The pluto-grill planning skill run on Fable 5.1 instead of Opus 5.5, for work where the analysis is hard. Started only by the user with /pluto-grill-deep; never invoked automatically.
argument-hint: "[check|fix|feature|improve|docs] <what you want>  |  fix <topic>"
model: claude-fable-5-1
effort: high
disable-model-invocation: true
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

# pluto-grill-deep

Read [`../pluto-grill/SKILL.md`](../pluto-grill/SKILL.md) now and follow it
exactly, with the arguments given here. Everything in it applies unchanged,
including the template, the modes and *After the plan*.

The only difference is the model: this turn runs on Fable 5.1. Subagents stay
on their own models ( `pluto-scout` and `pluto-reviewer` on Opus 5.5 ); do not
pass `fable` to any Agent call.
