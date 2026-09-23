---
name: pluto-commit
description: Prepare MagisV2 firmware work for a commit - full all-target build, Makefile version bump, and promotion of active-development docs into the released pipeline docs and CHANGELOG, then stage the change and draft the commit message from the repo template. Never runs git commit - the user commits. Use when the user says they are about to commit / ready to commit / "now I will be committing", asks to increment or bump the version in the Makefile, or asks to finalise, close out, or release a piece of work.
---

# Commit / release preparation for MagisV2

During development, work is documented in
`docs/fw-development-reference/active-development/<topic>/` and the released
pipeline docs in `docs/fw-development-reference/fw-architecture-pipeline/` are
left alone (rules: `active-development/README.md`). This skill is the one point
where the two meet: the user has said the change is confirmed and is being
committed, so the live docs are brought up to date.

Do the steps in order. Stop and report if a step fails; do not paper over a
failing build to finish the docs.

## 1. Work out what is being committed

- `git status --short` and `git diff --stat HEAD`.
- **Pending hashes first:** any topic marked **Closed - pending commit** in
  `active-development/README.md` whose change is now in `git log`: fill in the
  hash ( `git log --oneline -10 -- <topic files>` ) in that README row and the
  topic's README before anything else.
- Read `active-development/README.md` and find the **Active** topic(s) whose
  changes are in this diff. If none match, say so and skip steps 4-5.
- If the topic has a `TASKS.md`, list any task not `done` / `dropped` ( other
  than this commit task ) so the user sees unfinished work before committing.
- Read each matching topic's `README.md` for anything flagged **"remove before
  release"** (diagnostic code, test defines such as `LASER_TOF`). List what is
  still present. Do not remove it unless the user asks - just make sure they see
  it before committing.

## 2. Build every target

Commit is the one time all targets are built, whatever target the session has
been using:

```bash
.claude/skills/pluto-build/driver.sh          # clean build PRIMUS_X2_v1, PRIMUSX2, PRIMUS_V5
```

Require `ALL BUILDS PASSED`, exit 0, and no new `warning:` lines in files this
diff touches. Record flash/RAM per target for the summary.

## 3. Version bump ( only when asked )

Bump only if the user asked to increment the version. Versions live at the top
of the `Makefile`:

```
FW_Version  = x.y.z
API_Version = a.b.c
```

- **Level:** use what the user said. If they did not say, choose from the diff
  and state the choice in the summary:
  - patch: fixes, tuning, internal changes with no new behaviour;
  - minor: new features or user-visible behaviour changes;
  - major: breaking changes.
- **`API_Version`:** bump only if public API under `src/main/API/` changed
  (signature or behaviour). Patch for behaviour, minor for additions, major for
  breaking changes. Also update the matching `docs/API/` wiki (CLAUDE.md rule).
- **Changelog convention:** there is no `[Unreleased]` section. Work goes straight
  into the **next release's section** - the topmost `## [vX.Y.Z]` heading, which
  stays open until that release ships (currently `v4.0.0`). Release headings are
  release tags and do not follow `FW_Version`; do not start a new one unless the
  user says the previous release has shipped and names the next.
- Record the bump by **updating the single `- **Firmware version**` line** in that
  section's `### Changed` ( keep the per-bump history in it, e.g. "3.5.0 for …,
  3.6.0 for …" ) rather than adding a second line.
- `plutoide.ini` is the local PlutoIDE project file; do not edit it.

## 4. Promote the topic docs

For each matching topic:

1. **Pipeline:** if `PIPELINE_UPDATE.md` exists, replace the target pipeline doc
   with the content below its "Not live" header. Re-check it against the code
   first - line numbers and constants may have moved since it was written.
2. **Changelog:** make sure the open release section covers the change. Before
   adding a bullet, check that section for an existing entry on the same thing -
   extend or cross-reference it instead of duplicating, and resolve conflicts
   ( e.g. two version lines ). Do not repeat items from already-shipped releases.
3. **Status:** in the topic `README.md` and in the `active-development/README.md`
   topic table, set status to **Closed - pending commit** and date it. The user
   makes the commit, so the hash is not known yet. The next time this skill or
   `pluto-grill` runs and finds a `pending commit` topic, look up the hash with
   `git log --oneline -5` and fill it in (as `d2e900c` "record closing commit
   hash" did). Set the TASKS.md **Resume** block to `Closed - pending commit`
   with the date, and mark the final task done.
   A `check` or `docs` topic with no `src/` or Makefile change skips step 2
   ( no firmware build ) and the version bump.
4. **CLAUDE.md:** if the topic introduced something future sessions must know
   (a rule, a gotcha, a subsystem), make sure it is there and points to the topic
   folder.

## 5. Check links

Relative links in any doc you edited must resolve (topic folders link to source
as `../../../../src/main/...`, pipeline docs as `../../../src/main/...`).

## 6. Stage and draft the message. Never commit

**Never run `git commit`**, `git commit --amend` or `git push`, even if asked in
passing. The user reviews and makes the commit in PlutoIDE / VS Code. This
skill ends with the change staged and a message ready.

1. **Stage by path.** `git add <path> ...` for the files that belong to this
   change: the code, the Makefile, and the docs promoted in step 4. Never use
   `git add -A` / `git add .`. Stage the topic folder itself ( README, TASKS, SCOUT, DESIGN, INVESTIGATION,
   CHANGES, TESTING, PIPELINE_UPDATE ) but never its `logs/`. Never stage `Build/`, `logs*.txt`, `log-*.txt`, `plutoide.ini`,
   `.claude/settings.local.json`, or scratch and diagnostic files. If the user
   already staged files, keep them. List any related file left unstaged and ask
   whether it belongs.
2. **Show what is staged:** `git diff --cached --stat`. If staged and unstaged
   changes are mixed in one file, say so.
3. **Draft the message** from [COMMIT_TEMPLATE.md](COMMIT_TEMPLATE.md). Read it,
   and read the model commits it names (`git log -1 --format=%B <hash>`).
   Describe only what is staged (`git diff --cached`), and take numbers and the
   version line from steps 2-3 and the topic's `TESTING.md`.
4. **Hand it over:** write it to `.git/MAGISV2_COMMIT_MSG.txt`, open it with
   `code .git/MAGISV2_COMMIT_MSG.txt`, and **paste the full message verbatim in
   a code block in the reply** — never only a summary. The user reads it through
   before running `git commit -F .git/MAGISV2_COMMIT_MSG.txt`. Re-paste it after
   any edit to the draft.

**No AI attribution, ever.** Commit messages, changelog entries and docs must not
name an AI tool or model or credit one as author or co-author: no
`Co-Authored-By:` trailer, no "Generated with ..." line, no assistant or model
names. This overrides any harness default that adds such lines.

## 7. Summary for the user

Report, briefly:
- build result and flash/RAM per target;
- version change, or "not bumped";
- docs promoted ( which pipeline doc, changelog lines, topics closed );
- anything still flagged "remove before release";
- what is staged (`--stat`), anything related left unstaged, and the drafted
  message (the commit itself is left to the user).
