---
name: commit-magisv2
description: Prepare MagisV2 firmware work for a commit - full all-target build, Makefile version bump, and promotion of active-development docs into the released pipeline docs and CHANGELOG. Use when the user says they are about to commit / ready to commit / "now I will be committing", asks to increment or bump the version in the Makefile, or asks to finalise, close out, or release a piece of work.
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
- Read `active-development/README.md` and find the **Active** topic(s) whose
  changes are in this diff. If none match, say so and skip steps 4-5.
- Read each matching topic's `README.md` for anything flagged **"remove before
  release"** (diagnostic code, test defines such as `LASER_TOF`). List what is
  still present. Do not remove it unless the user asks - just make sure they see
  it before committing.

## 2. Build every target

Commit is the one time all targets are built, whatever target the session has
been using:

```bash
.claude/skills/run-magisv2/driver.sh          # clean build PRIMUS_X2_v1, PRIMUSX2, PRIMUS_V5
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
   topic table, set status to **Closed**, date it, and write the commit hash. If
   the commit has not been made yet, write `Closed - pending commit` and fill in
   the hash straight after the commit.
4. **CLAUDE.md:** if the topic introduced something future sessions must know
   (a rule, a gotcha, a subsystem), make sure it is there and points to the topic
   folder.

## 5. Check links

Relative links in any doc you edited must resolve (topic folders link to source
as `../../../../src/main/...`, pipeline docs as `../../../src/main/...`).

## 6. Commit only if asked

Do not run `git commit` unless the user asked for the commit itself. If they
did, follow the repository commit conventions ( conventional prefix such as
`fix(altitudeHold):`, a paragraph, then a `Details:` list ), then fill the hash
into the topic status.

**No AI attribution, ever.** Commit messages, changelog entries and docs must not
name an AI tool or model or credit one as author or co-author: no
`Co-Authored-By:` trailer, no "Generated with ..." line, no assistant or model
names. This applies to messages written for the user to paste as well as to
commits made directly.

## 7. Summary for the user

Report, briefly:
- build result and flash/RAM per target;
- version change, or "not bumped";
- docs promoted ( which pipeline doc, changelog lines, topics closed );
- anything still flagged "remove before release";
- whether the commit was made, and its hash.
