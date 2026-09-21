# MagisV2 commit message template

Taken from this repo's history. Best models: `2a8d59a` (feat), `4657cbf` (fix),
`b91e6a7` (chore), `d08760d` (feat with API change). Read one with
`git log -1 --format=%B <hash>` before drafting.

## Shape

```
<type>(<scope>): <imperative summary, lower case, no full stop, ≤ 72 chars>

<paragraph: problem → change → measured result → version line>

Details:
- <one bullet per distinct change: what, with numbers, constants and names>
- ...

<type>(<scope>): <summary of a secondary part of the same commit>

<short paragraph>
```

- **A blank line after the subject is required.** Without it git treats the
  whole first paragraph as the subject, so `git log --oneline` and GitHub show
  one huge line. `c569c0f` has this defect. Check the draft with
  `git log -1 --format=%s` after committing, or look at line 2 of the draft file:
  it must be empty.
- **Secondary parts** (the docs promotion, a tooling change in the same commit)
  get their own `type(scope): summary` line and a short paragraph after the
  details, separated by blank lines. See `4657cbf`, which carries a `docs(...)`
  and a `chore(tooling)` part.
- A small commit (one change) is subject + paragraph only, with no Details list
  (`278f77d`, `f3f9a95`).

## Type and scope

| Type | Use for |
|---|---|
| `feat` | new behaviour or API |
| `fix` | a bug: wrong behaviour on hardware, corruption, crash |
| `refactor` | no behaviour change intended |
| `docs` | docs only |
| `chore` | tooling, skills, agents, build scripts, graph data |

Scope is the subsystem in the form already used: `altitudeHold`, `rc`,
`RGB-LED`, `oled`, `compass`, `bms`, `drivers`, `sensors`, `ranging`, `led`,
`failsafe`, `tooling`, `changelog`. Reuse an existing scope before inventing one:
`git log --format=%s | sed 's/:.*//' | sort | uniq -c`.

## The paragraph

In this order, as plain prose:

1. **Problem**, as seen on the drone or by the user ("Altitude hold sank
   steadily while the log reported a stable altitude").
2. **Change**: what the firmware does now, in behaviour terms.
3. **Evidence**: the measured result, with the board and method
   ("Checked against a VL53L0X laser on PRIMUS_V5, a 4 minute hover now holds
   within about ±8 cm"). Leave it out if nothing was measured. Do not invent it.
4. **Version line**, always last, in exactly one of these forms:
   - `The firmware version has been updated from X to Y; the public API is unchanged (A).`
   - `The firmware version has been updated from X to Y and the API from A to B (behaviour only, no signature change).`
   - `The firmware version has been updated from X to Y and the API from A to B.`
   - Omit it for `docs` / `chore` commits that do not touch firmware.

## Details bullets

- Start with the part touched ("Barometer:", "Stick:", "ICP-10111:") or a verb
  ("Fix", "Remove", "Name", "Hold").
- Give real values with units and the constant names: `ALT_MAX_CLIMB_CMS`,
  `30 -> 300 deci-degrees`, `EEPROM_CONF_VERSION 106 -> 107`.
- Call out anything a flyer must know: settings reset on first boot, a changed
  default, a behaviour that feels different.
- File or function names are fine. Line numbers are not; they go stale.

## Never

- No AI attribution: no `Co-Authored-By:` trailer, no "Generated with" line, no
  tool or model names.
- No "This commit ...", "This update addresses ...", "aims to enhance". Say what
  changed.
- No items that belong to another commit or an already-shipped release.
