---
name: windows-pluto-toolchain-path
description: "On Windows, PlutoIDE's ARM toolchain is at C:\\PlutoIDE and is NOT auto-added to the persistent system PATH"
metadata: 
  node_type: memory
  type: project
  originSessionId: 40199355-844d-455f-9ef5-a7a2597d3270
---

On the maintainer's Windows machine the PlutoIDE `arm-none-eabi` GCC toolchain
lives at `C:\PlutoIDE\tools\ARM GNU ToolChain\bin` (a fixed root, NOT under the
user profile). Version: Arm GNU Toolchain 14.2.Rel1 (14.2.1).

**Key point:** the PlutoIDE VS Code extension does NOT add the toolchain to the
persistent system/user PATH. It spawns its own terminal with the PATH set for
that session only, runs build/clean inside it, and the PATH is gone once the
terminal closes. So a fresh Git Bash / WSL shell will NOT have
`arm-none-eabi-g++` on PATH by default.

The `~/.pluto-ide/tools/...` location that the `run-magisv2` driver probes does
NOT exist on this Windows machine — that path is Linux/macOS-only. The
maintainer added `C:\PlutoIDE\...\bin` to the system PATH manually, but that is
NOT the intended/default design — do not assume it is present.

To build from a fresh shell on Windows, put the bin on PATH first:
`export PATH="/c/PlutoIDE/tools/ARM GNU ToolChain/bin:$PATH"`

Documented in `.claude/skills/run-magisv2/SKILL.md` (Prerequisites + Troubleshooting).
