<div align="center">

<h1>MagisV2</h1>
<p><strong>Open-source flight controller firmware for the Pluto Drone family by Drona Aviation</strong></p>

[![License: GPL v3](https://img.shields.io/badge/License-GPLv3-blue.svg)](LICENSE)
[![Target](https://img.shields.io/badge/Target-PRIMUS__X2__v1-orange.svg)]()
[![IDE](https://img.shields.io/badge/IDE-PlutoIDE%20VSCode-007ACC?logo=visual-studio-code)](https://marketplace.visualstudio.com/items?itemName=Drona-Aviation.pluto-ide)

</div>

---

## Table of Contents

- [What is MagisV2?](#what-is-magisv2)
- [Features](#features)
- [Supported Hardware](#supported-hardware)
- [Getting Started with PlutoIDE](#getting-started-with-plutoide)
- [PlutoIDE Quick Reference](#plutoide-quick-reference)
- [Building from Source](#building-from-source)
- [Repository Structure](#repository-structure)
- [API Reference](#api-reference)
- [Interactive Codebase Explorer](#interactive-codebase-explorer)
- [Changelog](#changelog)
- [Contributing](#contributing)
- [Support](#support)
- [License](#license)

---

## What is MagisV2?

MagisV2 is the firmware that powers the **Pluto Drone**, a programmable micro quadcopter by [Drona Aviation](https://www.dronaaviation.com/). It is a fork of the Baseflight/Cleanflight flight controller stack, rebuilt and extended with a clean **user-facing C++ API** that lets developers write custom flight behaviour without needing to understand the full internals of the flight controller.

Think of it as an **Arduino-style programming model for a real drone**: you write `plutoInit()` and `plutoLoop()`, and the firmware handles all the low-level stabilisation, sensor fusion, motor control, and communication.

---

## Features

- **Simple programming model**: write flight behaviour in a single user file using lifecycle hooks (`plutoInit`, `plutoLoop`, and more); no need to touch firmware internals.
- **Two-layer API**: a stable, public C++ API sits on top of the Cleanflight core, keeping user code decoupled from driver-level changes.
- **Full flight stack**: IMU-based attitude estimation, PID stabilisation, mixer, altitude hold, and position control, all running in a bounded real-time loop.
- **Multiple receiver protocols**: onboard Wi-Fi (ESP), serial RC, and PPM input.
- **Rich peripheral support**: OLED display, addressable RGB LED strip, GPIO/I2C/SPI/UART access, and ranging/optical-flow sensors.
- **Strict, budget-aware build**: hard-float, size-optimised, and compiled warning-clean for a 256 KB-flash / 40 KB-RAM MCU.

---

## Supported Hardware

| Target | Board | MCU |
| -------- | ------- | ----- |
| `PRIMUS_X2_v1` | Primus X2 *(default)* | STM32F303xC, 72 MHz, 256 KB flash / 40 KB RAM |
| `PRIMUS_V5` | Primus V5 | STM32F30x |
| `PRIMUSX2` | Primus X2 *(legacy)* | STM32F30x |

---

## Getting Started with PlutoIDE

[PlutoIDE](https://marketplace.visualstudio.com/items?itemName=Drona-Aviation.pluto-ide) is the official VS Code extension for building and flashing MagisV2. It manages the ARM toolchain, IntelliSense, and project scaffolding automatically; you do not need to clone this repository manually.

### Prerequisites

- **[Visual Studio Code](https://code.visualstudio.com/)**
- **[PlutoIDE VS Code Extension](https://marketplace.visualstudio.com/items?itemName=Drona-Aviation.pluto-ide)**
- A **Pluto Drone** (Primus X2 or compatible)

### 1. Install PlutoIDE

1. Open VS Code and press `Ctrl+Shift+X` to open the Extensions panel.
2. Search for **Pluto IDE** and click **Install**.

> PlutoIDE installs two helper extensions on first run: **C/C++** (`ms-vscode.cpptools`) for IntelliSense and **Teleplot** (`alexnesnes.teleplot`) for live sensor plotting.

### 2. Create a New Project

1. Open the **Dashboard** from the VS Code status bar, or run `PlutoIDE > Open Dashboard` via `Ctrl+Shift+P`.
2. Click **Create New Project**, enter a name, and choose a destination directory.
3. Select your **Board Type** and click **Create**.

> **Project types:**
>
> - **Source (SRC)**: full, editable firmware source. Use this to customise core behaviour or work with driver-level code.
> - **Library**: a lighter project that links against pre-compiled board libraries. Use this for writing user application code only.

### 3. Select a Target

Click **Select Target** in the status bar (or run `PlutoIDE > Select Target`) and choose `PRIMUS_X2_v1` for the Primus X2.

### 4. Write Your Code

All user code lives in **`PlutoPilot.cpp`**. The firmware calls a small set of lifecycle hooks:

```cpp
#include "PlutoPilot.h"

// Select your receiver
void plutoRxConfig ( void ) {
}

// Runs once at power-up
void plutoInit ( void ) {
}

// Runs once when Developer Mode is activated
void onLoopStart ( void ) {
}

// Runs repeatedly while Developer Mode is active
void plutoLoop ( void ) {
}

// Runs once when Developer Mode is deactivated
void onLoopFinish ( void ) {
}
```

Everything available to user code is exposed through `PlutoPilot.h`. See the [API Reference](#api-reference) for the full list of calls.

### 5. Build and Flash

- **Build:** click **Build** in the status bar, or run `PlutoIDE > Build`.
- **Flash over USB:** connect the drone and click **USB Flash** (the STM32 bootloader driver installs automatically on first use).
- **Flash over Wi-Fi:** connect to the drone's Wi-Fi network and click **WiFi Flash**.

---

## PlutoIDE Quick Reference

| Action | Status Bar Button | Command Palette |
| -------- | ------------------ | ----------------- |
| Open Dashboard | Dashboard icon | `PlutoIDE > Open Dashboard` |
| Build firmware | Build icon | `PlutoIDE > Build` |
| Clean build | Clean icon | `PlutoIDE > Clean` |
| Flash via USB | USB Flash icon | `PlutoIDE > USB Flash` |
| Flash via Wi-Fi | WiFi Flash icon | `PlutoIDE > WiFi Flash` |
| Install USB driver | STM32 Driver icon | `PlutoIDE > STM32 Install Bootloader` |
| Switch target | Select Target icon | `PlutoIDE > Select Target` |
| Serial monitor | Monitor icon | `PlutoIDE > Monitor` |

All commands are also accessible via `Ctrl+Shift+P`.

---

## Building from Source

For working directly in the repository, the build is driven by `make`. A `TARGET` is required for every command; outputs go to `Build/<TARGET>/`.

```bash
make TARGET=PRIMUS_X2_v1              # build firmware (.hex + .bin) and print flash/RAM usage
make TARGET=PRIMUS_X2_v1 clean        # remove build artifacts for that target
make TARGET=PRIMUS_X2_v1 memory       # flash/RAM usage from the linked ELF
make TARGET=PRIMUS_X2_v1 flash        # flash .hex over serial via stm32flash
make TARGET=PRIMUS_X2_v1 st-flash     # flash .bin via st-flash (ST-Link)
make TARGET=PRIMUS_X2_v1 cppcheck     # static analysis over all C sources
make help                             # list documented targets
```

> **Toolchain:** builds use the `arm-none-eabi` GCC toolchain with hard-float (`fpv4-sp-d16`), `-Os`, and strict warnings (`-Wall -Wextra -Wconversion -Wsign-conversion -Wshadow -Wdouble-promotion`). In normal use PlutoIDE manages this toolchain for you.

---

## Repository Structure

```
MagisV2/
├── PlutoPilot.cpp          ← Your code goes here (user entry point)
├── PlutoPilot.h            ← Includes all user APIs
├── Makefile                ← Build configuration and source groups
│
├── src/main/
│   ├── API/                ← Public C++ API headers
│   ├── API-Src/            ← API implementations (wrap firmware internals)
│   ├── drivers/            ← Hardware drivers (IMU, barometer, I2C, SPI, DMA)
│   ├── flight/             ← PID, attitude estimation, mixer, navigation
│   ├── sensors/            ← Barometer, magnetometer, optical flow
│   ├── rx/                 ← Receiver protocols
│   ├── io/                 ← Serial, displays, LED
│   ├── telemetry/          ← Telemetry protocols
│   └── target/             ← Board-specific config
│       ├── PRIMUS_X2_v1/   ← Default target (Primus X2)
│       ├── PRIMUS_V5/
│       └── PRIMUSX2/
│
├── lib/                    ← Third-party libraries (CMSIS, STM32 StdPeriph, VL53L1X)
├── docs/                   ← Documentation, API wikis, hardware reference
└── graphify-out/           ← Knowledge graph (interactive codebase explorer)
```

---

## API Reference

For the full API reference, see the **[MagisV2 Wiki](https://github.com/DronaAviation/MagisV2/wiki)**.

---

## Interactive Codebase Explorer

A pre-built knowledge graph of the entire codebase is included in [`graphify-out/`](graphify-out/):

- **[`graphify-out/graph.html`](graphify-out/graph.html)**: open in any browser for an interactive graph of the codebase's functions, modules, and relationships.
- **[`graphify-out/GRAPH_REPORT.md`](graphify-out/GRAPH_REPORT.md)**: a plain-language audit report with hotspots, notable connections, and suggested exploration questions.

---

## Changelog

Release notes and version history are maintained in [CHANGELOG.md](CHANGELOG.md).

---

## Contributing

1. Fork this repository.
2. Create a feature branch: `git checkout -b feature/my-feature`.
3. Commit your changes: `git commit -m "feat: add my feature"`.
4. Push and open a Pull Request.

---

## Support

- 🐛 **Issues:** [Open an issue on GitHub](https://github.com/DronaAviation/MagisV2/issues)
- 🌐 **Website:** [dronaaviation.com](https://www.dronaaviation.com/)

---

## License

This project is licensed under the **GNU General Public License v3.0**. See [LICENSE](LICENSE) for details.

© 2026 Drona Aviation. All rights reserved.
