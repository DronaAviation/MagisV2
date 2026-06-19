<div align="center">

<h1>MagisV2</h1>
<p><strong>Open-source flight controller firmware for the Pluto Drone family by Drona Aviation</strong></p>

[![License: GPL v3](https://img.shields.io/badge/License-GPLv3-blue.svg)](LICENSE)
[![Target](https://img.shields.io/badge/Target-PRIMUS__X2__v1-orange.svg)]()
[![IDE](https://img.shields.io/badge/IDE-PlutoIDE%20VSCode-007ACC?logo=visual-studio-code)](https://marketplace.visualstudio.com/items?itemName=Drona-Aviation.pluto-ide)

</div>

---

## What is MagisV2?

MagisV2 is the firmware that powers the **Pluto Drone**, a programmable micro quadcopter by [Drona Aviation](https://www.dronaaviation.com/). It is a fork of the Baseflight/Cleanflight flight controller stack, rebuilt and extended with a clean **user-facing API** that lets developers write custom flight behaviour directly in C++, without needing to understand the full internals of the flight controller.

Think of it as an **Arduino-style programming model for a real drone**: you write `plutoInit()` and `plutoLoop()`, and the firmware handles all the low-level stabilisation, sensor fusion, motor control, and communication.

**Supported hardware targets:**

| Target | Board | MCU |
|--------|-------|-----|
| `PRIMUS_X2_v1` | Primus X2 *(default)* | STM32F303xC, 72 MHz, 40 KB RAM |
| `PRIMUS_V5` | Primus V5 | STM32F30x |
| `PRIMUSX2` | Primus X2 *(legacy)* | STM32F30x |

---

## Getting Started

### Prerequisites

- **[PlutoIDE VS Code Extension](https://marketplace.visualstudio.com/items?itemName=Drona-Aviation.pluto-ide):** handles toolchain, build, flash, and monitoring automatically
- **Visual Studio Code:** [download here](https://code.visualstudio.com/)
- A **Pluto Drone** (Primus X2 or compatible)

---

## Setting Up with PlutoIDE

PlutoIDE is the official VS Code extension for building and flashing MagisV2 firmware. It manages the ARM toolchain, IntelliSense, and project scaffolding automatically. You do not need to clone this repository manually.

### Step 1 - Install PlutoIDE

1. Open VS Code
2. Press `Ctrl+Shift+X` to open the Extensions panel
3. Search for **Pluto IDE**
4. Click **Install**

> PlutoIDE will automatically install two required extensions on first run:
> - **C/C++** by Microsoft (`ms-vscode.cpptools`): for IntelliSense
> - **Teleplot** by Alex Nesnes (`alexnesnes.teleplot`): for live sensor plotting

### Step 2 - Create a New Project

1. Click the **Dashboard** icon in the VS Code status bar (bottom), or run `PlutoIDE > Open Dashboard` via `Ctrl+Shift+P`
2. In the dashboard, click **Create New Project**
3. Fill in your **Project Name** and choose a **Destination Directory**
4. Select your **Board Type** (e.g. MagisV2 for Primus X2)
5. Check the **"Include Sources"** checkbox - this automatically downloads the latest MagisV2 firmware release from GitHub and sets up the full source project
6. Click **Create**. The workspace opens with all files and IntelliSense configured.

> **Project Types:**
> - **Source (SRC) - Include Sources checked:** Downloads the full editable firmware source. Use this when you want to customise core behaviour or work with driver-level code.
> - **Library - Include Sources unchecked:** Creates a lighter project that links against pre-compiled board libraries. Use this for writing user application code only (`PlutoPilot.cpp`).

### Step 3 - Select Target

Click the **Select Target** button in the VS Code status bar (bottom), or run:

```
PlutoIDE > Select Target
```

Choose `PRIMUS_X2_v1` for the Primus X2 drone.

### Step 4 - Write Your Code

All user code lives in **`PlutoPilot.cpp`**, the only file you need to edit:

```cpp
#include "PlutoPilot.h"

// Choose your receiver type
void plutoRxConfig ( void ) {
    Receiver_Mode ( Rx_ESP );    // Onboard ESP Wi-Fi (default)
    // Receiver_Mode ( Rx_ELRS );   // ExpressLRS (CRSF) on USART1
    // Receiver_Mode ( Rx_PPM );    // PPM receiver
    // Receiver_Mode ( Rx_CAM );    // Wi-Fi camera mode
}

// Runs once on power-up
void plutoInit ( void ) {
    Oled_Init();
}

// Runs once when Developer Mode is activated
void onLoopStart ( void ) {
}

// Runs in a loop while Developer Mode is active
void plutoLoop ( void ) {
    Oled_Begin();
    Oled_Text(0, 0, "Hello, Pluto!");
    Oled_Number(0, 16, FC_Data_Get(FC_ALTITUDE));
    Oled_End();
}

// Runs once when Developer Mode is deactivated
void onLoopFinish ( void ) {
    Oled_Clear();
}
```

### Step 5 - Build

Click **Build** in the status bar or run:

```
PlutoIDE > Build
```

### Step 6 - Flash

**USB:**
1. Connect your Pluto via USB
2. Click **USB Flash** (PlutoIDE installs the STM32 bootloader driver automatically on first use)

**Wi-Fi:**
1. Connect to your drone's Wi-Fi network
2. Click **WiFi Flash**

```
PlutoIDE > USB Flash
PlutoIDE > WiFi Flash
```

---

## PlutoIDE Quick Reference

| Action | Status Bar Button | Command Palette |
|--------|------------------|-----------------|
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

## API Reference

For the full API reference, see the **[MagisV2 Wiki](https://github.com/DronaAviation/MagisV2/wiki)**.

---


## ExpressLRS (ELRS) Support

MagisV2 supports **ExpressLRS** receivers via CRSF protocol on USART1.

### Wiring

Connect the ELRS receiver to the **CAM port** on the Primus X2. The CAM port exposes the following pins:

| CAM Port Pin | ELRS Receiver Pin | Notes |
|--------------|-------------------|-------|
| `RX-1` | `TX` | Serial data into the flight controller |
| `GND` | `GND` | Common ground |
| `vBAT` | `VCC` | Battery voltage out - check receiver input voltage rating |

> **Note:** `TX-1` on the CAM port is the flight controller's transmit line. It is not required for basic RC control but can be used if your receiver supports telemetry input. `vBAT` supplies raw battery voltage - use a voltage regulator if your receiver requires 3.3 V or 5 V.

### CRSF Protocol Parameters

| Parameter | Value |
|-----------|-------|
| Baud rate | 420,000 |
| Format | 8N1, non-inverted |
| Channels | 16 (11-bit packed, raw range: 172-1811, RC range: 988-2012) |
| Frame types | RC Channels `0x16`, Link Stats `0x14`, Battery Telemetry `0x08` |

### AUX Channel Mapping (default)

| AUX | Channel | Function | Trigger |
|-----|---------|----------|---------|
| AUX1 | CH5 | ARM | 2-pos switch HIGH = armed |
| AUX2 | CH6 | ANGLE / ACRO | 3-pos: low = ACRO, mid+high = ANGLE |
| AUX3 | CH7 | MAG | Switch HIGH (optional) |
| AUX4 | CH8 | Developer Mode | Switch HIGH |
| AUX5 | CH9 | ALT HOLD / THROTTLE | 2-pos: low = THROTTLE, high = ALT HOLD |

### Battery Telemetry

Battery voltage from the onboard INA219 sensor is sent back to the radio via CRSF frame type `0x08`. To view it on EdgeTX/OpenTX, go to **Model Setup - Telemetry - Discover sensors** and look for **RxBt**.


---

## Arming Sequence

1. Power on the drone and place it on a flat surface
2. Wait for the gyro/accelerometer calibration to complete (LED stops blinking)
3. Move the throttle stick fully down
4. Flip **AUX1 (CH5)** HIGH → drone arms

---

## Repository Structure

```
MagisV2/
├── PlutoPilot.cpp          ← Your code goes here (user entry point)
├── PlutoPilot.h            ← Includes all user APIs
├── plutoide.ini            ← PlutoIDE project config
│
├── src/main/
│   ├── API/                ← Public C++ API headers
│   │   ├── RxConfig.h      ← Receiver mode selection
│   │   ├── FC-Data.h       ← Flight controller sensor data
│   │   ├── FC-Control.h    ← Arming, flight modes
│   │   ├── Motor.h         ← Motor control
│   │   ├── Oled.h          ← OLED display driver
│   │   ├── Peripherals.h   ← GPIO, I2C, SPI, UART
│   │   ├── Serial-IO.h     ← Serial communication
│   │   ├── Debugging.h     ← Monitor_Print / debug output
│   │   ├── Scheduler-Timer.h ← Task scheduling
│   │   └── XRanging.h      ← ToF/optical flow sensor API
│   │
│   ├── drivers/            ← Hardware drivers (IMU, barometer, I2C, SPI)
│   ├── flight/             ← PID, attitude estimation, mixer
│   ├── rx/                 ← Receiver protocols (CRSF/ELRS, PPM, MSP)
│   │   ├── crsf.c          ← CRSF protocol driver (RX + battery telemetry TX)
│   │   └── crsf.h
│   ├── sensors/            ← Barometer, magnetometer, optical flow
│   ├── telemetry/          ← FrSky, HoTT, MSP telemetry
│   └── target/             ← Board-specific config
│       ├── PRIMUS_X2_v1/   ← Default target (Primus X2)
│       ├── PRIMUS_V5/
│       └── PRIMUSX2/
│
├── lib/                    ← Third-party libraries (CMSIS, VL53L1X, STM32 HAL)
├── docs/                   ← Documentation and API wikis
└── graphify-out/           ← Knowledge graph (interactive codebase explorer)
```

---

## Interactive Codebase Explorer

A pre-built knowledge graph of the entire codebase is included in [`graphify-out/`](graphify-out/):

- **[`graphify-out/graph.html`](graphify-out/graph.html):** Download and open in any browser. Shows all 4,547 functions, modules, and relationships as an interactive graph with 477 detected communities.
- **[`graphify-out/GRAPH_REPORT.md`](graphify-out/GRAPH_REPORT.md):** Plain-language audit report with god nodes, surprising connections, and suggested exploration questions.

---

## Documentation

For guides, API references, and configuration details, visit the **[MagisV2 Wiki](https://github.com/DronaAviation/MagisV2/wiki)**.


---

## Contributing

1. Fork this repository
2. Create a feature branch: `git checkout -b feature/my-feature`
3. Commit your changes: `git commit -m "feat: add my feature"`
4. Push and open a Pull Request

---

## Support

- 🐛 **Issues:** [Open an issue on GitHub](https://github.com/DronaAviation/MagisV2/issues)
- 🌐 **Website:** [dronaaviation.com](https://www.dronaaviation.com/)

---

## License

This project is licensed under the **GNU General Public License v3.0**. See [LICENSE](LICENSE) for details.

© 2025 Drona Aviation. All rights reserved.
