# Building & Debugging in VSCode

The MagisV2 firmware project has deprecated legacy IDEs like Eclipse and complex Cygwin environments in favor of a modernized, unified development workflow using **Visual Studio Code** and the **Pluto IDE Extension**.

## Prerequisites
1. **Visual Studio Code**: Download and install the latest version of VSCode for your operating system (Windows, macOS, or Linux).
2. **Pluto IDE Extension**: Open the Extensions tab (`Ctrl+Shift+X` or `Cmd+Shift+X`) in VSCode, search for `Pluto IDE`, and install it.
3. **ARM Toolchain**: Depending on your exact Pluto IDE setup, ensure you have the `arm-none-eabi-gcc` toolchain available in your system PATH, or let the extension manage the toolchain installation automatically.

## Workspace Setup
1. Clone the MagisV2 repository to your local machine:
   ```bash
   git clone https://github.com/MagisV2/MagisV2.git
   ```
2. Open the cloned folder in VSCode.
3. The Pluto IDE extension should automatically detect the MagisV2 workspace structure.

## Building the Firmware
The Pluto IDE extension abstracts away the raw Makefiles. To compile the code for the active `PRIMUS` target:
1. Open the **Pluto IDE** panel in the VSCode sidebar.
2. Select the **Build** option or click the Build icon in the status bar.
3. The extension will invoke the appropriate `make TARGET=PRIMUS` commands in the background and output the compilation logs to the terminal.
4. Upon success, the compiled `.hex` and `.elf` files will be placed in the `obj/` directory.

## Flashing & Hardware Debugging
The Pluto IDE extension also provides integrated flashing and debugging capabilities, bypassing the need to manually invoke `OpenOCD` or `GDB` from the command line.

1. Connect your PRIMUS board via USB.
2. Ensure any necessary ST-Link or DFU drivers are installed.
3. Click the **Flash** button in the Pluto IDE panel to upload the generated `.hex` file.
4. To step-debug the firmware, click the **Debug** button. This will automatically launch a GDB server, connect to the board, and halt at `main()`, allowing you to set breakpoints directly in the VSCode editor.
