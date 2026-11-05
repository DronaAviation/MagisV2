# RcCommand_Get API - Investigation

[README](README.md) · [TASKS](TASKS.md) · [SCOUT](SCOUT.md) · [INVESTIGATION](INVESTIGATION.md)

## Problem

API test B12 ( build ): calling `int16_t *RcCommand_Get ( void )` from user code fails to link with an undefined
reference; the source file defines `RcCommand_get`. The test note asks for the first four channels.

## Causal chain ( proven )

1. **Link error.** `RC-Interface.h:76` declares `RcCommand_Get ( void )`; `RC-Interface.cpp` ( HEAD ) defined
   `RcCommand_get ( void )`. C++ names are case-sensitive, so the declared symbol had no definition. The user's rename
   fixes this.
2. **Wrong value under the name.** The body fills a local `int16_t rcCommandArray [ 4 ]` and returns its address. The
   array dies when the function returns, so the pointer is dangling ( undefined behaviour ). GCC warns
   ( `-Wreturn-local-addr`, `RC-Interface.cpp:110`, baselined ) and, as it does for this case, returns NULL instead:
   the object code is `movs r0, #0; bx lr`.
3. **What the user sees.** `RcCommand_Get ( ) [ i ]` reads address `0x0 + 2 i`. On the STM32F303 address 0 aliases the
   boot flash, so it reads the initial stack pointer and reset vector: fixed garbage, no fault, no sticks.

## Fix decided

- File-static `int16_t [ 4 ]` snapshot, refreshed on each call; return its pointer. The signature stays.
- Element values: roll / pitch / yaw `1500 + rcCommand [ i ]` ( same as `RcCommand_Get ( channel )`, the
  `RcCommand_Set` range ), throttle `rcCommand [ THROTTLE ]` as is.

## Q&A ( 5 Nov 2026 )

| Question | Answer |
|---|---|
| How far? | Fix it properly ( not the rename only ). |
| Return mechanism | Static snapshot ( not a caller buffer, not a pointer to the live `rcCommand` ). |
| Scope | Only `RcCommand_Get ( )`. `RcData_Get ( )` ( same bug ) and the channel getter's throttle offset are out of scope. |
| Element values | Same as the channel getter for roll / pitch / yaw. |
| Throttle element | `rcCommand [ THROTTLE ]`, what the flight controller commands ( not the `rcData` stick value ). |
| Done when | Bench log with the app sticks, props off. |
| Docs | Header comments plus a new `docs/API/RC_INTERFACE_API_WIKI.md`. |
