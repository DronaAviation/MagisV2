# wiki_check.py self-test

Fixture for [wiki_check.py](wiki_check.py). Run it after any change to the harness; the expected result for each
block is written above it. Expected total: 8 checked, 3 passed ( 1 of them WARN-OK ), 5 failed, 1 skipped, exit 1.

Expected SKIP ( a header quote ):

<!-- wiki-check: skip -->
```cpp
typedef enum BMS_Option {
  Voltage,
  Current
} BMS_Option_e;
```

Expected PASS:

```cpp
void plutoLoop ( void ) {
  uint16_t mv = Bms_Get ( Voltage );
  Monitor_Println ( "mV: ", mv );
}
```

Expected FAIL compile ( undeclared function ):

```cpp
void plutoLoop ( void ) {
  Not_A_Real_Api ( 3 );
}
```

Expected FAIL link ( `SPI_Init ( )` is declared in `Serial-IO.h` but never defined, finding F1 ):

```cpp
void plutoInit ( void ) {
  SPI_Init ( );
}
```

Expected FAIL warning ( `3.7f` selects the int overload, `-Wfloat-conversion` ):

```cpp
void plutoLoop ( void ) {
  Monitor_Print ( "v:", 3.7f );
}
```

Expected WARN-OK ( the same warning, marked as intended ):

<!-- wiki-check: warn-ok -->
```cpp
void plutoLoop ( void ) {
  Monitor_Print ( "v:", 3.7f );
}
```

Expected FAIL no hook ( a fragment outside any hook ):

```cpp
uint16_t mv = Bms_Get ( Voltage );
```

Expected PASS ( an explicit include of a header that `PlutoPilot.h` does not pull in ):

```cpp
#include "API/Localisation.h"

void plutoInit ( void ) {
  Localisation.init ( UWB );
}
```

Expected FAIL compile ( a file-level name that collides with a firmware global: `Peripherals.h` declares
`extern bool _isAdcEnable [ ]`, so a user `static` of that name cannot compile ):

```cpp
static bool _isAdcEnable = false;

void plutoLoop ( void ) {
  _isAdcEnable = true;
}
```
