# ADBMS Interface Usage

This folder contains the ADBMS interface layer:

- `ADBMS6830_commands.h`: ADBMS command constants.
- `ADBMS6830_driver.h`: Low-level SPI driver and PEC helpers.
- `ADBMS_interface.h`: High-level BMS interface (telemetry + balancing).

## Basic Integration

1. Include the interface header:

```cpp
#include "ADBMS/ADBMS_interface.h"
```

2. Create the SPI driver and interface instances:

```cpp
const SPISettings kBmsSpiSettings(1000000, MSBFIRST, SPI_MODE0);
adbms6830::ADBMS6830Driver bmsDriver(SPI1, PIN_CS, kBmsSpiSettings);
adbms6830::BMSInterface bmsInterface(bmsDriver);
```

3. Initialize in `setup()`:

```cpp
bmsInterface.begin();
```

4. Read telemetry:

```cpp
adbms6830::BMSStatus cellStatus = bmsInterface.readAllCellVoltages();
adbms6830::BMSStatus thermStatus = bmsInterface.readAllThermistors();
```

5. Control balancing:

```cpp
bmsInterface.setBalanceCell(moduleIndex, cellIndex, true);  // enable one cell
bmsInterface.setBalanceCell(moduleIndex, cellIndex, false); // disable one cell
bmsInterface.balancingOff();                                // disable all
```

## Notes

- Module and cell indices are zero-based in the interface API.
- `BMSStatus::kOk` indicates success.
- `readPwmRegisters(...)` can be used to verify PWM/balance state.
