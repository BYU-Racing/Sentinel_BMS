# MCP2517Can

`MCP2517Can` is a small wrapper around the vendored `ACAN2517` and `ACAN2517FD` drivers for the Microchip MCP2517FD external CAN controller.

It gives you one class with:

- constructor taking an SPI bus, CS pin, CAN mode, and optional INT pin
- `begin()` to initialize the controller
- `send()` to transmit a message
- `receive()` to read a pending message
- `poll()` for foreground servicing when desired
- `lastError()` to inspect the last initialization error code

The wrapper supports:

- CAN 2.0B via `MCP2517Can::Mode::Can20B`
- CAN FD via `MCP2517Can::Mode::CanFd`

## Files

- Header: [MCP2517Can.h](/mnt/c/Code/FASE_BMS/Code/BMS/include/MCP2517Can.h)
- Implementation: [MCP2517Can.cpp](/mnt/c/Code/FASE_BMS/Code/BMS/src/MCP2517Can.cpp)
- Example/test: [main.cpp](/mnt/c/Code/FASE_BMS/Code/BMS/src/main.cpp)

## Constructor

```cpp
MCP2517Can(
    SPIClass& spiBus,
    uint8_t csPin,
    Mode mode,
    uint8_t intPin = 255,
    Oscillator oscillator = Oscillator::Osc4MHzPll,
    uint32_t arbitrationBitRate = 500000,
    uint8_t fdDataBitRateFactor = 2);
```

### Parameters

- `spiBus`: Arduino SPI bus object such as `SPI`
- `csPin`: chip select pin for this MCP2517FD
- `mode`: `Can20B` or `CanFd`
- `intPin`: interrupt pin from the controller; use `255` to disable interrupts
- `oscillator`: controller oscillator selection
- `arbitrationBitRate`: nominal CAN bit rate in bits per second
- `fdDataBitRateFactor`: CAN FD data phase multiplier, used only in FD mode

## Oscillator Options

Pick the option that matches the crystal or oscillator connected to the MCP2517FD:

- `Osc4MHz`
- `Osc4MHzDiv2`
- `Osc4MHzPll`
- `Osc4MHzPllDiv2`
- `Osc20MHz`
- `Osc20MHzDiv2`
- `Osc40MHz`
- `Osc40MHzDiv2`

For this board, the tested configuration is:

```cpp
MCP2517Can::Oscillator::Osc40MHz
```

## Message Type

```cpp
struct Message {
  uint32_t id = 0;
  bool extended = false;
  bool remote = false;
  bool fdFrame = false;
  bool bitRateSwitch = false;
  uint8_t length = 0;
  uint8_t data[64] = {0};
};
```

### Notes

- In CAN 2.0B mode, `length` must be `0..8`
- In CAN 2.0B mode, `fdFrame` must be `false`
- In FD mode, payloads up to 64 bytes are supported
- `extended = true` selects a 29-bit identifier
- `remote = true` sends a remote frame
- `bitRateSwitch` is only meaningful for CAN FD frames

## Basic Bring-Up

For the RP2040 board used in this project, the SPI bus is configured explicitly before creating traffic:

```cpp
SPI.setRX(CAN_SPI_MISO);
SPI.setSCK(CAN_SPI_SCLK);
SPI.setTX(CAN_SPI_MOSI);
SPI.begin();
```

Then instantiate the controller:

```cpp
MCP2517Can can0(
    SPI,
    CAN0_CS,
    MCP2517Can::Mode::Can20B,
    CAN0_INT,
    MCP2517Can::Oscillator::Osc40MHz,
    250000);
```

Initialize it:

```cpp
if (!can0.begin()) {
  Serial.print("CAN init failed, error=0x");
  Serial.println(can0.lastError(), HEX);
}
```

## Sending

```cpp
MCP2517Can::Message tx;
tx.id = 0x101;
tx.length = 3;
tx.data[0] = 0x12;
tx.data[1] = 0x34;
tx.data[2] = 0x56;

const bool ok = can0.send(tx);
```

## Receiving

```cpp
MCP2517Can::Message rx;
if (can0.receive(rx)) {
  Serial.print("ID: 0x");
  Serial.println(rx.id, HEX);
}
```

## Interrupts and Polling

If a valid `intPin` is supplied, the wrapper installs an interrupt handler and forwards it to the underlying ACAN driver.

The current implementation supports up to two wrapper instances with interrupts, which matches this board's two MCP2517FD controllers.

You can still call `poll()` in the main loop:

```cpp
can0.poll();
can1.poll();
```

This is harmless and can help drain queued work in foreground code.

## Tested Board Assumptions

The current test in `src/main.cpp` assumes:

- `CAN_SPI_MISO = GPIO0`
- `CAN_SPI_SCLK = GPIO2`
- `CAN_SPI_MOSI = GPIO3`
- `CAN0_CS = GPIO4`
- `CAN1_CS = GPIO5`
- `CAN0_INT = GPIO18`
- `CAN1_INT = GPIO19`
- `LED_DATA = GPIO1`
- MCP2517FD crystal = `40 MHz`
- test bitrate = `250000`

## Example Output

Successful CAN 2.0B loop testing between the two onboard controllers looks like:

```text
CAN0 sent ping 0
CAN1 received id=0x101 len=3 data=00 CA 10
CAN1 sent pong for 0
CAN0 received id=0x102 len=3 data=00 AC 00
```

## Error Handling

`lastError()` returns the raw error code from the underlying ACAN library after `begin()`.

One useful example:

- `0x1` means the controller did not enter configuration mode during initialization

That kind of error usually points to SPI communication, CS wiring, power, clock selection, or reset/standby state rather than CAN bus traffic.
