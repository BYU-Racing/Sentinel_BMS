#pragma once

#include <Arduino.h>
#include <SPI.h>

class ACAN2517;
class ACAN2517FD;

class MCP2517Can {
public:
  enum class Mode : uint8_t {
    Can20B,
    CanFd,
  };

  enum class Oscillator : uint8_t {
    Osc4MHz,
    Osc4MHzDiv2,
    Osc4MHzPll,
    Osc4MHzPllDiv2,
    Osc20MHz,
    Osc20MHzDiv2,
    Osc40MHz,
    Osc40MHzDiv2,
  };

  struct Message {
    uint32_t id = 0;
    bool extended = false;
    bool remote = false;
    bool fdFrame = false;
    bool bitRateSwitch = false;
    uint8_t length = 0;
    uint8_t data[64] = {0};
  };

  MCP2517Can(
      SPIClass& spiBus,
      uint8_t csPin,
      Mode mode,
      uint8_t intPin = 255,
      Oscillator oscillator = Oscillator::Osc4MHzPll,
      uint32_t arbitrationBitRate = 500000,
      uint8_t fdDataBitRateFactor = 2);

  ~MCP2517Can();

  bool begin();
  bool send(const Message& message);
  bool receive(Message& message);
  void poll();

  uint32_t lastError() const { return mLastError; }
  Mode mode() const { return mMode; }
  uint8_t csPin() const { return mCsPin; }
  uint8_t intPin() const { return mIntPin; }

  MCP2517Can(const MCP2517Can&) = delete;
  MCP2517Can& operator=(const MCP2517Can&) = delete;

private:
  SPIClass& mSpiBus;
  uint8_t mCsPin;
  uint8_t mIntPin;
  Mode mMode;
  Oscillator mOscillator;
  uint32_t mArbitrationBitRate;
  uint8_t mFdDataBitRateFactor;
  uint32_t mLastError = 0;
  uint8_t mInterruptSlot = 255;

  ACAN2517* mCan20 = nullptr;
  ACAN2517FD* mCanFd = nullptr;

  void onInterrupt();
  static void handleInterrupt0();
  static void handleInterrupt1();
};
