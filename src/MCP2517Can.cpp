#include "MCP2517Can.h"

#include <ACAN2517.h>
#include <ACAN2517FD.h>
#include <new>
#include <string.h>

namespace {

constexpr uint8_t kNoInterruptSlot = 255;
MCP2517Can* gInterruptOwners[2] = {nullptr, nullptr};

ACAN2517Settings::Oscillator toCan20Oscillator(MCP2517Can::Oscillator oscillator) {
  switch (oscillator) {
    case MCP2517Can::Oscillator::Osc4MHz:
      return ACAN2517Settings::OSC_4MHz;
    case MCP2517Can::Oscillator::Osc4MHzDiv2:
      return ACAN2517Settings::OSC_4MHz_DIVIDED_BY_2;
    case MCP2517Can::Oscillator::Osc4MHzPll:
      return ACAN2517Settings::OSC_4MHz10xPLL;
    case MCP2517Can::Oscillator::Osc4MHzPllDiv2:
      return ACAN2517Settings::OSC_4MHz10xPLL_DIVIDED_BY_2;
    case MCP2517Can::Oscillator::Osc20MHz:
      return ACAN2517Settings::OSC_20MHz;
    case MCP2517Can::Oscillator::Osc20MHzDiv2:
      return ACAN2517Settings::OSC_20MHz_DIVIDED_BY_2;
    case MCP2517Can::Oscillator::Osc40MHz:
      return ACAN2517Settings::OSC_40MHz;
    case MCP2517Can::Oscillator::Osc40MHzDiv2:
      return ACAN2517Settings::OSC_40MHz_DIVIDED_BY_2;
  }
  return ACAN2517Settings::OSC_4MHz10xPLL;
}

ACAN2517FDSettings::Oscillator toCanFdOscillator(MCP2517Can::Oscillator oscillator) {
  switch (oscillator) {
    case MCP2517Can::Oscillator::Osc4MHz:
      return ACAN2517FDSettings::OSC_4MHz;
    case MCP2517Can::Oscillator::Osc4MHzDiv2:
      return ACAN2517FDSettings::OSC_4MHz_DIVIDED_BY_2;
    case MCP2517Can::Oscillator::Osc4MHzPll:
      return ACAN2517FDSettings::OSC_4MHz10xPLL;
    case MCP2517Can::Oscillator::Osc4MHzPllDiv2:
      return ACAN2517FDSettings::OSC_4MHz10xPLL_DIVIDED_BY_2;
    case MCP2517Can::Oscillator::Osc20MHz:
      return ACAN2517FDSettings::OSC_20MHz;
    case MCP2517Can::Oscillator::Osc20MHzDiv2:
      return ACAN2517FDSettings::OSC_20MHz_DIVIDED_BY_2;
    case MCP2517Can::Oscillator::Osc40MHz:
      return ACAN2517FDSettings::OSC_40MHz;
    case MCP2517Can::Oscillator::Osc40MHzDiv2:
      return ACAN2517FDSettings::OSC_40MHz_DIVIDED_BY_2;
  }
  return ACAN2517FDSettings::OSC_4MHz10xPLL;
}

DataBitRateFactor toDataBitRateFactor(uint8_t factor) {
  switch (factor) {
    case 1:
      return DataBitRateFactor::x1;
    case 2:
      return DataBitRateFactor::x2;
    case 3:
      return DataBitRateFactor::x3;
    case 4:
      return DataBitRateFactor::x4;
    case 5:
      return DataBitRateFactor::x5;
    case 6:
      return DataBitRateFactor::x6;
    case 7:
      return DataBitRateFactor::x7;
    case 8:
      return DataBitRateFactor::x8;
    case 9:
      return DataBitRateFactor::x9;
    case 10:
      return DataBitRateFactor::x10;
    default:
      return DataBitRateFactor::x2;
  }
}

}  // namespace

MCP2517Can::MCP2517Can(
    SPIClass& spiBus,
    uint8_t csPin,
    Mode mode,
    uint8_t intPin,
    Oscillator oscillator,
    uint32_t arbitrationBitRate,
    uint8_t fdDataBitRateFactor)
    : mSpiBus(spiBus),
      mCsPin(csPin),
      mIntPin(intPin),
      mMode(mode),
      mOscillator(oscillator),
      mArbitrationBitRate(arbitrationBitRate),
      mFdDataBitRateFactor(fdDataBitRateFactor) {}

MCP2517Can::~MCP2517Can() {
  if (mInterruptSlot < 2 && gInterruptOwners[mInterruptSlot] == this) {
    gInterruptOwners[mInterruptSlot] = nullptr;
  }
  delete mCan20;
  delete mCanFd;
}

bool MCP2517Can::begin() {
  if (mInterruptSlot < 2 && gInterruptOwners[mInterruptSlot] == this) {
    gInterruptOwners[mInterruptSlot] = nullptr;
  }
  mInterruptSlot = kNoInterruptSlot;
  delete mCan20;
  delete mCanFd;
  mCan20 = nullptr;
  mCanFd = nullptr;
  mLastError = 0;

  void (*interruptHandler)(void) = nullptr;
  if (mIntPin != 255) {
    if (gInterruptOwners[0] == nullptr || gInterruptOwners[0] == this) {
      gInterruptOwners[0] = this;
      mInterruptSlot = 0;
      interruptHandler = &MCP2517Can::handleInterrupt0;
    } else if (gInterruptOwners[1] == nullptr || gInterruptOwners[1] == this) {
      gInterruptOwners[1] = this;
      mInterruptSlot = 1;
      interruptHandler = &MCP2517Can::handleInterrupt1;
    } else {
      mLastError = 0xFFFFFFFEu;
      return false;
    }
  }

  if (mMode == Mode::Can20B) {
    mCan20 = new (std::nothrow) ACAN2517(mCsPin, mSpiBus, mIntPin);
    if (mCan20 == nullptr) {
      mLastError = 0xFFFFFFFFu;
      return false;
    }

    ACAN2517Settings settings(toCan20Oscillator(mOscillator), mArbitrationBitRate);
    settings.mRequestedMode = ACAN2517Settings::Normal20B;
    mLastError = mCan20->begin(settings, interruptHandler);
    return mLastError == 0;
  }

  mCanFd = new (std::nothrow) ACAN2517FD(mCsPin, mSpiBus, mIntPin);
  if (mCanFd == nullptr) {
    mLastError = 0xFFFFFFFFu;
    return false;
  }

  ACAN2517FDSettings settings(
      toCanFdOscillator(mOscillator),
      mArbitrationBitRate,
      toDataBitRateFactor(mFdDataBitRateFactor));
  settings.mRequestedMode = ACAN2517FDSettings::NormalFD;
  settings.mControllerTransmitFIFOPayload = ACAN2517FDSettings::PAYLOAD_64;
  settings.mControllerReceiveFIFOPayload = ACAN2517FDSettings::PAYLOAD_64;
  mLastError = mCanFd->begin(settings, interruptHandler);
  return mLastError == 0;
}

bool MCP2517Can::send(const Message& message) {
  if (mMode == Mode::Can20B) {
    if (mCan20 == nullptr || message.length > 8 || message.fdFrame) {
      return false;
    }

    CANMessage canMessage;
    canMessage.id = message.id;
    canMessage.ext = message.extended;
    canMessage.rtr = message.remote;
    canMessage.len = message.length;
    memcpy(canMessage.data, message.data, message.length);
    return mCan20->tryToSend(canMessage);
  }

  if (mCanFd == nullptr || message.length > sizeof(message.data)) {
    return false;
  }

  CANFDMessage canFdMessage;
  canFdMessage.id = message.id;
  canFdMessage.ext = message.extended;
  canFdMessage.len = message.length;
  memcpy(canFdMessage.data, message.data, message.length);

  if (message.remote) {
    canFdMessage.type = CANFDMessage::CAN_REMOTE;
  } else if (message.fdFrame) {
    canFdMessage.type = message.bitRateSwitch
                            ? CANFDMessage::CANFD_WITH_BIT_RATE_SWITCH
                            : CANFDMessage::CANFD_NO_BIT_RATE_SWITCH;
  } else {
    canFdMessage.type = CANFDMessage::CAN_DATA;
  }

  return mCanFd->tryToSend(canFdMessage);
}

bool MCP2517Can::receive(Message& message) {
  if (mMode == Mode::Can20B) {
    if (mCan20 == nullptr) {
      return false;
    }

    CANMessage canMessage;
    if (!mCan20->receive(canMessage)) {
      return false;
    }

    message.id = canMessage.id;
    message.extended = canMessage.ext;
    message.remote = canMessage.rtr;
    message.fdFrame = false;
    message.bitRateSwitch = false;
    message.length = canMessage.len;
    memset(message.data, 0, sizeof(message.data));
    memcpy(message.data, canMessage.data, canMessage.len);
    return true;
  }

  if (mCanFd == nullptr) {
    return false;
  }

  CANFDMessage canFdMessage;
  if (!mCanFd->receive(canFdMessage)) {
    return false;
  }

  message.id = canFdMessage.id;
  message.extended = canFdMessage.ext;
  message.length = canFdMessage.len;
  message.fdFrame = canFdMessage.type == CANFDMessage::CANFD_NO_BIT_RATE_SWITCH ||
                    canFdMessage.type == CANFDMessage::CANFD_WITH_BIT_RATE_SWITCH;
  message.bitRateSwitch = canFdMessage.type == CANFDMessage::CANFD_WITH_BIT_RATE_SWITCH;
  message.remote = canFdMessage.type == CANFDMessage::CAN_REMOTE;
  memset(message.data, 0, sizeof(message.data));
  memcpy(message.data, canFdMessage.data, canFdMessage.len);
  return true;
}

void MCP2517Can::poll() {
  if (mCan20 != nullptr) {
    mCan20->poll();
  }
  if (mCanFd != nullptr) {
    mCanFd->poll();
  }
}

void MCP2517Can::onInterrupt() {
  if (mCan20 != nullptr) {
    mCan20->isr();
  }
  if (mCanFd != nullptr) {
    mCanFd->isr();
  }
}

void MCP2517Can::handleInterrupt0() {
  if (gInterruptOwners[0] != nullptr) {
    gInterruptOwners[0]->onInterrupt();
  }
}

void MCP2517Can::handleInterrupt1() {
  if (gInterruptOwners[1] != nullptr) {
    gInterruptOwners[1]->onInterrupt();
  }
}
