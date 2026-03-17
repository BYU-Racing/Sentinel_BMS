#pragma once

#include <cstddef>
#include <cstdint>

#include "ADBMS/ADBMS_interface.h"

namespace constants {
	constexpr std::size_t kModuleCount = adbms6830::BMSInterface::kNumModules;
	constexpr std::size_t kCellsPerModule = adbms6830::BMSInterface::kCellsPerModule;
	constexpr std::size_t kThermistorsPerModule = 7;

	constexpr uint32_t kPollIntervalMs = 250;
	constexpr uint32_t kLogIntervalMs = 2000;
	constexpr uint32_t kModuleScanIntervalMs = 2000;

	constexpr uint8_t kConnectDebounce = 2;
	constexpr uint8_t kDisconnectDebounce = 2;

	constexpr uint16_t kCellVoltageErrorMinMv = 2550;
	constexpr uint16_t kCellVoltageWarningMinMv = 3000;
	constexpr uint16_t kCellVoltageWarningMaxMv = 4200;
	constexpr uint16_t kCellVoltageErrorMaxMv = 4300;

	constexpr float kTempWarningMinC = 5.0f;
	constexpr float kTempWarningMaxC = 60.0f;
	constexpr float kTempErrorMaxC = 70.0f;

	constexpr std::size_t kLedCount = 12;
	constexpr uint8_t kLedBrightness = 32;
} // namespace constants
