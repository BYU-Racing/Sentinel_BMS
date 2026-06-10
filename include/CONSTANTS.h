#pragma once

#include <cstddef>
#include <cstdint>

#include "ADBMS/ADBMS_interface.h"

namespace constants {
	constexpr std::size_t kModuleCount = adbms6830::BMSInterface::kNumModules;
	constexpr std::size_t kCellsPerModule = adbms6830::BMSInterface::kCellsPerModule;
	constexpr std::size_t kThermistorsPerModule = 7;
	constexpr std::size_t kMonitoredThermistorsPerModule = 6;
	constexpr std::size_t kMinValidThermistorsPerModule = 6;
	constexpr std::size_t kBoardThermistorIndex = 6;
	static_assert(kMinValidThermistorsPerModule <= kMonitoredThermistorsPerModule,
	              "Minimum valid thermistor count must not exceed thermistors per module");
	static_assert(kMonitoredThermistorsPerModule < kThermistorsPerModule,
	              "Dedicated board thermistor index must refer to an installed thermistor");
	static_assert(kBoardThermistorIndex == kMonitoredThermistorsPerModule,
	              "Board thermistor should be the first thermistor excluded from aggregate status");

	constexpr uint32_t kPollIntervalMs = 500;
	constexpr uint32_t kLogIntervalMs = 2000;
	constexpr uint32_t kModuleScanIntervalMs = 2000;

	constexpr uint32_t kCanBitRate = 250000;
	constexpr uint32_t kCanStatusIntervalMs = 1000;						// Rate at which data CAN status information is sent
	constexpr uint32_t kCanChargerIntervalMs = 1000;					// Rate to check if the charger sent a CAN message to toggle on charging mode
	constexpr uint32_t kCanChargerTimeOutMs = 2000;						// If the charger stops sending CAN messages then toggle off charging mode
	constexpr uint32_t kCanChargerControlIntervalMs = 1000;				// Rate at which the charger needs a CAN control message
	constexpr uint32_t kChargerStatusUpdateIntervalMs = 250;
	constexpr uint8_t kCanStatusPayloadLength = 8;

	constexpr uint8_t kConnectDebounce = 2;
	constexpr uint8_t kDisconnectDebounce = 2;

	constexpr uint16_t kCellVoltageErrorMinMv = 2550;
	constexpr uint16_t kCellVoltageExhaustedMinMv = 2800;
	constexpr uint16_t kCellVoltageGoodMinMv = 3100;
	constexpr uint16_t kCellVoltageGoodMaxMv = 4150;
	constexpr uint16_t kCellVoltageWarningMaxMv = 4200;
	constexpr uint16_t kCellVoltageErrorMaxMv = 4250;

	constexpr uint16_t kBalanceThresholdMv = 50;
	constexpr uint16_t kBalanceDisableDeltaMv = 5;
	constexpr uint16_t kBalanceMaxCellMv = 5500;

	constexpr float kTempWarningMinC = 5.0f;
	constexpr float kTempGoodMaxC = 55.0f;
	constexpr uint8_t kTempChargingFaultC = 65;				
	constexpr float kTempWarningMaxC = 75.0f;			// if cells reach 75 C then the BMS should error
	constexpr float kTempExhaustedMaxC = 80.0f;			// the cells should not go any higher than this per the datasheet
	constexpr float kBoardThermistorFaultMinC = 70.0f;

	constexpr std::size_t kLedCount = 13;
	constexpr uint8_t kLedBrightness = 32;

	// TODO constants for charging
	constexpr float kSocChargingLimit = 1.0f;		  	// SOC percentage charge limit
	constexpr float kVoltageChargerMaxPackV = 445.0f; 	// Charger will shutoff if limit is over-reached and BMS or wiring fails; this parameter is sent to the charger via CAN in charger mode
	constexpr uint16_t kStartBalancingMv = 3900; 		// when any cell reaches this value then balancing will start
	constexpr float kMaxChargerPowerOutputW = 6550.0f;	// ELCON charger specification
	constexpr float kStartChargeA = 9.0f; 				// 1.0C begining charging amperage

} // namespace constants 
