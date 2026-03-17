#pragma once

#include <Arduino.h>
#include <SPI.h>

#include <array>
#include <cstddef>
#include <cstdint>

#include "ADBMS/ADBMS_interface.h"
#include "CONSTANTS.h"
#include "PINS.h"

class ReadBMS {
public:
	struct ModuleReadings {
		bool connected = false;
		bool cellDataValid = false;
		bool thermistorDataValid = false;
		std::array<uint16_t, adbms6830::BMSInterface::kCellsPerModule> cellVoltages{};
		std::array<float, adbms6830::BMSInterface::kGpioPerModule> thermistorTempsC{};
	};

	struct PollData {
		std::size_t connectedModuleCount = 0;
		std::array<ModuleReadings, constants::kModuleCount> modules{};
	};

	ReadBMS();

	void begin();
	void pollBMS();
	const PollData& data() const;
	void logConnectedModules() const;

private:
	static constexpr std::size_t kModuleCount = constants::kModuleCount;
	static constexpr std::size_t kThermistorsPerModule = constants::kThermistorsPerModule;
	static constexpr uint32_t kModuleScanIntervalMs = constants::kModuleScanIntervalMs;
	static constexpr uint8_t kConnectDebounce = constants::kConnectDebounce;
	static constexpr uint8_t kDisconnectDebounce = constants::kDisconnectDebounce;

	struct ModuleState {
		bool connected = false;
		uint8_t seenCount = 0;
		uint8_t missedCount = 0;
	};

	struct AggregateStats {
		float minValue = 0.0f;
		float maxValue = 0.0f;
		float avgValue = 0.0f;
	};

	static bool isUsableStatus(adbms6830::BMSStatus status);
	static bool hasAnyValidCell(const adbms6830::BMSInterface::ModuleData& module);
	static bool hasAnyValidThermistor(const adbms6830::BMSInterface::ModuleData& module);
	static AggregateStats cellStatsForModule(const adbms6830::BMSInterface::ModuleData& module);
	static AggregateStats thermistorStatsForModule(const adbms6830::BMSInterface::ModuleData& module);

	void configureSpiPins() const;
	void updatePollData();
	bool allConfiguredModulesHaveCells(std::size_t moduleCount) const;
	std::size_t scanModuleCount();

	static const SPISettings kBmsSpiSettings;

	adbms6830::ADBMS6830Driver bmsDriver_;
	adbms6830::BMSInterface bmsInterface_;
	std::array<ModuleState, kModuleCount> moduleStates_{};
	PollData pollData_{};
	std::size_t detectedModuleCount_ = 1;
	uint32_t lastModuleScanMs_ = 0;
};
